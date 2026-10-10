library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;
    use work.fpga_interconnect_pkg.all;

-- HyperRAM test engine controlled through the fpga_interconnect bus.
--
-- register map, offsets from g_base_address
--   0  W: command  1 single read, 2 single write, 3 start pattern test, 4 stop test,
--         5 recalibrate the read clock (see hyperram_calibrator)
--         (commands wait until the controller is free)
--      R: status   bit0 ready for a command, bit1 test running, bit2 test done,
--                  bit3 test passed, bit4 last read timed out
--   1  RW address of single access, bit31 selects register space, bits 24:0 word address
--   2  RW write data of single write
--   3  R  read data of last single read
--   4  RW latency count used by the controller, must match CR0[7:4], default 7
--   5  RW number of words in pattern test, multiple of 32, default 2**25 (whole device)
--   6  RW pattern seed
--   7  R  error count
--   8  R  first error address
--   9  R  first error, expected(31:16) actual(15:0)
--  10  R  words checked
--  11  R  read timeout count
--  12  R  number of accepted single access commands
--
-- streaming port, consecutive 16 bit words packed two per bus word, first word in 31:16
--  13  W  stream address, an even word address. Pending stream writes are finished first,
--         then the read prefetch restarts from this address
--  14  R  next two words from the address, the address advances by two. The read waits
--         for the prefetch when the read FIFO is empty
--  14  W  two words written to the address, the address advances by two
--  15  R  stream status: bits 9:0 read FIFO words, 19:10 write FIFO words,
--         bit 20 writes pending, bit 21 prefetch running
--  16  R  stream words lost because the write FIFO was full
--  17  R  stream reads that had to wait for the prefetch
--  18  W  read clock setting by hand: phase step bits 2:0, bit 8 lo input half first
--      R  read clock setting in use, same format
--  19  R  debug: read clock samples with RWDS high on the hi input half
--  20  R  debug: read clock samples with RWDS high on the lo input half
--  21  R  debug: transactions started by the controller
--  22  R  read clock calibration status, see hyperram_calibrator
entity hyperram_tester is
    generic(
        g_base_address : natural := 200
    );
    port(
        clock        : in std_logic;
        bus_in       : in fpga_interconnect_record;
        bus_out      : out fpga_interconnect_record;
        hyperram_in  : out hyperram_in_record;
        hyperram_out : in hyperram_out_record;
        -- read clock calibration, see hyperram_calibrator
        calibration_set      : out boolean;
        calibration_phase    : out natural range 0 to 7;
        calibration_lo_first : out boolean;
        recalibrate          : out boolean;
        calibration_status   : in std_logic_vector(31 downto 0) := (others => '0');
        debug_counters : in std_logic_vector(95 downto 0) := (others => '0')
    );
end entity hyperram_tester;

architecture rtl of hyperram_tester is

    constant burst_length : natural := 32;

    function pattern(address : natural; seed : std_logic_vector) return std_logic_vector is
        variable a : unsigned(24 downto 0);
    begin
        a := to_unsigned(address, 25);
        return std_logic_vector(a(15 downto 0) xor (a(24 downto 16) & a(24 downto 18))) xor seed(15 downto 0);
    end function;

    type tester_states is (idle, single_access, write_burst, wait_write, read_burst, wait_read,
                           stream_write, stream_read);
    signal state : tester_states := idle;

    signal access_address : std_logic_vector(31 downto 0) := (others => '0');
    signal access_data    : std_logic_vector(31 downto 0) := (others => '0');
    signal read_result    : std_logic_vector(31 downto 0) := (others => '0');
    signal latency_reg    : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(7, 32));
    signal test_length    : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(2**25, 32));
    signal seed           : std_logic_vector(31 downto 0) := (others => '0');

    signal error_count       : unsigned(31 downto 0) := (others => '0');
    signal first_error_addr  : std_logic_vector(31 downto 0) := (others => '0');
    signal first_error_data  : std_logic_vector(31 downto 0) := (others => '0');
    signal words_checked     : unsigned(31 downto 0) := (others => '0');
    signal timeout_count     : unsigned(31 downto 0) := (others => '0');
    signal accepted_commands : unsigned(31 downto 0) := (others => '0');

    signal test_is_running : boolean := false;
    signal test_is_done    : boolean := false;
    signal last_timed_out  : boolean := false;

    signal block_address : natural range 0 to 2**25 := 0;
    signal word_index    : natural range 0 to 64 := 0;
    signal status        : std_logic_vector(31 downto 0) := (others => '0');
    signal read_setup    : std_logic_vector(31 downto 0) := (others => '0');
    signal pending_command : natural range 0 to 7 := 0;

    -- streaming port
    constant stream_burst_words  : natural := 32;                    -- 16 bit words
    constant bus_words_per_burst : natural := stream_burst_words/2;  -- 32 bit words
    constant fifo_depth          : natural := 512;
    constant write_idle_limit    : natural := 255;
    constant die_words           : natural := 2**24;

    signal stream_address_pending : boolean := false;
    signal new_stream_address     : natural range 0 to 2**25-1 := 0;
    signal stream_read_address    : natural range 0 to 2**25-1 := 0;
    signal stream_write_address   : natural range 0 to 2**25-1 := 0;
    signal prefetch_is_enabled    : boolean := false;
    signal stream_burst_length    : natural range 0 to 64 := 0;

    signal read_half          : std_logic_vector(15 downto 0) := (others => '0');
    signal read_half_is_valid : boolean := false;
    signal write_low_half     : std_logic_vector(15 downto 0) := (others => '0');
    signal write_low_is_next  : boolean := false;
    signal write_word_popped  : boolean := false;   -- FIFO read requested last clock
    signal write_word_ready   : boolean := false;   -- FIFO output valid now
    signal write_idle_time    : natural range 0 to write_idle_limit := 0;

    signal bus_read_waiting   : boolean := false;
    signal read_popped        : std_logic_vector(1 downto 0) := (others => '0');

    signal read_fifo_write, read_fifo_read, read_fifo_clear : boolean := false;
    signal read_fifo_in, read_fifo_out : std_logic_vector(31 downto 0) := (others => '0');
    signal read_fifo_count  : natural range 0 to fifo_depth;
    signal read_fifo_empty  : boolean;
    signal write_fifo_write, write_fifo_read : boolean := false;
    signal write_fifo_in, write_fifo_out : std_logic_vector(31 downto 0) := (others => '0');
    signal write_fifo_count : natural range 0 to fifo_depth;
    signal write_fifo_empty, write_fifo_full : boolean;

    signal stream_status   : std_logic_vector(31 downto 0) := (others => '0');
    signal write_overflows : unsigned(31 downto 0) := (others => '0');
    signal read_stalls     : unsigned(31 downto 0) := (others => '0');

    -- burst length that does not cross the die boundary
    function burst_within_die(address : natural; wanted : natural) return natural is
        constant left_in_die : natural := die_words - address mod die_words;
    begin
        if wanted < left_in_die then return wanted; else return left_in_die; end if;
    end function;

    function to_bit(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

begin

    -- ready for a new command: nothing queued and the controller free
    status <= (0 => to_bit(hyperram_is_ready(hyperram_out) and pending_command = 0 and state = idle),
               1 => to_bit(test_is_running),
               2 => to_bit(test_is_done),
               3 => to_bit(test_is_done and error_count = 0),
               4 => to_bit(last_timed_out),
               others => '0');

    stream_status <= std_logic_vector(to_unsigned(0, 10))
                     & to_bit(prefetch_is_enabled)
                     & to_bit(not write_fifo_empty or write_word_popped or write_word_ready or write_low_is_next or stream_address_pending or state = stream_write)
                     & std_logic_vector(to_unsigned(write_fifo_count, 10))
                     & std_logic_vector(to_unsigned(read_fifo_count, 10));

    calibration_phase    <= to_integer(unsigned(read_setup(2 downto 0)));
    calibration_lo_first <= read_setup(8) = '1';

    tester : process(clock)
        variable expected : std_logic_vector(15 downto 0);
        variable bus_words : natural;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            init_hyperram(hyperram_in);
            hyperram_in.latency_count <= to_integer(unsigned(latency_reg(2 downto 0)));

            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0 , status);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 1 , access_address);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 2 , access_data);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3 , read_result);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 4 , latency_reg);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 5 , test_length);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 6 , seed);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7 , std_logic_vector(error_count));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8 , first_error_addr);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 9 , first_error_data);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 10, std_logic_vector(words_checked));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, std_logic_vector(timeout_count));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 12, std_logic_vector(accepted_commands));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 15, stream_status);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 16, std_logic_vector(write_overflows));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 17, std_logic_vector(read_stalls));
            -- the setting in use comes from the calibrator, a write sets it by hand
            calibration_set <= false;
            recalibrate     <= false;
            if write_is_requested_to_address(bus_in, g_base_address + 18) then
                read_setup      <= get_slv_data(bus_in);
                calibration_set <= true;
            end if;
            if write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(2 downto 0) = "101" then
                recalibrate <= true;
            end if;
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 18,
                x"00000" & "000" & calibration_status(27) & "00000" & calibration_status(26 downto 24));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 22, calibration_status);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 19, debug_counters(95 downto 64));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 20, debug_counters(63 downto 32));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 21, debug_counters(31 downto 0));

            read_fifo_write  <= false;
            read_fifo_read   <= false;
            read_fifo_clear  <= false;
            write_fifo_write <= false;
            write_fifo_read  <= false;

            -- commands wait until the tester is idle
            if write_is_requested_to_address(bus_in, g_base_address) then
                pending_command <= to_integer(unsigned(get_slv_data(bus_in)(2 downto 0)));
            end if;

            ------------------------------------------------------------
            -- streaming port, bus side
            if write_is_requested_to_address(bus_in, g_base_address + 13) then
                new_stream_address     <= 2*to_integer(unsigned(get_slv_data(bus_in)(24 downto 1)));
                stream_address_pending <= true;
            end if;

            if write_is_requested_to_address(bus_in, g_base_address + 14) then
                if write_fifo_full then
                    write_overflows <= write_overflows + 1;
                else
                    write_fifo_write <= true;
                    write_fifo_in    <= get_slv_data(bus_in);
                end if;
                prefetch_is_enabled <= false;
                write_idle_time     <= 0;
            elsif write_idle_time < write_idle_limit then
                write_idle_time <= write_idle_time + 1;
            end if;

            -- a read is answered two clocks after the FIFO is popped
            read_popped <= read_popped(0) & '0';
            if read_popped(1) = '1' then
                write_data_to_address(bus_out, 0, read_fifo_out);
            end if;
            if data_is_requested_from_address(bus_in, g_base_address + 14) then
                bus_read_waiting <= true;
                if read_fifo_empty then
                    read_stalls <= read_stalls + 1;
                end if;
            end if;
            if (bus_read_waiting or data_is_requested_from_address(bus_in, g_base_address + 14))
                and not read_fifo_empty and read_popped = "00"
            then
                read_fifo_read   <= true;
                read_popped(0)   <= '1';
                bus_read_waiting <= false;
            end if;

            -- write data for bursts and single writes
            write_word_popped <= false;
            write_word_ready  <= write_word_popped;
            if write_word_ready then
                send_hyperram_write_data(hyperram_in, write_fifo_out(31 downto 16));
                write_low_half    <= write_fifo_out(15 downto 0);
                write_low_is_next <= true;
            end if;
            if hyperram_write_data_is_requested(hyperram_out) then
                if state = single_access then
                    send_hyperram_write_data(hyperram_in, access_data(15 downto 0));
                elsif state = stream_write then
                    if write_low_is_next then
                        send_hyperram_write_data(hyperram_in, write_low_half);
                        write_low_is_next <= false;
                    else
                        write_fifo_read   <= true;   -- the high half follows when the FIFO has read
                        write_word_popped <= true;
                    end if;
                else
                    send_hyperram_write_data(hyperram_in, pattern(block_address + word_index, seed));
                    word_index <= word_index + 1;
                end if;
            end if;

            -- read data
            if hyperram_read_data_is_ready(hyperram_out) then
                if state = single_access then
                    read_result <= x"0000" & get_hyperram_read_data(hyperram_out);
                elsif state = stream_read then
                    if read_half_is_valid then
                        read_fifo_write    <= true;
                        read_fifo_in       <= read_half & get_hyperram_read_data(hyperram_out);
                        read_half_is_valid <= false;
                    else
                        read_half          <= get_hyperram_read_data(hyperram_out);
                        read_half_is_valid <= true;
                    end if;
                else
                    expected := pattern(block_address + word_index, seed);
                    word_index    <= word_index + 1;
                    words_checked <= words_checked + 1;
                    if get_hyperram_read_data(hyperram_out) /= expected then
                        if error_count = 0 then
                            first_error_addr <= std_logic_vector(to_unsigned(block_address + word_index, 32));
                            first_error_data <= expected & get_hyperram_read_data(hyperram_out);
                        end if;
                        if error_count /= x"ffffffff" then
                            error_count <= error_count + 1;
                        end if;
                    end if;
                end if;
            end if;

            CASE state is
                WHEN idle =>
                    if pending_command /= 0 and hyperram_is_ready(hyperram_out) then
                        pending_command <= 0;
                        CASE pending_command is
                            WHEN 1 =>
                                request_hyperram_read(hyperram_in
                                    , to_integer(unsigned(access_address(24 downto 0))), 1
                                    , register_space => access_address(31) = '1');
                                accepted_commands <= accepted_commands + 1;
                                state <= single_access;
                            WHEN 2 =>
                                request_hyperram_write(hyperram_in
                                    , to_integer(unsigned(access_address(24 downto 0))), 1
                                    , register_space => access_address(31) = '1');
                                accepted_commands <= accepted_commands + 1;
                                state <= single_access;
                            WHEN 3 =>
                                error_count      <= (others => '0');
                                words_checked    <= (others => '0');
                                timeout_count    <= (others => '0');
                                first_error_addr <= (others => '0');
                                first_error_data <= (others => '0');
                                block_address    <= 0;
                                test_is_running  <= true;
                                test_is_done     <= false;
                                state            <= write_burst;
                            WHEN others => -- do nothing
                        end CASE;

                    -- stream address change once the pending writes are done
                    elsif stream_address_pending and write_fifo_empty and not write_word_popped and not write_word_ready then
                        stream_read_address    <= new_stream_address;
                        stream_write_address   <= new_stream_address;
                        stream_address_pending <= false;
                        read_fifo_clear        <= true;
                        read_half_is_valid     <= false;
                        prefetch_is_enabled    <= true;

                    -- stream writes when a burst is full, when writes have stopped or before an address change
                    elsif not write_fifo_empty and hyperram_is_ready(hyperram_out)
                        and (write_fifo_count >= bus_words_per_burst or write_idle_time = write_idle_limit or stream_address_pending)
                    then
                        bus_words := write_fifo_count;
                        if bus_words > bus_words_per_burst then
                            bus_words := bus_words_per_burst;
                        end if;
                        stream_burst_length <= burst_within_die(stream_write_address, 2*bus_words);
                        request_hyperram_write(hyperram_in, stream_write_address, burst_within_die(stream_write_address, 2*bus_words));
                        write_low_is_next <= false;
                        state <= stream_write;

                    -- read prefetch while there is room in the read FIFO
                    elsif prefetch_is_enabled and hyperram_is_ready(hyperram_out)
                        and read_fifo_count + bus_words_per_burst <= fifo_depth and not read_fifo_clear
                    then
                        stream_burst_length <= burst_within_die(stream_read_address, stream_burst_words);
                        request_hyperram_read(hyperram_in, stream_read_address, burst_within_die(stream_read_address, stream_burst_words));
                        read_half_is_valid <= false;
                        state <= stream_read;
                    end if;

                WHEN stream_write =>
                    if hyperram_transaction_is_done(hyperram_out) then
                        stream_write_address <= (stream_write_address + stream_burst_length) mod 2**25;
                        state <= idle;
                    end if;

                WHEN stream_read =>
                    if hyperram_transaction_is_done(hyperram_out) then
                        if hyperram_out.read_timed_out then
                            timeout_count <= timeout_count + 1;
                        end if;
                        stream_read_address <= (stream_read_address + stream_burst_length) mod 2**25;
                        state <= idle;
                    end if;

                WHEN single_access =>
                    if hyperram_transaction_is_done(hyperram_out) then
                        last_timed_out <= hyperram_out.read_timed_out;
                        state          <= idle;
                    end if;

                WHEN write_burst =>
                    if hyperram_is_ready(hyperram_out) then
                        word_index <= 0;
                        request_hyperram_write(hyperram_in, block_address, burst_length);
                        state <= wait_write;
                    end if;

                WHEN wait_write =>
                    if hyperram_transaction_is_done(hyperram_out) then
                        if block_address + burst_length < to_integer(unsigned(test_length(25 downto 0))) then
                            block_address <= block_address + burst_length;
                            state         <= write_burst;
                        else
                            block_address <= 0;
                            state         <= read_burst;
                        end if;
                    end if;

                WHEN read_burst =>
                    if hyperram_is_ready(hyperram_out) then
                        word_index <= 0;
                        request_hyperram_read(hyperram_in, block_address, burst_length);
                        state <= wait_read;
                    end if;

                WHEN wait_read =>
                    if hyperram_transaction_is_done(hyperram_out) then
                        if hyperram_out.read_timed_out then
                            timeout_count <= timeout_count + 1;
                            error_count   <= error_count + (burst_length - word_index);
                            if error_count = 0 then
                                first_error_addr <= std_logic_vector(to_unsigned(block_address + word_index, 32));
                                first_error_data <= pattern(block_address + word_index, seed) & x"0000";
                            end if;
                        end if;
                        if block_address + burst_length < to_integer(unsigned(test_length(25 downto 0))) then
                            block_address <= block_address + burst_length;
                            state         <= read_burst;
                        else
                            test_is_running <= false;
                            test_is_done    <= true;
                            state           <= idle;
                        end if;
                    end if;
            end CASE;

            -- stop test
            if write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(2 downto 0) = "100" then
                pending_command <= 0;
                if state /= single_access and state /= stream_write and state /= stream_read then
                    test_is_running <= false;
                    state <= idle;
                end if;
            end if;

        end if; -- rising_edge
    end process tester;

------------------------------------------------------------------------
    u_read_fifo : entity work.sync_fifo
    generic map(g_width => 32, g_depth => fifo_depth)
    port map(clock, read_fifo_clear, read_fifo_write, read_fifo_in, read_fifo_read, read_fifo_out,
             read_fifo_count, read_fifo_empty, open);

    u_write_fifo : entity work.sync_fifo
    generic map(g_width => 32, g_depth => fifo_depth)
    port map(clock, false, write_fifo_write, write_fifo_in, write_fifo_read, write_fifo_out,
             write_fifo_count, write_fifo_empty, write_fifo_full);

end architecture rtl;
