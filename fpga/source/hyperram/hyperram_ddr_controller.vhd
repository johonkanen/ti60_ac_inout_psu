library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;

-- HyperRAM controller with DDIO pins, one CK cycle per hyperram_clock cycle.
--
-- Clocks, all from one PLL:
--   main_clock          user side, hyperram_in/hyperram_out as for hyperram_controller
--   hyperram_clock      CK frequency, 0 degrees: controller, CS#, DQ and RWDS outputs
--   ck clock            90 degrees, inverted at the CK pin (270 degrees): CK edges in the
--                       middle of the DQ bytes; driven from ck_hi/ck_lo
--   read_clock          CK frequency with dynamic phase shift (read_phase, 0.5 PLL cycles per
--                       step): DQ and RWDS input registers and the read capture
--
-- CK runs only while CS# is low, so CS# always changes with CK low. Cycle n of a
-- transaction counts from the cycle CS# goes low: CK runs from cycle 1, the
-- command/address takes cycles 1 to 3, data starts in cycle 2*latency_count + 3 (fixed
-- latency) or in cycle 4 for a register write.
-- Read data is taken from the RWDS pattern in the read clock domain, so the read latency
-- and the board delays only need the read clock phase to be in the data eye.
--
-- A write burst is collected completely on the user side before it is started.
--
-- Capture port: 128 bit records (eight 16 bit words, the first in bits 127:112) are written
-- in order into a ring of capture_ring_records records from word address capture_base, in
-- bursts of up to 64 records that end on 64 record boundaries, so a burst never crosses a
-- die boundary when the ring is aligned. Capture bursts go before queued commands.
-- capture_reset drops the records not yet written and restarts the ring at record 0, the
-- port is ready for new records with capture_is_ready; capture_written counts the records in
-- the memory. Records are only written in full 64 record bursts unless capture_flush is set.
entity hyperram_ddr_controller is
    generic(
        g_reset_low_clocks : positive := 250;      -- RESET# low, tRP > 200 ns, main clocks
        g_power_up_clocks  : positive := 25000;    -- tVCS > 150 us, main clocks
        g_read_timeout     : positive := 64        -- CK cycles after the expected last word
    );
    port(
        main_clock     : in std_logic;
        hyperram_in    : in hyperram_in_record;
        hyperram_out   : out hyperram_out_record;
        read_phase     : in natural range 0 to 7;
        phase_shift    : out pll_phase_shift_record;
        phase_shift_output : in natural range 0 to 4 := 3;   -- PLL output of the read clock

        hyperram_clock : in std_logic;
        fpga_out       : out hyperram_ddr_fpga_out_record;

        read_clock     : in std_logic;
        fpga_in        : in hyperram_ddr_fpga_in_record;
        -- order of the DDIO input halves in time: false, the rising edge sample (hi) is the
        -- earlier one; true, the falling edge sample presented with it (lo) is the earlier one
        input_lo_first : in boolean := false;
        -- capture port, main clock domain
        capture_reset        : in boolean := false;
        capture_write        : in boolean := false;
        capture_data         : in std_logic_vector(127 downto 0) := (others => '0');
        capture_flush        : in boolean := false;
        capture_is_full      : out boolean;
        capture_is_ready     : out boolean;   -- false from capture_reset until the ring is restarted
        capture_written      : out unsigned(31 downto 0);
        capture_base         : in natural range 0 to 2**25-1 := 0;
        capture_ring_records : in natural range 1 to 2**22 := 2**22;   -- power of two
        -- bring-up counters in the main clock domain: read clock samples with RWDS high on
        -- the hi and lo input halves, and transactions started
        debug_rwds_hi_samples : out unsigned(31 downto 0);
        debug_rwds_lo_samples : out unsigned(31 downto 0);
        debug_transactions    : out unsigned(31 downto 0)
    );
end entity hyperram_ddr_controller;

architecture rtl of hyperram_ddr_controller is

    constant max_burst : natural := 512;

    -- command from the user side: read, register, latency, length-1, address
    constant command_width : natural := 1 + 1 + 3 + 9 + 25;
    signal command_write, command_read : boolean := false;
    signal command_in, command_out     : std_logic_vector(command_width-1 downto 0);
    signal command_empty               : boolean;

    signal write_data_write, write_data_read : boolean := false;
    signal write_data_in, write_data_out     : std_logic_vector(15 downto 0) := (others => '0');
    signal write_data_empty                  : boolean;

    signal read_data_write, read_data_read : boolean := false;
    signal read_data_in, read_data_out     : std_logic_vector(15 downto 0) := (others => '0');
    signal read_data_empty                 : boolean;

    -- done from the hyperram clock domain: timed out, words received
    signal done_write, done_read : boolean := false;
    signal done_in, done_out     : std_logic_vector(10 downto 0) := (others => '0');
    signal done_empty            : boolean;

    ------------------------------------------------------------------------
    -- user side
    type user_states is (reset_low, power_up, idle, collect_write_data, wait_done);
    signal user_state      : user_states := reset_low;
    signal counter         : natural range 0 to g_power_up_clocks := 0;
    signal words_requested : natural range 0 to max_burst := 0;
    signal words_collected : natural range 0 to max_burst := 0;
    signal words_delivered : natural range 0 to max_burst := 0;
    signal words_to_deliver : natural range 0 to max_burst := 0;
    signal pop_pipe        : std_logic_vector(1 downto 0) := (others => '0');
    signal done_pipe       : std_logic_vector(1 downto 0) := (others => '0');
    signal has_count       : boolean := false;
    signal request_pending : boolean := false;
    signal user_command    : std_logic_vector(command_width-1 downto 0) := (others => '0');
    signal user_length     : natural range 1 to max_burst := 1;
    signal user_is_read    : boolean := false;
    signal timed_out       : boolean := false;

    signal ready, write_request, read_ready, done : boolean := false;
    signal read_word : std_logic_vector(15 downto 0) := (others => '0');
    signal reset_n_r : std_logic := '0';

    -- read clock phase stepping
    signal applied_phase : natural range 0 to 7 := 0;
    signal phase_timer   : natural range 0 to 31 := 0;
    signal shift_r       : std_logic_vector(2 downto 0) := (others => '0');
    signal shift_enable_r : std_logic := '0';

    ------------------------------------------------------------------------
    -- hyperram clock domain
    type core_states is (idle, wait_for_command, fetch_command, transaction, finish, recovery);
    signal core_state : core_states := idle;
    signal cycle      : natural range 0 to 1023 := 0;
    signal header     : hyperram_header := (others => '0');
    signal is_read, is_register : boolean := false;
    signal length     : natural range 1 to max_burst := 1;
    signal data_cycle : natural range 0 to 63 := 0;
    signal last_cycle : natural range 0 to 1023 := 0;
    signal ck_enable  : boolean := false;
    signal core_timed_out : boolean := false;
    signal recovery_count : natural range 0 to 15 := 0;

    -- capture port
    constant capture_burst_records : natural := 64;
    signal capture_fifo_read    : boolean := false;
    signal capture_fifo_out     : std_logic_vector(127 downto 0);
    signal capture_fifo_count   : natural range 0 to 256;
    signal capture_record       : std_logic_vector(127 downto 0) := (others => '0');
    signal is_capture           : boolean := false;
    signal capture_index        : unsigned(31 downto 0) := (others => '0');
    signal capture_index_gray   : unsigned(31 downto 0) := (others => '0');
    signal capture_records      : natural range 0 to capture_burst_records := 0;
    -- capture resets are counted in gray code, so the core can tell which one it has handled
    signal capture_reset_count  : unsigned(3 downto 0) := (others => '0');
    signal capture_reset_gray   : unsigned(3 downto 0) := (others => '0');
    signal reset_sync1, reset_sync2 : unsigned(3 downto 0) := (others => '0');
    signal reset_handled        : unsigned(3 downto 0) := (others => '0');
    signal capture_draining     : boolean := false;
    signal drain_quiet          : natural range 0 to 7 := 0;
    signal reset_done           : unsigned(3 downto 0) := (others => '0');
    signal reset_done_sync1, reset_done_sync2 : unsigned(3 downto 0) := (others => '0');
    signal flush_sync           : std_logic_vector(1 downto 0) := (others => '0');
    signal written_gray_sync1, written_gray_sync2 : unsigned(31 downto 0) := (others => '0');
    signal next_word  : boolean := false;
    signal last_latency : natural range 0 to 7 := 7;

    signal cs_n_r     : std_logic := '1';
    signal dq_hi_r, dq_lo_r : std_logic_vector(7 downto 0) := (others => '0');
    signal dq_oe_r    : std_logic := '0';
    signal rwds_oe_r  : std_logic := '0';
    signal ck_hi_r, ck_lo_r : std_logic := '0';

    -- transaction start and expected word count to the read clock domain
    signal capture_toggle   : std_logic := '0';
    signal capture_enable   : std_logic := '0';
    signal capture_length   : natural range 1 to max_burst := 1;

    -- words received, gray coded from the read clock domain
    signal received_gray_sync1, received_gray_sync2 : unsigned(9 downto 0) := (others => '0');
    signal received : natural range 0 to max_burst := 0;

    ------------------------------------------------------------------------
    -- read clock domain
    signal toggle_sync : std_logic_vector(2 downto 0) := (others => '0');
    signal enable_sync : std_logic_vector(1 downto 0) := (others => '0');
    signal capture_count : unsigned(9 downto 0) := (others => '0');
    signal capture_gray  : unsigned(9 downto 0) := (others => '0');
    signal previous_rwds    : std_logic := '0';
    signal previous_dq      : std_logic_vector(7 downto 0) := (others => '0');
    signal lo_first_sync    : std_logic_vector(1 downto 0) := (others => '0');

    signal rwds_hi_count, rwds_lo_count : unsigned(31 downto 0) := (others => '0');
    signal transaction_count : unsigned(31 downto 0) := (others => '0');
    signal read_debug_main   : std_logic_vector(63 downto 0);
    signal core_debug_main   : std_logic_vector(31 downto 0);

    attribute async_reg : boolean;
    attribute async_reg of received_gray_sync1, received_gray_sync2, toggle_sync, enable_sync : signal is true;
    attribute async_reg of reset_sync1, reset_sync2, flush_sync, reset_done_sync1, reset_done_sync2, written_gray_sync1, written_gray_sync2 : signal is true;

    function to_gray(u : unsigned) return unsigned is
    begin
        return u xor ('0' & u(u'left downto 1));
    end function;

    function from_gray(g : unsigned) return unsigned is
        variable u : unsigned(g'range);
    begin
        u(g'left) := g(g'left);
        for i in g'left-1 downto 0 loop
            u(i) := u(i+1) xor g(i);
        end loop;
        return u;
    end function;

    function to_std(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

begin

    hyperram_out <= (
        is_ready             => ready,
        write_data_requested => write_request,
        read_data            => read_word,
        read_data_is_ready   => read_ready,
        transaction_is_done  => done,
        read_timed_out       => timed_out);

    phase_shift.shift        <= shift_r;
    phase_shift.shift_select <= std_logic_vector(to_unsigned(2**phase_shift_output, 5)) when shift_enable_r = '1' else "00000";
    phase_shift.shift_enable <= shift_enable_r;

------------------------------------------------------------------------
    user_side : process(main_clock)
        variable header_bits : std_logic_vector(command_width-1 downto 0);
    begin
        if rising_edge(main_clock) then
            if capture_reset then
                capture_reset_count <= capture_reset_count + 1;
                capture_reset_gray  <= to_gray(capture_reset_count + 1);
            end if;
            reset_done_sync1   <= reset_done;
            reset_done_sync2   <= reset_done_sync1;
            capture_is_ready   <= reset_done_sync2 = capture_reset_gray and not capture_reset;
            written_gray_sync1 <= capture_index_gray;
            written_gray_sync2 <= written_gray_sync1;
            capture_written    <= from_gray(written_gray_sync2);

            write_request    <= false;
            read_ready       <= false;
            done             <= false;
            command_write    <= false;
            write_data_write <= false;
            read_data_read   <= false;
            done_read        <= false;
            fpga_out.reset_n <= reset_n_r;

            -- read clock phase: SHIFT is set first, SHIFT_SEL and SHIFT_ENA follow and are
            -- held for more than four read clock cycles
            if phase_timer > 0 then
                phase_timer <= phase_timer - 1;
                if phase_timer = 20 then
                    shift_enable_r <= '1';
                elsif phase_timer = 10 then
                    shift_enable_r <= '0';
                end if;
            elsif read_phase /= applied_phase then
                shift_r       <= std_logic_vector(to_unsigned(read_phase, 3));
                applied_phase <= read_phase;
                phase_timer   <= 24;
            end if;

            CASE user_state is
                WHEN reset_low =>
                    reset_n_r <= '0';
                    if counter < g_reset_low_clocks then
                        counter <= counter + 1;
                    else
                        counter    <= 0;
                        user_state <= power_up;
                    end if;

                WHEN power_up =>
                    reset_n_r <= '1';
                    if counter < g_power_up_clocks then
                        counter <= counter + 1;
                    else
                        counter    <= 0;
                        ready      <= true;
                        user_state <= idle;
                    end if;

                WHEN idle =>
                    if hyperram_in.command /= none then
                        ready <= false;
                        header_bits := to_std(hyperram_in.command = read)
                                       & to_std(hyperram_in.register_space)
                                       & std_logic_vector(to_unsigned(hyperram_in.latency_count, 3))
                                       & std_logic_vector(to_unsigned(hyperram_in.burst_length - 1, 9))
                                       & std_logic_vector(to_unsigned(hyperram_in.address, 25));
                        user_command    <= header_bits;
                        user_length     <= hyperram_in.burst_length;
                        user_is_read    <= hyperram_in.command = read;
                        words_requested <= 0;
                        words_collected <= 0;
                        words_delivered <= 0;
                        request_pending <= false;
                        if hyperram_in.command = read then
                            command_in    <= header_bits;
                            command_write <= true;
                            user_state    <= wait_done;
                        else
                            user_state <= collect_write_data;
                        end if;
                    end if;

                -- the whole burst goes to the write FIFO before the transaction starts
                WHEN collect_write_data =>
                    if hyperram_in.write_data_valid then
                        write_data_in    <= hyperram_in.write_data;
                        write_data_write <= true;
                        words_collected  <= words_collected + 1;
                        request_pending  <= false;
                    end if;
                    if not request_pending and not hyperram_in.write_data_valid and words_requested < user_length then
                        write_request   <= true;
                        request_pending <= true;
                        words_requested <= words_requested + 1;
                    end if;
                    if words_collected = user_length then
                        command_in    <= user_command;
                        command_write <= true;
                        user_state    <= wait_done;
                    end if;

                -- read words go to the user as they arrive, the transaction is done when the
                -- core has reported the number of words received and all of them are delivered
                WHEN wait_done =>
                    pop_pipe  <= pop_pipe(0) & '0';
                    done_pipe <= done_pipe(0) & '0';
                    if pop_pipe(1) = '1' then
                        read_word       <= read_data_out;
                        read_ready      <= true;
                        words_delivered <= words_delivered + 1;
                    end if;
                    if not read_data_empty and pop_pipe = "00" and not read_data_read then
                        read_data_read <= true;
                        pop_pipe(0)    <= '1';
                    end if;
                    if not done_empty and done_pipe = "00" and not has_count and not done_read then
                        done_read    <= true;
                        done_pipe(0) <= '1';
                    end if;
                    if done_pipe(1) = '1' then
                        words_to_deliver <= to_integer(unsigned(done_out(9 downto 0)));
                        timed_out        <= done_out(10) = '1';
                        has_count        <= true;
                    end if;
                    if has_count and words_delivered >= words_to_deliver and pop_pipe = "00" and not read_data_read then
                        has_count  <= false;
                        done       <= true;
                        ready      <= true;
                        user_state <= idle;
                    end if;
            end CASE;
        end if;
    end process;

------------------------------------------------------------------------
    core : process(hyperram_clock)
        variable latency  : natural range 0 to 7;
        variable first    : natural;
        variable received_words : natural;
    begin
        if rising_edge(hyperram_clock) then
            command_read    <= false;
            write_data_read <= false;
            done_write      <= false;

            received_gray_sync1 <= capture_gray;
            received_gray_sync2 <= received_gray_sync1;
            received_words := to_integer(from_gray(received_gray_sync2));

            -- CK of the next cycle, the CK pin register samples this a quarter cycle later
            ck_hi_r <= '0';
            ck_lo_r <= to_std(ck_enable);

            capture_fifo_read <= false;
            reset_sync1 <= capture_reset_gray;
            reset_sync2 <= reset_sync1;
            flush_sync <= flush_sync(0) & to_std(capture_flush);

            CASE core_state is
                WHEN idle =>
                    cs_n_r    <= '1';
                    dq_oe_r   <= '0';
                    rwds_oe_r <= '0';
                    ck_enable <= false;
                    is_capture <= false;
                    if reset_sync2 /= reset_handled then
                        -- capture reset: drop the records still in the FIFO, then restart the ring
                        reset_handled    <= reset_sync2;
                        capture_draining <= true;
                        drain_quiet      <= 0;
                    elsif capture_draining then
                        if capture_fifo_count > 0 then
                            capture_fifo_read <= true;
                            drain_quiet       <= 0;
                        elsif drain_quiet < 7 then
                            drain_quiet <= drain_quiet + 1;
                        else
                            capture_draining   <= false;
                            reset_done         <= reset_handled;
                            capture_index      <= (others => '0');
                            capture_index_gray <= (others => '0');
                        end if;
                    -- capture bursts end on 64 record boundaries, partial ones only when flushing
                    elsif capture_fifo_count >= capture_burst_records - to_integer(capture_index(5 downto 0))
                        or (flush_sync(1) = '1' and capture_fifo_count > 0)
                    then
                        if capture_fifo_count >= capture_burst_records - to_integer(capture_index(5 downto 0)) then
                            capture_records <= capture_burst_records - to_integer(capture_index(5 downto 0));
                        else
                            capture_records <= capture_fifo_count;
                        end if;
                        is_capture <= true;
                        core_state <= fetch_command;
                    elsif not command_empty then
                        command_read <= true;
                        core_state   <= wait_for_command;
                    end if;

                -- the FIFO output is valid two clocks after the read
                WHEN wait_for_command =>
                    core_state <= fetch_command;

                WHEN fetch_command =>
                    if is_capture then
                        -- a capture burst: the latency comes from the last user command
                        latency     := last_latency;
                        is_read     <= false;
                        is_register <= false;
                        length      <= 8*capture_records;
                        header      <= build_hyperram_header(
                                           read_transaction => false,
                                           register_space   => false,
                                           address          => (capture_base + 8*to_integer(capture_index and to_unsigned(capture_ring_records-1, 32))) mod 2**25);
                        first := 2*latency + 3;
                        data_cycle <= first;
                        last_cycle <= first + 8*capture_records - 1;
                        capture_length <= 1;
                    else
                        -- command_out is valid now
                        is_read     <= command_out(command_width-1) = '1';
                        is_register <= command_out(command_width-2) = '1';
                        latency     := to_integer(unsigned(command_out(command_width-3 downto command_width-5)));
                        last_latency <= latency;
                        length      <= to_integer(unsigned(command_out(33 downto 25))) + 1;
                        header      <= build_hyperram_header(
                                           read_transaction => command_out(command_width-1) = '1',
                                           register_space   => command_out(command_width-2) = '1',
                                           address          => to_integer(unsigned(command_out(24 downto 0))));
                        if command_out(command_width-1) = '0' and command_out(command_width-2) = '1' then
                            first := 4;
                        else
                            first := 2*latency + 3;
                        end if;
                        data_cycle <= first;
                        last_cycle <= first + to_integer(unsigned(command_out(33 downto 25)));
                        capture_length <= to_integer(unsigned(command_out(33 downto 25))) + 1;
                    end if;
                    capture_enable <= '0';
                    capture_toggle <= not capture_toggle;
                    core_timed_out <= false;
                    cycle      <= 0;
                    cs_n_r     <= '0';          -- cycle 0, CK still stopped
                    transaction_count <= transaction_count + 1;
                    ck_enable  <= true;         -- CK runs from cycle 1
                    core_state <= transaction;

                WHEN transaction =>
                    -- the outputs set here are those of cycle 'cycle + 1'
                    cycle <= cycle + 1;
                    -- write data: the FIFO output is valid two cycles after the read, so words
                    -- are read two cycles before the cycle they are sent in. For a register
                    -- write that is still during the command/address cycles.
                    write_data_read <= not is_read and not is_capture and cycle + 1 + 2 >= data_cycle and cycle + 1 + 2 <= last_cycle;
                    -- capture records: one record per eight data cycles, read three cycles ahead
                    capture_fifo_read <= is_capture and cycle + 1 + 3 >= data_cycle and cycle + 1 + 3 <= last_cycle
                                         and (cycle + 1 + 3 - data_cycle) mod 8 = 0;
                    if cycle + 1 <= 3 then
                        dq_hi_r <= get_header_byte(header, 2*cycle);
                        dq_lo_r <= get_header_byte(header, 2*cycle + 1);
                        dq_oe_r <= '1';
                    elsif is_read then
                        dq_oe_r <= '0';
                        -- the capture starts a few cycles before the first data word
                        if cycle + 4 >= data_cycle then
                            capture_enable <= '1';
                        end if;
                        -- the count from the capture is from this transaction once data has started
                        if (cycle + 1 >= data_cycle and received_words >= length) or cycle + 1 > last_cycle + g_read_timeout then
                            core_timed_out <= received_words < length;
                            ck_enable      <= false;
                            core_state     <= finish;
                        end if;
                    else
                        if cycle + 1 >= data_cycle and cycle + 1 <= last_cycle then
                            if is_capture then
                                -- word k of the burst is word k mod 8 of record k/8
                                if (cycle + 1 - data_cycle) mod 8 = 0 then
                                    capture_record <= capture_fifo_out;
                                    dq_hi_r <= capture_fifo_out(127 downto 120);
                                    dq_lo_r <= capture_fifo_out(119 downto 112);
                                else
                                    dq_hi_r <= capture_record(127 - 16*((cycle + 1 - data_cycle) mod 8) downto 120 - 16*((cycle + 1 - data_cycle) mod 8));
                                    dq_lo_r <= capture_record(119 - 16*((cycle + 1 - data_cycle) mod 8) downto 112 - 16*((cycle + 1 - data_cycle) mod 8));
                                end if;
                            else
                                dq_hi_r <= write_data_out(15 downto 8);
                                dq_lo_r <= write_data_out(7 downto 0);
                            end if;
                            dq_oe_r <= '1';
                        else
                            dq_oe_r <= '0';
                        end if;
                        -- memory writes: RWDS low as data mask from the cycle before the data
                        if not is_register and cycle + 2 >= data_cycle and cycle + 1 <= last_cycle then
                            rwds_oe_r <= '1';
                        else
                            rwds_oe_r <= '0';
                        end if;
                        if cycle + 1 = last_cycle then
                            ck_enable  <= false;    -- no CK after the last data cycle
                            core_state <= finish;
                        end if;
                    end if;

                WHEN finish =>
                    -- CS# goes high with CK stopped low
                    cs_n_r    <= '1';
                    dq_oe_r   <= '0';
                    rwds_oe_r <= '0';
                    capture_enable <= '0';
                    if is_capture then
                        capture_index      <= capture_index + capture_records;
                        capture_index_gray <= to_gray(capture_index + capture_records);
                    else
                        done_in    <= to_std(core_timed_out) & std_logic_vector(to_unsigned(received_words, 10));
                        done_write <= true;
                    end if;
                    recovery_count <= 0;
                    core_state <= recovery;

                WHEN recovery =>
                    -- tRWR and tCSHI between transactions
                    if recovery_count < 10 then
                        recovery_count <= recovery_count + 1;
                    else
                        core_state <= idle;
                    end if;
            end CASE;
        end if;
    end process;

    fpga_out.cs_n    <= cs_n_r;
    fpga_out.dq_hi   <= dq_hi_r;
    fpga_out.dq_lo   <= dq_lo_r;
    fpga_out.dq_oe   <= dq_oe_r;
    fpga_out.rwds_hi <= '0';
    fpga_out.rwds_lo <= '0';
    fpga_out.rwds_oe <= rwds_oe_r;
    fpga_out.ck_hi   <= ck_hi_r;
    fpga_out.ck_lo   <= ck_lo_r;

------------------------------------------------------------------------
    -- read capture: RWDS is high with the first byte of a word and low with the second.
    -- The two samples of each read clock cycle are put in time order, then a word starts at
    -- each sample with RWDS high: either within this cycle, or at the later sample of the
    -- previous cycle.
    capture : process(read_clock)
        variable word      : std_logic_vector(15 downto 0);
        variable has_word  : boolean;
        variable first_dq, second_dq     : std_logic_vector(7 downto 0);
        variable first_rwds, second_rwds : std_logic;
    begin
        if rising_edge(read_clock) then
            toggle_sync <= toggle_sync(1 downto 0) & capture_toggle;
            enable_sync <= enable_sync(0) & capture_enable;
            read_data_write <= false;

            if toggle_sync(2) /= toggle_sync(1) then
                capture_count <= (others => '0');
                capture_gray  <= (others => '0');
            end if;

            lo_first_sync <= lo_first_sync(0) & to_std(input_lo_first);
            if fpga_in.rwds_hi = '1' then rwds_hi_count <= rwds_hi_count + 1; end if;
            if fpga_in.rwds_lo = '1' then rwds_lo_count <= rwds_lo_count + 1; end if;
            if lo_first_sync(1) = '1' then
                first_dq  := fpga_in.dq_lo;  first_rwds  := fpga_in.rwds_lo;
                second_dq := fpga_in.dq_hi;  second_rwds := fpga_in.rwds_hi;
            else
                first_dq  := fpga_in.dq_hi;  first_rwds  := fpga_in.rwds_hi;
                second_dq := fpga_in.dq_lo;  second_rwds := fpga_in.rwds_lo;
            end if;

            has_word := false;
            if first_rwds = '1' then
                word     := first_dq & second_dq;
                has_word := true;
            elsif previous_rwds = '1' then
                word     := previous_dq & first_dq;
                has_word := true;
            end if;
            previous_rwds <= second_rwds;
            previous_dq   <= second_dq;

            if enable_sync(1) = '1' and has_word and capture_count < capture_length then
                read_data_in    <= word;
                read_data_write <= true;
                capture_count   <= capture_count + 1;
                capture_gray    <= to_gray(capture_count + 1);
            end if;
        end if;
    end process;

------------------------------------------------------------------------
    u_read_debug : entity work.cdc_snapshot
    generic map(64)
    port map(read_clock, std_logic_vector(rwds_hi_count) & std_logic_vector(rwds_lo_count), main_clock, read_debug_main);

    u_core_debug : entity work.cdc_snapshot
    generic map(32)
    port map(hyperram_clock, std_logic_vector(transaction_count), main_clock, core_debug_main);

    debug_rwds_hi_samples <= unsigned(read_debug_main(63 downto 32));
    debug_rwds_lo_samples <= unsigned(read_debug_main(31 downto 0));
    debug_transactions    <= unsigned(core_debug_main);

------------------------------------------------------------------------
    u_command_fifo : entity work.async_fifo
    generic map(g_width => command_width, g_address_bits => 2)
    port map(main_clock, command_write, command_in, open, hyperram_clock, command_read, command_out, command_empty);

    u_write_data_fifo : entity work.async_fifo
    generic map(g_width => 16, g_address_bits => 9)
    port map(main_clock, write_data_write, write_data_in, open, hyperram_clock, write_data_read, write_data_out, write_data_empty);

    u_read_data_fifo : entity work.async_fifo
    generic map(g_width => 16, g_address_bits => 9)
    port map(read_clock, read_data_write, read_data_in, open, main_clock, read_data_read, read_data_out, read_data_empty);

    u_capture_fifo : entity work.async_fifo
    generic map(g_width => 128, g_address_bits => 8)
    port map(main_clock, capture_write, capture_data, capture_is_full, hyperram_clock, capture_fifo_read,
             capture_fifo_out, open, capture_fifo_count);

    u_done_fifo : entity work.async_fifo
    generic map(g_width => 11, g_address_bits => 2)
    port map(hyperram_clock, done_write, done_in, open, main_clock, done_read, done_out, done_empty);

end architecture rtl;
