LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.hyperram_pkg.all;
    use work.ethernet_pkg.all;
    use work.ethernet_protocol_pkg.all;
    use work.udp_stream_pkg.all;

-- udp_stream_sender reading the HyperRAM through hyperram_stream_source, sharing the
-- HyperRAM user port with another user through hyperram_user_arbiter. The user port is a
-- behavioural model with a fixed pattern; the datagrams are decoded from the GMII output
-- and every word is checked.
entity hyperram_stream_source_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of hyperram_stream_source_tb is

    signal clock : std_logic := '0';

    signal request       : stream_request_record := no_stream_request;
    signal read_address  : natural;
    signal read_data     : std_logic_vector(31 downto 0);
    signal hr_read_data  : std_logic_vector(31 downto 0);
    signal bram_data     : std_logic_vector(31 downto 0) := (others => '0');
    signal bram_pipe     : std_logic_vector(31 downto 0) := (others => '0');
    signal prefetch      : stream_prefetch_record;
    signal data_is_ready : boolean;
    signal wants_to_send, is_sending : boolean;
    signal gmii          : gmii_record;

    signal source_in, other_in, shared_in    : hyperram_in_record := init_hyperram_in;
    signal source_out, other_out, shared_out : hyperram_out_record;

    signal other_enabled : boolean := false;
    signal other_reads, other_errors : natural := 0;

    -- decoded datagrams
    signal datagrams     : natural := 0;
    signal words_checked : natural := 0;
    signal word_errors   : natural := 0;
    signal last_id_words : natural := 0;   -- words of the datagrams of the latest request id
    signal next_id       : natural := 1;
    signal order_errors  : natural := 0;

    function pattern(address : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned((address * 7 + address / 65536) mod 65536, 16) xor x"1234");
    end function;

    function stream_word(w : natural) return std_logic_vector is
        constant a : natural := 2 * (w mod 2**24);
    begin
        return pattern(a) & pattern(a + 1);
    end function;

    function bram_word(w : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(w * 13 + 5, 32));
    end function;

begin

    clock <= not clock after 4 ns;

    u_sender : entity work.udp_stream_sender
    generic map(g_own_mac => x"020000000001", g_own_ip => x"C0A8140A", g_read_latency => 3)
    port map(
        clock          => clock
        ,request       => request
        ,read_address  => read_address
        ,read_data     => read_data
        ,prefetch      => prefetch
        ,data_is_ready => data_is_ready
        ,may_start     => not is_sending
        ,wants_to_send => wants_to_send
        ,is_sending    => is_sending
        ,gmii_out      => gmii
        ,datagrams     => open);

    u_source : entity work.hyperram_stream_source
    port map(clock, prefetch, data_is_ready, read_address, hr_read_data, source_in, source_out);

    -- block RAM source with the scope read latency
    bram : process(clock)
    begin
        if rising_edge(clock) then
            bram_pipe <= bram_word(read_address);
            bram_data <= bram_pipe;
        end if;
    end process;

    read_data <= hr_read_data when prefetch.is_active else bram_data;

    u_arbiter : entity work.hyperram_user_arbiter
    port map(clock, other_in, other_out, source_in, source_out, shared_in, shared_out);

------------------------------------------------------------------------
    -- user port model: reads return the pattern after a delay, with gaps between words
    user_port : process
        variable address, length : natural;
    begin
        shared_out <= (is_ready => true, write_data_requested => false, read_data => (others => '0'),
                       read_data_is_ready => false, transaction_is_done => false, read_timed_out => false);
        loop
            wait until rising_edge(clock);
            shared_out.read_data_is_ready  <= false;
            shared_out.transaction_is_done <= false;
            if shared_in.command = read then
                address := shared_in.address;
                length  := shared_in.burst_length;
                check(length <= 512 and (address mod 512) + length <= 512, "burst within a 512 word block");
                shared_out.is_ready <= false;
                for i in 1 to 20 loop wait until rising_edge(clock); end loop;
                for i in 0 to length-1 loop
                    shared_out.read_data          <= pattern(address + i);
                    shared_out.read_data_is_ready <= true;
                    wait until rising_edge(clock);
                    shared_out.read_data_is_ready <= false;
                    if i mod 5 = 4 then wait until rising_edge(clock); end if;
                end loop;
                shared_out.transaction_is_done <= true;
                wait until rising_edge(clock);
                shared_out.transaction_is_done <= false;
                for i in 1 to 3 loop wait until rising_edge(clock); end loop;
                shared_out.is_ready <= true;
            elsif shared_in.command = write then
                check(false, "no writes expected");
            end if;
        end loop;
    end process;

    -- another user reading single words and bursts while the stream runs
    other_user : process(clock)
        variable seed : natural := 12345;
        variable expected_address : natural := 0;
        variable index : natural := 0;
        variable busy : boolean := false;
    begin
        if rising_edge(clock) then
            init_hyperram(other_in);
            if other_enabled and not busy and hyperram_is_ready(other_out) then
                seed := (seed * 1103 + 12345) mod 2**19;
                expected_address := seed * 64;
                request_hyperram_read(other_in, expected_address, 1 + seed mod 40);
                index := 0;
                busy  := true;
            end if;
            if hyperram_read_data_is_ready(other_out) then
                if get_hyperram_read_data(other_out) /= pattern(expected_address + index) then
                    other_errors <= other_errors + 1;
                end if;
                index := index + 1;
            end if;
            if hyperram_transaction_is_done(other_out) then
                other_reads <= other_reads + 1;
                busy := false;
            end if;
        end if;
    end process;

------------------------------------------------------------------------
    -- datagram decoder
    decoder : process
        type byte_array is array (0 to 2047) of byte;
        variable frame : byte_array;
        variable n : natural;
        variable first, words : natural;
        variable word : std_logic_vector(31 downto 0);
        variable first_bits : std_logic_vector(31 downto 0);
        variable from_hyperram : boolean;
        variable next_expected : integer := -1;
        variable id, last_id : natural := 0;
    begin
        wait until rising_edge(clock) and gmii.is_valid;
        n := 0;
        while gmii.is_valid loop
            frame(n) := gmii.data;
            n := n + 1;
            wait until rising_edge(clock);
        end loop;
        -- 8 bytes preamble and start of frame, 4 bytes fcs
        first_bits := frame(8 + 46) & frame(8 + 47) & frame(8 + 48) & frame(8 + 49);
        first := to_integer(unsigned(first_bits(30 downto 0)));
        from_hyperram := first_bits(31) = '1';
        word := x"0000" & frame(8 + 50) & frame(8 + 51);
        words := to_integer(unsigned(word));
        -- a new request id starts a new stream
        word := x"0000" & frame(8 + 42) & frame(8 + 43);
        id := to_integer(unsigned(word));
        if id /= last_id then
            last_id := id;
            next_expected := -1;
            last_id_words <= words;
        else
            last_id_words <= last_id_words + words;
        end if;
        if next_expected >= 0 and first /= next_expected then
            order_errors <= order_errors + 1;
        end if;
        next_expected := first + words;
        for j in 0 to words-1 loop
            word := frame(8 + stream_data_offset + 4*j) & frame(8 + stream_data_offset + 4*j + 1)
                  & frame(8 + stream_data_offset + 4*j + 2) & frame(8 + stream_data_offset + 4*j + 3);
            if (from_hyperram and word /= stream_word(first + j)) or (not from_hyperram and word /= bram_word(first + j)) then
                word_errors <= word_errors + 1;
            end if;
        end loop;
        words_checked <= words_checked + words;
        datagrams <= datagrams + 1;
    end process;

------------------------------------------------------------------------
    main : process
        procedure stream(first_word : unsigned(31 downto 0); count : natural) is
        begin
            wait until rising_edge(clock);
            request <= (is_requested => true, destination_mac => x"001122334455", destination_ip => x"C0A81419",
                        destination_port => x"C000", source_port => x"138A", request_id => std_logic_vector(to_unsigned(next_id, 16)),
                        pacing => (others => '0'), first_word => first_word, word_count => to_unsigned(count, 32));
            next_id <= next_id + 1;
            wait until rising_edge(clock);
            request.is_requested <= false;
        end procedure;

        procedure wait_for_words(count : natural) is
        begin
            wait until words_checked >= count for 5 ms;
            check_equal(words_checked, count, "words received");
            wait for 2 us;
        end procedure;

        constant hyperram : unsigned(31 downto 0) := x"80000000";
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("stream_from_hyperram") then
                -- crosses 512 word blocks inside datagrams, last datagram partial
                stream(hyperram + 1000, 2000);
                wait_for_words(2000);
                check_equal(word_errors, 0, "wrong words");
                check_equal(order_errors, 0, "datagrams out of order");
                check_equal(datagrams, 6, "datagrams");

            elsif run("short_streams") then
                stream(hyperram + 7, 1);
                wait_for_words(1);
                stream(hyperram + 2**24 - 3, 6);   -- wraps at the end of the memory
                wait_for_words(7);
                check_equal(word_errors, 0, "wrong words");

            elsif run("competing_user") then
                other_enabled <= true;
                stream(hyperram + 123456, 5000);
                wait_for_words(5000);
                other_enabled <= false;
                wait for 2 us;
                info(integer'image(other_reads) & " reads of the other user");
                check(other_reads > 10, "the other user got turns");
                check_equal(other_errors, 0, "wrong words of the other user");
                check_equal(word_errors, 0, "wrong stream words");
                check_equal(order_errors, 0, "datagrams out of order");

            elsif run("new_request_replaces_stream") then
                stream(hyperram + 0, 100000);
                wait until datagrams = 3;
                stream(hyperram + 50000, 1000);
                -- the running datagram finishes, then the new stream follows
                wait for 200 us;
                check_equal(last_id_words, 1000, "words of the new stream");
                check_equal((words_checked - 1000) mod 360, 0, "only whole datagrams of the old stream");
                check_equal(word_errors, 0, "wrong words");
                check_equal(order_errors, 0, "datagrams out of order");

            elsif run("block_ram_stream_unchanged") then
                stream(to_unsigned(10, 32), 800);
                wait_for_words(800);
                check_equal(word_errors, 0, "wrong words");
                check(not prefetch.is_active, "no prefetching for block RAM streams");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
