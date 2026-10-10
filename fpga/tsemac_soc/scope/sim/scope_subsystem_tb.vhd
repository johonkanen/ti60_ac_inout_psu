library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library vunit_lib;
    context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;

-- scope_subsystem behind apb_bus_bridge, the way the CPU drives it, with a DMA stand-in that
-- takes the stream with gaps: register access, an undecoded address, and deep captures of
-- the test generator's counter whose records must arrive complete, in order and in the
-- channel order of the memory layout. At a slow sample rate the writer is idle when the last
-- record comes, which once let padding overtake it.
entity scope_subsystem_tb is
    generic(runner_cfg : string);
end entity;

architecture sim of scope_subsystem_tb is
    constant clock_period : time := 8 ns;
    signal clock : std_logic := '0';
    signal done  : boolean := false;

    signal psel, penable, pwrite, pready, pslverror : std_logic := '0';
    signal paddr  : std_logic_vector(15 downto 0) := (others => '0');
    signal pwdata, prdata : std_logic_vector(31 downto 0) := (others => '0');
    signal tvalid, tready, tlast : std_logic := '0';
    signal tdata  : std_logic_vector(63 downto 0);
    signal tkeep  : std_logic_vector(7 downto 0);

    constant max_beats : natural := 4096;
    type beat_array is array (0 to max_beats-1) of std_logic_vector(63 downto 0);
    signal beats      : beat_array := (others => (others => '0'));
    signal beat_count : natural := 0;
    signal gap        : unsigned(2 downto 0) := (others => '0');
    signal restart    : boolean := false;   -- the software restarts the DMA at its buffer

    signal bus_to_scope, bus_from_scope : fpga_interconnect_record := init_fpga_interconnect;

    constant scope : natural := 600;
begin

    clock <= not clock after clock_period/2 when not done;

    bridge : entity work.apb_bus_bridge
    port map(clock, psel, penable, pwrite, paddr, pwdata, prdata, pready, pslverror,
             bus_out => bus_to_scope, bus_in => bus_from_scope);

    dut : entity work.scope_subsystem
    port map(clock, bus_to_scope, bus_from_scope, tvalid, tready, tdata, tkeep, tlast,
             status => x"a5a5", sec_positions => (others => x"0042"),
             pri_positions => (others => x"0043"),
             sdm_results => (others => x"ff44"), sdm_means => (others => x"ff40"), sdm_sampled => (others => x"ff3c"), observer_probes => (others => x"0011"), control_probes => (others => x"0022"), sdm_midpoint => x"0044", dhb_probe => x"0001", output_probe => x"0001");

    -- the DMA: not ready one clock in four
    dma : process(clock)
    begin
        if rising_edge(clock) then
            gap <= gap + 1;
            if restart then
                beat_count <= 0;
            elsif tvalid = '1' and tready = '1' then
                if beat_count < max_beats then
                    beats(beat_count) <= tdata;
                end if;
                beat_count <= beat_count + 1;
            end if;
        end if;
    end process;
    tready <= '0' when gap(1 downto 0) = "11" else '1';

    main : process
        constant pre        : natural := 20;
        constant length     : natural := 200;
        constant min_pad    : natural := 16;
        procedure apb(write : boolean; register_address : natural; value : std_logic_vector(31 downto 0);
                      result : out std_logic_vector(31 downto 0)) is
        begin
            wait until rising_edge(clock);
            psel   <= '1';
            pwrite <= '1' when write else '0';
            paddr  <= std_logic_vector(to_unsigned(4*register_address, 16));
            pwdata <= value;
            wait until rising_edge(clock);
            penable <= '1';
            loop
                wait until rising_edge(clock);
                exit when pready = '1';
            end loop;
            result  := prdata;
            psel    <= '0';
            penable <= '0';
        end procedure;

        procedure write_register(register_address : natural; value : natural) is
            variable unused : std_logic_vector(31 downto 0);
        begin
            apb(true, register_address, std_logic_vector(to_unsigned(value, 32)), unused);
        end procedure;

        procedure read_register(register_address : natural; result : out unsigned(31 downto 0)) is
            variable data : std_logic_vector(31 downto 0);
        begin
            apb(false, register_address, (others => '0'), data);
            result := unsigned(data);
        end procedure;

        -- deep capture of the counter on channel 0 and the status probe on channel 1,
        -- trigger disabled so it is taken after the pre-trigger samples
        procedure deep_capture(divider : natural) is
            variable value : unsigned(31 downto 0);
            variable start : natural;
            variable total : natural;
            variable beat0 : std_logic_vector(63 downto 0);
            variable first : unsigned(15 downto 0);
        begin
            info("deep capture, a sample every " & integer'image(divider + 1) & " clocks");
            wait until rising_edge(clock);
            restart <= true;
            wait until rising_edge(clock);
            restart <= false;
            write_register(scope + 1, divider);
            write_register(scope + 6, 16#0302_0A07#);
            write_register(scope + 2, 0);
            write_register(scope + 4, pre);
            write_register(scope + 5, length);
            write_register(scope + 10, 1);
            write_register(scope + 0, 1);
            for i in 1 to 20000 loop
                read_register(scope + 0, value);
                exit when value(2 downto 0) = "100";
            end loop;
            check_equal(to_integer(value(2 downto 0)), 4, "the capture is done");
            read_register(scope + 11, value);
            check_equal(to_integer(value), 0, "no records lost");

            read_register(scope + 12, value);
            start := to_integer(value);
            -- at least min_pad padding records, up to a 16 record burst boundary
            total := start + length + min_pad;
            total := 16 * ((total + 15) / 16);
            check_equal(beat_count, 2 * total, "records and padding all taken, to a burst boundary");
            first := unsigned(beats(2*start)(15 downto 0));
            for k in 0 to length-1 loop
                beat0 := beats(2*(start + k));
                check_equal(unsigned(beat0(15 downto 0)), first + k, "channel 0, the counter, in order");
                check_equal(beat0(31 downto 16), std_logic_vector'(x"a5a5"), "channel 1, the status probe");
            end loop;
        end procedure;

        -- continuous deep capture: records keep coming past any length, register 22 follows
        -- them, stop ends it
        procedure continuous_capture is
            variable value, written : unsigned(31 downto 0);
            variable first : unsigned(15 downto 0);
            variable beats_then : natural;
        begin
            info("continuous deep capture");
            wait until rising_edge(clock);
            restart <= true;
            wait until rising_edge(clock);
            restart <= false;
            write_register(scope + 1, 3);
            write_register(scope + 6, 16#0302_0A07#);
            write_register(scope + 2, 0);
            write_register(scope + 4, 0);
            write_register(scope + 5, 50);      -- a length the capture goes past
            write_register(scope + 10, 3);
            write_register(scope + 0, 1);
            for i in 1 to 20000 loop
                read_register(scope + 22, written);
                exit when written >= 1500;
            end loop;
            check(written >= 1500, "register 22 counts the records past the length");
            read_register(scope + 0, value);
            check_equal(to_integer(value(2 downto 0)), 3, "still capturing");
            read_register(scope + 11, value);
            check_equal(to_integer(value), 0, "no records lost");
            check(beat_count >= 2 * to_integer(written), "register 22 behind the beats taken");
            first := unsigned(beats(0)(15 downto 0));
            for k in 0 to 1499 loop
                check_equal(unsigned(beats(2*k)(15 downto 0)), first + k,
                            "channel 0, the counter, in order");
            end loop;
            write_register(scope + 0, 4);       -- stop
            read_register(scope + 0, value);
            check_equal(to_integer(value(2 downto 0)), 0, "stopped");
            wait for 1 us;
            beats_then := beat_count;
            wait for 2 us;
            check_equal(beat_count, beats_then, "no records after the stop");
            read_register(scope + 22, written);
            check_equal(beat_count, 2 * to_integer(written), "register 22: every record taken");
        end procedure;

        variable value      : unsigned(31 downto 0);
    begin
        test_runner_setup(runner, runner_cfg);

        -- information: 4096 deep, 8 channels, 8 + 25 sources
        read_register(scope + 8, value);
        check_equal(to_integer(value(15 downto 0)), 4096, "depth");
        check_equal(to_integer(value(23 downto 16)), 8, "channels");
        check_equal(to_integer(value(31 downto 24)), 33, "sources");

        write_register(scope + 1, 3);
        read_register(scope + 1, value);
        check_equal(to_integer(value), 3, "divider reads back");
        read_register(1000, value);
        check_equal(to_integer(value), 0, "an undecoded address reads 0");

        deep_capture(3);
        deep_capture(40);
        continuous_capture;
        deep_capture(3);                        -- and a normal one after it

        done <= true;
        test_runner_cleanup(runner);
    end process;

end architecture sim;
