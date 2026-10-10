LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;

-- touch_reader against a model of the XPT2046: its command byte taken at DCLK's rises, the
-- channel's 12 bits out after the falls of clocks 9..20 (after the busy clock), zeros otherwise.
-- Checked: a touch (Z1 over the threshold) read as X, Y, Z1, Z2; no touch keeping the last X, Y;
-- the commands' channels; chip select high between polls; the stop button: a press inside its
-- area one stop pulse, after two polls, none for a touch outside or a press held.
entity touch_reader_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of touch_reader_tb is

    constant base : natural := 1320;
    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;
    signal t_cs, t_clk, t_din : std_logic;
    signal t_dout : std_logic := '0';

    -- the panel: each channel's value
    signal x_value, y_value, z1_value, z2_value : natural := 0;
    signal bad_commands : natural := 0;
    signal stop : boolean;
    signal stops : natural := 0;

begin

    clock <= not clock after 4 ns;

    dut : entity work.touch_reader
    generic map(g_base_address => base, g_half_period => 3, g_poll_clocks => 2000)
    port map(clock, bus_in, bus_out, t_cs, t_clk, t_din, t_dout, stop);

    count_stops : process(clock)
    begin
        if rising_edge(clock) and stop then
            stops <= stops + 1;
        end if;
    end process;

    capture_bus_reads : process(clock)
    begin
        if rising_edge(clock) then
            bus_read_done <= false;
            if write_to_address_is_requested(bus_out, 0) then
                bus_read_data <= get_slv_data(bus_out);
                bus_read_done <= true;
            end if;
        end if;
    end process;

    -- the XPT2046: rises counted from chip select's fall
    controller : process(t_clk, t_cs)
        variable rises   : natural := 0;
        variable command : std_logic_vector(7 downto 0);
        variable value   : natural;
    begin
        if t_cs = '1' then
            rises := 0;
            t_dout <= '0';
        elsif rising_edge(t_clk) then
            if rises < 8 then
                command := command(6 downto 0) & t_din;
            end if;
            rises := rises + 1;
            if rises = 24 then
                rises := 0;
            end if;
        elsif falling_edge(t_clk) then
            if rises = 8 then
                -- the command whole: its channel
                case command is
                    when x"B0" => value := z1_value;
                    when x"C0" => value := z2_value;
                    when x"D0" => value := x_value;
                    when x"90" => value := y_value;
                    when others => value := 0; bad_commands <= bad_commands + 1;
                end case;
            end if;
            if rises >= 9 and rises <= 20 then
                t_dout <= to_unsigned(value, 12)(11 - (rises - 9));
            else
                t_dout <= '0';
            end if;
        end if;
    end process;

    main : process
        procedure bus_read(address : natural; data : out integer) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_in, address);
            wait until rising_edge(clock);
            init_bus(bus_in);
            wait until rising_edge(clock) and bus_read_done for 1 us;
            check(bus_read_done, "no response from " & integer'image(address));
            data := to_integer(signed(bus_read_data));
        end procedure;

        procedure two_polls is
            variable polls : integer;
            variable start : integer;
        begin
            bus_read(base + 5, start);
            loop
                wait for 5 us;
                bus_read(base + 5, polls);
                exit when polls >= start + 2;
            end loop;
        end procedure;

        procedure bus_write(address : natural; data : natural) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(to_unsigned(data, 32)));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        procedure one_poll is
            variable polls : integer;
            variable start : integer;
        begin
            bus_read(base + 5, start);
            loop
                wait for 2 us;
                bus_read(base + 5, polls);
                exit when polls >= start + 1;
            end loop;
        end procedure;

        variable v : integer;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("reads_a_touch_and_keeps_it") then
                x_value <= 1234; y_value <= 3001; z1_value <= 500; z2_value <= 2900;
                two_polls;
                bus_read(base, v);     check_equal(v, 1, "touched");
                bus_read(base + 1, v); check_equal(v, 1234, "X");
                bus_read(base + 2, v); check_equal(v, 3001, "Y");
                bus_read(base + 3, v); check_equal(v, 500, "Z1");
                bus_read(base + 4, v); check_equal(v, 2900, "Z2");
                -- lifted: Z1 under the threshold, X and Y kept
                x_value <= 4095; y_value <= 7; z1_value <= 20;
                two_polls;
                bus_read(base, v);     check_equal(v, 0, "not touched");
                bus_read(base + 1, v); check_equal(v, 1234, "X kept");
                bus_read(base + 2, v); check_equal(v, 3001, "Y kept");
                bus_read(base + 3, v); check_equal(v, 20, "Z1 read anyway");
                -- touched again elsewhere: the extremes
                z1_value <= 4095;
                two_polls;
                bus_read(base + 1, v); check_equal(v, 4095, "X 4095");
                bus_read(base + 2, v); check_equal(v, 7, "Y 7");
                check_equal(bad_commands, 0, "only the four commands");
                wait until t_cs = '1';
                wait for 1 us;
                check(t_cs = '1', "chip select high between polls");
            elsif run("the_stop_button") then
                bus_write(base + 7, 1000); bus_write(base + 8, 2000);
                bus_write(base + 9, 500);  bus_write(base + 10, 1500);
                -- a touch outside: no stop
                x_value <= 2500; y_value <= 1000; z1_value <= 500;
                two_polls; two_polls;
                check_equal(stops, 0, "outside: none");
                -- lifted, then pressed inside: a stop after the second poll inside, one only
                z1_value <= 0;
                two_polls;
                x_value <= 1500; y_value <= 1000; z1_value <= 500;
                one_poll;
                check_equal(stops, 0, "one poll inside: not yet");
                two_polls; two_polls;
                check_equal(stops, 1, "a press: one stop");
                bus_read(base + 11, v); check_equal(v, 1, "a press counted");
                -- light, under the threshold, inside: nothing; pressed again: a second stop
                z1_value <= 50;
                two_polls;
                z1_value <= 500;
                two_polls; two_polls;
                check_equal(stops, 2, "pressed again: a second stop");
                -- the edges count as inside
                z1_value <= 0; two_polls;
                x_value <= 2000; y_value <= 500; z1_value <= 500;
                two_polls; two_polls;
                check_equal(stops, 3, "the area's corner");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end vunit_simulation;
