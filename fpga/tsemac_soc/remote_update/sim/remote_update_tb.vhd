library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library vunit_lib;
    context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;

-- remote_update through apb_bus_bridge as the CPU drives it: the role and error registers,
-- that a wrong command does nothing, and the reconfiguration sequence of a right one; the
-- fallback timer (seconds of 1000 clocks here): stopped by its command, else back to image 0.
entity remote_update_tb is
    generic(runner_cfg : string; fallback_seconds : natural := 0);
end entity remote_update_tb;

architecture sim of remote_update_tb is
    constant base : natural := 800;
    signal clock : std_logic := '0';
    signal done  : boolean := false;
    signal psel, penable, pwrite, pready, pslverror : std_logic := '0';
    signal paddr  : std_logic_vector(15 downto 0) := (others => '0');
    signal pwdata, prdata : std_logic_vector(31 downto 0) := (others => '0');
    signal bus_to, bus_from : fpga_interconnect_record := init_fpga_interconnect;
    signal cfg_CBSEL : std_logic_vector(1 downto 0);
    signal cfg_ENA, cfg_CONFIG : std_logic;
    signal cfg_ERROR : std_logic := '0';
begin
    clock <= not clock after 4 ns when not done;

    bridge : entity work.apb_bus_bridge
    port map(clock, psel, penable, pwrite, paddr, pwdata, prdata, pready, pslverror,
             bus_out => bus_to, bus_in => bus_from);

    dut : entity work.remote_update
    generic map(g_base_address => base, g_role => 1, g_fallback_seconds => fallback_seconds,
                g_clock_frequency => 1000)
    port map(clock, bus_to, bus_from, cfg_CBSEL, cfg_ENA, cfg_CONFIG, cfg_ERROR);

    main : process
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
        variable value : std_logic_vector(31 downto 0);
        variable t_ena, t_config : time;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("commands_reconfigure") then

                apb(false, base + 0, x"00000000", value);
                check_equal(value, std_logic_vector'(x"52550001"), "role: RU, application");
                apb(false, base + 1, x"00000000", value);
                check_equal(value, std_logic_vector'(x"00000000"), "no error");
                cfg_ERROR <= '1';
                wait for 100 ns;
                apb(false, base + 1, x"00000000", value);
                check_equal(value, std_logic_vector'(x"00000001"), "error seen");

                -- wrong magic, and a right magic with junk in the image bits: nothing happens
                apb(true, base + 2, x"52434501", value);
                apb(true, base + 2, x"52434605", value);
                wait for 5 us;
                check_equal(cfg_ENA, '0', "no enable after wrong commands");
                check_equal(cfg_CONFIG, '0', "no reconfiguration after wrong commands");

                -- reconfigure from image 1
                apb(true, base + 2, x"52434601", value);
                wait until cfg_ENA = '1' for 10 us;
                check_equal(cfg_ENA, '1', "enable");
                t_ena := now;
                check_equal(cfg_CBSEL, std_logic_vector'("01"), "image 1 selected before the enable");
                check_equal(cfg_CONFIG, '0', "not yet configuring");
                wait until cfg_CONFIG = '1' for 10 us;
                t_config := now;
                check_equal(cfg_CONFIG, '1', "reconfiguration");
                check(t_config - t_ena >= 2 us, "enable high 2 us before the reconfiguration");
                wait for 1 us;
                check_equal(cfg_ENA & cfg_CONFIG & cfg_CBSEL, std_logic_vector'("1101"), "all held");


            elsif run("the_fallback_timer_stopped_in_time_does_nothing") then
                apb(false, base + 3, x"00000000", value);
                check_equal(value(0), '1', "the timer runs");
                check_equal(to_integer(unsigned(value(31 downto 16))), fallback_seconds, "its seconds");
                wait for (fallback_seconds - 1) * 8 us;
                apb(false, base + 3, x"00000000", value);
                check_equal(to_integer(unsigned(value(31 downto 16))), 1, "a second left");
                -- a wrong value does not stop it, the right one does
                apb(true, base + 3, x"4F4B4F4C", value);
                apb(false, base + 3, x"00000000", value);
                check_equal(value(0), '1', "still runs after a wrong value");
                apb(true, base + 3, x"4F4B4F4B", value);
                apb(false, base + 3, x"00000000", value);
                check_equal(value(0), '0', "stopped");
                wait for 3 * fallback_seconds * 8 us;
                check_equal(cfg_ENA, '0', "no enable");
                check_equal(cfg_CONFIG, '0', "no reconfiguration");

            elsif run("the_fallback_timer_not_stopped_reloads_the_golden_image") then
                apb(true, base + 3, x"00000000", value);
                wait until cfg_CONFIG = '1' for 2 * fallback_seconds * 8 us;
                check_equal(cfg_CONFIG, '1', "reconfiguration");
                check(now >= fallback_seconds * 8 us, "not before its time");
                check(now < fallback_seconds * 8 us + 10 us, "at its time");
                check_equal(cfg_CBSEL, std_logic_vector'("00"), "the golden image");
                check_equal(cfg_ENA, '1', "enable");
            end if;
        end loop;

        done <= true;
        test_runner_cleanup(runner);
    end process;
end architecture sim;
