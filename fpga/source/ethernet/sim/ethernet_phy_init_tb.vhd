LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.mdio_pkg.all;

-- ethernet_phy_init with the MDIO driver and PHY model: configuration, link polling,
-- reconfiguration after a PHY reset and bridge requests mixed with the polling.
entity ethernet_phy_init_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of ethernet_phy_init_tb is

    constant clock_period : time := 8 ns;
    constant poll_interval : natural := 400;

    signal clock      : std_logic := '0';
    signal bridge_in  : mdio_in_record := init_mdio_in;
    signal bridge_out : mdio_out_record;
    signal mdio_in    : mdio_in_record;
    signal mdio_out   : mdio_out_record;
    signal fpga_out   : mdio_fpga_out_record;
    signal mdio       : std_logic;
    signal is_done, link_is_up, link_came_up : boolean;
    signal status     : std_logic_vector(31 downto 0);
    signal reset_phy, link : boolean := false;
    signal link_up_pulses  : natural := 0;

    function configurations(s : std_logic_vector) return natural is
    begin
        return to_integer(unsigned(s(15 downto 0)));
    end function;

begin

    clock <= not clock after clock_period/2;

    u_phy_init : entity work.ethernet_phy_init
    generic map(g_phy_address => 0, g_start_delay => 100, g_poll_interval => poll_interval)
    port map(clock, bridge_in, bridge_out, mdio_in, mdio_out, is_done, link_is_up, link_came_up, status);

    u_driver : entity work.mdio_driver
    generic map(g_clocks_per_half_mdc => 2)
    port map(clock, mdio_in, mdio_out, fpga_out, mdio);

    mdio <= fpga_out.mdio_out when fpga_out.mdio_oe = '1' else 'H';

    u_phy : entity work.mdio_phy_model
    generic map(g_phy_address => 0, g_output_delay => 10 ns)
    port map(fpga_out.mdc, mdio, reset_phy, link);

    count_pulses : process(clock)
    begin
        if rising_edge(clock) and link_came_up then
            link_up_pulses <= link_up_pulses + 1;
        end if;
    end process;

    main : process

        -- a bridge transaction like mdio_bus_bridge does it
        procedure bridge(command : mdio_command; reg : natural; data : std_logic_vector(15 downto 0) := x"0000") is
        begin
            wait until rising_edge(clock) and mdio_is_ready(bridge_out);
            if command = read then
                request_mdio_read(bridge_in, 0, reg);
            else
                request_mdio_write(bridge_in, 0, reg, data);
            end if;
            wait until rising_edge(clock);
            init_mdio(bridge_in);
            wait until rising_edge(clock) and mdio_transaction_is_done(bridge_out) for 100 us;
            check(mdio_transaction_is_done(bridge_out), "bridge transaction finished");
        end procedure;

        procedure check_register(reg : natural; expected : std_logic_vector(15 downto 0)) is
        begin
            bridge(read, reg);
            check_equal(get_mdio_read_data(bridge_out), expected, "register " & integer'image(reg));
        end procedure;

        variable wrong : natural;

    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("configures_phy") then
                wait until is_done for 1 ms;
                check(is_done, "configured");
                check_register(16#17#, x"C500");
                check_register(16#04#, x"0001");
                check_register(16#09#, x"0200");
                check_register(16#00#, x"1240");
                check_equal(configurations(status), 1, "configurations");

            elsif run("link_up_restarts_rx_pll") then
                wait until is_done for 1 ms;
                wait for 100 us;
                check_equal(link_up_pulses, 0, "no link yet");
                link <= true;
                wait until link_is_up for 1 ms;
                check(link_is_up, "link seen");
                wait until rising_edge(clock);
                check(status(31) = '1', "link bit in status");
                wait for 200 us;
                check_equal(link_up_pulses, 1, "one pulse per link up");
                link <= false;
                wait until not link_is_up for 1 ms;
                link <= true;
                wait until link_is_up for 1 ms;
                wait for 10 us;
                check_equal(link_up_pulses, 2, "pulse after the link came back");
                check_equal(to_integer(unsigned(status(30 downto 16))), 2, "link ups in status");

            elsif run("reconfigures_after_phy_reset") then
                wait until is_done for 1 ms;
                link <= true;
                wait until link_is_up for 1 ms;
                reset_phy <= true;
                wait until configurations(status) = 2 for 1 ms;
                check_equal(configurations(status), 2, "configured again");
                check_register(16#17#, x"C500");
                wait for 200 us;
                check_equal(configurations(status), 2, "stays configured");
                check_equal(link_up_pulses, 2, "link up again after the reconfiguration");

            elsif run("bridge_shares_mdio_with_polling") then
                wait until is_done for 1 ms;
                link <= true;
                wrong := 0;
                for i in 0 to 199 loop
                    bridge(read, 2);
                    if get_mdio_read_data(bridge_out) /= x"D565" then
                        wrong := wrong + 1;
                    end if;
                    -- vary the phase against the polling
                    for k in 0 to i mod 7 loop
                        wait until rising_edge(clock);
                    end loop;
                end loop;
                check_equal(wrong, 0, "wrong bridge reads");
                check(link_is_up, "polling went on");
                check_equal(configurations(status), 1, "no reconfiguration");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end vunit_simulation;
