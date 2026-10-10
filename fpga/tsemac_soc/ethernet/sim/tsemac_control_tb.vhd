LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

-- tsemac_control as the drivers use it (the MAC base + 0x200 offsets less 0x200): the four
-- resets written and read back over APB3, their outputs, other addresses reading 0 and taking
-- no writes, all released by the system reset.
entity tsemac_control_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of tsemac_control_tb is

    signal clock   : std_logic := '0';
    signal reset   : std_logic := '0';
    signal paddr   : std_logic_vector(8 downto 0) := (others => '0');
    signal psel, penable, pwrite : std_logic := '0';
    signal pwdata  : std_logic_vector(31 downto 0) := (others => '0');
    signal pready, pslverror : std_logic;
    signal prdata  : std_logic_vector(31 downto 0);
    signal mac_reset, phy_reset, dma_rx_reset, dma_tx_reset : std_logic;

begin

    clock <= not clock after 4 ns;

    dut : entity work.tsemac_control
    port map(clock, reset, paddr, psel, penable, pwrite, pwdata, pready, prdata, pslverror,
             mac_reset, phy_reset, dma_rx_reset, dma_tx_reset);

    main : process
        -- an APB3 transfer: the setup phase, then the access phase until pready
        procedure apb_write(address : natural; data : natural) is
        begin
            wait until rising_edge(clock);
            paddr  <= std_logic_vector(to_unsigned(address, 9));
            pwdata <= std_logic_vector(to_unsigned(data, 32));
            pwrite <= '1';
            psel   <= '1';
            wait until rising_edge(clock);
            penable <= '1';
            wait until rising_edge(clock) and pready = '1';
            psel    <= '0';
            penable <= '0';
            pwrite  <= '0';
            wait for 1 ns;   -- the register taken at that edge
        end procedure;

        procedure apb_read(address : natural; data : out natural) is
        begin
            wait until rising_edge(clock);
            paddr <= std_logic_vector(to_unsigned(address, 9));
            psel  <= '1';
            wait until rising_edge(clock);
            penable <= '1';
            wait until rising_edge(clock) and pready = '1';
            data := to_integer(unsigned(prdata));
            check(pslverror = '0', "no slave error");
            psel    <= '0';
            penable <= '0';
        end procedure;

        function bits(m, p, rx, tx : std_logic) return std_logic_vector is
        begin
            return m & p & rx & tx;
        end function;

        variable value : natural;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("the_resets_written_and_read_back") then
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "0000", "all released at the start");
                -- the mdio driver's sequence, then the DMA's
                apb_write(16#04#, 1);
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "0100", "the PHY in reset");
                apb_write(16#00#, 1);
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "1100", "the MAC too");
                apb_write(16#08#, 1);
                apb_write(16#0c#, 1);
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "1111", "all four");
                for n in 0 to 3 loop
                    apb_read(4 * n, value);
                    check_equal(value, 1, "register " & integer'image(n) & " reads 1");
                end loop;
                apb_write(16#00#, 0);
                apb_write(16#04#, 0);
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "0011", "the MAC and the PHY released");
                apb_read(16#00#, value);
                check_equal(value, 0, "the MAC's reads 0");
                -- only bit 0 is kept
                apb_write(16#08#, 16#7ffffffe#);
                check(dma_rx_reset = '0', "bit 0 the reset");
                apb_read(16#08#, value);
                check_equal(value, 0, "only bit 0 read back");

            elsif run("other_addresses_read_0") then
                apb_write(16#10#, 1);
                apb_write(16#1fc#, 1);
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "0000", "no reset written");
                apb_read(16#10#, value);
                check_equal(value, 0, "0x10 reads 0");
                apb_read(16#100#, value);
                check_equal(value, 0, "0x100 reads 0");

            elsif run("the_system_reset_releases_them") then
                apb_write(16#00#, 1);
                apb_write(16#0c#, 1);
                wait until rising_edge(clock);
                reset <= '1';
                wait until rising_edge(clock);
                reset <= '0';
                wait until rising_edge(clock);
                check(bits(mac_reset, phy_reset, dma_rx_reset, dma_tx_reset) = "0000", "all released");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 100 us);

end vunit_simulation;
