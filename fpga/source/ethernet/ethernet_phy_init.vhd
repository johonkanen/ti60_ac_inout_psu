library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.mdio_pkg.all;

-- Configures the PEF7071 and keeps it configured:
--   MIICTRL  (0x17) = 0xC500  RX clock kept running, RX skew 2.0 ns, 3.3 V, TX skew 2.5 ns
--   ANAR     (0x04) = 0x0001  no 10/100 abilities advertised
--   GCTRL    (0x09) = 0x0200  1000BASE-T full duplex only
--   BMCR     (0x00) = 0x1240  auto negotiation enabled and restarted
-- RGMII skews measured on the board: RX passes 0.5..3.0 ns, TX 2.0..3.0 ns.
--
-- Every g_poll_interval MIICTRL is read back and the configuration is written again when it
-- has been lost (the PHY was reset, lost power or did not answer yet), then the link status
-- is read. link_came_up pulses when the link goes up, so the receive PLL can be restarted on
-- the new RX_CLK.
--
-- The bus bridge shares the MDIO driver: its requests pass through, bridge_out hides the
-- driver while an own transaction is running.
--
-- status: bits 15:0 configurations written, bits 30:16 link ups, bit 31 link is up
entity ethernet_phy_init is
    generic(
        g_phy_address      : natural := 0;
        g_start_delay      : positive := 1_250_000;    -- 10 ms at 125 MHz
        g_poll_interval    : positive := 12_500_000    -- 100 ms at 125 MHz
    );
    port(
        clock        : in std_logic;
        bridge_in    : in mdio_in_record;     -- requests from the bus bridge
        bridge_out   : out mdio_out_record;
        mdio_in      : out mdio_in_record;    -- to the mdio driver
        mdio_out     : in mdio_out_record;
        is_done      : out boolean;           -- configured at least once
        link_is_up   : out boolean;
        link_came_up : out boolean;
        status       : out std_logic_vector(31 downto 0)
    );
end entity ethernet_phy_init;

architecture rtl of ethernet_phy_init is

    type init_write is record
        register_address : natural range 0 to 31;
        data             : std_logic_vector(15 downto 0);
    end record;
    type init_list is array (natural range <>) of init_write;

    constant init_sequence : init_list := (
        (16#17#, x"C500"),
        (16#04#, x"0001"),
        (16#09#, x"0200"),
        (16#00#, x"1240"));

    constant miictrl_register : natural := 16#17#;

    type states is (start_delay, configure, read_miictrl, check_miictrl, read_status, check_status, wait_poll);
    signal state : states := start_delay;

    signal timer   : natural range 0 to maximum(g_start_delay, g_poll_interval) := 0;
    signal step    : natural range 0 to init_sequence'length := 0;
    signal request : mdio_in_record := init_mdio_in;

    -- MDIO ownership: claim hides the driver from the bridge, own marks a running own transaction
    signal claim, own      : boolean := false;
    signal result_is_ready : boolean := false;
    signal result          : std_logic_vector(15 downto 0) := (others => '0');

    signal configured     : boolean := false;
    signal link           : boolean := false;
    signal configurations : unsigned(15 downto 0) := (others => '0');
    signal link_ups       : unsigned(14 downto 0) := (others => '0');
    signal link_bit       : std_logic;

begin

    is_done    <= configured;
    link_is_up <= link;
    status     <= link_bit & std_logic_vector(link_ups) & std_logic_vector(configurations);
    link_bit   <= '1' when link else '0';

    mdio_in <= request when request.command /= none else bridge_in;
    bridge_out <= (
        is_ready            => mdio_out.is_ready and not claim,
        transaction_is_done => mdio_out.transaction_is_done and not own,
        read_data           => mdio_out.read_data);

    process(clock)
        -- starts an own transaction when the driver is free, result_is_ready when done
        procedure transfer(command : mdio_command; register_address : natural; data : std_logic_vector(15 downto 0)) is
        begin
            if own then
                null;
            elsif not claim then
                if mdio_is_ready(mdio_out) and bridge_in.command = none then
                    claim <= true;
                end if;
            elsif bridge_in.command /= none then
                -- the bridge started on the same clock, let it finish first
                claim <= false;
            elsif command = read then
                request_mdio_read(request, g_phy_address, register_address);
                own <= true;
            else
                request_mdio_write(request, g_phy_address, register_address, data);
                own <= true;
            end if;
        end procedure;
    begin
        if rising_edge(clock) then
            init_mdio(request);
            link_came_up    <= false;
            result_is_ready <= false;
            if own and mdio_transaction_is_done(mdio_out) then
                own    <= false;
                claim  <= false;
                result <= get_mdio_read_data(mdio_out);
                result_is_ready <= true;
            end if;

            CASE state is
                WHEN start_delay =>
                    if timer < g_start_delay then
                        timer <= timer + 1;
                    else
                        step  <= 0;
                        state <= configure;
                    end if;

                WHEN configure =>
                    if result_is_ready then
                        if step = init_sequence'length - 1 then
                            configured     <= true;
                            configurations <= configurations + 1;
                            timer <= 0;
                            state <= wait_poll;
                        else
                            step <= step + 1;
                        end if;
                    elsif not own then
                        transfer(write, init_sequence(step).register_address, init_sequence(step).data);
                    end if;

                WHEN wait_poll =>
                    if timer < g_poll_interval then
                        timer <= timer + 1;
                    else
                        state <= read_miictrl;
                    end if;

                WHEN read_miictrl =>
                    if result_is_ready then
                        state <= check_miictrl;
                    elsif not own then
                        transfer(read, miictrl_register, x"0000");
                    end if;

                WHEN check_miictrl =>
                    if result /= init_sequence(0).data then
                        -- configuration lost, the link restarts with the new one
                        link  <= false;
                        step  <= 0;
                        state <= configure;
                    else
                        state <= read_status;
                    end if;

                WHEN read_status =>
                    if result_is_ready then
                        state <= check_status;
                    elsif not own then
                        transfer(read, basic_status_register, x"0000");
                    end if;

                WHEN check_status =>
                    -- the link bit latches low, so a drop between polls is seen as well
                    link <= result(2) = '1';
                    if result(2) = '1' and not link then
                        link_came_up <= true;
                        link_ups     <= link_ups + 1;
                    end if;
                    timer <= 0;
                    state <= wait_poll;
            end CASE;
        end if;
    end process;

end architecture rtl;
