library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- The Ethernet subsystem's control registers on an APB3 slave (the upper half of the MAC's
-- APB window, MAC base + 0x200): software resets of the MAC, the PHY and the two DMA channels,
-- each a register whose bit 0 holds its reset (1: in reset), all released after the system
-- reset. Zero wait states: the read data is valid in the access phase.
--
--   0x00 RW bit0 the MAC's reset (its protocol logic and its register bridge)
--   0x04 RW bit0 the PHY's reset (to its pin)
--   0x08 RW bit0 the receive DMA channel's reset
--   0x0c RW bit0 the transmit DMA channel's reset (and the checksum inserter's)
-- Other addresses read 0, writes to them are ignored.
entity tsemac_control is
    port(
        clock   : in std_logic;
        reset   : in std_logic;   -- synchronous, active high
        paddr   : in std_logic_vector(8 downto 0);
        psel    : in std_logic;
        penable : in std_logic;
        pwrite  : in std_logic;
        pwdata  : in std_logic_vector(31 downto 0);
        pready  : out std_logic;
        prdata  : out std_logic_vector(31 downto 0);
        pslverror : out std_logic;

        mac_reset    : out std_logic;
        phy_reset    : out std_logic;
        dma_rx_reset : out std_logic;
        dma_tx_reset : out std_logic
    );
end entity tsemac_control;

architecture rtl of tsemac_control is

    signal resets : std_logic_vector(0 to 3) := (others => '0');

    -- the register an address selects, 4 none
    function register_of(address : std_logic_vector(8 downto 0)) return natural is
    begin
        if address(8 downto 4) = "00000" then
            return to_integer(unsigned(address(3 downto 2)));
        end if;
        return 4;
    end function;

begin

    mac_reset    <= resets(0);
    phy_reset    <= resets(1);
    dma_rx_reset <= resets(2);
    dma_tx_reset <= resets(3);

    pready    <= '1';
    pslverror <= '0';

    read_mux : process(all)
    begin
        prdata <= (others => '0');
        if register_of(paddr) < 4 then
            prdata(0) <= resets(register_of(paddr));
        end if;
    end process;

    registers : process(clock)
    begin
        if rising_edge(clock) then
            if reset = '1' then
                resets <= (others => '0');
            elsif psel = '1' and penable = '1' and pwrite = '1' and register_of(paddr) < 4 then
                resets(register_of(paddr)) <= pwdata(0);
            end if;
        end if;
    end process;

end architecture rtl;
