library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.mdio_pkg.all;
    use work.fpga_interconnect_pkg.all;

-- MDIO access through the fpga_interconnect bus.
-- register map, offsets from g_base_address
--   0  W: command 1 read, 2 write    R: status bit0 ready
--   1  RW phy address
--   2  RW register address
--   3  RW write data
--   4  R  read data of last read
--   5  R  number of completed transactions
--   6  R  PHY supervisor status, see ethernet_phy_init
entity mdio_bus_bridge is
    generic(
        g_base_address : natural := 300
    );
    port(
        clock    : in std_logic;
        bus_in   : in fpga_interconnect_record;
        bus_out  : out fpga_interconnect_record;
        mdio_in  : out mdio_in_record;
        mdio_out : in mdio_out_record;
        phy_status : in std_logic_vector(31 downto 0) := (others => '0')
    );
end entity mdio_bus_bridge;

architecture rtl of mdio_bus_bridge is

    signal phy_address      : std_logic_vector(31 downto 0) := (others => '0');
    signal register_address : std_logic_vector(31 downto 0) := (others => '0');
    signal write_data       : std_logic_vector(31 downto 0) := (others => '0');
    signal read_data        : std_logic_vector(31 downto 0) := (others => '0');
    signal transactions     : unsigned(31 downto 0) := (others => '0');
    signal status           : std_logic_vector(31 downto 0) := (others => '0');

begin

    status(0) <= '1' when mdio_is_ready(mdio_out) else '0';
    status(31 downto 1) <= (others => '0');

    bridge : process(clock)
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            init_mdio(mdio_in);

            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0, status);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 1, phy_address);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 2, register_address);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 3, write_data);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4, read_data);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5, std_logic_vector(transactions));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, phy_status);

            if write_is_requested_to_address(bus_in, g_base_address) and mdio_is_ready(mdio_out) then
                CASE get_slv_data(bus_in)(1 downto 0) is
                    WHEN "01" =>
                        request_mdio_read(mdio_in
                            , to_integer(unsigned(phy_address(4 downto 0)))
                            , to_integer(unsigned(register_address(4 downto 0))));
                    WHEN "10" =>
                        request_mdio_write(mdio_in
                            , to_integer(unsigned(phy_address(4 downto 0)))
                            , to_integer(unsigned(register_address(4 downto 0)))
                            , write_data(15 downto 0));
                    WHEN others => -- do nothing
                end CASE;
            end if;

            if mdio_transaction_is_done(mdio_out) then
                read_data    <= x"0000" & get_mdio_read_data(mdio_out);
                transactions <= transactions + 1;
            end if;
        end if;
    end process bridge;

end architecture rtl;
