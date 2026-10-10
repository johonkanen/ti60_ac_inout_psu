library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Minimal board bring-up design: UART register interface only.
--   address 1 : read only  0x0000acdc identification word
--   address 2 : read only  free running counter at main_clock
--   address 3 : read/write scratch register
--   address 4 : read only  pll_locked
entity uart_test_top is
    port (
        main_clock  : in std_logic
        ;pll_locked : in std_logic

        ;uart_rx    : in std_logic
        ;uart_tx    : out std_logic
    );
end entity uart_test_top;

architecture rtl of uart_test_top is

    use work.fpga_interconnect_pkg.all;

    signal bus_to_communications   : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_from_communications : fpga_interconnect_record := init_fpga_interconnect;

    signal uart_rx_meta : std_logic_vector(1 downto 0) := (others => '1');

    constant id_word        : std_logic_vector(31 downto 0) := x"0000acdc";
    signal free_run_counter : unsigned(31 downto 0)         := (others => '0');
    signal scratch          : std_logic_vector(31 downto 0) := (others => '0');
    signal pll_status       : std_logic_vector(31 downto 0) := (others => '0');

begin

------------------------------------------------------------------------
    process(main_clock) is
    begin
        if rising_edge(main_clock) then
            uart_rx_meta <= uart_rx_meta(0) & uart_rx;

            init_bus(bus_to_communications);
            free_run_counter <= free_run_counter + 1;
            pll_status       <= (0 => pll_locked, others => '0');

            connect_read_only_data_to_address(bus_from_communications , bus_to_communications , 1 , id_word);
            connect_read_only_data_to_address(bus_from_communications , bus_to_communications , 2 , std_logic_vector(free_run_counter));
            connect_data_to_address(bus_from_communications           , bus_to_communications , 3 , scratch);
            connect_read_only_data_to_address(bus_from_communications , bus_to_communications , 4 , pll_status);
        end if;
    end process;
------------------------------------------------------------------------
    u_fpga_communications : entity work.fpga_communications
    generic map(fpga_interconnect_pkg => work.fpga_interconnect_pkg, g_clock_divider => 24)
        port map(
            clock                    => main_clock
            ,uart_rx                 => uart_rx_meta(1)
            ,uart_tx                 => uart_tx
            ,bus_to_communications   => bus_to_communications
            ,bus_from_communications => bus_from_communications
        );
------------------------------------------------------------------------
end rtl;
