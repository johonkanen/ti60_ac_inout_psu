library ieee;
    use ieee.std_logic_1164.all;

-- Sapphire SoC for running Zephyr on the Ti60F225 development kit.
--   system_clock : 100 MHz from the 25 MHz board oscillator
--   uart         : console on the kit's FT4232H channel C
--   gpio(3:0)    : the kit's LEDs, see the leds node of the zephyr titanium_ti60_f225 board
--   reset_n      : push button, also held while the PLL is not locked
--   jtag_*       : FPGA JTAG TAP USER1 for the RISC-V debugger
entity ti60_evm_top is
    port (
        system_clock : in std_logic
        ;pll_locked  : in std_logic
        ;reset_n     : in std_logic

        ;uart_rx     : in std_logic
        ;uart_tx     : out std_logic

        ;gpio_in     : in std_logic_vector(3 downto 0)
        ;gpio_out    : out std_logic_vector(3 downto 0)
        ;gpio_oe     : out std_logic_vector(3 downto 0)

        ;jtag_inst1_TCK     : in std_logic
        ;jtag_inst1_TDI     : in std_logic
        ;jtag_inst1_TDO     : out std_logic
        ;jtag_inst1_SEL     : in std_logic
        ;jtag_inst1_CAPTURE : in std_logic
        ;jtag_inst1_SHIFT   : in std_logic
        ;jtag_inst1_UPDATE  : in std_logic
        ;jtag_inst1_RESET   : in std_logic
    );
end entity ti60_evm_top;

architecture rtl of ti60_evm_top is

    component sapphire is
    port (
        io_systemClk : in std_logic;
        jtagCtrl_enable : in std_logic;
        jtagCtrl_tdi : in std_logic;
        jtagCtrl_capture : in std_logic;
        jtagCtrl_shift : in std_logic;
        jtagCtrl_update : in std_logic;
        jtagCtrl_reset : in std_logic;
        jtagCtrl_tdo : out std_logic;
        jtagCtrl_tck : in std_logic;
        io_asyncReset : in std_logic;
        io_systemReset : out std_logic;
        system_uart_0_io_txd : out std_logic;
        system_uart_0_io_rxd : in std_logic;
        system_gpio_0_io_writeEnable : out std_logic_vector(3 downto 0);
        system_gpio_0_io_write : out std_logic_vector(3 downto 0);
        system_gpio_0_io_read : in std_logic_vector(3 downto 0)
    );
    end component sapphire;

    signal reset_meta     : std_logic_vector(4 downto 0) := (others => '1');
    signal system_reset   : std_logic;

begin

------------------------------------------------------------------------
    -- asynchronous assert, synchronous release
    process(system_clock, reset_n, pll_locked) is
    begin
        if reset_n = '0' or pll_locked = '0' then
            reset_meta <= (others => '1');
        elsif rising_edge(system_clock) then
            reset_meta <= reset_meta(reset_meta'left-1 downto 0) & '0';
        end if;
    end process;
------------------------------------------------------------------------
    u_sapphire : sapphire
    port map(
        io_systemClk                  => system_clock
        ,jtagCtrl_enable              => jtag_inst1_SEL
        ,jtagCtrl_tdi                 => jtag_inst1_TDI
        ,jtagCtrl_capture             => jtag_inst1_CAPTURE
        ,jtagCtrl_shift               => jtag_inst1_SHIFT
        ,jtagCtrl_update              => jtag_inst1_UPDATE
        ,jtagCtrl_reset               => jtag_inst1_RESET
        ,jtagCtrl_tdo                 => jtag_inst1_TDO
        ,jtagCtrl_tck                 => jtag_inst1_TCK
        ,io_asyncReset                => reset_meta(reset_meta'left)
        ,io_systemReset               => system_reset
        ,system_uart_0_io_txd         => uart_tx
        ,system_uart_0_io_rxd         => uart_rx
        ,system_gpio_0_io_writeEnable => gpio_oe
        ,system_gpio_0_io_write       => gpio_out
        ,system_gpio_0_io_read        => gpio_in
    );
------------------------------------------------------------------------
end rtl;
