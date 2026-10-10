library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

-- The face panel's touch screen: its XPT2046 controller polled over SPI (its pen interrupt is not
-- connected). Every g_poll_ms, four conversions of 12 bits, each a command byte and 16 clocks
-- reading back (the result's 12 bits after a busy clock, then zeros): the pressure's Z1 and Z2,
-- then X and Y, all differential, powered down between (PD 00). A touch is Z1 at or over the
-- threshold; X and Y are kept from the last poll that was one.
--
-- SPI mode 0 at 125 MHz / (2 g_half_period): 1 MHz by default (the XPT2046 takes up to 2.5 MHz
-- at 3.3 V). Its data out changes after DCLK's fall and is taken at the next rise.
--
-- register map, offsets from g_base_address
--   0  R  bit0 touched (the last poll)
--   1  R  X, 0..4095, of the last touch     2 R Y
--   3  R  Z1 of the last poll               4 R Z2
--   5  R  polls done
--   6  RW the touch threshold on Z1 (100)
entity touch_reader is
    generic(
        g_base_address : natural := 1320;
        g_half_period  : natural := 62;         -- DCLK's half period, clocks
        g_poll_clocks  : natural := 625000      -- 5 ms
    );
    port(
        clock   : in std_logic;
        bus_in  : in fpga_interconnect_record;
        bus_out : out fpga_interconnect_record;
        t_cs    : out std_logic;
        t_clk   : out std_logic;
        t_din   : out std_logic;    -- to the controller
        t_dout  : in std_logic      -- from it
    );
end entity touch_reader;

architecture rtl of touch_reader is

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

    -- the command bytes: start, channel, 12 bits, differential, power down between
    type command_array is array (0 to 3) of std_logic_vector(7 downto 0);
    constant commands : command_array := (x"B0", x"C0", x"D0", x"90");   -- Z1, Z2, X, Y

    signal threshold_r : std_logic_vector(31 downto 0) := to_slv(100);
    signal polls       : unsigned(31 downto 0) := (others => '0');
    signal touched     : boolean := false;
    signal x_r, y_r, z1_r, z2_r : natural range 0 to 4095 := 0;

    type value_array is array (0 to 3) of natural range 0 to 4095;
    signal values : value_array := (others => 0);

    signal wait_count : natural range 0 to g_poll_clocks - 1 := 0;
    signal half_count : natural range 0 to g_half_period - 1 := 0;
    signal running    : boolean := false;
    signal conversion : natural range 0 to 3 := 0;
    signal clock_edge : natural range 0 to 48 := 0;     -- 24 DCLK cycles, a rise and a fall each
    signal shift_out  : std_logic_vector(7 downto 0) := (others => '0');
    signal shift_in   : std_logic_vector(15 downto 0) := (others => '0');
    signal cs_r       : std_logic := '1';
    signal clk_r      : std_logic := '0';
    signal din_r      : std_logic := '0';
    signal dout_sync  : std_logic_vector(1 downto 0) := (others => '0');

    attribute async_reg : boolean;
    attribute async_reg of dout_sync : signal is true;

begin

    t_cs  <= cs_r;
    t_clk <= clk_r;
    t_din <= din_r;

    process(clock)
        variable value : natural range 0 to 4095;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0, (31 downto 1 => '0') & bit_of(touched));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 1, to_slv(x_r));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 2, to_slv(y_r));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, to_slv(z1_r));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4, to_slv(z2_r));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5, std_logic_vector(polls));
            connect_data_to_address(bus_in, bus_out, g_base_address + 6, threshold_r);

            dout_sync <= dout_sync(0) & t_dout;

            if not running then
                cs_r  <= '1';
                clk_r <= '0';
                if wait_count < g_poll_clocks - 1 then
                    wait_count <= wait_count + 1;
                else
                    wait_count <= 0;
                    running    <= true;
                    conversion <= 0;
                    clock_edge <= 0;
                    half_count <= 0;
                    cs_r       <= '0';
                    shift_out  <= commands(0)(6 downto 0) & '0';
                    din_r      <= commands(0)(7);
                end if;
            elsif half_count < g_half_period - 1 then
                half_count <= half_count + 1;
            else
                half_count <= 0;
                if clock_edge mod 2 = 0 then
                    -- a rise: the controller takes din; data out taken (synchronized, its
                    -- level settled for a half period)
                    clk_r    <= '1';
                    shift_in <= shift_in(14 downto 0) & dout_sync(1);
                    clock_edge <= clock_edge + 1;
                else
                    -- a fall: the next command bit out (zeros after the command)
                    clk_r     <= '0';
                    din_r     <= shift_out(7);
                    shift_out <= shift_out(6 downto 0) & '0';
                    if clock_edge < 47 then
                        clock_edge <= clock_edge + 1;
                    else
                        -- the conversion's 24 clocks done: the 16 read after the command, the
                        -- busy clock first, then 12 bits, then 3 zeros
                        value := to_integer(unsigned(shift_in(14 downto 3)));
                        values(conversion) <= value;
                        clock_edge <= 0;
                        if conversion < 3 then
                            conversion <= conversion + 1;
                            shift_out  <= commands(conversion + 1)(6 downto 0) & '0';
                            din_r      <= commands(conversion + 1)(7);
                        else
                            -- the poll done
                            running <= false;
                            cs_r    <= '1';
                            din_r   <= '0';
                            polls   <= polls + 1;
                            z1_r    <= values(0);
                            z2_r    <= values(1);
                            touched <= values(0) >= to_integer(unsigned(threshold_r(11 downto 0)));
                            if values(0) >= to_integer(unsigned(threshold_r(11 downto 0))) then
                                x_r <= values(2);
                                y_r <= value;
                            end if;
                        end if;
                    end if;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
