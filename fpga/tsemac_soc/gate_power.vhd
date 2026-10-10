library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

-- The power stage's six isolated gate driver supplies, each an isolated converter run by a PWM
-- output, as the power supply design's aux_pwm: period clocks long and high for duty of them
-- (239.5 kHz, 44 %: the design's 501 and 220 clocks at 120 MHz). They also power the analog
-- side of the sigma-delta current modulators. All off at power up; the gate drives themselves
-- stay held low elsewhere.
--
-- Two rules of the hardware are kept here, whatever the software writes:
--   - one supply comes on at a time: their inrush together can overload the auxiliary supply.
--     A supply whose enable bit is set comes on stagger_ms after the previous one did; a
--     cleared bit turns its supply off at once.
--   - they are noisy, so no two switch together: output n runs (n-1) * shift clocks behind
--     output 1 (87, a sixth of the period by default: with the default duty every edge of the
--     six is at least 32 clocks from the others). Shifts past the period are not wrapped:
--     keep 5 * shift below the period.
--
-- register map, offsets from g_base_address
--   0  RW enables: bit n-1 gate_power<n>_pwm, n = 1..6
--   1  RW period in clocks, 2..65535 (522: 239.5 kHz at 125 MHz)
--   2  RW high clocks, at most half the period (229)
--   3  RW milliseconds between two supplies coming on, 1..65535 (100)
--   4  RW phase shift between neighbouring outputs in clocks (87)
--   5  R  the supplies on now, bit n-1 for n
entity gate_power is
    generic(
        g_base_address : natural := 760;
        g_clock_hz     : positive := 125_000_000
    );
    port(
        clock   : in std_logic;
        bus_in  : in fpga_interconnect_record;
        bus_out : out fpga_interconnect_record;
        pwm     : out std_logic_vector(1 to 6)
    );
end entity gate_power;

architecture rtl of gate_power is

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    constant clocks_per_ms : positive := g_clock_hz / 1000;

    signal enables_r : std_logic_vector(31 downto 0) := (others => '0');
    signal period_r  : std_logic_vector(31 downto 0) := to_slv(522);
    signal duty_r    : std_logic_vector(31 downto 0) := to_slv(229);
    signal stagger_r : std_logic_vector(31 downto 0) := to_slv(100);
    signal shift_r   : std_logic_vector(31 downto 0) := to_slv(87);

    signal is_on       : std_logic_vector(1 to 6) := (others => '0');
    signal ms_count    : natural range 0 to clocks_per_ms - 1 := 0;
    signal since_on_ms : natural range 0 to 2**16-1 := 2**16-1;   -- since a supply last came on

    type counter_array is array (1 to 6) of natural range 0 to 2**16-1;
    signal counters : counter_array := (others => 0);
    signal starts   : counter_array := (0, 87, 174, 261, 348, 435);  -- where output n starts
    signal period   : natural range 2 to 2**16-1 := 522;
    signal duty     : natural range 0 to 2**15 := 229;
    signal pwm_r    : std_logic_vector(1 to 6) := (others => '0');

begin

    pwm <= pwm_r;

    process(clock)
        variable p : natural range 0 to 2**16-1;
        variable d : natural range 0 to 2**16-1;
        variable started : boolean;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address(bus_in, bus_out, g_base_address + 0, enables_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, period_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, duty_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, stagger_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, shift_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5,
                x"000000" & "00" & is_on(6) & is_on(5) & is_on(4) & is_on(3) & is_on(2) & is_on(1));

            -- the supplies: off at once, on one at a time
            if ms_count < clocks_per_ms - 1 then
                ms_count <= ms_count + 1;
            else
                ms_count <= 0;
                if since_on_ms < 2**16-1 then
                    since_on_ms <= since_on_ms + 1;
                end if;
            end if;
            started := false;
            for n in 1 to 6 loop
                if enables_r(n - 1) = '0' then
                    is_on(n) <= '0';
                elsif is_on(n) = '0' and not started
                      and since_on_ms >= to_integer(unsigned(stagger_r(15 downto 0))) then
                    is_on(n)    <= '1';
                    started     := true;
                    since_on_ms <= 0;
                    ms_count    <= 0;
                end if;
            end loop;

            -- output 1's counter sets the period and takes new settings at its start; the
            -- others follow it shift clocks apart
            if counters(1) < period - 1 then
                counters(1) <= counters(1) + 1;
            else
                counters(1) <= 0;
                p := to_integer(unsigned(period_r(15 downto 0)));
                if p >= 2 then
                    period <= p;
                else
                    p := period;
                end if;
                d := to_integer(unsigned(duty_r(15 downto 0)));
                if d > p / 2 then
                    d := p / 2;
                end if;
                duty <= d;
                for n in 2 to 6 loop
                    starts(n) <= to_integer(resize((n - 1) * unsigned(shift_r(15 downto 0)), 16));
                end loop;
            end if;
            -- the others start again where output 1's counter passes their start
            for n in 2 to 6 loop
                if counters(1) = starts(n) then
                    counters(n) <= 1;       -- 0 with output 1's at its start, a clock ago
                elsif counters(n) < period - 1 then
                    counters(n) <= counters(n) + 1;
                else
                    counters(n) <= 0;
                end if;
            end loop;

            for n in 1 to 6 loop
                if is_on(n) = '1' and counters(n) < duty then
                    pwm_r(n) <= '1';
                else
                    pwm_r(n) <= '0';
                end if;
            end loop;
        end if;
    end process;

end architecture rtl;
