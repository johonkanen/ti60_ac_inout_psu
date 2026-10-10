library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fixed_format_pkg.all;
    use work.lut_divider_pkg.all;

-- The duty of a complementary modulated full bridge (full_bridge_modulator) for an output
-- voltage: the bridge averages (2 * duty - 1) * Vdc, so
--
--   m    = voltage_reference / dc_voltage   (lut_divider, the fixed point library's)
--   duty = (1 + m) / 2                      m limited to -1 .. 1
--
-- A request computes one duty from the reference and the DC voltage at the time; ready pulses
-- when duty has it (lut_divider's latency, under 30 clocks). A DC voltage below g_min_dc_voltage
-- (or negative) gives duty 0.5, no output, instead of dividing by next to nothing. limited is
-- set while the reference is beyond the DC voltage and m held at +-1.
--
-- All fixed point with 20 fractional bits (fixed_format_pkg).
entity voltage_modulator is
    generic(
        g_min_dc_voltage : real := 5.0
    );
    port(
        clock             : in std_logic;
        request           : in boolean;
        voltage_reference : in fixed_number;
        dc_voltage        : in fixed_number;
        duty              : out fixed_number;
        limited           : out boolean;
        ready             : out boolean
    );
end entity voltage_modulator;

architecture rtl of voltage_modulator is

    constant one            : fixed_number := to_fixed(1.0);
    constant half           : fixed_number := to_fixed(0.5);
    constant min_dc_voltage : fixed_number := to_fixed(g_min_dc_voltage);

    signal divider_in  : lut_divider_in_record(numerator(fixed_wordlength-1 downto 0), denominator(fixed_wordlength-1 downto 0));
    signal divider_out : lut_divider_out_record(quotient(fixed_wordlength-1 downto 0));

    -- the DC voltage was too low at the request: duty 0.5 when the quotient comes
    signal no_dc : boolean := true;

    signal duty_r    : fixed_number := half;
    signal limited_r : boolean := false;
    signal ready_r   : boolean := false;

begin

    duty    <= duty_r;
    limited <= limited_r;
    ready   <= ready_r;

    u_divider : entity work.lut_divider
    generic map(g_quotient_radix => fixed_radix)
    port map(clock => clock, lut_divider_in => divider_in, lut_divider_out => divider_out);

    process(clock)
        variable m : fixed_number;
    begin
        if rising_edge(clock) then
            init_lut_divider(divider_in);
            ready_r <= false;

            if request then
                request_lut_division(divider_in, voltage_reference, dc_voltage);
                no_dc <= dc_voltage < min_dc_voltage;
            end if;

            if divider_out.ready_with_1 = '1' then
                m := divider_out.quotient;
                limited_r <= false;
                if no_dc then
                    m := fixed_zero;
                elsif m > one then
                    m := one;
                    limited_r <= true;
                elsif m < -one then
                    m := -one;
                    limited_r <= true;
                end if;
                duty_r  <= half + shift_right(m, 1);
                ready_r <= true;
            end if;
        end if;
    end process;

end architecture rtl;
