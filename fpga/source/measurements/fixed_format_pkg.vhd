library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;

-- The number format of all signal processing in the FPGA: 32 bit signed fixed point with 20
-- fractional bits, a range of +-2048 at a resolution of 1/2^20 (about 1e-6). Measurements
-- are scaled into it (measurement_scaler) and controllers compute in it.
package fixed_format_pkg is

    constant fixed_wordlength : natural := 32;
    constant fixed_radix      : natural := 20;   -- fractional bits

    subtype fixed_number is signed(fixed_wordlength-1 downto 0);
    type fixed_array is array (natural range <>) of fixed_number;

    constant fixed_zero : fixed_number := (others => '0');
    constant fixed_max  : fixed_number := (fixed_wordlength-1 => '0', others => '1');
    constant fixed_min  : fixed_number := (fixed_wordlength-1 => '1', others => '0');

    -- a real as fixed point with radix fractional bits (constants, tests)
    function to_fixed(value : real; radix : natural := fixed_radix) return fixed_number;
    -- and back
    function to_real(value : signed; radix : natural := fixed_radix) return real;

end package fixed_format_pkg;

package body fixed_format_pkg is

    function to_fixed(value : real; radix : natural := fixed_radix) return fixed_number is
    begin
        return to_signed(integer(round(value * 2.0**radix)), fixed_wordlength);
    end function;

    function to_real(value : signed; radix : natural := fixed_radix) return real is
    begin
        return real(to_integer(value)) / 2.0**radix;
    end function;

end package body fixed_format_pkg;
