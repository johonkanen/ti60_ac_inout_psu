library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fixed_format_pkg.all;

-- The ports of pi_controller: a PI controller in the FPGA's fixed point format (32 bit, 20
-- fractional bits, fixed_format_pkg) on the fixed point library's fixed_dsp.
package pi_controller_pkg is

    type pi_controller_in_record is record
        request     : boolean;        -- strobe: a new measurement, compute
        reset       : boolean;        -- strobe: integrator and output to zero
        reference   : fixed_number;
        measurement : fixed_number;
        kp          : fixed_number;   -- output units per measurement unit
        ki          : fixed_number;   -- output units per measurement unit and request
        limit_high  : fixed_number;   -- the output and the integrator kept within
        limit_low   : fixed_number;   -- limit_low .. limit_high (limit_low < limit_high)
    end record;

    type pi_controller_out_record is record
        ready     : boolean;          -- strobe: output is new
        output    : fixed_number;
        saturated : boolean;          -- the output is at a limit
    end record;

    constant pi_controller_in_init : pi_controller_in_record := (
        request     => false,
        reset       => false,
        reference   => fixed_zero,
        measurement => fixed_zero,
        kp          => fixed_zero,
        ki          => fixed_zero,
        limit_high  => fixed_zero,
        limit_low   => fixed_zero);

    constant pi_controller_out_init : pi_controller_out_record := (
        ready     => false,
        output    => fixed_zero,
        saturated => false);

end package pi_controller_pkg;
