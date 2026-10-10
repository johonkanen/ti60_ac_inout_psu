library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;

-- The output bridge's complementary (bipolar) modulation as functions, in clock counts, so the
-- same ones serve the RTL (full_bridge_modulator computes get_switch_times' result on a
-- fixed_dsp, full_bridge_carrier maps it with get_bridge_state) and an ODE simulation
-- (get_segments). Split as the DHB's (dhb_modulator_pkg): the modulator gives each period's
-- switch times, the carrier maps them onto its count, dhb_deadtime adds the dead time.
--
-- Leg 1's high side is on from the period's start while the count is below leg1_off, leg 2 its
-- complement: the bridge's output averages (2 * duty - 1) * Vdc. The bridge state bit 0 is
-- leg 1's ('1' its high side), bit 1 leg 2's.
package full_bridge_modulator_pkg is

    type fb_switch_times is record
        period   : dhb_count;   -- clock counts
        leg1_off : dhb_count;   -- leg 1 high below it, low from it
    end record;

    constant fb_switch_times_init : fb_switch_times := (others => 0);

    -- the duty kept within 0 .. 1
    function limit_duty(duty : fixed_number) return fixed_number;

    -- the switch times of a period: leg1_off = floor(duty * period), as the fixed_dsp gives
    -- it (the duty's 20 fractional bits off)
    function get_switch_times(period : dhb_count; duty : fixed_number) return fb_switch_times;

    -- the bridge state at a count: "10" leg 1 high (leg 2 low), "01" the other way
    function get_bridge_state(counter : dhb_count; times : fb_switch_times) return dhb_bridge_state;

    -- a period's segments for an ODE simulation: from start, the state, in clock counts
    type fb_segment is record
        start : dhb_count;
        state : dhb_bridge_state;
    end record;
    type fb_segment_array is array (0 to 1) of fb_segment;
    function get_segments(times : fb_switch_times) return fb_segment_array;

end package full_bridge_modulator_pkg;

package body full_bridge_modulator_pkg is

    constant one : fixed_number := to_fixed(1.0);

    function limit_duty(duty : fixed_number) return fixed_number is
    begin
        if duty < fixed_zero then
            return fixed_zero;
        elsif duty > one then
            return one;
        end if;
        return duty;
    end function;

    function get_switch_times(period : dhb_count; duty : fixed_number) return fb_switch_times is
        variable product : signed(63 downto 0);
    begin
        product := limit_duty(duty) * to_signed(period, 32);
        return (period => period, leg1_off => to_integer(shift_right(product, fixed_radix)(16 downto 0)));
    end function;

    function get_bridge_state(counter : dhb_count; times : fb_switch_times) return dhb_bridge_state is
    begin
        if counter < times.leg1_off then
            return "10";
        end if;
        return "01";
    end function;

    function get_segments(times : fb_switch_times) return fb_segment_array is
    begin
        return ((start => 0, state => "10"), (start => times.leg1_off, state => "01"));
    end function;

end package body full_bridge_modulator_pkg;
