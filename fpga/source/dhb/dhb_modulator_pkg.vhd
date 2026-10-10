library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Single-phase-shift (SPS) modulation of a dual active half bridge (DHB): one half-bridge leg
-- on each side of the transformer, both at a 50 % duty, the secondary shifted against the
-- primary. Everything is in clock counts, so the same functions serve the RTL (dhb_modulator,
-- dhb_carrier) and an ODE simulation of the converter, which converts counts to seconds with
-- its clock period.
--
-- The carrier counts 0 .. period-1. The primary leg is high for the first half of it,
-- [0, period/2). The secondary leg is high for period/2 counts starting at the phase:
--   phase > 0  the primary leads, power flows from the primary to the secondary
--   phase < 0  the secondary leads, power flows back
-- |phase| is at most period/2 (a quarter of a period, period/4, gives the most power).
--
-- A bridge state is one bit per leg, leg 0 the primary and leg 1 the secondary, '1' with the
-- high side switch on. The dead time between a leg's switches is added afterwards
-- (dhb_deadtime), it is not part of the state.
--
-- The four sub-intervals of a period, from carrier count 0, phase a = |phase| (h = period/2):
--   phase >= 0 :  "10" a,  "11" h-a,  "01" a,  "00" period-h-a
--   phase <  0 :  "11" h-a,  "10" a,  "00" period-h-a,  "01" a
-- get_segments returns them for a simulation that steps from one sub-interval to the next;
-- get_bridge_state gives the same states count by count, as the carrier does.
package dhb_modulator_pkg is

    subtype dhb_count is natural range 0 to 2**16-1;
    subtype dhb_phase is integer range -2**15 to 2**15-1;
    subtype dhb_bridge_state is std_logic_vector(0 to 1);   -- (0) primary, (1) secondary

    constant primary   : natural := 0;
    constant secondary : natural := 1;

    -- the carrier counts at which each leg's high side state starts and ends; a leg is high
    -- from its _on count up to, not including, its _off count, wrapping at period
    type dhb_switch_times is record
        period        : dhb_count;
        primary_on    : dhb_count;
        primary_off   : dhb_count;
        secondary_on  : dhb_count;
        secondary_off : dhb_count;
    end record;

    constant dhb_switch_times_init : dhb_switch_times := (others => 0);

    type dhb_segment is record
        state  : dhb_bridge_state;
        length : dhb_count;      -- clock counts, may be 0
    end record;

    type dhb_segment_array is array (0 to 3) of dhb_segment;

    -- a clock count per leg, (0) primary, (1) secondary
    type dhb_count_array is array (0 to 1) of dhb_count;

    -- a leg's two gates, '1' on
    type dhb_leg_gates is record
        high : std_logic;
        low  : std_logic;
    end record;

    constant dhb_leg_gates_off : dhb_leg_gates := (others => '0');

    -- both legs' gates, (primary) and (secondary)
    type dhb_gates is array (0 to 1) of dhb_leg_gates;

    constant dhb_gates_off : dhb_gates := (others => dhb_leg_gates_off);

    -- the secondary DC link's precharge (dhb_precharge)
    type dhb_precharge_state is (idle, charging, charged, failed);

    type dhb_precharge_settings is record
        deadtime_start  : dhb_count;   -- the primary's first dead time: most of a half period
        deadtime_final  : dhb_count;   -- the normal dead time, the narrowest it ramps to
        ramp_step       : dhb_count;   -- clocks of dead time less a step
        ramp_periods    : dhb_count;   -- carrier periods a step
        margin          : natural;     -- charged at secondary >= primary - margin
        timeout_periods : natural;     -- periods at deadtime_final before failing
    end record;

    -- phase clamped to +-max_phase and to +-period/2
    function limit_phase(phase : integer; period : dhb_count; max_phase : dhb_count) return dhb_phase;

    function get_switch_times(period : dhb_count; phase : integer) return dhb_switch_times;

    -- the bridge state at carrier count counter (0 .. period-1)
    function get_bridge_state(counter : dhb_count; times : dhb_switch_times) return dhb_bridge_state;

    -- the four sub-intervals of a period from carrier count 0, their lengths summing to period
    function get_segments(period : dhb_count; phase : integer) return dhb_segment_array;

end package dhb_modulator_pkg;

package body dhb_modulator_pkg is

    function limit_phase(phase : integer; period : dhb_count; max_phase : dhb_count) return dhb_phase is
        variable limit : integer := max_phase;
    begin
        if limit > period / 2 then
            limit := period / 2;
        end if;
        if phase > limit then
            return limit;
        elsif phase < -limit then
            return -limit;
        end if;
        return phase;
    end function;

    function get_switch_times(period : dhb_count; phase : integer) return dhb_switch_times is
        variable half  : natural := period / 2;
        variable p     : integer := limit_phase(phase, period, period / 2);
        variable times : dhb_switch_times;
    begin
        times.period      := period;
        times.primary_on  := 0;
        times.primary_off := half;
        if p >= 0 then
            times.secondary_on := p;
        else
            times.secondary_on := period + p;
        end if;
        if times.secondary_on + half < period then
            times.secondary_off := times.secondary_on + half;
        else
            times.secondary_off := times.secondary_on + half - period;
        end if;
        return times;
    end function;

    function get_bridge_state(counter : dhb_count; times : dhb_switch_times) return dhb_bridge_state is
        -- high from on up to off, the interval wrapping at the period when off < on
        function is_high(on_count, off_count : dhb_count) return std_logic is
        begin
            if on_count <= off_count then
                if counter >= on_count and counter < off_count then
                    return '1';
                end if;
            elsif counter >= on_count or counter < off_count then
                return '1';
            end if;
            return '0';
        end function;
    begin
        return is_high(times.primary_on, times.primary_off) & is_high(times.secondary_on, times.secondary_off);
    end function;

    function get_segments(period : dhb_count; phase : integer) return dhb_segment_array is
        variable half : natural := period / 2;
        variable p    : integer := limit_phase(phase, period, period / 2);
        variable a    : natural := abs(p);
    begin
        if p >= 0 then
            return (("10", a), ("11", half - a), ("01", a), ("00", period - half - a));
        end if;
        return (("11", half - a), ("10", a), ("00", period - half - a), ("01", a));
    end function;

end package body dhb_modulator_pkg;
