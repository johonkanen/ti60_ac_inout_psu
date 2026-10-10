library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.full_bridge_modulator_pkg.all;

-- The output bridge, complementary (bipolar) modulation of a full bridge: the modulator
-- (full_bridge_modulator: the switch times from the duty), the carrier (full_bridge_carrier:
-- the times onto the count) and the dead time (dhb_deadtime) joined, as the DHB's
-- (dhb_bridge). Leg 1's high side on for duty of each period, leg 2 the complement: the
-- bridge's output averages (2 * duty - 1) * Vdc, 0.5 gives 0, 1.0 +Vdc, 0.0 -Vdc. A duty is
-- taken g_request_clocks (8) before a period starts and holds the whole period.
--
--   in   enable    - false: the carrier held at 0, all four gates off
--   in   period    - carrier period in clock counts (at least 16)
--   in   duty      - 0 .. 1 of leg 1's high side (kept within)
--   in   deadtime  - clock counts, both legs
--   out  leg1, leg2  the gates
--   out  period_start - each period's start; sample - its start and middle
entity full_bridge is
    port(
        clock        : in std_logic;
        enable       : in boolean;
        period       : in dhb_count;
        duty         : in fixed_number;
        deadtime     : in dhb_count;
        leg1         : out dhb_leg_gates;
        leg2         : out dhb_leg_gates;
        period_start : out boolean;
        sample       : out boolean
    );
end entity full_bridge;

architecture rtl of full_bridge is

    signal modulation_requested, modulation_ready : boolean;
    signal switch_times : fb_switch_times;
    signal state        : dhb_bridge_state;
    signal counter      : dhb_count;
    signal gates        : dhb_gates;
    signal deadtimes    : dhb_count_array;

begin

    leg1      <= gates(0);
    leg2      <= gates(1);
    deadtimes <= (deadtime, deadtime);

    u_modulator : entity work.full_bridge_modulator
    port map(clock, modulation_requested, period, duty, modulation_ready, switch_times);

    u_carrier : entity work.full_bridge_carrier
    port map(clock, enable, switch_times, modulation_requested, period_start, sample, state, counter);

    u_deadtime : entity work.dhb_deadtime
    port map(clock, enable, "11", state, deadtimes, gates);

end architecture rtl;
