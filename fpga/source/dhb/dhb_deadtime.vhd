library ieee;
    use ieee.std_logic_1164.all;

    use work.dhb_modulator_pkg.all;

-- Dead time for the two legs of a DHB. Each leg follows its bridge state bit: when the state
-- changes, the switch that conducts turns off at once and the switch of the new state turns on
-- deadtime clocks after the change (at least one), so the two switches of a leg are never on
-- together. A dead time of most of a half period leaves short pulses (the precharge, see
-- dhb_precharge), one of half a period or more none at all. Disabled, a leg's switches are
-- off at once; enabled again, its switch waits its dead time.
--
--   in   enable       - false: all four gates off
--   in   leg_enable   - per leg, (0) primary, (1) secondary; '0' the leg's gates off
--   in   bridge_state - from dhb_carrier, '1' the high side
--   in   deadtime     - clock counts per leg
--   out  gates        - each leg's high and low side gates, (primary) and (secondary), '1' on
entity dhb_deadtime is
    port(
        clock        : in std_logic;
        enable       : in boolean;
        leg_enable   : in std_logic_vector(0 to 1);
        bridge_state : in dhb_bridge_state;
        deadtime     : in dhb_count_array;
        gates        : out dhb_gates
    );
end entity dhb_deadtime;

architecture rtl of dhb_deadtime is

    type count_array is array (0 to 1) of dhb_count;
    signal high_on, low_on : std_logic_vector(0 to 1) := "00";
    signal state           : std_logic_vector(0 to 1) := "00";   -- the state being timed
    signal since_change    : count_array := (others => 0);       -- clocks since it began

begin

    gates <= (primary   => (high => high_on(primary),   low => low_on(primary)),
              secondary => (high => high_on(secondary), low => low_on(secondary)));

    process(clock)
    begin
        if rising_edge(clock) then
            for leg in 0 to 1 loop
                if not enable or leg_enable(leg) = '0' then
                    high_on(leg)      <= '0';
                    low_on(leg)       <= '0';
                    state(leg)        <= bridge_state(leg);
                    since_change(leg) <= 0;
                elsif bridge_state(leg) /= state(leg) then
                    -- the state changed: the conducting switch off now, the time from here
                    high_on(leg)      <= '0';
                    low_on(leg)       <= '0';
                    state(leg)        <= bridge_state(leg);
                    since_change(leg) <= 0;
                elsif since_change(leg) + 1 >= deadtime(leg) then
                    -- the dead time has passed: the switch of the state on
                    high_on(leg) <= state(leg);
                    low_on(leg)  <= not state(leg);
                else
                    since_change(leg) <= since_change(leg) + 1;
                end if;
            end loop;
        end if;
    end process;

end architecture rtl;
