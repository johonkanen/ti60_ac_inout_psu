library ieee;
    use ieee.std_logic_1164.all;

    use work.dhb_modulator_pkg.all;

-- The DHB carrier: a counter of 0 .. period-1 that maps the switch times onto it, the bridge
-- state being dhb_modulator_pkg's get_bridge_state of the count. New switch times (and with
-- them a new period) are taken only at the start of a period, so a change never cuts a pulse
-- short; the next ones are requested two clocks before the period ends, in time for a
-- registered modulator. The period is at least 4 counts.
--
-- A phase step of d counts moves one secondary edge by d, so one secondary half cycle is d
-- longer or shorter than the others: the transformer sees a volt-second imbalance of d counts
-- once. Change the phase in small steps (a rate limited control loop) to keep it small.
--
--   in   enable               - false: the counter held at 0, the bridge state "00", and the
--                               modulator asked every clock, so a rising enable starts a period
--                               with switch times already computed
--   in   switch_times         - from the modulator
--   out  modulation_requested - strobe two clocks before a period ends
--   out  period_start         - strobe the clock a period starts (count 0), e.g. for sampling
--   out  bridge_state         - '1' a leg's high side, to dhb_deadtime
--   out  counter              - the carrier count
entity dhb_carrier is
    port(
        clock                : in std_logic;
        enable               : in boolean;
        switch_times         : in dhb_switch_times;
        modulation_requested : out boolean;
        period_start         : out boolean;
        bridge_state         : out dhb_bridge_state;
        counter              : out dhb_count
    );
end entity dhb_carrier;

architecture rtl of dhb_carrier is

    signal count   : dhb_count := 0;
    signal times   : dhb_switch_times := dhb_switch_times_init;
    signal running : boolean := false;

begin

    counter <= count;

    process(clock)
        variable next_count : dhb_count;
        variable next_times : dhb_switch_times;
    begin
        if rising_edge(clock) then
            modulation_requested <= false;
            period_start         <= false;

            if not enable then
                modulation_requested <= true;
                running      <= false;
                count        <= 0;
                bridge_state <= "00";
            else
                -- the count and the times of this clock's state
                if not running or count >= times.period - 1 then
                    next_count := 0;
                    next_times := switch_times;
                    times      <= switch_times;
                    period_start <= true;
                else
                    next_count := count + 1;
                    next_times := times;
                end if;
                running      <= true;
                count        <= next_count;
                bridge_state <= get_bridge_state(next_count, next_times);
                -- the next times two clocks before the end; every clock while the ones held
                -- are not a period yet (enabled before the modulator had computed any)
                if next_count = next_times.period - 3 or next_times.period < 4 then
                    modulation_requested <= true;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
