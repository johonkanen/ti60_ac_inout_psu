library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.dhb_modulator_pkg.all;
    use work.full_bridge_modulator_pkg.all;

-- The output bridge's carrier: a counter of 0 .. period-1 that maps the switch times onto it,
-- the bridge state being full_bridge_modulator_pkg's get_bridge_state of the count. New switch
-- times (and with them a new period) are taken only at the start of a period, so a change never
-- cuts a pulse short; the next ones are requested g_request_clocks before the period ends, in
-- time for the modulator's fixed_dsp. The period is at least 16 counts.
--
--   in   enable               - false: the counter held at 0, the bridge state "00", and the
--                               modulator asked every clock, so a rising enable starts a period
--                               with switch times already computed (with none yet, it waits
--                               for them)
--   in   switch_times         - from the modulator
--   out  modulation_requested - strobe g_request_clocks before a period ends
--   out  period_start         - strobe the clock a period starts (count 0)
--   out  sample               - strobe at each period's start and middle: twice a period, for
--                               the measurements (sdm_adc's period means)
--   out  bridge_state         - "10" leg 1 high, "01" leg 2 high, to dhb_deadtime
--   out  counter              - the carrier count
entity full_bridge_carrier is
    generic(
        g_request_clocks : positive := 8
    );
    port(
        clock                : in std_logic;
        enable               : in boolean;
        switch_times         : in fb_switch_times;
        modulation_requested : out boolean;
        period_start         : out boolean;
        sample               : out boolean;
        bridge_state         : out dhb_bridge_state;
        counter              : out dhb_count
    );
end entity full_bridge_carrier;

architecture rtl of full_bridge_carrier is

    signal count   : dhb_count := 0;
    signal times   : fb_switch_times := fb_switch_times_init;
    signal running : boolean := false;

begin

    counter <= count;

    process(clock)
        variable next_count : dhb_count;
        variable next_times : fb_switch_times;
    begin
        if rising_edge(clock) then
            modulation_requested <= false;
            period_start         <= false;
            sample               <= false;

            if not enable or (not running and switch_times.period < 16) then
                -- off, or enabled before the modulator has given times of a period: wait
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
                    sample       <= true;
                else
                    next_count := count + 1;
                    next_times := times;
                    if next_count = next_times.period / 2 then
                        sample <= true;
                    end if;
                end if;
                running      <= true;
                count        <= next_count;
                bridge_state <= get_bridge_state(next_count, next_times);
                -- the next times g_request_clocks before the end; every clock while the ones
                -- held are not a period yet (enabled before the modulator had computed any)
                if next_count = next_times.period - g_request_clocks or next_times.period < 16 then
                    modulation_requested <= true;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
