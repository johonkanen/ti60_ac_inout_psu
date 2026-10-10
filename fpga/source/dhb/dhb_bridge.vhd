library ieee;
    use ieee.std_logic_1164.all;

    use work.dhb_modulator_pkg.all;
    use work.dhb_bridge_pkg.all;

-- The dual active half bridge as a DC link isolator: the modulator, carrier, dead time and
-- precharge joined, and the sequence
--
--   off         - all four gates off
--   precharging - only the primary switches, its pulses widening until the secondary DC link
--                 is at the primary's voltage (dhb_precharge)
--   running     - from the period start after the precharge: both legs at 50 %, the secondary
--                 phase shifted by phase, both with run_deadtime (100 ns)
--   failed      - the precharge did not get there; the gates off until stop or a new start
--
-- stop turns everything off at once, from any state. start begins a precharge from off or
-- failed. The ports are dhb_bridge_pkg's records; clock stays a port of its own.
--
--   in   dhb_in.start, dhb_in.stop - strobes
--   in   dhb_in.phase              - signed phase shift in clock counts when running, > 0 the
--                                    primary leads
--   in   dhb_in.primary_voltage, dhb_in.secondary_voltage - the DC links, any unit the same
--                                    for both
--   in   dhb_in.settings           - see dhb_bridge_settings
--   out  dhb_out.primary_gates     - the primary leg's high and low side gates
--   out  dhb_out.secondary_gates   - the secondary leg's
--   out  dhb_out.state
--   out  dhb_out.period_start      - strobe at each carrier period start, e.g. to sample the
--                                    control
--   out  dhb_out.charged           - the precharge got there; running from the next period
--                                    start
entity dhb_bridge is
    port(
        clock   : in std_logic;
        dhb_in  : in dhb_input_record;
        dhb_out : out dhb_output_record := dhb_output_init
    );
end entity dhb_bridge;

architecture rtl of dhb_bridge is

    alias start             is dhb_in.start;
    alias stop              is dhb_in.stop;
    alias phase             is dhb_in.phase;
    alias primary_voltage   is dhb_in.primary_voltage;
    alias secondary_voltage is dhb_in.secondary_voltage;
    alias settings          is dhb_in.settings;

    signal mode                 : dhb_mode := off;
    signal enable               : boolean;
    signal modulation_requested : boolean;
    signal modulation_ready     : boolean;
    signal switch_times         : dhb_switch_times;
    signal carrier_period_start : boolean;
    signal bridge_state         : dhb_bridge_state;
    signal counter              : dhb_count;
    signal phase_now            : dhb_phase;
    signal deadtime             : dhb_count_array;
    signal leg_enable           : std_logic_vector(0 to 1);
    signal gates                : dhb_gates;

    signal precharge_settings   : dhb_precharge_settings;
    signal precharge_start      : boolean;
    signal precharge_stop       : boolean;
    signal precharge_deadtime   : dhb_count;
    signal precharge_leg_enable : std_logic_vector(0 to 1);
    signal precharge_state      : dhb_precharge_state;

begin

    dhb_out.state        <= mode;
    dhb_out.period_start <= carrier_period_start;
    dhb_out.charged      <= precharge_state = charged;
    enable       <= mode = precharging or mode = running;

    -- the primary alone at phase 0 while precharging (the secondary is off then anyway)
    phase_now  <= phase when mode = running else 0;
    deadtime   <= (settings.run_deadtime, settings.run_deadtime) when mode = running else
                  (precharge_deadtime, settings.run_deadtime);
    leg_enable <= "11" when mode = running else
                  precharge_leg_enable when mode = precharging else "00";

    -- the precharge ramps down to the dead time of running
    precharge_settings <= (settings.precharge.deadtime_start, settings.run_deadtime,
                           settings.precharge.ramp_step, settings.precharge.ramp_periods,
                           settings.precharge.margin, settings.precharge.timeout_periods);
    precharge_start <= start and (mode = off or mode = failed);
    precharge_stop  <= stop or mode = running;   -- running, the precharge idles

    sequencer : process(clock)
    begin
        if rising_edge(clock) then
            if stop then
                mode <= off;
            else
                case mode is
                    when off | failed =>
                        if start then
                            mode <= precharging;
                        end if;
                    when precharging =>
                        if precharge_state = failed then
                            mode <= failed;
                        elsif precharge_state = charged and carrier_period_start then
                            mode <= running;
                        end if;
                    when running =>
                        null;
                end case;
            end if;
        end if;
    end process;

    u_modulator : entity work.dhb_modulator
    port map(clock, modulation_requested, settings.period, phase_now, settings.max_phase,
             modulation_ready, switch_times);

    u_carrier : entity work.dhb_carrier
    port map(clock, enable, switch_times, modulation_requested, carrier_period_start,
             bridge_state, counter);

    u_deadtime : entity work.dhb_deadtime
    port map(clock, enable, leg_enable, bridge_state, deadtime, gates);

    dhb_out.primary_gates   <= gates(primary);
    dhb_out.secondary_gates <= gates(secondary);

    u_precharge : entity work.dhb_precharge
    port map(clock, precharge_start, precharge_stop, carrier_period_start, primary_voltage,
             secondary_voltage, precharge_settings, precharge_deadtime, precharge_leg_enable,
             precharge_state);

end architecture rtl;
