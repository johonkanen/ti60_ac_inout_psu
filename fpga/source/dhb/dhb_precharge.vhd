library ieee;
    use ieee.std_logic_1164.all;

    use work.dhb_modulator_pkg.all;

-- Precharge of the DHB's secondary DC link. The DHB is a DC link isolator: the secondary DC
-- link follows the primary's, so the primary DC link voltage is the reference.
--
-- Only the primary leg switches; the secondary's gates stay off and its body diodes rectify
-- the transformer's pulses into the secondary DC link. The primary's dead time starts at
-- deadtime_start, most of a half period, so each switch is on for a few clocks and the
-- transformer sees very short voltage pulses. Every ramp_periods carrier periods the dead time
-- comes down by ramp_step, widening the pulses, until the secondary voltage is within margin of
-- the primary's: charged, the dead time held where it got. A dead time down at deadtime_final
-- without the voltage there for timeout_periods periods is a failure: the primary stops too.
--
-- The voltages are in any unit, the same for both (the caller scales the two measurements).
--
--   in   start            - strobe: from idle, a precharge from deadtime_start
--   in   stop             - strobe: back to idle, the bridge off
--   in   period_start     - the carrier's strobe, the ramp's time base
--   in   primary_voltage, secondary_voltage
--   in   settings         - see dhb_precharge_settings
--   out  deadtime         - the primary leg's, to dhb_deadtime
--   out  leg_enable       - "10" while charging and charged, "00" otherwise
--   out  state
entity dhb_precharge is
    port(
        clock             : in std_logic;
        start             : in boolean;
        stop              : in boolean;
        period_start      : in boolean;
        primary_voltage   : in integer;
        secondary_voltage : in integer;
        settings          : in dhb_precharge_settings;
        deadtime          : out dhb_count;
        leg_enable        : out std_logic_vector(0 to 1);
        state             : out dhb_precharge_state
    );
end entity dhb_precharge;

architecture rtl of dhb_precharge is

    signal phase_of_charge : dhb_precharge_state := idle;
    signal dt              : dhb_count := 0;
    signal ramp_count      : dhb_count := 0;     -- periods since the last ramp step
    signal stalled         : natural range 0 to 2**24-1 := 0;   -- periods at deadtime_final

begin

    deadtime   <= dt;
    state      <= phase_of_charge;
    leg_enable <= "10" when phase_of_charge = charging or phase_of_charge = charged else "00";

    process(clock)
    begin
        if rising_edge(clock) then
            if stop then
                phase_of_charge <= idle;
            else
                case phase_of_charge is
                    when idle | failed =>
                        if start then
                            phase_of_charge <= charging;
                            dt              <= settings.deadtime_start;
                            ramp_count      <= 0;
                            stalled         <= 0;
                        end if;

                    when charging =>
                        if period_start then
                            if secondary_voltage >= primary_voltage - settings.margin then
                                phase_of_charge <= charged;
                            elsif dt <= settings.deadtime_final then
                                -- the pulses are at full width and the voltage is not there
                                if stalled >= settings.timeout_periods then
                                    phase_of_charge <= failed;
                                else
                                    stalled <= stalled + 1;
                                end if;
                            elsif ramp_count + 1 >= settings.ramp_periods then
                                ramp_count <= 0;
                                if dt - settings.deadtime_final > settings.ramp_step then
                                    dt <= dt - settings.ramp_step;
                                else
                                    dt <= settings.deadtime_final;
                                end if;
                            else
                                ramp_count <= ramp_count + 1;
                            end if;
                        end if;

                    when charged =>
                        null;   -- held until stop, or the converter's run mode takes over
                end case;
            end if;
        end if;
    end process;

end architecture rtl;
