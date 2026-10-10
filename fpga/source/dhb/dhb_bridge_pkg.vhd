library ieee;
    use ieee.std_logic_1164.all;

    use work.dhb_modulator_pkg.all;

-- The ports of dhb_bridge, the dual active half bridge as a DC link isolator: its settings,
-- its sequence and the two port records, with their initial values. The modulation itself is
-- dhb_modulator_pkg's.
package dhb_bridge_pkg is

    -- off -> precharging -> running, or failed (see dhb_bridge)
    type dhb_mode is (off, precharging, running, failed);

    type dhb_bridge_settings is record
        period       : dhb_count;     -- carrier period, clock counts
        max_phase    : dhb_count;     -- |phase| limit, clock counts
        run_deadtime : dhb_count;     -- both legs' dead time when running (100 ns), and the
                                      -- narrowest the precharge ramps to
        precharge    : dhb_precharge_settings;   -- its deadtime_final is run_deadtime
    end record;

    type dhb_input_record is record
        start             : boolean;     -- strobe: precharge, then run, from off or failed
        stop              : boolean;     -- strobe: all gates off at once
        phase             : dhb_phase;   -- clock counts when running, > 0 the primary leads
        primary_voltage   : integer;     -- the DC links, in any unit the same for both
        secondary_voltage : integer;
        settings          : dhb_bridge_settings;
    end record;

    type dhb_output_record is record
        primary_gates   : dhb_leg_gates;    -- to the primary leg's gate drivers
        secondary_gates : dhb_leg_gates;    -- to the secondary leg's
        state           : dhb_mode;
        period_start    : boolean;          -- strobe at each carrier period start
        charged         : boolean;          -- the precharge has charged the secondary DC link;
                                            -- running follows at the next period start
    end record;

    -- 150 kHz at 200 MHz (1333 counts), 100 ns (20 counts) of dead time, the phase up to a
    -- quarter period; the precharge from 10 clock pulses, 5 clocks wider a period, charged
    -- within 50 units (5 V in 0.1 V), failing 20 periods after reaching full width
    constant dhb_bridge_settings_init : dhb_bridge_settings := (
        period       => 1333,
        max_phase    => 333,
        run_deadtime => 20,
        precharge    => (deadtime_start => 1333 / 2 - 10, deadtime_final => 20, ramp_step => 5,
                         ramp_periods => 1, margin => 50, timeout_periods => 20));

    constant dhb_input_init : dhb_input_record := (
        start             => false,
        stop              => false,
        phase             => 0,
        primary_voltage   => 0,
        secondary_voltage => 0,
        settings          => dhb_bridge_settings_init);

    constant dhb_output_init : dhb_output_record := (
        primary_gates   => dhb_leg_gates_off,
        secondary_gates => dhb_leg_gates_off,
        state           => off,
        period_start    => false,
        charged         => false);

end package dhb_bridge_pkg;
