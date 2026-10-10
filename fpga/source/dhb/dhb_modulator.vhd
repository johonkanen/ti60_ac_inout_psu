library ieee;
    use ieee.std_logic_1164.all;

    use work.dhb_modulator_pkg.all;

-- The DHB modulator: on each request (the carrier's, shortly before a period ends) the switch
-- times of the next period from the switching period and the phase shift, the phase clamped to
-- +-max_phase. The computation is dhb_modulator_pkg's get_switch_times, which an ODE
-- simulation calls directly.
--
--   in   modulation_requested - strobe: compute the next period's switch times
--   in   period               - carrier period in clock counts
--   in   phase                - signed phase shift in clock counts, > 0 primary leads
--   in   max_phase            - |phase| limit in clock counts
--   out  modulation_ready     - pulses the clock switch_times is new
--   out  switch_times         - for the carrier
entity dhb_modulator is
    port(
        clock                : in std_logic;
        modulation_requested : in boolean;
        period               : in dhb_count;
        phase                : in dhb_phase;
        max_phase            : in dhb_count;
        modulation_ready     : out boolean;
        switch_times         : out dhb_switch_times
    );
end entity dhb_modulator;

architecture rtl of dhb_modulator is
begin

    process(clock)
    begin
        if rising_edge(clock) then
            modulation_ready <= false;
            if modulation_requested then
                switch_times     <= get_switch_times(period, limit_phase(phase, period, max_phase));
                modulation_ready <= true;
            end if;
        end if;
    end process;

end architecture rtl;
