library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.output_bridge_pkg.all;

-- An open loop test of the output bridge (output_inu, a full bridge): complementary modulation
-- (full_bridge) at 40 kHz from a duty, which steps once, to record the current's and
-- voltage's response to the step. A start arms it, and it starts once the DHB runs (at the sample
-- timer's next period: the carrier against the sigma-delta's bits the same every run) (allowed: the
-- secondary DC link held at the primary's), so it can be armed with the DHB's precharge started
-- after it. It stops at once when the DHB stops, on a trip, or on stop (all also disarm it);
-- else after its set number of periods. The new duty is set at the start of the period the step
-- counts to and the modulator takes it at the next period start: stepped rises a period
-- (45 us) before the bridge's output changes.
--
-- In voltage mode (register 7) the step is in the output voltage instead: voltage_modulator
-- turns the reference into the duty, (1 + Vref / Vdc) / 2 with dc_voltage (the DHB controller's
-- filtered secondary DC link), computed at each period start for the next period. The first
-- period runs at duty 0.5.
--
-- Clocked at 125 MHz: output_bridge_pkg's period, 3125 counts, 25 us (40 kHz): exactly 500 bits of
-- the sigma-delta modulators' 20 MHz, the window of their period means (sdm_adc). 100 ns of dead
-- time is 13 (104 ns).
--
-- register map, offsets from g_base_address; duties fixed point with 20 fractional bits
--   0  W  bit0 arm: start when allowed, bit1 stop
--      R  bit0 running, bit1 stepped, bit2 armed, bit3 the last run stopped by the watchdog
--   1  RW duty before the step, 0 .. 1 (0.5: 0 V)
--   2  RW duty after the step (0.5)
--   3  RW carrier periods before the step (2000: 50 ms)
--   4  RW carrier periods in all (4000: 100 ms); 0: until stopped (a closed loop waveform also
--         until it finishes)
--   5  RW dead time, clock counts (13)
--   6  R  carrier periods run
--   7  RW bit0 voltage mode: registers 8 and 9 instead of 1 and 2
--         bit1 closed loop: the output voltage controlled (output_voltage_control) to a square
--         wave of +-register 10 V, register 11 periods each half; its bridge references to
--         the voltage modulator (as voltage mode's); registers 1..3, 8, 9 unused
--         bit2 with bit1: the reference the waveform player's (waveform_player) instead of the
--         square wave: its points from the first, a step each period; the run ends when
--         it finishes, or after its periods, whichever is first
--   8  RW output voltage before the step, V (0.0)
--   9  RW output voltage after the step, V (0.0)
--   10 RW closed loop: the square wave's amplitude, V (20.0)
--   11 RW closed loop: its half cycle in periods, 1 .. 40000 (200: 10 ms a cycle)
--   12 W  the watchdog's kick: any write
--   13 RW the watchdog's time, ms (1000); 0: none
-- The watchdog: armed or running, a run is stopped as on stop (a closed loop brought down
-- first) when no kick came for the watchdog's time; the software kicks it while someone is
-- watching, so a run does not go on with nobody there (a page closed, the network or the
-- software gone).
-- In closed loop the probe's lane 1 is the square wave high. A closed loop run ends with the
-- output brought down: its periods run, or on stop, the reference goes to 0 V for
-- ramp_down_periods (5 ms: the control's prefilter takes it there smoothly), then the bridge
-- stops; nothing else discharges the output filter. The DHB stopping or a trip still stop it at
-- once.
entity output_bridge_test is
    generic(
        g_base_address : natural := 1140
    );
    port(
        clock   : in std_logic;
        bus_in  : in fpga_interconnect_record;
        bus_out : out fpga_interconnect_record;
        allowed : in boolean;    -- the DHB running
        trip    : in boolean;    -- the protection's: everything off
        dc_voltage : in fixed_number := fixed_zero;  -- the bridge's DC link, voltage mode's
        leg1    : out dhb_leg_gates;
        leg2    : out dhb_leg_gates;
        -- twice a period: at its start and middle, to sample the measurements; the modulator's
        -- while running, a timer's of the same period while stopped (the observer runs on)
        sample  : out boolean;
        -- switching, and the voltage reference of the duty it applies now (voltage mode: the
        -- one of the period before; 0 in duty mode or stopped), for the observer
        running_out       : out boolean;
        applied_reference : out fixed_number;
        -- closed loop: the control running, the voltage reference, the periods' starts; the
        -- control's bridge reference, ready a clock
        closed_loop_out   : out boolean;
        voltage_reference : out fixed_number;
        period_start_out  : out boolean;
        control_u         : in fixed_number := fixed_zero;
        control_ready     : in boolean := false;
        -- closed loop on the waveform: the player's reference and end; its start (at the run's)
        -- and a step at each period start (after the reference is taken), pulses
        waveform_ref      : in fixed_number := fixed_zero;
        waveform_finished : in boolean := false;
        waveform_start    : out boolean;
        waveform_step     : out boolean;
        -- armed or running (its ramp down too): the DHB keeps running while it is
        active_out        : out boolean;
        -- a scope probe: lane 0 running, 1 stepped, 2 and 3 leg 1's and leg 2's high sides
        probe   : out scope_sample;
        -- a clock's pulse: stop, as the register's stop (the face panel's stop button)
        stop_in : in boolean := false
    );
end entity output_bridge_test;

architecture rtl of output_bridge_test is

    constant period      : natural := output_period_clocks;   -- 40 kHz at 125 MHz, sdm_adc's window
    constant max_periods : natural := 40000;   -- 1 s: the step's time, the square wave's half
    constant max_total   : natural := 2**30;   -- the run's periods: 7.5 hours
    constant periods_a_ms : natural := 125000 / period;   -- 40: 125 MHz clocks

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    signal before_r   : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(0.5));
    signal after_r    : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(0.5));
    signal step_r     : std_logic_vector(31 downto 0) := to_slv(2000);
    signal total_r    : std_logic_vector(31 downto 0) := to_slv(4000);
    signal deadtime_r : std_logic_vector(31 downto 0) := to_slv(13);
    signal mode_r     : std_logic_vector(31 downto 0) := (others => '0');
    signal vbefore_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal vafter_r   : std_logic_vector(31 downto 0) := (others => '0');

    signal voltage_mode : boolean := false;
    signal waveform     : boolean := false;
    signal reference    : fixed_number := fixed_zero;
    signal vm_request   : boolean := false;
    signal vm_duty      : fixed_number;
    signal vm_limited   : boolean;
    signal vm_ready     : boolean;
    signal applied_r    : fixed_number := fixed_zero;
    signal modulator_sample, timer_sample : boolean := false;
    signal timer_count  : natural range 0 to period - 1 := 0;
    signal amplitude_r  : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(20.0));
    signal half_r       : std_logic_vector(31 downto 0) := to_slv(200);
    signal watchdog_r   : std_logic_vector(31 downto 0) := to_slv(1000);
    -- the periods since the last kick, armed or running
    signal watchdog_count   : natural range 0 to 2**26 := 0;
    signal watchdog_expired : boolean := false;
    signal watchdog_stopped : boolean := false;
    signal closed_loop  : boolean := false;
    signal square_count : natural range 0 to max_periods := 0;
    signal half_cycle   : natural range 1 to max_periods := 200;
    signal v_ref_r      : fixed_number := fixed_zero;
    -- closed loop: the output brought to 0 V before the bridge stops
    constant ramp_down_periods : natural := 200;
    signal ramping_down : boolean := false;
    signal ramp_count   : natural range 0 to ramp_down_periods := 0;

    signal running      : boolean := false;
    signal armed        : boolean := false;
    signal stepped      : boolean := false;
    signal enable       : boolean;
    signal duty         : fixed_number := to_fixed(0.5);
    signal periods_run  : natural range 0 to max_total := 0;
    signal step_at      : natural range 0 to max_periods := 0;
    signal total        : natural range 0 to max_total := 0;    -- 0: until stopped
    signal period_start : boolean;
    signal leg1_r, leg2_r : dhb_leg_gates;
    signal deadtime       : dhb_count;

    function limit(v : std_logic_vector; low, high : natural) return natural is
    begin
        if unsigned(v) > high then
            return high;
        elsif unsigned(v) < low then
            return low;
        end if;
        return to_integer(unsigned(v));
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

begin

    -- off at once without the DHB or on a trip, whatever the test's state
    enable <= running and allowed and not trip;
    leg1   <= leg1_r;
    leg2   <= leg2_r;

    u_bridge : entity work.full_bridge
    port map(clock, enable, period, duty, deadtime, leg1_r, leg2_r, period_start, modulator_sample);

    -- the samples: the modulator's while it runs, else a timer's
    sample <= modulator_sample when enable else timer_sample;

    sample_timer : process(clock)
    begin
        if rising_edge(clock) then
            timer_sample <= false;
            if timer_count < period - 1 then
                timer_count <= timer_count + 1;
            else
                timer_count <= 0;
            end if;
            if timer_count = 0 or timer_count = period / 2 then
                timer_sample <= true;
            end if;
        end if;
    end process;

    deadtime <= limit(deadtime_r, 1, 255);
    running_out       <= enable;
    active_out        <= armed or running;
    applied_reference <= applied_r;
    closed_loop_out   <= enable and closed_loop;
    voltage_reference <= v_ref_r;
    period_start_out  <= period_start;

    u_voltage_modulator : entity work.voltage_modulator
    port map(clock, vm_request, reference, dc_voltage, vm_duty, vm_limited, vm_ready);

    process(clock)
        variable next_reference : fixed_number;
        variable stop : boolean;
    begin
        if rising_edge(clock) then
            vm_request <= false;
            init_bus(bus_out);
            waveform_start <= false;
            waveform_step  <= false;
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                (31 downto 4 => '0') & bit_of(watchdog_stopped) & bit_of(armed) & bit_of(stepped) &
                bit_of(running));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, before_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, after_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, step_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, total_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, deadtime_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, to_slv(periods_run));
            connect_data_to_address(bus_in, bus_out, g_base_address + 7, mode_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 8, vbefore_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 9, vafter_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 10, amplitude_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 11, half_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 13, watchdog_r);

            -- the watchdog, counting the timer's periods since the last kick while armed or
            -- running
            if write_is_requested_to_address(bus_in, g_base_address + 12) or not (armed or running) then
                watchdog_count <= 0;
            elsif timer_count = period - 1 and watchdog_count < 2**26 then
                watchdog_count <= watchdog_count + 1;
            end if;
            watchdog_expired <= unsigned(watchdog_r) /= 0 and (armed or running)
                and watchdog_count >= to_integer(unsigned(watchdog_r(15 downto 0))) * periods_a_ms;

            -- (the watchdog's once: not again while the output is brought down)
            stop := (write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(1) = '1')
                    or (watchdog_expired and not ramping_down) or stop_in;
            if watchdog_expired then
                watchdog_stopped <= true;
            end if;
            if trip or (running and not allowed) or (stop and not (running and closed_loop)) then
                running <= false;
                armed   <= false;
                ramping_down <= false;
            elsif stop then
                -- stop in closed loop: the output brought down first
                armed        <= false;
                ramping_down <= true;
                v_ref_r      <= fixed_zero;
                stepped      <= false;
            elsif write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(0) = '1'
                  and not running then
                armed <= true;
                watchdog_stopped <= false;
            elsif armed and allowed and timer_count = period - 1 then
                -- the start on the sample timer's period: the carrier always in the same place
                -- against the sigma-delta modulators' bits (the timer and their clock run from
                -- the same PLL since power up), so the switching edges' effect on their readings
                -- is the same every run
                armed       <= false;
                running     <= true;
                stepped     <= false;
                voltage_mode <= mode_r(0) = '1' or mode_r(1) = '1';
                closed_loop  <= mode_r(1) = '1';
                waveform     <= mode_r(1) = '1' and mode_r(2) = '1';
                if mode_r(1) = '1' and mode_r(2) = '1' then
                    v_ref_r        <= waveform_ref;   -- the first point, read ahead
                    waveform_start <= true;
                else
                    v_ref_r <= -signed(amplitude_r);
                end if;
                square_count <= 0;
                ramping_down <= false;
                ramp_count   <= 0;
                half_cycle   <= limit(half_r, 1, max_periods);
                if mode_r(1) = '1' then
                    -- closed loop: the first period at 0 V, then the control's references
                    duty       <= to_fixed(0.5);
                    reference  <= fixed_zero;
                    vm_request <= true;
                elsif mode_r(0) = '1' then
                    duty       <= to_fixed(0.5);
                    reference  <= signed(vbefore_r);
                    vm_request <= true;
                else
                    duty <= signed(before_r);
                end if;
                step_at     <= limit(step_r, 0, max_periods);
                total       <= limit(total_r, 0, max_total);
                periods_run <= 0;
            elsif running and period_start and closed_loop then
                -- the period starting now runs on the control's reference of the period before;
                -- the square wave's half cycles
                applied_r <= reference;
                if ramping_down then
                    -- the reference at 0 V for ramp_down_periods, then the bridge off
                    v_ref_r <= fixed_zero;
                    if ramp_count + 1 >= ramp_down_periods then
                        running      <= false;
                        ramping_down <= false;
                    else
                        ramp_count <= ramp_count + 1;
                    end if;
                elsif (total /= 0 and periods_run + 1 >= total) or (waveform and waveform_finished) then
                    -- the run's periods done or the waveform's: the output brought down
                    ramping_down <= true;
                    ramp_count   <= 0;
                    v_ref_r      <= fixed_zero;
                    stepped      <= false;
                elsif waveform then
                    -- the player's point for this period, then its step to the next period
                    v_ref_r       <= waveform_ref;
                    waveform_step <= true;
                    stepped       <= waveform_ref > 0;   -- lane 1: the reference positive
                elsif square_count + 1 >= half_cycle then
                    square_count <= 0;
                    v_ref_r <= -v_ref_r;
                    stepped <= v_ref_r < 0;   -- lane 1: the square wave high
                else
                    square_count <= square_count + 1;
                end if;
                -- the run's periods (the ramp down after them)
                if not ramping_down and periods_run < max_total then
                    periods_run <= periods_run + 1;
                end if;
            elsif running and period_start then
                -- the period starting now runs on the duty from the reference before this update
                if voltage_mode then
                    applied_r <= reference;
                end if;
                next_reference := reference;
                if total /= 0 and periods_run + 1 >= total then
                    running <= false;
                end if;
                if periods_run + 1 >= step_at and not stepped then
                    if voltage_mode then
                        next_reference := signed(vafter_r);
                    else
                        duty <= signed(after_r);
                    end if;
                    stepped <= true;
                end if;
                if periods_run < max_total then
                    periods_run <= periods_run + 1;
                end if;
                -- voltage mode: the next period's duty from the reference and the DC link now
                reference <= next_reference;
                if voltage_mode then
                    vm_request <= true;
                end if;
            end if;

            if not enable then
                applied_r <= fixed_zero;
            end if;

            -- closed loop: the control's reference, a microsecond or so into the period
            if running and closed_loop and control_ready then
                reference  <= control_u;
                vm_request <= true;
            end if;

            -- the duty comes 16 clocks after the request, long before the modulator takes it
            if running and voltage_mode and vm_ready then
                duty <= vm_duty;
            end if;

            probe <= x"000" & leg2_r.high & leg1_r.high & bit_of(stepped and running) & bit_of(running);
        end if;
    end process;

end architecture rtl;
