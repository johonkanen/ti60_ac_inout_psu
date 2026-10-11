library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.output_bridge_pkg.all;
    use work.output_control_program_pkg.boost_max_readings;
    use work.scaler_channels_pkg.all;
    use work.scope_pkg.all;

-- The input bridge as a boost converter from a DC source on the input (a lab supply, which takes
-- no current back): the primary DC link's voltage held at a reference by a voltage PI straight to
-- the duty. The input current is not controlled: its measurement is too noisy for a current loop.
--
-- The PI runs on output_control_processor's core (output_control_program_pkg's boost programs):
-- this entity keeps the registers, the switching period, the gate and every stop, and hands the
-- core a step a period with the period's readings of the link (their mean is the control's: the
-- noise of single readings averaged), the reference, the reference's ramp, d_max and the gains.
--
-- The plant: in DCM (the inductor's current back to zero each period, as at these loads with an
-- input inductance of some 15 uH) a period's charge into the link is
--   q = (v_in t_on)^2 / (2 L (v_dc - v_in)),
-- so the link's current goes with the duty's square: its gain to the duty, 2 q / d a period,
-- grows with the load (at 94.6 V into 105 V with 15 uH: 12 A a unit of duty at 0.05 A, 38 A at
-- 0.5 A). The defaults (kp 0.005 duty/V, ki 1.5e-6 duty/V a period: the PI's zero at 2 Hz) put
-- the crossover at 7..23 Hz on 1.3 mF, well under the input filter's ~1 kHz resonance (15 uH with
-- 1.3 mF) and slow against the 100 Hz ripple an AC output's power brings. The soft start's ramp
-- keeps the link's current C dV/dt while it charges.
--
-- Only leg 2's low side switches (the input's positive terminal is on leg 2): while it is on the
-- input inductor's current builds, when it turns off it flows on through leg 2's high side body
-- diode into the link. Leg 2's high side and leg 1 stay off, the current returning through leg
-- 1's low side body diode, so no current can flow back into the source whatever the control does.
-- The duty is limited to 0 .. d_max, d_max itself to at most 0.2 (d_max_ceiling) whatever is
-- written: at most a fifth of the period on, so a runaway cannot pump the link up fast.
--
-- Once a switching period (output_bridge_pkg's, 40 kHz), at its start: the readings of the link
-- since the period before handed to the core with a step's request (a reset's at a run's first
-- period); the on time it returns applies from the next period.
--
-- A run stops at once (all gates off: the link keeps its charge) on stop, the protection's trip,
-- inhibit (the face panel's STOP button), the link above the overvoltage stop for 8 of its readings
-- in a row (single readings carry the switching's noise, +-30 V: one alone must neither stop it
-- nor be needed to), or the watchdog: no kick for its time while running. Stopping needs no ramp: the boost only adds
-- charge.
--
-- register map, offsets from g_base_address; values with 20 fractional bits, the gains with 24
--   0  W  bit0 start (when stopped), bit1 stop
--      R  bit0 running; why the last run stopped: bit1 the overvoltage, bit2 a trip, bit3 the
--         watchdog, bit4 inhibit
--   1  RW the link's voltage reference, V (105)
--   2  RW kp, duty/V (0.005)        3 RW ki, duty/V a period (0.0000015)
--   7  RW d_max (0.05), at most 0.2 (above it 0.2 is used)
--   8  RW the overvoltage stop, V (140)
--   9  W  the watchdog's kick      10 RW its time, ms (1000); 0: none
--   11 R  the reference as ramped, V
--   12 R  the duty
--   13 R  the input current, A (its latest period mean, not used by the control)
--   14 R  the link's latest reading, V
--   15 R  periods run
--   16 RW the reference's ramp, V/ms (0.5); 0: none
--   17 R  the link's period mean, V (the control's)
--   18 R  the input's voltage, V
entity input_boost_control is
    generic(
        g_base_address : natural := 1260
    );
    port(
        clock          : in std_logic;
        bus_in         : in fpga_interconnect_record;
        bus_out        : out fpga_interconnect_record;
        -- from measurement_scaler
        scaled         : in fixed_array;
        scaled_ready   : in boolean;
        scaled_channel : in natural;
        trip           : in boolean;
        inhibit        : in boolean;      -- the face panel's STOP: no boost meanwhile
        running_out    : out boolean;
        leg1           : out dhb_leg_gates;
        leg2           : out dhb_leg_gates;
        -- to output_control_processor's boost inputs, held until the next request
        request        : out boolean;
        reset          : out boolean;
        v_sum          : out fixed_number;
        v_count        : out natural range 0 to boost_max_readings;
        v_ref          : out fixed_number;
        ramp           : out fixed_number;
        d_half         : out fixed_number;
        kp             : out signed(31 downto 0);
        ki             : out signed(31 downto 0);
        -- from its boost outputs
        ready          : in boolean;
        duty_in        : in fixed_number;
        on_half        : in fixed_number;
        target_in      : in fixed_number;
        v_mean_in      : in fixed_number;
        -- scope lanes: 4 running, 5 the switch on
        probe          : out scope_sample
    );
end entity input_boost_control;

architecture rtl of input_boost_control is

    constant period      : natural := output_period_clocks;     -- 3125: 40 kHz
    constant d_max_ceiling : fixed_number := to_fixed(0.2);
    constant over_in_a_row : natural := 8;
    constant periods_a_ms : natural := 125000 / period;         -- 40

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    function slv(f : fixed_number) return std_logic_vector is
    begin
        return std_logic_vector(f);
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

    function clamp(value, low, high : fixed_number) return fixed_number is
    begin
        if value > high then return high; end if;
        if value < low then return low; end if;
        return value;
    end function;

    signal v_ref_r    : std_logic_vector(31 downto 0) := slv(to_fixed(105.0));
    signal kp_r       : std_logic_vector(31 downto 0) := std_logic_vector(to_signed(integer(0.005 * 2.0**24), 32));
    signal ki_r       : std_logic_vector(31 downto 0) := std_logic_vector(to_signed(integer(0.0000015 * 2.0**24), 32));
    signal ramp_r     : std_logic_vector(31 downto 0) := slv(to_fixed(0.5));
    signal d_max_r    : std_logic_vector(31 downto 0) := slv(to_fixed(0.05));
    signal limit_r    : std_logic_vector(31 downto 0) := slv(to_fixed(140.0));
    signal watchdog_r : std_logic_vector(31 downto 0) := to_slv(1000);

    signal v_dc, i_in, v_ac : fixed_number := fixed_zero;
    -- the link's readings this period, and the period's before as handed to the core
    signal reading_sum   : fixed_number := fixed_zero;
    signal reading_count : natural range 0 to boost_max_readings := 0;
    signal period_sum    : fixed_number := fixed_zero;
    signal period_count  : natural range 0 to boost_max_readings := 0;

    signal running    : boolean := false;
    signal first      : boolean := false;     -- a run's first period: the core's reset
    signal why        : std_logic_vector(3 downto 0) := (others => '0');   -- 4 inhibit .. 1 overvoltage
    signal count      : natural range 0 to period - 1 := 0;
    signal on_clocks, next_on : natural range 0 to period := 0;
    signal gate_on    : boolean := false;
    signal periods    : unsigned(31 downto 0) := (others => '0');
    signal watchdog_count : natural range 0 to 2**26 := 0;
    signal duty, target, v_mean : fixed_number := fixed_zero;
    signal over_count : natural range 0 to over_in_a_row := 0;    -- readings above the stop in a row

begin

    running_out <= running;
    v_sum   <= period_sum;
    v_count <= period_count;
    v_ref   <= signed(v_ref_r);
    -- no ramp: the reference at once (a step of 50 V a period)
    ramp    <= to_fixed(2000.0) when signed(ramp_r) <= 0 else signed(ramp_r);
    d_half  <= shift_right(clamp(signed(d_max_r), fixed_zero, d_max_ceiling), 1);
    kp      <= signed(kp_r);
    ki      <= signed(ki_r);

    process(clock)
        variable stop_now : boolean;
        variable on_time  : fixed_number;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                (31 downto 5 => '0') & why & bit_of(running));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, v_ref_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, kp_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, ki_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 7, d_max_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 8, limit_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 10, watchdog_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, slv(target));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 12, slv(duty));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 13, slv(i_in));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 14, slv(v_dc));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 15, std_logic_vector(periods));
            connect_data_to_address(bus_in, bus_out, g_base_address + 16, ramp_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 17, slv(v_mean));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 18, slv(v_ac));

            -- the measurements as they come, the link's summed for the period
            if scaled_ready and scaled_channel = scaled_vdc_in then
                v_dc <= scaled(scaled_vdc_in);
                if scaled(scaled_vdc_in) <= signed(limit_r) then
                    over_count <= 0;
                elsif over_count < over_in_a_row then
                    over_count <= over_count + 1;
                end if;
                if reading_count < boost_max_readings then
                    reading_sum   <= reading_sum + scaled(scaled_vdc_in);
                    reading_count <= reading_count + 1;
                end if;
            end if;
            if scaled_ready and scaled_channel = scaled_iac_in then
                i_in <= scaled(scaled_iac_in);
            end if;
            if scaled_ready and scaled_channel = scaled_vac_in then
                v_ac <= scaled(scaled_vac_in);
            end if;

            -- the switching period: the on time taken at its start
            if count < period - 1 then
                count <= count + 1;
            else
                count <= 0;
            end if;
            if count = period - 1 then
                on_clocks <= next_on;
            end if;
            gate_on <= running and count < on_clocks;

            -- the period's start: its readings to the core, a step (a reset at a run's first)
            request <= false;
            if count = 0 then
                period_sum    <= reading_sum;
                period_count  <= reading_count;
                reading_sum   <= fixed_zero;
                reading_count <= 0;
                if running then
                    request <= true;
                    reset   <= first;
                    first   <= false;
                    periods <= periods + 1;
                end if;
            end if;
            if ready then
                duty   <= duty_in;
                target <= target_in;
                v_mean <= v_mean_in;
                -- the on time from its half clocks, within the period
                on_time := shift_right(on_half, fixed_radix - 1);
                if on_time < 0 then
                    next_on <= 0;
                elsif on_time > period then
                    next_on <= period;
                else
                    next_on <= to_integer(on_time);
                end if;
            end if;

            -- the watchdog: periods since the last kick while running
            if write_is_requested_to_address(bus_in, g_base_address + 9) or not running then
                watchdog_count <= 0;
            elsif count = period - 1 and watchdog_count < 2**26 then
                watchdog_count <= watchdog_count + 1;
            end if;

            -- stopping and starting
            stop_now := false;
            if running then
                if trip then
                    why <= "0010"; stop_now := true;
                elsif inhibit then
                    why <= "1000"; stop_now := true;
                elsif over_count >= over_in_a_row then
                    why <= "0001"; stop_now := true;
                elsif unsigned(watchdog_r) /= 0
                      and watchdog_count >= to_integer(unsigned(watchdog_r(15 downto 0))) * periods_a_ms then
                    why <= "0100"; stop_now := true;
                end if;
            end if;
            if write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(1) = '1' then
                stop_now := true;
            end if;
            if stop_now then
                running   <= false;
                gate_on   <= false;
                on_clocks <= 0;
            elsif write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(0) = '1'
                  and not running and not trip and not inhibit then
                running   <= true;
                first     <= true;
                why       <= (others => '0');
                periods   <= (others => '0');
                on_clocks <= 0;
            end if;
            -- stopped or starting, no on time: a late result from a run before changes nothing
            if not running or stop_now or first then
                next_on <= 0;
            end if;

            -- only leg 2's low side ever switches
            leg1 <= dhb_leg_gates_off;
            leg2 <= dhb_leg_gates_off;
            if gate_on and running and not stop_now and not trip then
                leg2.low <= '1';
            end if;

            probe <= x"00" & "00" & bit_of(gate_on and running) & bit_of(running) & x"0";
        end if;
    end process;

end architecture rtl;
