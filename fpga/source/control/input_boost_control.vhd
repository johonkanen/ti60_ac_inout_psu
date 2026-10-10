library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.pi_controller_pkg.all;
    use work.lut_divider_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.output_bridge_pkg.all;
    use work.scaler_channels_pkg.all;
    use work.scope_pkg.all;

-- The input bridge as a boost converter from a DC source on the input (a lab supply, which takes
-- no current back): the primary DC link's voltage held at a reference by a voltage PI, whose
-- output is the reference of the bridge's output current (into the link), held by a current PI,
-- whose output is the duty.
--
-- The bridge's output current is the input's power over the link's voltage,
--   i_bridge = iac_in * (vac_in / vdc_in)    (the ratio by lut_divider; the product iac_in *
--                                            vac_in itself would overflow the fixed point's
--                                            2048 at a few amperes at 400 V)
-- so the voltage loop's plant is the link's capacitance alone, C dv/dt = i_bridge - i_load,
-- whatever the input's voltage is to the link's: the voltage transfer ratio is in the current
-- loop, not in the voltage loop's gain. With an AC input the same holds for the period means,
-- the product the instantaneous power.
--
-- The voltage loop is slow, its crossover some Hz (kp_v 0.05 A/V on 1.3 mF: 6 Hz; the PI's zero
-- at 2 Hz), so the link's 100 Hz ripple that rectifying AC brings (its power pulsing at twice the
-- grid's frequency) hardly reaches the current reference: the loop's gain at 100 Hz about 0.06.
-- The price is a slow answer to load steps: the link dips C-limited, a 0.5 A step some volts.
--
-- Only leg 2's low side switches (the input's positive terminal is on leg 2): while it is on the
-- input inductor's current builds, when it turns off it flows on through leg 2's high side body
-- diode into the link. Leg 2's high side and leg 1 stay off, the current returning through leg
-- 1's low side body diode, so no current can flow back into the source whatever the control does.
-- The current reference is limited to 0 .. i_max (no reverse current asked for either), the duty
-- to 0 .. d_max.
--
-- Once a switching period (output_bridge_pkg's, 40 kHz): near its end the bridge's output current
-- from the input current's period mean (measurement_scaler's scaled_iac_in: the sigma-delta's
-- mean over exactly the last period, its ripple gone), the input's voltage (scaled_vac_in) and the
-- link's (below 5 V the ratio 1, no division by next to nothing); at its start the
-- voltage PI on the link's latest voltage, then the current PI on the bridge's output current; the
-- duty applies from the next period. Each PI is pi_controller (its integrator kept within the output's
-- limits: the anti-windup), reset at a start.
--
-- A run stops at once (all gates off: the link keeps its charge) on stop, the protection's trip,
-- inhibit (the face panel's STOP button), the link above the overvoltage stop, or the watchdog: no
-- kick for its time while running. Stopping needs no ramp: the boost only adds charge.
--
-- register map, offsets from g_base_address; values with 20 fractional bits
--   0  W  bit0 start (when stopped), bit1 stop
--      R  bit0 running; why the last run stopped: bit1 the overvoltage, bit2 a trip, bit3 the
--         watchdog, bit4 inhibit
--   1  RW the link's voltage reference, V (105)
--   2  RW kp_v, A/V (0.05)         3 RW ki_v, A/V a period (0.000015)
--   4  RW kp_i, duty/A (0.003)     5 RW ki_i, duty/A a period (0.0005)
--   6  RW i_max, the bridge's output current's, A (1.0)
--   7  RW d_max (0.3), at most 0.5
--   8  RW the overvoltage stop, V (140)
--   9  W  the watchdog's kick      10 RW its time, ms (1000); 0: none
--   11 R  the current reference, A 12 R  the duty
--   13 R  the input current, A     14 R  the link's voltage, V
--   15 R  periods run
--   16 RW the reference's ramp, V/ms (0.5); 0: none
--   17 R  the bridge's output current, A
--   18 R  the input's voltage, V
--
-- The soft start: at a start the voltage PI's reference is the link's voltage, ramping to
-- v_ref at the ramp's rate, so it asks for C dV/dt and no more while the link charges. Without
-- it the first pulses, with the link at the source's voltage, leave the inductor's current
-- hardly falling (little voltage across it) and the current overshoots its limit. The current PI's
-- proportional part damps that region (each period's on time adding to the current, as in CCM),
-- its integral part holds DCM's.
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
        -- scope probes: the current reference (256 counts an ampere) and the duty (32768 = 1.0);
        -- and lanes: 4 running, 5 the switch on
        i_ref_probe    : out scope_sample;
        duty_probe     : out scope_sample;
        probe          : out scope_sample
    );
end entity input_boost_control;

architecture rtl of input_boost_control is

    constant period      : natural := output_period_clocks;     -- 3125: 40 kHz
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

    function to_probe_scaled(value : fixed_number; shift : natural) return scope_sample is
        variable v : fixed_number := shift_right(value, shift);
    begin
        if v > 32767 then return to_probe(32767); end if;
        if v < -32768 then return to_probe(-32768); end if;
        return to_probe(v(15 downto 0));
    end function;

    signal v_ref_r    : std_logic_vector(31 downto 0) := slv(to_fixed(105.0));
    signal kp_v_r     : std_logic_vector(31 downto 0) := slv(to_fixed(0.05));
    signal ki_v_r     : std_logic_vector(31 downto 0) := slv(to_fixed(0.000015));
    signal kp_i_r     : std_logic_vector(31 downto 0) := slv(to_fixed(0.003));
    signal ki_i_r     : std_logic_vector(31 downto 0) := slv(to_fixed(0.0005));
    signal ramp_r     : std_logic_vector(31 downto 0) := slv(to_fixed(0.5));
    -- the reference the voltage PI follows: from the link's voltage at the start towards v_ref
    signal v_target   : fixed_number := fixed_zero;
    signal i_max_r    : std_logic_vector(31 downto 0) := slv(to_fixed(1.0));
    signal d_max_r    : std_logic_vector(31 downto 0) := slv(to_fixed(0.3));
    signal limit_r    : std_logic_vector(31 downto 0) := slv(to_fixed(140.0));
    signal watchdog_r : std_logic_vector(31 downto 0) := to_slv(1000);

    constant min_dc   : fixed_number := to_fixed(5.0);

    signal v_dc, i_in, v_ac, i_bridge : fixed_number := fixed_zero;
    signal ratio      : fixed_number := fixed_zero;   -- vac_in / vdc_in
    signal no_dc      : boolean := true;
    signal divider_in  : lut_divider_in_record(numerator(fixed_wordlength-1 downto 0), denominator(fixed_wordlength-1 downto 0));
    signal divider_out : lut_divider_out_record(quotient(fixed_wordlength-1 downto 0));
    signal running    : boolean := false;
    signal why        : std_logic_vector(3 downto 0) := (others => '0');   -- 4 inhibit .. 1 overvoltage
    signal count      : natural range 0 to period - 1 := 0;
    signal on_clocks, next_on : natural range 0 to period := 0;
    signal gate_on    : boolean := false;
    signal periods    : unsigned(31 downto 0) := (others => '0');
    signal watchdog_count : natural range 0 to 2**26 := 0;

    signal v_in, i_pi_in : pi_controller_in_record := pi_controller_in_init;
    signal v_out, i_pi_out : pi_controller_out_record;
    signal i_ref, duty : fixed_number := fixed_zero;

begin

    running_out <= running;
    i_ref_probe <= to_probe_scaled(i_ref, fixed_radix - 8);
    duty_probe  <= to_probe_scaled(duty, fixed_radix - 15);

    u_voltage_pi : entity work.pi_controller
    port map(clock => clock, pi_in => v_in, pi_out => v_out);

    u_current_pi : entity work.pi_controller
    port map(clock => clock, pi_in => i_pi_in, pi_out => i_pi_out);

    u_divider : entity work.lut_divider
    generic map(g_quotient_radix => fixed_radix)
    port map(clock => clock, lut_divider_in => divider_in, lut_divider_out => divider_out);

    process(clock)
        variable d_max : fixed_number;
        variable step  : fixed_number;
        variable stop_now : boolean;
        variable product  : signed(2 * fixed_wordlength - 1 downto 0);
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                (31 downto 5 => '0') & why & bit_of(running));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, v_ref_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, kp_v_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, ki_v_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, kp_i_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, ki_i_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 6, i_max_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 7, d_max_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 8, limit_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 10, watchdog_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, slv(i_ref));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 12, slv(duty));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 13, slv(i_in));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 14, slv(v_dc));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 15, std_logic_vector(periods));
            connect_data_to_address(bus_in, bus_out, g_base_address + 16, ramp_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 17, slv(i_bridge));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 18, slv(v_ac));

            -- the measurements as they come
            if scaled_ready and scaled_channel = scaled_vdc_in then
                v_dc <= scaled(scaled_vdc_in);
            end if;
            if scaled_ready and scaled_channel = scaled_iac_in then
                i_in <= scaled(scaled_iac_in);
            end if;
            if scaled_ready and scaled_channel = scaled_vac_in then
                v_ac <= scaled(scaled_vac_in);
            end if;

            -- the bridge's output current, ready well before the period's start (the divider's
            -- latency under 30 clocks): the voltages' ratio, then the input current times it
            init_lut_divider(divider_in);
            if count = period - 64 then
                request_lut_division(divider_in, v_ac, v_dc);
                no_dc <= v_dc < min_dc;
            end if;
            if divider_out.ready_with_1 = '1' then
                if no_dc then
                    ratio <= to_fixed(1.0);
                else
                    ratio <= divider_out.quotient;
                end if;
            end if;
            if count = period - 16 then
                product := i_in * ratio;
                i_bridge <= resize(shift_right(product, fixed_radix), fixed_wordlength);
            end if;

            v_in.request    <= false;
            v_in.reset      <= false;
            i_pi_in.request <= false;
            i_pi_in.reset   <= false;

            d_max := clamp(signed(d_max_r), fixed_zero, to_fixed(0.5));

            -- the switching period: the duty taken at its start
            if count < period - 1 then
                count <= count + 1;
            else
                count <= 0;
            end if;
            if count = period - 1 then
                on_clocks <= next_on;
            end if;
            gate_on <= running and count < on_clocks;

            -- the watchdog: periods since the last kick while running
            if write_is_requested_to_address(bus_in, g_base_address + 9) or not running then
                watchdog_count <= 0;
            elsif count = period - 1 and watchdog_count < 2**26 then
                watchdog_count <= watchdog_count + 1;
            end if;

            if running and count = 0 then
                -- the period's start: the reference a ramp's step on, the voltage PI, then the
                -- current PI on its output
                periods <= periods + 1;
                step := shift_right(signed(ramp_r), 0) / periods_a_ms;    -- V a period
                if signed(ramp_r) <= 0 then
                    v_target <= signed(v_ref_r);
                elsif v_target + step < signed(v_ref_r) then
                    v_target <= v_target + step;
                elsif v_target - step > signed(v_ref_r) then
                    v_target <= v_target - step;
                else
                    v_target <= signed(v_ref_r);
                end if;
                v_in <= (request => true, reset => false, reference => v_target, measurement => v_dc,
                         kp => signed(kp_v_r), ki => signed(ki_v_r),
                         limit_high => clamp(signed(i_max_r), fixed_zero, to_fixed(30.0)), limit_low => fixed_zero);
            end if;
            if v_out.ready then
                i_ref <= v_out.output;
                i_pi_in <= (request => true, reset => false, reference => v_out.output, measurement => i_bridge,
                            kp => signed(kp_i_r), ki => signed(ki_i_r), limit_high => d_max, limit_low => fixed_zero);
            end if;
            if i_pi_out.ready then
                duty    <= i_pi_out.output;
                next_on <= to_integer(shift_right(clamp(i_pi_out.output, fixed_zero, d_max) * to_signed(period, 32),
                                                  fixed_radix)(31 downto 0));
            end if;

            -- stopping and starting
            stop_now := false;
            if running then
                if trip then
                    why <= "0010"; stop_now := true;
                elsif inhibit then
                    why <= "1000"; stop_now := true;
                elsif v_dc > signed(limit_r) then
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
                next_on   <= 0;
                on_clocks <= 0;
            elsif write_is_requested_to_address(bus_in, g_base_address) and get_slv_data(bus_in)(0) = '1'
                  and not running and not trip and not inhibit then
                running   <= true;
                why       <= (others => '0');
                periods   <= (others => '0');
                next_on   <= 0;
                on_clocks <= 0;
                i_ref     <= fixed_zero;
                duty      <= fixed_zero;
                v_target  <= v_dc;      -- the soft start's ramp from where the link is
                v_in.reset    <= true;
                i_pi_in.reset <= true;
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
