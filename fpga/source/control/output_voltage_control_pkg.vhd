library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;

    use work.output_observer_pkg.all;

-- The output voltage's control, cascaded, as functions in real numbers (for designing and
-- tuning, output_voltage_control_tb; output_control_processor's program follows them, its
-- testbench checks it against them): once a switching period, at its start, from the output
-- observer's estimates.
--
--   0  the reference processed before the voltage PI: a prefilter, two first order low passes
--      in series (critically damped, prefilter_alpha each a period; alpha 1: none), the second's
--      output v_ref' the PI's reference; its slope fed forward as the capacitor's current,
--      C dv_ref'/dt (feedforward = C alpha / period, on the second stage's input less its
--      output): the step's current comes from the reference, not from the PI's error
--   1  the prediction: the bridge applies the reference computed now from the next period on,
--      so the filter's state a period ahead from the observer's model, with the reference it
--      applies now (u_applied), the observer's slow du and iload
--   2  the voltage PI: v_ref' - v -> the capacitor current; with the load current and the
--      feedforward, the inductor current reference, limited to +-i_max (the current limit)
--   3  the current PI: i_ref - i -> the inductor voltage reference, limited to +-vl_max
--   4  the bridge voltage reference: the inductor's voltage, the capacitor's and R i (feed
--      forward), less the bridge's own error du low passed (du_hz: its steady offset, not the
--      observer's model errors in transients); limited to +-u_max (the DC link)
--
-- The voltage PI's proportional part acts on b v_ref' - v, v the predicted voltage (setpoint
-- weight b); its integral on v_ref' less the observer's estimate now: the prediction, with the
-- observer's slow du only, is biased while du moves, and an integral of its error would hold the
-- prediction at the reference, not the voltage.
--
-- Each PI integrates only while its output is within its limits (anti-windup). The bridge's
-- reference goes to the voltage modulator, which applies it the next period.
package output_voltage_control_pkg is

    type voltage_control_design_record is record
        period   : real;    -- s, the switching period (the control's)
        l, r     : real;    -- the filter's inductor (H) and its loss (ohm)
        kp_v     : real;    -- A/V       the voltage's PI
        ki_v     : real;    -- A/(V s)
        kp_i     : real;    -- V/A       the current's PI
        ki_i     : real;    -- V/(A s)
        i_max    : real;    -- A         the inductor current's limit
        vl_max   : real;    -- V         the inductor voltage's limit
        u_max    : real;    -- V         the bridge voltage's limit
        b        : real;    --           the voltage PI's setpoint weight
        du_alpha : real;    --           du's low pass: its share of a period's difference
        prefilter_alpha : real;   --     the prefilter's stages: their share of a period's difference
        feedforward     : real;   -- A/V the filtered reference's slope to the current
    end record;

    -- gains from the loops' bandwidths (Hz): the current PI's zero a fifth of its bandwidth below
    -- it, the voltage PI's voltage_zero of its; the current loop L s with its gain kp_i, the
    -- voltage loop C s with kp_v; the prefilter's stages at prefilter_hz (0: none, nor the
    -- feedforward), feedforward the share of C dv_ref'/dt fed forward (1.0 all of it)
    function voltage_control_design(period, l, c, r, current_hz, voltage_hz, i_max, vl_max, u_max : real;
        b : real := 1.0; du_hz : real := 20.0; voltage_zero : real := 0.2;
        prefilter_hz : real := 0.0; feedforward : real := 1.0) return voltage_control_design_record;

    type voltage_control_record is record
        integral_v, integral_i : real;   -- the PIs' integrals: A, V
        du_filtered            : real;   -- du low passed, V
        reference_1, reference_2 : real; -- the prefilter's stages, V: the second v_ref'
        -- the step's results
        i_feedforward            : real;
        v_predicted, i_predicted : real;
        i_ref, vl_ref, u_ref     : real;
        current_limited, voltage_limited, bridge_limited : boolean;
    end record;

    constant init_voltage_control : voltage_control_record := (
        integral_v => 0.0, integral_i => 0.0, du_filtered => 0.0, reference_1 => 0.0, reference_2 => 0.0,
        i_feedforward => 0.0, v_predicted => 0.0, i_predicted => 0.0,
        i_ref => 0.0, vl_ref => 0.0, u_ref => 0.0,
        current_limited => false, voltage_limited => false, bridge_limited => false);

    -- a step: the observer's estimates now (i, v, the slow du and iload: its PIs' integrals)
    -- and its half period coefficients, the reference applied now
    function voltage_control_step(state : voltage_control_record; d : voltage_control_design_record;
        k : observer_coefficients_record; v_ref, i_est, v_est, du_slow, iload_slow, u_applied : real)
        return voltage_control_record;

    -- the state a run starts from: the PIs cleared, du's filter at the observer's du (the
    -- bridge's error as learned the last time it ran: held while it is off), the prefilter at
    -- the observer's voltage (no step from where the output is)
    function voltage_control_start(du_slow : real; v_est : real := 0.0) return voltage_control_record;

end package output_voltage_control_pkg;

package body output_voltage_control_pkg is

    function voltage_control_design(period, l, c, r, current_hz, voltage_hz, i_max, vl_max, u_max : real;
        b : real := 1.0; du_hz : real := 20.0; voltage_zero : real := 0.2;
        prefilter_hz : real := 0.0; feedforward : real := 1.0) return voltage_control_design_record is
        variable retval : voltage_control_design_record;
        constant wi : real := MATH_2_PI * current_hz;
        constant wv : real := MATH_2_PI * voltage_hz;
    begin
        retval.period := period;
        retval.l      := l;
        retval.r      := r;
        retval.kp_i   := l * wi;
        retval.ki_i   := retval.kp_i * wi / 5.0;
        retval.kp_v   := c * wv;
        retval.ki_v   := retval.kp_v * wv * voltage_zero;
        retval.i_max  := i_max;
        retval.vl_max := vl_max;
        retval.u_max  := u_max;
        retval.b      := b;
        retval.du_alpha := MATH_2_PI * du_hz * period;
        if prefilter_hz > 0.0 then
            retval.prefilter_alpha := 1.0 - exp(-MATH_2_PI * prefilter_hz * period);
            retval.feedforward     := feedforward * c * retval.prefilter_alpha / period;
        else
            retval.prefilter_alpha := 1.0;
            retval.feedforward     := 0.0;
        end if;
        return retval;
    end function;

    function clamp(value, limit : real) return real is
    begin
        return realmax(-limit, realmin(limit, value));
    end function;

    function voltage_control_start(du_slow : real; v_est : real := 0.0) return voltage_control_record is
        variable retval : voltage_control_record := init_voltage_control;
    begin
        retval.du_filtered := du_slow;
        retval.reference_1 := v_est;
        retval.reference_2 := v_est;
        return retval;
    end function;

    function voltage_control_step(state : voltage_control_record; d : voltage_control_design_record;
        k : observer_coefficients_record; v_ref, i_est, v_est, du_slow, iload_slow, u_applied : real)
        return voltage_control_record is
        variable retval : voltage_control_record := state;
        variable i, v, i_next, reference, error_v, error_i, i_ref, vl_ref, u : real;
    begin
        -- 0: the prefilter and the feedforward of its slope
        retval.reference_1   := state.reference_1 + d.prefilter_alpha * (v_ref - state.reference_1);
        retval.i_feedforward := d.feedforward * (retval.reference_1 - state.reference_2);
        retval.reference_2   := state.reference_2 + d.prefilter_alpha * (retval.reference_1 - state.reference_2);
        reference := retval.reference_2;

        -- 1: a period ahead, two of the observer's half period steps without corrections
        i := i_est;
        v := v_est;
        for n in 1 to 2 loop
            i_next := k.phi11 * i + k.phi12 * v + k.gamma11 * (u_applied + du_slow) + k.gamma12 * iload_slow;
            v      := k.phi21 * i + k.phi22 * v + k.gamma21 * (u_applied + du_slow) + k.gamma22 * iload_slow;
            i      := i_next;
        end loop;
        retval.i_predicted := i;
        retval.v_predicted := v;

        -- 2: the voltage's PI, the current reference with the load's current and the feedforward;
        -- its integral on the estimate's error
        error_v := reference - v_est;
        i_ref   := d.kp_v * (d.b * reference - v) + state.integral_v + iload_slow + retval.i_feedforward;
        retval.current_limited := abs(i_ref) > d.i_max;
        if not retval.current_limited or (i_ref > 0.0) /= (error_v > 0.0) then
            retval.integral_v := state.integral_v + d.ki_v * d.period * error_v;
        end if;
        i_ref := clamp(i_ref, d.i_max);
        retval.i_ref := i_ref;

        -- 3: the current's PI, the inductor's voltage
        error_i := i_ref - i;
        vl_ref  := d.kp_i * error_i + state.integral_i;
        retval.voltage_limited := abs(vl_ref) > d.vl_max;
        if not retval.voltage_limited or (vl_ref > 0.0) /= (error_i > 0.0) then
            retval.integral_i := state.integral_i + d.ki_i * d.period * error_i;
        end if;
        vl_ref := clamp(vl_ref, d.vl_max);
        retval.vl_ref := vl_ref;

        -- 4: the bridge's voltage, less the bridge's error low passed
        retval.du_filtered := state.du_filtered + d.du_alpha * (du_slow - state.du_filtered);
        u := vl_ref + v + d.r * i - retval.du_filtered;
        retval.bridge_limited := abs(u) > d.u_max;
        retval.u_ref := clamp(u, d.u_max);
        return retval;
    end function;

end package body output_voltage_control_pkg;
