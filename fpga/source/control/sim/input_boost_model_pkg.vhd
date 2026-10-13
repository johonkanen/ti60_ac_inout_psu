library ieee;
    use ieee.math_real.all;

-- input_boost_control's voltage control in real numbers, step for step as
-- output_control_program_pkg's boost programs compute it on the core (boost_code a period's
-- step, boost_reset_code a run's first period): the fast model of the loop the testbenches
-- simulate for seconds, and the reference input_boost_control_tb checks the RTL against,
-- period by period, fed the same readings.
--
-- A period's step:
--   v        the link's period mean (kept when the period had no readings)
--   target   the reference a ramp's step on towards v_ref, at most ramp * 0.025 ms
--   ref_1    first prefilter stage   ref_1 += alpha * (target - ref_1)
--   ref_f    second stage            ref_f += alpha * (ref_1 - ref_f)
--   e        ref_f - v
--   duty     kp * e + int, limited to 0 .. d_max (as the core: about d_max/2, +-d_max/2)
--   int      += ki * e, held while the limit's excess and the increment push the same way
-- The gains as the registers hold them (24 fractional bits): boost_law_settings' quantized.
package input_boost_model_pkg is

    constant boost_period_clocks : natural := 3125;     -- 40 kHz at 125 MHz
    constant boost_t_period      : real    := 25.0e-6;
    constant boost_ms_a_period   : real    := 0.025;
    constant boost_d_max_ceiling : real    := 0.4;

    type boost_law_settings is record
        v_ref : real;   -- V
        ramp  : real;   -- V/ms; 0 or less: none (2000 V/ms, a step at once)
        d_max : real;   -- limited to 0 .. 0.2
        kp    : real;   -- duty/V
        ki    : real;   -- duty/V a period
        alpha : real;   -- each prefilter stage's share of a period's difference
    end record;

    -- the registers' defaults
    constant boost_law_defaults : boost_law_settings := (
        v_ref => 105.0, ramp => 0.5, d_max => 0.05, kp => 0.022, ki => 1.16e-5,
        alpha => 1.16e-5 / 0.022);

    type boost_law_state is record
        v      : real;
        target : real;
        ref_1  : real;
        ref_f  : real;
        int    : real;
        duty   : real;
        excess : real;
    end record;

    constant boost_law_zero : boost_law_state := (others => 0.0);

    -- the settings as the registers keep them: the gains and alpha with 24 fractional bits,
    -- the voltages, the ramp and d_max with 20
    function quantized(s : boost_law_settings) return boost_law_settings;

    -- a run's first period: the mean taken, the reference and the prefilter from it, the
    -- integral and the duty cleared
    procedure boost_law_reset(
        state     : inout boost_law_state;
        has       : boolean;
        v_mean    : real);

    -- a period's step from its readings' mean
    procedure boost_law_step(
        state     : inout boost_law_state;
        has       : boolean;
        v_mean    : real;
        settings  : boost_law_settings);

    -- the on time the RTL takes from a duty, clocks
    function boost_on_clocks(duty : real) return natural;

end package input_boost_model_pkg;

package body input_boost_model_pkg is

    function q(x : real; bits : natural) return real is
    begin
        return round(x * 2.0**bits) / 2.0**bits;
    end function;

    function quantized(s : boost_law_settings) return boost_law_settings is
    begin
        return (v_ref => q(s.v_ref, 20), ramp => q(s.ramp, 20), d_max => q(s.d_max, 20),
                kp => q(s.kp, 24), ki => q(s.ki, 24), alpha => q(s.alpha, 24));
    end function;

    procedure boost_law_reset(
        state     : inout boost_law_state;
        has       : boolean;
        v_mean    : real) is
    begin
        if has then
            state.v := v_mean;
        end if;
        state.target := state.v;
        state.ref_1  := state.v;
        state.ref_f  := state.v;
        state.int    := 0.0;
        state.duty   := 0.0;
        state.excess := 0.0;
    end procedure;

    procedure boost_law_step(
        state     : inout boost_law_state;
        has       : boolean;
        v_mean    : real;
        settings  : boost_law_settings) is
        variable ramp, step_max, step, e, raw, d_i, d_half, centered, lim : real;
    begin
        if has then
            state.v := v_mean;
        end if;
        -- the reference a ramp's step on towards v_ref
        ramp := settings.ramp;
        if ramp <= 0.0 then
            ramp := 2000.0;
        end if;
        -- a period in ms as the core's constant (24 fractional bits)
        step_max := ramp * round(boost_ms_a_period * 2.0**24) / 2.0**24;
        step := realmax(-step_max, realmin(step_max, settings.v_ref - state.target));
        state.target := state.target + step;
        -- the prefilter's two stages
        state.ref_1 := state.ref_1 + settings.alpha * (state.target - state.ref_1);
        state.ref_f := state.ref_f + settings.alpha * (state.ref_1 - state.ref_f);
        -- the PI, about d_max/2 limited to +-d_max/2
        e := state.ref_f - state.v;
        raw := settings.kp * e + state.int;
        d_i := settings.ki * e;
        d_half := realmax(0.0, realmin(boost_d_max_ceiling, settings.d_max)) / 2.0;
        centered := raw - d_half;
        lim := realmax(-d_half, realmin(d_half, centered));
        state.duty := lim + d_half;
        state.excess := centered - lim;
        -- the integral held while the excess and the increment push the same way
        if not ((d_i > 0.0 and state.excess > 0.0) or (d_i < 0.0 and state.excess < 0.0)) then
            state.int := state.int + d_i;
        end if;
    end procedure;

    function boost_on_clocks(duty : real) return natural is
        constant clocks : real := floor(duty * real(boost_period_clocks));
    begin
        if clocks <= 0.0 then
            return 0;
        elsif clocks >= real(boost_period_clocks) then
            return boost_period_clocks;
        end if;
        return natural(clocks);
    end function;

end package body input_boost_model_pkg;
