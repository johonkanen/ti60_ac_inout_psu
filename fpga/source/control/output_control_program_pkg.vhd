library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.microinstruction_pkg.all;
    use work.microprogram_assembler_pkg.all;
    use work.execution_unit_pkg.all;
    use work.microprogram_interface_pkg.jump_delay_slots;
    use work.dual_port_ram_pkg.ram_array;
    use work.fixed_format_pkg.all;
    use work.output_observer_pkg.all;
    use work.output_voltage_control_pkg.all;

-- The output filter's observer and the output voltage's control as programs of
-- hVHDL_microprogram_processor (output_control_processor runs them): the steps of
-- output_observer_pkg and output_voltage_control_pkg, on one fixed point multiply-add.
--
-- The data is 36 bits with 24 fractional bits (+-2048); each result is rounded to them
-- (execution_unit's g_round_result: truncated, the observer's voltage integral, ki_v dt ~1e-4
-- a step, would bias its estimate by ~0.4 mV); the packages' fixed point steps round to 20
-- bits once, so the programs follow them to a few of their last bits, not bit for bit. The RTL's values (fixed_number, 20 fractional bits)
-- go in shifted up by 4 bits, out shifted down; the gains (coefficient_number, 24) as they are.
--
-- Four programs:
--   observer_start  a step of the observer (observer_step): its inputs i_meas, the readings'
--                   sum and 1/count (has_reading 1.0, or 0.0 with none: no correction), u and
--                   on (1.0 the bridge running, 0.0 off: the bridge off mode)
--   disabled_start  the observer disabled: no current, the PIs cleared, the voltage a low
--                   pass of the readings' mean
--   control_start   a step of the control (voltage_control_step) from the observer's state
--                   as left by its step, with v_ref and u_applied: the reference through the
--                   prefilter and its slope fed forward, then the PIs; the anti-windup by
--                   mi_block on each limit's excess
--   reset_start     the control stopped: its integrals cleared, du's filter at the observer's
--                   du, the prefilter at its voltage (voltage_control_start)
-- and two of the input boost's (input_boost_control's voltage control, its link's voltage held
-- by the duty alone: the input current too noisy to control):
--   boost_start        a period's step: the link's readings' mean (none: the mean before), the
--                      reference a ramp's step on towards v_ref, then through the prefilter (two
--                      first order low passes, each's share of a period's difference alpha: at
--                      the PI's zero, ki / kp, the first cancels the zero, whose lead overshoots
--                      a step, the second damps what the loop's own poles still overshoot),
--                      the PI on the filtered reference less the mean to the duty, limited to 0 .. d_max (the limits as +-d_max/2 about
--                      d_max/2; the integral held while the limit's excess and its increment
--                      push the same way), the on time as half clocks
--   boost_reset_start  a start: the mean taken, the reference and the prefilter from it, the
--                      integral and the duty cleared
package output_control_program_pkg is

    constant program_radix      : natural := 24;
    constant program_data_width : natural := 36;

    -- fixed_dsp's pre-adder and product registers, the data ram's output register, no
    -- forwarding: a result readable 9 instructions after
    constant program_config : processor_config := (
        instruction_width => 36
        ,data_width       => program_data_width
        ,radix            => program_radix
        ,result_latency   => fixed_point_result_latency(true, true, true, false)
        ,delay_slots      => jump_delay_slots(true)
        ,math_latency     => 0
        ,forwarded        => 0);

    constant data_words    : natural := 256;
    constant program_words : natural := 1024;

    -- the data ram
    constant a_zero : natural := 0;    constant a_one   : natural := 1;
    constant a_half : natural := 2;    constant a_disabled_alpha : natural := 3;
    -- the observer's coefficients, its registers 1..12 at a_observer_k + register - 1
    constant a_observer_k : natural := 8;
    constant a_phi11 : natural := 8;   constant a_phi12 : natural := 9;
    constant a_phi21 : natural := 10;  constant a_phi22 : natural := 11;
    constant a_g11   : natural := 12;  constant a_g12   : natural := 13;
    constant a_g21   : natural := 14;  constant a_g22   : natural := 15;
    constant a_kp_i  : natural := 16;  constant a_ki_i  : natural := 17;
    constant a_kp_v  : natural := 18;  constant a_ki_v  : natural := 19;
    -- the control's, its registers 1..10 at a_control_k + register - 1
    constant a_control_k : natural := 20;
    constant a_c_kpvb : natural := 20; constant a_c_kpv  : natural := 21;
    constant a_c_kiv  : natural := 22; constant a_c_kpi  : natural := 23;
    constant a_c_kii  : natural := 24; constant a_c_r    : natural := 25;
    constant a_c_alpha : natural := 26;
    constant a_c_imax : natural := 27; constant a_c_vlmax : natural := 28;
    constant a_c_umax : natural := 29;
    -- the control's prefilter and feedforward, its registers 17 and 18
    constant a_c_prefilter : natural := 30; constant a_c_feedforward : natural := 31;
    -- the inputs
    constant a_i_meas : natural := 32; constant a_v_sum : natural := 33;
    constant a_recip  : natural := 34; constant a_has   : natural := 35;
    constant a_u      : natural := 36; constant a_on    : natural := 37;
    constant a_v_ref  : natural := 38; constant a_u_app : natural := 39;
    -- the observer's state
    constant a_i      : natural := 40; constant a_v      : natural := 41;
    constant a_i_prev : natural := 42; constant a_v_prev : natural := 43;
    constant a_int_i  : natural := 44; constant a_int_v  : natural := 45;
    constant a_du     : natural := 46; constant a_iload  : natural := 47;
    -- the control's state and results
    constant a_cint_v : natural := 48; constant a_cint_i : natural := 49;
    constant a_du_f   : natural := 50;
    constant a_i_pred : natural := 51; constant a_v_pred : natural := 52;
    constant a_i_ref  : natural := 53; constant a_vl_ref : natural := 54;
    constant a_u_ref  : natural := 55;
    constant a_excess_i : natural := 56; constant a_excess_vl : natural := 57;
    constant a_excess_u : natural := 58;
    -- the prefilter's stages: the second the voltage PI's reference
    constant a_ref_1 : natural := 60; constant a_ref_2 : natural := 61;
    constant a_i_ff  : natural := 62;
    -- the boost's: its inputs (the gains with 24 fractional bits), its state and results, its
    -- constants (a period in ms, the period's half clocks)
    constant a_b_v_sum : natural := 96;  constant a_b_recip : natural := 97;
    constant a_b_has   : natural := 98;  constant a_b_v_ref : natural := 99;
    constant a_b_ramp  : natural := 100; constant a_b_dhalf : natural := 101;
    constant a_b_kp    : natural := 102; constant a_b_ki    : natural := 103;
    constant a_b_v     : natural := 104; constant a_b_target : natural := 105;
    constant a_b_int   : natural := 106; constant a_b_duty  : natural := 107;
    constant a_b_on    : natural := 108; constant a_b_excess : natural := 109;
    constant a_b_ms    : natural := 110; constant a_b_half_clocks : natural := 111;
    constant a_b_alpha : natural := 112; constant a_b_ref_f : natural := 113;
    constant a_b_ref_1 : natural := 114;
    -- the most of the link's readings a period
    constant boost_max_readings : natural := 8;

    function observer_code return microprogram;
    function disabled_code return microprogram;
    function control_code return microprogram;
    function reset_code return microprogram;
    function boost_code return microprogram;
    function boost_reset_code return microprogram;

    constant observer_start : natural := 0;
    constant disabled_start : natural := 128;
    constant control_start  : natural := 192;
    constant reset_start    : natural := 448;
    constant boost_start    : natural := 512;
    constant boost_reset_start : natural := 640;

    function control_program return ram_array;

    -- the data ram's contents: the constants and the default coefficients
    function control_data(k : observer_coefficients_record; d : voltage_control_design_record) return ram_array;

    -- the formats
    subtype program_word is signed(program_data_width-1 downto 0);
    function to_program(value : fixed_number) return program_word;     -- 20 fractional bits
    function coefficient_to_program(value : signed) return program_word; -- 24
    function from_program(value : std_logic_vector) return fixed_number;

end package output_control_program_pkg;

package body output_control_program_pkg is

    -- temporaries: the output's programs', the boost's
    constant t0 : natural := 64;
    constant t1_boost : natural := 116;

    function observer_code return microprogram is
        constant a1 : natural := t0;       constant b1 : natural := t0 + 1;
        constant v_mean : natural := t0 + 2;
        constant v_meas : natural := t0 + 3;
        constant e_i : natural := t0 + 4;  constant e_v : natural := t0 + 5;
        constant a2 : natural := t0 + 6;   constant b2 : natural := t0 + 7;
        constant upd : natural := t0 + 8;  constant u_total : natural := t0 + 9;
        constant a3 : natural := t0 + 10;  constant b3 : natural := t0 + 11;
    begin
        return (
            -- the state's products, the errors' operands
             mi(mpy_add       , a1      , a_i      , a_phi11 , a_zero)
            ,mi(mpy_add       , b1      , a_i      , a_phi21 , a_zero)
            ,mi(a_add_b_mpy_c , v_mean  , a_v      , a_v_prev, a_half)
            ,mi(mpy_add       , v_meas  , a_v_sum  , a_recip , a_zero)
            -- the current's error: the measured is the model's of a step ago; none with the
            -- bridge off
            ,mi(a_sub_b_mpy_c , e_i     , a_i_meas , a_i_prev, a_on)
            ,mi(mpy_add       , a2      , a_v      , a_phi12 , a1)
            ,mi(mpy_add       , b2      , a_v      , a_phi22 , b1)
            ,mi(mpy_add       , a_i_prev, a_i      , a_one   , a_zero)
            ,mi(mpy_add       , a_v_prev, a_v      , a_one   , a_zero)
            ,mi(mpy_add       , a_int_i , e_i      , a_ki_i  , a_int_i)
            -- the voltage's error: the readings' mean against the model's over the step, none
            -- without readings
            ,mi(a_sub_b_mpy_c , e_v     , v_meas   , v_mean  , a_has)
            ,mi(mpy_add       , a_du    , e_i      , a_kp_i  , a_int_i)
            ,mi(mpy_add       , a_int_v , e_v      , a_ki_v  , a_int_v)
            ,mi(mpy_add       , upd     , a_u      , a_one   , a_du)
            ,mi(neg_mpy_sub   , a_iload , e_v      , a_kp_v  , a_int_v)
            -- the bridge's voltage u + du, or with the bridge off the capacitor's: no current
            ,mi(lp_filter     , u_total , upd      , a_v     , a_on)
            ,mi(mpy_add       , a3      , a_iload  , a_g12   , a2)
            ,mi(mpy_add       , b3      , a_iload  , a_g22   , b2)
            ,mi(mpy_add       , a_i     , u_total  , a_g11   , a3)
            ,mi(mpy_add       , a_v     , u_total  , a_g21   , b3)
            ,mi(program_end));
    end function;

    function disabled_code return microprogram is
        constant v_meas : natural := t0;
        constant alpha  : natural := t0 + 1;
    begin
        return (
             mi(mpy_add   , v_meas  , a_v_sum , a_recip , a_zero)
            ,mi(mpy_add   , alpha   , a_has   , a_disabled_alpha, a_zero)
            ,mi(mpy_add   , a_i     , a_zero  , a_zero  , a_zero)
            ,mi(mpy_add   , a_i_prev, a_zero  , a_zero  , a_zero)
            ,mi(mpy_add   , a_int_i , a_zero  , a_zero  , a_zero)
            ,mi(mpy_add   , a_int_v , a_zero  , a_zero  , a_zero)
            ,mi(mpy_add   , a_du    , a_zero  , a_zero  , a_zero)
            ,mi(mpy_add   , a_iload , a_zero  , a_zero  , a_zero)
            ,mi(lp_filter , a_v     , v_meas  , a_v     , alpha)
            ,mi(mpy_add   , a_v_prev, a_v     , a_one   , a_zero)
            ,mi(program_end));
    end function;

    function control_code return microprogram is
        constant u_eff  : natural := t0;       constant load   : natural := t0 + 1;
        constant ci1    : natural := t0 + 2;   constant cv1    : natural := t0 + 3;
        constant ci     : natural := t0 + 4;   constant cv     : natural := t0 + 5;
        constant p      : natural := t0 + 6;   constant q      : natural := t0 + 7;
        constant i1     : natural := t0 + 8;   constant v1     : natural := t0 + 9;
        constant p2     : natural := t0 + 10;  constant q2     : natural := t0 + 11;
        constant t1     : natural := t0 + 12;  constant t2     : natural := t0 + 13;
        constant e_v    : natural := t0 + 14;  constant raw_i  : natural := t0 + 15;
        constant d_v    : natural := t0 + 16;  constant d_vb   : natural := t0 + 17;
        constant e_i    : natural := t0 + 18;  constant raw_vl : natural := t0 + 19;
        constant d_i    : natural := t0 + 20;  constant d_ib   : natural := t0 + 21;
        constant s1     : natural := t0 + 22;  constant s2     : natural := t0 + 23;
        constant raw_u  : natural := t0 + 24;
        constant t3     : natural := t0 + 25;
    begin
        return (
            -- 0: the reference processed before the voltage PI: the prefilter's two stages, the
            -- second's slope (its input less its output before) fed forward as C dv/dt
             mi(lp_filter    , a_ref_1, a_v_ref , a_ref_1 , a_c_prefilter)
            ,mi(a_sub_b_mpy_c, a_i_ff , a_ref_1 , a_ref_2 , a_c_feedforward)
            ,mi(lp_filter    , a_ref_2, a_ref_1 , a_ref_2 , a_c_prefilter)
            -- 1: the prediction, two of the observer's model steps with the reference applied
            -- now, the slow du (the current PI's integral) and iload (the voltage's, negated);
            -- their inputs' part the same in both
            ,mi(mpy_add      , u_eff  , a_u_app , a_one   , a_int_i)
            ,mi(neg_mpy_add  , load   , a_int_v , a_one   , a_zero)
            ,mi(mpy_add      , t1     , a_ref_2 , a_c_kpvb, a_cint_v)
            ,mi(lp_filter    , a_du_f , a_int_i , a_du_f  , a_c_alpha)
            ,mi(mpy_add      , ci1    , u_eff   , a_g11   , a_zero)
            ,mi(mpy_add      , cv1    , u_eff   , a_g21   , a_zero)
            ,mi(mpy_add      , t3     , load    , a_one   , t1)
            ,mi(mpy_add      , t2     , a_i_ff  , a_one   , t3)
            ,mi(mpy_add      , ci     , load    , a_g12   , ci1)
            ,mi(mpy_add      , cv     , load    , a_g22   , cv1)
            ,mi(mpy_add      , p      , a_i     , a_phi11 , ci)
            ,mi(mpy_add      , q      , a_i     , a_phi21 , cv)
            ,mi(mpy_add      , i1     , a_v     , a_phi12 , p)
            ,mi(mpy_add      , v1     , a_v     , a_phi22 , q)
            ,mi(mpy_add      , p2     , i1      , a_phi11 , ci)
            ,mi(mpy_add      , q2     , i1      , a_phi21 , cv)
            ,mi(mpy_add      , a_i_pred, v1     , a_phi12 , p2)
            ,mi(mpy_add      , a_v_pred, v1     , a_phi22 , q2)
            -- 2: the voltage's PI, the current reference with the load's current, limited;
            -- its integral held while the limit's excess and the error push the same way
            -- its integral on the estimate's error (the prediction is biased while du moves)
            ,mi(mpy_sub      , e_v    , a_ref_2 , a_one   , a_v)
            ,mi(neg_mpy_add  , raw_i  , a_v_pred, a_c_kpv , t2)
            ,mi(mpy_add      , d_v    , e_v     , a_c_kiv , a_zero)
            ,mi_limit(a_i_ref, raw_i, a_c_imax)
            ,mi(mpy_sub      , a_excess_i, raw_i, a_one   , a_i_ref)
            -- 3: the current's PI, the inductor's voltage, the same way
            ,mi(mpy_sub      , e_i    , a_i_ref , a_one   , a_i_pred)
            ,mi_block(d_vb, d_v, a_excess_i)
            ,mi(mpy_add      , raw_vl , e_i     , a_c_kpi , a_cint_i)
            ,mi(mpy_add      , d_i    , e_i     , a_c_kii , a_zero)
            ,mi(mpy_add      , a_cint_v, d_vb   , a_one   , a_cint_v)
            ,mi_limit(a_vl_ref, raw_vl, a_c_vlmax)
            ,mi(mpy_sub      , a_excess_vl, raw_vl, a_one , a_vl_ref)
            ,mi(mpy_add      , s1     , a_i_pred, a_c_r   , a_vl_ref)
            ,mi_block(d_ib, d_i, a_excess_vl)
            -- 4: the bridge's voltage: the inductor's, the capacitor's and R i, less du's low pass
            ,mi(mpy_add      , s2     , a_v_pred, a_one   , s1)
            ,mi(mpy_add      , a_cint_i, d_ib   , a_one   , a_cint_i)
            ,mi(mpy_sub      , raw_u  , s2      , a_one   , a_du_f)
            ,mi_limit(a_u_ref, raw_u, a_c_umax)
            ,mi(mpy_sub      , a_excess_u, raw_u, a_one   , a_u_ref)
            ,mi(program_end));
    end function;

    function reset_code return microprogram is
    begin
        return (
             mi(mpy_add , a_cint_v  , a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_ref_1   , a_v     , a_one  , a_zero)
            ,mi(mpy_add , a_ref_2   , a_v     , a_one  , a_zero)
            ,mi(mpy_add , a_i_ff    , a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_cint_i  , a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_du_f    , a_int_i , a_one  , a_zero)
            ,mi(mpy_add , a_i_ref   , a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_vl_ref  , a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_u_ref   , a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_excess_i, a_zero  , a_zero , a_zero)
            ,mi(mpy_add , a_excess_vl, a_zero , a_zero , a_zero)
            ,mi(mpy_add , a_excess_u, a_zero  , a_zero , a_zero)
            ,mi(program_end));
    end function;

    function boost_code return microprogram is
        constant v_meas   : natural := t1_boost;      constant d_t      : natural := t1_boost + 1;
        constant step_max : natural := t1_boost + 2;  constant step     : natural := t1_boost + 3;
        constant e        : natural := t1_boost + 4;  constant raw      : natural := t1_boost + 5;
        constant d_i      : natural := t1_boost + 6;  constant centered : natural := t1_boost + 7;
        constant lim      : natural := t1_boost + 8;  constant d_ib     : natural := t1_boost + 9;
    begin
        return (
            -- the link's mean, the reference's step towards v_ref
             mi(mpy_add   , v_meas    , a_b_v_sum , a_b_recip , a_zero)
            ,mi(mpy_sub   , d_t       , a_b_v_ref , a_one     , a_b_target)
            ,mi(mpy_add   , step_max  , a_b_ramp  , a_b_ms    , a_zero)
            ,mi(lp_filter , a_b_v     , v_meas    , a_b_v     , a_b_has)
            ,mi_limit(step, d_t, step_max)
            ,mi(mpy_add   , a_b_target, step      , a_one     , a_b_target)
            -- the prefilter, then the PI, its output about d_max/2 limited to +-d_max/2
            ,mi(lp_filter , a_b_ref_1 , a_b_target, a_b_ref_1 , a_b_alpha)
            ,mi(lp_filter , a_b_ref_f , a_b_ref_1 , a_b_ref_f , a_b_alpha)
            ,mi(mpy_sub   , e         , a_b_ref_f , a_one     , a_b_v)
            ,mi(mpy_add   , raw       , e         , a_b_kp    , a_b_int)
            ,mi(mpy_add   , d_i       , e         , a_b_ki    , a_zero)
            ,mi(mpy_sub   , centered  , raw       , a_one     , a_b_dhalf)
            ,mi_limit(lim, centered, a_b_dhalf)
            ,mi(mpy_add   , a_b_duty  , lim       , a_one     , a_b_dhalf)
            ,mi(mpy_sub   , a_b_excess, centered  , a_one     , lim)
            ,mi(mpy_add   , a_b_on    , a_b_duty  , a_b_half_clocks, a_zero)
            ,mi_block(d_ib, d_i, a_b_excess)
            ,mi(mpy_add   , a_b_int   , d_ib      , a_one     , a_b_int)
            ,mi(program_end));
    end function;

    function boost_reset_code return microprogram is
        constant v_meas : natural := t1_boost;
    begin
        return (
             mi(mpy_add   , v_meas    , a_b_v_sum , a_b_recip , a_zero)
            ,mi(lp_filter , a_b_v     , v_meas    , a_b_v     , a_b_has)
            ,mi(mpy_add   , a_b_target, a_b_v     , a_one     , a_zero)
            ,mi(mpy_add   , a_b_ref_1 , a_b_v     , a_one     , a_zero)
            ,mi(mpy_add   , a_b_ref_f , a_b_v     , a_one     , a_zero)
            ,mi(mpy_add   , a_b_int   , a_zero    , a_zero    , a_zero)
            ,mi(mpy_add   , a_b_duty  , a_zero    , a_zero    , a_zero)
            ,mi(mpy_add   , a_b_on    , a_zero    , a_zero    , a_zero)
            ,mi(mpy_add   , a_b_excess, a_zero    , a_zero    , a_zero)
            ,mi(program_end));
    end function;

    function control_program return ram_array is
        variable retval : microprogram(0 to program_words-1) := empty_program(program_words);
    begin
        retval := place(retval, observer_start, schedule(program_config, observer_code));
        retval := place(retval, disabled_start, schedule(program_config, disabled_code));
        retval := place(retval, control_start,  schedule(program_config, control_code));
        retval := place(retval, reset_start,    schedule(program_config, reset_code));
        retval := place(retval, boost_start,    schedule(program_config, boost_code));
        retval := place(retval, boost_reset_start, schedule(program_config, boost_reset_code));
        return encode(retval, program_config.instruction_width);
    end function;

    function control_data(k : observer_coefficients_record; d : voltage_control_design_record) return ram_array is
        constant zeros : ram_array(0 to data_words-1)(program_data_width-1 downto 0) := (others => (others => '0'));
    begin
        return set_data(zeros, (
            (a_one, 1.0), (a_half, 0.5), (a_disabled_alpha, 0.125),
            (a_b_ms, 0.025), (a_b_half_clocks, 1562.5),
            (a_phi11, k.phi11), (a_phi12, k.phi12), (a_phi21, k.phi21), (a_phi22, k.phi22),
            (a_g11, k.gamma11), (a_g12, k.gamma12), (a_g21, k.gamma21), (a_g22, k.gamma22),
            (a_kp_i, k.kp_i), (a_ki_i, k.ki_dt_i), (a_kp_v, k.kp_v), (a_ki_v, k.ki_dt_v),
            (a_c_kpvb, d.kp_v * d.b), (a_c_kpv, d.kp_v), (a_c_kiv, d.ki_v * d.period),
            (a_c_kpi, d.kp_i), (a_c_kii, d.ki_i * d.period), (a_c_r, d.r), (a_c_alpha, d.du_alpha),
            (a_c_imax, d.i_max), (a_c_vlmax, d.vl_max), (a_c_umax, d.u_max),
            (a_c_prefilter, d.prefilter_alpha), (a_c_feedforward, d.feedforward)), program_config);
    end function;

    function to_program(value : fixed_number) return program_word is
    begin
        return shift_left(resize(value, program_data_width), program_radix - fixed_radix);
    end function;

    function coefficient_to_program(value : signed) return program_word is
    begin
        return resize(value, program_data_width);
    end function;

    function from_program(value : std_logic_vector) return fixed_number is
    begin
        return resize(shift_right(signed(value), program_radix - fixed_radix), fixed_wordlength);
    end function;

end package body output_control_program_pkg;
