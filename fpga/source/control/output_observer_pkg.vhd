library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;

    use work.fixed_format_pkg.all;

-- The output filter's observer, as functions: a model of the filter run alongside it, pulled
-- to the measurements by two PI controllers whose outputs are the disturbances.
--
--   L di/dt = u + du - R i - v        du     = PI_i(i_meas - i)    the bridge's voltage error
--   C dv/dt = i - iload               iload  = -PI_v(v_meas - v)   the load current
--
-- u is the modulator's voltage reference (the bridge's average voltage but for du: dead time,
-- the DC link's error, offsets), i the inductor current, v the filter capacitor's voltage.
--
-- With the bridge off (bridge_on false: its gates off, nothing driving the filter) the model's
-- bridge voltage is its own capacitor voltage, so its current stays at none, and the current's
-- PI stops: its error taken as none, du held at its integral, the bridge's error as learned the
-- last time it ran. The voltage's PI goes on: the voltage estimate follows the readings, iload
-- any load (a bleed resistor) - the observer runs whether the bridge does or not.
--
-- A step is half a switching period (the modulator's sample pulses): the model advances by the
-- exact zero order hold discretization of the filter with u + du and iload held over the step,
--
--   [i v](k+1) = phi [i v](k) + gamma [u + du, iload](k)
--
-- The measurements of a step: i_meas the current's mean over the last switching period, which
-- is the current half a period (a step) before, so compared with the model's current of the
-- step before; v_meas the capacitor voltage's mean of the readings since the last step,
-- compared with the model's mean over the step, (v(k) + v(k-1)) / 2.
--
-- Two versions: in real numbers, for designing and checking (the coefficients from the
-- filter's L, C, R and the step), and in fixed point as the FPGA computes it, bit for bit: the
-- RTL (output_observer) is checked against observer_step below. The fixed point step:
--   states, inputs and outputs fixed_number (20 fractional bits);
--   coefficients coefficient_number: 32 bits, 24 fractional bits (+-128);
--   products and sums at 44 fractional bits in 64 bits (fixed_dsp's), the integrators kept
--   there, each result rounded to 20 fractional bits once.
package output_observer_pkg is

    -- the filter and the observer's design, real
    type observer_design_record is record
        l, c, r  : real;    -- H, F, ohm
        dt       : real;    -- s, a step: half a switching period
        kp_i     : real;    -- V/A    the current's PI
        ki_i     : real;    -- V/(A s)
        kp_v     : real;    -- A/V    the voltage's PI
        ki_v     : real;    -- A/(V s)
    end record;

    -- the PI gains for a bandwidth and damping of each error loop: the current's error as
    -- L s^2 + (R + Kp) s + Ki, the voltage's as C s^2 + Kp s + Ki
    function observer_design(l, c, r, dt, current_hz, voltage_hz : real; damping : real := 0.7)
        return observer_design_record;

    -- the step's coefficients, real
    type observer_coefficients_record is record
        phi11, phi12, phi21, phi22         : real;   -- the state's transition
        gamma11, gamma12, gamma21, gamma22 : real;   -- from u + du and iload
        kp_i, ki_dt_i, kp_v, ki_dt_v       : real;   -- the PIs', the integral gains times dt
    end record;

    function observer_coefficients(design : observer_design_record) return observer_coefficients_record;

    -- the observer's state, real
    type observer_real_record is record
        i, v               : real;   -- the model
        i_prev, v_prev     : real;   -- the model's of the step before
        integral_i         : real;   -- the PIs' integrals: du's and iload's steady parts
        integral_v         : real;
        du, iload          : real;   -- the PIs' outputs of the step
    end record;

    constant init_observer_real : observer_real_record := (others => 0.0);

    function observer_step(state : observer_real_record; k : observer_coefficients_record;
        u, i_meas, v_meas : real; bridge_on : boolean := true) return observer_real_record;

    ------------------------------------------------------------------------
    -- fixed point
    constant coefficient_radix : natural := 24;
    subtype coefficient_number is signed(31 downto 0);
    constant product_radix     : natural := fixed_radix + coefficient_radix;   -- 44
    subtype product_number is signed(63 downto 0);

    function to_coefficient(value : real) return coefficient_number;

    type observer_fixed_coefficients_record is record
        phi11, phi12, phi21, phi22         : coefficient_number;
        gamma11, gamma12, gamma21, gamma22 : coefficient_number;
        kp_i, ki_dt_i, kp_v, ki_dt_v       : coefficient_number;
    end record;

    function to_fixed(k : observer_coefficients_record) return observer_fixed_coefficients_record;

    type observer_fixed_record is record
        i, v               : fixed_number;
        i_prev, v_prev     : fixed_number;
        integral_i         : product_number;   -- at product_radix
        integral_v         : product_number;
        du, iload          : fixed_number;
    end record;

    constant init_observer_fixed : observer_fixed_record := (
        i => fixed_zero, v => fixed_zero, i_prev => fixed_zero, v_prev => fixed_zero,
        integral_i => (others => '0'), integral_v => (others => '0'),
        du => fixed_zero, iload => fixed_zero);

    -- a product, as fixed_dsp's: a fixed_number times a coefficient at product_radix
    function product(a : fixed_number; b : coefficient_number) return product_number;
    -- a sum at product_radix to fixed_number, rounded
    function to_fixed_rounded(value : product_number) return fixed_number;

    function observer_step(state : observer_fixed_record; k : observer_fixed_coefficients_record;
        u, i_meas, v_meas : fixed_number; bridge_on : boolean := true) return observer_fixed_record;

    -- the voltage measurement of a step: the mean of the readings since the last step (their
    -- sum, 1..max_readings of them), a product by 1/count; with none, the model's own mean over
    -- the step (no correction)
    constant max_readings : natural := 4;
    function readings_mean(sum : fixed_number; count : natural; state : observer_fixed_record) return fixed_number;

    -- the PIs' integrals, the disturbances' slow parts: du's (V) and iload's (A)
    function slow_du(state : observer_fixed_record) return fixed_number;
    function slow_iload(state : observer_fixed_record) return fixed_number;

    -- for checks: the fixed point state in reals
    function to_real(state : observer_fixed_record) return observer_real_record;

end package output_observer_pkg;

package body output_observer_pkg is

    function observer_design(l, c, r, dt, current_hz, voltage_hz : real; damping : real := 0.7)
        return observer_design_record is
        variable retval : observer_design_record;
        constant wi : real := MATH_2_PI * current_hz;
        constant wv : real := MATH_2_PI * voltage_hz;
    begin
        retval.l    := l;
        retval.c    := c;
        retval.r    := r;
        retval.dt   := dt;
        retval.kp_i := 2.0 * damping * wi * l - r;
        retval.ki_i := l * wi**2;
        retval.kp_v := 2.0 * damping * wv * c;
        retval.ki_v := c * wv**2;
        return retval;
    end function;

    ------------------------------------------------------------------------
    type matrix2 is array (1 to 2, 1 to 2) of real;

    function "*"(a, b : matrix2) return matrix2 is
        variable retval : matrix2;
    begin
        for m in 1 to 2 loop
            for n in 1 to 2 loop
                retval(m, n) := a(m, 1) * b(1, n) + a(m, 2) * b(2, n);
            end loop;
        end loop;
        return retval;
    end function;

    -- the zero order hold: phi = e^(A dt), gamma = (integral of e^(A s) over 0..dt) B, both by
    -- their series (A dt is ~0.2 at most: 30 terms are exact to the last bit)
    function observer_coefficients(design : observer_design_record) return observer_coefficients_record is
        constant a : matrix2 := ((-design.r / design.l, -1.0 / design.l),
                                 (1.0 / design.c, 0.0));
        -- the inputs u + du and iload
        constant b : matrix2 := ((1.0 / design.l, 0.0),
                                 (0.0, -1.0 / design.c));
        variable term     : matrix2 := ((1.0, 0.0), (0.0, 1.0));   -- (A dt)^n / n!
        variable phi      : matrix2 := ((1.0, 0.0), (0.0, 1.0));
        variable integral : matrix2 := ((design.dt, 0.0), (0.0, design.dt));
        variable adt      : matrix2;
        variable gamma    : matrix2;
        variable retval   : observer_coefficients_record;
    begin
        for m in 1 to 2 loop
            for n in 1 to 2 loop
                adt(m, n) := a(m, n) * design.dt;
            end loop;
        end loop;
        for k in 1 to 30 loop
            term := term * adt;
            for m in 1 to 2 loop
                for n in 1 to 2 loop
                    term(m, n)     := term(m, n) / real(k);
                    phi(m, n)      := phi(m, n) + term(m, n);
                    -- integral of (A s)^k / k! over 0..dt: (A dt)^k / k! * dt / (k + 1)
                    integral(m, n) := integral(m, n) + term(m, n) * design.dt / real(k + 1);
                end loop;
            end loop;
        end loop;
        gamma := integral * b;

        retval.phi11   := phi(1, 1);
        retval.phi12   := phi(1, 2);
        retval.phi21   := phi(2, 1);
        retval.phi22   := phi(2, 2);
        retval.gamma11 := gamma(1, 1);
        retval.gamma12 := gamma(1, 2);
        retval.gamma21 := gamma(2, 1);
        retval.gamma22 := gamma(2, 2);
        retval.kp_i    := design.kp_i;
        retval.ki_dt_i := design.ki_i * design.dt;
        retval.kp_v    := design.kp_v;
        retval.ki_dt_v := design.ki_v * design.dt;
        return retval;
    end function;

    function observer_step(state : observer_real_record; k : observer_coefficients_record;
        u, i_meas, v_meas : real; bridge_on : boolean := true) return observer_real_record is
        variable retval : observer_real_record := state;
        variable error_i, error_v, u_total : real;
    begin
        -- the errors: the measured current is the model's of a step ago, the measured voltage
        -- the mean over the step
        error_i := i_meas - state.i_prev;
        if not bridge_on then
            error_i := 0.0;
        end if;
        error_v := v_meas - (state.v + state.v_prev) / 2.0;

        retval.integral_i := state.integral_i + k.ki_dt_i * error_i;
        retval.integral_v := state.integral_v + k.ki_dt_v * error_v;
        retval.du    := k.kp_i * error_i + retval.integral_i;
        retval.iload := -(k.kp_v * error_v + retval.integral_v);

        u_total       := u + retval.du;
        if not bridge_on then
            u_total := state.v;
        end if;
        retval.i      := k.phi11 * state.i + k.phi12 * state.v + k.gamma11 * u_total + k.gamma12 * retval.iload;
        retval.v      := k.phi21 * state.i + k.phi22 * state.v + k.gamma21 * u_total + k.gamma22 * retval.iload;
        retval.i_prev := state.i;
        retval.v_prev := state.v;
        return retval;
    end function;

    ------------------------------------------------------------------------
    function to_coefficient(value : real) return coefficient_number is
    begin
        return to_signed(integer(round(value * 2.0**coefficient_radix)), 32);
    end function;

    function to_fixed(k : observer_coefficients_record) return observer_fixed_coefficients_record is
    begin
        return (
            phi11   => to_coefficient(k.phi11),   phi12   => to_coefficient(k.phi12),
            phi21   => to_coefficient(k.phi21),   phi22   => to_coefficient(k.phi22),
            gamma11 => to_coefficient(k.gamma11), gamma12 => to_coefficient(k.gamma12),
            gamma21 => to_coefficient(k.gamma21), gamma22 => to_coefficient(k.gamma22),
            kp_i    => to_coefficient(k.kp_i),    ki_dt_i => to_coefficient(k.ki_dt_i),
            kp_v    => to_coefficient(k.kp_v),    ki_dt_v => to_coefficient(k.ki_dt_v));
    end function;

    function product(a : fixed_number; b : coefficient_number) return product_number is
    begin
        return a * b;
    end function;

    -- half a step of the result's resolution added, then the bits below it dropped
    constant rounding : product_number := shift_left(to_signed(1, 64), coefficient_radix - 1);

    function to_fixed_rounded(value : product_number) return fixed_number is
    begin
        return resize(shift_right(value + rounding, coefficient_radix), fixed_wordlength);
    end function;

    function observer_step(state : observer_fixed_record; k : observer_fixed_coefficients_record;
        u, i_meas, v_meas : fixed_number; bridge_on : boolean := true) return observer_fixed_record is
        variable retval : observer_fixed_record := state;
        variable error_i, error_v, u_total : fixed_number;
    begin
        error_i := i_meas - state.i_prev;
        if not bridge_on then
            error_i := fixed_zero;
        end if;
        error_v := v_meas - shift_right(state.v + state.v_prev, 1);

        retval.integral_i := state.integral_i + product(error_i, k.ki_dt_i);
        retval.integral_v := state.integral_v + product(error_v, k.ki_dt_v);
        retval.du    := to_fixed_rounded(product(error_i, k.kp_i) + retval.integral_i);
        retval.iload := to_fixed_rounded(-(product(error_v, k.kp_v) + retval.integral_v));

        u_total       := u + retval.du;
        if not bridge_on then
            u_total := state.v;
        end if;
        retval.i      := to_fixed_rounded(product(state.i, k.phi11) + product(state.v, k.phi12) +
                                          product(u_total, k.gamma11) + product(retval.iload, k.gamma12));
        retval.v      := to_fixed_rounded(product(state.i, k.phi21) + product(state.v, k.phi22) +
                                          product(u_total, k.gamma21) + product(retval.iload, k.gamma22));
        retval.i_prev := state.i;
        retval.v_prev := state.v;
        return retval;
    end function;

    function readings_mean(sum : fixed_number; count : natural; state : observer_fixed_record) return fixed_number is
    begin
        if count = 0 then
            return shift_right(state.v + state.v_prev, 1);
        end if;
        return to_fixed_rounded(product(sum, to_coefficient(1.0 / real(count))));
    end function;

    function slow_du(state : observer_fixed_record) return fixed_number is
    begin
        return to_fixed_rounded(state.integral_i);
    end function;

    function slow_iload(state : observer_fixed_record) return fixed_number is
    begin
        return to_fixed_rounded(-state.integral_v);
    end function;

    function to_real(state : observer_fixed_record) return observer_real_record is
        -- 64 bits to real: the high and the low 32
        function wide_to_real(value : product_number) return real is
        begin
            return (real(to_integer(value(63 downto 32))) * 2.0**32 +
                    real(to_integer(unsigned(value(31 downto 0)) srl 1)) * 2.0 +
                    real(to_integer(unsigned(value(0 downto 0))))) / 2.0**product_radix;
        end function;
    begin
        return (
            i => to_real(state.i), v => to_real(state.v),
            i_prev => to_real(state.i_prev), v_prev => to_real(state.v_prev),
            integral_i => wide_to_real(state.integral_i), integral_v => wide_to_real(state.integral_v),
            du => to_real(state.du), iload => to_real(state.iload));
    end function;

end package body output_observer_pkg;
