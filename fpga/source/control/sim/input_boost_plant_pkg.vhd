library ieee;
    use ieee.math_real.all;

    use work.ode_pkg.all;

-- The input boost's power stage for the testbenches, integrated with hVHDL_ode's rk4 a
-- switching period at a time:
--
-- boost_plant_period   the converter: a DC source (vs) through the input inductance (l) into the
--                      link's capacitance (c) with a load (i_load). While the switch is on the
--                      inductor's current rises at vs / l; after it falls at (v - vs) / l into
--                      the link until it is back at zero (DCM; the diodes block it below), the
--                      link meanwhile carrying the load. Below the source the diodes conduct:
--                      the link stays at least at the source.
-- fitted_plant_period  the board as a capture showed it (2026-10-13, from a 60 V source): the
--                      link a first order lag to v0 + k * duty, tau (55.2 V, 62.7 V a unit of
--                      duty, 42 ms); c_scale multiplies tau (the DHB's capacitance doubling the
--                      link's: 2.0).
package input_boost_plant_pkg is

    type boost_plant is record
        vs     : real;      -- the source, V
        l      : real;      -- the input inductance, H
        c      : real;      -- the link's capacitance, F
        il     : real;      -- the inductor's current, A
        v      : real;      -- the link, V
        charge : real;      -- the period's charge into the link, C
        i_mean : real;      -- the inductor's mean current over the period, A
    end record;

    -- the testbench's converter: 94.6 V (the lab supply's 98.6 V less the diodes), 15 uH, 1.3 mF
    constant boost_plant_init : boost_plant := (
        vs => 94.6, l => 15.0e-6, c => 1.3e-3, il => 0.0, v => 94.6, charge => 0.0, i_mean => 0.0);

    procedure boost_plant_period(
        plant  : inout boost_plant;
        t_on   : real;          -- s
        i_load : real;          -- A, from the link
        t_period : real := 25.0e-6);

    type fitted_plant is record
        v0      : real;
        k       : real;
        tau     : real;
        c_scale : real;
        v       : real;
    end record;

    constant fitted_plant_init : fitted_plant := (
        v0 => 55.2, k => 62.7, tau => 42.0e-3, c_scale => 1.0, v => 55.2);

    procedure fitted_plant_period(
        plant : inout fitted_plant;
        duty  : real;
        t_period : real := 25.0e-6);

end package input_boost_plant_pkg;

package body input_boost_plant_pkg is

    procedure boost_plant_period(
        plant  : inout boost_plant;
        t_on   : real;
        i_load : real;
        t_period : real := 25.0e-6) is

        -- the circuit's state (0 => the inductor's current, 1 => the link) in each interval:
        -- the switch on, the inductor conducting into the link, or neither
        type intervals is (switch_on, into_link, blocked);
        variable interval : intervals := switch_on;
        variable vs : real := plant.vs;
        variable l  : real := plant.l;
        variable c  : real := plant.c;

        impure function deriv(t : real; s : real_vector) return real_vector is
            variable retval : real_vector(0 to 1);
        begin
            case interval is
                when switch_on =>
                    retval(0) := vs / l;
                    retval(1) := -i_load / c;
                when into_link =>
                    retval(0) := (vs - s(1)) / l;
                    retval(1) := (s(0) - i_load) / c;
                when blocked =>
                    retval(0) := 0.0;
                    retval(1) := -i_load / c;
            end case;
            return retval;
        end function;

        procedure rk4 is new generic_rk4 generic map(deriv);

        variable s : real_vector(0 to 1) := (plant.il, plant.v);
        variable t_off, t_zero, area, v0 : real;
    begin
        v0 := plant.v;
        t_off := t_period - t_on;
        -- on: the current builds
        if t_on > 0.0 then
            interval := switch_on;
            rk4(0.0, s, t_on);
        end if;
        area := (plant.il + s(0)) / 2.0 * t_on;
        -- off: into the link until the current is back at zero, then blocked
        if s(1) > vs and s(0) > 0.0 then
            t_zero := s(0) * l / (s(1) - vs);
        else
            t_zero := t_off;
        end if;
        if s(1) <= vs then
            -- the link under the source: the inductor's current flows on (CCM through the diodes)
            interval := into_link;
            rk4(t_on, s, t_off);
        elsif s(0) <= 0.0 then
            -- no current and the link above the source: the diodes block the whole interval
            s(0) := 0.0;
            interval := blocked;
            rk4(t_on, s, t_off);
        elsif t_zero >= t_off then
            interval := into_link;
            rk4(t_on, s, t_off);
        else
            interval := into_link;
            rk4(t_on, s, t_zero);
            s(0) := 0.0;
            interval := blocked;
            rk4(t_on + t_zero, s, t_off - t_zero);
        end if;
        if s(0) < 0.0 then
            s(0) := 0.0;
        end if;
        -- below the source the diodes conduct: the link at least the source's
        if s(1) < vs then
            s(1) := vs;
        end if;
        plant.charge := (s(1) - v0) * c + i_load * t_period;
        plant.i_mean := (area + plant.charge) / t_period;
        plant.il := s(0);
        plant.v  := s(1);
    end procedure;

    procedure fitted_plant_period(
        plant : inout fitted_plant;
        duty  : real;
        t_period : real := 25.0e-6) is

        variable v0  : real := plant.v0;
        variable k   : real := plant.k;
        variable tau : real := plant.tau * plant.c_scale;

        impure function deriv(t : real; s : real_vector) return real_vector is
            variable retval : real_vector(0 to 0);
        begin
            retval(0) := (v0 + k * duty - s(0)) / tau;
            return retval;
        end function;

        procedure rk4 is new generic_rk4 generic map(deriv);

        variable s : real_vector(0 to 0) := (0 => plant.v);
    begin
        rk4(0.0, s, t_period);
        plant.v := s(0);
    end procedure;

end package body input_boost_plant_pkg;
