-- The output bridge's switching period, everything that depends on it in one place: the
-- modulator (output_bridge_test), the sigma-delta channels' period mean (sdm_adc's window), the
-- observer's step (half a period) and the control's (a period).
--
-- The period is a whole number of the 125 MHz clock's 8 ns and of the sigma-delta modulators'
-- 50 ns bits, so the period mean takes exactly one period: a multiple of 200 ns. 25 us: 40 kHz,
-- 3125 clocks, 500 bits (was 45.4 us, 22.03 kHz, 5675 clocks, 908 bits).
package output_bridge_pkg is

    constant output_period_bits   : natural := 500;
    constant output_period_clocks : natural := 3125;
    constant output_period        : real    := 25.0e-6;     -- s
    constant output_step          : real    := output_period / 2.0;   -- the observer's

end package output_bridge_pkg;
