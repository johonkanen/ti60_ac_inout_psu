LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fixed_format_pkg.all;

-- voltage_modulator: duty = (1 + Vref / Vdc) / 2 within the lookup table divider's accuracy
-- over references and DC voltages of the board's range, either sign; the limits at +-Vdc;
-- duty 0.5 for a DC voltage too low to divide by; the latency.
entity voltage_modulator_tb is
  generic (runner_cfg : string);
end;

architecture vunit_simulation of voltage_modulator_tb is

    signal clock      : std_logic := '0';
    signal request    : boolean := false;
    signal reference  : fixed_number := fixed_zero;
    signal dc_voltage : fixed_number := fixed_zero;
    signal duty       : fixed_number;
    signal limited    : boolean;
    signal ready      : boolean;

begin

    clock <= not clock after 4 ns;

    dut : entity work.voltage_modulator
    port map(clock, request, reference, dc_voltage, duty, limited, ready);

    main : process
        variable clocks : natural;

        procedure modulate(vref, vdc : real) is
        begin
            wait until rising_edge(clock);
            reference  <= to_fixed(vref);
            dc_voltage <= to_fixed(vdc);
            request    <= true;
            wait until rising_edge(clock);
            request <= false;
            clocks  := 1;
            while not ready loop
                wait until rising_edge(clock);
                clocks := clocks + 1;
                check(clocks < 30, "a duty within 30 clocks (16)");
            end loop;
        end procedure;

        function real_of(f : fixed_number) return real is
        begin
            return real(to_integer(f)) / 2.0**fixed_radix;
        end function;

        variable seed1, seed2 : positive := 7;
        variable r1, r2, vdc, vref, expected, worst : real;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("duty_from_the_reference") then
                worst := 0.0;
                for k in 1 to 500 loop
                    uniform(seed1, seed2, r1);
                    uniform(seed1, seed2, r2);
                    vdc  := 20.0 + 430.0 * r1;           -- 20 .. 450 V
                    vref := (2.0 * r2 - 1.0) * vdc;      -- within +-Vdc
                    modulate(vref, vdc);
                    expected := 0.5 + 0.5 * vref / vdc;
                    worst := maximum(worst, abs(real_of(duty) - expected));
                    check(abs(real_of(duty) - expected) < 1.0e-4,
                        "duty " & real'image(real_of(duty)) & " for " & real'image(vref) & " V of " &
                        real'image(vdc) & " V, expected " & real'image(expected));
                    check(not limited, "within the DC voltage: not limited");
                end loop;
                info("worst duty error " & real'image(worst));
                modulate(0.0, 94.3);
                check_equal(real_of(duty), 0.5, "0 V: duty 0.5");
                modulate(18.86, 94.3);
                check(abs(real_of(duty) - 0.6) < 1.0e-4, "the board's step: 18.86 V of 94.3 V is 0.6");

            elsif run("limited_at_the_dc_voltage") then
                modulate(150.0, 94.3);
                check_equal(real_of(duty), 1.0, "150 V of 94.3 V: duty 1");
                check(limited, "limited");
                modulate(-200.0, 94.3);
                check_equal(real_of(duty), 0.0, "-200 V: duty 0");
                check(limited, "limited");
                modulate(50.0, 94.3);
                check(not limited, "50 V: not limited");

            elsif run("no_dc_voltage_no_output") then
                modulate(10.0, 0.0);
                check_equal(real_of(duty), 0.5, "0 V DC link: duty 0.5");
                modulate(2.0, 4.0);
                check_equal(real_of(duty), 0.5, "under 5 V: duty 0.5");
                modulate(10.0, -94.3);
                check_equal(real_of(duty), 0.5, "a negative DC link: duty 0.5");
                modulate(10.0, 94.3);
                check(abs(real_of(duty) - (0.5 + 5.0 / 94.3)) < 1.0e-4, "back with the DC link");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end vunit_simulation;
