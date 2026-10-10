LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;
    use ieee.math_real.all;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.output_bridge_pkg.all;

-- sdm_adc with three second order sigma-delta modulator models, each with its own constant
-- input (and a switching ripple, a triangle of a period's bits, when ripple is set), their data
-- output_delay_ns after the modulator clock's rising edge as the isolated modulators of the
-- board. For 4 ns either side of a change the data is random, as a bit read near its edge.
entity sdm_adc_tb is
  generic (runner_cfg : string; output_delay_ns : real := 20.0);
end;

architecture vunit_simulation of sdm_adc_tb is

    constant base     : natural := 740;
    constant channels : natural := 3;
    type real_array is array (natural range <>) of real;
    constant inputs : real_array(0 to channels-1) := (0.5, -0.3, 0.0123);

    signal clock : std_logic := '0';
    signal adc_clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal bus_read_data : std_logic_vector(31 downto 0);
    signal bus_read_done : boolean := false;

    signal results   : scope_sample_array(0 to channels-1);
    signal means     : scope_sample_array(0 to channels-1);
    signal ripple    : real := 0.0;   -- the triangle's amplitude, of full scale
    -- a ramp instead of the constant inputs, from -0.4 full scale at ramp_start
    signal ramp_start : time := 0 ns;
    signal ramping    : boolean := false;
    constant ramp_rate : real := 2000.0;  -- full scales per second: 0.2 in 100 us
    signal sample        : boolean := false;
    signal sampled       : scope_sample_array(0 to channels-1);
    signal sampled_ready : boolean;
    signal marker        : std_logic := '0';
    -- a switching spike: the marker toggling every half period (250 bits), channel 1's input
    -- +0.8 for two bits six bits after each edge, as the board's spikes after its gate edges
    signal spikes        : boolean := false;
    -- the modulator's period starts: 60 bits after the ripple's start (its triangle rising the
    -- first half, as the board's current while leg 1 is high): the middles at bits 65 and 315
    signal period_start   : boolean := false;
    signal start_toggle   : std_logic := '0';
    signal start_seen     : std_logic := '0';
    signal midpoint       : scope_sample;
    signal midpoint_ready : boolean;
    signal raw_bits      : scope_sample;
    signal is_new    : boolean;
    signal sdm_clock : std_logic;
    signal sdm_data  : std_logic_vector(0 to channels-1) := (others => '0');

begin

    clock     <= not clock after 4 ns;
    adc_clock <= not adc_clock after 2.5 ns;

    u_sdm : entity work.sdm_adc
    generic map(g_base_address => base, g_channels => channels)
    port map(clock, bus_in, bus_out, results, means, is_new, sample, sampled, sampled_ready,
             marker, raw_bits, period_start, midpoint, midpoint_ready, adc_clock, sdm_clock, sdm_data);

    -- second order modulators (TI, How delta-sigma ADCs work, part 1): a bit each rising edge
    modulators : for c in 0 to channels-1 generate
        process(sdm_clock)
            variable integral1, integral2 : real := 0.0;
            variable y : real := -1.0;
            variable seed1, seed2 : positive := 1 + c;
            variable r : real;
            variable noise, data : std_logic;
            variable bits : natural := 0;
            variable input, position : real;
        begin
            if rising_edge(sdm_clock) then
                -- a triangle over a period's bits, the window: -1..1 times ripple
                position := real(bits mod output_period_bits) / real(output_period_bits);
                input    := inputs(c) + ripple * (1.0 - 4.0 * abs(position - 0.5));
                if spikes and c = 1 and (bits mod 250 = 106 or bits mod 250 = 107) then
                    input := input + 0.8;
                end if;
                if ramping then
                    input := -0.4 + ramp_rate * real((now - ramp_start) / 1 ns) * 1.0e-9 +
                             ripple * (1.0 - 4.0 * abs(position - 0.5));
                end if;
                bits     := bits + 1;
                integral1 := integral1 + input - y;
                integral2 := integral2 + integral1 - y;
                if integral2 > 0.0 then
                    y := 1.0;
                    data := '1';
                else
                    y := -1.0;
                    data := '0';
                end if;
                uniform(seed1, seed2, r);
                if r > 0.5 then noise := '1'; else noise := '0'; end if;
                sdm_data(c) <= transport noise after (output_delay_ns - 4.0) * 1 ns;
                sdm_data(c) <= transport data after (output_delay_ns + 4.0) * 1 ns;
            end if;
        end process;
    end generate;

    -- a period's start where the ripple's triangle starts (bits mod a period = 0), a clock's pulse
    starts : process(sdm_clock)
        variable bits : natural := 0;
    begin
        if rising_edge(sdm_clock) then
            if bits mod output_period_bits = 60 then
                start_toggle <= not start_toggle;
            end if;
            bits := bits + 1;
        end if;
    end process;

    start_pulses : process(clock)
    begin
        if rising_edge(clock) then
            start_seen   <= start_toggle;
            period_start <= start_seen /= start_toggle;
        end if;
    end process;

    marker_process : process(sdm_clock)
        variable bits : natural := 0;
    begin
        if rising_edge(sdm_clock) then
            if spikes and bits mod 250 = 100 then
                marker <= not marker;
            end if;
            bits := bits + 1;
        end if;
    end process;

    capture_bus_reads : process(clock)
    begin
        if rising_edge(clock) then
            bus_read_done <= false;
            if write_to_address_is_requested(bus_out, 0) then
                bus_read_data <= get_slv_data(bus_out);
                bus_read_done <= true;
            end if;
        end if;
    end process;

    main : process
        procedure bus_write(address : natural; data : natural) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(to_unsigned(data, 32)));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        procedure bus_read(address : natural; data : out integer) is
        begin
            wait until rising_edge(clock);
            request_data_from_address(bus_in, address);
            wait until rising_edge(clock);
            init_bus(bus_in);
            wait until rising_edge(clock) and bus_read_done for 1 us;
            check(bus_read_done, "no response from " & integer'image(address));
            data := to_integer(signed(bus_read_data));
        end procedure;

        -- the largest error of the channels' results (16 bit full scale) against their inputs,
        -- in full scales
        impure function worst_error(log2 : natural; values : integer_vector) return real is
            variable worst : real := 0.0;
        begin
            for c in 0 to channels-1 loop
                worst := realmax(worst, abs(real(values(c)) / 32768.0 - inputs(c)));
            end loop;
            return worst;
        end function;

        -- the filtered results (offset 4) or the period means (offset 8)
        procedure read_results(values : out integer_vector; offset : natural := 4) is
            variable v : integer;
        begin
            for c in 0 to channels-1 loop
                bus_read(base + offset + c, v);
                values(c) := v;
            end loop;
        end procedure;

        variable low, high : integer_vector(0 to channels-1);
        variable started : time;
        variable expected, delay : real;
        variable worst_mean_error : real;

        variable v, first_count : integer;
        variable values : integer_vector(0 to channels-1);
        variable error  : real;
        variable window : std_logic_vector(0 to 9);
        variable passing : natural;
    begin
        test_runner_setup(runner, runner_cfg);

        while test_suite loop
            if run("converts_at_the_default_decimation") then
                -- 20 MHz, R = 64: a result every 3.2 us, the filter full after three
                wait for 20 us;
                read_results(values);
                error := worst_error(6, values);
                info("R 64: worst error " & real'image(error) & " of full scale");
                check(error < 0.003, "results within 0.3 % of the inputs");
                for c in 0 to channels-1 loop
                    check(abs(to_integer(signed(results(c))) - integer(inputs(c) * 32768.0)) < 100,
                          "scope probe " & integer'image(c) & " scaled to 16 bits");
                end loop;
                bus_read(base + 3, first_count);
                wait for 32 us;
                bus_read(base + 3, v);
                check(v - first_count >= 9 and v - first_count <= 11, "a result every 3.2 us");

            elsif run("converts_at_every_decimation") then
                for log2 in 5 to 8 loop
                    bus_write(base + 0, 16#001# + log2 * 256);
                    wait for 3.5 * 2.0**log2 * 0.05 us + 10 us;
                    read_results(values);
                    error := worst_error(log2, values);
                    info("R " & integer'image(2**log2) & ": worst error " & real'image(error));
                    check(error < 0.006, "results within 0.6 % at R " & integer'image(2**log2));
                end loop;

            elsif run("the_period_mean_removes_the_ripple") then
                -- a ripple of 2 % of full scale (+-2.56 A of 128 A), a triangle of a period
                ripple <= 0.02;
                wait for 150 us;
                -- the filtered results show it: their spread over a few periods
                low  := (others => integer'high);
                high := (others => integer'low);
                worst_mean_error := 0.0;
                for k in 1 to 60 loop
                    read_results(values, 4);
                    for c in 0 to channels-1 loop
                        low(c)  := minimum(low(c), values(c));
                        high(c) := maximum(high(c), values(c));
                    end loop;
                    read_results(values, 8);
                    for c in 0 to channels-1 loop
                        worst_mean_error := realmax(worst_mean_error, abs(real(values(c)) / 32768.0 - inputs(c)));
                    end loop;
                    wait for 2.3 us;
                end loop;
                for c in 0 to channels-1 loop
                    info("channel " & integer'image(c) & ": filtered " & integer'image(low(c)) & " .. " &
                         integer'image(high(c)) & ", input " & integer'image(integer(inputs(c) * 32768.0)));
                    check(real(high(c) - low(c)) / 32768.0 > 0.03, "the filtered results show the ripple");
                end loop;
                info("worst period mean error " & real'image(worst_mean_error) & " of full scale");
                check(worst_mean_error < 0.0005, "the period means within 0.05 % of the inputs: no ripple");
                -- a probe too
                check(abs(to_integer(signed(means(0))) - integer(inputs(0) * 32768.0)) < 20, "the mean's scope probe");

            elsif run("samples_at_the_pulses") then
                -- pulses twice a period (half its bits) on a ramp with the ripple: each
                -- sample the ramp half a period and the filter's delay before its pulse
                ripple     <= 0.02;
                ramp_start <= now;
                ramping    <= true;
                wait for 60 us;
                worst_mean_error := 0.0;
                for k in 1 to 12 loop
                    wait until rising_edge(clock);
                    sample <= true;
                    started := now;
                    wait until rising_edge(clock);
                    sample <= false;
                    wait until sampled_ready for 1 us;
                    check(sampled_ready, "a sample within 1 us of the pulse");
                    wait for 1 ns;   -- sampled through its concurrent assignment
                    if k = 1 then
                        info("sampled " & time'image(now - started) & " after the pulse");
                    end if;
                    -- the ramp's time of the value sampled, before the pulse
                    expected := -0.4 + ramp_rate * real((started - ramp_start) / 1 ns) * 1.0e-9;
                    delay := (expected - real(to_integer(signed(sampled(0)))) / 32768.0) / ramp_rate;
                    info("sample " & integer'image(k) & ": the ramp " & real'image(delay * 1.0e6) &
                         " us before the pulse");
                    -- half a period, the filter's 2.25 us, the bits' capture
                    check(delay > output_step + 1.9e-6 and delay < output_step + 2.9e-6,
                          "the ramp half a period + the filter's delay before the pulse");
                    for c in 1 to channels-1 loop
                        check(sampled(c) = sampled(0), "every channel sampled at the same bit");
                    end loop;
                    wait for output_step * 1 sec - (now - started);
                end loop;
                bus_read(base + 14, v);
                check_equal(v, 12, "12 samples counted");
                bus_read(base + 11, v);
                check_equal(v, to_integer(signed(sampled(0))), "the latest sample in register 11");

            elsif run("the_midpoint_sampler_finds_the_middles") then
                -- a 2 % ripple (a triangle of a period): the samples a half period apart at the
                -- triangle's middles, the average; tau settles
                ripple <= 0.02;
                wait for 1 ms;
                worst_mean_error := 0.0;
                for k in 1 to 40 loop
                    wait until midpoint_ready for 20 us;
                    wait for 1 ns;
                    worst_mean_error := realmax(worst_mean_error, abs(real(to_integer(signed(midpoint))) / 32768.0 - inputs(1)));
                end loop;
                bus_read(base + 18, v);
                info("tau " & real'image(real(v) / 65536.0) & " bits, worst midpoint error " &
                     real'image(worst_mean_error) & " of full scale");
                check(worst_mean_error < 0.0006, "the midpoints the average: the ripple (0.02) gone");
                -- from 172 to the rise's middle: bit 65 of the period, plus the filter's delay
                check(abs(real(v) / 65536.0 - 65.0 - 48.0) < 15.0, "tau at the rise's middle, the filter's ~48 bits on");

            elsif run("a_new_window") then
                -- half the window: the ripple no longer cancels, then the window back
                ripple <= 0.02;
                bus_write(base + 7, output_period_bits / 2);
                wait for 150 us;
                worst_mean_error := 0.0;
                for k in 1 to 30 loop
                    read_results(values, 8);
                    worst_mean_error := realmax(worst_mean_error, abs(real(values(0)) / 32768.0 - inputs(0)));
                    wait for 3.1 us;
                end loop;
                info("half the period: worst mean error " & real'image(worst_mean_error));
                check(worst_mean_error > 0.002, "a window of half the period leaves ripple");
                bus_write(base + 7, output_period_bits);
                wait for 150 us;
                read_results(values, 8);
                check(worst_error(6, values) < 0.0005, "the full window again: exact");

            elsif run("phase_window") then
                -- the phases that read clean bits for this data delay
                for phase in 0 to 9 loop
                    bus_write(base + 2, phase);
                    wait for 20 us;
                    read_results(values);
                    error := worst_error(6, values);
                    if error < 0.003 then window(phase) := '1'; else window(phase) := '0'; end if;
                end loop;
                passing := 0;
                for phase in 0 to 9 loop
                    if window(phase) = '1' then passing := passing + 1; end if;
                end loop;
                info("phases 0..9 passing: " & to_string(window));
                check(passing >= 6, "a window of at least six phases");
                check(passing < 10, "the phases at the data's edges fail");
                check(window(1) = '1', "the default phase 1 passes");

            elsif run("slower_clock") then
                -- 12.5 MHz: 16 clocks a period
                bus_write(base + 1, 16);
                bus_write(base + 2, 12);
                wait for 30 us;
                read_results(values);
                check(worst_error(6, values) < 0.003, "results at 12.5 MHz");
                bus_read(base + 3, first_count);
                wait for 51.2 us;
                bus_read(base + 3, v);
                check(v - first_count >= 9 and v - first_count <= 11, "a result every 5.12 us");

            elsif run("stops") then
                bus_write(base + 0, 16#600#);
                wait for 5 us;
                bus_read(base + 3, first_count);
                wait until sdm_clock = '1' for 10 us;
                check(sdm_clock = '0', "no modulator clock when stopped");
                bus_read(base + 3, v);
                check_equal(v, first_count, "no results when stopped");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 10 ms);

end vunit_simulation;
