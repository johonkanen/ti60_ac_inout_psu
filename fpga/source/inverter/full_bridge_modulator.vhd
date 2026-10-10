library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fixed_dsp_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dhb_modulator_pkg.all;
    use work.full_bridge_modulator_pkg.all;

-- The output bridge's modulator: on each request (the carrier's, shortly before a period ends)
-- the next period's switch times from the period and the duty, duty * period on a fixed_dsp;
-- the result is full_bridge_modulator_pkg's get_switch_times, which an ODE simulation calls
-- directly. Requests may come every clock (the fixed_dsp is pipelined): each gives its times
-- a few clocks later, with modulation_ready. The period is quasi static (a change takes effect
-- with the times computed after it).
--
--   in   modulation_requested - strobe: compute the next period's switch times
--   in   period               - carrier period in clock counts
--   in   duty                 - 0 .. 1 of leg 1's high side (kept within)
--   out  modulation_ready     - pulses the clock switch_times is new
--   out  switch_times         - for the carrier
entity full_bridge_modulator is
    port(
        clock                : in std_logic;
        modulation_requested : in boolean;
        period               : in dhb_count;
        duty                 : in fixed_number;
        modulation_ready     : out boolean;
        switch_times         : out fb_switch_times
    );
end entity full_bridge_modulator;

architecture rtl of full_bridge_modulator is

    subtype dsp_in_record  is fixed_dsp_in_record(a(31 downto 0), d(31 downto 0), b(31 downto 0), c(63 downto 0));
    subtype dsp_out_record is fixed_dsp_out_record(result(63 downto 0));
    signal dsp_in  : dsp_in_record := init_fixed_dsp_in(32);
    signal dsp_out : dsp_out_record;


begin

    u_fixed_dsp : entity work.fixed_dsp(rtl)
    generic map(g_pre_add_register => false, g_product_register => true)
    port map(clock => clock, fixed_dsp_in => dsp_in, fixed_dsp_out => dsp_out);

    process(clock)
    begin
        if rising_edge(clock) then
            init_fixed_dsp(dsp_in);
            modulation_ready <= false;
            if modulation_requested then
                add(dsp_in, a => limit_duty(duty), b => to_signed(period, 32), c => to_signed(0, 64));
            end if;
            if dsp_out.ready_with_1 = '1' then
                -- 20 fractional bits off: whole counts
                switch_times     <= (period => period,
                                     leg1_off => to_integer(shift_right(dsp_out.result, fixed_radix)(16 downto 0)));
                modulation_ready <= true;
            end if;
        end if;
    end process;

end architecture rtl;
