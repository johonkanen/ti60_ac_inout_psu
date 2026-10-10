library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fixed_dsp_pkg.all;
    use work.fixed_format_pkg.all;
    use work.pi_controller_pkg.all;

-- A PI controller in the FPGA's fixed point format (32 bit, 20 fractional bits). On each
-- request:
--
--   error      = reference - measurement
--   integrator = integrator + ki * error,      kept within limit_low .. limit_high (the
--                                               anti-windup)
--   output     = kp * error + integrator,      kept within limit_low .. limit_high
--
-- the two multiply-adds one after the other on a fixed_dsp of its own; ready pulses with the
-- output (2 x the fixed_dsp's latency and a few clocks after the request). A request while one
-- is being computed is taken once that one is done. reset clears the integrator and the output.
-- The gains and the limits come with each request, so they can change at run time.
entity pi_controller is
    generic(
        g_product_register : boolean := true    -- fixed_dsp's, for 32 x 32 at the clock
    );
    port(
        clock  : in std_logic;
        pi_in  : in pi_controller_in_record;
        pi_out : out pi_controller_out_record := pi_controller_out_init
    );
end entity pi_controller;

architecture rtl of pi_controller is

    subtype dsp_in_record  is fixed_dsp_in_record(a(31 downto 0), d(31 downto 0), b(31 downto 0), c(63 downto 0));
    subtype dsp_out_record is fixed_dsp_out_record(result(63 downto 0));
    signal dsp_in  : dsp_in_record := init_fixed_dsp_in(32);
    signal dsp_out : dsp_out_record;

    type states is (idle, integrating, adding);
    signal state      : states := idle;
    signal requested  : boolean := false;    -- a request to take when idle
    signal error_r    : fixed_number := fixed_zero;
    signal integrator : fixed_number := fixed_zero;
    signal high_r     : fixed_number := fixed_zero;
    signal low_r      : fixed_number := fixed_zero;
    signal kp_r       : fixed_number := fixed_zero;

    -- a value at the product's radix (40) to the format's radix (20), as a 64 bit value
    function to_wide(v : fixed_number) return signed is
    begin
        return shift_left(resize(v, 64), fixed_radix);
    end function;

    -- the product's 40 fractional bits back to 20, within low .. high
    function limited(product : signed(63 downto 0); high, low : fixed_number) return fixed_number is
        constant shifted : signed(63 downto 0) := shift_right(product, fixed_radix);
    begin
        if shifted > resize(high, 64) then
            return high;
        elsif shifted < resize(low, 64) then
            return low;
        end if;
        return shifted(fixed_wordlength-1 downto 0);
    end function;

    -- a - b in the format, saturated
    function difference(a, b : fixed_number) return fixed_number is
        constant d : signed(fixed_wordlength downto 0) := resize(a, fixed_wordlength+1) - resize(b, fixed_wordlength+1);
    begin
        if d > resize(fixed_max, fixed_wordlength+1) then
            return fixed_max;
        elsif d < resize(fixed_min, fixed_wordlength+1) then
            return fixed_min;
        end if;
        return d(fixed_wordlength-1 downto 0);
    end function;

begin

    u_fixed_dsp : entity work.fixed_dsp(rtl)
    generic map(g_pre_add_register => false, g_product_register => g_product_register)
    port map(clock => clock, fixed_dsp_in => dsp_in, fixed_dsp_out => dsp_out);

    process(clock)
        variable e   : fixed_number;
        variable i   : fixed_number;
        variable out_v : fixed_number;
    begin
        if rising_edge(clock) then
            init_fixed_dsp(dsp_in);
            pi_out.ready <= false;
            if pi_in.request then
                requested <= true;
            end if;

            if pi_in.reset then
                state      <= idle;
                requested  <= false;
                integrator <= fixed_zero;
                pi_out.output    <= fixed_zero;
                pi_out.saturated <= false;
            else
                case state is
                    when idle =>
                        if pi_in.request or requested then
                            requested <= false;
                            e := difference(pi_in.reference, pi_in.measurement);
                            error_r <= e;
                            kp_r    <= pi_in.kp;
                            high_r  <= pi_in.limit_high;
                            low_r   <= pi_in.limit_low;
                            -- integrator + ki * error
                            add(dsp_in, a => e, b => pi_in.ki, c => to_wide(integrator));
                            state <= integrating;
                        end if;

                    when integrating =>
                        if dsp_out.ready_with_1 = '1' then
                            i := limited(dsp_out.result, high_r, low_r);
                            integrator <= i;
                            -- kp * error + integrator
                            add(dsp_in, a => error_r, b => kp_r, c => to_wide(i));
                            state <= adding;
                        end if;

                    when adding =>
                        if dsp_out.ready_with_1 = '1' then
                            out_v := limited(dsp_out.result, high_r, low_r);
                            pi_out.output    <= out_v;
                            pi_out.saturated <= out_v = high_r or out_v = low_r;
                            pi_out.ready     <= true;
                            state <= idle;
                        end if;
                end case;
            end if;
        end if;
    end process;

end architecture rtl;
