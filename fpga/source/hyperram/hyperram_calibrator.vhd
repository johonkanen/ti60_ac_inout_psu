library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;

-- Read clock calibration of hyperram_ddr_controller, between the user and the controller.
--
-- After power up (and on recalibrate) a known pattern is written to g_pattern_words words at
-- g_address, then read back with every read clock phase and both orders of the DDIO input
-- halves. The middle of the widest window of error free phases is used. Meanwhile the
-- user sees the controller as not ready. The writes do not depend on the read clock, so
-- the pattern is written once.
--
-- status: bit0 calibrated, bit1 no error free setting was found (the default is used),
--         bits 15:8 phases that passed with the hi input half first, bits 23:16 with the lo
--         half first, bits 26:24 selected phase, bit 27 selected order (lo half first)
entity hyperram_calibrator is
    generic(
        g_pattern_words : natural := 64;
        g_address       : natural := 2**25 - 64;   -- last words of the memory
        g_settle_clocks : natural := 100;          -- after a phase change
        g_default_phase : natural := 6;
        g_default_lo_first : boolean := true
    );
    port(
        clock          : in std_logic;
        user_in        : in hyperram_in_record;
        user_out       : out hyperram_out_record;
        controller_in  : out hyperram_in_record;
        controller_out : in hyperram_out_record;

        recalibrate     : in boolean;
        manual_set      : in boolean;
        manual_phase    : in natural range 0 to 7;
        manual_lo_first : in boolean;

        read_phase      : out natural range 0 to 7;
        input_lo_first  : out boolean;
        status          : out std_logic_vector(31 downto 0)
    );
end entity hyperram_calibrator;

architecture rtl of hyperram_calibrator is

    type states is (wait_for_controller, write_pattern, wait_write, set_phase, settle, read_pattern, wait_read, select_setting, done);
    signal state : states := wait_for_controller;

    signal phase        : natural range 0 to 7 := g_default_phase;
    signal lo_first     : boolean := g_default_lo_first;
    signal setting      : natural range 0 to 16 := 0;     -- lo first = setting/8, phase = setting mod 8
    signal timer        : natural range 0 to g_settle_clocks := 0;
    signal word_index   : natural range 0 to g_pattern_words := 0;
    signal read_ok      : boolean := true;
    signal pass_map     : std_logic_vector(15 downto 0) := (others => '0');
    signal calibrated   : boolean := false;
    signal failed       : boolean := false;
    signal request      : hyperram_in_record := init_hyperram_in;

    function pattern(i : natural) return std_logic_vector is
        variable v : unsigned(15 downto 0);
    begin
        -- every bit toggles between neighbouring words, with varying patterns
        v := to_unsigned((i * 40503 + 12345) mod 65536, 16);
        if i mod 2 = 1 then
            v := not v;
        end if;
        return std_logic_vector(v);
    end function;

    function to_std(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

begin

    read_phase     <= phase;
    input_lo_first <= lo_first;
    status <= x"0" & to_std(lo_first) & std_logic_vector(to_unsigned(phase, 3))
              & pass_map(15 downto 8) & pass_map(7 downto 0)
              & "000000" & to_std(failed) & to_std(calibrated);

    -- the user owns the controller once calibration is done
    controller_in <= user_in when state = done else request;
    user_out <= controller_out when state = done else (
        is_ready             => false,
        write_data_requested => false,
        read_data            => (others => '0'),
        read_data_is_ready   => false,
        transaction_is_done  => false,
        read_timed_out       => false);

    process(clock)
        procedure select_widest_window is
            variable best_length, best_start, length_now, start_now : natural := 0;
            variable best_order : natural := 0;
        begin
            for order in 0 to 1 loop
                length_now := 0;
                for p in 0 to 7 loop
                    if pass_map(8*order + p) = '1' then
                        if length_now = 0 then start_now := p; end if;
                        length_now := length_now + 1;
                        if length_now > best_length then
                            best_length := length_now;
                            best_start  := start_now;
                            best_order  := order;
                        end if;
                    else
                        length_now := 0;
                    end if;
                end loop;
            end loop;
            if best_length = 0 then
                phase    <= g_default_phase;
                lo_first <= g_default_lo_first;
                failed   <= true;
            else
                phase    <= best_start + best_length/2;
                lo_first <= best_order = 1;
                failed   <= false;
            end if;
        end procedure;
    begin
        if rising_edge(clock) then
            init_hyperram(request);
            request.latency_count <= user_in.latency_count;

            CASE state is
                WHEN wait_for_controller =>
                    if hyperram_is_ready(controller_out) then
                        word_index <= 0;
                        request_hyperram_write(request, g_address, g_pattern_words);
                        state <= write_pattern;
                    end if;

                WHEN write_pattern =>
                    if hyperram_write_data_is_requested(controller_out) then
                        send_hyperram_write_data(request, pattern(word_index));
                        word_index <= word_index + 1;
                    end if;
                    if hyperram_transaction_is_done(controller_out) then
                        setting  <= 0;
                        pass_map <= (others => '0');
                        state    <= set_phase;
                    end if;

                WHEN set_phase =>
                    phase    <= setting mod 8;
                    lo_first <= setting >= 8;
                    timer    <= 0;
                    state    <= settle;

                WHEN settle =>
                    if timer < g_settle_clocks then
                        timer <= timer + 1;
                    elsif hyperram_is_ready(controller_out) then
                        word_index <= 0;
                        read_ok    <= true;
                        request_hyperram_read(request, g_address, g_pattern_words);
                        state <= wait_read;
                    end if;

                WHEN wait_read =>
                    if hyperram_read_data_is_ready(controller_out) then
                        if get_hyperram_read_data(controller_out) /= pattern(word_index) then
                            read_ok <= false;
                        end if;
                        word_index <= word_index + 1;
                    end if;
                    if hyperram_transaction_is_done(controller_out) then
                        -- all words read, none wrong and in time
                        if read_ok and word_index = g_pattern_words and not controller_out.read_timed_out then
                            pass_map(setting) <= '1';
                        end if;
                        if setting = 15 then
                            state <= select_setting;
                        else
                            setting <= setting + 1;
                            state   <= set_phase;
                        end if;
                    end if;

                WHEN select_setting =>
                    select_widest_window;
                    calibrated <= true;
                    state      <= done;

                WHEN done =>
                    if manual_set then
                        phase    <= manual_phase;
                        lo_first <= manual_lo_first;
                    end if;
                    if recalibrate and hyperram_is_ready(controller_out) and user_in.command = none then
                        calibrated <= false;
                        state      <= wait_for_controller;
                    end if;

                WHEN others => null;
            end CASE;
        end if;
    end process;

end architecture rtl;
