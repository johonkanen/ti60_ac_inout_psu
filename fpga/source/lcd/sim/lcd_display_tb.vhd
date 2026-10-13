LIBRARY ieee  ;
    USE ieee.NUMERIC_STD.all  ;
    USE ieee.std_logic_1164.all  ;

library vunit_lib;
context vunit_lib.vunit_context;

    use work.fpga_interconnect_pkg.all;
    use work.lcd_font_pkg.all;

-- lcd_display against a model of the ILI9488's SPI input: each byte taken at SCK's rises with
-- chip select low, data/command telling commands from their parameters and the pixels. Checked:
-- the reset pulse, the set up (18-bit colour, the memory access control, sleep out, display on),
-- the draw's window (0..479 x 0..319) and every pixel of the screen against the font and the text
-- as it should be (the STOP button in its own colours): the first text, then text written through
-- the registers; the colours; the other two buttons (written areas and colours, overlapping);
-- a draw asked for during a draw following it; chip select high when idle. height: the screen's rows of
-- pixels, 320 as the panel's (the first text, the STOP button); vunit_run.py draws the tests of
-- several screens on 96 (three rows of text), each screen then 2.2 M clocks instead of 7.4 M.
entity lcd_display_tb is
  generic (runner_cfg : string; height : natural := 320);
end;

architecture vunit_simulation of lcd_display_tb is

    constant base   : natural := 1300;
    constant width  : natural := 480;
    constant scale  : natural := 2;
    constant cols   : natural := 24;
    constant rows   : natural := height / (16 * scale);
    constant clocks_a_ms : natural := 20;

    signal clock : std_logic := '0';
    signal bus_in, bus_out : fpga_interconnect_record := init_fpga_interconnect;
    signal lcd_cs, lcd_reset, lcd_dc, lcd_sck, lcd_mosi, lcd_led : std_logic;

    -- the text the screen should show
    type text_array is array (0 to cols * 10 - 1) of character;   -- the panel's 10 rows
    function blank_text return text_array is
        variable t : text_array := (others => ' ');
        constant title : string := "ti60 AC PSU";
        constant ip    : string := "IP  ...";
    begin
        for i in title'range loop t(i - 1) := title(i); end loop;
        for i in ip'range loop t(2 * cols + i - 1) := ip(i); end loop;
        t(8 * cols + 18 to 8 * cols + 21) := "STOP";
        return t;
    end function;
    signal expected_text : text_array := blank_text;
    signal foreground : std_logic_vector(23 downto 0) := x"FFFFFF";
    signal background : std_logic_vector(23 downto 0) := x"000000";
    -- the buttons' areas and colours: 0 the STOP button, 1 and 2 none until written
    type button_record is record
        x0, x1, y0, y1 : natural;
        fg, bg : std_logic_vector(23 downto 0);
    end record;
    type button_array is array (0 to 2) of button_record;
    signal buttons : button_array := (
        0 => (330, 469, 236, 309, x"FFFFFF", x"C00000"),
        others => (1023, 0, 1023, 0, x"FFFFFF", x"404040"));

    -- the model's findings
    signal bytes_seen   : natural := 0;
    signal commands     : natural := 0;     -- commands seen
    signal last_command : natural := 0;
    signal pixel_format, madctl : natural := 999;
    signal sleep_out, display_on : boolean := false;
    signal window_ok    : boolean := false;
    signal frames       : natural := 0;     -- complete, correct screens
    signal pixel_errors : natural := 0;
    signal pixels       : natural := 0;
    signal reset_low_clocks : natural := 0;

begin

    clock <= not clock after 4 ns;

    dut : entity work.lcd_display
    generic map(g_base_address => base, g_half_period => 1, g_clocks_a_ms => clocks_a_ms, g_scale => scale,
                g_height => height)
    port map(clock, bus_in, bus_out, lcd_cs, lcd_reset, lcd_dc, lcd_sck, lcd_mosi, lcd_led);

    reset_pulse : process(clock)
    begin
        if rising_edge(clock) and lcd_reset = '0' then
            reset_low_clocks <= reset_low_clocks + 1;
        end if;
    end process;

    -- the panel: bytes at SCK's rises, decoded
    panel : process(lcd_sck)
        variable shift  : std_logic_vector(7 downto 0);
        variable bits   : natural := 0;
        variable params : natural := 0;
        variable window : natural := 0;     -- the window's parameters' check
        variable byte   : natural;
        variable dc     : std_logic;
        variable rgb    : std_logic_vector(23 downto 0);
        variable sub    : natural := 0;     -- a pixel's byte
        variable p, x, y, ch, glyph_row : natural;
        variable code   : natural;
        variable want   : std_logic_vector(23 downto 0);
        variable in_button : integer;
        variable errors : natural := 0;
        variable in_frame : boolean := false;
        type window_values is array (0 to 3) of natural;
        constant columns_window : window_values := (0, 0, 1, 16#DF#);
        constant pages_window   : window_values := (0, 0, (height - 1) / 256, (height - 1) mod 256);
    begin
        if rising_edge(lcd_sck) and lcd_cs = '0' then
            shift := shift(6 downto 0) & lcd_mosi;
            dc := lcd_dc;
            bits := bits + 1;
            if bits = 8 then
                bits := 0;
                byte := to_integer(unsigned(shift));
                bytes_seen <= bytes_seen + 1;
                if dc = '0' then
                    if in_frame then
                        in_frame := false;      -- a command in the middle of the pixels
                    end if;
                    last_command <= byte;
                    commands <= commands + 1;
                    params := 0;
                    case byte is
                        when 16#11# => sleep_out <= true;
                        when 16#29# => display_on <= true;
                        when 16#2A# => window := 0;
                        when 16#2C# =>
                            in_frame := window = 8;
                            window_ok <= window = 8;
                            p := 0; sub := 0; errors := 0;
                        when others => null;
                    end case;
                else
                    case last_command is
                        when 16#3A# => pixel_format <= byte;
                        when 16#36# => madctl <= byte;
                        when 16#2A# =>
                            if params < 4 and byte = columns_window(params) then window := window + 1; end if;
                        when 16#2B# =>
                            if params < 4 and byte = pages_window(params) then window := window + 1; end if;
                        when 16#2C# =>
                            if in_frame then
                                rgb(23 - 8 * sub downto 16 - 8 * sub) := shift;
                                sub := sub + 1;
                                if sub = 3 then
                                    sub := 0;
                                    x := p mod width;
                                    y := p / width;
                                    ch := (y / (16 * scale)) * cols + x / (10 * scale);
                                    code := character'pos(expected_text(ch));
                                    glyph_row := to_integer(unsigned(lcd_font((code - 32) * 16 + (y / scale) mod 16)));
                                    in_button := -1;
                                    for b in 2 downto 0 loop
                                        if x >= buttons(b).x0 and x <= buttons(b).x1 and
                                           y >= buttons(b).y0 and y <= buttons(b).y1 then
                                            in_button := b;
                                        end if;
                                    end loop;
                                    if in_button >= 0 then
                                        if (glyph_row / 2**(9 - (x / scale) mod 10)) mod 2 = 1 then
                                            want := buttons(in_button).fg;
                                        else
                                            want := buttons(in_button).bg;
                                        end if;
                                    elsif (glyph_row / 2**(9 - (x / scale) mod 10)) mod 2 = 1 then
                                        want := foreground;
                                    else
                                        want := background;
                                    end if;
                                    if rgb /= want then
                                        errors := errors + 1;
                                    end if;
                                    p := p + 1;
                                    pixels <= p;
                                    if p = width * height then
                                        in_frame := false;
                                        pixel_errors <= errors;
                                        if errors = 0 then
                                            frames <= frames + 1;
                                        end if;
                                    end if;
                                end if;
                            end if;
                        when others => null;
                    end case;
                    params := params + 1;
                end if;
            end if;
        end if;
    end process;

    main : process
        procedure bus_write(address : natural; data : std_logic_vector) is
        begin
            wait until rising_edge(clock);
            write_data_to_address(bus_in, address, std_logic_vector(resize(unsigned(data), 32)));
            wait until rising_edge(clock);
            init_bus(bus_in);
        end procedure;

        -- a string into the text at a place, through register 4
        procedure write_text(place : natural; s : string) is
        begin
            for i in s'range loop
                bus_write(base + 4, std_logic_vector(to_unsigned((place + i - 1) * 65536 + character'pos(s(i)), 32)));
            end loop;
        end procedure;

        variable t : text_array;
        variable frames_before : natural;
        type natural_list is array (0 to 5) of natural;
        constant button_values_1 : natural_list := (10, 179, 36, 75, 16#FFFFFF#, 16#008000#);
        constant button_values_2 : natural_list := (170, 319, 36, 75, 16#FFFF00#, 16#0050C0#);
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 100 ns;

        while test_suite loop
            if run("sets_up_and_draws_the_first_text") then
                wait until frames = 1 or pixel_errors > 0 for 200 ms;
                info("the reset low " & integer'image(reset_low_clocks) & " clocks, " &
                     integer'image(commands) & " commands, pixel errors " & integer'image(pixel_errors));
                check(reset_low_clocks >= 9 * clocks_a_ms, "the reset low about 10 ms (the panel needs 10 us)");
                check_equal(pixel_format, 16#66#, "18 bits a pixel");
                check_equal(madctl, 16#28#, "landscape");
                check(sleep_out and display_on, "sleep out, display on");
                check(window_ok, "the window 0..479 x 0..height - 1");
                check_equal(pixel_errors, 0, "every pixel as the text and the font say");
                check_equal(frames, 1, "one screen");
                wait for 1 us;
                check(lcd_cs = '1', "chip select high when idle");
                check(lcd_led = '1', "the backlight on");

            elsif run("draws_written_text_in_its_colours") then
                wait until frames = 1 for 200 ms;
                t := blank_text;
                for i in 0 to 6 loop t(2 * cols + i) := ' '; end loop;
                t(2 * cols to 2 * cols + 16) := "IP 192.168.20.11 ";
                t((rows - 1) * cols + 20 to (rows - 1) * cols + 23) := "~{|}";
                write_text(2 * cols, "IP 192.168.20.11 ");
                write_text((rows - 1) * cols + 20, "~{|}");
                bus_write(base + 1, x"00FF00");
                bus_write(base + 2, x"102030");
                expected_text <= t;
                foreground <= x"00FF00";
                background <= x"102030";
                bus_write(base, x"02");
                wait until frames = 2 or pixel_errors > 0 for 200 ms;
                check_equal(pixel_errors, 0, "the new text, every pixel");
                check_equal(frames, 2, "a second screen");

            elsif run("draws_the_other_buttons") then
                -- buttons 1 and 2 written (the panel's START and OUTPUT), their labels in them,
                -- button 1 overlapping 2 by ten pixels: there button 1's colours
                wait until frames = 1 for 200 ms;
                t := blank_text;
                t(cols + 2 to cols + 6) := "START";
                t(cols + 9 to cols + 14) := "OUTPUT";
                write_text(cols + 2, "START");
                write_text(cols + 9, "OUTPUT");
                for r in 0 to 5 loop
                    bus_write(1360 + r, std_logic_vector(to_unsigned(button_values_1(r), 24)));
                    bus_write(1366 + r, std_logic_vector(to_unsigned(button_values_2(r), 24)));
                end loop;
                expected_text <= t;
                buttons(1) <= (10, 179, 36, 75, x"FFFFFF", x"008000");
                buttons(2) <= (170, 319, 36, 75, x"FFFF00", x"0050C0");
                bus_write(base, x"02");
                wait until frames = 2 or pixel_errors > 0 for 200 ms;
                check_equal(pixel_errors, 0, "the buttons, every pixel");
                check_equal(frames, 2, "a second screen");

            elsif run("a_draw_asked_for_during_a_draw_follows_it") then
                wait until frames = 1 for 200 ms;
                bus_write(base, x"02");
                wait until pixels > 1000 for 10 ms;
                frames_before := frames;
                bus_write(base, x"02");      -- during the draw
                wait until frames = frames_before + 2 for 400 ms;
                check_equal(frames, frames_before + 2, "the pending draw followed");
                wait for 1 us;
                check(lcd_cs = '1', "then idle");
            end if;
        end loop;

        test_runner_cleanup(runner);
        wait;
    end process;

    test_runner_watchdog(runner, 1 sec);

end vunit_simulation;
