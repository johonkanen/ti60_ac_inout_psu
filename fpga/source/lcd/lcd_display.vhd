library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.dual_port_ram_pkg.all;
    use work.lcd_font_pkg.all;

-- A text display on the face panel's LCD (MSP3520: a 3.5" 480 x 320 ILI9488 over 4-wire SPI,
-- written only): a screen of text, cols x rows characters of lcd_font_pkg's 10 x 16 font scaled
-- by g_scale (2: 24 x 10 characters of 20 x 32 pixels), one foreground and one background colour.
--
-- At start, and on init, the panel is reset (its reset pin low 10 ms, then 120 ms), set up (the
-- program below: 18-bit colour, the memory access control from register 5, sleep out, display on)
-- and drawn. A draw writes the whole screen: the column and page addresses 0..479 and 0..319,
-- memory write, then each pixel's three bytes (red, green, blue, their upper 6 bits used) row by
-- row, each pixel's character read from the text buffer and its glyph row from the font (both
-- block RAMs, hVHDL_memory_library's dual_port_ram). A draw asked for while busy follows the
-- current one, so text written meanwhile shows. At 12.5 MHz (g_half_period 5 clocks of 125 MHz)
-- a screen takes 0.3 s.
--
-- SPI mode 0, most significant bit first; data/command low for a command byte, high for its
-- parameters and the pixels; chip select low while a sequence runs.
--
-- register map, offsets from g_base_address
--   0  W  bit0 init (reset, set up, draw), bit1 draw
--      R  bit0 busy, bit1 set up, bit2 a draw pending
--   1  RW the foreground colour, 0xRRGGBB (white)
--   2  RW the background colour, 0xRRGGBB (black)
--   3  RW bit0 the backlight on (1)
--   4  W  a character: bits 7..0 its code, bits 23..16 its place (row * cols + col)
--   5  RW the memory access control (command 0x36) at set up (0x28: landscape)
--   6  R  screens drawn
--   7  R  the text's size: bits 7..0 cols, 15..8 rows
entity lcd_display is
    generic(
        g_base_address : natural := 1300;
        g_half_period  : natural := 5;          -- SCK's half period, clocks
        g_clocks_a_ms  : natural := 125000;
        g_scale        : natural := 2
    );
    port(
        clock    : in std_logic;
        bus_in   : in fpga_interconnect_record;
        bus_out  : out fpga_interconnect_record;
        lcd_cs    : out std_logic;
        lcd_reset : out std_logic;
        lcd_dc    : out std_logic;
        lcd_sck   : out std_logic;
        lcd_mosi  : out std_logic;
        lcd_led   : out std_logic
    );
end entity lcd_display;

architecture rtl of lcd_display is

    constant width  : natural := 480;
    constant height : natural := 320;
    constant cols   : natural := width / (lcd_font_width * g_scale);
    constant rows   : natural := height / (lcd_font_height * g_scale);

    function to_slv(n : natural) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(n, 32));
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

    ------------------------------------------------------------------------
    -- the set up and the draw's start as a program: commands, their parameters, delays
    type step_kind is (k_command, k_data, k_madctl, k_delay, k_pixels, k_end);
    type step_record is record
        kind  : step_kind;
        value : natural range 0 to 255;
    end record;
    type step_array is array (natural range <>) of step_record;

    function cmd(v : natural) return step_record is begin return (k_command, v); end function;
    function dat(v : natural) return step_record is begin return (k_data, v); end function;
    function dly(v : natural) return step_record is begin return (k_delay, v); end function;

    -- the set up (ILI9488's usual, as for the MSP3520's own examples), then a draw
    constant steps : step_array := (
        cmd(16#01#), dly(150),                                                  -- software reset
        cmd(16#E0#), dat(16#00#), dat(16#03#), dat(16#09#), dat(16#08#), dat(16#16#), dat(16#0A#), dat(16#3F#),
                   dat(16#78#), dat(16#4C#), dat(16#09#), dat(16#0A#), dat(16#08#), dat(16#16#), dat(16#1A#),
                   dat(16#0F#),                                               -- positive gamma
        cmd(16#E1#), dat(16#00#), dat(16#16#), dat(16#19#), dat(16#03#), dat(16#0F#), dat(16#05#), dat(16#32#),
                   dat(16#45#), dat(16#46#), dat(16#04#), dat(16#0E#), dat(16#0D#), dat(16#35#), dat(16#37#),
                   dat(16#0F#),                                               -- negative gamma
        cmd(16#C0#), dat(16#17#), dat(16#15#),                                    -- power control 1
        cmd(16#C1#), dat(16#41#),                                               -- power control 2
        cmd(16#C5#), dat(16#00#), dat(16#12#), dat(16#80#),                         -- VCOM
        cmd(16#36#), (k_madctl, 0),                                             -- memory access control
        cmd(16#3A#), dat(16#66#),                                               -- 18 bits a pixel
        cmd(16#B0#), dat(16#00#),                                               -- interface mode
        cmd(16#B1#), dat(16#A0#),                                               -- frame rate
        cmd(16#B4#), dat(16#02#),                                               -- inversion
        cmd(16#B6#), dat(16#02#), dat(16#02#), dat(16#3B#),                         -- display function
        cmd(16#B7#), dat(16#C6#),                                               -- entry mode
        cmd(16#F7#), dat(16#A9#), dat(16#51#), dat(16#2C#), dat(16#82#),              -- adjust control 3
        cmd(16#11#), dly(120),                                                  -- sleep out
        cmd(16#29#), dly(25),                                                   -- display on
        -- the draw
        cmd(16#2A#), dat(0), dat(0), dat((width - 1) / 256), dat((width - 1) mod 256),
        cmd(16#2B#), dat(0), dat(0), dat((height - 1) / 256), dat((height - 1) mod 256),
        cmd(16#2C#), (k_pixels, 0),
        (k_end, 0));

    function draw_start return natural is
    begin
        for i in steps'range loop
            if steps(i).kind = k_command and steps(i).value = 16#2A# then
                return i;
            end if;
        end loop;
        return 0;
    end function;

    ------------------------------------------------------------------------
    -- the text buffer: spaces, and a first line until the software writes its own
    constant text_bits : natural := 8;
    constant text_size : natural := 2**text_bits;

    function initial_text return ram_array is
        constant title : string := "ti60 AC PSU";
        constant ip    : string := "IP  ...";
        variable text  : ram_array(0 to text_size - 1)(7 downto 0) := (others => x"20");
    begin
        for i in title'range loop
            text(i - 1) := std_logic_vector(to_unsigned(character'pos(title(i)), 8));
        end loop;
        for i in ip'range loop
            text(2 * cols + i - 1) := std_logic_vector(to_unsigned(character'pos(ip(i)), 8));
        end loop;
        return text;
    end function;

    constant text_subtype : dpram_ref_record := create_ref_subtypes(datawidth => 8, addresswidth => text_bits);
    constant font_subtype : dpram_ref_record := create_ref_subtypes(datawidth => lcd_font_width, addresswidth => 11);

    signal text_a_in, text_b_in : text_subtype.ram_in'subtype;
    signal text_a_out, text_b_out : text_subtype.ram_out'subtype;
    signal font_a_in, font_b_in : font_subtype.ram_in'subtype;
    signal font_a_out, font_b_out : font_subtype.ram_out'subtype;

    ------------------------------------------------------------------------
    signal foreground_r : std_logic_vector(31 downto 0) := x"00FFFFFF";
    signal background_r : std_logic_vector(31 downto 0) := x"00000000";
    signal backlight_r  : std_logic_vector(31 downto 0) := x"00000001";
    signal madctl_r     : std_logic_vector(31 downto 0) := x"00000028";
    signal screens      : unsigned(31 downto 0) := (others => '0');

    -- the byte sender
    signal tx_busy   : boolean := false;
    signal tx_shift  : std_logic_vector(7 downto 0) := (others => '0');
    signal tx_bits   : natural range 0 to 8 := 0;
    signal tx_half   : natural range 0 to g_half_period - 1 := 0;
    signal sck       : std_logic := '0';
    signal cs_r, reset_r, dc_r : std_logic := '1';
    signal mosi_r    : std_logic := '0';

    -- the sequencer
    type state_type is (s_reset_low, s_reset_wait, s_step, s_wait, s_text, s_glyph, s_pixel, s_idle);
    signal state     : state_type := s_reset_low;
    signal set_up    : boolean := false;
    signal pending   : boolean := false;
    signal pc        : natural range 0 to steps'length - 1 := 0;
    signal ms_left   : natural range 0 to 255 := 10;
    signal ms_count  : natural range 0 to g_clocks_a_ms - 1 := 0;
    signal requested : boolean := false;     -- a RAM read under way

    -- the pixel: its place on the screen, in its character and in the text
    signal x_count : natural range 0 to width - 1 := 0;
    signal y_count : natural range 0 to height - 1 := 0;
    signal sx, sy  : natural range 0 to g_scale - 1 := 0;
    signal fx      : natural range 0 to lcd_font_width - 1 := 0;
    signal fy      : natural range 0 to lcd_font_height - 1 := 0;
    signal col     : natural range 0 to cols := 0;
    signal row     : natural range 0 to rows := 0;
    signal row_base   : natural range 0 to text_size - 1 := 0;
    signal colour     : std_logic_vector(23 downto 0) := (others => '0');
    signal byte_index : natural range 0 to 3 := 0;

begin

    lcd_sck   <= sck;
    lcd_cs    <= cs_r;
    lcd_reset <= reset_r;
    lcd_dc    <= dc_r;
    lcd_mosi  <= mosi_r;
    lcd_led <= backlight_r(0);

    u_text : entity work.dual_port_ram
    generic map(g_dpram_subtype => text_subtype, g_ram_init_values => initial_text, g_output_register => false)
    port map(clock => clock, ram_a_in => text_a_in, ram_a_out => text_a_out,
             ram_b_in => text_b_in, ram_b_out => text_b_out);

    u_font : entity work.dual_port_ram
    generic map(g_dpram_subtype => font_subtype, g_ram_init_values => lcd_font, g_output_register => false)
    port map(clock => clock, ram_a_in => font_a_in, ram_a_out => font_a_out,
             ram_b_in => font_b_in, ram_b_out => font_b_out);

    process(clock)
        variable start_byte : boolean;
        variable byte       : std_logic_vector(7 downto 0);
        variable dc         : std_logic;
        variable code       : natural range 0 to 255;
        variable s          : step_record;
        variable glyph      : std_logic_vector(lcd_font_width - 1 downto 0);
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                (31 downto 3 => '0') & bit_of(pending) & bit_of(set_up) & bit_of(state /= s_idle));
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, foreground_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, background_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, backlight_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, madctl_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, std_logic_vector(screens));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7, to_slv(rows * 256 + cols));

            init_ram(text_a_in);
            init_ram(text_b_in);
            init_ram(font_a_in);
            init_ram(font_b_in);

            -- a character into the text
            if write_is_requested_to_address(bus_in, g_base_address + 4) then
                write_data_to_ram(text_a_in, to_integer(unsigned(get_slv_data(bus_in)(16 + text_bits - 1 downto 16))),
                                  get_slv_data(bus_in)(7 downto 0));
            end if;

            ------------------------------------------------------------
            -- the byte sender: mode 0, a bit's data out with SCK low, taken at its rise
            start_byte := false;
            byte := (others => '0');
            dc := '1';
            if tx_busy then
                if tx_half < g_half_period - 1 then
                    tx_half <= tx_half + 1;
                else
                    tx_half <= 0;
                    if sck = '0' then
                        sck <= '1';
                    else
                        sck <= '0';
                        if tx_bits > 1 then
                            tx_bits  <= tx_bits - 1;
                            tx_shift <= tx_shift(6 downto 0) & '0';
                            mosi_r <= tx_shift(6);
                        else
                            tx_bits <= 0;
                            tx_busy <= false;
                        end if;
                    end if;
                end if;
            end if;

            ------------------------------------------------------------
            -- the sequencer
            case state is
                when s_reset_low =>
                    reset_r <= '0';
                    cs_r    <= '1';
                    set_up    <= false;
                    if ms_count < g_clocks_a_ms - 1 then
                        ms_count <= ms_count + 1;
                    elsif ms_left > 1 then
                        ms_count <= 0;
                        ms_left  <= ms_left - 1;
                    else
                        ms_count  <= 0;
                        ms_left   <= 120;
                        reset_r <= '1';
                        state     <= s_reset_wait;
                    end if;

                when s_reset_wait =>
                    if ms_count < g_clocks_a_ms - 1 then
                        ms_count <= ms_count + 1;
                    elsif ms_left > 1 then
                        ms_count <= 0;
                        ms_left  <= ms_left - 1;
                    else
                        ms_count <= 0;
                        pc       <= 0;
                        cs_r   <= '0';
                        state    <= s_step;
                    end if;

                when s_step =>
                    if not tx_busy and not start_byte then
                        s := steps(pc);
                        case s.kind is
                            when k_command =>
                                byte := std_logic_vector(to_unsigned(s.value, 8));
                                dc := '0';
                                start_byte := true;
                                pc <= pc + 1;
                            when k_data =>
                                byte := std_logic_vector(to_unsigned(s.value, 8));
                                start_byte := true;
                                pc <= pc + 1;
                            when k_madctl =>
                                byte := madctl_r(7 downto 0);
                                start_byte := true;
                                pc <= pc + 1;
                            when k_delay =>
                                ms_left  <= s.value;
                                ms_count <= 0;
                                pc       <= pc + 1;
                                state    <= s_wait;
                            when k_pixels =>
                                x_count <= 0; sx <= 0; fx <= 0; col <= 0;
                                y_count <= 0; sy <= 0; fy <= 0; row <= 0;
                                row_base <= 0;
                                pc       <= pc + 1;
                                state    <= s_text;
                            when k_end =>
                                set_up <= true;
                                cs_r <= '1';
                                state  <= s_idle;
                        end case;
                    end if;

                when s_wait =>
                    if ms_left = 0 then
                        state <= s_step;
                    elsif ms_count < g_clocks_a_ms - 1 then
                        ms_count <= ms_count + 1;
                    else
                        ms_count <= 0;
                        ms_left  <= ms_left - 1;
                    end if;

                -- a pixel: its character, then its glyph's row, then its three bytes
                when s_text =>
                    if not requested then
                        request_data_from_ram(text_b_in, row_base + col);
                        requested <= true;
                    elsif ram_read_is_ready(text_b_out) then
                        code := to_integer(unsigned(get_ram_data(text_b_out)));
                        if row >= rows or code < lcd_font_first or code > lcd_font_last then
                            code := lcd_font_first;     -- a space
                        end if;
                        request_data_from_ram(font_b_in, (code - lcd_font_first) * lcd_font_height + fy);
                        state <= s_glyph;
                    end if;

                when s_glyph =>
                    if ram_read_is_ready(font_b_out) then
                        requested <= false;
                        -- the row copied whole into a known range first: indexing the RAM's
                        -- output (a record's unconstrained field) directly, Efinity numbered
                        -- its bits the other way and mirrored each character
                        glyph := get_ram_data(font_b_out);
                        if glyph(lcd_font_width - 1 - fx) = '1' and row < rows and col < cols then
                            colour <= foreground_r(23 downto 0);
                        else
                            colour <= background_r(23 downto 0);
                        end if;
                        byte_index <= 0;
                        state <= s_pixel;
                    end if;

                when s_pixel =>
                    if not tx_busy and not start_byte then
                        if byte_index < 3 then
                            byte := colour(23 - 8 * byte_index downto 16 - 8 * byte_index);
                            start_byte := true;
                            byte_index <= byte_index + 1;
                        else
                            -- the next pixel: along the row, then the next row
                            state <= s_text;
                            if x_count < width - 1 then
                                x_count <= x_count + 1;
                                if sx < g_scale - 1 then
                                    sx <= sx + 1;
                                else
                                    sx <= 0;
                                    if fx < lcd_font_width - 1 then
                                        fx <= fx + 1;
                                    else
                                        fx  <= 0;
                                        col <= col + 1;
                                    end if;
                                end if;
                            else
                                x_count <= 0; sx <= 0; fx <= 0; col <= 0;
                                if y_count < height - 1 then
                                    y_count <= y_count + 1;
                                    if sy < g_scale - 1 then
                                        sy <= sy + 1;
                                    else
                                        sy <= 0;
                                        if fy < lcd_font_height - 1 then
                                            fy <= fy + 1;
                                        else
                                            fy  <= 0;
                                            row <= row + 1;
                                            if row_base + cols <= text_size - 1 then
                                                row_base <= row_base + cols;
                                            end if;
                                        end if;
                                    end if;
                                else
                                    screens <= screens + 1;
                                    state   <= s_step;     -- to the program's end
                                end if;
                            end if;
                        end if;
                    end if;

                when s_idle =>
                    cs_r <= '1';
                    if pending then
                        pending <= false;
                        pc      <= draw_start;
                        cs_r  <= '0';
                        state   <= s_step;
                    end if;
            end case;

            -- the requests
            if write_is_requested_to_address(bus_in, g_base_address) then
                if get_slv_data(bus_in)(0) = '1' then
                    state     <= s_reset_low;
                    ms_left   <= 10;
                    ms_count  <= 0;
                    pending   <= false;
                    requested <= false;
                elsif get_slv_data(bus_in)(1) = '1' then
                    pending <= true;
                end if;
            end if;

            -- a byte to send: its first bit out at once
            if start_byte then
                tx_busy  <= true;
                tx_shift <= byte;
                tx_bits  <= 8;
                tx_half  <= 0;
                sck      <= '0';
                dc_r   <= dc;
                mosi_r <= byte(7);
            end if;
        end if;
    end process;

end architecture rtl;
