library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;
    use work.dual_port_ram_pkg.all;

-- The output voltage's arbitrary waveform: a table of points, each a voltage held for a number
-- of the output bridge's periods, played in order and repeated, as the closed loop's reference.
--
-- The table is a block RAM (hVHDL_memory_library's dual_port_ram) of g_points entries, the
-- voltage (fixed point, 20 fractional bits) and the periods (at least 1) of each. Software
-- writes it through the registers: the index, the voltage, then the periods, which writes the
-- entry and moves the index on, so a table goes in as voltage, periods, voltage, periods...
--
-- start begins the waveform at its first point; each step (the bridge's period start) counts a
-- period of the point being held, and after its periods the next point follows, from the
-- first again after the last. v_ref is the point being held. After the set number of passes
-- (0: until stopped) finished rises and v_ref stays at the last point; running low stops it.
-- The point after the one held is always read ahead, so a step never waits for the RAM; while
-- stopped the first two are kept read, so a start has them at once.
--
-- register map, offsets from g_base_address
--   0  RW the number of points, 1 .. g_points
--   1  RW passes, 0: until stopped
--   2  RW the index the next entry goes to
--   3  RW the entry's voltage, V (20 fractional bits)
--   4  RW the entry's periods; a write puts the entry into the table at the index, index + 1
--   5  R  bit0 playing, bit1 finished, bits 31:16 the point held
--   6  R  passes completed
entity waveform_player is
    generic(
        g_base_address : natural := 1200;
        g_points       : natural := 512
    );
    port(
        clock    : in std_logic;
        bus_in   : in fpga_interconnect_record;
        bus_out  : out fpga_interconnect_record;
        start    : in boolean;   -- a clock's pulse: the waveform from its first point
        step     : in boolean;   -- a clock's pulse a period: one more period of the point held
        running  : in boolean;   -- low: stopped
        v_ref    : out fixed_number;
        finished : out boolean
    );
end entity waveform_player;

architecture rtl of waveform_player is

    function bits_for(count : natural) return natural is
        variable retval : natural := 1;
    begin
        while 2**retval < count loop
            retval := retval + 1;
        end loop;
        return retval;
    end function;

    constant index_bits : natural := bits_for(g_points);

    constant ram_subtype : dpram_ref_record := create_ref_subtypes(datawidth => 64, addresswidth => index_bits);
    constant ram_zeros   : ram_array(0 to 2**index_bits - 1)(63 downto 0) := (others => (others => '0'));
    signal write_in, read_in   : ram_subtype.ram_in'subtype;
    signal write_out, read_out : ram_subtype.ram_out'subtype;

    signal count_r   : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(1, 32));
    signal passes_r  : std_logic_vector(31 downto 0) := (others => '0');
    signal index_r   : std_logic_vector(31 downto 0) := (others => '0');
    signal voltage_r : std_logic_vector(31 downto 0) := (others => '0');
    signal periods_r : std_logic_vector(31 downto 0) := (others => '0');

    subtype point_index is natural range 0 to g_points - 1;
    signal count : natural range 1 to g_points := 1;

    -- the point held and the one after it
    signal held_index : point_index := 0;
    signal held_v     : fixed_number := fixed_zero;
    signal held_n     : unsigned(31 downto 0) := to_unsigned(1, 32);
    signal next_v     : fixed_number := fixed_zero;
    signal next_n     : unsigned(31 downto 0) := to_unsigned(1, 32);
    signal held_for   : unsigned(31 downto 0) := (others => '0');

    signal playing     : boolean := false;
    signal finished_r  : boolean := false;
    signal passes_done : unsigned(31 downto 0) := (others => '0');

    -- the reads: their destination, a clock behind the request (the RAM's read latency)
    type destination_type is (none, to_held, to_next);
    signal requested, arriving : destination_type := none;
    signal next_wanted : boolean := false;    -- the point after the held one to read
    signal idle_turn   : boolean := false;    -- stopped: the first, then the second, in turn

    function following(index : point_index; count : natural) return point_index is
    begin
        if index + 1 >= count then
            return 0;
        end if;
        return index + 1;
    end function;

    function bit_of(b : boolean) return std_logic is
    begin
        if b then return '1'; end if;
        return '0';
    end function;

    function at_least_one(n : std_logic_vector) return unsigned is
    begin
        if unsigned(n(31 downto 0)) = 0 then
            return to_unsigned(1, 32);
        end if;
        return unsigned(n(31 downto 0));
    end function;

begin

    v_ref    <= held_v;
    finished <= finished_r;

    u_table : entity work.dual_port_ram
    generic map(g_dpram_subtype => ram_subtype, g_ram_init_values => ram_zeros, g_output_register => false)
    port map(clock => clock, ram_a_in => write_in, ram_a_out => write_out,
             ram_b_in => read_in, ram_b_out => read_out);

    process(clock)
        variable data : std_logic_vector(63 downto 0);
        variable index : natural;
        variable wrapped, last_pass : boolean;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address(bus_in, bus_out, g_base_address + 0, count_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, passes_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 2, index_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 3, voltage_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 4, periods_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5,
                std_logic_vector(to_unsigned(held_index, 16)) & (15 downto 2 => '0') &
                bit_of(finished_r) & bit_of(playing));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6, std_logic_vector(passes_done));

            init_ram(write_in);
            init_ram(read_in);

            if unsigned(count_r) < 1 then
                count <= 1;
            elsif unsigned(count_r) > g_points then
                count <= g_points;
            else
                count <= to_integer(unsigned(count_r));
            end if;

            -- an entry written: the voltage held, then the periods put both in at the index
            if write_is_requested_to_address(bus_in, g_base_address + 4) then
                index := to_integer(unsigned(index_r(index_bits - 1 downto 0)));
                write_data_to_ram(write_in, index, voltage_r & get_slv_data(bus_in));
                index_r <= std_logic_vector(to_unsigned((index + 1) mod 2**index_bits, 32));
            end if;

            -- the reads coming back
            arriving <= requested;
            requested <= none;
            if ram_read_is_ready(read_out) then
                data := get_ram_data(read_out);
                case arriving is
                    when to_held =>
                        held_v <= signed(data(63 downto 32));
                        held_n <= at_least_one(data(31 downto 0));
                    when to_next =>
                        next_v <= signed(data(63 downto 32));
                        next_n <= at_least_one(data(31 downto 0));
                    when none =>
                end case;
            end if;

            if not running then
                playing <= false;
            end if;

            if start then
                -- the first point is held already (read while stopped)
                playing     <= true;
                finished_r  <= false;
                held_for    <= (others => '0');
                passes_done <= (others => '0');
            elsif playing and step then
                if held_for + 1 >= held_n then
                    -- the held point done: the next one, or after the last pass the end
                    wrapped   := following(held_index, count) = 0;
                    last_pass := wrapped and unsigned(passes_r) /= 0 and passes_done + 1 >= unsigned(passes_r);
                    if wrapped then
                        passes_done <= passes_done + 1;
                    end if;
                    if last_pass then
                        playing    <= false;
                        finished_r <= true;
                    else
                        held_index  <= following(held_index, count);
                        held_v      <= next_v;
                        held_n      <= next_n;
                        held_for    <= (others => '0');
                        next_wanted <= true;
                    end if;
                else
                    held_for <= held_for + 1;
                end if;
            end if;

            -- the reads: playing, the point after the held one when it changes; stopped, the
            -- first two in turn (a table being written, a count changed: always the latest)
            if playing or start then
                if next_wanted and not (playing and step) then
                    next_wanted <= false;
                    request_data_from_ram(read_in, following(held_index, count));
                    requested <= to_next;
                end if;
            elsif not (finished_r and running) then
                -- (finished, the last point stays until the run is over)
                idle_turn <= not idle_turn;
                if idle_turn then
                    request_data_from_ram(read_in, 0);
                    requested  <= to_held;
                    held_index <= 0;
                else
                    request_data_from_ram(read_in, following(0, count));
                    requested <= to_next;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
