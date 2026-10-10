library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_dsp_pkg.all;
    use work.dual_port_ram_pkg.all;
    use work.fixed_format_pkg.all;

-- One scaler for all the ADC measurements: a channel's raw ADC counts to fixed point (32 bit,
-- 20 fractional bits, fixed_format_pkg) as
--
--   scaled = counts * slope + offset
--
-- a single fused multiply-add of the fixed point library's fixed_dsp, slope and offset from two
-- block RAMs of the memory library (dual_port_ram), an entry per channel: port A takes the
-- register bus's writes, port B the pipeline's reads. When a channel is ready it is marked
-- pending; each clock the lowest pending channel goes through the pipeline, so channels ready
-- at the same time are scaled one after another, a clock apart. Its raw value is taken from the
-- input when it is selected, the newest: the ADC modules hold each value until their next
-- result, microseconds later.
--
--   select    the channel, its raw value, the RAM reads requested
--   RAM       the request registered, slope and offset read (dual_port_ram without its output
--             register); the raw value and channel delayed alongside
--   fixed_dsp counts * slope + offset (2 clocks, 3 with g_product_register)
--   result    the 20 fractional bit value, saturated at +-2048, to scaled(channel)
--
-- The slope carries 30 fractional bits (|slope| < 2 units a count), so the product's radix
-- is 30; the offset is held with 20 and shifted to 30 for the add, the result shifted back.
-- The RAMs start at zero: every channel reads 0 until its slope and offset are written.
--
-- register map, offsets from g_base_address, channel i < 32
--   i       W  slope of channel i, 30 fractional bits
--   32 + i  W  offset of channel i, 20 fractional bits
--   64 + i  R  the scaled value of channel i, 20 fractional bits
entity measurement_scaler is
    generic(
        g_base_address     : natural := 900;
        g_channels         : positive := 19;
        g_product_register : boolean := true    -- fixed_dsp's, for 32 x 32 at the clock
    );
    port(
        clock          : in std_logic;
        bus_in         : in fpga_interconnect_record;
        bus_out        : out fpga_interconnect_record;
        -- the measurements: raw ADC counts, and a strobe when a new one is there
        raw            : in fixed_array(0 to g_channels-1);
        ready          : in std_logic_vector(0 to g_channels-1);
        -- the scaled values, each updated in turn, and a strobe with the channel updated
        scaled         : out fixed_array(0 to g_channels-1);
        scaled_ready   : out boolean;
        scaled_channel : out natural range 0 to g_channels-1
    );
end entity measurement_scaler;

architecture rtl of measurement_scaler is

    constant slope_radix : natural := 30;
    constant dsp_latency : natural := 2 + boolean'pos(g_product_register);

    subtype dsp_in_record  is fixed_dsp_in_record(a(31 downto 0), d(31 downto 0), b(31 downto 0), c(63 downto 0));
    subtype dsp_out_record is fixed_dsp_out_record(result(63 downto 0));
    signal dsp_in  : dsp_in_record := init_fixed_dsp_in(32);
    signal dsp_out : dsp_out_record;

    -- the slope and offset RAMs: 32 entries of 32 bits, starting at zero
    constant ram_subtype  : dpram_ref_record := create_ref_subtypes(datawidth => 32, addresswidth => 5);
    constant ram_zeros    : ram_array(0 to 31)(31 downto 0) := (others => (others => '0'));
    signal slope_write_in,   slope_read_in   : ram_subtype.ram_in'subtype;
    signal slope_write_data, slope_read_out  : ram_subtype.ram_out'subtype;
    signal offset_write_in,   offset_read_in  : ram_subtype.ram_in'subtype;
    signal offset_write_data, offset_read_out : ram_subtype.ram_out'subtype;

    -- waiting to be scaled
    signal pending : std_logic_vector(0 to g_channels-1) := (others => '0');

    -- the channel and raw value of each RAM read, delayed to its data: the request register
    -- and the RAM's read
    type channel_array is array (natural range <>) of natural range 0 to g_channels-1;
    signal ram_channels : channel_array(1 to 2) := (others => 0);
    signal ram_raws     : fixed_array(1 to 2) := (others => fixed_zero);
    -- the channels in fixed_dsp: the request is registered into it, then dsp_latency clocks
    -- to its result, so the channel leaving it is dsp_latency + 1 stages down
    signal dsp_channels : channel_array(1 to dsp_latency + 1) := (others => 0);

    signal scaled_r : fixed_array(0 to g_channels-1) := (others => fixed_zero);

    -- the product's 30 fractional bits back to 20, saturated
    function to_scaled(product : signed(63 downto 0)) return fixed_number is
        constant shift : natural := slope_radix - fixed_radix;
        -- the bits above the result must all be its sign
        constant top   : signed(63 downto fixed_wordlength - 1 + shift) := product(63 downto fixed_wordlength - 1 + shift);
    begin
        if top = (top'range => '0') or top = (top'range => '1') then
            return product(fixed_wordlength - 1 + shift downto shift);
        elsif product(63) = '0' then
            return fixed_max;
        end if;
        return fixed_min;
    end function;

begin

    scaled <= scaled_r;

    u_slopes : entity work.dual_port_ram
    generic map(g_dpram_subtype => ram_subtype, g_ram_init_values => ram_zeros, g_output_register => false)
    port map(clock => clock, ram_a_in => slope_write_in, ram_a_out => slope_write_data,
             ram_b_in => slope_read_in, ram_b_out => slope_read_out);

    u_offsets : entity work.dual_port_ram
    generic map(g_dpram_subtype => ram_subtype, g_ram_init_values => ram_zeros, g_output_register => false)
    port map(clock => clock, ram_a_in => offset_write_in, ram_a_out => offset_write_data,
             ram_b_in => offset_read_in, ram_b_out => offset_read_out);

    u_fixed_dsp : entity work.fixed_dsp(rtl)
    generic map(g_pre_add_register => false, g_product_register => g_product_register)
    port map(clock => clock, fixed_dsp_in => dsp_in, fixed_dsp_out => dsp_out);

    process(clock)
        variable next_pending : std_logic_vector(0 to g_channels-1);
        variable found        : boolean;
    begin
        if rising_edge(clock) then
            -- the RAMs written from the bus at a decoded address, the scaled values read back
            init_bus(bus_out);
            init_ram(slope_write_in);
            init_ram(offset_write_in);
            if write_is_requested_to_address_range(bus_in, g_base_address, g_base_address + 32) then
                write_data_to_ram(slope_write_in, get_address(bus_in) - g_base_address, get_slv_data(bus_in));
            end if;
            if write_is_requested_to_address_range(bus_in, g_base_address + 32, g_base_address + 64) then
                write_data_to_ram(offset_write_in, get_address(bus_in) - g_base_address - 32, get_slv_data(bus_in));
            end if;
            if data_is_requested_from_address_range(bus_in, g_base_address + 64, g_base_address + 64 + g_channels) then
                write_data_to_address(bus_out, 0,
                    std_logic_vector(scaled_r(get_address(bus_in) - g_base_address - 64)));
            end if;

            -- the lowest pending channel selected, its slope and offset requested, and the
            -- ready channels marked pending
            init_ram(slope_read_in);
            init_ram(offset_read_in);
            next_pending := pending;
            found        := false;
            for i in 0 to g_channels-1 loop
                if pending(i) = '1' and not found then
                    found := true;
                    request_data_from_ram(slope_read_in, i);
                    request_data_from_ram(offset_read_in, i);
                    ram_channels(1) <= i;
                    ram_raws(1)     <= raw(i);
                    next_pending(i) := '0';
                end if;
            end loop;
            for i in 0 to g_channels-1 loop
                if ready(i) = '1' then
                    next_pending(i) := '1';
                end if;
            end loop;
            pending <= next_pending;
            ram_channels(2) <= ram_channels(1);
            ram_raws(2)     <= ram_raws(1);

            -- counts * slope + offset when the RAMs have them, the offset at the product's radix
            init_fixed_dsp(dsp_in);
            if ram_read_is_ready(slope_read_out) then
                add(dsp_in, a => ram_raws(2), b => signed(get_ram_data(slope_read_out)),
                    c => shift_left(resize(signed(get_ram_data(offset_read_out)), 64), slope_radix - fixed_radix));
            end if;
            dsp_channels <= ram_channels(2) & dsp_channels(1 to dsp_latency);

            -- the result to its channel
            scaled_ready <= false;
            if dsp_out.ready_with_1 = '1' then
                scaled_r(dsp_channels(dsp_latency + 1)) <= to_scaled(dsp_out.result);
                scaled_channel <= dsp_channels(dsp_latency + 1);
                scaled_ready   <= true;
            end if;
        end if;
    end process;

end architecture rtl;
