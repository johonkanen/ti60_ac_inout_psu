library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.fixed_format_pkg.all;

-- The DHB's trips, on the scaled measurements as the measurement scaler gives them:
--
--   overcurrent   |the DHB's current| or |the output bridge's| above its limit (5 A) for a set
--                 number of that channel's readings in a row (3, ~10 us at 3.2 us a reading):
--                 the filtered streams carry spikes of a few us from the switching
--   overvoltage   either DC link above its limit for a set number of its readings in a row
--                 (8): the readings carry the switching's noise, single ones +-30 V around a
--                 ~95 V link, so a single reading must not trip it
--
-- A trip is latched and holds trip true (which turns the bridge off) until it is cleared from
-- the register bus; the measurement that tripped it is kept. The current is the sigma-delta
-- channel's filtered value (sinc3, ~10 us), not the peaks of single pulses.
--
-- register map, offsets from g_base_address; fixed point with 20 fractional bits
--   0  RW overcurrent limit, A (5.0)
--   1  RW overvoltage limit, V (120.0)
--   2  R  bit0 overcurrent, bit1 overvoltage tripped
--      W  bit0 clear the trips
--   3  R  the current that tripped
--   4  R  the voltage that tripped (the last of the readings in a row)
--   5  RW overvoltage readings in a row that trip, 1..255 (8)
--   6  RW overcurrent readings in a row that trip, 1..255 (3)
entity dhb_protection is
    generic(
        g_base_address      : natural := 1120;
        g_current_channel   : natural := 18;   -- the measurement scaler's sdm i_dab
        g_output_current_channel : natural := 17;  -- sdm iac_out, the output bridge's
        g_primary_channel   : natural := 9;    -- pri 1 vdc_in
        g_secondary_channel : natural := 0     -- sec 0 vdc_out
    );
    port(
        clock          : in std_logic;
        bus_in         : in fpga_interconnect_record;
        bus_out        : out fpga_interconnect_record;
        scaled         : in fixed_array;
        scaled_ready   : in boolean;
        scaled_channel : in natural;
        trip           : out boolean
    );
end entity dhb_protection;

architecture rtl of dhb_protection is

    signal current_limit_r : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(5.0));
    signal voltage_limit_r : std_logic_vector(31 downto 0) := std_logic_vector(to_fixed(120.0));
    signal overcurrent     : std_logic := '0';
    signal overvoltage     : std_logic := '0';
    signal trip_current    : fixed_number := fixed_zero;
    signal trip_voltage    : fixed_number := fixed_zero;
    signal readings_r      : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(8, 32));
    -- readings over the voltage limit in a row, primary and secondary
    signal primary_over, secondary_over : natural range 0 to 255 := 0;
    -- readings over the current limit in a row, the DHB's and the output bridge's
    signal current_readings_r : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(3, 32));
    signal dhb_over, output_over : natural range 0 to 255 := 0;

begin

    trip <= overcurrent = '1' or overvoltage = '1';

    process(clock)
        variable value : fixed_number;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            connect_data_to_address(bus_in, bus_out, g_base_address + 0, current_limit_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 1, voltage_limit_r);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 2,
                (31 downto 2 => '0') & overvoltage & overcurrent);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 3, std_logic_vector(trip_current));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4, std_logic_vector(trip_voltage));
            connect_data_to_address(bus_in, bus_out, g_base_address + 5, readings_r);
            connect_data_to_address(bus_in, bus_out, g_base_address + 6, current_readings_r);

            if scaled_ready then
                value := scaled(scaled_channel);
                -- each current's readings over the limit in a row; enough of them trip
                if scaled_channel = g_current_channel then
                    if abs(resize(value, fixed_wordlength + 1)) <= resize(signed(current_limit_r), fixed_wordlength + 1) then
                        dhb_over <= 0;
                    elsif dhb_over < 255 then
                        dhb_over <= dhb_over + 1;
                        if dhb_over + 1 >= to_integer(unsigned(current_readings_r(7 downto 0))) and overcurrent = '0' then
                            overcurrent  <= '1';
                            trip_current <= value;
                        end if;
                    end if;
                end if;
                if scaled_channel = g_output_current_channel then
                    if abs(resize(value, fixed_wordlength + 1)) <= resize(signed(current_limit_r), fixed_wordlength + 1) then
                        output_over <= 0;
                    elsif output_over < 255 then
                        output_over <= output_over + 1;
                        if output_over + 1 >= to_integer(unsigned(current_readings_r(7 downto 0))) and overcurrent = '0' then
                            overcurrent  <= '1';
                            trip_current <= value;
                        end if;
                    end if;
                end if;
                -- each DC link's readings over the limit in a row; enough of them trip
                if scaled_channel = g_primary_channel then
                    if value <= signed(voltage_limit_r) then
                        primary_over <= 0;
                    elsif primary_over < 255 then
                        primary_over <= primary_over + 1;
                        if primary_over + 1 >= to_integer(unsigned(readings_r(7 downto 0))) and overvoltage = '0' then
                            overvoltage  <= '1';
                            trip_voltage <= value;
                        end if;
                    end if;
                end if;
                if scaled_channel = g_secondary_channel then
                    if value <= signed(voltage_limit_r) then
                        secondary_over <= 0;
                    elsif secondary_over < 255 then
                        secondary_over <= secondary_over + 1;
                        if secondary_over + 1 >= to_integer(unsigned(readings_r(7 downto 0))) and overvoltage = '0' then
                            overvoltage  <= '1';
                            trip_voltage <= value;
                        end if;
                    end if;
                end if;
            end if;

            if write_is_requested_to_address(bus_in, g_base_address + 2) and get_slv_data(bus_in)(0) = '1' then
                overcurrent <= '0';
                overvoltage <= '0';
            end if;
        end if;
    end process;

end architecture rtl;
