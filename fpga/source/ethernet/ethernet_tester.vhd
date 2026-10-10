library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;
    use work.fpga_interconnect_pkg.all;

-- RGMII loopback test through the fpga_interconnect bus: transmits numbered test frames
-- and checks what comes back, intended for PHY loopback. The RGMII interface itself is in
-- the ethernet entity, which gives the test generator priority on the transmitter.
--
-- register map, offsets from g_base_address
--   0  W: command bit0 start transmit, bit1 stop transmit, bit2 clear receive counters
--      R: status  bit0 transmitter busy, bit1 rx pll locked
--   1  RW number of frames to transmit, 0 transmits until stopped
--   2  RW payload length 46..1500
--   3  RW options bit0 delay falling edge tx nibble by one clock, bit1 hold rx pll in reset
--      (both are used by the RGMII interface of the ethernet entity)
--   4  R  frames sent
--   5  R  frames received
--   6  R  good frames received
--   7  R  received frames with crc error
--   8  R  received frames with payload error
--   9  R  sequence number gaps in good frames
--  10  R  length of last received frame, from destination address to FCS
--  11  R  receive nibble alignment, 3 = no frame start found yet
--  12  R  receive clock cycle counter
entity ethernet_tester is
    generic(
        g_base_address : natural := 400
    );
    port(
        clock             : in std_logic;   -- transmit and bus clock, 125 MHz
        bus_in            : in fpga_interconnect_record;
        bus_out           : out fpga_interconnect_record;

        tx_gmii_out       : out gmii_record;
        tx_is_busy        : out boolean;
        delay_lo_half     : out boolean;
        hold_rx_pll_reset : out boolean;
        rx_pll_locked     : in std_logic;

        rx_clock          : in std_logic;
        rx_gmii_in        : in gmii_record;
        rx_frame_is_ended : in boolean;
        rx_alignment      : in natural range 0 to 3
    );
end entity ethernet_tester;

architecture rtl of ethernet_tester is

    signal frame_count    : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(10, 32));
    signal payload_length : std_logic_vector(31 downto 0) := std_logic_vector(to_unsigned(46, 32));
    signal options        : std_logic_vector(31 downto 0) := (others => '0');
    signal status         : std_logic_vector(31 downto 0) := (others => '0');

    signal start, stop, clear : boolean := false;
    signal tx_busy        : boolean;
    signal frames_sent    : unsigned(31 downto 0);
    signal length_to_send : natural range 46 to 1500 := 46;

    -- receive clock domain
    signal rx_clear  : boolean;
    signal rx_frames, rx_good, rx_crc_errors, rx_payload_errors, rx_gaps : unsigned(31 downto 0);
    signal rx_length : unsigned(15 downto 0);
    signal rx_cycles : unsigned(31 downto 0) := (others => '0');

    constant rx_status_width : natural := 6*32 + 16 + 2;
    signal rx_status_rx_domain : std_logic_vector(rx_status_width-1 downto 0);
    signal rx_status           : std_logic_vector(rx_status_width-1 downto 0) := (others => '0');

    signal rx_pll_locked_sync : std_logic_vector(1 downto 0) := (others => '0');

    function field(v : std_logic_vector; n : natural) return std_logic_vector is
    begin
        return v(32*n+31 downto 32*n);
    end function;

begin

    tx_is_busy        <= tx_busy;
    delay_lo_half     <= options(0) = '1';
    hold_rx_pll_reset <= options(1) = '1';

    status(0) <= '1' when tx_busy else '0';
    status(1) <= rx_pll_locked_sync(1);
    status(31 downto 2) <= (others => '0');

    process(clock)
        variable length : natural;
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            rx_pll_locked_sync <= rx_pll_locked_sync(0) & rx_pll_locked;

            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0 , status);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 1 , frame_count);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 2 , payload_length);
            connect_data_to_address          (bus_in, bus_out, g_base_address + 3 , options);
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 4 , std_logic_vector(frames_sent));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 5 , field(rx_status, 0));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 6 , field(rx_status, 1));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 7 , field(rx_status, 2));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 8 , field(rx_status, 3));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 9 , field(rx_status, 4));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 10, x"0000" & rx_status(6*32+15 downto 6*32));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 11, (31 downto 2 => '0') & rx_status(6*32+17 downto 6*32+16));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 12, field(rx_status, 5));

            length := to_integer(unsigned(payload_length(10 downto 0)));
            if length < 46 then
                length_to_send <= 46;
            elsif length > 1500 then
                length_to_send <= 1500;
            else
                length_to_send <= length;
            end if;

            start <= false;
            stop  <= false;
            clear <= false;
            if write_is_requested_to_address(bus_in, g_base_address) then
                start <= get_slv_data(bus_in)(0) = '1';
                stop  <= get_slv_data(bus_in)(1) = '1';
                clear <= get_slv_data(bus_in)(2) = '1';
            end if;
        end if;
    end process;

------------------------------------------------------------------------
    u_generator : entity work.ethernet_frame_generator
    port map(
        clock           => clock
        ,start          => start
        ,stop           => stop
        ,frame_count    => to_integer(unsigned(frame_count(30 downto 0)))
        ,payload_length => length_to_send
        ,gmii_out       => tx_gmii_out
        ,is_busy        => tx_busy
        ,frames_sent    => frames_sent
    );

------------------------------------------------------------------------
    u_clear_to_rx : entity work.cdc_pulse
    port map(clock, clear, rx_clock, rx_clear);

    u_checker : entity work.ethernet_frame_checker
    port map(
        clock           => rx_clock
        ,clear          => rx_clear
        ,gmii_in        => rx_gmii_in
        ,frame_is_ended => rx_frame_is_ended
        ,frames         => rx_frames
        ,good_frames    => rx_good
        ,crc_errors     => rx_crc_errors
        ,payload_errors => rx_payload_errors
        ,sequence_gaps  => rx_gaps
        ,last_length    => rx_length
    );

    rx_cycle_counter : process(rx_clock)
    begin
        if rising_edge(rx_clock) then
            rx_cycles <= rx_cycles + 1;
        end if;
    end process;

    rx_status_rx_domain <= std_logic_vector(to_unsigned(rx_alignment, 2))
                           & std_logic_vector(rx_length)
                           & std_logic_vector(rx_cycles)
                           & std_logic_vector(rx_gaps)
                           & std_logic_vector(rx_payload_errors)
                           & std_logic_vector(rx_crc_errors)
                           & std_logic_vector(rx_good)
                           & std_logic_vector(rx_frames);

    u_rx_status : entity work.cdc_snapshot
    generic map(rx_status_width)
    port map(rx_clock, rx_status_rx_domain, clock, rx_status);

end architecture rtl;
