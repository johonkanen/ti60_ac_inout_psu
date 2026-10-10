library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- TI ADS7056, 14 bit SAR ADC with a serial interface, up to 60 MHz SCLK and 2.5 MS/s.
-- A conversion frame: CS falls (the sample is taken), the ADC shifts out two leading zeros,
-- the 14 bit result MSB first and two trailing zeros, the first bit on the CS edge.
-- 18 SCLK cycles per frame, SCLK idles low. On the control board SDO changes shortly after
-- the SCLK rising edges as seen from the FPGA core.
-- The ADC has a pipeline delay of one frame: the result read in a frame is the sample taken
-- at the CS falling edge of the previous frame. sample_taken pulses at each CS falling edge
-- of a conversion frame, result_is_new when the frame has been read.
--
-- SCLK is sclk_low clocks low and sclk_high clocks high. Bit k is read sample_delay + 1
-- clocks after the k:th SCLK falling edge (bit 0 after the CS edge); the delay is set for the
-- round trip through the pins and the ADC, which is a large part of an SCLK period.
-- leading_error / trailing_error flag frames whose zero bits were not zero, a sign of a
-- wrong sample_delay.
--
-- After power up the ADC needs a dummy frame of at least 24 SCLKs and an offset calibration
-- frame of at least 64 SCLKs; calibrate repeats both.
entity ads7056 is
    generic(
        g_startup_clocks : positive := 2000    -- power up time of the ADC
    );
    port(
        clock          : in std_logic;
        sclk_high      : in natural range 1 to 15;
        sclk_low       : in natural range 1 to 15;
        cs_high        : in natural range 1 to 255;   -- clocks CS is high between frames
        sample_delay   : in natural range 0 to 31;
        start          : in boolean;                  -- a conversion when ready
        calibrate      : in boolean;
        is_ready       : out boolean;
        is_calibrated  : out boolean;
        result         : out unsigned(13 downto 0);
        result_is_new  : out boolean;
        leading_error  : out boolean;
        trailing_error : out boolean;
        sample_taken   : out boolean;

        sclk           : out std_logic;
        cs_n           : out std_logic;
        sdo            : in std_logic
    );
end entity ads7056;

architecture rtl of ads7056 is

    constant dummy_clocks       : natural := 24;
    constant calibration_clocks : natural := 64;
    constant conversion_clocks  : natural := 18;

    type states is (startup, dummy_frame, calibration_frame, idle, frame, readout, cs_wait);
    signal state      : states := startup;
    signal next_state : states := idle;

    signal startup_count : natural range 0 to g_startup_clocks := 0;
    signal phase       : natural range 0 to 31 := 0;      -- clock within the SCLK period
    signal sclk_count  : natural range 0 to calibration_clocks := 0;
    signal frame_sclks : natural range 0 to calibration_clocks := 0;
    signal sclk_r      : std_logic := '0';
    signal cs_n_r      : std_logic := '1';
    signal is_conversion : boolean := false;
    signal calibrated  : boolean := false;

    -- the launch strobes, delayed by sample_delay clocks, read the bits
    signal launch      : std_logic := '0';
    signal launches    : std_logic_vector(31 downto 0) := (others => '0');
    signal bits_read   : natural range 0 to conversion_clocks := 0;
    signal shift       : std_logic_vector(conversion_clocks-1 downto 0) := (others => '0');
    signal wait_count  : natural range 0 to 255 := 0;
    signal readout_count : natural range 0 to 63 := 0;

begin

    sclk          <= sclk_r;
    cs_n          <= cs_n_r;
    is_ready      <= state = idle and not start;
    is_calibrated <= calibrated;

    process(clock)
        procedure start_frame(sclks : natural; conversion : boolean; following : states) is
        begin
            cs_n_r        <= '0';
            launch        <= '1';           -- the first bit leaves on the CS edge
            phase         <= 0;
            sclk_count    <= 0;
            frame_sclks   <= sclks;
            is_conversion <= conversion;
            next_state    <= following;
            bits_read     <= 0;
            state         <= frame;
        end procedure;

        variable sampled : std_logic;
    begin
        if rising_edge(clock) then
            result_is_new <= false;
            sample_taken  <= false;
            launch        <= '0';
            launches      <= launches(30 downto 0) & launch;

            -- read a bit sample_delay clocks after it was launched
            if sample_delay = 0 then
                sampled := launch;
            else
                sampled := launches(sample_delay - 1);
            end if;
            if sampled = '1' and bits_read < conversion_clocks then
                shift     <= shift(shift'left-1 downto 0) & sdo;
                bits_read <= bits_read + 1;
            end if;

            CASE state is
                WHEN startup =>
                    if startup_count < g_startup_clocks then
                        startup_count <= startup_count + 1;
                    else
                        state <= dummy_frame;
                    end if;

                WHEN dummy_frame =>
                    start_frame(dummy_clocks, false, calibration_frame);

                WHEN calibration_frame =>
                    start_frame(calibration_clocks, false, idle);

                WHEN idle =>
                    if calibrate then
                        calibrated <= false;
                        state      <= dummy_frame;
                    elsif start then
                        start_frame(conversion_clocks, true, idle);
                        sample_taken <= true;
                    end if;

                WHEN frame =>
                    -- low for sclk_low clocks, then high for sclk_high clocks
                    if phase = sclk_low - 1 then
                        sclk_r <= '1';
                    end if;
                    if phase < sclk_low + sclk_high - 1 then
                        phase <= phase + 1;
                    else
                        phase  <= 0;
                        sclk_r <= '0';
                        if sclk_count = frame_sclks - 1 then
                            cs_n_r        <= '1';
                            wait_count    <= 0;
                            readout_count <= 0;
                            state         <= readout;
                        else
                            sclk_count <= sclk_count + 1;
                            launch     <= '1';
                        end if;
                    end if;

                WHEN readout =>
                    -- the last bits are read up to sample_delay clocks after the frame; a
                    -- sample_delay changed during the frame may lose a bit, so give up after 32 clocks
                    if wait_count < cs_high then
                        wait_count <= wait_count + 1;
                    end if;
                    if readout_count < 63 then
                        readout_count <= readout_count + 1;
                    end if;
                    if bits_read = conversion_clocks or not is_conversion or readout_count > 32 then
                        if is_conversion then
                            result         <= unsigned(shift(15 downto 2));
                            leading_error  <= shift(17 downto 16) /= "00";
                            trailing_error <= shift(1 downto 0) /= "00";
                            result_is_new  <= true;
                        end if;
                        state <= cs_wait;
                    end if;

                WHEN cs_wait =>
                    -- starting the next frame takes three more clocks: idle, start, frame
                    if wait_count + 3 < cs_high then
                        wait_count <= wait_count + 1;
                    else
                        if next_state = idle and not is_conversion then
                            calibrated <= true;
                        end if;
                        state <= next_state;
                    end if;
            end CASE;
        end if;
    end process;

end architecture rtl;
