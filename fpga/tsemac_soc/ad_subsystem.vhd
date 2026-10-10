library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;
    use work.fixed_format_pkg.all;

-- The control board's two ADS7056, each behind its own 8 input analog mux
-- (source/adc/board_adc.vhd), with the pins and names of the power supply design
-- (ac_in_ac_out_lab_power_supply):
--   sec  registers at g_sec_address (700..), board_test's ADC: mux 0 vdc_out, 1 vac_out,
--        2 vfilter_out
--   pri  registers at g_pri_address (720..): mux 0 vac_in, 1 vdc_in, 3 vfilter_in. Its SPI
--        goes through the primary side's 150 Mbps digital isolators (SDO back on a 3.3 V
--        pin), so its bits arrive about 10 ns later than sec's: sample delays 3..6 read
--        clean frames (1 and 2 one bit early, 7 one bit late), 5 is used
-- sdm  registers at g_sdm_address (740..): the power stage's three isolated sigma-delta current
--      modulators on a common 20 MHz clock (source/adc/sdm_adc.vhd), channel 0 grid_inu
--      (iac_in), 1 output_inu (iac_out), 2 dab (i_dab); their filtered results are probes too.
-- Both run from power up with all eight mux positions enabled. The ADCs run from adc_clock
-- (200 MHz: 50 MHz SCLK, 2.5 MS/s each), the registers and results from clock. The latest
-- results, 0..16383, are probes of the scope.
entity ad_subsystem is
    generic(
        g_sec_address : natural := 700;
        g_pri_address : natural := 720;
        g_sdm_address : natural := 740
    );
    port(
        clock     : in std_logic;
        adc_clock : in std_logic;
        bus_in    : in fpga_interconnect_record;
        bus_out   : out fpga_interconnect_record;
        -- to the scope, synchronous to clock
        sec_latest    : out scope_sample;
        sec_positions : out scope_sample_array(0 to 7);
        pri_latest    : out scope_sample;
        pri_positions : out scope_sample_array(0 to 7);
        sdm_results   : out scope_sample_array(0 to 2);
        sdm_means     : out scope_sample_array(0 to 2);
        -- the sigma-delta channels' period means taken at its pulses (the output bridge's
        -- modulator: twice a period)
        sdm_sample    : in boolean := false;
        sdm_sampled   : out scope_sample_array(0 to 2);
        -- the output current modulator's raw bits with sdm_marker's state (sdm_adc's probe)
        sdm_marker    : in std_logic := '0';
        sdm_raw_bits  : out scope_sample;
        -- the output current's midpoint sampler (sdm_adc): the modulator's period starts in
        sdm_period_start : in boolean := false;
        sdm_midpoint     : out scope_sample;
        -- every measurement for the scaler, raw counts, synchronous to clock: 0..7 the sec mux
        -- positions, 8..15 the pri ones (0..16383), 16..18 the sigma-delta results (signed,
        -- filtered at 20 MHz), 19..21 their means over a switching period, 22..24 those
        -- means taken at sdm_sample's pulses,
        -- each with a strobe the clock a new value is there
        measurements       : out fixed_array(0 to 25);
        measurements_ready : out std_logic_vector(0 to 25);
        -- the ADCs and the muxes
        sec_sclk  : out std_logic;
        sec_cs_n  : out std_logic;
        sec_sdo   : in std_logic;
        sec_mux   : out std_logic_vector(2 downto 0);
        pri_sclk  : out std_logic;
        pri_cs_n  : out std_logic;
        pri_sdo   : in std_logic;
        pri_mux   : out std_logic_vector(2 downto 0);
        -- the sigma-delta modulators: their clock, and data 0 grid_inu, 1 output_inu, 2 dab
        sdm_clock : out std_logic;
        sdm_data  : in std_logic_vector(0 to 2)
    );
end entity ad_subsystem;

architecture rtl of ad_subsystem is
    signal sec_result, pri_result : unsigned(13 downto 0);
    signal sec_pos, pri_pos       : scope_sample_array(0 to 7);
    signal sdm_res, sdm_mean      : scope_sample_array(0 to 2);
    signal sdm_samp               : scope_sample_array(0 to 2);
    signal sdm_sampled_ready      : boolean;
    signal sdm_mid                : scope_sample;
    signal sdm_mid_ready          : boolean;
    signal sec_new, pri_new, sdm_new : boolean;
    signal sec_new_position, pri_new_position : natural range 0 to 7;
    signal bus_from_sec, bus_from_pri, bus_from_sdm : fpga_interconnect_record := init_fpga_interconnect;
begin

    u_sec_adc : entity work.board_adc
    generic map(g_base_address => g_sec_address)
    port map(
        clock      => clock,
        bus_in     => bus_in,
        bus_out    => bus_from_sec,
        latest     => sec_result,
        positions  => sec_pos,
        is_new     => sec_new,
        adc_clock  => adc_clock,
        mux        => sec_mux,
        sclk       => sec_sclk,
        cs_n       => sec_cs_n,
        sdo        => sec_sdo,
        new_position => sec_new_position
    );

    u_pri_adc : entity work.board_adc
    generic map(g_base_address => g_pri_address, g_sample_delay => 5)
    port map(
        clock      => clock,
        bus_in     => bus_in,
        bus_out    => bus_from_pri,
        latest     => pri_result,
        positions  => pri_pos,
        is_new     => pri_new,
        adc_clock  => adc_clock,
        mux        => pri_mux,
        sclk       => pri_sclk,
        cs_n       => pri_cs_n,
        sdo        => pri_sdo,
        new_position => pri_new_position
    );

    u_sdm_adc : entity work.sdm_adc
    generic map(g_base_address => g_sdm_address, g_channels => 3)
    port map(
        clock      => clock,
        bus_in     => bus_in,
        bus_out    => bus_from_sdm,
        results    => sdm_res,
        means      => sdm_mean,
        is_new     => sdm_new,
        sample        => sdm_sample,
        sampled       => sdm_samp,
        sampled_ready => sdm_sampled_ready,
        marker        => sdm_marker,
        raw_bits      => sdm_raw_bits,
        period_start   => sdm_period_start,
        midpoint       => sdm_mid,
        midpoint_ready => sdm_mid_ready,
        adc_clock  => adc_clock,
        sdm_clock  => sdm_clock,
        sdm_data   => sdm_data
    );

    bus_out    <= bus_from_sec and bus_from_pri and bus_from_sdm;
    sec_latest <= to_probe(sec_result);
    pri_latest <= to_probe(pri_result);
    sec_positions <= sec_pos;
    pri_positions <= pri_pos;
    sdm_results   <= sdm_res;
    sdm_means     <= sdm_mean;
    sdm_sampled   <= sdm_samp;
    sdm_midpoint  <= sdm_mid;
    -- 25: the output current's midpoint samples (twice a period)
    measurements(25)       <= resize(signed(sdm_mid), 32);
    measurements_ready(25) <= '1' when sdm_mid_ready else '0';

    -- the measurements for the scaler: the ADC counts as they are (unsigned 0..16383), the
    -- sigma-delta results sign extended; a strobe for the position a result is new for, all
    -- three sigma-delta channels at once
    measurement_channels : for p in 0 to 7 generate
        measurements(p)     <= resize(signed('0' & sec_pos(p)), 32);
        measurements(8 + p) <= resize(signed('0' & pri_pos(p)), 32);
        measurements_ready(p)     <= '1' when sec_new and sec_new_position = p else '0';
        measurements_ready(8 + p) <= '1' when pri_new and pri_new_position = p else '0';
    end generate;
    sdm_channels : for c in 0 to 2 generate
        measurements(16 + c)       <= resize(signed(sdm_res(c)), 32);
        measurements_ready(16 + c) <= '1' when sdm_new else '0';
        -- 19..21: their means over a switching period
        measurements(19 + c)       <= resize(signed(sdm_mean(c)), 32);
        measurements_ready(19 + c) <= '1' when sdm_new else '0';
        -- 22..24: the means at the sample pulses
        measurements(22 + c)       <= resize(signed(sdm_samp(c)), 32);
        measurements_ready(22 + c) <= '1' when sdm_sampled_ready else '0';
    end generate;

end architecture rtl;
