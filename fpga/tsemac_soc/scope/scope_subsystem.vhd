library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;
    use work.scope_pkg.all;

-- The oscilloscope of board_test (source/scope) for the SoC of sapphire_subsystem:
--   a slave of the register bus at its board_test addresses, 600..
--   the deep capture streams into the scope DMA (scope_axis_writer), which writes a
--   circular buffer in the HyperRAM that the CPU owns
-- Everything runs on one clock, the SoC's peripheral clock, which is also the sample clock.
entity scope_subsystem is
    port(
        clock     : in std_logic;
        bus_in    : in fpga_interconnect_record;
        bus_out   : out fpga_interconnect_record;
        -- to the scope DMA's stream input
        tvalid    : out std_logic;
        tready    : in std_logic;
        tdata     : out std_logic_vector(63 downto 0);
        tkeep     : out std_logic_vector(7 downto 0);
        tlast     : out std_logic;
        -- probes, synchronous to clock: the status lanes and the ADCs (ad_subsystem); of the
        -- muxed ADCs only the converted positions (sec 0..2, pri 0, 1, 3)
        status        : in std_logic_vector(15 downto 0);
        sec_positions : in scope_sample_array(0 to 7);
        pri_positions : in scope_sample_array(0 to 7);
        sdm_results   : in scope_sample_array(0 to 2);
        sdm_means     : in scope_sample_array(0 to 2);
        sdm_sampled   : in scope_sample_array(0 to 2);
        observer_probes : in scope_sample_array(0 to 3);
        control_probes  : in scope_sample_array(0 to 1);
        sdm_midpoint    : in scope_sample;
        dhb_probe     : in scope_sample;
        output_probe  : in scope_sample;
        input_probe   : in scope_sample
    );
end entity scope_subsystem;

architecture rtl of scope_subsystem is

    constant scope_base : natural := work.fpga_registers_pkg.scope_registers.base;

    -- probes: add a signal with a line below and its entry in probe_table
    constant probe_count : natural := 26;
    signal probes        : scope_sample_array(0 to probe_count-1) := (others => (others => '0'));
    -- names: 16 characters at most
    constant probe_table : probe_table_array(0 to probe_count-1) := (
        0  => probe_entry("counter 15:0", unsigned_probe),
        1  => probe_entry("counter 31:16", unsigned_probe),
        2  => probe_entry("status", digital_probe),
        3  => probe_entry("sec 0 vdc_out", unsigned_probe),
        4  => probe_entry("sec 1 vac_out", unsigned_probe),
        5  => probe_entry("sec 2 vfilt_out", unsigned_probe),
        6  => probe_entry("pri 0 vac_in", unsigned_probe),
        7  => probe_entry("pri 1 vdc_in", unsigned_probe),
        8  => probe_entry("pri 3 vfilt_in", unsigned_probe),
        9  => probe_entry("sdm iac_in", signed_probe),
        10 => probe_entry("sdm iac_out", signed_probe),
        11 => probe_entry("sdm i_dab", signed_probe),
        12 => probe_entry("dhb test", digital_probe),
        13 => probe_entry("output test", digital_probe),
        14 => probe_entry("sdm iac_in mean", signed_probe),
        15 => probe_entry("sdm iac_out mean", signed_probe),
        16 => probe_entry("sdm i_dab mean", signed_probe),
        17 => probe_entry("iac_out sampled", signed_probe),
        18 => probe_entry("observer i", signed_probe),
        19 => probe_entry("observer v", signed_probe),
        20 => probe_entry("observer du", signed_probe),
        21 => probe_entry("observer iload", signed_probe),
        22 => probe_entry("control i_ref", signed_probe),
        23 => probe_entry("control u_ref", signed_probe),
        24 => probe_entry("iac_out midpoint", signed_probe),
        25 => probe_entry("input test", digital_probe));
    signal counter       : unsigned(31 downto 0) := (others => '0');

    signal capture_reset, capture_write, capture_flush : boolean;
    signal capture_is_full, capture_is_ready           : boolean;
    signal capture_data    : std_logic_vector(127 downto 0);
    signal capture_written : unsigned(31 downto 0);

begin

    process(clock)
    begin
        if rising_edge(clock) then
            counter <= counter + 1;
        end if;
    end process;
    probes(0)  <= to_probe(counter(15 downto 0));
    probes(1)  <= to_probe(counter(31 downto 16));
    probes(2)  <= status;
    -- the muxed ADCs' converted positions only: sec 0..2, pri 0, 1 and 3
    probes(3)  <= sec_positions(0);
    probes(4)  <= sec_positions(1);
    probes(5)  <= sec_positions(2);
    probes(6)  <= pri_positions(0);
    probes(7)  <= pri_positions(1);
    probes(8)  <= pri_positions(3);
    probes(9 to 11)  <= sdm_results;
    probes(12) <= dhb_probe;        -- lane 0 a burst, 1 the primary's high side, 2 its low side
    probes(13) <= output_probe;     -- lane 0 running, 1 stepped, 2 and 3 the legs' high sides
    probes(14 to 16) <= sdm_means;  -- over a switching period
    probes(17) <= sdm_sampled(1);   -- the output current's, at the modulator's pulses
    probes(18 to 21) <= observer_probes;   -- 256 counts an ampere, 64 a volt
    probes(22 to 23) <= control_probes;    -- 256 counts an ampere, 64 a volt
    probes(24) <= sdm_midpoint;     -- the output current at the middles of its rise and fall
    probes(25) <= input_probe;      -- lane 0 running, 1 the switch on, 2 stopped at the overvoltage

    u_scope : entity work.scope_capture
    generic map(g_base_address => scope_base, g_depth => 4096, g_deep_records => 2**22,
                g_probes => probe_count, g_probe_table => probe_table)
    port map(
        clock            => clock,
        bus_in           => bus_in,
        bus_out          => bus_out,
        probes           => probes,
        read_address     => 0,
        read_data        => open,
        capture_reset    => capture_reset,
        capture_write    => capture_write,
        capture_data     => capture_data,
        capture_flush    => capture_flush,
        capture_is_full  => capture_is_full,
        capture_is_ready => capture_is_ready,
        capture_written  => capture_written
    );

    u_writer : entity work.scope_axis_writer
    port map(
        clock            => clock,
        capture_reset    => capture_reset,
        capture_write    => capture_write,
        capture_data     => capture_data,
        capture_flush    => capture_flush,
        capture_is_full  => capture_is_full,
        capture_is_ready => capture_is_ready,
        capture_written  => capture_written,
        tvalid           => tvalid,
        tready           => tready,
        tdata            => tdata,
        tkeep            => tkeep,
        tlast            => tlast
    );

end architecture rtl;
