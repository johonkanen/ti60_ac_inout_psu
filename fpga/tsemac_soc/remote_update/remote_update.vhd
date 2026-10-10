library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

-- The Titanium's internal reconfiguration interface (AN 010) on the register bus, for
-- updating the FPGA over the network without JTAG.
--
-- The flash holds a golden image at address 0, which the FPGA always loads at power-up and
-- which is never written remotely, and an application image (image 1) that the software
-- replaces. Both are this design, built with g_role 0 (golden) or 1 (application): the
-- software reads the role to know which one is running, and from the golden image switches
-- to the application image. If that image fails its CRC check, the FPGA retries it and falls
-- back to the golden image with cfg_ERROR high (AN 010: six tries, up to 41 s).
--
-- register map, offsets from g_base_address
--   0  R  0x5255_00RR: "RU" and the role RR, 0 golden, 1 application
--   1  R  bit0 cfg_ERROR: the last reconfiguration failed and the golden image was restored
--   2  W  0x5243_46_0I ("RCF" and I): reconfigure from image I (0..3). Any other value is
--         ignored. The select goes out first, cfg_ENA high 0.5 us later, cfg_CONFIG high
--         2 us after that, and they stay: the FPGA reloads, this design is gone.
--
-- cfg_CBSEL and cfg_ENA are latched on the interface's own clock (at most 100 MHz, the
-- internal oscillator here), cfg_CONFIG is asynchronous: all three are levels that are steady
-- for many of its clocks before they matter.
entity remote_update is
    generic(
        g_base_address : natural := 800;
        g_role         : natural range 0 to 255
    );
    port(
        clock      : in std_logic;
        bus_in     : in fpga_interconnect_record;
        bus_out    : out fpga_interconnect_record;
        cfg_CBSEL  : out std_logic_vector(1 downto 0);
        cfg_ENA    : out std_logic;
        cfg_CONFIG : out std_logic;
        cfg_ERROR  : in std_logic
    );
end entity remote_update;

architecture rtl of remote_update is
    constant command_magic : std_logic_vector(23 downto 0) := x"524346";   -- "RCF"
    constant select_clocks : natural := 64;     -- 0.5 us at 125 MHz
    constant enable_clocks : natural := 256;    -- 2 us

    type states is (idle, selecting, enabling, configuring);
    signal state     : states := idle;
    signal timer     : natural range 0 to enable_clocks := 0;
    signal image     : std_logic_vector(1 downto 0) := "00";
    signal error_sync : std_logic_vector(1 downto 0) := "00";
begin

    process(clock)
        variable data : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clock) then
            init_bus(bus_out);
            error_sync <= error_sync(0) & cfg_ERROR;
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 0,
                x"5255" & x"00" & std_logic_vector(to_unsigned(g_role, 8)));
            connect_read_only_data_to_address(bus_in, bus_out, g_base_address + 1,
                (31 downto 1 => '0') & error_sync(1));

            if write_is_requested_to_address(bus_in, g_base_address + 2) and state = idle then
                data := get_slv_data(bus_in);
                if data(31 downto 8) = command_magic and data(7 downto 2) = "000000" then
                    image <= data(1 downto 0);
                    timer <= 0;
                    state <= selecting;
                end if;
            end if;

            CASE state is
                WHEN selecting =>
                    if timer = select_clocks then
                        timer <= 0;
                        state <= enabling;
                    else
                        timer <= timer + 1;
                    end if;
                WHEN enabling =>
                    if timer = enable_clocks then
                        state <= configuring;
                    else
                        timer <= timer + 1;
                    end if;
                WHEN others => null;
            end CASE;
        end if;
    end process;

    cfg_CBSEL  <= image;
    cfg_ENA    <= '1' when state = enabling or state = configuring else '0';
    cfg_CONFIG <= '1' when state = configuring else '0';

end architecture rtl;
