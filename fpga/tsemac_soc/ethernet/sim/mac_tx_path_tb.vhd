library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;
    use ieee.math_real.all;

library vunit_lib;
    context vunit_lib.vunit_context;

    use work.ethernet_pkg.all;

-- The transmit path of ethernet_subsystem with ethernet_mac: tx_checksum_inserter in front of
-- ethernet_mac_tx, which needs every frame without gaps once it has taken its first byte. The
-- frames come in with random gaps, as from the DMA; each must leave on GMII whole (no frame cut
-- short) and with its bytes as sent (EtherType 0x88B5: the inserter leaves them alone).
entity mac_tx_path_tb is
    generic(runner_cfg : string);
end entity mac_tx_path_tb;

architecture sim of mac_tx_path_tb is

    constant frame_count : natural := 60;

    signal clock : std_logic := '0';
    signal reset : std_logic := '1';

    signal s_tvalid, s_tlast : std_logic := '0';
    signal s_tready : std_logic;
    signal s_tdata  : std_logic_vector(7 downto 0) := (others => '0');
    signal m_tvalid, m_tready, m_tlast : std_logic;
    signal m_tdata  : std_logic_vector(7 downto 0);

    signal gmii : gmii_record;
    signal frame_sent, frame_error : boolean;
    signal frames_sent, frames_cut, frames_checked : natural := 0;
    signal byte_errors : natural := 0;

    function frame_length(n : natural) return natural is
    begin
        return 60 + (n * 97) mod 1455;
    end function;

    function frame_byte(n, i : natural) return byte is
    begin
        if i = 12 then
            return x"88";
        elsif i = 13 then
            return x"B5";
        end if;
        return std_logic_vector(to_unsigned((i * 13 + n * 7) mod 256, 8));
    end function;

begin

    clock <= not clock after 4 ns;

    u_inserter : entity work.tx_checksum_inserter
    port map(
        clock     => clock
        ,reset    => reset
        ,s_tvalid => s_tvalid
        ,s_tready => s_tready
        ,s_tdata  => s_tdata
        ,s_tkeep  => '1'
        ,s_tdest  => "0000"
        ,s_tlast  => s_tlast
        ,m_tvalid => m_tvalid
        ,m_tready => m_tready
        ,m_tdata  => m_tdata
        ,m_tdest  => open
        ,m_tlast  => m_tlast
    );

    u_mac_tx : entity work.ethernet_mac_tx
    port map(
        clock        => clock
        ,enable      => true
        ,s_tvalid    => m_tvalid
        ,s_tready    => m_tready
        ,s_tdata     => m_tdata
        ,s_tlast     => m_tlast
        ,gmii_out    => gmii
        ,frame_sent  => frame_sent
        ,frame_error => frame_error
    );

    counters : process(clock)
    begin
        if rising_edge(clock) then
            if frame_sent then frames_sent <= frames_sent + 1; end if;
            if frame_error then frames_cut <= frames_cut + 1; end if;
        end if;
    end process;

    -- the line: after the SFD the frame's bytes, then its FCS, then idle
    line : process
        variable n, i : natural := 0;
    begin
        wait until rising_edge(clock) and gmii.is_valid and gmii.data = start_of_frame_byte;
        i := 0;
        loop
            wait until rising_edge(clock);
            exit when not gmii.is_valid;
            if i < frame_length(n) and gmii.data /= frame_byte(n, i) then
                byte_errors <= byte_errors + 1;
            end if;
            i := i + 1;
        end loop;
        check_equal(i, frame_length(n) + 4, "frame " & integer'image(n) & " length on the line");
        frames_checked <= n + 1;
        n := n + 1;
    end process;

    main : process
        variable seed1, seed2 : positive := 7;
        variable r : real;
    begin
        test_runner_setup(runner, runner_cfg);
        wait for 50 ns;
        wait until rising_edge(clock);
        reset <= '0';

        while test_suite loop
            if run("frames_with_input_gaps_leave_whole") then
                for n in 0 to frame_count-1 loop
                    for i in 0 to frame_length(n)-1 loop
                        uniform(seed1, seed2, r);
                        if r < 0.2 then
                            s_tvalid <= '0';
                            wait until rising_edge(clock);
                        end if;
                        s_tvalid <= '1';
                        s_tdata  <= frame_byte(n, i);
                        s_tlast  <= '1' when i = frame_length(n)-1 else '0';
                        loop
                            wait until rising_edge(clock);
                            exit when s_tready = '1';
                        end loop;
                    end loop;
                end loop;
                s_tvalid <= '0';
                wait until frames_checked = frame_count for 1 ms;
                check_equal(frames_checked, frame_count, "frames on the line");
                check_equal(frames_sent, frame_count, "frames sent");
                check_equal(frames_cut, 0, "no frame cut short");
                check_equal(byte_errors, 0, "bytes as sent");
            end if;
        end loop;
        test_runner_cleanup(runner);
    end process;

end architecture sim;
