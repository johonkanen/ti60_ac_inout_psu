library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

library vunit_lib;
    context vunit_lib.vunit_context;

-- tx_checksum_inserter with frames of every kind it handles and some it must leave alone:
-- TCP, UDP and ICMP over IPv4 with and without IP options, short UDP padded to 60 bytes,
-- an IPv4 fragment (header checksum only) and ARP (untouched). The checksum fields go in
-- holding garbage. Input gaps, bytes with tkeep low and output backpressure are random.
-- Frame n is a function of n, so the checker rebuilds what the driver sent and computes
-- the checksums itself.
entity tx_checksum_inserter_tb is
    generic(runner_cfg : string);
end entity tx_checksum_inserter_tb;

architecture sim of tx_checksum_inserter_tb is

    constant frame_count : natural := 400;

    type byte_array is array (natural range <>) of natural range 0 to 255;
    subtype frame_bytes is byte_array(0 to 1517);
    type frame_type is record
        data   : frame_bytes;
        length : natural;
    end record;

    -- small deterministic generator
    function hash(a, b : natural) return natural is
        variable x : unsigned(31 downto 0);
    begin
        x := resize(to_unsigned(a, 32) * to_unsigned(2654435, 32), 32) + to_unsigned(b * 40503 + 12345, 32);
        x := x xor shift_right(x, 13);
        x := resize(x * to_unsigned(1103515, 32), 32);
        x := x xor shift_right(x, 16);
        return to_integer(x(30 downto 0));
    end function;

    function ones_sum(f : frame_bytes; first, last_excl : natural; start : natural) return natural is
        -- 16 bit words from offset first (even relative to start), last odd byte padded
        variable s : natural := start;
    begin
        for i in first to last_excl - 1 loop
            if (i - first) mod 2 = 0 then
                s := s + f(i) * 256;
            else
                s := s + f(i);
            end if;
        end loop;
        while s > 65535 loop
            s := (s mod 65536) + s / 65536;
        end loop;
        return s;
    end function;

    -- the frame as sent, checksum fields holding garbage
    function make_frame(n : natural) return frame_type is
        variable f : frame_type;
        variable kind, ihl, payload, l4, ip_len, proto : natural;
    begin
        f.data := (others => 0);
        kind := n mod 6;
        for i in 0 to 11 loop
            f.data(i) := hash(n, i) mod 256;        -- MAC addresses
        end loop;
        if kind = 3 then                             -- ARP
            f.data(12) := 16#08#; f.data(13) := 16#06#;
            f.length := 42;
            for i in 14 to 41 loop f.data(i) := hash(n, i) mod 256; end loop;
            return f;
        end if;
        f.data(12) := 16#08#; f.data(13) := 16#00#;
        ihl := 5;
        if n mod 7 = 0 then ihl := 6; end if;        -- with an IP option
        case kind is
            when 0 => proto := 6;  l4 := 20;         -- TCP
            when 1 | 4 | 5 => proto := 17; l4 := 8;  -- UDP
            when others => proto := 1; l4 := 8;      -- ICMP echo
        end case;
        payload := hash(n, 99) mod (1480 - 4 * ihl - l4 + 1);
        if kind = 5 then payload := hash(n, 98) mod 10; end if;   -- short, padded
        if n mod 11 = 0 then payload := 0; end if;
        ip_len := 4 * ihl + l4 + payload;
        f.data(14) := 16#40# + ihl;
        f.data(16) := ip_len / 256; f.data(17) := ip_len mod 256;
        f.data(18) := hash(n, 18) mod 256; f.data(19) := hash(n, 19) mod 256;
        if kind = 4 then f.data(20) := 16#20#; end if;      -- more fragments
        f.data(22) := 64; f.data(23) := proto;
        f.data(24) := 16#a5#; f.data(25) := 16#5a#;         -- garbage checksum
        for i in 26 to 13 + 4 * ihl + l4 + payload loop
            f.data(i) := hash(n, i) mod 256;
        end loop;
        if ihl = 6 then f.data(34) := 1; f.data(35) := 1; f.data(36) := 1; f.data(37) := 0; end if;
        f.length := 14 + ip_len;
        if f.length < 60 then
            for i in f.length to 59 loop f.data(i) := hash(n, 500 + i) mod 256; end loop;  -- padding
            f.length := 60;
        end if;
        return f;
    end function;

    -- the frame as it should come out
    function expected_frame(n : natural) return frame_type is
        variable f : frame_type;
        variable ihl, ip_len, proto, l4s, at, s : natural;
        variable fragment : boolean;
    begin
        f := make_frame(n);
        if not (f.data(12) = 8 and f.data(13) = 0) then
            return f;
        end if;
        ihl := f.data(14) mod 16;
        ip_len := f.data(16) * 256 + f.data(17);
        proto := f.data(23);
        f.data(24) := 0; f.data(25) := 0;
        s := 65535 - ones_sum(f.data, 14, 14 + 4 * ihl, 0);
        f.data(24) := s / 256; f.data(25) := s mod 256;
        fragment := f.data(20) mod 64 /= 0 or f.data(21) /= 0;
        if fragment then
            return f;
        end if;
        l4s := 14 + 4 * ihl;
        case proto is
            when 6 => at := l4s + 16;
            when 17 => at := l4s + 6;
            when others => at := l4s + 2;
        end case;
        f.data(at) := 0; f.data(at + 1) := 0;
        if proto = 1 then
            s := ones_sum(f.data, l4s, 14 + ip_len, 0);
        else
            s := ones_sum(f.data, 26, 34, proto + ip_len - 4 * ihl);
            s := ones_sum(f.data, l4s, 14 + ip_len, s);
        end if;
        s := 65535 - s;
        if proto = 17 and s = 0 then s := 65535; end if;
        f.data(at) := s / 256; f.data(at + 1) := s mod 256;
        return f;
    end function;

    signal clock : std_logic := '0';
    signal reset : std_logic := '1';
    signal s_tvalid, s_tready, s_tkeep, s_tlast : std_logic := '0';
    signal s_tdata : std_logic_vector(7 downto 0) := (others => '0');
    signal s_tdest : std_logic_vector(3 downto 0) := (others => '0');
    signal m_tvalid, m_tready, m_tlast : std_logic := '0';
    signal m_tdata : std_logic_vector(7 downto 0);
    signal m_tdest : std_logic_vector(3 downto 0);
    signal checked : natural := 0;
    type time_array is array (natural range <>) of time;
    signal arrived : time_array(0 to frame_count - 1) := (others => 1 sec);

begin

    clock <= not clock after 4 ns;

    dut : entity work.tx_checksum_inserter
    port map(clock => clock, reset => reset,
             s_tvalid => s_tvalid, s_tready => s_tready, s_tdata => s_tdata, s_tkeep => s_tkeep,
             s_tdest => s_tdest, s_tlast => s_tlast,
             m_tvalid => m_tvalid, m_tready => m_tready, m_tdata => m_tdata, m_tdest => m_tdest,
             m_tlast => m_tlast);

    driver : process
        variable f : frame_type;
        variable i, r : natural;
    begin
        wait until rising_edge(clock);
        wait until rising_edge(clock);
        reset <= '0';
        for n in 0 to frame_count - 1 loop
            f := make_frame(n);
            i := 0;
            r := 0;
            while i < f.length loop
                r := r + 1;
                -- the first 100 frames back to back, then gaps and dropped bytes
                if n >= 100 and hash(n, 3000 + r) mod 5 = 0 then
                    s_tvalid <= '0';
                    wait until rising_edge(clock);
                    next;
                end if;
                s_tvalid <= '1';
                s_tdest  <= std_logic_vector(to_unsigned(n mod 16, 4));
                if n >= 100 and hash(n, 6000 + r) mod 9 = 0 then
                    s_tkeep <= '0';                 -- dropped
                    s_tdata <= x"ee";
                    s_tlast <= '0';
                    wait until rising_edge(clock) and s_tready = '1';
                    next;
                end if;
                s_tkeep <= '1';
                s_tdata <= std_logic_vector(to_unsigned(f.data(i), 8));
                s_tlast <= '1' when i = f.length - 1 else '0';
                wait until rising_edge(clock) and s_tready = '1';
                if i = f.length - 1 then
                    arrived(n) <= now;
                end if;
                i := i + 1;
            end loop;
            s_tvalid <= '0';
            s_tlast  <= '0';
        end loop;
        wait;
    end process;

    ready : process(clock)
        variable k : natural := 0;
    begin
        if rising_edge(clock) then
            k := k + 1;
            if checked < 100 then
                m_tready <= '1';                     -- full speed first
            else
                m_tready <= '1' when hash(k, 7) mod 3 /= 0 else '0';
            end if;
        end if;
    end process;

    checker : process
        variable f : frame_type;
    begin
        test_runner_setup(runner, runner_cfg);
        for n in 0 to frame_count - 1 loop
            f := expected_frame(n);
            for i in 0 to f.length - 1 loop
                wait until rising_edge(clock) and m_tvalid = '1' and m_tready = '1';
                check_equal(to_integer(unsigned(m_tdata)), f.data(i),
                            "frame " & integer'image(n) & " byte " & integer'image(i));
                check_equal(m_tlast, i = f.length - 1,
                            "frame " & integer'image(n) & " tlast at byte " & integer'image(i));
                check_equal(to_integer(unsigned(m_tdest)), n mod 16, "tdest");
            end loop;
            checked <= n + 1;
        end loop;
        info(integer'image(frame_count) & " frames checked");
        test_runner_cleanup(runner);
    end process;

    -- throughput: while the output takes a byte every clock (the first 100 frames), a frame
    -- that is all in by the time the previous one has gone out follows it within a few clocks
    throughput : process
        variable last_out : time := 0 ns;
        variable gap, worst, counted : natural := 0;
    begin
        for n in 0 to 99 loop
            wait until rising_edge(clock) and m_tvalid = '1' and m_tready = '1';
            if n > 0 and arrived(n) < last_out then
                gap := (now - last_out) / 8 ns;
                counted := counted + 1;
                if gap > worst then worst := gap; end if;
            end if;
            wait until rising_edge(clock) and m_tvalid = '1' and m_tready = '1' and m_tlast = '1';
            last_out := now;
        end loop;
        info("frames waiting when the one before ended: " & integer'image(counted)
             & ", longest gap " & integer'image(worst) & " clocks");
        check(counted > 30, "too few frames waiting to tell");
        check(worst <= 8, "gap between frames");
        wait;
    end process;

    test_runner_watchdog(runner, 20 ms);

end architecture sim;
