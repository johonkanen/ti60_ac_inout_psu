library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;

-- HyperRAM controller for bring-up and testing.
--
-- The HyperRAM clock is generated in fabric with g_clocks_per_half_ck system clocks
-- per CK half period, so CK = clock / (2*g_clocks_per_half_ck). Command, address and
-- write data change halfway between CK edges (center aligned). Read data is captured
-- by oversampling DQ and RWDS: a byte is taken one sample after each RWDS transition,
-- so read capture does not depend on latency or board delays.
--
-- Fixed latency mode is assumed (mandatory for the dual die W959D8NFYA): data starts
-- on CK edge 4*latency_count + 5, counting the first rising edge after CS# as edge 1.
-- Register writes have zero latency, data on edges 7 and 8.
entity hyperram_controller is
    generic(
        g_clocks_per_half_ck : positive := 4;      -- even, at least 4
        g_reset_low_clocks   : positive := 240;    -- RESET# low time, tRP > 200 ns
        g_power_up_clocks    : positive := 24000   -- wait after reset, tVCS > 150 us
    );
    port(
        clock        : in std_logic;
        hyperram_in  : in hyperram_in_record;
        hyperram_out : out hyperram_out_record;
        fpga_out     : out hyperram_fpga_out_record;
        fpga_in      : in hyperram_fpga_in_record
    );
end entity hyperram_controller;

architecture rtl of hyperram_controller is

    constant half_ck          : natural := g_clocks_per_half_ck;
    constant data_change_phase : natural := half_ck/2 - 1;
    constant edge_phase        : natural := half_ck - 1;
    constant recovery_clocks   : natural := 2*half_ck + 4;  -- tRWR + tCSHI
    constant read_timeout_edges : natural := 16;

    type controller_states is (reset_low, power_up, idle, transaction, finish, recovery);
    signal state : controller_states := reset_low;

    signal counter : natural range 0 to g_power_up_clocks := 0;
    signal phase   : natural range 0 to half_ck-1 := 0;
    signal slot    : natural range 0 to 255 := 0;

    signal header          : hyperram_header := (others => '0');
    signal is_read         : boolean := false;
    signal is_register     : boolean := false;
    signal burst_length    : natural range 1 to 64 := 1;
    signal first_data_edge : natural range 0 to 63 := 0;
    signal last_data_edge  : natural range 0 to 255 := 0;

    signal ck_r      : std_logic := '0';
    signal cs_n_r    : std_logic := '1';
    signal reset_n_r : std_logic := '0';
    signal dq_r      : std_logic_vector(7 downto 0) := (others => '0');
    signal dq_oe_r   : std_logic := '0';
    signal rwds_r    : std_logic := '0';
    signal rwds_oe_r : std_logic := '0';

    signal next_word    : std_logic_vector(15 downto 0) := (others => '0');
    signal current_word : std_logic_vector(15 downto 0) := (others => '0');
    signal words_sent   : natural range 0 to 64 := 0;

    signal rwds_sample      : std_logic_vector(1 downto 0) := (others => '0');
    signal dq_sample        : std_logic_vector(7 downto 0) := (others => '0');
    signal rwds_rose        : boolean := false;
    signal rwds_fell        : boolean := false;
    signal capture_is_enabled : boolean := false;
    signal high_byte        : std_logic_vector(7 downto 0) := (others => '0');
    signal high_byte_is_valid : boolean := false;
    signal words_received   : natural range 0 to 64 := 0;

    signal ready             : boolean := false;
    signal write_data_request : boolean := false;
    signal read_data         : std_logic_vector(15 downto 0) := (others => '0');
    signal read_data_ready   : boolean := false;
    signal done              : boolean := false;
    signal timed_out         : boolean := false;

begin

    hyperram_out <= (
        is_ready             => ready,
        write_data_requested => write_data_request,
        read_data            => read_data,
        read_data_is_ready   => read_data_ready,
        transaction_is_done  => done,
        read_timed_out       => timed_out);

    fpga_out <= (
        ck       => ck_r,
        cs_n     => cs_n_r,
        reset_n  => reset_n_r,
        dq_out   => dq_r,
        dq_oe    => dq_oe_r,
        rwds_out => rwds_r,
        rwds_oe  => rwds_oe_r);

    controller : process(clock)
        variable edge : natural range 0 to 256;
    begin
        if rising_edge(clock) then

            write_data_request <= false;
            read_data_ready    <= false;
            done               <= false;

            ------------------------------------------------------------
            -- read capture, a byte is taken one sample after an RWDS transition
            rwds_sample <= rwds_sample(0) & fpga_in.rwds_in;
            dq_sample   <= fpga_in.dq_in;
            rwds_rose   <= rwds_sample = "01";
            rwds_fell   <= rwds_sample = "10";

            if capture_is_enabled and words_received < burst_length then
                if rwds_rose then
                    high_byte          <= dq_sample;
                    high_byte_is_valid <= true;
                end if;
                if rwds_fell and high_byte_is_valid then
                    read_data          <= high_byte & dq_sample;
                    read_data_ready    <= true;
                    words_received     <= words_received + 1;
                    high_byte_is_valid <= false;
                end if;
            end if;

            ------------------------------------------------------------
            if hyperram_in.write_data_valid then
                next_word <= hyperram_in.write_data;
            end if;

            ------------------------------------------------------------
            CASE state is
                WHEN reset_low =>
                    reset_n_r <= '0';
                    cs_n_r    <= '1';
                    if counter < g_reset_low_clocks then
                        counter <= counter + 1;
                    else
                        counter <= 0;
                        state   <= power_up;
                    end if;

                WHEN power_up =>
                    reset_n_r <= '1';
                    if counter < g_power_up_clocks then
                        counter <= counter + 1;
                    else
                        counter <= 0;
                        ready   <= true;
                        state   <= idle;
                    end if;

                WHEN idle =>
                    if hyperram_in.command /= none then
                        ready        <= false;
                        header       <= build_hyperram_header(
                                            read_transaction => hyperram_in.command = read,
                                            register_space   => hyperram_in.register_space,
                                            address          => hyperram_in.address);
                        is_read      <= hyperram_in.command = read;
                        is_register  <= hyperram_in.register_space;
                        burst_length <= hyperram_in.burst_length;

                        if hyperram_in.command = write and hyperram_in.register_space then
                            first_data_edge <= 7;
                            last_data_edge  <= 7 + 2*hyperram_in.burst_length - 1;
                        else
                            first_data_edge <= 4*hyperram_in.latency_count + 5;
                            last_data_edge  <= 4*hyperram_in.latency_count + 5 + 2*hyperram_in.burst_length - 1;
                        end if;

                        if hyperram_in.command = write then
                            write_data_request <= true;
                        end if;

                        cs_n_r             <= '0';
                        ck_r               <= '0';
                        phase              <= 0;
                        slot               <= 0;
                        words_sent         <= 0;
                        words_received     <= 0;
                        high_byte_is_valid <= false;
                        capture_is_enabled <= false;
                        timed_out          <= false;
                        state              <= transaction;
                    end if;

                WHEN transaction =>
                    if phase < half_ck-1 then
                        phase <= phase + 1;
                    else
                        phase <= 0;
                        slot  <= slot + 1;
                    end if;

                    -- edge number of the next CK transition
                    edge := slot + 1;

                    if is_read and edge >= first_data_edge - 1 then
                        capture_is_enabled <= true;
                    end if;

                    -- outputs change halfway before the next CK edge
                    if phase = data_change_phase then
                        if edge <= 6 then
                            dq_r    <= get_header_byte(header, edge-1);
                            dq_oe_r <= '1';

                        elsif is_read then
                            dq_oe_r <= '0';

                        elsif edge >= first_data_edge and edge <= last_data_edge then
                            dq_oe_r <= '1';
                            if (edge - first_data_edge) mod 2 = 0 then
                                dq_r         <= next_word(15 downto 8);
                                current_word <= next_word;
                                words_sent   <= words_sent + 1;
                                if words_sent + 1 < burst_length then
                                    write_data_request <= true;
                                end if;
                            else
                                dq_r <= current_word(7 downto 0);
                            end if;
                            if not is_register then
                                rwds_r    <= '0';
                                rwds_oe_r <= '1';
                            end if;

                        elsif edge >= first_data_edge - 2 and not is_register then
                            -- write data mask preamble, RWDS low before the first data byte
                            dq_oe_r   <= '0';
                            rwds_r    <= '0';
                            rwds_oe_r <= '1';

                        else
                            dq_oe_r <= '0';
                        end if;
                    end if;

                    if phase = edge_phase then
                        if not is_read then
                            ck_r <= not ck_r;
                            if edge = last_data_edge then
                                state <= finish;
                            end if;
                        else
                            -- keep clocking until the whole burst has been captured
                            if words_received < burst_length and edge <= last_data_edge + read_timeout_edges then
                                ck_r <= not ck_r;
                            elsif ck_r = '1' then
                                ck_r <= '0';
                            else
                                timed_out <= words_received < burst_length;
                                state     <= finish;
                            end if;
                        end if;
                    end if;

                WHEN finish =>
                    -- CS# goes high half a CK period after the last falling CK edge
                    if counter < half_ck/2 then
                        counter <= counter + 1;
                    else
                        counter            <= 0;
                        cs_n_r             <= '1';
                        dq_oe_r            <= '0';
                        rwds_oe_r          <= '0';
                        capture_is_enabled <= false;
                        done               <= true;
                        state              <= recovery;
                    end if;

                WHEN recovery =>
                    if counter < recovery_clocks then
                        counter <= counter + 1;
                    else
                        counter <= 0;
                        ready   <= true;
                        state   <= idle;
                    end if;
            end CASE;

        end if; -- rising_edge
    end process controller;

end architecture rtl;
