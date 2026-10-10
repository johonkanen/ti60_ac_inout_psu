library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;
    use work.udp_stream_pkg.all;

-- HyperRAM data source of udp_stream_sender. The datagrams the sender announces through
-- prefetch are read from the HyperRAM in order into two datagram buffers, the oldest
-- announced datagram is read out of its buffer by the sender.
--
-- Stream word w is HyperRAM words 2w and 2w+1, the first in bits 31:16, so a 128 bit capture
-- record is stream words 4r..4r+3. Reads are split at 512 word boundaries, so a burst never
-- crosses the die boundary.
-- read_data follows read_address after two clocks, which udp_stream_sender reads three
-- clocks ahead (g_read_latency 3).
entity hyperram_stream_source is
    port(
        clock         : in std_logic;
        prefetch      : in stream_prefetch_record;
        data_is_ready : out boolean;
        read_address  : in natural;
        read_data     : out std_logic_vector(31 downto 0);

        hyperram_in   : out hyperram_in_record;
        hyperram_out  : in hyperram_out_record
    );
end entity hyperram_stream_source;

architecture rtl of hyperram_stream_source is

    constant buffer_words : natural := 512;   -- 32 bit words per datagram buffer
    type buffer_memory is array (0 to 2*buffer_words-1) of std_logic_vector(31 downto 0);
    signal memory : buffer_memory := (others => (others => '0'));

    type first_word_array is array (0 to 1) of unsigned(30 downto 0);
    type words_array is array (0 to 1) of natural range 0 to stream_words_per_datagram;
    signal slot_first : first_word_array := (others => (others => '0'));
    signal slot_words : words_array := (others => 0);

    -- announced datagrams are slots head, head+1; the first `filled` of them are in memory
    signal head      : natural range 0 to 1 := 0;
    signal announced : natural range 0 to 2 := 0;
    signal filled    : natural range 0 to 2 := 0;

    type states is (idle, issue, wait_done);
    signal state       : states := idle;
    signal fill_slot   : natural range 0 to 1 := 0;
    signal address     : natural range 0 to 2**25-1 := 0;     -- next HyperRAM word
    signal remaining   : natural range 0 to 2*stream_words_per_datagram := 0;
    signal burst       : natural range 0 to 512 := 0;
    signal position    : natural range 0 to 2*buffer_words := 0;   -- HyperRAM words received
    signal high_word   : std_logic_vector(15 downto 0) := (others => '0');
    signal discarding  : boolean := false;    -- the stream was restarted during this fill

    signal read_index  : natural range 0 to 2*buffer_words-1 := 0;
    signal request     : hyperram_in_record := init_hyperram_in;

begin

    data_is_ready <= filled > 0;
    hyperram_in   <= request;

    process(clock)
        variable v_head      : natural range 0 to 1;
        variable v_announced : natural range 0 to 2;
        variable v_filled    : natural range 0 to 2;
        variable n           : natural range 0 to 512;
    begin
        if rising_edge(clock) then
            init_hyperram(request);
            v_head      := head;
            v_announced := announced;
            v_filled    := filled;

            -- read data into the buffer of the slot being filled
            if hyperram_read_data_is_ready(hyperram_out) and state = wait_done then
                if position mod 2 = 0 then
                    high_word <= get_hyperram_read_data(hyperram_out);
                else
                    memory(fill_slot*buffer_words + position/2) <= high_word & get_hyperram_read_data(hyperram_out);
                end if;
                position <= position + 1;
            end if;

            CASE state is
                WHEN idle =>
                    null;

                WHEN issue =>
                    if hyperram_is_ready(hyperram_out) then
                        -- up to the next 512 word boundary
                        n := 512 - address mod 512;
                        if remaining < n then
                            n := remaining;
                        end if;
                        burst <= n;
                        request_hyperram_read(request, address, n);
                        state <= wait_done;
                    end if;

                WHEN wait_done =>
                    if hyperram_transaction_is_done(hyperram_out) then
                        address   <= (address + burst) mod 2**25;
                        remaining <= remaining - burst;
                        if remaining > burst then
                            state <= issue;
                        else
                            if not discarding then
                                v_filled := v_filled + 1;
                            end if;
                            discarding <= false;
                            state      <= idle;
                        end if;
                    end if;
            end CASE;

            if prefetch.datagram_done and v_announced > 0 then
                v_head      := 1 - v_head;
                v_announced := v_announced - 1;
                if v_filled > 0 then
                    v_filled := v_filled - 1;
                end if;
            end if;

            if prefetch.reset then
                v_head      := 0;
                v_announced := 0;
                v_filled    := 0;
                if state /= idle then
                    discarding <= true;
                end if;
            end if;

            if prefetch.request and v_announced < 2 then
                slot_first((v_head + v_announced) mod 2) <= prefetch.first_word;
                slot_words((v_head + v_announced) mod 2) <= prefetch.words;
                v_announced := v_announced + 1;
            end if;

            -- start filling the next announced slot
            if state = idle and v_filled < v_announced and not (prefetch.request and v_announced = v_filled + 1) then
                fill_slot <= (v_head + v_filled) mod 2;
                address   <= to_integer(slot_first((v_head + v_filled) mod 2)(23 downto 0)) * 2;
                remaining <= 2 * slot_words((v_head + v_filled) mod 2);
                position  <= 0;
                state     <= issue;
            end if;

            head      <= v_head;
            announced <= v_announced;
            filled    <= v_filled;

            -- read port, the oldest announced datagram
            read_index <= head*buffer_words + to_integer((to_unsigned(read_address, 31) - slot_first(head)) mod buffer_words);
            read_data  <= memory(read_index);
        end if;
    end process;

end architecture rtl;
