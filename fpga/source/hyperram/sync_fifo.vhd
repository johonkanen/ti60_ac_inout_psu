library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Single clock FIFO in block RAM. read_data is valid the clock after read is asserted.
-- Writes to a full FIFO and reads from an empty one are ignored. clear empties it.
entity sync_fifo is
    generic(
        g_width : positive := 32;
        g_depth : positive := 512    -- power of two
    );
    port(
        clock      : in std_logic;
        clear      : in boolean := false;
        write      : in boolean;
        write_data : in std_logic_vector(g_width-1 downto 0);
        read       : in boolean;
        read_data  : out std_logic_vector(g_width-1 downto 0);
        count      : out natural range 0 to g_depth;
        is_empty   : out boolean;
        is_full    : out boolean
    );
end entity sync_fifo;

architecture rtl of sync_fifo is
    type memory_array is array (0 to g_depth-1) of std_logic_vector(g_width-1 downto 0);
    signal memory : memory_array := (others => (others => '0'));
    signal write_pointer, read_pointer : natural range 0 to g_depth-1 := 0;
    signal fill : natural range 0 to g_depth := 0;
begin

    count    <= fill;
    is_empty <= fill = 0;
    is_full  <= fill = g_depth;

    process(clock)
        variable do_write, do_read : boolean;
    begin
        if rising_edge(clock) then
            do_write := write and fill < g_depth;
            do_read  := read and fill > 0;

            if do_write then
                memory(write_pointer) <= write_data;
                write_pointer <= (write_pointer + 1) mod g_depth;
            end if;
            if do_read then
                read_data    <= memory(read_pointer);
                read_pointer <= (read_pointer + 1) mod g_depth;
            end if;

            if do_write and not do_read then
                fill <= fill + 1;
            elsif do_read and not do_write then
                fill <= fill - 1;
            end if;

            if clear then
                write_pointer <= 0;
                read_pointer  <= 0;
                fill          <= 0;
            end if;
        end if;
    end process;

end architecture rtl;
