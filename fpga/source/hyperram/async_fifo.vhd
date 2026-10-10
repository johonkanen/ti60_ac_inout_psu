library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Dual clock FIFO with gray coded pointers crossing between the clock domains.
-- read_data is valid the read clock after read is asserted. Writes to a full FIFO and
-- reads from an empty one are ignored. The flags are conservative: is_full and is_empty
-- may stay set a few clocks after the other side has changed the fill.
entity async_fifo is
    generic(
        g_width        : positive;
        g_address_bits : positive := 4   -- depth 2**g_address_bits
    );
    port(
        write_clock : in std_logic;
        write       : in boolean;
        write_data  : in std_logic_vector(g_width-1 downto 0);
        is_full     : out boolean;

        read_clock  : in std_logic;
        read        : in boolean;
        read_data   : out std_logic_vector(g_width-1 downto 0);
        is_empty    : out boolean;
        read_count  : out natural range 0 to 2**g_address_bits   -- words to read, may lag behind
    );
end entity async_fifo;

architecture rtl of async_fifo is

    subtype pointer is unsigned(g_address_bits downto 0);   -- one extra bit tells full from empty
    type memory_array is array (0 to 2**g_address_bits-1) of std_logic_vector(g_width-1 downto 0);
    signal memory : memory_array := (others => (others => '0'));

    signal write_pointer, read_pointer : pointer := (others => '0');
    signal write_gray, read_gray       : pointer := (others => '0');
    signal read_gray_sync1, read_gray_sync2   : pointer := (others => '0');
    signal write_gray_sync1, write_gray_sync2 : pointer := (others => '0');

    attribute async_reg : boolean;
    attribute async_reg of read_gray_sync1, read_gray_sync2, write_gray_sync1, write_gray_sync2 : signal is true;

    function to_gray(p : pointer) return pointer is
    begin
        return p xor ('0' & p(p'left downto 1));
    end function;

    function from_gray(g : pointer) return pointer is
        variable p : pointer;
    begin
        p(p'left) := g(g'left);
        for i in p'left-1 downto 0 loop
            p(i) := p(i+1) xor g(i);
        end loop;
        return p;
    end function;

    signal full, empty : boolean;

begin

    -- full when the write pointer is one lap ahead of the read pointer
    full  <= write_gray = (not read_gray_sync2(g_address_bits downto g_address_bits-1)
                          & read_gray_sync2(g_address_bits-2 downto 0));
    empty <= read_gray = write_gray_sync2;
    is_full  <= full;
    is_empty <= empty;
    read_count <= to_integer(from_gray(write_gray_sync2) - read_pointer);

    write_side : process(write_clock)
        variable next_pointer : pointer;
    begin
        if rising_edge(write_clock) then
            read_gray_sync1 <= read_gray;
            read_gray_sync2 <= read_gray_sync1;
            if write and not full then
                memory(to_integer(write_pointer(g_address_bits-1 downto 0))) <= write_data;
                next_pointer  := write_pointer + 1;
                write_pointer <= next_pointer;
                write_gray    <= to_gray(next_pointer);
            end if;
        end if;
    end process;

    read_side : process(read_clock)
        variable next_pointer : pointer;
    begin
        if rising_edge(read_clock) then
            write_gray_sync1 <= write_gray;
            write_gray_sync2 <= write_gray_sync1;
            if read and not empty then
                read_data    <= memory(to_integer(read_pointer(g_address_bits-1 downto 0)));
                next_pointer := read_pointer + 1;
                read_pointer <= next_pointer;
                read_gray    <= to_gray(next_pointer);
            end if;
        end if;
    end process;

end architecture rtl;
