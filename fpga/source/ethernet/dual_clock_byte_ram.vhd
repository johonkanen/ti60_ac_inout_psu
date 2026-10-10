library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Simple dual port byte RAM with separate write and read clocks, block RAM template.
-- Read data is registered: it appears one read clock after the address.
entity dual_clock_byte_ram is
    generic(g_depth : positive := 2048);
    port(
        write_clock   : in std_logic;
        write_enable  : in boolean;
        write_address : in natural range 0 to g_depth-1;
        write_data    : in std_logic_vector(7 downto 0);

        read_clock    : in std_logic;
        read_address  : in natural range 0 to g_depth-1;
        read_data     : out std_logic_vector(7 downto 0)
    );
end entity dual_clock_byte_ram;

architecture rtl of dual_clock_byte_ram is
    type memory_array is array (0 to g_depth-1) of std_logic_vector(7 downto 0);
    signal memory : memory_array := (others => (others => '0'));
begin
    process(write_clock)
    begin
        if rising_edge(write_clock) then
            if write_enable then
                memory(write_address) <= write_data;
            end if;
        end if;
    end process;

    process(read_clock)
    begin
        if rising_edge(read_clock) then
            read_data <= memory(read_address);
        end if;
    end process;
end architecture rtl;
