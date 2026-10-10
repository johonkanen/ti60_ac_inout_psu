library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

-- Drives uart_test_top over UART at 5 Mbaud: writes the scratch register, reads it back.
-- The host receiver runs as its own process, since replies can start before the
-- host has finished sending the stop bit of the request.
entity uart_test_tb is
end entity;

architecture sim of uart_test_tb is
    constant clock_period : time := 8.333 ns;     -- 120 MHz
    constant bit_period   : time := 24 * clock_period;

    signal clock   : std_logic := '0';
    signal uart_rx : std_logic := '1';
    signal uart_tx : std_logic;
    signal done    : boolean := false;

    type byte_array is array (natural range <>) of std_logic_vector(7 downto 0);
    signal host_rx_bytes : byte_array(0 to 63);
    signal host_rx_count : natural := 0;
begin
    clock <= not clock after clock_period/2 when not done;

    dut : entity work.uart_test_top
    port map(main_clock => clock, pll_locked => '1', uart_rx => uart_rx, uart_tx => uart_tx);

    host_receiver : process
        variable byte : std_logic_vector(7 downto 0);
    begin
        wait until uart_tx = '0';
        wait for bit_period * 1.5;
        for i in 0 to 7 loop
            byte(i) := uart_tx; wait for bit_period;
        end loop;
        host_rx_bytes(host_rx_count) <= byte;
        host_rx_count <= host_rx_count + 1;
    end process;

    stimulus : process
        procedure send(bytes : byte_array) is
        begin
            for b in bytes'range loop
                uart_rx <= '0'; wait for bit_period;
                for i in 0 to 7 loop
                    uart_rx <= bytes(b)(i); wait for bit_period;
                end loop;
                uart_rx <= '1'; wait for bit_period;
            end loop;
        end procedure;

        procedure read_register(address : natural; data : out std_logic_vector(31 downto 0)) is
            constant first : natural := host_rx_count;
        begin
            send((x"02", x"00", std_logic_vector(to_unsigned(address, 8))));
            wait until host_rx_count = first + 7 for 100 us;
            assert host_rx_count = first + 7 report "timeout reading address " & integer'image(address) severity failure;
            data := host_rx_bytes(first+3) & host_rx_bytes(first+4) & host_rx_bytes(first+5) & host_rx_bytes(first+6);
            report "address " & integer'image(address) & " = " & to_hstring(data);
        end procedure;

        variable data : std_logic_vector(31 downto 0);
    begin
        wait for 1 us;
        read_register(1, data);
        assert data = x"0000acdc" report "id mismatch" severity error;

        send((x"04", x"00", x"03", x"12", x"34", x"56", x"78"));
        wait for 2 us;
        read_register(3, data);
        assert data = x"12345678" report "scratch mismatch" severity error;

        report "testbench done";
        done <= true;
        wait;
    end process;
end architecture;
