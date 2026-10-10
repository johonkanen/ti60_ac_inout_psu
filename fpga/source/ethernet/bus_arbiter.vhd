library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.fpga_interconnect_pkg.all;

-- Shares the fpga_interconnect bus between g_masters bus masters (UART, UDP bridge, CPU).
-- Only one read is outstanding at a time, so each read response (a write to address 0
-- from the slaves) goes back to the master that asked for it. Waiting masters are served
-- round robin, so a busy master cannot starve the others.
--
-- Each master has a one deep request buffer. A master may either pulse its request for one
-- clock (the UART master) or hold it until it sees granted(i) (the UDP bridge, the CPU
-- bridge); granted(i) pulses for one clock after the request is forwarded. The request
-- input is ignored in the clock after a grant, so a holding master must not present its
-- next request in the clock it sees granted. A master has at most one request in flight.
-- A pulsing master gets no grant, so its next request must come after the buffered one
-- is forwarded, or it is dropped: the bus may be held by another master's read for up to
-- g_timeout clocks, while the UART master sends at most one command per ~175 clocks.
-- A read that no slave answers is given up after g_timeout clocks.
entity bus_arbiter is
    generic(
        g_masters : positive := 2;
        g_timeout : positive := 64
    );
    port(
        clock       : in std_logic;
        requests    : in bus_array(0 to g_masters-1);
        responses   : out bus_array(0 to g_masters-1);
        granted     : out boolean_vector(0 to g_masters-1);
        to_slaves   : out fpga_interconnect_record;
        from_slaves : in fpga_interconnect_record
    );
end entity bus_arbiter;

architecture rtl of bus_arbiter is

    subtype master_index is natural range 0 to g_masters-1;

    signal pending             : bus_array(0 to g_masters-1) := (others => init_fpga_interconnect);
    signal ignore_request      : boolean_vector(0 to g_masters-1) := (others => false);
    signal read_is_outstanding : boolean := false;
    signal owner               : master_index := 0;
    signal last_granted        : master_index := g_masters-1;
    signal timer               : natural range 0 to g_timeout := 0;

    function is_request(b : fpga_interconnect_record) return boolean is
    begin
        return b.data_read_is_requested_with_0 = '0' or b.data_write_is_requested_with_0 = '0';
    end function;

begin

    process(clock)
        variable waiting   : bus_array(0 to g_masters-1);
        variable is_free   : boolean;
        variable found     : boolean;
        variable chosen    : master_index;
        variable candidate : natural range 0 to 2*g_masters;
    begin
        if rising_edge(clock) then
            to_slaves      <= init_fpga_interconnect;
            responses      <= (others => init_fpga_interconnect);
            granted        <= (others => false);
            ignore_request <= (others => false);

            for i in waiting'range loop
                waiting(i) := pending(i);
                if not is_request(pending(i)) and is_request(requests(i)) and not ignore_request(i) then
                    waiting(i) := requests(i);
                end if;
            end loop;

            -- route a response to the owner of the outstanding read
            is_free := not read_is_outstanding;
            if read_is_outstanding then
                if write_to_address_is_requested(from_slaves, 0) then
                    responses(owner)    <= from_slaves;
                    read_is_outstanding <= false;
                elsif timer < g_timeout then
                    timer <= timer + 1;
                else
                    read_is_outstanding <= false;
                end if;
            end if;

            -- round robin, starting from the master after the one granted last
            found  := false;
            chosen := 0;
            for n in 1 to g_masters loop
                candidate := last_granted + n;
                if candidate >= g_masters then
                    candidate := candidate - g_masters;
                end if;
                if not found and is_request(waiting(candidate)) then
                    found  := true;
                    chosen := candidate;
                end if;
            end loop;

            if is_free and found then
                to_slaves              <= waiting(chosen);
                granted(chosen)        <= true;
                ignore_request(chosen) <= true;  -- a holding master sees the grant one clock later
                last_granted           <= chosen;
                if read_is_requested(waiting(chosen)) then
                    read_is_outstanding <= true;
                    owner               <= chosen;
                    timer               <= 0;
                end if;
                waiting(chosen) := init_fpga_interconnect;
            end if;

            pending <= waiting;
        end if;
    end process;

end architecture rtl;
