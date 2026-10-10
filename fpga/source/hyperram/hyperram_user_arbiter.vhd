library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.hyperram_pkg.all;

-- Shares one HyperRAM user port between two users, a transaction at a time.
-- A user sees is_ready only when the controller is offered to it. A command taken from a
-- user makes it the owner until transaction_is_done, the other user sees nothing of the
-- transaction and is offered the controller next. While the controller is ready and the
-- offered user does not take it, the offer alternates every clock.
-- The latency count of user 0 is used for all transactions.
entity hyperram_user_arbiter is
    port(
        clock      : in std_logic;
        user0_in   : in hyperram_in_record;
        user0_out  : out hyperram_out_record;
        user1_in   : in hyperram_in_record;
        user1_out  : out hyperram_out_record;
        shared_in  : out hyperram_in_record;
        shared_out : in hyperram_out_record
    );
end entity hyperram_user_arbiter;

architecture rtl of hyperram_user_arbiter is

    constant nothing : hyperram_out_record := (
        is_ready             => false,
        write_data_requested => false,
        read_data            => (others => '0'),
        read_data_is_ready   => false,
        transaction_is_done  => false,
        read_timed_out       => false);

    signal offer       : natural range 0 to 1 := 0;
    signal offered     : natural range 0 to 1 := 0;    -- offer of the previous clock
    signal owner       : natural range 0 to 1 := 0;
    signal is_owned    : boolean := false;
    signal selected    : natural range 0 to 1;
    signal is_starting : boolean;

    function with_ready(o : hyperram_out_record; ready : boolean) return hyperram_out_record is
        variable r : hyperram_out_record;
    begin
        r := o;
        r.is_ready := ready;
        return r;
    end function;

begin

    -- a command issued on the previous offer is passed on right away
    is_starting <= not is_owned and ((offered = 0 and user0_in.command /= none)
                                  or (offered = 1 and user1_in.command /= none));
    selected <= owner when is_owned else offered;

    process(all)
    begin
        if selected = 0 then
            shared_in <= user0_in;
        else
            shared_in <= user1_in;
        end if;
        shared_in.latency_count <= user0_in.latency_count;
    end process;

    process(all)
        variable may_issue : boolean;
    begin
        may_issue := shared_out.is_ready and not is_owned and not is_starting;
        user0_out <= nothing;
        user1_out <= nothing;
        if is_owned and owner = 0 then
            user0_out <= with_ready(shared_out, false);
        elsif is_owned then
            user1_out <= with_ready(shared_out, false);
        end if;
        if may_issue and offer = 0 then
            user0_out.is_ready <= true;
        elsif may_issue then
            user1_out.is_ready <= true;
        end if;
        user0_out.read_data <= shared_out.read_data;
        user1_out.read_data <= shared_out.read_data;
    end process;

    process(clock)
    begin
        if rising_edge(clock) then
            offered <= offer;
            if is_starting then
                owner    <= offered;
                is_owned <= true;
                offer    <= 1 - offered;
            elsif is_owned then
                if shared_out.transaction_is_done then
                    is_owned <= false;
                end if;
            elsif shared_out.is_ready then
                offer <= 1 - offer;
            end if;
        end if;
    end process;

end architecture rtl;
