library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;

-- Shares the transmit line between two frame sources. A source starts a frame only when
-- may_start is true and reports is_sending from its preamble to the end of its
-- interframe gap. The line goes to the other source after each frame when that one is
-- waiting, so both get every other frame under load.
entity gmii_frame_arbiter is
    port(
        clock         : in std_logic;
        gmii_in       : in gmii_record_array(0 to 1);
        wants_to_send : in boolean_vector(0 to 1);
        is_sending    : in boolean_vector(0 to 1);
        may_start     : out boolean_vector(0 to 1);
        gmii_out      : out gmii_record
    );
end entity gmii_frame_arbiter;

architecture rtl of gmii_frame_arbiter is
    signal grant       : natural range 0 to 1 := 0;
    signal was_sending : boolean := false;
begin

    gmii_out <= gmii_in(1) when is_sending(1) else gmii_in(0);

    may_start(0) <= grant = 0 and not is_sending(1);
    may_start(1) <= grant = 1 and not is_sending(0);

    process(clock)
        variable other : natural range 0 to 1;
    begin
        if rising_edge(clock) then
            other       := 1 - grant;
            was_sending <= is_sending(grant);
            if not is_sending(0) and not is_sending(1) and wants_to_send(other) then
                -- after a frame of the granted source, or when it has nothing to send
                if (was_sending and not is_sending(grant)) or not wants_to_send(grant) then
                    grant <= other;
                end if;
            end if;
        end if;
    end process;

end architecture rtl;
