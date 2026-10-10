library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;

-- Header layout of ARP, IPv4/ICMP and IPv4/UDP frames and construction of the replies.
-- Byte offsets count from the destination MAC address, the first byte after the SFD.
-- All three replies differ from the request only in the first 42 bytes, so a reply is
-- the 42 byte header from reply_byte followed by the rest of the request unchanged.
package ethernet_protocol_pkg is

    constant header_length : natural := 42;
    type header_array is array (0 to header_length-1) of byte;

    subtype mac_address is std_logic_vector(47 downto 0);
    subtype ip_address  is std_logic_vector(31 downto 0);

    constant ethertype_arp  : std_logic_vector(15 downto 0) := x"0806";
    constant ethertype_ipv4 : std_logic_vector(15 downto 0) := x"0800";
    constant ip_protocol_icmp : byte := x"01";
    constant ip_protocol_udp  : byte := x"11";

    -- ethernet
    constant eth_destination : natural := 0;
    constant eth_source      : natural := 6;
    constant eth_type        : natural := 12;
    -- arp
    constant arp_hardware_type   : natural := 14;
    constant arp_protocol_type   : natural := 16;
    constant arp_hardware_length : natural := 18;
    constant arp_protocol_length : natural := 19;
    constant arp_operation       : natural := 20;
    constant arp_sender_mac      : natural := 22;
    constant arp_sender_ip       : natural := 28;
    constant arp_target_mac      : natural := 32;
    constant arp_target_ip       : natural := 38;
    -- ipv4
    constant ip_version_ihl   : natural := 14;
    constant ip_total_length  : natural := 16;
    constant ip_checksum      : natural := 24;
    constant ip_flags_fragment : natural := 20;
    constant ip_protocol      : natural := 23;
    constant ip_source        : natural := 26;
    constant ip_destination   : natural := 30;
    -- icmp
    constant icmp_type     : natural := 34;
    constant icmp_code     : natural := 35;
    constant icmp_checksum : natural := 36;
    -- udp
    constant udp_source_port      : natural := 34;
    constant udp_destination_port : natural := 36;
    constant udp_length           : natural := 38;
    constant udp_checksum         : natural := 40;
    constant ip_udp_header_length : natural := 28;

    constant arp_reply_length : natural := 60;  -- minimum frame without FCS

    type reply_kind is (no_reply, arp_reply, icmp_echo_reply, udp_echo_reply, udp_bridge_reply);

    -- header fields of a reply whose length differs from the request
    type reply_length_fields is record
        ip_total_length : std_logic_vector(15 downto 0);
        ip_checksum     : std_logic_vector(15 downto 0);
        udp_length      : std_logic_vector(15 downto 0);
    end record;

    constant no_length_change : reply_length_fields := (others => (others => '0'));

    function get_bytes (h : header_array; offset : natural; length : natural) return std_logic_vector;

    -- frame classification and validity, used by the dispatcher and the protocol modules
    function is_arp_request_for (h : header_array; own_ip : ip_address) return boolean;
    -- ip_header_sum is the one's complement sum of the 20 byte ip header
    function is_ipv4_for (h : header_array; own_ip : ip_address; ip_header_sum : std_logic_vector(15 downto 0)) return boolean;
    function is_icmp_echo_request (h : header_array) return boolean;
    function is_udp_to_port (h : header_array; port_number : natural) return boolean;

    -- one's complement sum of two 16 bit values with end around carry
    function ones_complement_add (a, b : std_logic_vector(15 downto 0)) return std_logic_vector;

    -- checksum after changing an echo request (type 8) to an echo reply (type 0), RFC 1624
    function icmp_reply_checksum (request_checksum : std_logic_vector(15 downto 0)) return std_logic_vector;

    -- checksum after a 16 bit field covered by it changes from old_value to new_value, RFC 1624
    function updated_checksum (
        checksum, old_value, new_value : std_logic_vector(15 downto 0))
        return std_logic_vector;

    function reply_byte (
        kind          : reply_kind;
        h             : header_array;
        index         : natural;
        own_mac       : mac_address;
        own_ip        : ip_address;
        icmp_checksum_value : std_logic_vector(15 downto 0);
        lengths       : reply_length_fields := no_length_change)
        return byte;

end package ethernet_protocol_pkg;

package body ethernet_protocol_pkg is

    function get_bytes (h : header_array; offset : natural; length : natural) return std_logic_vector is
        variable retval : std_logic_vector(8*length-1 downto 0);
    begin
        for i in 0 to length-1 loop
            retval(8*(length-i)-1 downto 8*(length-i-1)) := h(offset + i);
        end loop;
        return retval;
    end get_bytes;

    function is_arp_request_for (h : header_array; own_ip : ip_address) return boolean is
    begin
        return get_bytes(h, eth_type, 2) = ethertype_arp
            and get_bytes(h, arp_hardware_type, 2) = x"0001"
            and get_bytes(h, arp_protocol_type, 2) = ethertype_ipv4
            and get_bytes(h, arp_operation, 2) = x"0001"
            and get_bytes(h, arp_target_ip, 4) = own_ip;
    end is_arp_request_for;

    function is_ipv4_for (h : header_array; own_ip : ip_address; ip_header_sum : std_logic_vector(15 downto 0)) return boolean is
    begin
        return get_bytes(h, eth_type, 2) = ethertype_ipv4
            and h(ip_version_ihl) = x"45"
            and ip_header_sum = x"FFFF"
            and get_bytes(h, ip_destination, 4) = own_ip
            and (h(ip_flags_fragment) and x"3F") = x"00" and h(ip_flags_fragment + 1) = x"00";
    end is_ipv4_for;

    function is_icmp_echo_request (h : header_array) return boolean is
    begin
        return h(ip_protocol) = ip_protocol_icmp and h(icmp_type) = x"08" and h(icmp_code) = x"00";
    end is_icmp_echo_request;

    function is_udp_to_port (h : header_array; port_number : natural) return boolean is
    begin
        return h(ip_protocol) = ip_protocol_udp
            and to_integer(unsigned(get_bytes(h, udp_destination_port, 2))) = port_number;
    end is_udp_to_port;

    function ones_complement_add (a, b : std_logic_vector(15 downto 0)) return std_logic_vector is
        variable sum : unsigned(16 downto 0);
        variable folded : unsigned(15 downto 0);
    begin
        sum    := ('0' & unsigned(a)) + ('0' & unsigned(b));
        folded := sum(15 downto 0);
        if sum(16) = '1' then
            folded := folded + 1;
        end if;
        return std_logic_vector(folded);
    end ones_complement_add;

    function icmp_reply_checksum (request_checksum : std_logic_vector(15 downto 0)) return std_logic_vector is
    begin
        -- HC' = ~(~HC + ~m + m'), m = 0x0800 (type 8, code 0), m' = 0x0000
        return not ones_complement_add(not request_checksum, not x"0800");
    end icmp_reply_checksum;

    function updated_checksum
    (
        checksum, old_value, new_value : std_logic_vector(15 downto 0)
    )
    return std_logic_vector
    is
    begin
        -- HC' = ~(~HC + ~m + m')
        return not ones_complement_add(ones_complement_add(not checksum, not old_value), new_value);
    end updated_checksum;

    function byte_of (value : std_logic_vector; index : natural) return byte is
        constant bytes : natural := value'length/8;
        variable v : std_logic_vector(value'length-1 downto 0) := value;
    begin
        return v(8*(bytes-index)-1 downto 8*(bytes-index-1));
    end byte_of;

    function reply_byte
    (
        kind          : reply_kind;
        h             : header_array;
        index         : natural;
        own_mac       : mac_address;
        own_ip        : ip_address;
        icmp_checksum_value : std_logic_vector(15 downto 0);
        lengths       : reply_length_fields := no_length_change
    )
    return byte
    is
    begin
        -- ethernet header, back to the sender from our address
        if index < eth_source then
            return h(eth_source + index);
        elsif index < eth_type then
            return byte_of(own_mac, index - eth_source);
        end if;

        CASE kind is
            WHEN arp_reply =>
                CASE index is
                    WHEN arp_operation     => return x"00";
                    WHEN arp_operation + 1 => return x"02";
                    WHEN arp_sender_mac to arp_sender_mac + 5 => return byte_of(own_mac, index - arp_sender_mac);
                    WHEN arp_sender_ip  to arp_sender_ip + 3  => return byte_of(own_ip, index - arp_sender_ip);
                    WHEN arp_target_mac to arp_target_mac + 5 => return h(arp_sender_mac + index - arp_target_mac);
                    WHEN arp_target_ip  to arp_target_ip + 3  => return h(arp_sender_ip + index - arp_target_ip);
                    WHEN others => return h(index);
                end CASE;

            WHEN icmp_echo_reply | udp_echo_reply | udp_bridge_reply =>
                -- swapping the addresses and ports keeps the ip and udp checksums valid
                CASE index is
                    WHEN ip_source to ip_source + 3           => return h(ip_destination + index - ip_source);
                    WHEN ip_destination to ip_destination + 3 => return h(ip_source + index - ip_destination);
                    WHEN others => null;
                end CASE;
                if kind = icmp_echo_reply then
                    CASE index is
                        WHEN icmp_type          => return x"00";
                        WHEN icmp_checksum      => return icmp_checksum_value(15 downto 8);
                        WHEN icmp_checksum + 1  => return icmp_checksum_value(7 downto 0);
                        WHEN others => return h(index);
                    end CASE;
                else
                    if kind = udp_bridge_reply then
                        CASE index is
                            WHEN ip_total_length     => return lengths.ip_total_length(15 downto 8);
                            WHEN ip_total_length + 1 => return lengths.ip_total_length(7 downto 0);
                            WHEN ip_checksum         => return lengths.ip_checksum(15 downto 8);
                            WHEN ip_checksum + 1     => return lengths.ip_checksum(7 downto 0);
                            WHEN udp_length          => return lengths.udp_length(15 downto 8);
                            WHEN udp_length + 1      => return lengths.udp_length(7 downto 0);
                            WHEN udp_checksum to udp_checksum + 1 => return x"00";  -- not computed, allowed for IPv4
                            WHEN others => null;
                        end CASE;
                    end if;
                    CASE index is
                        WHEN udp_source_port to udp_source_port + 1 =>
                            return h(udp_destination_port + index - udp_source_port);
                        WHEN udp_destination_port to udp_destination_port + 1 =>
                            return h(udp_source_port + index - udp_destination_port);
                        WHEN others => return h(index);
                    end CASE;
                end if;

            WHEN no_reply => return h(index);
        end CASE;
    end reply_byte;

end package body ethernet_protocol_pkg;
