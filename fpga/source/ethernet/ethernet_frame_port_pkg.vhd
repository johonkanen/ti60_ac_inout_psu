library ieee;
    use ieee.std_logic_1164.all;
    use ieee.numeric_std.all;

    use work.ethernet_pkg.all;

-- Interface between the frame buffers and the protocol modules.
--
-- Received frames sit in fixed 2 KB slots of the receive frame RAM and replies are
-- written into 2 KB slots of the transmit frame RAM. A protocol module only sees offsets:
-- the dispatcher gives it a frame with frame_is_ready, offset and length, and the module
-- reads the frame at offset + n through read_address/read_data (read data two clocks
-- after the address). A reply is written into the free transmit slot at tx_offset + n and
-- queued with tx_send and tx_length. release hands the receive slot back, replied tells
-- whether a reply was queued for it.
package ethernet_frame_port_pkg is

    constant slot_size     : natural := 2048;
    constant rx_slot_count : natural := 3;
    constant tx_slot_count : natural := 2;
    constant frame_read_latency : natural := 2;

    subtype rx_frame_address is natural range 0 to rx_slot_count*slot_size-1;
    subtype tx_frame_address is natural range 0 to tx_slot_count*slot_size-1;
    subtype frame_length_type is natural range 0 to slot_size-1;

    type frame_port_in_record is record
        frame_is_ready  : boolean;
        offset          : rx_frame_address;
        length          : frame_length_type;
        read_data       : byte;
        tx_slot_is_free : boolean;
        tx_offset       : tx_frame_address;
    end record;

    type frame_port_out_record is record
        read_address     : rx_frame_address;
        release_frame          : boolean;
        replied          : boolean;
        tx_write_enable  : boolean;
        tx_write_address : tx_frame_address;
        tx_write_data    : byte;
        tx_send          : boolean;
        tx_length        : frame_length_type;
    end record;

    constant idle_frame_port_out : frame_port_out_record := (
        read_address     => 0,
        release_frame          => false,
        replied          => false,
        tx_write_enable  => false,
        tx_write_address => 0,
        tx_write_data    => (others => '0'),
        tx_send          => false,
        tx_length        => 0);

    -- routing of received frames to protocol modules, by ethertype and optionally by the
    -- ip protocol and the udp destination port
    type frame_route is record
        ethertype      : std_logic_vector(15 downto 0);
        check_protocol : boolean;
        protocol       : byte;
        check_port     : boolean;
        udp_port       : natural range 0 to 2**16-1;
    end record;
    type frame_route_array is array (natural range <>) of frame_route;

    type frame_port_in_array  is array (natural range <>) of frame_port_in_record;
    type frame_port_out_array is array (natural range <>) of frame_port_out_record;

    -- pulses of frame_port_out_record are cleared every clock, the addresses keep their value
    procedure init_frame_port (signal self : out frame_port_out_record);

    procedure write_tx_byte (
        signal self : out frame_port_out_record;
        address     : tx_frame_address;
        data        : byte);

end package ethernet_frame_port_pkg;

package body ethernet_frame_port_pkg is

    procedure init_frame_port (signal self : out frame_port_out_record) is
    begin
        self.release_frame         <= false;
        self.replied         <= false;
        self.tx_write_enable <= false;
        self.tx_send         <= false;
    end init_frame_port;

    procedure write_tx_byte (
        signal self : out frame_port_out_record;
        address     : tx_frame_address;
        data        : byte) is
    begin
        self.tx_write_enable  <= true;
        self.tx_write_address <= address;
        self.tx_write_data    <= data;
    end write_tx_byte;

end package body ethernet_frame_port_pkg;
