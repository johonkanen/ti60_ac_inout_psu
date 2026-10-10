#!/usr/bin/env python3
# Ethernet PHY management test for the board_test build.
#   python3 mdio_test.py <port>          scan PHY addresses and dump registers
#   python3 mdio_test.py <port> read  <phy> <reg>
#   python3 mdio_test.py <port> write <phy> <reg> <value>
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)),
    '..', 'source', 'fpga_communication', 'fpga_uart_pc_software'))
import serial
from uart_communication_functions import uart_link

if not hasattr(serial.Serial, 'set_buffer_size'):
    serial.Serial.set_buffer_size = lambda self, rx_size=None, tx_size=None: None

BASE = 300
COMMAND, PHY_ADDRESS, REGISTER_ADDRESS, WRITE_DATA, READ_DATA, TRANSACTIONS = range(6)


class Mdio:
    def __init__(self, port, baud=5_000_000):
        self.uart = uart_link(port, baud, number_of_databytes=4)

    def _run(self, command, phy, register):
        before = self.uart.request_data_from_address(BASE + TRANSACTIONS)
        self.uart.write_data_to_address(BASE + PHY_ADDRESS, phy)
        self.uart.write_data_to_address(BASE + REGISTER_ADDRESS, register)
        self.uart.write_data_to_address(BASE + COMMAND, command)
        end = time.monotonic() + 0.1
        while self.uart.request_data_from_address(BASE + TRANSACTIONS) == before:
            if time.monotonic() > end:
                raise TimeoutError('mdio transaction did not complete')

    def read(self, phy, register):
        self._run(1, phy, register)
        return self.uart.request_data_from_address(BASE + READ_DATA) & 0xffff

    def write(self, phy, register, value):
        self.uart.write_data_to_address(BASE + WRITE_DATA, value)
        self._run(2, phy, register)


def decode_status(bmsr):
    return (f"link {'up' if bmsr & 0x4 else 'down'}, "
            f"autoneg {'complete' if bmsr & 0x20 else 'not complete'}, "
            f"remote fault {'yes' if bmsr & 0x10 else 'no'}")


def decode_control(bmcr):
    speed = {0: '10', 1: '100', 2: '1000', 3: '?'}[((bmcr >> 6) & 1) << 1 | ((bmcr >> 13) & 1)]
    return (f"{'loopback, ' if bmcr & 0x4000 else ''}"
            f"autoneg {'on' if bmcr & 0x1000 else 'off'}, forced speed {speed}, "
            f"{'full' if bmcr & 0x100 else 'half'} duplex"
            f"{', power down' if bmcr & 0x800 else ''}{', isolate' if bmcr & 0x400 else ''}")


if __name__ == '__main__':
    mdio = Mdio(sys.argv[1])
    if len(sys.argv) > 2:
        phy, register = int(sys.argv[3], 0), int(sys.argv[4], 0)
        if sys.argv[2] == 'write':
            mdio.write(phy, register, int(sys.argv[5], 0))
        print(f"phy {phy} reg {register}: 0x{mdio.read(phy, register):04x}")
        sys.exit(0)

    found = []
    for phy in range(32):
        id1, id2 = mdio.read(phy, 2), mdio.read(phy, 3)
        if (id1, id2) not in ((0xffff, 0xffff), (0x0000, 0x0000)):
            found.append(phy)
            print(f"phy address {phy}: PHYID1 0x{id1:04x} PHYID2 0x{id2:04x}"
                  f" (OUI bits 0x{(id1 << 6) | (id2 >> 10):06x}, model 0x{(id2 >> 4) & 0x3f:02x}, rev {id2 & 0xf})")
    if not found:
        print("no PHY answered on any address")
        sys.exit(1)

    for phy in found:
        print(f"\nphy {phy} registers 0..31:")
        values = [mdio.read(phy, r) for r in range(32)]
        for r in range(0, 32, 8):
            print(f"  {r:2d}: " + " ".join(f"{v:04x}" for v in values[r:r+8]))
        print(f"  BMCR: {decode_control(values[0])}")
        print(f"  BMSR: {decode_status(values[1])}")
        # read the latched link bit twice so the second read shows the current state
        print(f"  BMSR now: {decode_status(mdio.read(phy, 1))}")
