#!/usr/bin/env python3
# Prints the URL of an FTDI device found by its USB description, in the form
# ftdi_program.py --list_usb shows for devices without a serial number.
#   find_ftdi.py "Titanium Ti60F225 Development Kit" 2  -> ftdi://0x0403:0x6011:1:24/2
import sys
from pyftdi.ftdi import Ftdi

description, interface = sys.argv[1], sys.argv[2]
for device, _ in Ftdi.list_devices():
    if device.description == description:
        selector = device.sn or f"{device.bus:x}:{device.address:x}"
        print(f"ftdi://0x{device.vid:04x}:0x{device.pid:04x}:{selector}/{interface}")
        break
else:
    sys.exit(f"no FTDI device '{description}' found")
