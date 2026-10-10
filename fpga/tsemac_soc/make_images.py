#!/usr/bin/env python3
"""The flash images of the remote update scheme (remote_update/remote_update.vhd), from
images/golden.hex and images/application.hex (build.sh golden, build.sh):

  images/flash.hex                 both, combined by Efinix's merger as a remote update
                                   (internal reconfiguration) image: the golden image at 0,
                                   the application image at 1 MB, and the board's record of
                                   that image (untried) at 0x1ff000 (the firmware's
                                   fpga_update.c). For flash.sh, over JTAG, once.
  images/application_update.bin    the application image as it is in that flash image (its
                                   header holds the image addresses), behind a header of our
                                   own, for the board's /update (the web page's Update).

The update header, little endian: "TI60FPGA", header size (64), role (1, application),
flash address, payload size, payload CRC-32, then the device and project, NUL padded.
The board takes only this, never a golden image or a bare bitstream."""
import os
import struct
import subprocess
import sys
import zlib

APPLICATION_ADDRESS = 0x100000
STATE_ADDRESS = 0x1ff000            # a sector after it for the board's record of the image
APPLICATION_MAX = STATE_ADDRESS - APPLICATION_ADDRESS
HEADER_SIZE = 64

here = os.path.dirname(os.path.abspath(__file__))
images = os.path.join(here, 'images')


def hex_bytes(path):
    with open(path) as f:
        return bytes(int(x, 16) for x in f.read().split())


def main():
    golden = os.path.join(images, 'golden.hex')
    application = os.path.join(images, 'application.hex')
    for path in (golden, application):
        if not os.path.isfile(path):
            sys.exit(f'{path} missing: ./build.sh golden and ./build.sh')
    golden_size = len(hex_bytes(golden))
    if golden_size > APPLICATION_ADDRESS:
        sys.exit(f'the golden image is {golden_size} bytes, more than fits before 0x{APPLICATION_ADDRESS:x}')

    combined = os.path.join(images, 'flash.hex')
    merger = os.path.join(os.environ['EFINITY_HOME'], 'pgm', 'bin', 'multi_image_merger.py')
    subprocess.run([sys.executable, merger, '-m', 'selectable_flash_image', '-t', 'internal',
                    '-ifile', golden, '-iaddr', '0x00000000',
                    '-ifile', application, '-iaddr', f'0x{APPLICATION_ADDRESS:08x}',
                    '-o', combined], check=True, stdout=subprocess.DEVNULL)
    flash = hex_bytes(combined)
    payload = flash[APPLICATION_ADDRESS:].rstrip(b'\xff')
    if len(payload) > APPLICATION_MAX:
        sys.exit(f'the application image is {len(payload)} bytes, more than {APPLICATION_MAX}')
    if not payload.startswith(b'Version: '):
        sys.exit('no bitstream at the application address of the combined image')

    header = struct.pack('<8sIIIII', b'TI60FPGA', HEADER_SIZE, 1, APPLICATION_ADDRESS,
                         len(payload), zlib.crc32(payload))
    header += b'Ti60F225 tsemac_soc'.ljust(HEADER_SIZE - len(header), b'\0')
    # the state record of fpga_update.c: magic, version, size, CRC-32, sequence, header CRC,
    # two reserved words; the tried, confirmed and failed words stay all ones
    record = struct.pack('<IIIII', 0x53475046, 1, len(payload), zlib.crc32(payload), 1)
    record += struct.pack('<I', zlib.crc32(record))
    flash = flash[:APPLICATION_ADDRESS] + payload
    flash = flash.ljust(STATE_ADDRESS, b'\xff') + record.ljust(0x1000, b'\xff')
    with open(combined, 'w', newline='\n') as f:
        f.write(''.join(f'{b:02X}\n' for b in flash))

    update = os.path.join(images, 'application_update.bin')
    with open(update, 'wb') as f:
        f.write(header + payload)
    print(f'{combined}: golden {golden_size} bytes at 0, application at 0x{APPLICATION_ADDRESS:x}')
    print(f'{update}: {len(payload)} bytes, CRC-32 {zlib.crc32(payload):08x}')


if __name__ == '__main__':
    main()
