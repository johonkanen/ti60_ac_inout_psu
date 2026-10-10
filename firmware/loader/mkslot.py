#!/usr/bin/env python3
"""Wrap a zephyr.bin into a slot for the second-stage loader (sapphire_loader.h):
the header, padding to SLOT_IMAGE_OFFSET, then the image.
    mkslot.py zephyr.bin slot.bin [--sequence N] [--load-address A] [--entry E]
"""
import argparse
import struct
import zlib

SLOT_MAGIC = 0x544f4c53
SLOT_IMAGE_OFFSET = 0x1000
HEADER = struct.Struct('<8I')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('image')
    parser.add_argument('slot')
    parser.add_argument('--sequence', type=int, default=1)
    parser.add_argument('--load-address', type=lambda s: int(s, 0), default=0x01000000)
    parser.add_argument('--entry', type=lambda s: int(s, 0))
    args = parser.parse_args()

    image = open(args.image, 'rb').read()
    entry = args.load_address if args.entry is None else args.entry
    fields = [SLOT_MAGIC, HEADER.size, args.sequence, len(image), zlib.crc32(image),
              args.load_address, entry]
    header = struct.pack('<7I', *fields)
    header += struct.pack('<I', zlib.crc32(header))
    with open(args.slot, 'wb') as f:
        f.write(header.ljust(SLOT_IMAGE_OFFSET, b'\xff') + image)
    print(f'{args.slot}: sequence {args.sequence}, {len(image)} bytes, '
          f'crc32 {zlib.crc32(image):08x}')


if __name__ == '__main__':
    main()
