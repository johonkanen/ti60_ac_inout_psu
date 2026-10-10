#!/bin/bash
# Write the TSEMAC design bitstream, the second-stage loader and a Zephyr image to the
# kit's SPI flash, so the board boots Zephyr on power-up without JTAG.
# The design's bootloader copies flash 0x600000 to 0x01000000 and jumps there: that is
# the loader (loader/), which boots the newest valid image slot. The image goes into
# slot 0 (0x200000) with sequence 1; slot 1's header is cleared so slot 0 is the one.
# Later images are best sent over the network with update.sh.
#   ./flash.sh build/scope/zephyr/zephyr.bin [bitstream.hex]
set -e
APP=$(realpath "$1")
HEX=$(realpath "${2:-$HOME/dev/ti60f225_evmlinux/fpga/ti60-tsemac-linux/outflow/ti60-tsemac-linux.hex}")
cd "$(dirname "$0")"

MAX=$((0x200000 - 0x1000))   # a slot less its header sector
if [ "$(stat -c %s "$APP")" -gt $MAX ]; then
    echo "$APP is $(stat -c %s "$APP") bytes, a slot holds $MAX" >&2
    exit 1
fi
../../loader/build.sh

unset NO_PROXY
source ${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}
export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}

rm -rf flash && mkdir flash
bin_to_hex() { python3 "$EFINITY_HOME/pgm/bin/efx_pgm/export_bitstream.py" bin_to_hex "$1" "$2"; }
python3 ../../loader/mkslot.py "$APP" flash/slot0.bin --sequence 1
head -c 4096 /dev/zero | tr '\0' '\377' > flash/slot1_header.bin
bin_to_hex flash/slot0.bin flash/slot0.hex
bin_to_hex flash/slot1_header.bin flash/slot1_header.hex
bin_to_hex ../../build/loader/loader.bin flash/loader.hex
python3 "$EFINITY_HOME/pgm/bin/multi_image_merger.py" -m generic_comb_image \
    -ifile "$HEX" -iaddr 0x0 -ifile flash/slot0.hex -iaddr 0x00200000 \
    -ifile flash/slot1_header.hex -iaddr 0x00400000 -ifile flash/loader.hex -iaddr 0x00600000 \
    -o flash/combined.hex

# the kit's FT4232H channel A drives the SPI flash
SPI_URL=$(python3 ../ti60_evm/find_ftdi.py "Titanium Ti60F225 Development Kit" 1)
"$EFINITY_HOME/bin/python3" "$EFXPGM_HOME/bin/efx_pgm/ftdi_program.py" flash/combined.hex -m active -u "$SPI_URL"
