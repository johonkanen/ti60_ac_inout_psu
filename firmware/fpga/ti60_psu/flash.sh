#!/bin/bash
# Write tsemac_soc's bitstream, the second-stage loader and a Zephyr image to the
# ti60_ac_psu board's SPI flash, so the board boots Zephyr on power-up without JTAG.
# The design's boot0 (in its on-chip RAM) copies the loader from flash 0x600000 to
# 0x01000000 and starts it; the loader (loader/) boots the newest valid image slot. The
# image goes into slot 0 (0x200000) with sequence 1, slot 1's header is cleared. Later
# images are best sent over the network with update.sh.
# The flash is not on an FTDI channel: Efinix's flash loader is put into the FPGA over
# JTAG and writes the flash, then the design is loaded over JTAG again to run it.
# With the remote update scheme built (tsemac_soc's images/flash.hex: golden and application
# images and the record of the latter) that goes to 0..2 MB, and the golden image is the one
# loaded afterwards: it switches to the application image as at power-up. This is the only
# JTAG step: later FPGA images go over the network (the web page's Update). Only with the
# power supply off.
#   ./flash.sh build/scope_psu/zephyr/zephyr.bin [tsemac_soc dir]
set -e
APP=$(realpath "$1")
SOC=$(realpath "${2:-$(dirname "$0")/../../../fpga/tsemac_soc}")
if [ -f "$SOC/images/flash.hex" ] && [ -f "$SOC/images/golden.bit" ]; then
    FPGA_HEX=$SOC/images/flash.hex
    FPGA_BIT=$SOC/images/golden.bit
else
    FPGA_HEX=$SOC/outflow/tsemac_soc.hex
    FPGA_BIT=$SOC/outflow/tsemac_soc.bit
fi
echo "FPGA: $FPGA_HEX, then $FPGA_BIT"
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
    -ifile "$FPGA_HEX" -iaddr 0x0 -ifile flash/slot0.hex -iaddr 0x00200000 \
    -ifile flash/slot1_header.hex -iaddr 0x00400000 -ifile flash/loader.hex -iaddr 0x00600000 \
    -o flash/combined.hex

# the board's FT4232H MiniModule, channel B is JTAG
JTAG_URL=${JTAG_URL:-ftdi://0x0403:0x6011:FT80TJL7/2}
PROGRAM="$EFINITY_HOME/bin/python3 $EFXPGM_HOME/bin/efx_pgm/ftdi_program.py"
$PROGRAM "$EFINITY_HOME/pgm/fli/titanium/u10660A79.bit" -m jtag -u "$JTAG_URL"
$PROGRAM flash/combined.hex -m jtag_bridge -u "$JTAG_URL"
$PROGRAM "$FPGA_BIT" -m jtag -u "$JTAG_URL"
