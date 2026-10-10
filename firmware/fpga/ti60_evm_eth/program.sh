#!/bin/bash
# Program the kit over JTAG (volatile) with Efinix's TSEMAC Linux example design.
#   ./program.sh [bitstream]
set -e
cd "$(dirname "$0")"
BIT=$(realpath "${1:-$HOME/dev/ti60f225_evmlinux/fpga/ti60-tsemac-linux/outflow/ti60-tsemac-linux.bit}")
unset NO_PROXY
source ${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}
export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}
JTAG_URL=$(python3 ../ti60_evm/find_ftdi.py "Titanium Ti60F225 Development Kit" 2)
"$EFINITY_HOME/bin/python3" "$EFXPGM_HOME/bin/efx_pgm/ftdi_program.py" "$BIT" -m jtag -u "$JTAG_URL"
