#!/bin/bash
# Build (and optionally program) the UART bring-up design.
#   ./build.sh            -> generate peri.xml + full compile to bitstream
#   ./build.sh program [ftdi_url] -> program the board over JTAG with the last build
#   ./build.sh flash [ftdi_url]   -> write the last build to the SPI flash
set -e
cd "$(dirname "$0")"

EFINITY_SETUP=${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}
unset NO_PROXY   # Efinity's configparser chokes on both no_proxy and NO_PROXY being set
source "$EFINITY_SETUP"

if [ "$1" = "program" ]; then
    # FT4232H MiniModule on the control board, channel B is JTAG. Always pass an explicit
    # URL: efx_run --flow program silently picks the first cable when several are attached.
    export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}
    JTAG_URL=${2:-ftdi://0x0403:0x6011:FT80TJL7/2}
    "$EFINITY_HOME/bin/python3" "$EFXPGM_HOME/bin/efx_pgm/ftdi_program.py" outflow/board_test.bit -m jtag -u "$JTAG_URL"
    exit
fi

if [ "$1" = "flash" ]; then
    # The SPI flash is not on an FTDI channel: load Efinity's flash loader image over JTAG,
    # write the flash through it, then load the design over JTAG again.
    export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}
    JTAG_URL=${2:-ftdi://0x0403:0x6011:FT80TJL7/2}
    PROGRAM="$EFINITY_HOME/bin/python3 $EFXPGM_HOME/bin/efx_pgm/ftdi_program.py"
    $PROGRAM "$EFINITY_HOME/pgm/fli/titanium/u10660A79.bit" -m jtag -u "$JTAG_URL"
    $PROGRAM outflow/board_test.hex -m jtag_bridge -u "$JTAG_URL"
    $PROGRAM outflow/board_test.bit -m jtag -u "$JTAG_URL"
    exit
fi

python3 gen_peri.py
efx_run --prj board_test.xml
