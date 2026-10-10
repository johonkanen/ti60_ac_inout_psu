#!/bin/bash
# Sapphire SoC for Zephyr on the Ti60F225 development kit.
#   ./build.sh soc                   -> generate the Sapphire IP (needs java on PATH)
#   ./build.sh [app.bin]             -> load app.bin into on-chip RAM and compile to bitstream
#   ./build.sh program [ftdi_url]    -> program the kit over JTAG with the last build
set -e
cd "$(dirname "$0")"

EFINITY_SETUP=${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}
JAVA_HOME=${JAVA_HOME:-$HOME/.local/opt/temurin17-jre}
export PATH="$JAVA_HOME/bin:$PATH"
unset NO_PROXY   # Efinity's configparser chokes on both no_proxy and NO_PROXY being set
source "$EFINITY_SETUP"

case "$1" in
soc)
    python3 gen_soc.py
    ;;
program)
    # the kit's FT4232H has no serial number, select it by description. Channel B is JTAG.
    export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}
    JTAG_URL=${2:-$(python3 find_ftdi.py "Titanium Ti60F225 Development Kit" 2)}
    "$EFINITY_HOME/bin/python3" "$EFXPGM_HOME/bin/efx_pgm/ftdi_program.py" outflow/ti60_evm.bit -m jtag -u "$JTAG_URL"
    ;;
*)
    if [ -n "$1" ]; then
        python3 ram_init.py "$1"
    fi
    python3 gen_peri.py
    efx_run --prj ti60_evm.xml
    ;;
esac
