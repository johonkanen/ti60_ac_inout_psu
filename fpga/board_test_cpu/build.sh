#!/bin/bash
# board_test with a Sapphire RISC-V SoC beside it, for Zephyr. board_test itself is not
# touched: board_test_cpu_top instantiates board_test_top unchanged, and the project,
# interface and timing files are generated from board_test's at every build.
#   ./build.sh soc                  -> generate the Sapphire IP (needs java)
#   ./build.sh [app.bin]            -> optionally load app.bin into the on-chip RAM, then
#                                      compile to bitstream
#   ./build.sh program [ftdi_url]   -> program the board over JTAG with the last build
set -e
cd "$(dirname "$0")"

EFINITY_SETUP=${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}
JAVA_HOME=${JAVA_HOME:-$HOME/.local/opt/temurin17-jre}
export PATH="$JAVA_HOME/bin:$PATH"
unset NO_PROXY   # Efinity's configparser chokes on both no_proxy and NO_PROXY being set
source "$EFINITY_SETUP"
JTAG_URL=${2:-ftdi://0x0403:0x6011:FT80TJL7/2}

case "$1" in
soc)
    python3 gen_project.py   # the IP Manager wants the project file
    python3 gen_soc.py
    ;;
program)
    export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}
    "$EFINITY_HOME/bin/python3" "$EFXPGM_HOME/bin/efx_pgm/ftdi_program.py" outflow/board_test_cpu.bit -m jtag -u "$JTAG_URL"
    ;;
*)
    [ -f ip/sapphire/sapphire.v ] || { echo "no Sapphire IP yet: ./build.sh soc" >&2; exit 1; }
    if [ -n "$1" ]; then
        python3 ram_init.py "$1"
    fi
    python3 gen_project.py
    cat ../board_test/board_test.sdc cpu.sdc > board_test_cpu.sdc
    python3 gen_peri.py
    efx_run --prj board_test_cpu.xml
    ;;
esac
