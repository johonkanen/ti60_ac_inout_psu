#!/bin/bash
# The ti60_ac_psu board's design around Efinix's IPs (configured from the Ti60F225 kit's TSEMAC example),
# for Zephyr: Sapphire with the HyperRAM as main memory, TSEMAC and its scatter-gather
# DMA on the board's PHY, console on the board's UART. Separate from board_test.
#   ./build.sh ips                  -> generate the IPs (needs java)
#   ./build.sh all                  -> both images at once, then the flash images: what a release needs
#   ./build.sh                      -> compile the application image -> images/application.hex
#   ./build.sh golden               -> compile the golden image      -> images/golden.hex
#   ./build.sh images               -> images/flash.hex (both, for flash.sh) and
#                                      images/application_update.bin (for the web page)
#   ./build.sh program [role] [ftdi_url] -> load images/<role>.bit (application) over JTAG
# The golden image at flash 0 is what the FPGA loads at power-up and is written only over
# JTAG; the application image at 1 MB is replaced over the network (remote_update/).
#
# A role builds from a project of its own made from tsemac_soc.xml (tsemac_soc_<role>.xml,
# with its role package and work directories) into outflow_<role>/, so the two can run at
# the same time; the interface (gen_peri.py) is made once, before.
set -e
cd "$(dirname "$0")"

EFINITY_SETUP=${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}
JAVA_HOME=${JAVA_HOME:-$HOME/.local/opt/temurin17-jre}
export PATH="$JAVA_HOME/bin:$PATH"
unset NO_PROXY   # Efinity's configparser chokes on both no_proxy and NO_PROXY being set
source "$EFINITY_SETUP"

need_ips() {
    [ -f ip/SapphireCore/SapphireCore.v ] || { echo "no IPs yet: ./build.sh ips" >&2; exit 1; }
}

# the project of a role: its role package, project name, work directories, and the interface
# under the project's name (efx_pgm takes the remote update settings from <project>.peri.xml:
# without it the images lack the remote update bits and the FPGA rejects them)
make_project() {
    local role=$1 number=$2
    sed -e "s/name=\"tsemac_soc\"/name=\"tsemac_soc_$role\"/" tsemac_soc.peri.xml > "tsemac_soc_$role.peri.xml"
    printf '%s\n' \
        "-- written by build.sh: which image this build is, see remote_update/remote_update.vhd" \
        "package image_role_pkg is" \
        "    constant image_role : natural := $number;  -- $role" \
        "end package image_role_pkg;" > "image_role_pkg_$role.vhd"
    sed -e "s/name=\"tsemac_soc\"/name=\"tsemac_soc_$role\"/" \
        -e "s/\"image_role_pkg.vhd\"/\"image_role_pkg_$role.vhd\"/" \
        -e "s/\"tsemac_soc.peri.xml\"/\"tsemac_soc_$role.peri.xml\"/g" \
        -e "s/value=\"work_\(syn\|pnr\|dbg\)\"/value=\"work_\1_$role\"/" \
        tsemac_soc.xml > "tsemac_soc_$role.xml"
}

build_role() {
    local role=$1
    efx_run --prj "tsemac_soc_$role.xml" --output_dir "outflow_$role"
    mkdir -p images
    cp "outflow_$role/tsemac_soc_$role.hex" "images/$role.hex"
    cp "outflow_$role/tsemac_soc_$role.bit" "images/$role.bit"
    echo "images/$role.hex, images/$role.bit"
}

case "$1" in
ips)
    shift
    python3 gen_ips.py "$@"
    ;;
program)
    role=${2:-application}
    export EFINITY_USER_DIR=${EFINITY_USER_DIR:-$HOME/.efinity}
    "$EFINITY_HOME/bin/python3" "$EFXPGM_HOME/bin/efx_pgm/ftdi_program.py" "images/$role.bit" -m jtag \
        -u "${3:-ftdi://0x0403:0x6011:FT80TJL7/2}"
    ;;
images)
    python3 make_images.py
    ;;
""|golden)
    need_ips
    if [ "$1" = golden ]; then role=golden; number=0; else role=application; number=1; fi
    python3 gen_peri.py
    make_project $role $number
    build_role $role
    ;;
all)
    need_ips
    python3 gen_peri.py
    make_project golden 0
    make_project application 1
    start=$(date +%s)
    build_role golden > build_golden.log 2>&1 & golden=$!
    build_role application > build_application.log 2>&1 & application=$!
    status=0
    wait $golden || { echo "the golden image failed: build_golden.log" >&2; status=1; }
    wait $application || { echo "the application image failed: build_application.log" >&2; status=1; }
    [ $status = 0 ] || exit 1
    echo "both images in $(( $(date +%s) - start )) s"
    python3 make_images.py
    ;;
*)
    echo "unknown command: $1" >&2
    exit 1
    ;;
esac
