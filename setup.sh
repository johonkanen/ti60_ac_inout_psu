#!/bin/bash
# One-time setup of a fresh clone, run from anywhere:
#   - the FPGA's VHDL submodules
#   - firmware/.venv: Python 3.12 or newer with west and Zephyr's build requirements
#   - the west workspace: Zephyr v4.4.2 and the modules it needs (west.yml), fetched into
#     the directory this clone is in, beside it
#   - a check of the tools that are not fetched: Zephyr SDK, Efinity, Java, nvc
# Clone into a directory of its own, as that directory becomes the west workspace:
#   mkdir psu && cd psu && git clone https://github.com/johonkanen/ti60_ac_inout_psu && ti60_ac_inout_psu/setup.sh
set -e
repo=$(cd "$(dirname "$0")" && pwd)
top=$(dirname "$repo")
# Efinity's setup.sh points every Python at its own: not here
unset PYTHONHOME PYTHONPATH PYTHONNOUSERSITE

echo "== VHDL submodules"
git -C "$repo" submodule update --init --recursive

echo "== Python environment, firmware/.venv"
venv=$repo/firmware/.venv
if [ ! -x "$venv/bin/python" ]; then
    py=
    for candidate in python3.14 python3.13 python3.12; do
        if command -v $candidate >/dev/null; then py=$candidate; break; fi
    done
    if [ -n "$py" ]; then
        "$py" -m venv "$venv"
    elif command -v uv >/dev/null; then
        uv venv --seed --python 3.12 "$venv"
    else
        echo "Python 3.12 or newer is needed (or uv to fetch one: https://docs.astral.sh/uv/)" >&2
        exit 1
    fi
fi
source "$venv/bin/activate"
pip install -q --upgrade pip west

echo "== West workspace in $top"
cd "$top"
[ -d .west ] || west init -l "$repo"
west update --narrow -o=--depth=1
pip install -q -r zephyr/scripts/requirements-base.txt
# the test scripts (apps/scope/test_stream.py)
pip install -q numpy

echo "== Tools that are not fetched"
sdk=${ZEPHYR_SDK_INSTALL_DIR:-$HOME/zephyr-sdk-1.0.1}
check() {
    if eval "$2" >/dev/null 2>&1; then echo "  ok       $1"; else echo "  MISSING  $1: $3"; fi
}
check "Zephyr SDK 1.0.1 ($sdk)" "[ -x '$sdk/gnu/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-gcc' ]" \
      "https://github.com/zephyrproject-rtos/sdk-ng/releases, or set ZEPHYR_SDK_INSTALL_DIR"
check "Efinity 2026.1 (\${EFINITY_SETUP:-~/efinity/2026.1/bin/setup.sh})" \
      "[ -f '${EFINITY_SETUP:-$HOME/efinity/2026.1/bin/setup.sh}' ]" "from Efinix, for the FPGA builds and programming"
check "Java 17 for Efinix's IP generator (\${JAVA_HOME:-~/.local/opt/temurin17-jre})" \
      "[ -x '${JAVA_HOME:-$HOME/.local/opt/temurin17-jre}/bin/java' ] || command -v java" \
      "a JRE 17, e.g. Temurin, in JAVA_HOME"
check "nvc and VUnit for the VHDL testbenches" "command -v nvc && (python3 -c 'import vunit' || /usr/bin/python3 -c 'import vunit')" \
      "optional: https://github.com/nickg/nvc, pip install vunit_hdl"
echo "done: see README.md"
