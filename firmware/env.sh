# source env.sh -> Zephyr build environment for this workspace.
# A shell that has sourced Efinity's setup.sh has PYTHONHOME/PYTHONPATH pointing at Efinity's
# own python, which breaks every other interpreter. Zephyr needs python >= 3.12 (.venv).
unset PYTHONHOME PYTHONPATH PYTHONNOUSERSITE
source "$(dirname "${BASH_SOURCE[0]}")/.venv/bin/activate"
