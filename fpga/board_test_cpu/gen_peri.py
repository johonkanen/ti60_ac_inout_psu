#!/usr/bin/env python3
# Generates board_test_cpu.peri.xml: board_test's interface, from running
# ../board_test/gen_peri.py for this project, plus the JTAG TAP for the SoC's debugger.
# board_test's script is used as it is; only its project name and final save are changed.
# Run with Efinity's python after sourcing setup.sh (build.sh does this).
from pathlib import Path

here = Path(__file__).resolve().parent
source = (here.parent / "board_test" / "gen_peri.py").read_text()

for old, new in [('project = "board_test"', 'project = "board_test_cpu"'),
                 ('\ndesign.save()\n', '\n')]:
    if source.count(old) != 1:
        raise SystemExit(f"board_test/gen_peri.py: expected one {old!r}, found {source.count(old)}")
    source = source.replace(old, new)

# __file__ of this script: board_test's "here" becomes this directory
scope = {"__file__": str(here / "gen_peri.py"), "__name__": "board_test_gen_peri"}
exec(compile(source, str(here.parent / "board_test" / "gen_peri.py"), "exec"), scope)
design = scope["design"]

# the SoC's RISC-V debug module on the FPGA JTAG TAP, instruction USER1
design.create_block("jtag_inst1", "JTAG")
design.assign_resource("jtag_inst1", "JTAG_USER1", "JTAG")

design.save()
