# ti60_ac_inout_psu

An AC in, AC out lab power supply, meant as a high power arbitrary output voltage waveform
generator (the hardware is designed for 400 V, 450 V at most), and its control board: an
Efinix Titanium Ti60F225 FPGA with a
Sapphire RISC-V SoC running Zephyr, HyperRAM, Gigabit Ethernet, two ADS7056 ADCs behind
analog muxes, and the power stage's gate drives. The board serves a web page with an
oscilloscope, a data collection page that streams the FPGA's capture at up to 62.5 MB/s
([docs/data_stream.md](docs/data_stream.md) tells how), and an
update page that installs firmware, web pages and FPGA images over the network, so JTAG is
needed only once, to put the golden FPGA image in the flash.

| Path | What |
|---|---|
| `fpga/` | the FPGA designs and their VHDL ([details](#fpga)) |
| `fpga/tsemac_soc/` | the design the board runs: Sapphire SoC, TSEMAC Ethernet, scope, ADCs, remote update |
| `fpga/board_test/`, `fpga/board_test_cpu/` | the board's hardware test design, and it with a CPU |
| `fpga/source/` | VHDL shared by the designs: the power stage's modulators (`dhb/`, `inverter/`), measurements, control, ADCs, scope, HyperRAM, Ethernet; `fpga_communication`, `hVHDL_fixed_point` (fixed point math cores), `hVHDL_ode` (differential equation solvers and power stage models), `hVHDL_memory_library` (block RAMs and the HyperRAM controller) and `hVHDL_microprogram_processor` (a small fixed point processor for control programs) are submodules |
| `firmware/` | Zephyr: boards, drivers, libraries, apps, the second-stage loader ([README](firmware/README.md)) |
| `firmware/apps/scope/` | the board's application and its web pages |
| `firmware/fpga/ti60_psu/` | scripts for the board: JTAG load, flash, console |
| `west.yml` | Zephyr v4.4.2 and the modules the firmware uses |
| `setup.sh` | the one-time setup of a clone |

## Setup

Tools that are not in the repository, at these paths or given by the variables:

- Zephyr SDK 1.0.1 in `~/zephyr-sdk-1.0.1` (`ZEPHYR_SDK_INSTALL_DIR`)
- Efinity 2026.1 in `~/efinity/2026.1` (`EFINITY_SETUP`, its `bin/setup.sh`), for the FPGA
  builds, OpenOCD and programming
- a Java 17 runtime in `~/.local/opt/temurin17-jre` (`JAVA_HOME`), for Efinix's IP generator
- Python 3.12 or newer, or [uv](https://docs.astral.sh/uv/) to fetch one
- optional, for the VHDL testbenches: [nvc](https://github.com/nickg/nvc) and VUnit
  (`pip install vunit_hdl`)

Clone into a directory of its own, which becomes the west workspace with Zephyr beside the
clone, and run `setup.sh`: it fetches the submodules and Zephyr, makes `firmware/.venv`, and
says which of the tools above it does not find.

```
mkdir psu && cd psu
git clone https://github.com/johonkanen/ti60_ac_inout_psu
ti60_ac_inout_psu/setup.sh
```

## Build

FPGA, in `fpga/tsemac_soc`:

```
./build.sh ips          # Efinix's IP cores (Sapphire, TSEMAC, DMAs, APB bridge): once, about 8 minutes
./build.sh all          # both images at once (about 4 minutes), then images/flash.hex
                        # and images/application_update.bin
```

`./build.sh golden` and `./build.sh` build one image each, `./build.sh images` combines
them; each role builds in a project and output directory of its own (`outflow_<role>/`).

Firmware, in `firmware/` with its environment (`source env.sh`):

```
west build -b ti60_psu_eth -d build/scope_psu apps/scope    # the board's application
loader/build.sh                                             # the second-stage loader
```

`fpga/tsemac_soc/bootloader/build.sh` rebuilds `boot0.hex`, the FPGA's on-chip boot code,
which is in git since the bitstream needs it.

VHDL testbenches, from the root of the repository: `python3 vunit_run.py` (with a Python that has VUnit; not one
with Efinity's `PYTHONHOME` set).

## Program and update

Once, with the power stage off (JTAG is not safe while it runs): `firmware/fpga/ti60_psu/flash.sh
build/scope_psu/zephyr/zephyr.bin` from `firmware/` writes the golden and application FPGA
images, the loader and the firmware to the flash.

After that everything goes over the network, from the board's Update page
(`http://<board>/update.html`) or a script:

| File | What |
|---|---|
| `firmware/build/scope_psu/zephyr/zephyr.bin` | firmware |
| `firmware/build/loader/loader.bin` | the second-stage loader |
| `fpga/tsemac_soc/images/application_update.bin` | the FPGA application image |
| `firmware/apps/scope/web/*` | the web pages (`apps/scope/upload_web.sh <board>`) |

`firmware/fpga/ti60_evm_eth/update.sh <file> <board>` does the same from the command line.
How firmware and FPGA updates stay recoverable is in `firmware/README.md` and
`firmware/module/include/fpga_update.h`.

## Architecture

The power stage converts in three steps: an input bridge from the AC input to the primary DC
link, a dual half bridge (DHB, a dual active bridge with half bridge legs) across an isolating
transformer to the secondary DC link, and an output full bridge that makes the output voltage
through a two stage LC filter. Six isolated supplies power the gate drivers and the analog
side of the current measurements.

Measurements: two ADS7056 ADCs read the DC links and the filter voltages through analog muxes,
and three sigma-delta modulators (ADuM7702) the input, output and DHB currents; the FPGA
filters the sigma-delta streams and takes the output current's mean over each switching
period. `measurement_scaler` turns all of them into volts and amperes.

The control runs in the FPGA, at the switching rate:

- the DHB: precharge of the secondary DC link with a widening pulse, then its phase shift
  from a PI on the secondary's voltage, with overcurrent and overvoltage trips;
- the output: an observer of the output filter (a model of it pulled to the measurements by
  two PIs, which give the bridge's voltage error and the load current) runs twice a switching
  period; a cascaded control steps once a period from its estimates. The reference goes
  through a prefilter, its slope fed forward as the capacitor current; a voltage PI gives the
  inductor current reference and a current PI the inductor voltage, both with anti-windup; the
  bridge's voltage reference adds the capacitor voltage and R i and takes off the learned
  bridge error. The control predicts the filter's state a period ahead, since the bridge
  applies its reference from the next period. Both run as programs on
  `hVHDL_microprogram_processor` (`fpga/source/control/output_control_program_pkg.vhd`), one
  fixed point multiply-add; `output_observer_pkg` and `output_voltage_control_pkg` hold the
  same algorithms in real numbers, for design, tuning (`output_voltage_control_tb`) and as the
  reference the processor is tested against.
- the voltage modulator turns the bridge's voltage reference into a duty with the measured DC
  link; the full bridge modulator switches at 40 kHz with dead time.

The CPU (Zephyr) sets the controls' parameters and runs the tests over a register bus, and
serves the web pages: the oscilloscope (any 8 of the FPGA's signals, its deep capture
streamed through a DMA into the HyperRAM), the data collection page and the update page.

## FPGA

`fpga/tsemac_soc` is the design the board runs, with a VHDL top level (`tsemac_soc_top.vhd`):
Efinix's IP cores in `sapphire_subsystem.vhd` (the SoC, the scope's DMA, and in
`ethernet/ethernet_subsystem.vhd` the Ethernet MAC and its DMA, with a transmit checksum
inserter), our HyperRAM controller (`hyperram_axi` from hVHDL_memory_library) as the SoC's
main memory, a register bus for the CPU (scope at register 600, ADCs at 700 and 720, sigma-delta
at 740, gate supplies at 760, the DHB at 780 and 1100, the output bridge at 1140, its observer
at 1160 and control at 1180, remote update at 800), the oscilloscope (the
[fpga_scope](https://github.com/johonkanen/fpga_scope) submodule), the measurements and
controls above, and the internal reconfiguration that switches between the golden and
application images. The IPs' configuration started from Efinix's Ti60F225 TSEMAC example
design's.

## Licence

MIT (`LICENSE`); `firmware/apps/scope/web/uPlot.iife.min.js` is uPlot's, MIT
(`LICENSE.uPlot`). The FPGA designs use Efinix's IP cores (the Sapphire RISC-V SoC, the
Triple Speed Ethernet MAC, the DMA, the APB to AXI4-Lite bridge),
which are not here: `fpga/tsemac_soc/gen_ips.py` generates them with Efinity's IP Manager
from the parameters in `ip_settings.py`, under Efinix's licence for them. The Ti60F225 kit
(`ti60_evm*` boards, `firmware/fpga/ti60_evm_eth/`) runs Efinix's TSEMAC Linux example
design, which is not here either.
