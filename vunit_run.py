#!/usr/bin/env python3
# VUnit testbenches of the FPGA sources (fpga/), from the root of the repository:
#   python3 vunit_run.py [test pattern] [-p N]
# Uses nvc unless VUNIT_SIMULATOR is set; the output goes to vunit_out/ here.
import os
import sys
from pathlib import Path
from vunit import VUnit

os.environ.setdefault("VUNIT_SIMULATOR", "nvc")
root = Path(__file__).parent / "fpga"

vu = VUnit.from_argv(compile_builtins=False)
vu.add_vhdl_builtins()
lib = vu.add_library("lib")

comm = root / "source" / "fpga_communication"
lib.add_source_files(comm / "hVHDL_fpga_interconnect" / "fpga_interconnect_generic_pkg.vhd")
lib.add_source_files(comm / "fpga_interconnect_16bit_pkg.vhd")

lib.add_source_files(root / "source" / "ethernet" / "*.vhd")
lib.add_source_files(root / "source" / "ethernet" / "sim" / "*.vhd")
lib.add_source_files(root / "source" / "hyperram" / "*.vhd")
lib.add_source_files(root / "source" / "scope" / "*.vhd")
lib.add_source_files(root / "source" / "scope" / "sim" / "*.vhd", allow_empty=True)
lib.add_source_files(root / "source" / "hyperram" / "sim" / "*.vhd")
lib.add_source_files(root / "source" / "adc" / "*.vhd")
lib.add_source_files(root / "source" / "adc" / "sim" / "*.vhd")
fixed_point = root / "source" / "hVHDL_fixed_point"
lib.add_source_files(fixed_point / "fixed_dsp" / "fixed_dsp.vhd")
lib.add_source_files(fixed_point / "fixed_dsp" / "arch_rtl_fixed_dsp.vhd")
# the lookup table divider (voltage_modulator) and what it needs
lib.add_source_files(fixed_point / "lut_interpolation" / "lut_reciprocal_pkg.vhd")
lib.add_source_files(fixed_point / "leading_zeros" / "leading_zeros_pkg.vhd")
lib.add_source_files(fixed_point / "reciprocal_calculator" / "reciprocal_calculator.vhd")
lib.add_source_files(fixed_point / "lut_divider" / "lut_divider.vhd")
memory = fixed_point / "submodules" / "hVHDL_memory_library" / "vhdl2008"
lib.add_source_files(memory / "dp_ram_w_configurable_recrods.vhd")
lib.add_source_files(memory / "arch_rtl_dp_ram_w_configurable_records.vhd")
lib.add_source_files(memory / "mpram_w_configurable_records.vhd")
# hVHDL_microprogram_processor: the core and its fixed point multiply-add (output_control_processor)
mproc = root / "source" / "hVHDL_microprogram_processor" / "rtl"
for name in ["execution_unit", "microprogram_sequencer", "generic_microinstruction_pkg", "microinstruction_pkg",
             "microprogram_assembler_pkg", "microprogram_interface_pkg", "microprogram_core", "arch_fixed_mult_add"]:
    lib.add_source_files(mproc / (name + ".vhd"))
lib.add_source_files(root / "source" / "measurements" / "*.vhd")
lib.add_source_files(root / "source" / "measurements" / "sim" / "*.vhd")
lib.add_source_files(root / "source" / "control" / "*.vhd")
lib.add_source_files(root / "source" / "control" / "sim" / "*.vhd")
# hVHDL_ode's solvers, for the plant models
ode = root / "source" / "hVHDL_ode" / "ode_solvers"
lib.add_source_files(ode / "real_vector_pkg.vhd")
lib.add_source_files(ode / "linalg_pkg.vhd")
lib.add_source_files(ode / "ode_pkg.vhd")
lib.add_source_files(root / "source" / "inverter" / "*.vhd")
lib.add_source_files(root / "source" / "inverter" / "sim" / "*.vhd")
# the output filter's model fitted to a measured step: the data and the fit next to the testbench
fit_data = (root / "source" / "inverter" / "sim" / "data").resolve()
lib.test_bench("output_filter_fit_tb").set_generic("g_data_file", str(fit_data / "output_step_m25_p25.csv"))
lib.test_bench("output_filter_fit_tb").set_generic("g_out_file", str(fit_data / "output_filter_fit.csv"))
lib.test_bench("output_control_processor_tb").set_generic("g_data_file", str(fit_data / "output_step_m25_p25.csv"))
lib.test_bench("output_control_processor_tb").set_generic("g_out_file", str(fit_data / "output_observer_replay.csv"))
lib.test_bench("output_voltage_control_tb").set_generic("g_out_file", str(fit_data / "output_voltage_control.csv"))
lib.add_source_files(root / "source" / "dhb" / "*.vhd")
lib.add_source_files(root / "source" / "dhb" / "sim" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "scope" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "ad_subsystem.vhd")
lib.add_source_files(root / "tsemac_soc" / "gate_power.vhd")
lib.add_source_files(root / "tsemac_soc" / "dhb_test.vhd")
lib.add_source_files(root / "tsemac_soc" / "output_bridge_test.vhd")
lib.add_source_files(root / "tsemac_soc" / "sim" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "remote_update" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "remote_update" / "sim" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "ethernet" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "ethernet" / "sim" / "*.vhd")
lib.add_source_files(root / "tsemac_soc" / "scope" / "sim" / "*.vhd")

tester_tb = lib.test_bench("hyperram_tester_tb")
pattern_test = tester_tb.test("pattern_test")
pattern_test.add_config(name="no_fault", generics=dict(stuck_dq_bit=-1))
pattern_test.add_config(name="stuck_dq3", generics=dict(stuck_dq_bit=3))

loopback_tb = lib.test_bench("ethernet_loopback_tb")
loopback_tb.add_config(name="rx_lo_aligned", generics=dict(rx_lo_lags=False))
loopback_tb.add_config(name="rx_lo_lags", generics=dict(rx_lo_lags=True))

ddr_tb = lib.test_bench("hyperram_ddr_controller_tb")
for tckd in (1.5, 3.0, 4.5):
    for lo_lags in (False, True):
        ddr_tb.add_config(name=f"tckd_{tckd}_lo_{'lags' if lo_lags else 'leads'}",
                          generics=dict(tckd_ns=tckd, lo_lags=lo_lags))

sys.path.append(str(root / "source" / "ethernet" / "sim"))
import ethernet_test_frames
frame_directory = Path(__file__).parent / "vunit_out" / "ethernet_frames"
ethernet_test_frames.write(str(frame_directory))
adc_tb = lib.test_bench("board_adc_tb")
for delay in (6.0, 9.0):   # the board: SDO 5..10 ns after the SCLK rising edges
    adc_tb.add_config(name=f"sdo_delay_{delay}", generics=dict(output_delay_ns=delay))
sdm_tb = lib.test_bench("sdm_adc_tb")
for delay in (10.0, 30.0):  # the isolated modulators' data after the clock's rising edge
    sdm_tb.add_config(name=f"data_delay_{delay}", generics=dict(output_delay_ns=delay))
lib.test_bench("ethernet_stack_tb").set_generic("frame_directory", str(frame_directory))
loopback_tb.set_generic("frame_directory", str(frame_directory))

vu.set_compile_option("nvc.a_flags", ["--relaxed"])
vu.main()
