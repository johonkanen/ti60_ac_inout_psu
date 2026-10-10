#!/usr/bin/env python3
# Generates the IPs of tsemac_soc with the Efinity IP Manager API (needs java on PATH;
# build.sh ips sets it up). ip_settings.py holds each IP's parameters; the changes for this
# board are the overrides below, by the IP's name in the design.
# Run with Efinity's python after sourcing setup.sh.
import os
import sys
from pathlib import Path

sys.path.append(os.environ['EFXIPM_HOME'] + "/bin")
from ipm_api_service.design import IPMDesignAPI

here = Path(__file__).resolve().parent
sys.path.insert(0, str(here))
from ip_settings import ip_settings

overrides = {
    # Zephyr, not Linux: one hart, no MMU or FPU; the EVM's memory map, UART0, SPI0 for
    # the configuration flash and APB slaves 0 (TSEMAC) and 1 (DMA) are kept. Added for the
    # scope (scope/scope_subsystem.vhd): APB slave 2 for its DMA, APB slave 3 for the
    # register bus, DDR master port 1 for its DMA.
    "SapphireCore": {
        "CpuCount": "1",
        "LINUX": "1'b0",
        "FPU": "1'b0",
        "FPEXT": "1'b0",
        "MULDIV_EXT": "1'b1",
        "ATMEXT": "1'b1",
        "CMREXT": "1'b1",
        "REDUCED_CSR": "1'b0",
        "DDRSize": "32'd67108864",       # 64 MB W959D8NFYA
        "PeriFrequency": "125",           # io_tseClk: the PLL has no further global clock
        "SPI1": "1'b0",                   # the EVM's SD card
        "I2C0": "1'b0",
        "GPIO0": "1'b0",
        "HexFile_Path": '"bootloader/boot0.hex"',  # bootloader/boot0.c
        "APBSlave2": "1'b1",
        "APBSlave3": "1'b1",
        "AXIMaster_1": "1'b1",
        "AXIMasterWidth_1": "64",
    },
    # the board's x8 HyperRAM; read calibration shifts PLL output 3 (one-hot)
    "hy0_ram": {
        "RAM_DBW": "8",
        "RAM_ABW": "26",
        "CAL_CLK_CH": "8",
    },
    # RGMII receive: the board's DDIO resync inputs give the rising edge nibble one clock
    # before its falling edge partner, which is the IP's "rising edge" pairing. Transmit:
    # the PHY delays the clock (its MIICTRL register), not the FPGA.
    "efx_tsemac": {
        "RGMII_RXC_EDGE": "1'b0",
        "RGMII_TXC_DLY": "1'b0",
    },
}

# the scope's DMA: one stream input channel written in direct mode around a circular
# buffer (self restart), 64 bit stream in the scope's clock domain
overrides["dma_scope"] = {
    "CH0_SGMode": "1'b0",
    "CH0_SR": "1'b1",
    "CH0_Width": "64",
    "CH0_Input": "1'b1",
    "CH0_Output": "1'b0",
    "CH0_AsyncMode": "1'b1",
    "CH0_BurstSize": "256",
    "CH0_BufferSize": "2048",
    "CH1_Enable": "1'b0",
}

# name in the design -> its parameters in ip_settings.py
ips = {
    "SapphireCore": "SapphireCore",
    "hy0_ram": "hyperram",
    "efx_tsemac": "tsemac",
    "dma": "dma",
    "dma_scope": "dma",
    "apb3_2_axi4_lite": "apb3_2_axi4_lite",
}

only = sys.argv[1:]
design = IPMDesignAPI("Ti60F225", "Titanium", str(here), is_verbose=False)
for name, config in ips.items():
    if only and name not in only:
        continue
    settings = ip_settings[config]
    conf = dict(settings["conf"], **overrides.get(name, {}))
    print(f"{name}: {settings['library']}/{settings['name']} ({settings['version']})")
    design.create_ip(name, settings["vendor"], settings["library"], settings["name"])
    design.config_ip(name, conf)
    ok, results, params = design.validate_ip(name)
    if not ok:
        sys.exit(f"{name}: validation failed: {results}")
    design.generate_ip(name)
