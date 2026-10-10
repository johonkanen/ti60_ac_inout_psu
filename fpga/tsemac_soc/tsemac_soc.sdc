# ram_clk is also the SoC's system and memory clock, io_tseClk its peripheral clock
# (sapphire_subsystem.vhd)
create_clock -period 5.000 -name ram_clk          [get_ports {ram_clk}]
create_clock -period 5.000 -waveform {1.250 3.750} -name ram_clk90 [get_ports {ram_clk90}]
create_clock -period 5.000 -name ram_clk_cal      [get_ports {ram_clk_cal}]
create_clock -period 8.000 -name io_tseClk        [get_ports {io_tseClk}]
create_clock -period 8.000 -name rgmii_rxc        [get_ports {rgmii_rxc}]
create_clock -period 100.000 -name jtag_inst1_TCK [get_ports {jtag_inst1_TCK}]

# As Efinix's design: the HyperRAM clocks are timed together (the controller calibrates the
# read clock's phase); the SoC's peripheral domain with the TSEMAC, the RGMII receive clock and
# JTAG cross through the IPs' synchronisers and FIFOs.
set_clock_groups -asynchronous -group {ram_clk ram_clk90 ram_clk_cal} -group {io_tseClk} -group {rgmii_rxc} -group {jtag_inst1_TCK}

set_false_path -from [get_ports {system_uart_0_io_rxd}]
set_false_path -to   [get_ports {system_uart_0_io_txd}]
