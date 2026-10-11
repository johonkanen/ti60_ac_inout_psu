# ram_clk is also the SoC's system and memory clock, io_tseClk its peripheral clock
# (sapphire_subsystem.vhd)
create_clock -period 5.000 -name ram_clk          [get_ports {ram_clk}]
create_clock -period 5.000 -waveform {1.250 3.750} -name ram_clk90 [get_ports {ram_clk90}]
create_clock -period 5.000 -name ram_clk_cal      [get_ports {ram_clk_cal}]
create_clock -period 8.000 -name io_tseClk        [get_ports {io_tseClk}]
create_clock -period 8.000 -name rgmii_rxc        [get_ports {rgmii_rxc}]
create_clock -period 100.000 -name jtag_inst1_TCK [get_ports {jtag_inst1_TCK}]

# The HyperRAM clocks come from one PLL and are timed together; the SoC's peripheral domain
# with the TSEMAC, the RGMII receive clock and JTAG cross through the IPs' synchronisers and
# FIFOs.
set_clock_groups -asynchronous -group {ram_clk ram_clk90 ram_clk_cal} -group {io_tseClk} -group {rgmii_rxc} -group {jtag_inst1_TCK}

# hyperram_axi's read data: the input DDIO registers on ram_clk_cal (ram_clk delayed by its
# calibrated phase, 0..1.75 ns) to one register on ram_clk a cycle later, the only paths
# between the two. Setup: at most 3 ns (5 ns less the largest phase and some margin); hold is
# checked as usual, at phase 0 (a later phase only adds to it).
set_max_delay 3.0 -from [get_clocks ram_clk_cal] -to [get_clocks ram_clk]
# The input DDIO registers' outputs, as the Interface Designer gives them (outflow_*/
# *.pt.sdc: the reference pins follow from the pins, fixed in gen_peri.py), so the paths
# above are timed at all: without them the paths out of the interface are not analysed.
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~243}] -max 0.414 [get_ports {hbc_dq_IN_LO[0] hbc_dq_IN_HI[0]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~243}] -min 0.276 [get_ports {hbc_dq_IN_LO[0] hbc_dq_IN_HI[0]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~169}] -max 0.414 [get_ports {hbc_dq_IN_LO[1] hbc_dq_IN_HI[1]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~169}] -min 0.276 [get_ports {hbc_dq_IN_LO[1] hbc_dq_IN_HI[1]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~278}] -max 0.414 [get_ports {hbc_dq_IN_LO[2] hbc_dq_IN_HI[2]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~278}] -min 0.276 [get_ports {hbc_dq_IN_LO[2] hbc_dq_IN_HI[2]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~256}] -max 0.414 [get_ports {hbc_dq_IN_LO[3] hbc_dq_IN_HI[3]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~256}] -min 0.276 [get_ports {hbc_dq_IN_LO[3] hbc_dq_IN_HI[3]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~266}] -max 0.414 [get_ports {hbc_dq_IN_LO[4] hbc_dq_IN_HI[4]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~266}] -min 0.276 [get_ports {hbc_dq_IN_LO[4] hbc_dq_IN_HI[4]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~244}] -max 0.414 [get_ports {hbc_dq_IN_LO[5] hbc_dq_IN_HI[5]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~244}] -min 0.276 [get_ports {hbc_dq_IN_LO[5] hbc_dq_IN_HI[5]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~179}] -max 0.414 [get_ports {hbc_dq_IN_LO[6] hbc_dq_IN_HI[6]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~179}] -min 0.276 [get_ports {hbc_dq_IN_LO[6] hbc_dq_IN_HI[6]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~178}] -max 0.414 [get_ports {hbc_dq_IN_LO[7] hbc_dq_IN_HI[7]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~178}] -min 0.276 [get_ports {hbc_dq_IN_LO[7] hbc_dq_IN_HI[7]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~255}] -max 0.414 [get_ports {hbc_rwds_IN_LO[0] hbc_rwds_IN_HI[0]}]
set_input_delay -clock ram_clk_cal -reference_pin [get_ports {ram_clk_cal~CLKOUT~1~255}] -min 0.276 [get_ports {hbc_rwds_IN_LO[0] hbc_rwds_IN_HI[0]}]


set_false_path -from [get_ports {system_uart_0_io_rxd}]
set_false_path -to   [get_ports {system_uart_0_io_txd}]
