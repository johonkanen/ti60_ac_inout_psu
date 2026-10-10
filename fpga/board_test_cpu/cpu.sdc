
# board_test_cpu: appended to board_test.sdc by build.sh
# The SoC runs from hyperram_clock (constrained above). Its debug module is clocked by the
# JTAG TCK and crosses to the SoC through its own synchronisers.
create_clock -period 100.000 -name jtag_inst1_TCK [get_ports {jtag_inst1_TCK}]
set_clock_groups -asynchronous -group {jtag_inst1_TCK} -group {main_clock hyperram_clock hyperram_ck_clock hyperram_read_clock ethernet_rx_clock}
