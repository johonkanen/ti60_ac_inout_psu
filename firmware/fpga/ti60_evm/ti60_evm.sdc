create_clock -period 10.000 -name system_clock   [get_ports {system_clock}]
create_clock -period 100.00 -name jtag_inst1_TCK [get_ports {jtag_inst1_TCK}]
set_clock_groups -exclusive -group {system_clock} -group {jtag_inst1_TCK}

set_false_path -from [get_ports {reset_n uart_rx gpio_in[*]}]
set_false_path -to   [get_ports {uart_tx gpio_out[*] gpio_oe[*]}]
