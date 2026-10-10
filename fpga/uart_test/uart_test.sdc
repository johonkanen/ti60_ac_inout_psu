create_clock -period 8.333  -name main_clock       [get_ports {main_clock}]

set_false_path -from [get_ports {uart_rx}]
set_false_path -to   [get_ports {uart_tx}]
