create_clock -period 8.000  -name main_clock        [get_ports {main_clock}]
create_clock -period 8.000  -name ethernet_rx_clock [get_ports {ethernet_rx_clock}]
create_clock -period 5.000 -name hyperram_clock      [get_ports {hyperram_clock}]
create_clock -period 5.000 -waveform {1.250 3.750} -name hyperram_ck_clock [get_ports {hyperram_ck_clock}]
create_clock -period 5.000 -name hyperram_read_clock [get_ports {hyperram_read_clock}]

# the domains exchange data through async FIFOs and synchronizers only; the read clock phase
# is set at run time by calibration, so the read capture is not timed against the outputs
set_clock_groups -asynchronous -group {main_clock} -group {hyperram_clock hyperram_ck_clock} -group {hyperram_read_clock} -group {ethernet_rx_clock}

set_false_path -from [get_ports {uart_rx}]
set_false_path -to   [get_ports {uart_tx}]
