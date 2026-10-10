#!/bin/bash
# RTL simulation of uart_test_top with nvc
set -e
cd "$(dirname "$0")"
C=../../source/fpga_communication
nvc --std=2019 -a \
    $C/hVHDL_fpga_interconnect/fpga_interconnect_generic_pkg.vhd \
    $C/fpga_interconnect_16bit_pkg.vhd \
    $C/hVHDL_uart/uart_rx/uart_rx_pkg.vhd \
    $C/hVHDL_uart/uart_tx/uart_tx_pkg.vhd \
    $C/serial_protocol_generic_pkg.vhd \
    $C/communications.vhd \
    ../uart_test_top.vhd \
    uart_test_tb.vhd \
    -e uart_test_tb -r
