/*
 * The FPGA's register bus (hVHDL_fpga_interconnect), from Zephyr. See
 * dts/bindings/misc/hvhdl,register-bus-uart.yaml. Calls are serialised; each returns
 * -ETIMEDOUT when the FPGA does not answer.
 */
#ifndef REGISTER_BUS_H
#define REGISTER_BUS_H

#include <stddef.h>
#include <stdint.h>

int register_bus_read(uint16_t address, uint32_t *data);
int register_bus_write(uint16_t address, uint32_t data);
/* count reads of the same address, as the bus's stream command does */
int register_bus_read_stream(uint16_t address, uint32_t *data, size_t count);

#endif
