/*
 * The FPGA's register bus through a Sapphire UART wired to fpga_communications, see
 * register_bus.h and hvhdl,register-bus-uart.yaml. Polled: a read is answered within a
 * few tens of microseconds at 5 Mbaud.
 */
#define DT_DRV_COMPAT hvhdl_register_bus_uart

#include <errno.h>
#include <stdlib.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>
#include <register_bus.h>

#define BASE DT_INST_REG_ADDR(0)
#define BAUD DT_INST_PROP(0, current_speed)

/* Sapphire UART */
#define UART_DATA          (BASE + 0x00)
#define UART_STATUS        (BASE + 0x04)
#define UART_CLOCK_DIVIDER (BASE + 0x08)
#define UART_FRAME_CONFIG  (BASE + 0x0c)
#define UART_TX_SPACE      GENMASK(23, 16)
#define UART_RX_WAITING    GENMASK(31, 24)
#define UART_SAMPLES       8 /* per bit */
#define UART_FRAME_8N1     7 /* data bits - 1, no parity, one stop bit */

#define CMD_READ   0x02
#define CMD_WRITE  0x04
#define CMD_STREAM 0x05

BUILD_ASSERT(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC % (BAUD * UART_SAMPLES) == 0,
	     "the bus UART's baud rate must divide the SoC clock exactly");

static K_MUTEX_DEFINE(bus_lock);

static void put(uint8_t byte)
{
	while ((sys_read32(UART_STATUS) & UART_TX_SPACE) == 0) {
	}
	sys_write32(byte, UART_DATA);
}

static void put_be(uint32_t value, int bytes)
{
	while (bytes--) {
		put((uint8_t)(value >> (8 * bytes)));
	}
}

static int get(uint8_t *byte)
{
	uint32_t start = k_cycle_get_32();
	uint32_t timeout = k_us_to_cyc_ceil32(CONFIG_REGISTER_BUS_TIMEOUT_US);

	while ((sys_read32(UART_STATUS) & UART_RX_WAITING) == 0) {
		if (k_cycle_get_32() - start > timeout) {
			return -ETIMEDOUT;
		}
	}
	*byte = (uint8_t)sys_read32(UART_DATA);
	return 0;
}

static int get_be(uint32_t *value, int bytes)
{
	uint8_t byte;

	*value = 0;
	while (bytes--) {
		int ret = get(&byte);

		if (ret) {
			return ret;
		}
		*value = *value << 8 | byte;
	}
	return 0;
}

/* bytes from an earlier, timed out exchange */
static void flush(void)
{
	while (sys_read32(UART_STATUS) & UART_RX_WAITING) {
		(void)sys_read32(UART_DATA);
	}
}

int register_bus_read(uint16_t address, uint32_t *data)
{
	uint32_t length, zero;
	int ret;

	k_mutex_lock(&bus_lock, K_FOREVER);
	flush();
	put(CMD_READ);
	put_be(address, 2);
	/* the reply: its length, an address field fpga_communications always sends as 0, data */
	ret = get_be(&length, 1);
	if (!ret) {
		ret = get_be(&zero, 2);
	}
	if (!ret) {
		ret = get_be(data, 4);
	}
	if (!ret && length != 6) {
		ret = -EIO;
	}
	k_mutex_unlock(&bus_lock);
	return ret;
}

int register_bus_write(uint16_t address, uint32_t data)
{
	k_mutex_lock(&bus_lock, K_FOREVER);
	put(CMD_WRITE);
	put_be(address, 2);
	put_be(data, 4);
	k_mutex_unlock(&bus_lock);
	return 0;
}

int register_bus_read_stream(uint16_t address, uint32_t *data, size_t count)
{
	int ret = 0;

	if (count == 0 || count >= BIT(24)) {
		return -EINVAL;
	}
	k_mutex_lock(&bus_lock, K_FOREVER);
	flush();
	put(CMD_STREAM);
	put_be(address, 2);
	put_be(count, 3);
	for (size_t i = 0; i < count && !ret; i++) {
		ret = get_be(&data[i], 4);
	}
	k_mutex_unlock(&bus_lock);
	return ret;
}

static int register_bus_init(void)
{
	sys_write32(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / (BAUD * UART_SAMPLES) - 1,
		    UART_CLOCK_DIVIDER);
	sys_write32(UART_FRAME_8N1, UART_FRAME_CONFIG);
	flush();
	return 0;
}

SYS_INIT(register_bus_init, POST_KERNEL, 0);

#ifdef CONFIG_REGISTER_BUS_SHELL

static int cmd_read(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t address = (uint16_t)strtoul(argv[1], NULL, 0);
	uint32_t data;
	int ret = register_bus_read(address, &data);

	if (ret) {
		shell_error(sh, "read %u: %d", address, ret);
		return ret;
	}
	shell_print(sh, "%u: 0x%08x %u", address, data, data);
	return 0;
}

static int cmd_write(const struct shell *sh, size_t argc, char **argv)
{
	return register_bus_write((uint16_t)strtoul(argv[1], NULL, 0),
				  (uint32_t)strtoul(argv[2], NULL, 0));
}

static int cmd_stream(const struct shell *sh, size_t argc, char **argv)
{
	uint16_t address = (uint16_t)strtoul(argv[1], NULL, 0);
	size_t count = strtoul(argv[2], NULL, 0);
	uint32_t data[16];
	int ret;

	if (count == 0 || count > ARRAY_SIZE(data)) {
		shell_error(sh, "1 to %u words", (unsigned int)ARRAY_SIZE(data));
		return -EINVAL;
	}
	ret = register_bus_read_stream(address, data, count);
	for (size_t i = 0; i < count && !ret; i++) {
		shell_print(sh, "%u: 0x%08x", address, data[i]);
	}
	if (ret) {
		shell_error(sh, "stream %u: %d", address, ret);
	}
	return ret;
}

SHELL_STATIC_SUBCMD_SET_CREATE(bus_cmds,
	SHELL_CMD_ARG(read, NULL, "read <address>", cmd_read, 2, 0),
	SHELL_CMD_ARG(write, NULL, "write <address> <value>", cmd_write, 3, 0),
	SHELL_CMD_ARG(stream, NULL, "stream <address> <count>, up to 16", cmd_stream, 3, 0),
	SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(bus, &bus_cmds, "FPGA register bus", NULL);

#endif
