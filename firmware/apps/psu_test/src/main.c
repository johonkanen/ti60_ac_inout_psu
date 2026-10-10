/*
 * First run on the ti60_ac_psu board: board_test's register bus from Zephyr, the checks
 * of fpga/scripts/uart_ping.py. Then the shell, with "bus read/write/stream".
 */
#include <zephyr/kernel.h>
#include <register_bus.h>

#define ID_WORD       1
#define COUNTER       2 /* main_clock, 125 MHz */
#define SCRATCH       3
#define PLL_LOCKED    4
#define MAIN_CLOCK_HZ 125000000

static int failures;

static void check(const char *name, bool ok, const char *format, uint32_t a, uint32_t b)
{
	printk("%s  %-12s ", ok ? "PASS" : "FAIL", name);
	printk(format, a, b);
	printk("\n");
	failures += !ok;
}

int main(void)
{
	static const uint32_t patterns[] = {0x12345678, 0xa5a5a5a5, 0x5a5a5a5a, 0};
	uint32_t value, c0, c1, t0, t1, mhz_x100;
	int ret;

	printk("psu_test on %s\n", CONFIG_BOARD_TARGET);
	k_msleep(100); /* past the shell's first prompt, which would land in the middle */

	ret = register_bus_read(ID_WORD, &value);
	check("id word", ret == 0 && value == 0xacdc, "0x%08x (%d)", value, ret);

	ret = register_bus_read(COUNTER, &c0);
	t0 = k_cycle_get_32();
	k_msleep(1000);
	ret |= register_bus_read(COUNTER, &c1);
	t1 = k_cycle_get_32();
	mhz_x100 = (uint32_t)((uint64_t)(c1 - c0) * sys_clock_hw_cycles_per_sec() / (t1 - t0) /
			      10000);
	check("main_clock", ret == 0 && mhz_x100 > 12375 && mhz_x100 < 12625,
	      "%u.%02u MHz", mhz_x100 / 100, mhz_x100 % 100);

	for (int i = 0; i < ARRAY_SIZE(patterns); i++) {
		ret = register_bus_write(SCRATCH, patterns[i]);
		ret |= register_bus_read(SCRATCH, &value);
		check("scratch", ret == 0 && value == patterns[i], "wrote 0x%08x read 0x%08x",
		      patterns[i], value);
	}

	ret = register_bus_read(PLL_LOCKED, &value);
	check("pll locked", ret == 0 && value == 1, "%u (%d)", value, ret);

	t0 = k_cycle_get_32();
	for (int i = 0; i < 1000; i++) {
		register_bus_read(COUNTER, &value);
	}
	t1 = k_cycle_get_32();
	printk("1000 reads in %u us\n", k_cyc_to_us_floor32(t1 - t0));

	printk(failures ? "%d check(s) failed\n" : "board OK\n", failures);
	return 0;
}
