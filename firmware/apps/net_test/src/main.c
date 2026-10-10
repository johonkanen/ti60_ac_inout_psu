/*
 * Ethernet bring-up for the Efinix TSEMAC: gets an IPv4 address with DHCP and
 * answers ping. Use the shell over the console UART: net iface, net arp,
 * net ping, net stats.
 */
#include <zephyr/kernel.h>

int main(void)
{
	printk("net_test on %s\n", CONFIG_BOARD_TARGET);
	return 0;
}
