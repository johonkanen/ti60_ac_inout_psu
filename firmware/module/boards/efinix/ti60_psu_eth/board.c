/*
 * Reboot for the TSEMAC design, which has no reset the CPU can trigger: hart 0 restarts
 * the design's flash bootloader at the reset address, which loads the image from flash
 * again and jumps to it, as at power-up.
 *
 * Hart 1 is not restarted. Zephyr parks it in a wfi loop with mie = 0, so it never
 * wakes and the bootloader can overwrite the RAM around it. Before the jump, nothing
 * else may write HyperRAM: the TSEMAC's DMA is stopped, as receive frames would land
 * in the middle of the image being copied, and so is the scope's (its capture ring, wherever
 * this image had it: an update to an image with another layout hung in its start when it
 * went on writing there).
 */
#include <zephyr/arch/cpu.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/sys_io.h>

#define BOOTLOADER_ADDRESS 0xf9000000 /* the SoC's reset address, in on-chip RAM */

#define ETH_NODE DT_NODELABEL(eth0)
#define SCOPE_NODE DT_NODELABEL(scope)

/* TSEMAC registers, see drivers/ethernet/eth_efinix_tsemac.c */
#define MAC_COMMAND_CONFIG 0x008
#define CMD_TX_RX_ENA      (BIT(0) | BIT(1))
#define DMA_RX             0x00
#define DMA_TX             0x80
#define DMA_STATUS         0x2c
#define DMA_STATUS_BUSY    BIT(0)
#define DMA_STATUS_STOP    BIT(2)
#define DMA_INTERRUPT_ENABLE 0x50

static void tsemac_stop(void)
{
#if DT_NODE_HAS_STATUS_OKAY(ETH_NODE)
	uintptr_t mac = DT_REG_ADDR_BY_NAME(ETH_NODE, mac);
	uintptr_t dma = DT_REG_ADDR_BY_NAME(ETH_NODE, dma);

	sys_write32(sys_read32(mac + MAC_COMMAND_CONFIG) & ~CMD_TX_RX_ENA,
		    mac + MAC_COMMAND_CONFIG);
	for (uintptr_t channel = dma + DMA_RX; channel <= dma + DMA_TX; channel += DMA_TX) {
		sys_write32(0, channel + DMA_INTERRUPT_ENABLE);
		sys_write32(DMA_STATUS_STOP, channel + DMA_STATUS);
		for (int i = 0; i < 100000 && (sys_read32(channel + DMA_STATUS) & DMA_STATUS_BUSY);
		     i++) {
		}
	}
#endif
}

/* the scope's DMA channel (its capture into the HyperRAM) */
static void scope_dma_stop(void)
{
#if DT_NODE_HAS_STATUS_OKAY(SCOPE_NODE)
	uintptr_t channel = DT_REG_ADDR_BY_NAME(SCOPE_NODE, dma);

	sys_write32(0, channel + DMA_INTERRUPT_ENABLE);
	sys_write32(DMA_STATUS_STOP, channel + DMA_STATUS);
	for (int i = 0; i < 100000 && (sys_read32(channel + DMA_STATUS) & DMA_STATUS_BUSY); i++) {
	}
#endif
}

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	/* sys_reboot() has already locked interrupts */
	tsemac_stop();
	scope_dma_stop();
	csr_write(mie, 0);
	__asm__ volatile("fence\n"
			 ".word 0x500f\n" /* invalidate the data cache */
			 "fence.i\n"      /* nothing at the old image's addresses stays cached */
			 "jr %0" ::"r"(BOOTLOADER_ADDRESS));
	CODE_UNREACHABLE;
}
