/*
 * First stage boot code of tsemac_soc, in the SoC's on-chip RAM: sets up the console UART
 * and loads the second-stage loader from the configuration flash into HyperRAM.
 *
 * The loader (firmware/loader) sits at flash 0x600000. Its first word jumps over the
 * magic "Z2LD" in its second word; the third is its size. It is copied to 0x01000000 and
 * started there, as the bootloader of Efinix's example design starts its payload. The
 * HyperRAM is ready: the design keeps the SoC in reset until its calibration passes.
 *
 * After configuration the flash is in deep power-down (the bitstream's SPI low power
 * mode) and reads as all ones until it is woken.
 */
#include <stdint.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

#define PERIPHERAL_HZ 125000000 /* io_tseClk */
#define BAUD          115200

#define UART            0xf8010000
#define UART_DATA       REG(UART + 0x00)
#define UART_STATUS     REG(UART + 0x04)
#define UART_CLOCK      REG(UART + 0x08)
#define UART_FRAME      REG(UART + 0x0c)

#define SPI             0xf8014000
#define SPI_DATA        REG(SPI + 0x00)
#define SPI_BUFFER      REG(SPI + 0x04)
#define SPI_CONFIG      REG(SPI + 0x08)
#define SPI_CLK_DIVIDER REG(SPI + 0x20)
#define SPI_SS_SETUP    REG(SPI + 0x24)
#define SPI_SS_HOLD     REG(SPI + 0x28)
#define SPI_SS_DISABLE  REG(SPI + 0x2c)
#define SPI_FIFO_DEPTH  256

#define LOADER_FLASH  0x600000
#define LOADER_RAM    0x01000000
#define LOADER_MAX    0x10000 /* its partition */
#define LOADER_MAGIC  0x444c325au

static void puts(const char *s)
{
	for (; *s; s++) {
		while (((UART_STATUS >> 16) & 0xff) == 0) {
		}
		UART_DATA = (uint8_t)*s;
	}
}

static void spi_command(uint32_t command)
{
	while ((SPI_BUFFER & 0xffff) == 0) {
	}
	SPI_DATA = command;
}

/* single line read (0x03), commands queued ahead of the responses */
static void flash_read(uint32_t address, uint8_t *p, uint32_t len)
{
	uint32_t sent = 0, received = 0;

	spi_command(0x880);                /* select */
	spi_command(0x100 | 0x03);
	spi_command(0x100 | ((address >> 16) & 0xff));
	spi_command(0x100 | ((address >> 8) & 0xff));
	spi_command(0x100 | (address & 0xff));
	while (received < len) {
		uint32_t buffer = SPI_BUFFER;
		uint32_t space = buffer & 0xffff;
		uint32_t waiting = buffer >> 16;

		while (space && sent < len && sent - received < SPI_FIFO_DEPTH) {
			SPI_DATA = 0x200;      /* read a byte */
			space--;
			sent++;
		}
		while (waiting--) {
			p[received++] = (uint8_t)SPI_DATA;
		}
	}
	spi_command(0x800);                /* deselect */
	while ((SPI_BUFFER & 0xffff) != SPI_FIFO_DEPTH) {
	}
}

void main(void)
{
	uint32_t *loader = (uint32_t *)LOADER_RAM;

	UART_CLOCK = PERIPHERAL_HZ / (BAUD * 8) - 1;
	UART_FRAME = 7; /* 8 data bits, no parity, one stop bit */

	while ((SPI_BUFFER & 0xffff) != SPI_FIFO_DEPTH) {
	}
	while ((SPI_DATA & (1u << 31)) == 0) { /* drop stale responses */
	}
	SPI_CONFIG = 0;
	SPI_CLK_DIVIDER = 1; /* 125 MHz / 4 */
	SPI_SS_SETUP = 5;
	SPI_SS_HOLD = 2;
	SPI_SS_DISABLE = 7;

	/* release from deep power-down, then tRES1 (3 us) */
	spi_command(0x880);
	spi_command(0x100 | 0xab);
	spi_command(0x800);
	while ((SPI_BUFFER & 0xffff) != SPI_FIFO_DEPTH) {
	}
	for (volatile int i = 0; i < 2000; i++) {
	}

	puts("\r\nboot0: tsemac_soc\r\n");
	flash_read(LOADER_FLASH, (uint8_t *)loader, 12);
	if (loader[1] != LOADER_MAGIC || loader[2] < 12 || loader[2] > LOADER_MAX) {
		puts("boot0: no loader at flash 0x600000\r\n");
		return;
	}
	flash_read(LOADER_FLASH, (uint8_t *)loader, loader[2]);
	__asm__ volatile("fence\n"
			 "fence.i\n"
			 "jr %0" ::"r"(LOADER_RAM));
}
