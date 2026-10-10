/*
 * Second-stage loader: picks the image slot to boot, loads it into HyperRAM, checks it
 * and starts both harts in it. See sapphire_loader.h for the slot format.
 *
 * Candidates are the slots with a valid header, the highest sequence first. A slot
 * whose image does not load with the right CRC is passed over for the next one.
 */
#include <stddef.h>
#include <stdint.h>
#include "../module/include/sapphire_loader.h"
#include "config.h"

#define REG(addr) (*(volatile uint32_t *)(addr))

#define UART_DATA   REG(UART_BASE + 0x00)
#define UART_STATUS REG(UART_BASE + 0x04)

#define SPI_DATA        REG(SPI_BASE + 0x00)
#define SPI_BUFFER      REG(SPI_BASE + 0x04)
#define SPI_CONFIG      REG(SPI_BASE + 0x08)
#define SPI_CLK_DIVIDER REG(SPI_BASE + 0x20)
#define SPI_SS_SETUP    REG(SPI_BASE + 0x24)
#define SPI_SS_HOLD     REG(SPI_BASE + 0x28)
#define SPI_SS_DISABLE  REG(SPI_BASE + 0x2c)
#define SPI_CMD_WRITE   (1u << 8)
#define SPI_CMD_READ    (1u << 9)
#define SPI_CMD_SS      (1u << 11)
#define SPI_SS_ENABLE   (1u << 7)
#define SPI_FIFO_DEPTH  256 /* both FIFOs, SYSTEM_SPI_0_IO_PARAMETER_*_FIFO_DEPTH */

#define FLASH_READ 0x03

/* set by start.S's parking loop and read by it */
volatile uint32_t secondary_parked;
volatile uint32_t secondary_entry;

static uint32_t crc_table[256];

static void putc(char c)
{
	while (((UART_STATUS >> 16) & 0xff) == 0) {
	}
	UART_DATA = (uint8_t)c;
}

static void puts(const char *s)
{
	while (*s) {
		if (*s == '\n') {
			putc('\r');
		}
		putc(*s++);
	}
}

static void puthex(uint32_t value)
{
	for (int shift = 28; shift >= 0; shift -= 4) {
		putc("0123456789abcdef"[(value >> shift) & 0xf]);
	}
}

static void putdec(uint32_t value)
{
	char digits[10];
	int n = 0;

	do {
		digits[n++] = '0' + value % 10;
		value /= 10;
	} while (value);
	while (n) {
		putc(digits[--n]);
	}
}

static void crc_init(void)
{
	for (uint32_t i = 0; i < 256; i++) {
		uint32_t c = i;

		for (int k = 0; k < 8; k++) {
			c = (c & 1) ? 0xedb88320u ^ (c >> 1) : c >> 1;
		}
		crc_table[i] = c;
	}
}

static uint32_t crc32(const uint8_t *data, size_t len)
{
	uint32_t c = 0xffffffffu;

	while (len--) {
		c = crc_table[(c ^ *data++) & 0xff] ^ (c >> 8);
	}
	return c ^ 0xffffffffu;
}

static uint32_t spi_space(void)
{
	return SPI_BUFFER & 0xffff;
}

static void spi_command(uint32_t command)
{
	while (spi_space() == 0) {
	}
	SPI_DATA = command;
}

static void spi_init(void)
{
	while (spi_space() != SPI_FIFO_DEPTH) {
	}
	while ((SPI_DATA & (1u << 31)) == 0) { /* drop leftover responses */
	}
	SPI_CONFIG = 0;
	SPI_CLK_DIVIDER = SPI_CLOCK_DIVIDER;
	SPI_SS_SETUP = 5;
	SPI_SS_HOLD = 2;
	SPI_SS_DISABLE = 7;
}

/* read len bytes of flash at address into memory, keeping the command FIFO full */
static void flash_read(uint32_t address, void *destination, uint32_t len)
{
	uint8_t *p = destination;
	uint32_t sent = 0, received = 0;

	spi_command(SPI_CMD_SS | SPI_SS_ENABLE | SPI_CS);
	spi_command(SPI_CMD_WRITE | FLASH_READ);
	spi_command(SPI_CMD_WRITE | ((address >> 16) & 0xff));
	spi_command(SPI_CMD_WRITE | ((address >> 8) & 0xff));
	spi_command(SPI_CMD_WRITE | (address & 0xff));
	while (received < len) {
		uint32_t buffer = SPI_BUFFER;
		uint32_t space = buffer & 0xffff;
		uint32_t waiting = buffer >> 16;

		while (space && sent < len && sent - received < SPI_FIFO_DEPTH) {
			SPI_DATA = SPI_CMD_READ;
			space--;
			sent++;
		}
		while (waiting--) {
			p[received++] = (uint8_t)SPI_DATA;
		}
	}
	spi_command(SPI_CMD_SS | SPI_CS);
	while (spi_space() != SPI_FIFO_DEPTH) {
	}
}

static int header_valid(const struct slot_header *h)
{
	return h->magic == SLOT_MAGIC && h->header_size == sizeof(*h) &&
	       crc32((const uint8_t *)h, offsetof(struct slot_header, header_crc32)) ==
		       h->header_crc32 &&
	       h->size > 0 && h->size <= SLOT_SIZE - SLOT_IMAGE_OFFSET &&
	       h->load_address >= LOAD_START && h->size <= LOAD_END - h->load_address &&
	       h->entry >= h->load_address && h->entry < h->load_address + h->size;
}

static void boot(const struct slot_header *h)
{
	void (*entry)(void) = (void (*)(void))h->entry;

	__asm__ volatile("fence" ::: "memory");
	secondary_entry = h->entry; /* hart 1, if parked here, starts the image too */
	__asm__ volatile("fence\n"
			 "fence.i" ::: "memory");
	entry();
}

void main(void)
{
	static const uint32_t addresses[SLOT_COUNT] = SLOT_FLASH_ADDRESSES;
	static struct slot_header headers[SLOT_COUNT];
	int order[SLOT_COUNT], count = 0;

	/* hart 1 normally parks within microseconds; after a warm reboot it never comes */
	for (volatile int i = 0; i < 200000 && !secondary_parked; i++) {
		__asm__ volatile(".word 0x500f"); /* invalidate the data cache */
	}

	crc_init();
	spi_init();
	puts("\nloader: hart 1 ");
	puts(secondary_parked ? "parked\n" : "absent\n");

	for (int s = 0; s < SLOT_COUNT; s++) {
		flash_read(addresses[s], &headers[s], sizeof(headers[s]));
		puts("loader: slot ");
		putdec(s);
		if (!header_valid(&headers[s])) {
			puts(" empty\n");
			continue;
		}
		puts(" sequence ");
		putdec(headers[s].sequence);
		puts(", ");
		putdec(headers[s].size);
		puts(" bytes\n");
		/* insertion by sequence, highest first */
		int i = count++;

		while (i > 0 && headers[order[i - 1]].sequence < headers[s].sequence) {
			order[i] = order[i - 1];
			i--;
		}
		order[i] = s;
	}

	for (int i = 0; i < count; i++) {
		const struct slot_header *h = &headers[order[i]];
		uint32_t crc;

		flash_read(addresses[order[i]] + SLOT_IMAGE_OFFSET, (void *)h->load_address,
			   h->size);
		crc = crc32((const uint8_t *)h->load_address, h->size);
		puts("loader: slot ");
		putdec(order[i]);
		if (crc != h->crc32) {
			puts(" crc ");
			puthex(crc);
			puts(" != ");
			puthex(h->crc32);
			puts(", skipped\n");
			continue;
		}
		puts(" ok, starting at ");
		puthex(h->entry);
		puts("\n");
		boot(h);
	}

	puts("loader: no image to boot\n");
	for (;;) {
		__asm__ volatile("wfi");
	}
}
