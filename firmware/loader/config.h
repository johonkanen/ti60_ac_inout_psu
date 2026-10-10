/*
 * Both Ti60F225 boards: the kit with Efinix's TSEMAC design and ti60_ac_psu with tsemac_soc,
 * which share the SoC's memory map and the W25Q64 flash layout. The slots must match the
 * partitions of the ti60_evm_eth and ti60_psu_eth devicetrees.
 */
#ifndef LOADER_CONFIG_H
#define LOADER_CONFIG_H

#define UART_BASE 0xf8010000
#define SPI_BASE  0xf8014000
#define SPI_CS    0
#define SPI_CLOCK_DIVIDER 1 /* 100 or 125 MHz / ((1 + 1) * 2): 25 or 31 MHz */

#define SLOT_COUNT 2
#define SLOT_FLASH_ADDRESSES { 0x200000, 0x400000 }
#define SLOT_SIZE  0x200000

/* where an image may be loaded: HyperRAM from Zephyr's image region up to the loader */
#define LOAD_START 0x01000000
#define LOAD_END   0x01ff0000 /* the loader runs above, see loader.ld */

#endif
