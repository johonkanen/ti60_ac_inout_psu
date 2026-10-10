/*
 * Flash format shared by the second-stage loader (loader/) and the firmware update
 * (module/lib/fw_update), plain C for both.
 *
 * Images live in two slots. A slot starts with a header and has the image
 * SLOT_IMAGE_OFFSET bytes in. The loader boots the valid slot with the highest
 * sequence number, and the other one if that image does not check out. An update writes
 * the slot that is not running, the header last, so it only counts once it is complete.
 *
 * The loader itself is recognised by LOADER_MAGIC in its second word: its first word is
 * a jump over it, where a Zephyr image starts with an auipc.
 */
#ifndef SAPPHIRE_LOADER_H
#define SAPPHIRE_LOADER_H

#define LOADER_MAGIC      0x444c325aU /* "Z2LD" */
#define SLOT_MAGIC        0x544f4c53U /* "SLOT" */
#define SLOT_IMAGE_OFFSET 0x1000U     /* the header has a flash sector of its own */

#ifndef __ASSEMBLER__
#include <stdint.h>

struct slot_header {
	uint32_t magic;
	uint32_t header_size; /* sizeof(struct slot_header) */
	uint32_t sequence;    /* the highest valid one boots */
	uint32_t size;        /* image bytes */
	uint32_t crc32;       /* of the image, IEEE (zlib) */
	uint32_t load_address;
	uint32_t entry;
	uint32_t header_crc32; /* of the fields above */
};
#endif

#endif
