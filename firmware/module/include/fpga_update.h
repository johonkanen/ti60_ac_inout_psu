/*
 * FPGA update over the network, for designs with ti60_ac_psu's remote_update block (the
 * Titanium's internal reconfiguration): no JTAG, which cannot be used safely while the power
 * supply runs.
 *
 * The flash holds a golden image at 0, which the FPGA loads at power-up and which is never
 * written here, an application image (fpga-application partition) and a state sector
 * (fpga-state) recording the application image: its size, CRC-32 and sequence, and words
 * cleared, without an erase, as it is tried and confirmed. Both images are the same design,
 * built as golden or application; the role register tells the software which one runs.
 *
 * At boot the golden image switches to the application image if that one has been
 * confirmed, or has never been tried (then it is marked tried first). It stays golden if the
 * application image was tried but never confirmed (it hung: the board was power cycled) or
 * if the FPGA fell back to golden after the application image failed its CRC. The
 * application image confirms itself once Zephyr has run for CONFIG_FW_UPDATE_FPGA_CONFIRM_S.
 *
 * After a fallback the FPGA loads no other image until it is powered off (its error flag
 * stays set, even through a reload of the golden image): an image installed then waits,
 * untried, for the next power cycle.
 *
 * An upload (fw_update, POST /update) is images/application_update.bin of the design's
 * make_images.py: a header ("TI60FPGA", role, address, size, CRC-32, device) and the image as
 * the combined flash image holds it. The state sector is cleared before the image is
 * erased, so an interrupted upload leaves the golden image running; after the image reads
 * back the new state is written and the FPGA reloads from it.
 */
#ifndef FPGA_UPDATE_H
#define FPGA_UPDATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FPGA_UPDATE_MAGIC "TI60FPGA"

/* the upload, from fw_update: the whole file in pieces, then finish */
void fpga_update_begin(void);
int fpga_update_write(const uint8_t *data, size_t len, const char **error);
int fpga_update_finish(const char **error);
uint32_t fpga_update_size(void);
uint32_t fpga_update_crc(void);

/* the FPGA fell back to the golden image: it takes no other image before a power cycle */
bool fpga_update_fell_back(void);

/*
 * Reload the FPGA from the application image now: this design and the CPU stop. Returns, with
 * an error logged, only if the FPGA did not reload.
 */
void fpga_update_switch(void);

/* the role, fallback and application image as JSON, for GET /update; returns its length */
int fpga_update_status(char *buf, size_t size);

#endif
