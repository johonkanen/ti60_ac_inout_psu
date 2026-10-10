/*
 * Firmware update over HTTP for boards started by the second-stage loader (loader/):
 * POST a zephyr.bin to the resource and the board writes it to the slot that is not
 * running and reboots into it. POSTing the loader's loader.bin replaces the loader
 * without a reboot. GET shows the slots.
 *
 *   HTTP_RESOURCE_DEFINE(update, my_service, "/update", &fw_update_resource);
 *   curl --data-binary @zephyr.bin http://<address>/update
 */
#ifndef FW_UPDATE_H
#define FW_UPDATE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/net/http/server.h>

extern struct http_resource_detail_dynamic fw_update_resource;

struct fw_update_slot {
	bool valid;
	bool intact;  /* its image checks out in flash */
	bool running; /* booted from it: the loader picked it and the image region holds it */
	uint32_t sequence;
	uint32_t size;
	uint32_t crc32;
};

/* the two slots; returns the running one, or -1 when not started from a slot */
int fw_update_slots(struct fw_update_slot slots[2]);

/* the steps behind the resource, usable on their own */
void fw_update_begin(void);
int fw_update_write(const uint8_t *data, size_t len);
int fw_update_finish(void); /* checks the upload and makes it the image to boot */
bool fw_update_is_loader(void);
const char *fw_update_error(void);
uint32_t fw_update_size(void);
uint32_t fw_update_crc(void);

#endif
