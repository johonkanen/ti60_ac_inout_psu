/*
 * Firmware update for the second-stage loader's slots (sapphire_loader.h).
 *
 * An image streams into the slot that is not running, erased ahead of the data, is read
 * back and only then gets its header with the next sequence number, which makes the
 * loader boot it. Until that header is written the old image stays the one that boots,
 * so a failed upload or a power cut at any point leaves the board bootable.
 *
 * The running slot is the one the loader booted, if its image is what the image region
 * holds now: with the XIP layout Zephyr never writes there. A JTAG-loaded image is not
 * running from a slot; a slot that cannot boot, or else the older one, is written then.
 *
 * The loader itself goes into the same slot first, as a staging area, and is then
 * copied over the loader partition. That copy, a few tens of milliseconds, is the only
 * window in which a power cut leaves the board unbootable.
 *
 * With CONFIG_FW_UPDATE_FPGA an FPGA application image (fpga_update.h) goes the same way,
 * to its own partition, and the FPGA reloads from it instead of the reboot.
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/reboot.h>
#include <fw_update.h>
#include <sapphire_loader.h>
#ifdef CONFIG_FW_UPDATE_FPGA
#include <fpga_update.h>
#endif

LOG_MODULE_REGISTER(fw_update, LOG_LEVEL_INF);

#define SLOTS        2
#define LOADER       PARTITION_ID(loader_partition)
#define IMAGE_BASE   CONFIG_FLASH_BASE_ADDRESS
#define IMAGE_REGION (CONFIG_FLASH_SIZE * 1024)
#define SECTOR       4096
#define BLOCK        (64 * 1024)

static const uint8_t slot_ids[SLOTS] = {
	PARTITION_ID(slot0_partition),
	PARTITION_ID(slot1_partition),
};

enum kind {
	KIND_UNKNOWN,
	KIND_IMAGE,
	KIND_LOADER,
	KIND_FPGA,
};

static struct {
	const struct flash_area *slot;
	int target;
	uint32_t sequence;
	enum kind kind;
	uint32_t size;
	uint32_t crc;
	uint32_t erased;
	const char *error;
} update;

static uint8_t buffer[SECTOR];

static int fail(const char *error, int ret)
{
	if (!update.error) {
		update.error = error;
		LOG_ERR("%s (%d)", error, ret);
	}
	return ret ? ret : -EINVAL;
}

static bool header_valid(const struct slot_header *h, size_t slot_size)
{
	return h->magic == SLOT_MAGIC && h->header_size == sizeof(*h) &&
	       crc32_ieee((const uint8_t *)h, offsetof(struct slot_header, header_crc32)) ==
		       h->header_crc32 &&
	       h->size > 0 && h->size <= slot_size - SLOT_IMAGE_OFFSET;
}

static int crc_of(const struct flash_area *area, off_t start, uint32_t size, uint32_t *crc);

/*
 * The slot the loader booted is the first one, newest first, whose image checks out in
 * flash. It is running if the image region holds that image; else Zephyr was loaded
 * another way, through JTAG.
 */
int fw_update_slots(struct fw_update_slot slots[2])
{
	struct slot_header headers[SLOTS];
	int booted = -1;

	for (int s = 0; s < SLOTS; s++) {
		const struct flash_area *area;
		uint32_t crc;

		memset(&slots[s], 0, sizeof(slots[s]));
		if (flash_area_open(slot_ids[s], &area)) {
			return -ENODEV;
		}
		if (flash_area_read(area, 0, &headers[s], sizeof(headers[s])) == 0 &&
		    header_valid(&headers[s], area->fa_size)) {
			slots[s].valid = true;
			slots[s].sequence = headers[s].sequence;
			slots[s].size = headers[s].size;
			slots[s].crc32 = headers[s].crc32;
			slots[s].intact = crc_of(area, SLOT_IMAGE_OFFSET, headers[s].size, &crc) == 0 &&
					  crc == headers[s].crc32;
		}
		flash_area_close(area);
	}
	for (int s = 0; s < SLOTS; s++) {
		if (slots[s].intact && (booted < 0 || slots[s].sequence > slots[booted].sequence)) {
			booted = s;
		}
	}
	if (booted < 0 || headers[booted].load_address != IMAGE_BASE ||
	    headers[booted].size > IMAGE_REGION ||
	    crc32_ieee((const uint8_t *)IMAGE_BASE, headers[booted].size) != headers[booted].crc32) {
		return -1;
	}
	slots[booted].running = true;
	return booted;
}

void fw_update_begin(void)
{
	struct fw_update_slot slots[SLOTS];
	int running;

	if (update.slot) {
		flash_area_close(update.slot);
	}
	memset(&update, 0, sizeof(update));

	running = fw_update_slots(slots);
	if (running >= 0) {
		update.target = !running;
	} else {
		/* not booted from a slot: replace one that cannot boot, or the older one */
		update.target = !slots[0].intact                        ? 0
				: !slots[1].intact                      ? 1
				: slots[0].sequence <= slots[1].sequence ? 0
									 : 1;
	}
	update.sequence = MAX(slots[0].valid ? slots[0].sequence : 0,
			      slots[1].valid ? slots[1].sequence : 0) +
			  1;
	if (flash_area_open(slot_ids[update.target], &update.slot)) {
		update.slot = NULL;
		fail("no slot partition", -ENODEV);
	}
}

/* the file type, from its first bytes, before anything is written */
static int check_start(const uint8_t *data, size_t len)
{
	if (len < 8) {
		return fail("the first part of the upload is too short to tell what it is",
			    -EINVAL);
	}
	if (memcmp(data, "\x7f" "ELF", 4) == 0) {
		return fail("an ELF file, send zephyr.bin", -EINVAL);
	}
	if (data[0] == ':') {
		return fail("an Intel hex file, send zephyr.bin", -EINVAL);
	}
	if (memcmp(data, "TI60FPGA", 8) == 0) {
		if (!IS_ENABLED(CONFIG_FW_UPDATE_FPGA)) {
			return fail("an FPGA image, and this board has no FPGA update", -EINVAL);
		}
		update.kind = KIND_FPGA;
		return 0;
	}
	if (memcmp(data, "Version: ", 9) == 0) {
		return fail("a bare bitstream: send images/application_update.bin", -EINVAL);
	}
	/* the loader: a jal over the magic word */
	if ((data[0] & 0x7f) == 0x6f && sys_get_le32(data + 4) == LOADER_MAGIC) {
		update.kind = KIND_LOADER;
		return 0;
	}
	/* Zephyr's RISC-V reset vector starts with the auipc of a call to __initialize */
	if ((data[0] & 0x7f) == 0x17) {
		update.kind = KIND_IMAGE;
		return 0;
	}
	return fail("not a Zephyr RISC-V image or the loader", -EINVAL);
}

static uint32_t max_size(void)
{
	const struct flash_area *loader;
	uint32_t max = 0;

	if (update.kind == KIND_LOADER) {
		if (flash_area_open(LOADER, &loader) == 0) {
			max = loader->fa_size;
			flash_area_close(loader);
		}
		return max;
	}
	return MIN(update.slot->fa_size - SLOT_IMAGE_OFFSET, IMAGE_REGION);
}

int fw_update_write(const uint8_t *data, size_t len)
{
	int ret;

	if (update.error || len == 0) {
		return update.error ? -EINVAL : 0;
	}
	if (update.size == 0 && check_start(data, len)) {
		return -EINVAL;
	}
#ifdef CONFIG_FW_UPDATE_FPGA
	if (update.kind == KIND_FPGA) {
		const char *error = NULL;

		if (update.size == 0) {
			fpga_update_begin();
		}
		ret = fpga_update_write(data, len, &error);
		if (ret) {
			return fail(error, ret);
		}
		update.size += len;
		return 0;
	}
#endif
	if (update.size + len > max_size()) {
		return fail(update.kind == KIND_LOADER ? "too large for the loader partition"
						       : "image too large for a slot",
			    -EFBIG);
	}

	/* from the header sector on: the slot stops being bootable with the first erase */
	while (update.erased < SLOT_IMAGE_OFFSET + update.size + len) {
		uint32_t n = MIN(BLOCK, update.slot->fa_size - update.erased);

		ret = flash_area_erase(update.slot, update.erased, n);
		if (ret) {
			return fail("erasing the slot failed", ret);
		}
		update.erased += n;
	}
	ret = flash_area_write(update.slot, SLOT_IMAGE_OFFSET + update.size, data, len);
	if (ret) {
		return fail("writing the slot failed", ret);
	}
	update.crc = crc32_ieee_update(update.crc, data, len);
	update.size += len;
	return 0;
}

static int crc_of(const struct flash_area *area, off_t start, uint32_t size, uint32_t *crc)
{
	*crc = 0;
	for (uint32_t offset = 0; offset < size; offset += SECTOR) {
		uint32_t n = MIN(SECTOR, size - offset);
		int ret = flash_area_read(area, start + offset, buffer, n);

		if (ret) {
			return ret;
		}
		*crc = crc32_ieee_update(*crc, buffer, n);
	}
	return 0;
}

static int write_header(void)
{
	struct slot_header h = {
		.magic = SLOT_MAGIC,
		.header_size = sizeof(h),
		.sequence = update.sequence,
		.size = update.size,
		.crc32 = update.crc,
		.load_address = IMAGE_BASE,
		.entry = IMAGE_BASE,
	};
	struct slot_header back;
	int ret;

	h.header_crc32 = crc32_ieee((const uint8_t *)&h, offsetof(struct slot_header, header_crc32));
	ret = flash_area_write(update.slot, 0, &h, sizeof(h));
	if (!ret) {
		ret = flash_area_read(update.slot, 0, &back, sizeof(back));
	}
	if (ret || memcmp(&h, &back, sizeof(h)) != 0) {
		return fail("writing the slot header failed", ret ? ret : -EIO);
	}
	return 0;
}

static int install_loader(void)
{
	const struct flash_area *loader;
	uint32_t crc;
	int ret;

	ret = flash_area_read(update.slot, SLOT_IMAGE_OFFSET + 8, buffer, 4);
	if (ret || sys_get_le32(buffer) != update.size) {
		return fail("the loader's own size does not match the upload", ret ? ret : -EINVAL);
	}
	if (flash_area_open(LOADER, &loader)) {
		return fail("no loader partition", -ENODEV);
	}
	ret = flash_area_erase(loader, 0, ROUND_UP(update.size, SECTOR));
	for (uint32_t offset = 0; !ret && offset < update.size; offset += SECTOR) {
		uint32_t n = MIN(SECTOR, update.size - offset);

		ret = flash_area_read(update.slot, SLOT_IMAGE_OFFSET + offset, buffer, n);
		if (!ret) {
			ret = flash_area_write(loader, offset, buffer, n);
		}
	}
	if (ret) {
		flash_area_close(loader);
		return fail("writing the loader partition failed", ret);
	}
	ret = crc_of(loader, 0, update.size, &crc);
	flash_area_close(loader);
	if (ret || crc != update.crc) {
		return fail("the installed loader does not read back", ret ? ret : -EIO);
	}
	return 0;
}

int fw_update_finish(void)
{
	int64_t start = k_uptime_get();
	uint32_t crc;
	int ret;

	if (update.error) {
		return -EINVAL;
	}
	if (update.size < 64) {
		return fail("nothing to install received", -EINVAL);
	}
#ifdef CONFIG_FW_UPDATE_FPGA
	if (update.kind == KIND_FPGA) {
		const char *error = NULL;

		ret = fpga_update_finish(&error);
		if (ret) {
			return fail(error, ret);
		}
		update.size = fpga_update_size();
		update.crc = fpga_update_crc();
		return 0;
	}
#endif
	ret = crc_of(update.slot, SLOT_IMAGE_OFFSET, update.size, &crc);
	if (ret || crc != update.crc) {
		return fail("the upload does not read back from the slot", ret ? ret : -EIO);
	}
	ret = update.kind == KIND_LOADER ? install_loader() : write_header();
	if (!ret) {
		if (update.kind == KIND_LOADER) {
			LOG_INF("installed the loader, %u bytes, crc32 %08x", update.size,
				update.crc);
		} else {
			LOG_INF("slot %d: sequence %u, %u bytes, crc32 %08x, in %u ms",
				update.target, update.sequence, update.size, update.crc,
				(unsigned int)(k_uptime_get() - start));
		}
	}
	return ret;
}

bool fw_update_is_loader(void)
{
	return update.kind == KIND_LOADER;
}

const char *fw_update_error(void)
{
	return update.error;
}

uint32_t fw_update_size(void)
{
	return update.size;
}

uint32_t fw_update_crc(void)
{
	return update.crc;
}

static void reboot(struct k_work *work)
{
	ARG_UNUSED(work);
#ifdef CONFIG_FW_UPDATE_FPGA
	if (update.kind == KIND_FPGA) {
		fpga_update_switch();
		return;         /* only if the FPGA did not reload */
	}
#endif
	LOG_INF("rebooting into the new image");
	LOG_PANIC();
	sys_reboot(SYS_REBOOT_WARM);
}

static K_WORK_DELAYABLE_DEFINE(reboot_work, reboot);

static char reply[640];

static int status_reply(void)
{
	struct fw_update_slot slots[SLOTS];
	int running = fw_update_slots(slots);
	int len = snprintf(reply, sizeof(reply), "{\"running\":%d,\"slots\":[", running);

	for (int s = 0; s < SLOTS; s++) {
		len += snprintf(reply + len, sizeof(reply) - len,
				"%s{\"valid\":%s,\"intact\":%s,\"sequence\":%u,\"size\":%u,"
				"\"crc32\":\"%08x\"}",
				s ? "," : "", slots[s].valid ? "true" : "false",
				slots[s].intact ? "true" : "false", slots[s].sequence,
				slots[s].size, slots[s].crc32);
	}
	len += snprintf(reply + len, sizeof(reply) - len, "]");
#ifdef CONFIG_FW_UPDATE_FPGA
	len += snprintf(reply + len, sizeof(reply) - len, ",\"fpga\":");
	len += fpga_update_status(reply + len, sizeof(reply) - len);
#endif
	len += snprintf(reply + len, sizeof(reply) - len, "}\n");
	return MIN(len, sizeof(reply) - 1);
}

static int update_cb(struct http_client_ctx *client, enum http_transaction_status status,
		     const struct http_request_ctx *request_ctx,
		     struct http_response_ctx *response_ctx, void *user_data)
{
	static bool receiving, reboot_after;
	bool ok;

	ARG_UNUSED(user_data);

	switch (status) {
	case HTTP_SERVER_TRANSACTION_ABORTED:
		if (receiving) {
			LOG_WRN("upload aborted after %u bytes", update.size);
		}
		receiving = false;
		return 0;
	case HTTP_SERVER_TRANSACTION_COMPLETE:
		if (reboot_after) {
			k_work_schedule(&reboot_work, K_MSEC(CONFIG_FW_UPDATE_REBOOT_DELAY_MS));
		}
		receiving = false;
		reboot_after = false;
		return 0;
	default:
		break;
	}

	if (client->method == HTTP_GET) {
		response_ctx->status = HTTP_200_OK;
		response_ctx->body = reply;
		response_ctx->body_len = status_reply();
		response_ctx->final_chunk = true;
		return 0;
	}

	if (!receiving) {
		fw_update_begin();
		LOG_INF("receiving into slot %d", update.target);
		receiving = true;
		reboot_after = false;
	}
	(void)fw_update_write(request_ctx->data, request_ctx->data_len);
	if (status != HTTP_SERVER_REQUEST_DATA_FINAL) {
		return 0;
	}

	ok = fw_update_finish() == 0;
	if (ok && update.kind == KIND_FPGA) {
#ifdef CONFIG_FW_UPDATE_FPGA
		/* after a fallback the FPGA takes the new image only after a power cycle */
		reboot_after = !fpga_update_fell_back();
#endif
		snprintf(reply, sizeof(reply),
			 "{\"ok\":true,\"kind\":\"fpga\",\"size\":%u,\"crc32\":\"%08x\","
			 "\"message\":\"%s\"}\n",
			 update.size, update.crc, reboot_after ? "reconfiguring" : "power cycle");
	} else if (ok && update.kind == KIND_LOADER) {
		snprintf(reply, sizeof(reply),
			 "{\"ok\":true,\"size\":%u,\"crc32\":\"%08x\",\"message\":\"loader "
			 "installed\"}\n",
			 update.size, update.crc);
	} else if (ok) {
		snprintf(reply, sizeof(reply),
			 "{\"ok\":true,\"slot\":%d,\"sequence\":%u,\"size\":%u,\"crc32\":\"%08x\","
			 "\"message\":\"rebooting\"}\n",
			 update.target, update.sequence, update.size, update.crc);
		reboot_after = true;
	} else {
		snprintf(reply, sizeof(reply), "{\"ok\":false,\"size\":%u,\"message\":\"%s\"}\n",
			 update.size, update.error ? update.error : "failed");
	}
	response_ctx->status = ok ? HTTP_200_OK : HTTP_400_BAD_REQUEST;
	response_ctx->body = reply;
	response_ctx->body_len = strlen(reply);
	response_ctx->final_chunk = true;
	return 0;
}

struct http_resource_detail_dynamic fw_update_resource = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET) | BIT(HTTP_POST),
		.content_type = "application/json",
	},
	.cb = update_cb,
};
