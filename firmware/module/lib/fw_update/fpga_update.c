/*
 * FPGA update over the network, see fpga_update.h.
 */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/sys_io.h>
#include <fpga_update.h>

LOG_MODULE_REGISTER(fpga_update, LOG_LEVEL_INF);

#define APPLICATION_IMAGE PARTITION_ID(fpga_application_partition)
#define STATE_AREA  PARTITION_ID(fpga_state_partition)
#define REGISTERS   DT_REG_ADDR(DT_INST(0, hvhdl_remote_update))

/* remote_update.vhd */
#define REG_ROLE        0
#define REG_STATUS      1
#define REG_RECONFIGURE 2
#define ROLE_MAGIC      0x52550000u     /* "RU" */
#define ROLE_GOLDEN     0
#define ROLE_APPLICATION 1
#define RECONFIGURE_TO_APPLICATION 0x52434601u  /* "RCF", image 1 */

/* the upload's header, make_images.py */
#define HEADER_SIZE     64
#define DEVICE          "Ti60F225 tsemac_soc"

#define SECTOR          4096
#define BLOCK           (64 * 1024)

#define STATE_MAGIC     0x53475046u     /* "FPGS" */
#define STATE_VERSION   1
#define CLEARED         0u              /* a flag word programmed from all ones */

/* the state sector, its words written once each after an erase */
struct state_record {
	uint32_t magic;
	uint32_t version;
	uint32_t size;
	uint32_t crc32;
	uint32_t sequence;
	uint32_t header_crc32;  /* of the words before it */
	uint32_t reserved[2];
	uint32_t tried;         /* CLEARED once the golden image has switched to it */
	uint32_t confirmed;     /* CLEARED once it ran long enough as the application */
	uint32_t failed;        /* CLEARED when the FPGA fell back from it to the golden image */
};

static struct {
	const struct flash_area *area;
	uint8_t header[HEADER_SIZE];
	uint32_t header_len;
	uint32_t expected_size;
	uint32_t expected_crc;
	uint32_t size;
	uint32_t crc;
	uint32_t erased;
	uint32_t sequence;
} up;

static uint8_t buffer[SECTOR];

static uint32_t reg_read(int r)
{
	return sys_read32(REGISTERS + 4 * r);
}

static int role(void)
{
	uint32_t value = reg_read(REG_ROLE);

	return (value & 0xffff0000u) == ROLE_MAGIC ? (int)(value & 0xff) : -1;
}

static bool fell_back(void)
{
	return reg_read(REG_STATUS) & 1;
}

static bool read_state(struct state_record *s)
{
	const struct flash_area *area;
	bool valid;

	if (flash_area_open(STATE_AREA, &area)) {
		return false;
	}
	valid = flash_area_read(area, 0, s, sizeof(*s)) == 0 && s->magic == STATE_MAGIC &&
		s->version == STATE_VERSION &&
		s->header_crc32 == crc32_ieee((const uint8_t *)s, offsetof(struct state_record, header_crc32)) &&
		s->size > 0 && s->size <= PARTITION_SIZE(fpga_application_partition);
	flash_area_close(area);
	return valid;
}

/* clears one flag word of the state record */
static void clear_flag(size_t offset)
{
	const struct flash_area *area;
	uint32_t zero = CLEARED;

	if (flash_area_open(STATE_AREA, &area) == 0) {
		(void)flash_area_write(area, offset, &zero, sizeof(zero));
		flash_area_close(area);
	}
}

static int crc_of(const struct flash_area *area, uint32_t size, uint32_t *crc)
{
	*crc = 0;
	for (uint32_t offset = 0; offset < size; offset += SECTOR) {
		uint32_t n = MIN(SECTOR, size - offset);
		int ret = flash_area_read(area, offset, buffer, n);

		if (ret) {
			return ret;
		}
		*crc = crc32_ieee_update(*crc, buffer, n);
	}
	return 0;
}

/* ------------------------------------------------------------------ the upload */

void fpga_update_begin(void)
{
	struct state_record s;

	if (up.area) {
		flash_area_close(up.area);
	}
	memset(&up, 0, sizeof(up));
	up.sequence = (read_state(&s) ? s.sequence : 0) + 1;
	if (flash_area_open(APPLICATION_IMAGE, &up.area)) {
		up.area = NULL;
	}
}

static int check_header(const char **error)
{
	const uint8_t *h = up.header;

	if (sys_get_le32(h + 8) != HEADER_SIZE) {
		*error = "unknown FPGA update header";
		return -EINVAL;
	}
	if (sys_get_le32(h + 12) != ROLE_APPLICATION) {
		*error = "not an application image: the golden image is written only over JTAG";
		return -EINVAL;
	}
	if (sys_get_le32(h + 16) != up.area->fa_off) {
		*error = "built for another address than this board's application image";
		return -EINVAL;
	}
	if (strncmp((const char *)h + 28, DEVICE, sizeof(DEVICE)) != 0) {
		*error = "an image for another device or design";
		return -EINVAL;
	}
	up.expected_size = sys_get_le32(h + 20);
	up.expected_crc = sys_get_le32(h + 24);
	if (up.expected_size == 0 || up.expected_size > up.area->fa_size) {
		*error = "the FPGA image does not fit its partition";
		return -EFBIG;
	}
	return 0;
}

int fpga_update_write(const uint8_t *data, size_t len, const char **error)
{
	const struct flash_area *state;
	int ret;

	if (!up.area) {
		*error = "no partition for the FPGA application image";
		return -ENODEV;
	}
	if (up.header_len < HEADER_SIZE) {
		size_t n = MIN(len, HEADER_SIZE - up.header_len);

		memcpy(up.header + up.header_len, data, n);
		up.header_len += n;
		data += n;
		len -= n;
		if (up.header_len < HEADER_SIZE) {
			return 0;
		}
		ret = check_header(error);
		if (ret) {
			return ret;
		}
		/* no record of an application image from here until the new one is in */
		if (flash_area_open(STATE_AREA, &state)) {
			*error = "no partition for the FPGA state";
			return -ENODEV;
		}
		ret = flash_area_erase(state, 0, state->fa_size);
		flash_area_close(state);
		if (ret) {
			*error = "erasing the FPGA state failed";
			return ret;
		}
	}
	if (len == 0) {
		return 0;
	}
	if (up.size + len > up.expected_size) {
		*error = "more FPGA image than its header says";
		return -EFBIG;
	}
	while (up.erased < up.size + len) {
		uint32_t n = MIN(BLOCK - up.erased % BLOCK, up.area->fa_size - up.erased);

		ret = flash_area_erase(up.area, up.erased, n);
		if (ret) {
			*error = "erasing the FPGA application image failed";
			return ret;
		}
		up.erased += n;
	}
	ret = flash_area_write(up.area, up.size, data, len);
	if (ret) {
		*error = "writing the FPGA application image failed";
		return ret;
	}
	up.crc = crc32_ieee_update(up.crc, data, len);
	up.size += len;
	return 0;
}

int fpga_update_finish(const char **error)
{
	const struct flash_area *state;
	struct state_record s, back;
	uint32_t crc;
	int ret;

	if (up.size != up.expected_size || up.crc != up.expected_crc) {
		*error = up.size != up.expected_size ? "the FPGA image is not complete"
						      : "the FPGA image does not match its CRC-32";
		return -EINVAL;
	}
	ret = crc_of(up.area, up.size, &crc);
	if (ret || crc != up.crc) {
		*error = "the FPGA image does not read back";
		return ret ? ret : -EIO;
	}

	memset(&s, 0xff, sizeof(s));
	s.magic = STATE_MAGIC;
	s.version = STATE_VERSION;
	s.size = up.size;
	s.crc32 = up.crc;
	s.sequence = up.sequence;
	s.header_crc32 = crc32_ieee((const uint8_t *)&s, offsetof(struct state_record, header_crc32));
	if (flash_area_open(STATE_AREA, &state)) {
		*error = "no partition for the FPGA state";
		return -ENODEV;
	}
	ret = flash_area_write(state, 0, &s, sizeof(s));
	if (!ret) {
		ret = flash_area_read(state, 0, &back, sizeof(back));
	}
	flash_area_close(state);
	if (ret || memcmp(&s, &back, sizeof(s)) != 0) {
		*error = "writing the FPGA state failed";
		return ret ? ret : -EIO;
	}
	LOG_INF("FPGA application image %u: %u bytes, crc32 %08x", s.sequence, s.size, s.crc32);
	return 0;
}

bool fpga_update_fell_back(void)
{
	return role() >= 0 && fell_back();
}

uint32_t fpga_update_size(void)
{
	return up.size;
}

uint32_t fpga_update_crc(void)
{
	return up.crc;
}

/* ------------------------------------------------------------------ switching */

void fpga_update_switch(void)
{
	struct state_record s;

	if (read_state(&s) && s.tried != CLEARED) {
		clear_flag(offsetof(struct state_record, tried));
	}
	LOG_INF("reloading the FPGA from the application image");
	LOG_PANIC();
	k_busy_wait(1000);
	sys_write32(RECONFIGURE_TO_APPLICATION, REGISTERS + 4 * REG_RECONFIGURE);
	/* the FPGA reloads within microseconds and nothing after this runs, unless it does not */
	k_busy_wait(200000);
	LOG_ERR("the FPGA did not reload: still the %s image (loaded over JTAG?)",
		role() == ROLE_GOLDEN ? "golden" : "application");
}

static void confirm(struct k_work *work)
{
	struct state_record s;

	ARG_UNUSED(work);
	if (role() == ROLE_APPLICATION && read_state(&s) && s.confirmed != CLEARED) {
		clear_flag(offsetof(struct state_record, confirmed));
		LOG_INF("FPGA application image %u confirmed", s.sequence);
	}
}

static K_WORK_DELAYABLE_DEFINE(confirm_work, confirm);

/* the golden image's decision at boot, see fpga_update.h */
static int fpga_update_boot(void)
{
	struct state_record s;
	int r = role();
	bool valid;

	if (r < 0) {
		return 0;       /* a design without remote_update */
	}
	if (r == ROLE_APPLICATION) {
		k_work_schedule(&confirm_work, K_SECONDS(CONFIG_FW_UPDATE_FPGA_CONFIRM_S));
		return 0;
	}
	valid = read_state(&s);
	if (fell_back()) {
		/*
		 * The FPGA loads no other image until it is powered off: an image installed
		 * since then has not been tried, and will be after the power cycle.
		 */
		LOG_WRN("the FPGA application image failed to load: running the golden image");
		if (valid && s.tried == CLEARED && s.failed != CLEARED) {
			clear_flag(offsetof(struct state_record, failed));
		}
		return 0;
	}
	if (!valid) {
		LOG_INF("golden FPGA image, no application image");
		return 0;
	}
	if (s.confirmed == CLEARED || s.tried != CLEARED) {
		LOG_INF("golden FPGA image: switching to application image %u%s", s.sequence,
			s.confirmed == CLEARED ? "" : ", a first try");
		fpga_update_switch();
		return 0;
	}
	LOG_WRN("golden FPGA image: application image %u was tried and never confirmed", s.sequence);
	return 0;
}

SYS_INIT(fpga_update_boot, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* ------------------------------------------------------------------ status */

int fpga_update_status(char *buf, size_t size)
{
	struct state_record s;
	int r = role();
	int len;

	len = snprintf(buf, size, "{\"role\":\"%s\",\"fell_back\":%s",
		       r == ROLE_GOLDEN ? "golden" : r == ROLE_APPLICATION ? "application" : "none",
		       r >= 0 && fell_back() ? "true" : "false");
	if (read_state(&s)) {
		len += snprintf(buf + len, size - len,
				",\"image\":{\"sequence\":%u,\"size\":%u,\"crc32\":\"%08x\","
				"\"tried\":%s,\"confirmed\":%s,\"failed\":%s}",
				s.sequence, s.size, s.crc32, s.tried == CLEARED ? "true" : "false",
				s.confirmed == CLEARED ? "true" : "false",
				s.failed == CLEARED ? "true" : "false");
	}
	len += snprintf(buf + len, size - len, "}");
	return MIN(len, (int)size - 1);
}
