/*
 * SPI flash check: JEDEC ID, the start of image slot 0 (its header), read speed, then
 * erase, write and verify the last 64 KB of slot 1 with timings, which only an image of
 * nearly 2 MB would reach.
 * Erase and write through the flash shell ("flash", "flash_map").
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>

#define CHUNK           4096
#define WRITE_TEST_SIZE (64 * 1024)

static uint8_t buffer[CHUNK];

static uint32_t ms_since(uint32_t start)
{
	return k_cyc_to_ms_floor32(k_cycle_get_32() - start);
}

static void write_test(void)
{
	const struct flash_area *slot;
	off_t base;
	uint32_t start, erase_ms, write_ms = 0;
	int ret;

	ret = flash_area_open(PARTITION_ID(slot1_partition), &slot);
	if (ret) {
		printk("slot 1: %d\n", ret);
		return;
	}
	base = slot->fa_size - WRITE_TEST_SIZE;
	start = k_cycle_get_32();
	ret = flash_area_erase(slot, base, WRITE_TEST_SIZE);
	erase_ms = ms_since(start);
	for (off_t offset = 0; !ret && offset < WRITE_TEST_SIZE; offset += CHUNK) {
		for (int i = 0; i < CHUNK; i++) {
			buffer[i] = (uint8_t)((offset + i) * 7 + (offset + i) / 251);
		}
		start = k_cycle_get_32();
		ret = flash_area_write(slot, base + offset, buffer, CHUNK);
		write_ms += ms_since(start);
	}
	for (off_t offset = 0; !ret && offset < WRITE_TEST_SIZE; offset += CHUNK) {
		ret = flash_area_read(slot, base + offset, buffer, CHUNK);
		for (int i = 0; !ret && i < CHUNK; i++) {
			if (buffer[i] != (uint8_t)((offset + i) * 7 + (offset + i) / 251)) {
				printk("mismatch at 0x%lx\n", (long)(slot->fa_off + base + offset + i));
				ret = -EIO;
			}
		}
	}
	printk("end of slot 1, %u KB: erase %u ms, write %u ms, verify %s (%d)\n",
	       WRITE_TEST_SIZE / 1024, erase_ms, write_ms, ret ? "failed" : "ok", ret);
	flash_area_close(slot);
}

int main(void)
{
	const struct device *flash = DEVICE_DT_GET(DT_NODELABEL(flash0));
	const struct flash_area *image;
	uint8_t id[3];
	uint32_t start, cycles, crc;
	int ret;

	if (!device_is_ready(flash)) {
		printk("flash not ready\n");
		return 0;
	}
	ret = flash_read_jedec_id(flash, id);
	printk("jedec id %02x %02x %02x (%d)\n", id[0], id[1], id[2], ret);

	ret = flash_area_open(PARTITION_ID(slot0_partition), &image);
	if (ret) {
		printk("slot 0: %d\n", ret);
		return 0;
	}
	ret = flash_area_read(image, 0, buffer, 256);
	printk("slot 0 starts: %08x %08x (%d)\n", sys_get_le32(buffer), sys_get_le32(buffer + 4),
	       ret);

	cycles = 0;
	crc = 0;
	for (off_t offset = 0; offset < image->fa_size; offset += CHUNK) {
		start = k_cycle_get_32();
		ret = flash_area_read(image, offset, buffer, CHUNK);
		cycles += k_cycle_get_32() - start;
		if (ret) {
			printk("read at 0x%lx: %d\n", (long)offset, ret);
			break;
		}
		crc = crc32_ieee_update(crc, buffer, CHUNK);
	}
	printk("read %u KB in %u ms, %u KB/s, crc32 %08x\n", (unsigned)image->fa_size / 1024,
	       k_cyc_to_ms_floor32(cycles),
	       (unsigned)((uint64_t)image->fa_size * sys_clock_hw_cycles_per_sec() / 1024 / cycles), crc);
	flash_area_close(image);

	write_test();
	return 0;
}
