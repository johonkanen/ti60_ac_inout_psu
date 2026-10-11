/*
 * The stream: the FPGA's continuous capture to a browser over a WebSocket on port
 * SCOPE_STREAM_PORT (ws_stream), as fast as the link goes.
 *
 * A client connecting starts a continuous capture with the settings the browser has made
 * (sample rate, the eight sources) into the stream ring; segment i of the stream is records
 * 91*i..91*i+90 of the capture, ring segment i mod 23040 (32 MB, the HyperRAM's second
 * die), and each WebSocket message is 720 segments: 65520 records of eight int16 channels,
 * sample-major, 1 MB. A segment goes out once the FPGA's DMA has taken MARGIN_RECORDS
 * records past it: the DMA counts a record when it takes it into its 2 KB buffer (128
 * records) and writes it to the memory later, whole 16 record bursts as the memory lets it.
 * Sent sooner, a burst still holds the previous lap.
 *
 * The ring is that large for slack: the client may fall up to a ring behind the capture
 * (537 ms at 3.9 MS/s, two seconds at 1 MS/s) and catch up, a slow acknowledgement or a
 * pause of the PC's being no loss.
 *
 * The sample rate decides whether the link keeps up, 16 bytes a sample: about 62 MB/s
 * (3.9 MS/s) here, the HyperRAM carrying the FPGA's writes and the Ethernet DMA's reads.
 * When the capture gets a ring ahead of a segment that is still to be sent, the data is
 * gone: the connection is reset rather than sent what the ring holds now, and the overrun
 * counted (scope_stream_lost(), in the scope's status).
 */
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <tcp_stream.h>
#include <ws_stream.h>
#include "scope.h"

LOG_MODULE_REGISTER(stream, LOG_LEVEL_INF);

#define SEGMENT_RECORDS SCOPE_STREAM_SEGMENT_RECORDS
#define RING_RECORDS    (SCOPE_STREAM_RING_SEGMENTS * SEGMENT_RECORDS)
#define RECORD_BYTES    (SCOPE_CHANNELS * sizeof(int16_t))
#define MARGIN_RECORDS  256
/*
 * How far behind the capture a segment may be when it goes into a slot: the Ethernet DMA
 * reads it up to two batches later (1.5 ms, 7500 records at 5 MS/s), and the capture's DMA
 * may be ahead of its count.
 */
#define SAFE_RECORDS    (RING_RECORDS - 16384)

static const uint8_t *ring;
static uint64_t written;        /* records the DMA has taken, extended to 64 bits */
static uint32_t written_low;
static atomic_t lost;
static atomic_t active;
static const char *last_end = "none";      /* why the last stream ended (scope_stream_end()) */

/*
 * A capture download (scope_download_offer()): a triggered capture longer than /ws carries,
 * sent from the capture ring on this port as one message (ws://<board>:5004/capture?id=<n>),
 * its segments of 91 records from the capture's start on, the last one filled up with the
 * ring's next records (the page takes the capture's length). The one segment that may cross
 * the ring's end is copied out first. The scope arms again once it is acknowledged, or when
 * it gives up waiting (the connection is then closed at the next segment).
 */
enum mode { MODE_NONE, MODE_STREAM, MODE_DOWNLOAD };
static enum mode mode;

static struct {
	atomic_t valid;
	uint32_t id;
	uint32_t start;
	uint32_t length;
} offer;
static K_SEM_DEFINE(downloaded, 0, 1);
static uint32_t download_segments;
static bool download_done;
static int64_t bounce_index = -1;
static uint8_t bounce[SEGMENT_RECORDS * RECORD_BYTES] __aligned(64);

void scope_download_offer(uint32_t id, uint32_t start, uint32_t length)
{
	k_sem_reset(&downloaded);
	offer.id = id;
	offer.start = start;
	offer.length = length;
	atomic_set(&offer.valid, 1);
}

bool scope_download_wait(int32_t ms)
{
	return k_sem_take(&downloaded, K_MSEC(ms)) == 0;
}

void scope_download_withdraw(void)
{
	atomic_set(&offer.valid, 0);
}

static void report(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(report_work, report);

static void update_written(void)
{
	uint32_t now = capture_stream_written();
	uint32_t step = now - written_low;

	if (step > 10000000) {
		static int reported;

		if (reported++ < 10) {
			LOG_WRN("register 22: %u after %u", now, written_low);
		}
	}
	written += step;
	written_low = now;
}

static void connected(const char *path, void *user_data)
{
	struct scope_config config;

	mode = MODE_NONE;
	if (!capture_stream_ram_usable()) {
		LOG_ERR("no stream on this FPGA image: its HyperRAM controller corrupts the ring's die");
		tcp_stream_close();
		return;
	}
	if (strncmp(path, "/capture", 8) == 0) {
		const char *id = strstr(path, "id=");

		if (!atomic_get(&offer.valid) || id == NULL || strtoul(id + 3, NULL, 10) != offer.id) {
			LOG_WRN("no capture to download for %s", path);
			tcp_stream_close();
			return;
		}
		mode = MODE_DOWNLOAD;
		download_segments = DIV_ROUND_UP(offer.length, SEGMENT_RECORDS);
		download_done = false;
		bounce_index = -1;
		LOG_INF("capture %u: %u records to download", offer.id, offer.length);
		return;
	}
	/* the stream: it takes the memory a capture to download is in */
	if (atomic_get(&offer.valid)) {
		atomic_set(&offer.valid, 0);
		k_sem_give(&downloaded);
	}
	mode = MODE_STREAM;
	atomic_set(&active, 1);
	scope_get_config(&config);
	written = 0;
	written_low = 0;
	atomic_set(&lost, 0);
	ring = (const uint8_t *)capture_stream_start(&config);
	k_work_reschedule(&report_work, K_SECONDS(1));
	LOG_INF("streaming at %u samples/s, %u bytes/s", (uint32_t)(SCOPE_CLOCK_HZ / (config.divider + 1)),
		(uint32_t)(SCOPE_CLOCK_HZ / (config.divider + 1) * RECORD_BYTES));
}

static void closed(void *user_data)
{
	if (mode == MODE_DOWNLOAD) {
		mode = MODE_NONE;
		if (!download_done) {
			LOG_WRN("capture %u: the download ended unfinished", offer.id);
		}
		return;
	}
	if (mode != MODE_STREAM) {
		return;
	}
	mode = MODE_NONE;
	capture_stream_stop();
	last_end = atomic_get(&lost) ? "overrun" : tcp_stream_end_name(tcp_stream_last_end());
	atomic_set(&active, 0);
	LOG_INF("stream closed%s", atomic_get(&lost) ? " by an overrun" : "");
}

/* a segment of the capture to download */
static const void *download_segment(uint64_t index)
{
	const uint32_t records = capture_ring_records();
	const uint8_t *capture = (const uint8_t *)capture_ring();
	uint32_t first;

	if (!atomic_get(&offer.valid)) {
		/* the scope has gone on: the ring no longer holds it */
		tcp_stream_close();
		return NULL;
	}
	if (index >= download_segments) {
		/* all sent: done once acknowledged */
		if (!download_done && tcp_stream_acked() >= download_segments) {
			download_done = true;
			LOG_INF("capture %u downloaded", offer.id);
			k_sem_give(&downloaded);
			tcp_stream_close();
		}
		return NULL;
	}
	first = (offer.start + (uint32_t)index * SEGMENT_RECORDS) & (records - 1);
	if (first + SEGMENT_RECORDS <= records) {
		return capture + first * RECORD_BYTES;
	}
	/* across the ring's end: its two parts, once, into the bounce buffer */
	if (bounce_index != (int64_t)index) {
		uint32_t part = records - first;

		memcpy(bounce, capture + first * RECORD_BYTES, part * RECORD_BYTES);
		memcpy(bounce + part * RECORD_BYTES, capture, (SEGMENT_RECORDS - part) * RECORD_BYTES);
		bounce_index = index;
	}
	return bounce;
}

static uint32_t message_segments(void *user_data)
{
	return mode == MODE_DOWNLOAD ? download_segments : SCOPE_STREAM_SEGMENTS;
}

static const void *segment(uint64_t index, void *user_data)
{
	if (mode == MODE_DOWNLOAD) {
		return download_segment(index);
	}
	if (mode != MODE_STREAM) {
		return NULL;
	}
	uint64_t first = index * SEGMENT_RECORDS;
	uint64_t end = first + SEGMENT_RECORDS;

	/* the count now, not the last one: a client behind is told by it */
	update_written();
	if (written < end + MARGIN_RECORDS) {
		return NULL;
	}
	if (written - first > SAFE_RECORDS) {
		/* overwritten: a first send that came too late, or a resend */
		LOG_WRN("overrun at segment %llu: the capture is %llu records ahead", index,
			written - first);
		atomic_inc(&lost);
		tcp_stream_close();
		return NULL;
	}
	return ring + (uint32_t)index % SCOPE_STREAM_RING_SEGMENTS * (SEGMENT_RECORDS * RECORD_BYTES);
}

static const struct ws_stream_source source = {
	.payload_len = SEGMENT_RECORDS * RECORD_BYTES,
	.segments_per_message = SCOPE_STREAM_SEGMENTS,
	.message_segments = message_segments,
	.connected = connected,
	.closed = closed,
	.segment = segment,
};

/* once a second while streaming, on the console */
static void report(struct k_work *work)
{
	static struct tcp_stream_stats last;
	struct tcp_stream_stats stats;

	if (!scope_stream_active()) {
		memset(&last, 0, sizeof(last));
		return;
	}
	tcp_stream_get_stats(&stats);
	if (stats.acked < last.acked) {
		memset(&last, 0, sizeof(last));
	}
	LOG_INF("%u kB/s, %u retransmits, the capture %u records ahead of the client",
		(uint32_t)((stats.acked - last.acked) / 1000), stats.retransmits,
		(uint32_t)(written - stats.acked / (SEGMENT_RECORDS * RECORD_BYTES + 4) *
					    SEGMENT_RECORDS));
	last = stats;
	k_work_reschedule(&report_work, K_SECONDS(1));
}

bool scope_stream_active(void)
{
	return atomic_get(&active) != 0;
}

uint32_t scope_stream_lost(void)
{
	return atomic_get(&lost);
}

const char *scope_stream_end(void)
{
	return last_end;
}

int scope_stream_init(void)
{
	int ret = ws_stream_start(net_if_get_default(), SCOPE_STREAM_PORT, &source, NULL);

	if (ret < 0) {
		LOG_ERR("stream: %d", ret);
	}
	return ret;
}
