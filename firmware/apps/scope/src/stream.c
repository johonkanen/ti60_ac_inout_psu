/*
 * The stream: the FPGA's continuous capture to a browser over a WebSocket on port
 * SCOPE_STREAM_PORT (ws_stream), as fast as the link goes.
 *
 * A client connecting starts a continuous capture with the settings the browser has made
 * (sample rate, the eight sources) into the stream ring; segment i of the stream is records
 * 91*i..91*i+90 of the capture, ring segment i mod 5760 (8 MB), and each WebSocket message is
 * 720 segments: 65520 records of eight int16 channels, sample-major, 1 MB. A segment goes out
 * once the FPGA's DMA has taken MARGIN_RECORDS records past it: the DMA counts a record when
 * it takes it into its 2 KB buffer (128 records) and writes it to the memory later, whole
 * 16 record bursts as the memory lets it. Sent sooner, a burst still holds the previous lap.
 *
 * The ring is that large for slack: the client may fall up to a ring behind the capture
 * (134 ms at 3.9 MS/s, half a second at 1 MS/s) and catch up, a slow acknowledgement or a
 * pause of the PC's being no loss.
 *
 * The sample rate decides whether the link keeps up, 16 bytes a sample: about 62 MB/s
 * (3.9 MS/s) here, the HyperRAM carrying the FPGA's writes and the Ethernet DMA's reads.
 * When the capture gets a ring ahead of a segment that is still to be sent, the data is
 * gone: the connection is reset rather than sent what the ring holds now, and the overrun
 * counted (scope_stream_lost(), in the scope's status).
 */
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

static void connected(void *user_data)
{
	struct scope_config config;

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
	capture_stream_stop();
	atomic_set(&active, 0);
	LOG_INF("stream closed%s", atomic_get(&lost) ? " by an overrun" : "");
}

static const void *segment(uint64_t index, void *user_data)
{
	uint64_t first = index * SEGMENT_RECORDS;
	uint64_t end = first + SEGMENT_RECORDS;

	if (written < end + MARGIN_RECORDS) {
		update_written();
		if (written < end + MARGIN_RECORDS) {
			return NULL;
		}
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

int scope_stream_init(void)
{
	int ret = ws_stream_start(net_if_get_default(), SCOPE_STREAM_PORT, &source, NULL);

	if (ret < 0) {
		LOG_ERR("stream: %d", ret);
	}
	return ret;
}
