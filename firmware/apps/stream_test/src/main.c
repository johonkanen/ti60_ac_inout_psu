/*
 * tcp_stream from a buffer in memory, to measure the sender alone: port 5004 sends segments
 * of 1456 bytes cut from a 65520 byte buffer over and over, as a WebSocket of 720 segment
 * (1 MB) messages (ws_stream) or, without CONFIG_STREAM_TEST_WEBSOCKET, as raw TCP. Word w
 * of the data (32 bits, little endian) is w mod 16380, which stream_client.py checks. The
 * rate is printed every second while a client is connected.
 */
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <tcp_stream.h>
#include <ws_stream.h>
#include <eth_efinix_tsemac.h>

#define PORT        5004
#define PAYLOAD     1456
#define SEGMENTS    45
#define WORDS       (SEGMENTS * PAYLOAD / 4)

static uint32_t buffer[WORDS] __aligned(64);
static volatile bool connected;

static const void *ws_segment(uint64_t index, void *user_data)
{
	return (const uint8_t *)buffer + (uint32_t)index % SEGMENTS * PAYLOAD;
}

static const void *segment(uint64_t index, uint8_t *prefix, void *user_data)
{
	return (const uint8_t *)buffer + (uint32_t)index % SEGMENTS * PAYLOAD;
}

static void on_connected(void *user_data)
{
	if (!IS_ENABLED(CONFIG_STREAM_TEST_WEBSOCKET)) {
		tcp_stream_open();
	}
	connected = true;
}

static void on_closed(void *user_data)
{
	connected = false;
}

/*
 * The transmit DMA alone: 32 frames of 1510 bytes to a made-up address, sent as a header and
 * a payload each, then as one buffer each; the time until the DMA is idle again.
 */
static uint8_t frame_buf[32][1536] __aligned(64);

static void dma_rate(const struct device *dev)
{
	struct eth_efinix_tx_frame frames[32];

	for (int f = 0; f < 32; f++) {
		static const uint8_t header[14] = {0x02, 0, 0, 0, 0, 1, 0x02, 0, 0, 0, 0, 2, 0x88, 0xb5};

		memcpy(frame_buf[f], header, sizeof(header));
	}
	for (int split = 1; split >= 0; split--) {
		for (int f = 0; f < 32; f++) {
			frames[f].header = frame_buf[f];
			frames[f].header_len = split ? 54 : 1510;
			frames[f].payload = split ? &frame_buf[f][64] : NULL;
			frames[f].payload_len = split ? 1456 : 0;
		}
		uint32_t best = UINT32_MAX;

		for (int run = 0; run < 20; run++) {
			while (!eth_efinix_tx_idle(dev)) {
			}
			uint32_t t0 = k_cycle_get_32();

			if (eth_efinix_tx_batch(dev, frames, 32) != 0) {
				continue;
			}
			while (!eth_efinix_tx_idle(dev)) {
			}
			best = MIN(best, k_cycle_get_32() - t0);
		}
		printk("DMA, 32 frames of 1510 bytes as %s: %u us (line rate 393 us)\n",
		       split ? "header + payload" : "one buffer", best / 125);
	}
}

static const struct tcp_stream_source source = {
	.prefix_len = 0,
	.payload_len = PAYLOAD,
	.connected = on_connected,
	.closed = on_closed,
	.segment = segment,
};

static const struct ws_stream_source ws_source = {
	.payload_len = PAYLOAD,
	.segments_per_message = 720,
	.connected = on_connected,
	.closed = on_closed,
	.segment = ws_segment,
};

int main(void)
{
	struct tcp_stream_stats stats, last = {0};
	int ret;

	for (uint32_t w = 0; w < WORDS; w++) {
		buffer[w] = w;
	}
	k_sleep(K_SECONDS(1));
	dma_rate(net_if_get_device(net_if_get_default()));
	if (IS_ENABLED(CONFIG_STREAM_TEST_WEBSOCKET)) {
		ret = ws_stream_start(net_if_get_default(), PORT, &ws_source, NULL);
	} else {
		ret = tcp_stream_start(net_if_get_default(), PORT, &source, NULL);
	}
	if (ret < 0) {
		printk("tcp_stream_start: %d\n", ret);
		return 0;
	}

	for (;;) {
		k_sleep(K_SECONDS(1));
		tcp_stream_get_stats(&stats);
		if (connected && stats.acked >= last.acked) {
			printk("%u.%02u MB/s, %u segments in %u batches (%u short at the window "
			       "of %u), %u retransmits, %u rx dropped\n",
			       (uint32_t)((stats.acked - last.acked) / 1000000),
			       (uint32_t)((stats.acked - last.acked) / 10000 % 100),
			       stats.segments - last.segments, stats.batches - last.batches,
			       stats.window_full - last.window_full, stats.window,
			       stats.retransmits, stats.rx_dropped);
			printk("  per batch: DMA busy %u us, idle to next start %u us\n",
			       (stats.busy_cycles - last.busy_cycles) / 125 /
				       MAX(1, stats.batches - last.batches),
			       (stats.gap_cycles - last.gap_cycles) / 125 /
				       MAX(1, stats.batches - last.batches));
		}
		last = stats;
	}
	return 0;
}
