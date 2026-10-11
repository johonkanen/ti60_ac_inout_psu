/*
 * Fast path of the Efinix TSEMAC driver, beside the network stack: for a sender that builds
 * its own frames (tcp_stream).
 *
 * The receive hook sees every received frame before the stack does, in the driver's
 * interrupt, and takes the ones it returns true for. A batch is a run of frames the transmit
 * DMA sends in one go, each a header and a payload read where they are: the call starts the
 * DMA and returns, and the memory must stay as it is until eth_efinix_tx_idle() says the
 * batch has gone. The stack's own frames wait for a batch to finish.
 *
 * Slots are the fast form of a batch for frames that all look alike: set up once with a
 * header buffer each (at a fixed stride) and the header and payload lengths, after which a
 * send only says where each slot's payload is. There are twice as many slots as frames in
 * a batch, so the caller can fill one half while the DMA sends the other.
 */
#ifndef ETH_EFINIX_TSEMAC_H
#define ETH_EFINIX_TSEMAC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

struct eth_efinix_tx_frame {
	const void *header;
	uint16_t header_len;
	const void *payload; /* may be NULL with payload_len 0 */
	uint16_t payload_len;
};

typedef bool (*eth_efinix_rx_hook_t)(const uint8_t *frame, size_t len, void *user_data);

void eth_efinix_set_rx_hook(const struct device *dev, eth_efinix_rx_hook_t hook, void *user_data);

/*
 * Starts sending count frames, at most CONFIG_ETH_EFINIX_TSEMAC_TX_BATCH. Returns 0, or
 * -EBUSY while the DMA is still sending; waits for a frame of the stack to go first.
 */
int eth_efinix_tx_batch(const struct device *dev, const struct eth_efinix_tx_frame *frames,
			int count);

/* true when the transmit DMA has sent everything it was given */
bool eth_efinix_tx_idle(const struct device *dev);

/* called in the transmit DMA's interrupt when it has sent everything it was given */
void eth_efinix_set_tx_hook(const struct device *dev, void (*hook)(void *user_data),
			    void *user_data);

/* count slots, at most 2 * CONFIG_ETH_EFINIX_TSEMAC_TX_BATCH; slot i's header at header + i*stride */
int eth_efinix_tx_slots_init(const struct device *dev, const uint8_t *header, size_t stride,
			     uint16_t header_len, uint16_t payload_len, int count);

/* the payload of slot i for its next send; not while the DMA is sending it */
void eth_efinix_tx_slot_set(const struct device *dev, int slot, const void *payload);

/* sends slots first..first+count-1; 0, or -EBUSY as eth_efinix_tx_batch() */
int eth_efinix_tx_slots_send(const struct device *dev, int first, int count);

/* the interface's state, for finding out where it stops: the driver's counts since start,
 * the MAC's own counters, the DMA channels' status registers and the link */
struct eth_efinix_stats {
	uint32_t rx_frames;      /* taken from the receive ring (the stack's and the hook's) */
	uint32_t rx_dropped;     /* no packet for it, or the stack refused it */
	uint32_t rx_restarts;    /* the receive DMA started again after it had stopped */
	uint32_t tx_frames;      /* sent by the stack's path */
	uint32_t tx_errors;      /* the stack's frames not sent (a DMA timeout or a failure) */
	int64_t last_rx_ms;      /* uptime of the last frame received, 0 none */
	uint32_t mac_rx_ok, mac_tx_ok, mac_crc_errors, mac_in_errors, mac_out_errors;
	uint32_t dma_rx_status, dma_tx_status;
	bool link_up;
	uint16_t link_mbps;
};

void eth_efinix_get_stats(const struct device *dev, struct eth_efinix_stats *stats);

#endif
