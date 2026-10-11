/*
 * A TCP sender beside Zephyr's stack for one fast stream: one connection at a time on one
 * port, data in one direction, built straight from where the data is.
 *
 * The stream is a run of segments of the same size: segment i is the source's prefix for i
 * (prefix_len bytes, written into the frame's header buffer, e.g. a WebSocket frame header)
 * followed by payload_len bytes the source keeps in memory (e.g. a ring the FPGA fills).
 * The transmit DMA reads that payload where it is, the design fills in the checksums
 * (tx-checksum-offload); the CPU writes only a header per segment. A segment is rebuilt
 * from its index when it has to be sent again, so the source must keep a segment's payload
 * until it has been acknowledged (tcp_stream_acked()).
 *
 * The frames of the port are taken from the TSEMAC driver's receive hook; Zephyr's stack
 * never sees the connection. Data from the client goes to received() in order.
 */
#ifndef TCP_STREAM_H
#define TCP_STREAM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/net/net_if.h>

struct tcp_stream_source {
	uint16_t prefix_len;
	uint16_t payload_len; /* prefix_len + payload_len must fit the client's MSS */
	/* a new connection: the stream starts again at segment 0 */
	void (*connected)(void *user_data);
	void (*closed)(void *user_data);
	/* bytes from the client, in order */
	void (*received)(const uint8_t *data, size_t len, void *user_data);
	/*
	 * Segment index: write its prefix and return its payload, or NULL when the source
	 * has nothing for it yet (or wants the stream to wait).
	 */
	const void *(*segment)(uint64_t index, uint8_t *prefix, void *user_data);
};

struct tcp_stream_stats {
	uint64_t acked;         /* stream bytes the client has acknowledged */
	uint32_t segments;      /* segments sent, retransmissions included */
	uint32_t retransmits;   /* times the stream went back to the last acknowledged byte */
	uint32_t batches;
	uint32_t rx_dropped;    /* frames for the port lost for want of a slot */
	uint32_t window;        /* the client's receive window, last seen */
	uint32_t window_full;   /* times a batch stopped short at the window */
	uint32_t busy_cycles;   /* CPU cycles the DMA was seen busy with batches */
	uint32_t gap_cycles;    /* from seeing it idle to starting the next batch */
	uint32_t source_empty;  /* times the source had no segment */
	uint32_t in_flight;     /* bytes sent and not acknowledged, now */
	bool tx_idle;           /* the transmit DMA, now */
};

int tcp_stream_start(struct net_if *iface, uint16_t port, const struct tcp_stream_source *source,
		     void *user_data);

/*
 * The callbacks run in the stream's thread, and the two calls below are for them.
 *
 * tcp_stream_reply() sends bytes ahead of the segments, a short answer before the stream
 * starts (the WebSocket handshake's), at most CONFIG_TCP_STREAM_REPLY_SIZE in all.
 * tcp_stream_open() lets the segments follow; until then none is sent.
 */
int tcp_stream_reply(const void *data, size_t len);
void tcp_stream_open(void);

/*
 * Ends the connection, e.g. when the source cannot give what was asked: no more segments,
 * and a reset once what was sent has been acknowledged (or nothing more comes).
 */
void tcp_stream_close(void);

/* every segment below this index has been acknowledged; any thread */
uint64_t tcp_stream_acked(void);

void tcp_stream_get_stats(struct tcp_stream_stats *stats);

/* why the last connection ended */
enum tcp_stream_end {
	TCP_STREAM_END_NONE,          /* none ended yet */
	TCP_STREAM_END_CLIENT_RESET,  /* the client reset it (RST) */
	TCP_STREAM_END_CLIENT_CLOSED, /* the client closed it (FIN) */
	TCP_STREAM_END_CLIENT_GONE,   /* nothing acknowledged for a while */
	TCP_STREAM_END_NEW_CLIENT,    /* a connection from another client or port took over */
	TCP_STREAM_END_SOURCE,        /* tcp_stream_close(): the source's decision */
};
enum tcp_stream_end tcp_stream_last_end(void);
const char *tcp_stream_end_name(enum tcp_stream_end end);

#endif
