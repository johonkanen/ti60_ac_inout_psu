/*
 * A WebSocket server over tcp_stream: a browser's new WebSocket("ws://<board>:<port>/<path>")
 * gets a stream of binary messages, each segments_per_message (or message_segments()) segments
 * of payload_len bytes, the source's data as it is in memory; the source is told the path.
 *
 * Every TCP segment carries one WebSocket frame of the message, a 4 byte header and the
 * payload: binary opcode on the first frame of a message, continuation on the others, FIN on
 * the last. The browser joins them into one message. Data from the browser is not read
 * after the handshake; closing the connection ends the stream.
 */
#ifndef WS_STREAM_H
#define WS_STREAM_H

#include <stdint.h>
#include <zephyr/net/net_if.h>

struct ws_stream_source {
	uint16_t payload_len;           /* at most 1456: a frame header and this fit one segment */
	uint32_t segments_per_message;
	/* if given, the segments of each message of this connection instead (asked per segment) */
	uint32_t (*message_segments)(void *user_data);
	/* the stream starts again at segment 0; path: the request's ("/", "/capture?id=3") */
	void (*connected)(const char *path, void *user_data);
	void (*closed)(void *user_data);
	/* the payload of segment index, or NULL when it is not there yet */
	const void *(*segment)(uint64_t index, void *user_data);
};

int ws_stream_start(struct net_if *iface, uint16_t port, const struct ws_stream_source *source,
		    void *user_data);

#endif
