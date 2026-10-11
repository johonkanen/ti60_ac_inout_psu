/*
 * WebSocket over tcp_stream, see ws_stream.h. The handshake (RFC 6455 4.2): the request is
 * collected up to its blank line, the Sec-WebSocket-Key answered with SHA-1 and base64 in a
 * 101 reply sent ahead of the segments; then the segments follow.
 */
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <psa/crypto.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/base64.h>
#include <tcp_stream.h>
#include <ws_stream.h>

LOG_MODULE_REGISTER(ws_stream, LOG_LEVEL_INF);

#define FRAME_HEADER 4          /* FIN/opcode, 126, 16 bit length */
#define REQUEST_MAX  1024
#define WS_GUID      "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

static const struct ws_stream_source *source;
static void *source_data;
static char request[REQUEST_MAX + 1];
static size_t request_len;
static bool upgraded;

static void connected(void *user_data)
{
	request_len = 0;
	upgraded = false;
}

static void closed(void *user_data)
{
	if (upgraded && source->closed != NULL) {
		source->closed(source_data);
	}
	upgraded = false;
}

static int accept_key(const char *key, size_t key_len, char *out, size_t out_size)
{
	char text[64 + sizeof(WS_GUID)];
	uint8_t hash[20];
	size_t hash_len, olen;

	if (key_len == 0 || key_len > 64) {
		return -EINVAL;
	}
	memcpy(text, key, key_len);
	memcpy(&text[key_len], WS_GUID, sizeof(WS_GUID) - 1);
	if (psa_hash_compute(PSA_ALG_SHA_1, (const uint8_t *)text, key_len + sizeof(WS_GUID) - 1,
			     hash, sizeof(hash), &hash_len) != PSA_SUCCESS) {
		return -EIO;
	}
	return base64_encode((uint8_t *)out, out_size, &olen, hash, sizeof(hash));
}

static void answer(void)
{
	static const char name[] = "\nsec-websocket-key:";
	char reply[160], accept[40];
	const char *key = NULL, *end;
	int len;

	/* the header, case-insensitively, at the start of a line */
	for (const char *p = request; *p != '\0'; p++) {
		if (strncasecmp(p, name, sizeof(name) - 1) == 0) {
			key = p + sizeof(name) - 1;
			break;
		}
	}
	if (key == NULL || strncmp(request, "GET ", 4) != 0) {
		LOG_WRN("not a WebSocket request");
		return;
	}
	while (*key == ' ') {
		key++;
	}
	end = key;
	while (*end != '\r' && *end != '\n' && *end != ' ' && *end != '\0') {
		end++;
	}
	if (accept_key(key, end - key, accept, sizeof(accept)) != 0) {
		LOG_WRN("bad Sec-WebSocket-Key");
		return;
	}
	len = snprintk(reply, sizeof(reply),
		       "HTTP/1.1 101 Switching Protocols\r\n"
		       "Upgrade: websocket\r\n"
		       "Connection: Upgrade\r\n"
		       "Sec-WebSocket-Accept: %s\r\n\r\n", accept);
	if (tcp_stream_reply(reply, len) != 0) {
		return;
	}
	upgraded = true;
	if (source->connected != NULL) {
		/* the request's path, "GET <path> HTTP/1.1": what the client asks for */
		char path[64];
		const char *p = request + 4;
		size_t n = 0;

		while (p[n] != ' ' && p[n] != '\r' && p[n] != '\0' && n < sizeof(path) - 1) {
			n++;
		}
		memcpy(path, p, n);
		path[n] = '\0';
		source->connected(path, source_data);
	}
	tcp_stream_open();
}

static void received(const uint8_t *data, size_t len, void *user_data)
{
	if (upgraded) {
		return;         /* frames from the browser: not used */
	}
	len = MIN(len, REQUEST_MAX - request_len);
	memcpy(&request[request_len], data, len);
	request_len += len;
	request[request_len] = '\0';
	if (strstr(request, "\r\n\r\n") != NULL) {
		answer();
	} else if (request_len == REQUEST_MAX) {
		LOG_WRN("request too long");
	}
}

static const void *segment(uint64_t index, uint8_t *prefix, void *user_data)
{
	const void *payload = source->segment(index, source_data);
	uint32_t n = source->message_segments != NULL ? source->message_segments(source_data)
						       : source->segments_per_message;
	uint32_t place = (uint32_t)index % n;  /* 2^32 segments are terabytes away */
	uint8_t opcode = place == 0 ? 0x2 : 0x0;       /* binary, then continuation */

	if (payload == NULL) {
		return NULL;
	}
	prefix[0] = (place == n - 1 || n == 1 ? 0x80 : 0) | opcode;
	prefix[1] = 126;
	prefix[2] = source->payload_len >> 8;
	prefix[3] = source->payload_len & 0xff;
	return payload;
}

static struct tcp_stream_source stream_source = {
	.prefix_len = FRAME_HEADER,
	.connected = connected,
	.closed = closed,
	.received = received,
	.segment = segment,
};

int ws_stream_start(struct net_if *iface, uint16_t port, const struct ws_stream_source *ws_source,
		    void *user_data)
{
	if (ws_source->payload_len < 126 ||
	    (ws_source->segments_per_message == 0 && ws_source->message_segments == NULL)) {
		return -EINVAL;         /* the 16 bit length form only */
	}
	if (psa_crypto_init() != PSA_SUCCESS) {
		return -EIO;
	}
	source = ws_source;
	source_data = user_data;
	stream_source.payload_len = ws_source->payload_len;
	return tcp_stream_start(iface, port, &stream_source, NULL);
}
