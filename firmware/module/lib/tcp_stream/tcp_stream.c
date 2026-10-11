/*
 * One fast TCP stream beside Zephyr's stack, see tcp_stream.h.
 *
 * Received: the driver's receive hook notes a plain acknowledgement from the client in its
 * interrupt (the latest one, its window, how many in a row were the same) and copies any
 * other frame of the port into a few slots for this thread: the handshake as a server,
 * data from the client in order, FIN and RST.
 *
 * Sent: segments go out in batches of the driver's transmit slots, two sets of them: one is
 * filled while the DMA sends the other, and the DMA's interrupt says when to start the
 * next. Every slot holds a
 * complete header (Ethernet, IPv4, TCP without options, then the source's prefix) written
 * when the connection opens, 2 bytes into a 64 byte buffer so that the IP and TCP fields
 * fall on word boundaries; a segment then takes three word stores (IP id, sequence and
 * acknowledgement numbers), the prefix, and its payload pointer for the DMA. Stores are
 * what costs here: each one goes through the write-through cache to the HyperRAM. The
 * design fills in the checksums. Control frames and the source's reply go one at a time
 * through eth_efinix_tx_batch().
 *
 * Kept simple for a LAN: no timestamps, no SACK, no congestion control beyond the
 * client's window. A loss is repaired by going back to the first unacknowledged segment,
 * after three duplicate acknowledgements or RTO_MS without progress. The stream offset is
 * 64 bits; sequence numbers are its low 32 bits plus the initial one.
 */
#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/byteorder.h>
#include <eth_efinix_tsemac.h>
#include <tcp_stream.h>

LOG_MODULE_REGISTER(tcp_stream, LOG_LEVEL_INF);

#define BATCH      CONFIG_TCP_STREAM_BATCH
#define RX_SLOTS   CONFIG_TCP_STREAM_RX_SLOTS
#define REPLY_SIZE CONFIG_TCP_STREAM_REPLY_SIZE
BUILD_ASSERT(BATCH <= CONFIG_ETH_EFINIX_TSEMAC_TX_BATCH, "a batch fits the driver's slots");

#define HDR_LEN     54          /* Ethernet 14, IPv4 20, TCP 20 */
#define SLOT_LEN    64
#define SLOT_PAD    2           /* the header starts here: IP at 16, TCP at 36 */
#define PREFIX_MAX  (SLOT_LEN - SLOT_PAD - HDR_LEN)
#define RX_LEN      1536
#define RTO_MS      200
#define CLOSE_WAIT_MS 100
#define GONE_MS     3000        /* nothing sent acknowledged for this long: the client is gone */
#define OUR_WINDOW  8192        /* the client sends little: a request, control messages */
#define OUR_MSS     1460

/* offsets in a frame */
#define IP          14
#define IP_ID       (IP + 4)
#define TCP         (IP + 20)
#define TCP_SEQ     (TCP + 4)
#define TCP_ACKNO   (TCP + 8)

#define TCP_FIN 0x01
#define TCP_SYN 0x02
#define TCP_RST 0x04
#define TCP_PSH 0x08
#define TCP_ACK 0x10

enum state {
	LISTEN,
	SYN_RECEIVED,
	ESTABLISHED,
};

static struct {
	const struct device *dev;
	uint16_t port;
	const struct tcp_stream_source *source;
	void *user_data;
	uint8_t mac[6];
	uint8_t ip[4];
	uint16_t seg_len;       /* prefix and payload */

	enum state state;
	uint8_t peer_mac[6];
	uint8_t peer_ip[4];
	uint16_t peer_port;
	uint8_t peer_shift;     /* window scale */
	bool peer_scales;
	uint32_t iss;
	uint32_t rcv_nxt;
	uint64_t una;           /* stream offsets: first unacknowledged byte, */
	uint64_t nxt;           /* next to send, */
	uint64_t max;           /* and the most ever sent */
	uint64_t nxt_segment;   /* the segment at nxt, when nxt is past the reply */
	uint32_t window;        /* bytes the client takes from una on */
	uint32_t went_back_at;  /* the acknowledgement the last fast retransmit answered */
	int64_t progress;       /* uptime of the last acknowledgement that took una forward, */
	int64_t acked_at;       /* the same, but not moved by a go-back */
	bool open;
	bool closing;           /* tcp_stream_close(): no new segments, a reset once all are in */
	enum tcp_stream_end end; /* why it is ending, once that is decided */
	uint8_t reply[REPLY_SIZE];
	size_t reply_len;
	uint16_t ip_id;
	uint8_t template[HDR_LEN];
	struct tcp_stream_stats stats;
} s;

static enum tcp_stream_end last_end = TCP_STREAM_END_NONE;

static struct k_spinlock lock;      /* acked_segments and stats for other threads */
static uint64_t acked_segments;

static uint8_t slots[2 * BATCH][SLOT_LEN] __aligned(64);
static int ready_first, ready_count;    /* a filled set waiting for the DMA */
static int running_first = -1;          /* the set the DMA is sending */
static uint8_t control_slot[SLOT_LEN + 16] __aligned(64);

/* plain acknowledgements, noted by the receive hook */
static volatile struct {
	uint32_t count;
	uint32_t ack;
	uint16_t window;
	uint16_t same;          /* how many in a row had this ack and window */
} fast;
static uint32_t fast_taken;

/* other frames for the port, from the driver's interrupt */
static struct {
	uint16_t len;
	uint8_t data[RX_LEN];
} rx_slot[RX_SLOTS] __aligned(4);
static volatile uint32_t rx_head, rx_tail;
static K_SEM_DEFINE(wake, 0, 1);

static K_THREAD_STACK_DEFINE(stack, CONFIG_TCP_STREAM_STACK_SIZE);
static struct k_thread thread;

static bool rx_hook(const uint8_t *frame, size_t len, void *user_data)
{
	uint32_t head = rx_head;
	const uint8_t *tcp;
	size_t ihl;

	ARG_UNUSED(user_data);
	if (len < HDR_LEN || frame[12] != 0x08 || frame[13] != 0x00 || (frame[14] >> 4) != 4 ||
	    frame[23] != 6 || memcmp(&frame[30], s.ip, 4) != 0) {
		return false;
	}
	ihl = (frame[14] & 0x0f) * 4;
	tcp = &frame[14 + ihl];
	if (ihl < 20 || len < 14 + ihl + 20 || sys_get_be16(&tcp[2]) != s.port) {
		return false;
	}

	if (s.state == ESTABLISHED && (tcp[13] & ~TCP_PSH) == TCP_ACK &&
	    sys_get_be16(&frame[16]) == ihl + (tcp[12] >> 4) * 4 &&
	    memcmp(&frame[26], s.peer_ip, 4) == 0 && sys_get_be16(&tcp[0]) == s.peer_port) {
		uint32_t ack = sys_get_be32(&tcp[8]);
		uint16_t window = sys_get_be16(&tcp[14]);

		fast.same = ack == fast.ack && window == fast.window ? fast.same + 1 : 0;
		fast.ack = ack;
		fast.window = window;
		fast.count++;
		k_sem_give(&wake);
		return true;
	}

	if (head - rx_tail >= RX_SLOTS) {
		s.stats.rx_dropped++;
		return true;
	}
	len = MIN(len, RX_LEN);
	memcpy(rx_slot[head % RX_SLOTS].data, frame, len);
	rx_slot[head % RX_SLOTS].len = len;
	__asm__ volatile("fence" ::: "memory");
	rx_head = head + 1;
	k_sem_give(&wake);
	return true;
}

static void set_template(void)
{
	uint8_t *h = s.template;

	memset(h, 0, HDR_LEN);
	memcpy(&h[0], s.peer_mac, 6);
	memcpy(&h[6], s.mac, 6);
	h[12] = 0x08;
	h[IP] = 0x45;
	sys_put_be16(0x4000, &h[IP + 6]);   /* don't fragment */
	h[IP + 8] = 64;
	h[IP + 9] = 6;
	memcpy(&h[IP + 12], s.ip, 4);
	memcpy(&h[IP + 16], s.peer_ip, 4);
	sys_put_be16(s.port, &h[TCP]);
	sys_put_be16(s.peer_port, &h[TCP + 2]);
	h[TCP + 12] = 5 << 4;
	sys_put_be16(OUR_WINDOW, &h[TCP + 14]);
	/* checksums are left to the design */
}

static uint32_t seq_of(uint64_t offset)
{
	return s.iss + 1 + (uint32_t)offset;
}

/* the data slots: complete headers of full segments, sent with ACK and PSH */
static void prepare_slots(void)
{
	for (int i = 0; i < 2 * BATCH; i++) {
		uint8_t *h = &slots[i][SLOT_PAD];

		memcpy(h, s.template, HDR_LEN);
		sys_put_be16(20 + 20 + s.seg_len, &h[IP + 2]);
		h[TCP + 13] = TCP_ACK | TCP_PSH;
	}
	eth_efinix_tx_slots_init(s.dev, &slots[0][SLOT_PAD], SLOT_LEN, HDR_LEN + s.source->prefix_len,
				 s.source->payload_len, 2 * BATCH);
	ready_count = 0;
}

static void send_frame(const struct eth_efinix_tx_frame *frame)
{
	for (int t = 0; t < 2000; t++) {
		if (eth_efinix_tx_batch(s.dev, frame, 1) == 0) {
			break;
		}
		k_busy_wait(1);
	}
	/* the buffers are used again by the next one: let this one go first */
	for (int t = 0; t < 2000 && !eth_efinix_tx_idle(s.dev); t++) {
		k_busy_wait(1);
	}
}

/* a frame from control_slot: no data, or a SYN-ACK with options, or bytes of the reply */
static void send_control(uint8_t flags, uint64_t offset, const void *data, size_t data_len)
{
	uint8_t *h = &control_slot[SLOT_PAD];
	struct eth_efinix_tx_frame frame = {
		.header = h, .header_len = HDR_LEN, .payload = data, .payload_len = data_len};
	size_t options = 0;

	memcpy(h, s.template, HDR_LEN);
	if (flags & TCP_SYN) {
		/* our MSS and, when the client scales, a window scale of 0 */
		uint8_t *o = &h[HDR_LEN];

		o[0] = 2;
		o[1] = 4;
		sys_put_be16(OUR_MSS, &o[2]);
		options = 4;
		if (s.peer_scales) {
			o[4] = 1;
			o[5] = 3;
			o[6] = 3;
			o[7] = 0;
			options = 8;
		}
		h[TCP + 12] = (5 + options / 4) << 4;
		sys_put_be32(s.iss, &h[TCP_SEQ]);
	} else {
		sys_put_be32(seq_of(offset), &h[TCP_SEQ]);
	}
	sys_put_be16(20 + 20 + options + data_len, &h[IP + 2]);
	sys_put_be16(s.ip_id++, &h[IP_ID]);
	sys_put_be32(s.rcv_nxt, &h[TCP_ACKNO]);
	h[TCP + 13] = flags;
	frame.header_len = HDR_LEN + options;
	send_frame(&frame);
}

static void close_connection(void)
{
	bool was_open = s.state == ESTABLISHED;

	if (was_open) {
		last_end = s.end != TCP_STREAM_END_NONE ? s.end : TCP_STREAM_END_SOURCE;
	}
	s.state = LISTEN;
	s.open = false;
	s.closing = false;
	if (was_open && s.source->closed != NULL) {
		s.source->closed(s.user_data);
	}
	LOG_INF("closed after %llu bytes, %u retransmits: %s", s.una, s.stats.retransmits,
		tcp_stream_end_name(last_end));
}

static void set_nxt(uint64_t offset)
{
	/* back to the start of a segment */
	if (offset <= s.reply_len) {
		s.nxt = offset;
		s.nxt_segment = 0;
	} else {
		s.nxt_segment = (offset - s.reply_len) / s.seg_len;
		s.nxt = s.reply_len + s.nxt_segment * s.seg_len;
	}
}

static void go_back(void)
{
	ready_count = 0;        /* filled from where the stream no longer is */
	set_nxt(s.una);
	s.progress = k_uptime_get();
	s.stats.retransmits++;
}

static void update_acked(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	acked_segments = s.una > s.reply_len ? (s.una - s.reply_len) / s.seg_len : 0;
	s.stats.acked = s.una;
	s.stats.window = s.window;
	k_spin_unlock(&lock, key);
}

static void take_ack(uint32_t ack, uint16_t window, uint32_t same)
{
	int32_t advance = (int32_t)(ack - seq_of(s.una));

	s.window = (uint32_t)window << s.peer_shift;
	if (advance > 0 && (uint64_t)advance <= s.max - s.una) {
		s.una += advance;
		if (s.nxt < s.una) {
			set_nxt(s.una);         /* acknowledged beyond where a go-back restarted */
		}
		s.progress = k_uptime_get();
		s.acked_at = s.progress;
		update_acked();
	} else if (advance == 0 && same >= 3 && s.max > s.una && ack != s.went_back_at) {
		s.went_back_at = ack;
		go_back();
	}
}

static void take_fast_acks(void)
{
	unsigned int key = irq_lock();
	uint32_t count = fast.count, ack = fast.ack, same = fast.same;
	uint16_t window = fast.window;

	irq_unlock(key);
	if (count != fast_taken) {
		fast_taken = count;
		take_ack(ack, window, same);
	}
}

/* the client's MSS, and its window scale into s */
static uint16_t parse_syn_options(const uint8_t *o, size_t len)
{
	uint16_t mss = 536;

	s.peer_scales = false;
	s.peer_shift = 0;
	for (size_t i = 0; i < len;) {
		uint8_t kind = o[i];

		if (kind == 0) {
			break;
		}
		if (kind == 1) {
			i++;
			continue;
		}
		if (i + 1 >= len || o[i + 1] < 2 || i + o[i + 1] > len) {
			break;
		}
		if (kind == 2 && o[i + 1] == 4) {
			mss = sys_get_be16(&o[i + 2]);
		} else if (kind == 3 && o[i + 1] == 3) {
			s.peer_scales = true;
			s.peer_shift = MIN(o[i + 2], 14);
		}
		i += o[i + 1];
	}
	return mss;
}

/* the connection from the handshake on: the stream's state and slots for this client */
static void establish(void)
{
	s.state = ESTABLISHED;
	s.end = TCP_STREAM_END_NONE;
	s.progress = k_uptime_get();
	s.acked_at = s.progress;
	s.went_back_at = 0;
	memset(&s.stats, 0, sizeof(s.stats));
	fast_taken = fast.count;
	prepare_slots();
	LOG_INF("connection from %u.%u.%u.%u:%u", s.peer_ip[0], s.peer_ip[1], s.peer_ip[2],
		s.peer_ip[3], s.peer_port);
	if (s.source->connected != NULL) {
		s.source->connected(s.user_data);
	}
}

/*
 * Another client while a stream runs: a candidate, taken through the handshake beside the
 * stream. It takes the stream over only with a complete request ("GET ..." and the empty
 * line): the old client is then closed the way it accepts (tcp_stream_close(): a reset once
 * all sent is acknowledged; one at an unacknowledged point it ignores, RFC 5961), and once
 * it is, the candidate gets the stream. One that sends nothing for CANDIDATE_MS is reset
 * instead and the stream goes on (a browser's speculative connection took it over before,
 * and then sent nothing).
 */
#define CANDIDATE_MS      2000
#define CANDIDATE_REQUEST 1024

static struct {
	bool active;
	bool ready;             /* its request is in: it takes over once the old client is closed */
	uint8_t mac[6];
	uint8_t ip[4];
	uint16_t port;
	bool scales;
	uint8_t shift;
	uint16_t mss;
	uint32_t iss;
	uint32_t rcv_nxt;
	int64_t since;
	size_t len;
	char request[CANDIDATE_REQUEST];
} cand;

/* a control frame to the candidate: the stream's template and numbers lent for it */
static void send_candidate(uint8_t flags)
{
	uint8_t template[HDR_LEN];
	uint32_t iss = s.iss, rcv_nxt = s.rcv_nxt;
	bool scales = s.peer_scales;

	memcpy(template, s.template, HDR_LEN);
	memcpy(&s.template[0], cand.mac, 6);
	memcpy(&s.template[IP + 16], cand.ip, 4);
	sys_put_be16(cand.port, &s.template[TCP + 2]);
	s.iss = cand.iss;
	s.rcv_nxt = cand.rcv_nxt;
	s.peer_scales = cand.scales;
	send_control(flags, 0, NULL, 0);
	memcpy(s.template, template, HDR_LEN);
	s.iss = iss;
	s.rcv_nxt = rcv_nxt;
	s.peer_scales = scales;
}

static void drop_candidate(const char *why)
{
	if (cand.active) {
		send_candidate(TCP_RST | TCP_ACK);
		cand.active = false;
		LOG_INF("a connection from %u.%u.%u.%u:%u %s: dropped, the stream goes on", cand.ip[0],
			cand.ip[1], cand.ip[2], cand.ip[3], cand.port, why);
	}
}

static void candidate_syn(const uint8_t *f, const uint8_t *tcp, size_t doff, uint32_t seq)
{
	bool scales = s.peer_scales;
	uint8_t shift = s.peer_shift;
	uint16_t port = sys_get_be16(&tcp[0]);

	if (cand.active && cand.port == port && memcmp(cand.ip, &f[26], 4) == 0) {
		if (seq + 1 == cand.rcv_nxt) {
			send_candidate(TCP_SYN | TCP_ACK);   /* our SYN-ACK was lost */
		}
		return;
	}
	if (cand.active && (cand.len > 0 || cand.ready)) {
		return;         /* the one asking already goes first; this one's TCP tries again */
	}
	memcpy(cand.mac, &f[6], 6);
	memcpy(cand.ip, &f[26], 4);
	cand.port = port;
	cand.mss = parse_syn_options(&tcp[20], doff - 20);
	cand.scales = s.peer_scales;
	cand.shift = s.peer_shift;
	s.peer_scales = scales;
	s.peer_shift = shift;
	cand.iss = sys_rand32_get();
	cand.rcv_nxt = seq + 1;
	cand.since = k_uptime_get();
	cand.len = 0;
	cand.ready = false;
	cand.active = true;
	send_candidate(TCP_SYN | TCP_ACK);
}

/* the candidate, its request in and the old client closed, takes the stream over */
static void take_over(void)
{
	cand.active = false;
	memcpy(s.peer_mac, cand.mac, 6);
	memcpy(s.peer_ip, cand.ip, 4);
	s.peer_port = cand.port;
	s.peer_scales = cand.scales;
	s.peer_shift = cand.shift;
	s.iss = cand.iss;
	s.rcv_nxt = cand.rcv_nxt;
	s.una = s.nxt = s.max = s.nxt_segment = 0;
	s.reply_len = 0;
	s.window = 0;
	set_template();
	if (cand.mss < s.seg_len) {
		LOG_WRN("client MSS %u below a segment of %u", cand.mss, s.seg_len);
		send_control(TCP_RST | TCP_ACK, 0, NULL, 0);
		s.state = LISTEN;
		return;
	}
	establish();
	if (s.source->received != NULL) {
		s.source->received((const uint8_t *)cand.request, cand.len, s.user_data);
	}
}

/* a frame of the candidate's: the end of its handshake, its request */
static void candidate_frame(uint8_t flags, uint32_t seq, const uint8_t *data, size_t data_len)
{
	if (flags & TCP_RST) {
		cand.active = false;
		return;
	}
	if (data_len > 0 && seq == cand.rcv_nxt && !cand.ready) {
		size_t n = MIN(data_len, sizeof(cand.request) - 1 - cand.len);

		memcpy(&cand.request[cand.len], data, n);
		cand.len += n;
		cand.request[cand.len] = '\0';
		cand.rcv_nxt += data_len;
		send_candidate(TCP_ACK);
		if (strstr(cand.request, "\r\n\r\n") != NULL) {
			if (strncmp(cand.request, "GET ", 4) == 0) {
				cand.ready = true;
			} else {
				drop_candidate("sent no WebSocket request");
			}
			return;
		}
		if (cand.len >= sizeof(cand.request) - 1) {
			drop_candidate("sent too long a request");
			return;
		}
	}
	if (flags & TCP_FIN) {
		drop_candidate("closed without a request");
	}
}

static void handle_frame(const uint8_t *f, size_t len)
{
	size_t ihl = (f[14] & 0x0f) * 4;
	const uint8_t *tcp = &f[14 + ihl];
	size_t ip_len = MIN(sys_get_be16(&f[16]), len - 14);
	size_t doff = (tcp[12] >> 4) * 4;
	uint8_t flags = tcp[13];
	uint32_t seq = sys_get_be32(&tcp[4]);
	bool from_peer = s.state != LISTEN && memcmp(&f[26], s.peer_ip, 4) == 0 &&
			 sys_get_be16(&tcp[0]) == s.peer_port;
	const uint8_t *data;
	size_t data_len;
	uint16_t mss;

	if (doff < 20 || ihl + doff > ip_len) {
		return;
	}
	data = tcp + doff;
	data_len = ip_len - ihl - doff;

	if (cand.active && !from_peer && memcmp(&f[26], cand.ip, 4) == 0 &&
	    sys_get_be16(&tcp[0]) == cand.port && !(flags & TCP_SYN)) {
		candidate_frame(flags, seq, data, data_len);
		return;
	}

	if (flags & TCP_RST) {
		if (from_peer) {
			s.end = TCP_STREAM_END_CLIENT_RESET;
			close_connection();
		}
		return;
	}

	if (flags & TCP_SYN) {
		if (from_peer && s.state == SYN_RECEIVED && seq + 1 == s.rcv_nxt) {
			send_control(TCP_SYN | TCP_ACK, 0, NULL, 0);   /* our SYN-ACK was lost */
			return;
		}
		if (s.state == ESTABLISHED && !from_peer) {
			/* a new client: a candidate until it asks for the stream */
			candidate_syn(f, tcp, doff, seq);
			return;
		}
		if (s.state != LISTEN) {
			s.end = TCP_STREAM_END_NEW_CLIENT;
			close_connection();
		}
		memcpy(s.peer_mac, &f[6], 6);
		memcpy(s.peer_ip, &f[26], 4);
		s.peer_port = sys_get_be16(&tcp[0]);
		mss = parse_syn_options(&tcp[20], doff - 20);
		s.iss = sys_rand32_get();
		s.rcv_nxt = seq + 1;
		s.una = s.nxt = s.max = s.nxt_segment = 0;
		s.reply_len = 0;
		set_template();
		if (mss < s.seg_len) {
			LOG_WRN("client MSS %u below a segment of %u", mss, s.seg_len);
			send_control(TCP_RST | TCP_ACK, 0, NULL, 0);
			return;
		}
		s.state = SYN_RECEIVED;
		send_control(TCP_SYN | TCP_ACK, 0, NULL, 0);
		return;
	}

	if (!from_peer) {
		return;
	}

	if (s.state == SYN_RECEIVED) {
		if (!(flags & TCP_ACK) || sys_get_be32(&tcp[8]) != s.iss + 1) {
			return;
		}
		establish();
	}

	if (flags & TCP_ACK) {
		take_ack(sys_get_be32(&tcp[8]), sys_get_be16(&tcp[14]), 0);
	}

	if (data_len > 0) {
		if (seq == s.rcv_nxt) {
			s.rcv_nxt += data_len;
			if (s.source->received != NULL) {
				s.source->received(data, data_len, s.user_data);
			}
		}
		send_control(TCP_ACK, s.nxt, NULL, 0);
	}

	if (flags & TCP_FIN) {
		s.end = TCP_STREAM_END_CLIENT_CLOSED;
		s.rcv_nxt = seq + data_len + 1;
		send_control(TCP_RST | TCP_ACK, s.nxt, NULL, 0);
		close_connection();
	}
}

/* the reply, if any of it is still to send: one frame, while the DMA is idle */
static bool send_reply(void)
{
	if (s.nxt >= s.reply_len) {
		return false;
	}
	if (s.max == s.una) {
		s.progress = k_uptime_get();
		s.acked_at = s.progress;
	}
	send_control(TCP_ACK | TCP_PSH, s.nxt, &s.reply[s.nxt], s.reply_len - s.nxt);
	s.nxt = s.reply_len;
	s.nxt_segment = 0;
	s.max = MAX(s.max, s.nxt);
	return true;
}

/* fills slots from first on with the segments from nxt that the window takes */
static int fill_slots(int first)
{
	uint32_t ack_word = sys_cpu_to_be32(s.rcv_nxt);
	int n = 0;

	if (!s.open || s.closing || s.nxt < s.reply_len) {
		return 0;
	}
	while (n < BATCH) {
		uint8_t *slot = slots[first + n];
		uint32_t id = s.ip_id++;
		uint64_t in_flight = s.nxt > s.una ? s.nxt - s.una : 0;
		const void *payload;

		if (in_flight + s.seg_len > s.window) {
			s.stats.window_full++;
			break;
		}
		payload = s.source->segment(s.nxt_segment, &slot[SLOT_PAD + HDR_LEN], s.user_data);
		if (payload == NULL) {
			s.stats.source_empty++;
			break;
		}
		if (s.max == s.una) {
			s.progress = k_uptime_get();    /* the timers run from the first in flight */
			s.acked_at = s.progress;
		}
		*(uint32_t *)&slot[SLOT_PAD + IP_ID] = sys_cpu_to_be32(id << 16 | 0x4000);
		*(uint32_t *)&slot[SLOT_PAD + TCP_SEQ] = sys_cpu_to_be32(seq_of(s.nxt));
		*(uint32_t *)&slot[SLOT_PAD + TCP_ACKNO] = ack_word;
		eth_efinix_tx_slot_set(s.dev, first + n, payload);
		s.nxt += s.seg_len;
		s.nxt_segment++;
		s.max = MAX(s.max, s.nxt);
		n++;
	}
	return n;
}

static void stream_thread(void *p1, void *p2, void *p3)
{
	uint32_t batch_started = 0, idle_seen = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		if (cand.active && cand.ready) {
			if (s.state != ESTABLISHED) {
				take_over();
			} else if (!s.closing) {
				LOG_INF("a new client asked for the stream: closing the stream of the old one");
				s.end = TCP_STREAM_END_NEW_CLIENT;
				tcp_stream_close();
			}
		} else if (cand.active && k_uptime_get() - cand.since > CANDIDATE_MS) {
			drop_candidate("asked for nothing");
		}
		while (rx_tail != rx_head) {
			uint32_t tail = rx_tail;

			handle_frame(rx_slot[tail % RX_SLOTS].data, rx_slot[tail % RX_SLOTS].len);
			rx_tail = tail + 1;
		}
		if (s.state != ESTABLISHED) {
			k_sem_take(&wake, K_FOREVER);
			continue;
		}
		take_fast_acks();
		if (s.closing) {
			/*
			 * The client takes a reset only at the byte it expects next (RFC 5961):
			 * the first one unacknowledged, once all sent is in or nothing more comes.
			 */
			if (s.max == s.una || k_uptime_get() - s.progress > CLOSE_WAIT_MS) {
				s.nxt = s.una;
				send_control(TCP_RST | TCP_ACK, s.una, NULL, 0);
				close_connection();
			} else {
				k_sem_take(&wake, K_MSEC(5));
			}
			continue;
		}
		if (s.max > s.una && k_uptime_get() - s.acked_at > GONE_MS) {
			LOG_WRN("nothing acknowledged for %u ms: the client is gone", GONE_MS);
			s.end = TCP_STREAM_END_CLIENT_GONE;
			send_control(TCP_RST | TCP_ACK, s.una, NULL, 0);
			close_connection();
			continue;
		}
		if (s.max > s.una && k_uptime_get() - s.progress > RTO_MS) {
			go_back();
		}

		bool idle = eth_efinix_tx_idle(s.dev);

		if (idle && running_first >= 0) {
			uint32_t now = k_cycle_get_32();

			s.stats.busy_cycles += now - batch_started;
			idle_seen = now;
			running_first = -1;
		}
		if (idle && send_reply()) {
			continue;
		}
		if (ready_count == 0) {
			ready_first = running_first == 0 ? BATCH : 0;
			ready_count = fill_slots(ready_first);
		}
		if (idle && ready_count > 0) {
			if (eth_efinix_tx_slots_send(s.dev, ready_first, ready_count) == 0) {
				batch_started = k_cycle_get_32();
				if (idle_seen != 0) {
					s.stats.gap_cycles += batch_started - idle_seen;
					idle_seen = 0;
				}
				running_first = ready_first;
				s.stats.segments += ready_count;
				s.stats.batches++;
				ready_count = 0;
			}
			continue;       /* fill the other set while this one goes */
		}
		if (!idle && IS_ENABLED(CONFIG_TCP_STREAM_POLL)) {
			k_busy_wait(1);
		} else {
			/* the DMA's interrupt, an acknowledgement, or the window and the source */
			k_sem_take(&wake, idle ? K_MSEC(5) : K_MSEC(2));
		}
	}
}

static void tx_done(void *user_data)
{
	ARG_UNUSED(user_data);
	k_sem_give(&wake);
}

int tcp_stream_reply(const void *data, size_t len)
{
	if (s.open || s.reply_len + len > REPLY_SIZE) {
		return -ENOMEM;
	}
	memcpy(&s.reply[s.reply_len], data, len);
	s.reply_len += len;
	return 0;
}

void tcp_stream_open(void)
{
	s.open = true;
}

void tcp_stream_close(void)
{
	if (s.end == TCP_STREAM_END_NONE) {
		s.end = TCP_STREAM_END_SOURCE;
	}
	if (s.state == ESTABLISHED) {
		s.closing = true;
		s.progress = k_uptime_get();
	} else if (s.state == SYN_RECEIVED) {
		close_connection();
	}
}

enum tcp_stream_end tcp_stream_last_end(void)
{
	return last_end;
}

const char *tcp_stream_end_name(enum tcp_stream_end end)
{
	switch (end) {
	case TCP_STREAM_END_CLIENT_RESET:
		return "client reset";
	case TCP_STREAM_END_CLIENT_CLOSED:
		return "client closed";
	case TCP_STREAM_END_CLIENT_GONE:
		return "client gone";
	case TCP_STREAM_END_NEW_CLIENT:
		return "new client";
	case TCP_STREAM_END_SOURCE:
		return "closed by the source";
	default:
		return "none";
	}
}

uint64_t tcp_stream_acked(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint64_t acked = acked_segments;

	k_spin_unlock(&lock, key);
	return acked;
}

void tcp_stream_get_stats(struct tcp_stream_stats *stats)
{
	k_spinlock_key_t key = k_spin_lock(&lock);

	*stats = s.stats;
	k_spin_unlock(&lock, key);
	stats->in_flight = (uint32_t)(s.max - s.una);
	stats->tx_idle = s.dev != NULL && eth_efinix_tx_idle(s.dev);
}

int tcp_stream_start(struct net_if *iface, uint16_t port, const struct tcp_stream_source *source,
		     void *user_data)
{
	struct net_in_addr *addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
	struct net_linkaddr *link = net_if_get_link_addr(iface);

	if (addr == NULL || link == NULL || link->len != 6 || source->prefix_len > PREFIX_MAX ||
	    source->payload_len == 0 || source->prefix_len + source->payload_len > OUR_MSS) {
		return -EINVAL;
	}
	s.dev = net_if_get_device(iface);
	s.port = port;
	s.source = source;
	s.user_data = user_data;
	s.seg_len = source->prefix_len + source->payload_len;
	memcpy(s.mac, link->addr, 6);
	memcpy(s.ip, addr, 4);
	s.state = LISTEN;

	k_thread_create(&thread, stack, K_THREAD_STACK_SIZEOF(stack), stream_thread, NULL, NULL,
			NULL, CONFIG_TCP_STREAM_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&thread, "tcp_stream");
	eth_efinix_set_rx_hook(s.dev, rx_hook, NULL);
	eth_efinix_set_tx_hook(s.dev, tx_done, NULL);
	LOG_INF("port %u, segments of %u + %u bytes", port, source->prefix_len,
		source->payload_len);
	return 0;
}
