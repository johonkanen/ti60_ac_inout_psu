/*
 * Efinix Triple Speed Ethernet MAC fed by the Sapphire scatter-gather DMA,
 * as wired in Efinix's TSEMAC example designs.
 *
 * DMA channel 0 writes received frames into a ring of descriptors, channel 1 reads
 * the frame to transmit straight from the packet's buffers, a descriptor per fragment
 * (a frame with more fragments than descriptors is copied into one buffer first). A
 * channel walks the linked list and stops at the first descriptor whose status already
 * has the completed bit set. The DMA reaches only
 * the external memory, so the descriptors and buffers must live there, which they
 * do when Zephyr runs from it.
 *
 * The data cache is write-through: transmit needs no flush, but everything the DMA
 * wrote must be invalidated before the CPU reads it.
 *
 * Register map and programming sequence from Efinix's Linux driver (efinix_tse.c).
 *
 * Beside the stack, eth_efinix_tsemac.h: a receive hook that takes frames before the stack,
 * and batches of frames the transmit DMA sends without the caller waiting.
 */

#define DT_DRV_COMPAT efinix_tsemac

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/mii.h>
#include <zephyr/net/phy.h>
#include <zephyr/logging/log.h>
#include <eth_efinix_tsemac.h>

LOG_MODULE_REGISTER(eth_efinix_tsemac, CONFIG_ETHERNET_LOG_LEVEL);

/* MAC registers */
#define MAC_COMMAND_CONFIG       0x008
#define MAC_ADDR_LO              0x00c
#define MAC_ADDR_HI              0x010
#define MAC_FRAMES_TX_OK         0x068
#define MAC_FRAMES_RX_OK         0x06c
#define MAC_CRC_ERRORS           0x070
#define MAC_IF_IN_ERRORS         0x088
#define MAC_IF_OUT_ERRORS        0x08c
#define MAC_ADDR_MASK_LO         0x144
#define MAC_ADDR_MASK_HI         0x148

#define CMD_TX_ENA               BIT(0)
#define CMD_RX_ENA               BIT(1)
#define CMD_PROMIS_EN            BIT(4)
#define CMD_PAUSE_IGNORE         BIT(8)
#define CMD_SPEED_MASK           GENMASK(18, 16)
#define CMD_SPEED_10             BIT(16)
#define CMD_SPEED_100            BIT(17)
#define CMD_SPEED_1000           BIT(18)

/* example design control registers */
#define CTRL_PHY_RESET           0x204
#define CTRL_DMA_RX_RESET        0x208
#define CTRL_DMA_TX_RESET        0x20c

/* DMA channel registers */
#define DMA_RX                   0x00
#define DMA_TX                   0x80
#define DMA_INPUT_CONFIG         0x0c
#define DMA_OUTPUT_CONFIG        0x1c
#define DMA_STATUS               0x2c
#define DMA_PRIORITY             0x44
#define DMA_INTERRUPT_ENABLE     0x50
#define DMA_INTERRUPT_PENDING    0x54
#define DMA_LINKED_LIST_HEAD     0x70

#define DMA_CONFIG_MEMORY        BIT(12)
#define DMA_CONFIG_COMPLETION_ON_PACKET BIT(13)
#define DMA_CONFIG_WAIT_ON_PACKET BIT(14)
#define DMA_BYTES_PER_BURST      64
#define DMA_STATUS_BUSY          BIT(0)
#define DMA_STATUS_STOP          BIT(2)
#define DMA_STATUS_LINKED_LIST_START BIT(4)
#define DMA_IRQ_CHANNEL_COMPLETION BIT(2)
#define DMA_IRQ_LINKED_LIST_UPDATE BIT(3)
#define DMA_IRQ_ALL              GENMASK(4, 0)

#define DESC_BYTES               GENMASK(26, 0)
#define DESC_END_OF_PACKET       BIT(30)
#define DESC_COMPLETED           BIT(31)

#define BUF_SIZE                 1536
#define RX_COUNT                 CONFIG_ETH_EFINIX_TSEMAC_RX_DESCRIPTORS
#define TX_COUNT                 CONFIG_ETH_EFINIX_TSEMAC_TX_DESCRIPTORS
#define BATCH_COUNT              CONFIG_ETH_EFINIX_TSEMAC_TX_BATCH
#define TX_TIMEOUT_US            100000

#define BMCR_RESTART_AN          (MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART)

struct dmasg_descriptor {
	uint32_t status;
	uint32_t control;
	uint64_t from;
	uint64_t to;
	uint64_t next;
} __aligned(64);

struct eth_efinix_config {
	mem_addr_t mac;
	mem_addr_t dma;
	const struct device *phy_dev;
	const uint32_t *phy_updates; /* register, mask, value */
	size_t phy_update_count;
	bool rx_clock_reset;
	bool tx_checksum;
	void (*irq_config)(void);
	struct net_eth_mac_config mcfg;
	struct dmasg_descriptor *rx_desc;
	uint8_t (*rx_buf)[BUF_SIZE];
	struct dmasg_descriptor *tx_desc;
	uint8_t *tx_buf;
	struct dmasg_descriptor *batch_desc; /* 2 per frame and the stop */
	struct dmasg_descriptor *slot_desc;  /* the same for the slots, 2 sets */
};

struct eth_efinix_data {
	struct net_if *iface;
	uint8_t mac_addr[6];
	uint32_t rx_next;
	uint32_t rx_dropped;
	uint32_t rx_frames;
	uint32_t rx_restarts;
	uint32_t tx_frames;
	uint32_t tx_errors;
	int64_t last_rx_ms;
	bool link_up;
	uint16_t link_mbps;
	struct k_mutex tx_lock;
	eth_efinix_rx_hook_t rx_hook;
	void *rx_hook_data;
	void (*tx_hook)(void *user_data);
	void *tx_hook_data;
};

static inline void dcache_invalidate_all(void)
{
	/* VexRiscv custom instruction: invalidate the whole data cache */
	__asm__ volatile(".word 0x500F" ::: "memory");
}

static inline uint32_t dma_read(const struct eth_efinix_config *cfg, uint32_t ch, uint32_t reg)
{
	return sys_read32(cfg->dma + ch + reg);
}

static inline void dma_write(const struct eth_efinix_config *cfg, uint32_t ch, uint32_t reg,
			     uint32_t value)
{
	sys_write32(value, cfg->dma + ch + reg);
}

static void rx_arm(const struct eth_efinix_config *cfg, uint32_t i)
{
	struct dmasg_descriptor *desc = &cfg->rx_desc[i];

	desc->to = (uintptr_t)cfg->rx_buf[i];
	desc->control = (BUF_SIZE - 1) & DESC_BYTES;
	desc->next = (uintptr_t)&cfg->rx_desc[(i + 1) % RX_COUNT];
	desc->status = 0;
}

static void rx_restart(const struct eth_efinix_config *cfg, struct eth_efinix_data *data)
{
	if (!(dma_read(cfg, DMA_RX, DMA_STATUS) & DMA_STATUS_BUSY)) {
		data->rx_restarts++;
		dma_write(cfg, DMA_RX, DMA_LINKED_LIST_HEAD,
			  (uintptr_t)&cfg->rx_desc[data->rx_next]);
		dma_write(cfg, DMA_RX, DMA_STATUS, DMA_STATUS_LINKED_LIST_START);
	}
}

static void eth_efinix_rx(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;

	dcache_invalidate_all();

	/* the DMA fills the ring in order from where it was started */
	for (int n = 0; n < RX_COUNT; n++) {
		uint32_t i = data->rx_next;
		uint32_t status = cfg->rx_desc[i].status;

		if (!(status & DESC_COMPLETED)) {
			break;
		}

		uint32_t len = status & DESC_BYTES;

		data->rx_frames++;
		data->last_rx_ms = k_uptime_get();

		if (len > 0 && len <= BUF_SIZE && data->rx_hook != NULL &&
		    data->rx_hook(cfg->rx_buf[i], len, data->rx_hook_data)) {
			/* taken by the fast path */
		} else if (len > 0 && len <= BUF_SIZE && data->iface != NULL) {
			struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(
				data->iface, len, NET_AF_UNSPEC, 0, K_NO_WAIT);

			if (pkt == NULL) {
				data->rx_dropped++;
			} else if (net_pkt_write(pkt, cfg->rx_buf[i], len) < 0 ||
				   net_recv_data(data->iface, pkt) < 0) {
				net_pkt_unref(pkt);
				data->rx_dropped++;
			}
		}

		rx_arm(cfg, i);
		data->rx_next = (i + 1) % RX_COUNT;
	}

	rx_restart(cfg, data);
}

static void eth_efinix_rx_isr(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;

	dma_write(cfg, DMA_RX, DMA_INTERRUPT_PENDING,
		  dma_read(cfg, DMA_RX, DMA_INTERRUPT_PENDING));
	eth_efinix_rx(dev);
}

static void eth_efinix_tx_isr(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;

	dma_write(cfg, DMA_TX, DMA_INTERRUPT_PENDING,
		  dma_read(cfg, DMA_TX, DMA_INTERRUPT_PENDING));
	if (data->tx_hook != NULL) {
		data->tx_hook(data->tx_hook_data);
	}
}

/* a descriptor per fragment, at desc[0..]; returns their number, 0 when they do not fit */
static int tx_chain(const struct eth_efinix_config *cfg, struct net_pkt *pkt)
{
	struct dmasg_descriptor *desc = cfg->tx_desc;
	int n = 0;

	for (struct net_buf *frag = pkt->frags; frag != NULL; frag = frag->frags) {
		if (frag->len == 0) {
			continue;
		}
		if (n == TX_COUNT) {
			return 0;
		}
		desc[n].from = (uintptr_t)frag->data;
		desc[n].control = (frag->len - 1) & DESC_BYTES;
		n++;
	}
	return n;
}

static int eth_efinix_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;
	struct dmasg_descriptor *desc = cfg->tx_desc;
	size_t len = net_pkt_get_len(pkt);
	int ret = 0;
	int n;

	if (len > BUF_SIZE) {
		return -EMSGSIZE;
	}

	k_mutex_lock(&data->tx_lock, K_FOREVER);

	/* a batch may still be going */
	for (int t = 0; dma_read(cfg, DMA_TX, DMA_STATUS) & DMA_STATUS_BUSY; t++) {
		if (t >= TX_TIMEOUT_US) {
			ret = -ETIMEDOUT;
			goto out;
		}
		k_busy_wait(1);
	}

	/*
	 * The DMA reads the fragments where they are: the data cache is write-through, so
	 * memory holds them, and this waits for the DMA before the stack may free them.
	 */
	n = tx_chain(cfg, pkt);
	if (n == 0) {
		if (net_pkt_read(pkt, cfg->tx_buf, len) < 0) {
			ret = -EIO;
			goto out;
		}
		desc[0].from = (uintptr_t)cfg->tx_buf;
		desc[0].control = (len - 1) & DESC_BYTES;
		n = 1;
	}
	desc[n - 1].control |= DESC_END_OF_PACKET;
	desc[n].status = DESC_COMPLETED; /* where the channel stops */
	for (int i = 0; i < n; i++) {
		desc[i].next = (uintptr_t)&desc[i + 1];
		desc[i].status = 0;
	}
	__asm__ volatile("fence" ::: "memory");

	dma_write(cfg, DMA_TX, DMA_LINKED_LIST_HEAD, (uintptr_t)&desc[0]);
	dma_write(cfg, DMA_TX, DMA_STATUS, DMA_STATUS_LINKED_LIST_START);

	for (int t = 0; dma_read(cfg, DMA_TX, DMA_STATUS) & DMA_STATUS_BUSY; t++) {
		if (t >= TX_TIMEOUT_US) {
			dma_write(cfg, DMA_TX, DMA_STATUS, DMA_STATUS_STOP);
			ret = -ETIMEDOUT;
			goto out;
		}
		k_busy_wait(1);
	}

	dcache_invalidate_all();
	if (!(desc[n - 1].status & DESC_COMPLETED)) {
		ret = -EIO;
	}

out:
	if (ret < 0) {
		data->tx_errors++;
	} else {
		data->tx_frames++;
	}
	k_mutex_unlock(&data->tx_lock);
	if (ret < 0) {
		LOG_ERR("TX of %u bytes failed: %d", (unsigned int)len, ret);
	}
	return ret;
}

void eth_efinix_get_stats(const struct device *dev, struct eth_efinix_stats *stats)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;

	stats->rx_frames = data->rx_frames;
	stats->rx_dropped = data->rx_dropped;
	stats->rx_restarts = data->rx_restarts;
	stats->tx_frames = data->tx_frames;
	stats->tx_errors = data->tx_errors;
	stats->last_rx_ms = data->last_rx_ms;
	stats->mac_rx_ok = sys_read32(cfg->mac + MAC_FRAMES_RX_OK);
	stats->mac_tx_ok = sys_read32(cfg->mac + MAC_FRAMES_TX_OK);
	stats->mac_crc_errors = sys_read32(cfg->mac + MAC_CRC_ERRORS);
	stats->mac_in_errors = sys_read32(cfg->mac + MAC_IF_IN_ERRORS);
	stats->mac_out_errors = sys_read32(cfg->mac + MAC_IF_OUT_ERRORS);
	stats->dma_rx_status = dma_read(cfg, DMA_RX, DMA_STATUS);
	stats->dma_tx_status = dma_read(cfg, DMA_TX, DMA_STATUS);
	stats->link_up = data->link_up;
	stats->link_mbps = data->link_mbps;
}

void eth_efinix_set_rx_hook(const struct device *dev, eth_efinix_rx_hook_t hook, void *user_data)
{
	struct eth_efinix_data *data = dev->data;
	unsigned int key = irq_lock();

	data->rx_hook = hook;
	data->rx_hook_data = user_data;
	irq_unlock(key);
}

void eth_efinix_set_tx_hook(const struct device *dev, void (*hook)(void *user_data),
			    void *user_data)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;
	unsigned int key = irq_lock();

	data->tx_hook = hook;
	data->tx_hook_data = user_data;
	dma_write(cfg, DMA_TX, DMA_INTERRUPT_PENDING, DMA_IRQ_ALL);
	dma_write(cfg, DMA_TX, DMA_INTERRUPT_ENABLE, hook != NULL ? DMA_IRQ_CHANNEL_COMPLETION : 0);
	irq_unlock(key);
}

bool eth_efinix_tx_idle(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;

	return !(dma_read(cfg, DMA_TX, DMA_STATUS) & DMA_STATUS_BUSY);
}

int eth_efinix_tx_batch(const struct device *dev, const struct eth_efinix_tx_frame *frames,
			int count)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;
	struct dmasg_descriptor *desc = cfg->batch_desc;
	int n = 0;

	if (count <= 0 || count > BATCH_COUNT) {
		return -EINVAL;
	}
	/*
	 * Wait for a frame of the stack: blocking lends this thread's priority to the
	 * stack's, which a caller that only retried would starve.
	 */
	k_mutex_lock(&data->tx_lock, K_FOREVER);
	if (!eth_efinix_tx_idle(dev)) {
		k_mutex_unlock(&data->tx_lock);
		return -EBUSY;
	}

	for (int f = 0; f < count; f++) {
		desc[n].from = (uintptr_t)frames[f].header;
		desc[n].control = (frames[f].header_len - 1) & DESC_BYTES;
		n++;
		if (frames[f].payload_len > 0) {
			desc[n].from = (uintptr_t)frames[f].payload;
			desc[n].control = (frames[f].payload_len - 1) & DESC_BYTES;
			n++;
		}
		desc[n - 1].control |= DESC_END_OF_PACKET;
	}
	desc[n].status = DESC_COMPLETED; /* where the channel stops */
	for (int i = 0; i < n; i++) {
		desc[i].next = (uintptr_t)&desc[i + 1];
		desc[i].status = 0;
	}
	__asm__ volatile("fence" ::: "memory");

	dma_write(cfg, DMA_TX, DMA_LINKED_LIST_HEAD, (uintptr_t)&desc[0]);
	dma_write(cfg, DMA_TX, DMA_STATUS, DMA_STATUS_LINKED_LIST_START);
	k_mutex_unlock(&data->tx_lock);
	return 0;
}

int eth_efinix_tx_slots_init(const struct device *dev, const uint8_t *header, size_t stride,
			     uint16_t header_len, uint16_t payload_len, int count)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct dmasg_descriptor *desc = cfg->slot_desc;

	if (count <= 0 || count > 2 * BATCH_COUNT || header_len == 0 || payload_len == 0) {
		return -EINVAL;
	}
	for (int i = 0; i < count; i++) {
		desc[2 * i].from = (uintptr_t)(header + i * stride);
		desc[2 * i].control = (header_len - 1) & DESC_BYTES;
		desc[2 * i].next = (uintptr_t)&desc[2 * i + 1];
		desc[2 * i + 1].from = 0;
		desc[2 * i + 1].control = ((payload_len - 1) & DESC_BYTES) | DESC_END_OF_PACKET;
		desc[2 * i + 1].next = (uintptr_t)&desc[2 * i + 2];
	}
	return 0;
}

void eth_efinix_tx_slot_set(const struct device *dev, int slot, const void *payload)
{
	const struct eth_efinix_config *cfg = dev->config;

	/* the low word: the high one stays 0 */
	*(volatile uint32_t *)&cfg->slot_desc[2 * slot + 1].from = (uintptr_t)payload;
}

int eth_efinix_tx_slots_send(const struct device *dev, int first, int count)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;
	struct dmasg_descriptor *desc = &cfg->slot_desc[2 * first];

	if (first < 0 || count <= 0 || first + count > 2 * BATCH_COUNT) {
		return -EINVAL;
	}
	/*
	 * Wait for a frame of the stack: blocking lends this thread's priority to the
	 * stack's, which a caller that only retried would starve.
	 */
	k_mutex_lock(&data->tx_lock, K_FOREVER);
	if (!eth_efinix_tx_idle(dev)) {
		k_mutex_unlock(&data->tx_lock);
		return -EBUSY;
	}
	for (int i = 0; i < 2 * count; i++) {
		desc[i].status = 0;
	}
	desc[2 * count].status = DESC_COMPLETED; /* where the channel stops */
	__asm__ volatile("fence" ::: "memory");

	dma_write(cfg, DMA_TX, DMA_LINKED_LIST_HEAD, (uintptr_t)&desc[0]);
	dma_write(cfg, DMA_TX, DMA_STATUS, DMA_STATUS_LINKED_LIST_START);
	k_mutex_unlock(&data->tx_lock);
	return 0;
}

static void eth_efinix_set_mac_address(const struct eth_efinix_config *cfg, const uint8_t *addr)
{
	sys_write32(addr[0] | addr[1] << 8 | addr[2] << 16 | addr[3] << 24, cfg->mac + MAC_ADDR_LO);
	sys_write32(addr[4] | addr[5] << 8, cfg->mac + MAC_ADDR_HI);
	sys_write32(0xffffffff, cfg->mac + MAC_ADDR_MASK_LO);
	sys_write32(0x0000ffff, cfg->mac + MAC_ADDR_MASK_HI);
}

static void eth_efinix_dma_init(const struct eth_efinix_config *cfg, struct eth_efinix_data *data)
{
	for (uint32_t i = 0; i < RX_COUNT; i++) {
		rx_arm(cfg, i);
	}
	for (uint32_t i = 0; i <= TX_COUNT; i++) {
		cfg->tx_desc[i].status = DESC_COMPLETED;
	}
	data->rx_next = 0;
	__asm__ volatile("fence" ::: "memory");

	dma_write(cfg, DMA_RX, DMA_INPUT_CONFIG,
		  DMA_CONFIG_COMPLETION_ON_PACKET | DMA_CONFIG_WAIT_ON_PACKET);
	dma_write(cfg, DMA_RX, DMA_OUTPUT_CONFIG, DMA_CONFIG_MEMORY | (DMA_BYTES_PER_BURST - 1));
	dma_write(cfg, DMA_RX, DMA_LINKED_LIST_HEAD, (uintptr_t)&cfg->rx_desc[0]);
	dma_write(cfg, DMA_RX, DMA_PRIORITY, 0);
	dma_write(cfg, DMA_RX, DMA_INTERRUPT_PENDING, DMA_IRQ_ALL);
	dma_write(cfg, DMA_RX, DMA_INTERRUPT_ENABLE, DMA_IRQ_LINKED_LIST_UPDATE);
	dma_write(cfg, DMA_RX, DMA_STATUS, DMA_STATUS_LINKED_LIST_START);

	dma_write(cfg, DMA_TX, DMA_STATUS, DMA_STATUS_STOP);
	dma_write(cfg, DMA_TX, DMA_INPUT_CONFIG, DMA_CONFIG_MEMORY | (DMA_BYTES_PER_BURST - 1));
	dma_write(cfg, DMA_TX, DMA_OUTPUT_CONFIG, 0);
	dma_write(cfg, DMA_TX, DMA_PRIORITY, 1);
	dma_write(cfg, DMA_TX, DMA_INTERRUPT_ENABLE, 0);
}

static int eth_efinix_init(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;
	uint32_t cmd;

	k_mutex_init(&data->tx_lock);
	(void)net_eth_mac_load(&cfg->mcfg, data->mac_addr);

	/* the MDIO driver has already taken the MAC and PHY out of reset */
	cmd = sys_read32(cfg->mac + MAC_COMMAND_CONFIG);
	cmd &= ~(CMD_TX_ENA | CMD_RX_ENA | CMD_PAUSE_IGNORE);
	if (IS_ENABLED(CONFIG_ETH_EFINIX_TSEMAC_PROMISCUOUS)) {
		cmd |= CMD_PROMIS_EN;
	} else {
		cmd &= ~CMD_PROMIS_EN;
	}
	sys_write32(cmd, cfg->mac + MAC_COMMAND_CONFIG);
	eth_efinix_set_mac_address(cfg, data->mac_addr);

	sys_write32(1, cfg->mac + CTRL_DMA_RX_RESET);
	sys_write32(1, cfg->mac + CTRL_DMA_TX_RESET);
	eth_efinix_dma_init(cfg, data);
	cfg->irq_config();
	sys_write32(0, cfg->mac + CTRL_DMA_TX_RESET);
	sys_write32(0, cfg->mac + CTRL_DMA_RX_RESET);

	sys_write32(sys_read32(cfg->mac + MAC_COMMAND_CONFIG) | CMD_TX_ENA | CMD_RX_ENA,
		    cfg->mac + MAC_COMMAND_CONFIG);

	return 0;
}

static void eth_efinix_phy_setup(const struct eth_efinix_config *cfg)
{
	const struct device *phy = cfg->phy_dev;
	uint32_t val;

	for (size_t i = 0; i + 2 < cfg->phy_update_count; i += 3) {
		uint16_t reg = cfg->phy_updates[i];
		uint32_t mask = cfg->phy_updates[i + 1];
		uint32_t value = cfg->phy_updates[i + 2];

		if (mask != 0xffff) {
			if (phy_read(phy, reg, &val) != 0) {
				LOG_ERR("PHY register 0x%02x does not read", reg);
				continue;
			}
			value = (val & ~mask) | (value & mask);
		}
		phy_write(phy, reg, value);
	}
	if (cfg->phy_update_count != 0 && phy_read(phy, MII_BMCR, &val) == 0) {
		phy_write(phy, MII_BMCR, val | BMCR_RESTART_AN);
	}
}

static void eth_efinix_link_changed(const struct device *phy, struct phy_link_state *state,
				    void *user_data)
{
	const struct device *dev = user_data;
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;
	uint32_t cmd;

	ARG_UNUSED(phy);

	data->link_up = state->is_up;
	data->link_mbps = PHY_LINK_IS_SPEED_1000M(state->speed) ? 1000
			  : PHY_LINK_IS_SPEED_100M(state->speed) ? 100 : 10;
	if (!state->is_up) {
		LOG_INF("link down");
		net_eth_carrier_off(data->iface);
		return;
	}

	if (cfg->rx_clock_reset) {
		/* the receive clock may have changed while the PLL behind it stayed locked */
		sys_write32(1, cfg->mac + CTRL_PHY_RESET);
		k_busy_wait(10);
		sys_write32(0, cfg->mac + CTRL_PHY_RESET);
	}

	cmd = sys_read32(cfg->mac + MAC_COMMAND_CONFIG) & ~CMD_SPEED_MASK;
	if (PHY_LINK_IS_SPEED_1000M(state->speed)) {
		cmd |= CMD_SPEED_1000;
	} else if (PHY_LINK_IS_SPEED_100M(state->speed)) {
		cmd |= CMD_SPEED_100;
	} else {
		cmd |= CMD_SPEED_10;
	}
	sys_write32(cmd, cfg->mac + MAC_COMMAND_CONFIG);

	LOG_INF("link up, %s Mb/s %s duplex",
		PHY_LINK_IS_SPEED_1000M(state->speed)  ? "1000"
		: PHY_LINK_IS_SPEED_100M(state->speed) ? "100"
						       : "10",
		PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");
	net_eth_carrier_on(data->iface);
}

static void eth_efinix_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;

	data->iface = iface;
	ethernet_init(iface);
	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);
	net_if_carrier_off(iface);

	if (!device_is_ready(cfg->phy_dev)) {
		LOG_ERR("PHY not ready");
		return;
	}
	eth_efinix_phy_setup(cfg);
	phy_link_callback_set(cfg->phy_dev, eth_efinix_link_changed, (void *)dev);
}

static enum ethernet_hw_caps eth_efinix_caps(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;
	enum ethernet_hw_caps caps = ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE |
				     ETHERNET_LINK_1000BASE;

	if (cfg->tx_checksum) {
		caps |= ETHERNET_HW_TX_CHKSUM_OFFLOAD;
	}
	return caps;
}

static int eth_efinix_get_config(const struct device *dev, enum ethernet_config_type type,
				 struct ethernet_config *config)
{
	ARG_UNUSED(dev);

	if (type != ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT) {
		return -ENOTSUP;
	}
	/* what the design's tx_checksum_inserter fills in, IPv4 only */
	config->chksum_support = ETHERNET_CHECKSUM_SUPPORT_IPV4_HEADER |
				 ETHERNET_CHECKSUM_SUPPORT_IPV4_ICMP |
				 ETHERNET_CHECKSUM_SUPPORT_TCP | ETHERNET_CHECKSUM_SUPPORT_UDP;
	return 0;
}

static int eth_efinix_set_config(const struct device *dev, enum ethernet_config_type type,
				 const struct ethernet_config *config)
{
	const struct eth_efinix_config *cfg = dev->config;
	struct eth_efinix_data *data = dev->data;

	if (type != ETHERNET_CONFIG_TYPE_MAC_ADDRESS) {
		return -ENOTSUP;
	}
	memcpy(data->mac_addr, config->mac_address.addr, sizeof(data->mac_addr));
	eth_efinix_set_mac_address(cfg, data->mac_addr);
	return net_if_set_link_addr(data->iface, data->mac_addr, sizeof(data->mac_addr),
				    NET_LINK_ETHERNET);
}

static const struct device *eth_efinix_get_phy(const struct device *dev)
{
	const struct eth_efinix_config *cfg = dev->config;

	return cfg->phy_dev;
}

static const struct ethernet_api eth_efinix_api = {
	.iface_api.init = eth_efinix_iface_init,
	.get_capabilities = eth_efinix_caps,
	.set_config = eth_efinix_set_config,
	.get_config = eth_efinix_get_config,
	.get_phy = eth_efinix_get_phy,
	.send = eth_efinix_send,
};

#define ETH_EFINIX_PHY_UPDATES(n)                                                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(n, phy_register_updates),                                \
		    (static const uint32_t eth_efinix_phy_updates_##n[] =                          \
			     DT_INST_PROP(n, phy_register_updates);),                              \
		    (static const uint32_t eth_efinix_phy_updates_##n[1];))

#define ETH_EFINIX_INIT(n)                                                                         \
	ETH_EFINIX_PHY_UPDATES(n)                                                                  \
	static struct dmasg_descriptor eth_efinix_rx_desc_##n[RX_COUNT];                           \
	static uint8_t eth_efinix_rx_buf_##n[RX_COUNT][BUF_SIZE] __aligned(64);                    \
	static struct dmasg_descriptor eth_efinix_tx_desc_##n[TX_COUNT + 1];                       \
	static uint8_t eth_efinix_tx_buf_##n[BUF_SIZE] __aligned(64);                              \
	static struct dmasg_descriptor eth_efinix_batch_desc_##n[2 * BATCH_COUNT + 1];             \
	static struct dmasg_descriptor eth_efinix_slot_desc_##n[4 * BATCH_COUNT + 1];              \
                                                                                                   \
	static void eth_efinix_irq_config_##n(void)                                                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, 0), DT_INST_IRQ_BY_IDX(n, 0, priority),          \
			    eth_efinix_rx_isr, DEVICE_DT_INST_GET(n), 0);                          \
		irq_enable(DT_INST_IRQN_BY_IDX(n, 0));                                             \
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(n, 1), (                                            \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, 1), DT_INST_IRQ_BY_IDX(n, 1, priority),          \
			    eth_efinix_tx_isr, DEVICE_DT_INST_GET(n), 0);                          \
		irq_enable(DT_INST_IRQN_BY_IDX(n, 1));))                                           \
	}                                                                                          \
                                                                                                   \
	static struct eth_efinix_data eth_efinix_data_##n;                                         \
                                                                                                   \
	static const struct eth_efinix_config eth_efinix_config_##n = {                            \
		.mac = DT_INST_REG_ADDR_BY_NAME(n, mac),                                           \
		.dma = DT_INST_REG_ADDR_BY_NAME(n, dma),                                           \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                          \
		.phy_updates = eth_efinix_phy_updates_##n,                                         \
		.phy_update_count = DT_INST_PROP_LEN_OR(n, phy_register_updates, 0),              \
		.rx_clock_reset = DT_INST_PROP(n, rx_clock_reset_on_link_up),                      \
		.tx_checksum = DT_INST_PROP(n, tx_checksum_offload),                               \
		.irq_config = eth_efinix_irq_config_##n,                                           \
		.mcfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(n),                                        \
		.rx_desc = eth_efinix_rx_desc_##n,                                                 \
		.rx_buf = eth_efinix_rx_buf_##n,                                                   \
		.tx_desc = eth_efinix_tx_desc_##n,                                                 \
		.tx_buf = eth_efinix_tx_buf_##n,                                                   \
		.batch_desc = eth_efinix_batch_desc_##n,                                           \
		.slot_desc = eth_efinix_slot_desc_##n,                                             \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_efinix_init, NULL, &eth_efinix_data_##n,              \
				      &eth_efinix_config_##n, CONFIG_ETH_INIT_PRIORITY,            \
				      &eth_efinix_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_EFINIX_INIT)
