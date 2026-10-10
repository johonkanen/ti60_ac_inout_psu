/*
 * SPI master of the Efinix Sapphire SoC (SpinalHDL SpiXdrMaster), polled, single data line.
 *
 * Every byte is a command word in the command FIFO: the byte to send, whether to send it
 * and whether to return the received byte through the response FIFO. Chip selects are
 * commands too, so a whole transfer is queued in order and the driver only keeps the
 * command FIFO fed and the response FIFO drained. Bytes whose response is not wanted are
 * sent as write-only commands and cost no response read.
 */
#define DT_DRV_COMPAT efinix_sapphire_spi

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>

LOG_MODULE_REGISTER(spi_efinix_sapphire, CONFIG_SPI_LOG_LEVEL);

#define SPI_DATA        0x00 /* write: command, read: response */
#define SPI_BUFFER      0x04 /* 15:0 command FIFO space, 31:16 response FIFO occupancy */
#define SPI_CONFIG      0x08
#define SPI_CLK_DIVIDER 0x20
#define SPI_SS_SETUP    0x24
#define SPI_SS_HOLD     0x28
#define SPI_SS_DISABLE  0x2c

#define CMD_WRITE     BIT(8)
#define CMD_READ      BIT(9)
#define CMD_SS        BIT(11)
#define CMD_SS_ENABLE BIT(7)
#define RSP_EMPTY     BIT(31) /* set when the response FIFO had nothing */

#define CONFIG_CPOL BIT(0)
#define CONFIG_CPHA BIT(1)

/* chip select timing in SPI block clocks, as Efinix's spiFlash_init */
#define SS_SETUP_CLOCKS   5
#define SS_HOLD_CLOCKS    2
#define SS_DISABLE_CLOCKS 7

struct spi_sapphire_config {
	uintptr_t base;
	uint32_t clock_frequency;
	uint16_t cmd_fifo_depth;
	uint16_t rsp_fifo_depth;
};

struct spi_sapphire_data {
	struct k_mutex lock;
	const struct spi_config *config; /* last applied */
};

/* byte cursor over a spi_buf_set; a NULL buffer is skipped (rx) or sends zeros (tx) */
struct cursor {
	const struct spi_buf *buf;
	size_t count;
	size_t offset;
};

static void cursor_init(struct cursor *c, const struct spi_buf_set *set)
{
	c->buf = set ? set->buffers : NULL;
	c->count = set ? set->count : 0;
	c->offset = 0;
}

static size_t cursor_total(const struct spi_buf_set *set)
{
	size_t total = 0;

	for (size_t i = 0; set && i < set->count; i++) {
		total += set->buffers[i].len;
	}
	return total;
}

/* the next byte's address, NULL when its buffer is NULL or the set has ended */
static uint8_t *cursor_next(struct cursor *c)
{
	uint8_t *p;

	while (c->count && c->offset >= c->buf->len) {
		c->buf++;
		c->count--;
		c->offset = 0;
	}
	if (!c->count) {
		return NULL;
	}
	p = c->buf->buf ? (uint8_t *)c->buf->buf + c->offset : NULL;
	c->offset++;
	return p;
}

static inline uint32_t cmd_space(const struct spi_sapphire_config *cfg)
{
	return sys_read32(cfg->base + SPI_BUFFER) & 0xffff;
}

static void command(const struct spi_sapphire_config *cfg, uint32_t value)
{
	while (cmd_space(cfg) == 0) {
	}
	sys_write32(value, cfg->base + SPI_DATA);
}

static void wait_idle(const struct spi_sapphire_config *cfg)
{
	while (cmd_space(cfg) != cfg->cmd_fifo_depth) {
	}
	k_busy_wait(1); /* the last command leaves the FIFO before it has finished */
}

static int configure(const struct device *dev, const struct spi_config *config)
{
	const struct spi_sapphire_config *cfg = dev->config;
	struct spi_sapphire_data *data = dev->data;
	uint32_t mode = 0;
	uint32_t divider;

	if (data->config == config) {
		return 0;
	}
	if (SPI_OP_MODE_GET(config->operation) != SPI_OP_MODE_MASTER ||
	    (config->operation & (SPI_MODE_LOOP | SPI_TRANSFER_LSB | SPI_HALF_DUPLEX)) ||
	    SPI_WORD_SIZE_GET(config->operation) != 8) {
		LOG_ERR("unsupported operation 0x%x", config->operation);
		return -ENOTSUP;
	}
	if (IS_ENABLED(CONFIG_SPI_EXTENDED_MODES) &&
	    (config->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("only single line transfers");
		return -ENOTSUP;
	}
	if (spi_cs_is_gpio(config)) {
		LOG_ERR("GPIO chip selects not supported");
		return -ENOTSUP;
	}
	if (config->frequency == 0) {
		return -EINVAL;
	}

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		mode |= CONFIG_CPOL;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		mode |= CONFIG_CPHA;
	}
	divider = DIV_ROUND_UP(cfg->clock_frequency, 2 * config->frequency);
	divider = divider ? divider - 1 : 0;

	wait_idle(cfg);
	sys_write32(mode, cfg->base + SPI_CONFIG);
	sys_write32(divider, cfg->base + SPI_CLK_DIVIDER);
	sys_write32(SS_SETUP_CLOCKS, cfg->base + SPI_SS_SETUP);
	sys_write32(SS_HOLD_CLOCKS, cfg->base + SPI_SS_HOLD);
	sys_write32(SS_DISABLE_CLOCKS, cfg->base + SPI_SS_DISABLE);
	data->config = config;

	LOG_DBG("%u Hz asked, SCLK %u Hz, mode %u", config->frequency,
		cfg->clock_frequency / ((divider + 1) * 2), SPI_MODE_GET(config->operation));
	return 0;
}

static int spi_sapphire_transceive(const struct device *dev, const struct spi_config *config,
				   const struct spi_buf_set *tx_bufs,
				   const struct spi_buf_set *rx_bufs)
{
	const struct spi_sapphire_config *cfg = dev->config;
	struct spi_sapphire_data *data = dev->data;
	size_t tx_total = cursor_total(tx_bufs);
	size_t rx_total = cursor_total(rx_bufs);
	size_t total = MAX(tx_total, rx_total);
	size_t sent = 0, received = 0;
	struct cursor tx, rx;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = configure(dev, config);
	if (ret) {
		k_mutex_unlock(&data->lock);
		return ret;
	}

	cursor_init(&tx, tx_bufs);
	cursor_init(&rx, rx_bufs);
	command(cfg, CMD_SS | CMD_SS_ENABLE | config->slave);

	/*
	 * Responses only for the first rx_total bytes, at most a response FIFO full in
	 * flight. The CPU's register accesses, not SCLK, set the pace, so each round queues
	 * as many commands as fit and then takes exactly the responses that are waiting.
	 */
	while (sent < total || received < MIN(sent, rx_total)) {
		uint32_t buffer = sys_read32(cfg->base + SPI_BUFFER);
		uint32_t space = buffer & 0xffff;
		uint32_t waiting = buffer >> 16;

		while (space && sent < total &&
		       (sent >= rx_total || sent - received < cfg->rsp_fifo_depth)) {
			uint8_t *p = cursor_next(&tx);
			uint32_t word = (p ? *p : 0) | CMD_WRITE;

			if (sent < rx_total) {
				word |= CMD_READ;
			}
			sys_write32(word, cfg->base + SPI_DATA);
			space--;
			sent++;
		}
		for (; waiting; waiting--) {
			uint8_t *p = cursor_next(&rx);
			uint8_t value = (uint8_t)sys_read32(cfg->base + SPI_DATA);

			if (p) {
				*p = value;
			}
			received++;
		}
	}

	if (!(config->operation & SPI_HOLD_ON_CS)) {
		command(cfg, CMD_SS | config->slave);
	}
	wait_idle(cfg);
	k_mutex_unlock(&data->lock);
	return 0;
}

static int spi_sapphire_release(const struct device *dev, const struct spi_config *config)
{
	const struct spi_sapphire_config *cfg = dev->config;
	struct spi_sapphire_data *data = dev->data;

	k_mutex_lock(&data->lock, K_FOREVER);
	command(cfg, CMD_SS | config->slave);
	wait_idle(cfg);
	k_mutex_unlock(&data->lock);
	return 0;
}

static DEVICE_API(spi, spi_sapphire_api) = {
	.transceive = spi_sapphire_transceive,
	.release = spi_sapphire_release,
};

static int spi_sapphire_init(const struct device *dev)
{
	const struct spi_sapphire_config *cfg = dev->config;
	struct spi_sapphire_data *data = dev->data;

	k_mutex_init(&data->lock);
	wait_idle(cfg);
	/* drop responses left over by the boot loader */
	while (!(sys_read32(cfg->base + SPI_DATA) & RSP_EMPTY)) {
	}
	return 0;
}

#define SPI_SAPPHIRE_INIT(n)                                                                       \
	static const struct spi_sapphire_config spi_sapphire_config_##n = {                        \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clock_frequency = DT_INST_PROP(n, clock_frequency),                               \
		.cmd_fifo_depth = DT_INST_PROP(n, cmd_fifo_depth),                                 \
		.rsp_fifo_depth = DT_INST_PROP(n, rsp_fifo_depth),                                 \
	};                                                                                         \
	static struct spi_sapphire_data spi_sapphire_data_##n;                                     \
	SPI_DEVICE_DT_INST_DEFINE(n, spi_sapphire_init, NULL, &spi_sapphire_data_##n,              \
				  &spi_sapphire_config_##n, POST_KERNEL,                           \
				  CONFIG_SPI_INIT_PRIORITY, &spi_sapphire_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_SAPPHIRE_INIT)
