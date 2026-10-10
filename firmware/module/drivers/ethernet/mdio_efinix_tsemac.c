/*
 * MDIO master of the Efinix Triple Speed Ethernet MAC.
 *
 * The MDIO registers live inside the MAC, and a MAC reset also resets them, so
 * this driver is also the one that releases the MAC and PHY resets of the example
 * design's control block. It initialises before the PHY driver probes the PHY.
 * Register map and sequence from Efinix's Linux driver (efinix_tse_mdio.c).
 */

#define DT_DRV_COMPAT efinix_tsemac_mdio

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdio_efinix_tsemac, CONFIG_MDIO_LOG_LEVEL);

#define MDIO_DIVIDER     0x00
#define MDIO_RD_WR_EN    0x04
#define MDIO_REG_PHY     0x08
#define MDIO_WR_DATA     0x0c
#define MDIO_RD_DATA     0x10
#define MDIO_STATUS      0x14

#define MDIO_RD_EN       BIT(0)
#define MDIO_WR_EN       BIT(1)
#define MDIO_STATUS_BUSY BIT(1)

#define CTRL_MAC_RST     0x00
#define CTRL_PHY_RST     0x04

#define MDIO_TIMEOUT_US  20000

struct mdio_efinix_config {
	mem_addr_t base;
	mem_addr_t ctrl;
};

struct mdio_efinix_data {
	struct k_sem lock;
};

static int mdio_efinix_wait(const struct mdio_efinix_config *cfg)
{
	for (int i = 0; i < MDIO_TIMEOUT_US; i++) {
		if (!(sys_read32(cfg->base + MDIO_STATUS) & MDIO_STATUS_BUSY)) {
			return 0;
		}
		k_busy_wait(1);
	}
	return -ETIMEDOUT;
}

static int mdio_efinix_read(const struct device *dev, uint8_t prtad, uint8_t regad,
			    uint16_t *data)
{
	const struct mdio_efinix_config *cfg = dev->config;
	struct mdio_efinix_data *dev_data = dev->data;
	int ret;

	k_sem_take(&dev_data->lock, K_FOREVER);
	ret = mdio_efinix_wait(cfg);
	if (ret == 0) {
		sys_write32((regad & 0x1f) | ((prtad & 0x1f) << 8), cfg->base + MDIO_REG_PHY);
		sys_write32(MDIO_RD_EN, cfg->base + MDIO_RD_WR_EN);
		ret = mdio_efinix_wait(cfg);
	}
	if (ret == 0) {
		*data = sys_read32(cfg->base + MDIO_RD_DATA) & 0xffff;
	}
	k_sem_give(&dev_data->lock);

	return ret;
}

static int mdio_efinix_write(const struct device *dev, uint8_t prtad, uint8_t regad,
			     uint16_t data)
{
	const struct mdio_efinix_config *cfg = dev->config;
	struct mdio_efinix_data *dev_data = dev->data;
	int ret;

	k_sem_take(&dev_data->lock, K_FOREVER);
	ret = mdio_efinix_wait(cfg);
	if (ret == 0) {
		sys_write32((regad & 0x1f) | ((prtad & 0x1f) << 8), cfg->base + MDIO_REG_PHY);
		sys_write32(data, cfg->base + MDIO_WR_DATA);
		sys_write32(MDIO_WR_EN, cfg->base + MDIO_RD_WR_EN);
		ret = mdio_efinix_wait(cfg);
	}
	k_sem_give(&dev_data->lock);

	return ret;
}

static int mdio_efinix_init(const struct device *dev)
{
	const struct mdio_efinix_config *cfg = dev->config;
	struct mdio_efinix_data *dev_data = dev->data;

	k_sem_init(&dev_data->lock, 1, 1);

	/* same sequence and timing as the Linux driver's device reset */
	sys_write32(1, cfg->ctrl + CTRL_PHY_RST);
	sys_write32(1, cfg->ctrl + CTRL_MAC_RST);
	k_msleep(100);
	sys_write32(0, cfg->ctrl + CTRL_MAC_RST);
	k_msleep(100);
	sys_write32(0, cfg->ctrl + CTRL_PHY_RST);
	k_msleep(100);

	/* the Linux driver ends up writing the largest divider, the slowest MDC */
	sys_write32(0xff, cfg->base + MDIO_DIVIDER);

	return mdio_efinix_wait(cfg);
}

static DEVICE_API(mdio, mdio_efinix_api) = {
	.read = mdio_efinix_read,
	.write = mdio_efinix_write,
};

#define MDIO_EFINIX_DEVICE(n)                                                                      \
	static const struct mdio_efinix_config mdio_efinix_config_##n = {                          \
		.base = DT_INST_REG_ADDR_BY_NAME(n, mdio),                                         \
		.ctrl = DT_INST_REG_ADDR_BY_NAME(n, ctrl),                                         \
	};                                                                                         \
	static struct mdio_efinix_data mdio_efinix_data_##n;                                       \
	DEVICE_DT_INST_DEFINE(n, mdio_efinix_init, NULL, &mdio_efinix_data_##n,                    \
			      &mdio_efinix_config_##n, POST_KERNEL, CONFIG_MDIO_INIT_PRIORITY,     \
			      &mdio_efinix_api);

DT_INST_FOREACH_STATUS_OKAY(MDIO_EFINIX_DEVICE)
