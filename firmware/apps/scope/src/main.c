/*
 * Oscilloscope web app: serves the scope web client from flash and its /ws WebSocket on
 * port 80, takes firmware updates on /update and web client files on /files/.
 * The address comes from DHCP and is printed on the console.
 */
#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/fs/fs.h>
#include <zephyr/net/net_if.h>
#include <fw_update.h>
#include <http_files.h>
#include "scope.h"

/*
 * The web client: files of the littlefs on flash, uploaded gzipped by upload_web.sh.
 * Everything the resources below do not take is looked up there.
 */
#define WEB_ROOT "/web"

static struct http_resource_detail_static_fs web_files = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_STATIC_FS,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.fs_path = WEB_ROOT,
};

static int ws_setup(int ws_socket, struct http_request_ctx *request_ctx, void *user_data)
{
	ARG_UNUSED(request_ctx);
	ARG_UNUSED(user_data);

	return scope_client_add(ws_socket);
}

static uint8_t ws_buffer[1024];

static struct http_resource_detail_websocket ws = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_WEBSOCKET,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.cb = ws_setup,
	.data_buffer = ws_buffer,
	.data_buffer_len = sizeof(ws_buffer),
};

static uint16_t http_port = 80;

HTTP_SERVICE_DEFINE(scope_service, NULL, &http_port, CONFIG_HTTP_SERVER_MAX_CLIENTS, 4, NULL,
		    &web_files.common, NULL);
HTTP_RESOURCE_DEFINE(ws_resource, scope_service, "/ws", &ws);
HTTP_RESOURCE_DEFINE(update_resource, scope_service, "/update", &fw_update_resource);
HTTP_RESOURCE_DEFINE(files_resource, scope_service, "/files/*", &http_files_resource);

int main(void)
{
	struct fs_dirent entry;

	printk("scope on %s, open http://<address>/ in a browser\n", CONFIG_BOARD_TARGET);
	capture_init(); /* source names for the clients, before they can connect */
	{
		struct scope_config c;

		scope_get_config(&c);
		capture_set_adc_masks(c.adc_masks); /* the FPGA starts with all eight */
		capture_set_adc_settle(c.adc_settle_ns);
		capture_set_sdm(c.sdm_log2, c.sdm_phase);
		capture_set_dhb_control(false, c.dhb_kp, c.dhb_ki, c.dhb_limit_high, c.dhb_limit_low);
		capture_set_dhb_protection(c.dhb_overcurrent, c.dhb_overvoltage, c.dhb_overvoltage_readings);
		capture_set_dhb_filter(c.dhb_filter);
		capture_set_gate_power(c.gate_power); /* off after a restart */
	}
	scope_settings_load();
	capture_scaler_init();
	if (fs_stat(WEB_ROOT "/index.html.gz", &entry) != 0) {
		printk("no web client in flash yet: apps/scope/upload_web.sh <address>\n");
	}
	http_server_start();
	panel_start();
#ifdef CONFIG_SCOPE_STREAM
	/* the stream answers on the board's address, which a DHCP lease may bring late */
	for (int s = 0; s < 30; s++) {
		if (net_if_ipv4_get_global_addr(net_if_get_default(), NET_ADDR_PREFERRED) != NULL) {
			scope_stream_init();
			break;
		}
		k_sleep(K_SECONDS(1));
	}
#endif
	return 0;
}
