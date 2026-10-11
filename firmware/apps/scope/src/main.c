/*
 * Oscilloscope web app: serves the scope web client from flash and its /ws WebSocket on
 * port 80, takes firmware updates on /update and web client files on /files/.
 * The address comes from DHCP and is printed on the console.
 */
#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/fs/fs.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log_ctrl.h>
#include <fw_update.h>
#include <http_files.h>
#include "scope.h"

/*
 * The web client: files of the littlefs on flash, uploaded gzipped by upload_web.sh.
 * Everything the resources below do not take is looked up there.
 */
#define WEB_ROOT "/web"

/*
 * Served by this dynamic resource, not Zephyr's static filesystem one: that one (v4.4.2) sends
 * "\r\n\r\n" after each file, four bytes its Content-Length does not count, so on a kept-alive
 * connection the next response seems to start with them; Firefox then took the next file for
 * one without headers and the pages did not start. Here a file goes out chunked, gzipped as it
 * is stored, its type from its name.
 */
static struct {
	bool open;
	struct fs_file_t file;
	char path[sizeof(WEB_ROOT) + 72];
	uint8_t chunk[4096];
	struct http_header headers[2];
} web;

static const char *content_type_of(const char *name)
{
	static const struct {
		const char *ext, *type;
	} types[] = {
		{".html", "text/html"}, {".js", "text/javascript"}, {".css", "text/css"},
		{".json", "application/json"}, {".svg", "image/svg+xml"}, {".png", "image/png"},
		{".ico", "image/x-icon"}, {".txt", "text/plain"},
	};
	size_t n = strlen(name);

	for (size_t i = 0; i < ARRAY_SIZE(types); i++) {
		size_t e = strlen(types[i].ext);

		if (n >= e && strcmp(name + n - e, types[i].ext) == 0) {
			return types[i].type;
		}
	}
	return "application/octet-stream";
}

static void web_close(void)
{
	if (web.open) {
		fs_close(&web.file);
		web.open = false;
	}
}

static int web_cb(struct http_client_ctx *client, enum http_transaction_status status,
		  const struct http_request_ctx *request_ctx, struct http_response_ctx *response,
		  void *user_data)
{
	ARG_UNUSED(request_ctx);
	ARG_UNUSED(user_data);

	if (status == HTTP_SERVER_TRANSACTION_ABORTED || status == HTTP_SERVER_TRANSACTION_COMPLETE) {
		web_close();
		return 0;
	}
	if (!web.open) {
		/* the request: the URL's path, "/" the index, any "?..." left out */
		char name[64];
		const char *url = (const char *)client->url_buffer;
		size_t len = strcspn(url, "?#");

		if (len <= 1) {
			strcpy(name, "index.html");
		} else if (len - 1 < sizeof(name) && strstr(url, "..") == NULL) {
			memcpy(name, url + 1, len - 1);
			name[len - 1] = '\0';
		} else {
			name[0] = '\0';
		}
		snprintf(web.path, sizeof(web.path), WEB_ROOT "/%s.gz", name);
		fs_file_t_init(&web.file);
		if (name[0] == '\0' || strchr(name, '/') != NULL ||
		    fs_open(&web.file, web.path, FS_O_READ) != 0) {
			static const char missing[] = "not found\n";

			response->status = HTTP_404_NOT_FOUND;
			response->body = (const uint8_t *)missing;
			response->body_len = sizeof(missing) - 1;
			response->final_chunk = true;
			return 0;
		}
		web.open = true;
		web.headers[0] = (struct http_header){"Content-Type", content_type_of(name)};
		web.headers[1] = (struct http_header){"Content-Encoding", "gzip"};
		response->headers = web.headers;
		response->header_count = ARRAY_SIZE(web.headers);
		response->status = HTTP_200_OK;
	}

	ssize_t n = fs_read(&web.file, web.chunk, sizeof(web.chunk));

	if (n < 0) {
		n = 0;
	}
	response->body = web.chunk;
	response->body_len = n;
	if (n < (ssize_t)sizeof(web.chunk)) {
		response->final_chunk = true;
		web_close();
	}
	return 0;
}

static struct http_resource_detail_dynamic web_files = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.cb = web_cb,
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
		    NULL, NULL);
HTTP_RESOURCE_DEFINE(ws_resource, scope_service, "/ws", &ws);
HTTP_RESOURCE_DEFINE(update_resource, scope_service, "/update", &fw_update_resource);
HTTP_RESOURCE_DEFINE(files_resource, scope_service, "/files/*", &http_files_resource);
/* the web client: everything the others do not take. A resource, not the service's fallback:
 * the server releases a dynamic resource whose transfer broke off (a reset connection) only
 * for its resources, a fallback stayed held and answered every later request 409. Its name
 * sorts last, so the other wildcards (the files resource) are matched first. */
HTTP_RESOURCE_DEFINE(zweb_resource, scope_service, "/*", &web_files.common);
/* and the bare address, which the wildcard does not match: the index */
HTTP_RESOURCE_DEFINE(zweb_root_resource, scope_service, "/", &web_files.common);

/*
 * The network's watchdog: no receive packet of the stack free for NET_STUCK_S seconds and the
 * network is dead (every frame dropped, the board unreachable), so the firmware starts again
 * (the FPGA brings a run down without its kicks; the STOP button works throughout). The
 * restarts (and a crash's, below) are counted in memory a reboot keeps, shown on the face panel.
 */
#define NET_STUCK_S 10
#define NET_RESETS_MAGIC 0x4e455452u     /* "NETR" */
static __noinit uint32_t net_resets_magic;
static __noinit uint32_t net_resets;

uint32_t scope_net_resets(void)
{
	return net_resets_magic == NET_RESETS_MAGIC ? net_resets : 0;
}

static void net_watchdog(struct k_work *work)
{
	static int stuck_s;
	struct k_mem_slab *rx, *tx;
	struct net_buf_pool *rx_data, *tx_data;

	net_pkt_get_info(&rx, &tx, &rx_data, &tx_data);
	stuck_s = k_mem_slab_num_free_get(rx) == 0 ? stuck_s + 1 : 0;
	if (stuck_s >= NET_STUCK_S) {
		if (net_resets_magic != NET_RESETS_MAGIC) {
			net_resets_magic = NET_RESETS_MAGIC;
			net_resets = 0;
		}
		net_resets++;
		LOG_PANIC();
		printk("network stuck: no receive packet free for %d s, restart %u\n", NET_STUCK_S,
		       net_resets);
		sys_reboot(SYS_REBOOT_COLD);
	}
	k_work_reschedule(k_work_delayable_from_work(work), K_SECONDS(1));
}

static K_WORK_DELAYABLE_DEFINE(net_watchdog_work, net_watchdog);

/*
 * A crash (its report on the console already) starts the firmware again rather than halting
 * it, which left the board unreachable until a power cycle; counted with the network's
 * restarts on the face panel.
 */
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	ARG_UNUSED(esf);

	if (net_resets_magic != NET_RESETS_MAGIC) {
		net_resets_magic = NET_RESETS_MAGIC;
		net_resets = 0;
	}
	net_resets++;
	LOG_PANIC();
	printk("fatal error %u: restart %u\n", reason, net_resets);
	sys_reboot(SYS_REBOOT_COLD);
	CODE_UNREACHABLE;
}

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
	k_work_reschedule(&net_watchdog_work, K_SECONDS(1));
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
