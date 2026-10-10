/*
 * Acquisition and the browser connection, the Zephyr counterpart of scope_server.py with
 * the same WebSocket protocol:
 *   browser -> scope  JSON {"type": "config", "config": {...}}, run, single, stop, force
 *   scope -> browser  JSON hello (config, source names, clock) and status messages, and
 *                     binary captures: uint32 header length, JSON header, int16 samples
 *                     little endian, sample-major.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/websocket.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "scope.h"

LOG_MODULE_REGISTER(scope, LOG_LEVEL_INF);

#define MAX_CLIENTS        4  /* the scope and data collection pages, a script */
#define CLIENT_STACK_SIZE  4096
#define SCOPE_STACK_SIZE   4096
#define COMMAND_MAX        16384 /* a waveform's 512 points as JSON, ~6 kB */
#define HEADER_MAX         512
#define SEND_TIMEOUT_MS    2000
#define DISPLAY_INTERVAL_MS 33 /* a browser redraws at most this often anyway */

static const char *const mode_names[] = {"auto", "normal", "free"};

#ifdef CONFIG_SCOPE_STREAM
#define stream_active scope_stream_active
#define stream_lost   scope_stream_lost
#else
static bool stream_active(void)
{
	return false;
}

static uint32_t stream_lost(void)
{
	return 0;
}
#endif

static struct scope_config config = {
	.divider = 124, /* 1 MS/s */
	.length = 4096,
	.pre_percent = 50,
	.trigger_channel = 0,
	.trigger_edge = SCOPE_EDGE_RISING,
	.trigger_level = 0,
	.mode = SCOPE_MODE_AUTO,
	.sources = {0, 1, 2, 3, 4, 5, 6, 7},
	.generator_hz = 10000.0,
	/* the named inputs only: sec 0 vdc_out, 1 vac_out, 2 vfilt_out; pri 0 vac_in,
	 * 1 vdc_in, 3 vfilt_in */
	.adc_masks = {0x07, 0x0b},
	.adc_settle_ns = {3000, 3000}, /* about 5 counts left of a neighbour, 107 kS/s per input of three */
	.sdm_log2 = 6, /* 312.5 kS/s at 20 MHz */
	.sdm_phase = 1,
	.gate_power = 0, /* off until asked */
	/* the DHB's voltage control: gains of a command of +-1.0 (a quarter period of phase,
	 * 333.25 counts) per volt; 40 and 0.4 counts a volt */
	.dhb_kp = 40.0 / 333.25,
	.dhb_ki = 0.4 / 333.25,
	.dhb_limit_high = 1.0,
	.dhb_limit_low = -1.0,
	.dhb_control = false, /* off until asked */
	/* the trips: 5 A, and for the 100 V bench 120 V; the hardware is rated for 450 V */
	.dhb_overcurrent = 15.0,
	.dhb_overvoltage = 120.0,
	/* the readings carry +-30 V of switching noise: 8 in a row over the limit trip, and the
	 * controller takes the DC links filtered over 2^6 readings (~0.6 ms) */
	.dhb_overvoltage_readings = 8,
	.dhb_filter = 6,
	/* the observer: the output filter as fitted to a measured step (output_filter_fit_tb),
	 * 3 kHz for the current's error, 100 Hz for the voltage's (its readings are noisy) */
	.observer = true,
	.observer_l = 0.83e-3,
	.observer_c = 18.0e-6,
	.observer_r = 0.51,
	.observer_current_hz = 1000.0, /* 3 kHz oscillated in the closed loop: L1 below the model's */
	.observer_voltage_hz = 100.0, /* 30 Hz drifted at 40 kHz and misses load steps */
	/* the control as tuned in output_voltage_control_tb */
	.control = true,
	.control_current_hz = 3000.0,
	/* the voltage PI's integral on the estimate, its zero low; the reference through the
	 * prefilter, its slope fed forward (output_voltage_control_tb's the_prefilter) */
	.control_voltage_hz = 1000.0,
	.control_b = 1.0,
	.control_voltage_zero = 0.1,
	.control_prefilter_hz = 1000.0,
	.control_feedforward = 1.0,
	.control_du_hz = 20.0,
	.control_i_max = 5.0,
	.control_vl_max = 60.0,
	.control_u_max = 0.95 * 94.0,
};

/* the ADCs' conversion rates, measured between status messages */
static struct {
	uint32_t conversions[SCOPE_ADCS];
	uint32_t sdm_results;
	int64_t at;
	uint32_t rates[SCOPE_ADCS];
	uint32_t sdm_rate;
} adc;

static struct {
	uint32_t config_version; /* bumped on every config command */
	bool running;
	bool single;
	bool force;
	uint32_t captures;
	bool forced;
	const char *state;
} status = {.state = "stopped"};

/* config and status are shared by the client threads and the acquisition thread */
static K_MUTEX_DEFINE(scope_lock);

static struct client {
	int sock;
	struct k_thread thread;
	char *command; /* commands[] */
} clients[MAX_CLIENTS] = {[0 ... MAX_CLIENTS - 1] = {.sock = -1}};

/* the clients' incoming commands, apart from clients[] so they are not in the image's data */
static char commands[MAX_CLIENTS][COMMAND_MAX];

static K_THREAD_STACK_ARRAY_DEFINE(client_stacks, MAX_CLIENTS, CLIENT_STACK_SIZE);
/* one message to a client at a time */
static K_MUTEX_DEFINE(send_lock);

/*
 * Capture messages: the header length, the JSON header padded with spaces to HEADER_MAX, the
 * samples, which capture_run writes in place. Two take turns: one is filled while the other
 * holds the last capture sent, for a page that connects later (last and last_size under
 * send_lock).
 */
#define MESSAGE_SIZE (4 + HEADER_MAX + SCOPE_MAX_LENGTH * SCOPE_CHANNELS * sizeof(int16_t))
static uint8_t messages[2][MESSAGE_SIZE] __aligned(4);
static int filling;
static int last = -1;
static size_t last_size;
/* how long the last capture took to copy out and to send, for the status message */
static uint32_t copy_us, send_us;

/* ------------------------------------------------------------------ JSON in */

static const char *json_value(const char *message, const char *key)
{
	char pattern[32];
	const char *p;

	snprintf(pattern, sizeof(pattern), "\"%s\"", key);
	p = strstr(message, pattern);
	if (p == NULL) {
		return NULL;
	}
	p = strchr(p + strlen(pattern), ':');
	if (p == NULL) {
		return NULL;
	}
	p++;
	while (*p == ' ') {
		p++;
	}
	return p;
}

static bool json_number(const char *message, const char *key, double *value)
{
	const char *p = json_value(message, key);
	char *end;

	if (p == NULL) {
		return false;
	}
	*value = strtod(p, &end);
	return end != p;
}

static bool json_string_is(const char *message, const char *key, const char *expected)
{
	const char *p = json_value(message, key);
	size_t n = strlen(expected);

	return p != NULL && p[0] == '"' && strncmp(p + 1, expected, n) == 0 && p[n + 1] == '"';
}

/* the waveform loaded last, its points (the player's table holds capture_waveform_capacity()) */
static uint32_t waveform_loaded;

/* a waveform_load message's points, [[volts, periods], ...], into the player's table, with
 * its passes; the points taken, as many as the table holds */
static uint32_t parse_waveform(const char *message, double passes)
{
	static float volts[512];
	static uint32_t periods[512];
	const uint32_t capacity = MIN(capture_waveform_capacity(), ARRAY_SIZE(volts));
	const char *p = json_value(message, "points");
	uint32_t count = 0;
	char *end;

	if (p == NULL || *p != '[' || capacity == 0) {
		return 0;
	}
	p++;
	while (count < capacity) {
		while (*p == ' ' || *p == ',') {
			p++;
		}
		if (*p != '[') {
			break;
		}
		double v = strtod(p + 1, &end);

		if (end == p + 1) {
			break;
		}
		p = end;
		while (*p == ' ' || *p == ',') {
			p++;
		}
		double n = strtod(p, &end);

		if (end == p) {
			break;
		}
		p = end;
		while (*p == ' ') {
			p++;
		}
		if (*p != ']') {
			break;
		}
		p++;
		volts[count] = (float)CLAMP(v, -450.0, 450.0);
		periods[count] = (uint32_t)CLAMP(n, 1.0, 4.0e9);
		count++;
	}
	if (count > 0) {
		capture_waveform_load(volts, periods, count, (uint32_t)passes);
	}
	return count;
}

/* gate_power and dhb_control only from a page: after a restart the gate driver supplies and
 * the DHB's voltage control stay off */
static void apply_config(const char *message, bool with_gate_power)
{
	const char *p;
	double v;

	if (json_number(message, "divider", &v)) {
		config.divider = CLAMP((uint32_t)v, 0, 1000000);
	}
	if (json_number(message, "length", &v)) {
		config.length = CLAMP((uint32_t)v, 16, SCOPE_MAX_LENGTH);
	}
	if (json_number(message, "pre_percent", &v)) {
		config.pre_percent = CLAMP((uint32_t)v, 0, 100);
	}
	if (json_number(message, "trigger_channel", &v)) {
		config.trigger_channel = CLAMP((uint32_t)v, 0, SCOPE_CHANNELS - 1);
	}
	if (json_number(message, "trigger_edge", &v)) {
		config.trigger_edge = CLAMP((int)v, SCOPE_EDGE_RISING, SCOPE_EDGE_EITHER);
	}
	if (json_number(message, "trigger_level", &v)) {
		config.trigger_level = CLAMP((int32_t)v, -32768, 32767);
	}
	if (json_number(message, "generator_hz", &v)) {
		config.generator_hz = MAX(v, 0.0);
	}
	for (int m = 0; m < ARRAY_SIZE(mode_names); m++) {
		if (json_string_is(message, "mode", mode_names[m])) {
			config.mode = m;
		}
	}
	p = json_value(message, "sources");
	if (p != NULL && *p == '[') {
		p++;
		for (int c = 0; c < SCOPE_CHANNELS; c++) {
			char *end;
			long s = strtol(p, &end, 10);

			if (end == p) {
				break;
			}
			config.sources[c] = CLAMP(s, 0, SCOPE_SOURCES - 1);
			p = end;
			while (*p == ',' || *p == ' ') {
				p++;
			}
		}
	}
	p = json_value(message, "adc_masks");
	if (p != NULL && *p == '[') {
		p++;
		for (int a = 0; a < SCOPE_ADCS; a++) {
			char *end;
			long m = strtol(p, &end, 10);

			if (end == p) {
				break;
			}
			config.adc_masks[a] = CLAMP(m, 1, 255); /* at least one position */
			p = end;
			while (*p == ',' || *p == ' ') {
				p++;
			}
		}
	}
	p = json_value(message, "adc_settle_ns");
	if (p != NULL && *p == '[') {
		p++;
		for (int a = 0; a < SCOPE_ADCS; a++) {
			char *end;
			long ns = strtol(p, &end, 10);

			if (end == p) {
				break;
			}
			config.adc_settle_ns[a] = CLAMP(ns, 0, 100000);
			p = end;
			while (*p == ',' || *p == ' ') {
				p++;
			}
		}
	}
	if (json_number(message, "sdm_log2", &v)) {
		config.sdm_log2 = CLAMP((uint32_t)v, 5, 8);
	}
	if (json_number(message, "sdm_phase", &v)) {
		config.sdm_phase = CLAMP((uint32_t)v, 0, 255);
	}
	if (json_number(message, "dhb_kp", &v)) {
		config.dhb_kp = v;
	}
	if (json_number(message, "dhb_ki", &v)) {
		config.dhb_ki = v;
	}
	if (json_number(message, "dhb_limit_high", &v)) {
		config.dhb_limit_high = CLAMP(v, -2.0, 2.0);
	}
	if (json_number(message, "dhb_limit_low", &v)) {
		config.dhb_limit_low = CLAMP(v, -2.0, 2.0);
	}
	if (json_number(message, "dhb_overcurrent", &v)) {
		config.dhb_overcurrent = CLAMP(v, 0.0, 30.0);
	}
	if (json_number(message, "dhb_overvoltage", &v)) {
		config.dhb_overvoltage = CLAMP(v, 0.0, 450.0);
	}
	if (json_number(message, "dhb_overvoltage_readings", &v)) {
		config.dhb_overvoltage_readings = CLAMP((uint32_t)v, 1, 255);
	}
	if (json_number(message, "dhb_filter", &v)) {
		config.dhb_filter = CLAMP((uint32_t)v, 0, 12);
	}
	if (json_number(message, "observer", &v)) {
		config.observer = v != 0;
	}
	if (json_number(message, "observer_l", &v)) {
		config.observer_l = CLAMP(v, 1.0e-5, 0.1);
	}
	if (json_number(message, "observer_c", &v)) {
		config.observer_c = CLAMP(v, 1.0e-7, 1.0e-2);
	}
	if (json_number(message, "observer_r", &v)) {
		config.observer_r = CLAMP(v, 0.0, 100.0);
	}
	if (json_number(message, "observer_current_hz", &v)) {
		config.observer_current_hz = CLAMP(v, 10.0, 8000.0);
	}
	if (json_number(message, "observer_voltage_hz", &v)) {
		config.observer_voltage_hz = CLAMP(v, 1.0, 4000.0);
	}
	if (json_number(message, "control", &v)) {
		config.control = v != 0;
	}
	if (json_number(message, "control_current_hz", &v)) {
		config.control_current_hz = CLAMP(v, 10.0, 6000.0);
	}
	if (json_number(message, "control_voltage_hz", &v)) {
		config.control_voltage_hz = CLAMP(v, 1.0, 3000.0);
	}
	if (json_number(message, "control_b", &v)) {
		config.control_b = CLAMP(v, 0.0, 1.0);
	}
	if (json_number(message, "control_voltage_zero", &v)) {
		config.control_voltage_zero = CLAMP(v, 0.0, 1.0);
	}
	if (json_number(message, "control_prefilter_hz", &v)) {
		config.control_prefilter_hz = CLAMP(v, 0.0, 10000.0);
	}
	if (json_number(message, "control_feedforward", &v)) {
		config.control_feedforward = CLAMP(v, 0.0, 2.0);
	}
	if (json_number(message, "control_du_hz", &v)) {
		config.control_du_hz = CLAMP(v, 0.1, 1000.0);
	}
	if (json_number(message, "control_i_max", &v)) {
		config.control_i_max = CLAMP(v, 0.0, 14.0);
	}
	if (json_number(message, "control_vl_max", &v)) {
		config.control_vl_max = CLAMP(v, 0.0, 450.0);
	}
	if (json_number(message, "control_u_max", &v)) {
		config.control_u_max = CLAMP(v, 0.0, 450.0);
	}
	if (with_gate_power && json_number(message, "dhb_control", &v)) {
		config.dhb_control = v != 0;
	}
	if (with_gate_power && json_number(message, "gate_power", &v)) {
		config.gate_power = (uint32_t)v & 0x3f;
	}
	capture_set_adc_masks(config.adc_masks);
	capture_set_adc_settle(config.adc_settle_ns);
	capture_set_sdm(config.sdm_log2, config.sdm_phase);
	capture_set_gate_power(config.gate_power);
	capture_set_dhb_control(config.dhb_control, config.dhb_kp, config.dhb_ki,
				config.dhb_limit_high, config.dhb_limit_low);
	capture_set_dhb_protection(config.dhb_overcurrent, config.dhb_overvoltage,
				   config.dhb_overvoltage_readings);
	capture_set_dhb_filter(config.dhb_filter);
	capture_set_observer(config.observer, config.observer_l, config.observer_c, config.observer_r,
			     config.observer_current_hz, config.observer_voltage_hz);
	capture_set_control(config.control, config.observer_l, config.observer_c, config.observer_r,
			    config.control_current_hz, config.control_voltage_hz, config.control_b,
			    config.control_voltage_zero, config.control_du_hz, config.control_i_max,
			    config.control_vl_max, config.control_u_max, config.control_prefilter_hz,
			    config.control_feedforward);
}

/* ------------------------------------------------------------------ JSON out */

static int print_sources(char *buf, size_t size)
{
	int n = snprintf(buf, size, "[");

	for (int c = 0; c < SCOPE_CHANNELS; c++) {
		n += snprintf(buf + n, size - n, "%s%u", c ? "," : "", config.sources[c]);
	}
	return n + snprintf(buf + n, size - n, "]");
}

static int print_config(char *buf, size_t size)
{
	int n = snprintf(buf, size,
			 "{\"divider\":%u,\"length\":%u,\"pre_percent\":%u,\"trigger_channel\":%u,"
			 "\"trigger_edge\":%d,\"trigger_level\":%d,\"mode\":\"%s\","
			 "\"generator_hz\":%.3f,\"sources\":",
			 config.divider, config.length, config.pre_percent, config.trigger_channel,
			 config.trigger_edge, config.trigger_level, mode_names[config.mode],
			 config.generator_hz);

	n += print_sources(buf + n, size - n);
	return n + snprintf(buf + n, size - n,
			    ",\"adc_masks\":[%u,%u],\"adc_settle_ns\":[%u,%u],\"sdm_log2\":%u,"
			    "\"sdm_phase\":%u,\"gate_power\":%u,\"dhb_kp\":%.6f,\"dhb_ki\":%.8f,"
			    "\"dhb_limit_high\":%.4f,\"dhb_limit_low\":%.4f,\"dhb_control\":%u,"
			    "\"dhb_overcurrent\":%.2f,\"dhb_overvoltage\":%.1f,"
			    "\"dhb_overvoltage_readings\":%u,\"dhb_filter\":%u,\"observer\":%u,"
			    "\"observer_l\":%.4e,\"observer_c\":%.4e,\"observer_r\":%.4f,"
			    "\"observer_current_hz\":%.1f,\"observer_voltage_hz\":%.1f,\"control\":%u,"
			    "\"control_current_hz\":%.1f,\"control_voltage_hz\":%.1f,\"control_b\":%.3f,"
			    "\"control_voltage_zero\":%.3f,\"control_prefilter_hz\":%.1f,"
			    "\"control_feedforward\":%.3f,"
			    "\"control_du_hz\":%.2f,\"control_i_max\":%.2f,\"control_vl_max\":%.1f,"
			    "\"control_u_max\":%.1f}",
			    config.adc_masks[0], config.adc_masks[1], config.adc_settle_ns[0],
			    config.adc_settle_ns[1], config.sdm_log2, config.sdm_phase, config.gate_power,
			    config.dhb_kp, config.dhb_ki, config.dhb_limit_high, config.dhb_limit_low,
			    config.dhb_control, config.dhb_overcurrent, config.dhb_overvoltage,
			    config.dhb_overvoltage_readings, config.dhb_filter, config.observer,
			    config.observer_l, config.observer_c, config.observer_r,
			    config.observer_current_hz, config.observer_voltage_hz, config.control,
			    config.control_current_hz, config.control_voltage_hz, config.control_b,
			    config.control_voltage_zero, config.control_prefilter_hz,
			    config.control_feedforward, config.control_du_hz, config.control_i_max,
			    config.control_vl_max, config.control_u_max);
}

static int send_text(int sock, const char *text)
{
	int ret;

	k_mutex_lock(&send_lock, K_FOREVER);
	ret = websocket_send_msg(sock, text, strlen(text), WEBSOCKET_OPCODE_DATA_TEXT, false, true,
				 SEND_TIMEOUT_MS);
	k_mutex_unlock(&send_lock);
	return ret;
}

static void send_hello(int sock)
{
	static char message[3072]; /* the config and up to SCOPE_SOURCES source names */
	int n;

	k_mutex_lock(&scope_lock, K_FOREVER);
	n = snprintf(message, sizeof(message), "{\"type\":\"hello\",\"config\":");
	n += print_config(message + n, sizeof(message) - n);
	k_mutex_unlock(&scope_lock);
	n += snprintf(message + n, sizeof(message) - n, ",\"source_names\":[");
	for (int s = 0; s < SCOPE_SOURCES; s++) {
		n += snprintf(message + n, sizeof(message) - n, "%s\"%s\"", s ? "," : "",
			      scope_source_names[s]);
	}
	n += snprintf(message + n, sizeof(message) - n, "],\"clock_hz\":%.0f,\"max_length\":%d",
		      SCOPE_CLOCK_HZ, SCOPE_MAX_LENGTH);
	if (capture_waveform_capacity() > 0) {
		n += snprintf(message + n, sizeof(message) - n, ",\"waveform\":{\"capacity\":%u}",
			      capture_waveform_capacity());
	}
	snprintf(message + n, sizeof(message) - n, "}");
	send_text(sock, message);
}

/* when the status last went out: an idle board sends it every STATUS_IDLE_MS anyway, so the
 * pages follow a run, the DC links and the waveform player without asking */
#define STATUS_IDLE_MS 500
static int64_t last_status;

static void broadcast_status(void)
{
	char message[1024];
	int64_t now = k_uptime_get();

	last_status = now;
	uint32_t input_state[4];
	uint32_t boost_state[2];

	capture_input_state(input_state);
	capture_boost_state(boost_state);

	k_mutex_lock(&scope_lock, K_FOREVER);
	if (now - adc.at >= 500) {
		for (int a = 0; a < SCOPE_ADCS; a++) {
			uint32_t conversions = capture_adc_conversions(a);

			adc.rates[a] = adc.at == 0 ? 0 :
				(uint32_t)((uint64_t)(conversions - adc.conversions[a]) * 1000 /
					   (now - adc.at));
			adc.conversions[a] = conversions;
		}
		uint32_t results = capture_sdm_results();

		adc.sdm_rate = adc.at == 0 ? 0 :
			(uint32_t)((uint64_t)(results - adc.sdm_results) * 1000 / (now - adc.at));
		adc.sdm_results = results;
		adc.at = now;
	}
	snprintf(message, sizeof(message),
		 "{\"type\":\"status\",\"state\":\"%s\",\"captures\":%u,\"forced\":%s,"
		 "\"error\":\"\",\"running\":%s,\"lost_words\":%u,\"pacing\":0,"
		 "\"adc_rates\":[%u,%u],\"sdm_rate\":%u,\"gate_power_on\":%u,"
		 "\"capture_us\":%u,\"send_us\":%u,\"dhb\":[%u,%u,%u],\"dc_links\":[%.3f,%.3f],"
		 "\"dhb_control\":[%.5f,%d,%u],\"dhb_trip\":[%u,%.3f,%.2f],\"dhb_filtered\":[%.2f,%.2f],"
		 "\"output\":[%u,%u],\"observer\":[%.3f,%.2f,%.2f,%.3f,%u],"
		 "\"control\":[%.3f,%.2f,%.2f,%.2f,%u,%u],\"waveform\":[%u,%u],"
		 "\"input\":[%u,%u,%.3f,%.3f],"
		 "\"boost\":[%u,%.3f,%.4f,%.3f,%.3f,%.3f,%.3f,%.2f,%u]}",
		 status.state, status.captures, status.forced ? "true" : "false",
		 status.running ? "true" : "false", stream_lost(), adc.rates[0], adc.rates[1],
		 adc.sdm_rate, capture_gate_power_on(), copy_us, send_us, capture_dhb_status(),
		 capture_dhb_periods(), capture_dhb_run_periods(), capture_scaled(SCALER_PRI_VDC_IN),
		 capture_scaled(SCALER_SEC_VDC_OUT), capture_dhb_command(), capture_dhb_phase(),
		 capture_dhb_updates(), capture_dhb_trips(), capture_dhb_trip_current(),
		 capture_dhb_trip_voltage(), capture_dhb_filtered(0), capture_dhb_filtered(1),
		 capture_output_status(), capture_output_periods(), capture_observer(0),
		 capture_observer(1), capture_observer(2), capture_observer(3), capture_observer_steps(),
		 capture_control(0), capture_control(1), capture_control(2), capture_control(3),
		 capture_control_flags(), capture_control_steps(), waveform_loaded,
		 capture_waveform_status(), input_state[0], input_state[1],
		 (int32_t)input_state[2] / 1048576.0, (int32_t)input_state[3] / 1048576.0,
		 boost_state[0], capture_boost_value(0), capture_boost_value(1), capture_boost_value(2),
		 capture_boost_value(3), capture_boost_value(4), capture_boost_value(5),
		 capture_boost_value(6), boost_state[1]);
	k_mutex_unlock(&scope_lock);

	for (int i = 0; i < MAX_CLIENTS; i++) {
		if (clients[i].sock >= 0) {
			send_text(clients[i].sock, message);
		}
	}
}

/* ------------------------------------------------------------------ kept settings */

/*
 * The settings survive a restart in the web client's littlefs, a dot file the file list
 * leaves out (so upload_web.sh keeps it): written 2 s after the last change and only when
 * changed, read back at start through apply_config. The gate driver supplies are not taken
 * from it, they stay off until a page turns them on.
 */
#define SETTINGS_FILE "/web/.scope_settings.json"

static char saved_settings[1536];

static void save_settings(struct k_work *work)
{
	static char text[sizeof(saved_settings)];
	struct fs_file_t file;
	int ret;

	ARG_UNUSED(work);
	k_mutex_lock(&scope_lock, K_FOREVER);
	print_config(text, sizeof(text));
	k_mutex_unlock(&scope_lock);
	if (strcmp(text, saved_settings) == 0) {
		return;
	}
	fs_file_t_init(&file);
	ret = fs_open(&file, SETTINGS_FILE ".new", FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret == 0) {
		ret = fs_write(&file, text, strlen(text)) == strlen(text) ? 0 : -EIO;
		fs_close(&file);
	}
	if (ret == 0) {
		fs_unlink(SETTINGS_FILE);
		ret = fs_rename(SETTINGS_FILE ".new", SETTINGS_FILE);
	}
	if (ret) {
		LOG_WRN("settings not kept: %d", ret);
		return;
	}
	strcpy(saved_settings, text);
	LOG_INF("settings kept");
}

static K_WORK_DELAYABLE_DEFINE(save_work, save_settings);

void scope_settings_load(void)
{
	struct fs_file_t file;
	ssize_t n;

	fs_file_t_init(&file);
	if (fs_open(&file, SETTINGS_FILE, FS_O_READ) != 0) {
		LOG_INF("no kept settings, the defaults");
		return;
	}
	n = fs_read(&file, saved_settings, sizeof(saved_settings) - 1);
	fs_close(&file);
	if (n <= 0) {
		saved_settings[0] = '\0';
		return;
	}
	saved_settings[n] = '\0';
	k_mutex_lock(&scope_lock, K_FOREVER);
	apply_config(saved_settings, false);
	k_mutex_unlock(&scope_lock);
	LOG_INF("kept settings: %s", saved_settings);
}

/* ------------------------------------------------------------------ clients */

/* the output's watchdog: kicked every WATCHDOG_KICK_MS while some client sent a message (a
 * page's heartbeat, a script's request) in the last CLIENT_SILENCE_MS; with nobody there, or
 * this firmware stuck, the FPGA brings a run down after its watchdog's 1 s */
#define WATCHDOG_KICK_MS  200
#define CLIENT_SILENCE_MS 3000
static int64_t last_client_message = INT64_MIN / 2;

static void watchdog_kick(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	if (k_uptime_get() - last_client_message < CLIENT_SILENCE_MS) {
		capture_output_kick();
		capture_boost_kick();
	}
}

K_TIMER_DEFINE(watchdog_timer, watchdog_kick, NULL);

static void handle_command(const char *message)
{
	last_client_message = k_uptime_get();
	if (json_string_is(message, "type", "heartbeat")) {
		return; /* a page still there: no more than that */
	}
	k_mutex_lock(&scope_lock, K_FOREVER);
	if (json_string_is(message, "type", "config")) {
		apply_config(message, true);
		status.config_version++;
		k_work_reschedule(&save_work, K_SECONDS(2));
	} else if (json_string_is(message, "type", "run")) {
		status.running = true;
		status.single = false;
	} else if (json_string_is(message, "type", "single")) {
		status.running = true;
		status.single = true;
	} else if (json_string_is(message, "type", "stop")) {
		status.running = false;
		status.state = "stopped";
	} else if (json_string_is(message, "type", "force")) {
		status.force = true;
	} else if (json_string_is(message, "type", "dhb_test")) {
		double periods = 3000, pulse = 40, ramp = 0, run = 0, invert = 0;

		json_number(message, "periods", &periods);
		json_number(message, "pulse", &pulse);
		json_number(message, "ramp", &ramp);
		json_number(message, "run", &run);
		json_number(message, "invert", &invert);
		LOG_WRN("DHB test: precharge %u periods from %u clock pulses, %u periods a ramp step, "
			"then run %u periods%s", (uint32_t)periods, (uint32_t)pulse, (uint32_t)ramp,
			(uint32_t)run, invert != 0 ? ", the secondary inverted" : "");
		capture_dhb_test((uint32_t)periods, (uint32_t)pulse, (uint32_t)ramp, (uint32_t)run,
				 invert != 0);
	} else if (json_string_is(message, "type", "waveform_load")) {
		double passes = 1;

		json_number(message, "passes", &passes);
		waveform_loaded = parse_waveform(message, MAX(passes, 0.0));
		LOG_WRN("waveform loaded: %u points, %u passes", waveform_loaded, (uint32_t)MAX(passes, 0.0));
	} else if (json_string_is(message, "type", "waveform_run")) {
		double periods = 4000;

		/* 0: until stopped (or the waveform's end); the DHB runs on while the output does */
		json_number(message, "periods", &periods);
		LOG_WRN("output bridge closed loop on the waveform armed: %u periods%s",
			(uint32_t)periods, periods < 1 ? " (until stopped)" : "");
		capture_output_waveform_test((uint32_t)CLAMP(periods, 0.0, 1073741824.0));
	} else if (json_string_is(message, "type", "input_test")) {
		double which = 0, duty = 0.1, periods = 400, limit = 120.0;

		json_number(message, "switch", &which);
		json_number(message, "duty", &duty);
		json_number(message, "periods", &periods);
		json_number(message, "limit", &limit);
		LOG_WRN("input bridge test: switch %u at duty %.3f for %u periods, stop at %.1f V",
			(uint32_t)which, duty, (uint32_t)periods, limit);
		capture_input_test((uint32_t)CLAMP(which, 0.0, 3.0), CLAMP(duty, 0.0, 0.2),
				   (uint32_t)CLAMP(periods, 1.0, 4000.0), CLAMP(limit, 0.0, 450.0));
	} else if (json_string_is(message, "type", "input_stop")) {
		capture_input_stop();
	} else if (json_string_is(message, "type", "boost_start") ||
		   json_string_is(message, "type", "boost_set")) {
		/* the settings given (each limited); boost_start then starts it (when stopped),
		 * boost_set only changes them: a run stopped meanwhile stays stopped */
		static const struct {
			const char *name;
			int reg;
			double low, high;
		} settings[] = {
			{"v_ref", 1, 0.0, 400.0}, {"kp_v", 2, 0.0, 10.0}, {"ki_v", 3, 0.0, 1.0},
			{"kp_i", 4, 0.0, 1.0}, {"ki_i", 5, 0.0, 1.0}, {"i_max", 6, 0.0, 5.0},
			{"d_max", 7, 0.0, 0.5}, {"limit", 8, 0.0, 450.0}, {"ramp", 16, 0.0, 10.0},
		};

		for (size_t i = 0; i < ARRAY_SIZE(settings); i++) {
			double value;

			if (json_number(message, settings[i].name, &value)) {
				capture_boost_set(settings[i].reg,
						  CLAMP(value, settings[i].low, settings[i].high));
			}
		}
		if (json_string_is(message, "type", "boost_start")) {
			LOG_WRN("input boost: start, the link to %.1f V", capture_boost_value(6));
			capture_boost_start();
		}
	} else if (json_string_is(message, "type", "boost_stop")) {
		capture_boost_stop();
	} else if (json_string_is(message, "type", "waveform_stop")) {
		capture_output_stop();
	} else if (json_string_is(message, "type", "output_test")) {
		double before = 0.5, after = 0.5, step = 1100, periods = 2200;

		json_number(message, "before", &before);
		json_number(message, "after", &after);
		json_number(message, "step", &step);
		json_number(message, "periods", &periods);
		if (json_string_is(message, "mode", "control")) {
			double amplitude = 20.0, half = 200;   /* 5 ms at 40 kHz */

			json_number(message, "amplitude", &amplitude);
			json_number(message, "half", &half);
			LOG_WRN("output bridge closed loop armed: +-%.1f V, %u periods a half, %u periods",
				amplitude, (uint32_t)half, (uint32_t)periods);
			/* the square wave starts at -amplitude: a negative one starts high (a fixed
			 * level with a half cycle longer than the run) */
			capture_output_control_test(CLAMP(amplitude, -400.0, 400.0), (uint32_t)half,
						    (uint32_t)periods);
		} else if (json_string_is(message, "mode", "voltage")) {
			LOG_WRN("output bridge test armed: %.2f V, after %u periods %.2f V, %u periods",
				before, (uint32_t)step, after, (uint32_t)periods);
			capture_output_voltage_test(CLAMP(before, -450.0, 450.0),
						    CLAMP(after, -450.0, 450.0), (uint32_t)step,
						    (uint32_t)periods);
		} else {
			LOG_WRN("output bridge test armed: duty %.3f, after %u periods %.3f, %u periods",
				before, (uint32_t)step, after, (uint32_t)periods);
			capture_output_test(CLAMP(before, 0.0, 1.0), CLAMP(after, 0.0, 1.0),
					    (uint32_t)step, (uint32_t)periods);
		}
	} else if (json_string_is(message, "type", "output_stop")) {
		capture_output_stop();
	} else if (json_string_is(message, "type", "dhb_clear_trip")) {
		capture_dhb_clear_trips();
	} else if (json_string_is(message, "type", "dhb_stop")) {
		capture_dhb_stop();
	}
	k_mutex_unlock(&scope_lock);
	broadcast_status();
}

static void client_thread(void *p1, void *p2, void *p3)
{
	struct client *client = p1;
	size_t length = 0;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	send_hello(client->sock);
	/* the last capture, so a page shows what the scope last took, stopped or not */
	k_mutex_lock(&send_lock, K_FOREVER);
	if (last >= 0) {
		websocket_send_msg(client->sock, messages[last], last_size,
				   WEBSOCKET_OPCODE_DATA_BINARY, false, true, SEND_TIMEOUT_MS);
	}
	k_mutex_unlock(&send_lock);
	broadcast_status();

	while (true) {
		uint32_t type;
		uint64_t remaining;
		int ret = websocket_recv_msg(client->sock, client->command + length,
					     COMMAND_MAX - 1 - length, &type, &remaining,
					     SYS_FOREVER_MS);

		if (ret < 0 || (type & WEBSOCKET_FLAG_CLOSE)) {
			break;
		}
		length += ret;
		if (length >= COMMAND_MAX - 1) {
			length = 0; /* longer than any command, drop it */
			continue;
		}
		if (remaining == 0 && (type & WEBSOCKET_FLAG_FINAL)) {
			if (type & WEBSOCKET_FLAG_TEXT) {
				client->command[length] = '\0';
				handle_command(client->command);
			}
			length = 0;
		}
	}

	LOG_INF("client %d disconnected", (int)(client - clients));
	k_mutex_lock(&send_lock, K_FOREVER);
	(void)websocket_unregister(client->sock);
	client->sock = -1;
	k_mutex_unlock(&send_lock);
}

int scope_client_add(int ws_socket)
{
	for (int i = 0; i < MAX_CLIENTS; i++) {
		struct client *client = &clients[i];

		if (client->sock < 0) {
			client->sock = ws_socket;
			client->command = commands[i];
			k_thread_create(&client->thread, client_stacks[i],
					K_THREAD_STACK_SIZEOF(client_stacks[i]), client_thread, client,
					NULL, NULL, K_PRIO_PREEMPT(8), 0, K_NO_WAIT);
			LOG_INF("client %d connected", i);
			return 0;
		}
	}
	LOG_WRN("no room for another client");
	return -ENOENT;
}

/* ------------------------------------------------------------------ acquisition */

static size_t build_capture_message(uint8_t *message, const struct scope_config *c,
				    const struct scope_capture *cap)
{
	char *header = (char *)message + 4;
	int n = snprintf(header, HEADER_MAX,
			 "{\"type\":\"capture\",\"channels\":%d,\"length\":%u,\"pre\":%u,"
			 "\"sample_rate\":%.6f,\"sources\":",
			 SCOPE_CHANNELS, cap->length, cap->pre, SCOPE_CLOCK_HZ / (c->divider + 1));

	n += print_sources(header + n, HEADER_MAX - n);
	n += snprintf(header + n, HEADER_MAX - n,
		      ",\"forced\":%s,\"capture\":%u,\"trigger_channel\":%u,\"trigger_level\":%d,"
		      "\"lost\":%u}",
		      cap->forced ? "true" : "false", status.captures, c->trigger_channel,
		      c->trigger_level, cap->lost);
	/* padded to the samples, which capture_run wrote after HEADER_MAX: JSON takes the spaces */
	n = MIN(n, HEADER_MAX);
	memset(header + n, ' ', HEADER_MAX - n);
	sys_put_le32(HEADER_MAX, message);
	return 4 + HEADER_MAX;
}

void scope_get_config(struct scope_config *c)
{
	k_mutex_lock(&scope_lock, K_FOREVER);
	*c = config;
	k_mutex_unlock(&scope_lock);
}

static void scope_thread(void *p1, void *p2, void *p3)
{
	int64_t last_publish = 0;
	uint32_t capture_cycles = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_timer_start(&watchdog_timer, K_MSEC(WATCHDOG_KICK_MS), K_MSEC(WATCHDOG_KICK_MS));
	while (true) {
		struct scope_config c;
		struct scope_capture cap;
		int64_t armed;
		size_t offset;
		int ret;

		uint32_t version;

		if (stream_active()) {
			/* the FPGA is the stream's: no triggered captures meanwhile */
			if (strcmp(status.state, "streaming") != 0) {
				status.state = "streaming";
				broadcast_status();
			}
			if (k_uptime_get() - last_status >= STATUS_IDLE_MS) {
				broadcast_status();
			}
			k_msleep(50);
			continue;
		}
		if (strcmp(status.state, "streaming") == 0) {
			status.state = status.running ? "waiting" : "stopped";
			broadcast_status();
		}
		if (!status.running) {
			if (k_uptime_get() - last_status >= STATUS_IDLE_MS) {
				broadcast_status();
			}
			k_msleep(50);
			continue;
		}

		k_mutex_lock(&scope_lock, K_FOREVER);
		c = config;
		k_mutex_unlock(&scope_lock);

		/* a capture takes at least its record time, and the display needs no more */
		uint32_t record_ms = (uint32_t)(1000.0 * c.length * (c.divider + 1) / SCOPE_CLOCK_HZ);
		int64_t wait = last_publish + MAX(DISPLAY_INTERVAL_MS, record_ms) - k_uptime_get();

		if (wait > 0) {
			k_msleep(wait);
		}

		/* the settings as they are now, a config command may have come during the wait */
		k_mutex_lock(&scope_lock, K_FOREVER);
		c = config;
		version = status.config_version;
		k_mutex_unlock(&scope_lock);

		armed = k_uptime_get();
		status.state = "waiting";
		offset = 4 + HEADER_MAX; /* the samples go after the header room */
		while (true) {
			bool force;

			k_mutex_lock(&scope_lock, K_FOREVER);
			force = status.force ||
				(c.mode == SCOPE_MODE_AUTO &&
				 k_uptime_get() - armed > MAX(100, 2 * (int64_t)record_ms));
			status.force = false;
			k_mutex_unlock(&scope_lock);

			uint32_t t0 = k_cycle_get_32();

			ret = capture_run(&c, force, (int16_t *)&messages[filling][offset], &cap);
			capture_cycles = k_cycle_get_32() - t0;
			if (ret == 0 || ret == -EBUSY || !status.running ||
			    status.config_version != version) {
				break;
			}
			k_msleep(MAX(10, record_ms / 4));
		}
		if (ret != 0) {
			continue;
		}

		k_mutex_lock(&scope_lock, K_FOREVER);
		if (status.config_version != version) {
			/* taken with settings that no longer apply */
			k_mutex_unlock(&scope_lock);
			continue;
		}
		status.captures++;
		status.forced = cap.forced;
		status.state = "done";
		if (status.single) {
			status.running = false;
			status.state = "stopped";
		}
		k_mutex_unlock(&scope_lock);

		size_t size = build_capture_message(messages[filling], &c, &cap) +
			      cap.length * SCOPE_CHANNELS * sizeof(int16_t);
		uint32_t t1 = k_cycle_get_32();

		k_mutex_lock(&send_lock, K_FOREVER);
		for (int i = 0; i < MAX_CLIENTS; i++) {
			if (clients[i].sock >= 0) {
				websocket_send_msg(clients[i].sock, messages[filling], size,
						   WEBSOCKET_OPCODE_DATA_BINARY, false, true,
						   SEND_TIMEOUT_MS);
			}
		}
		/* this one is the last capture now, the next goes into the other */
		last = filling;
		last_size = size;
		filling = !filling;
		k_mutex_unlock(&send_lock);
		copy_us = k_cyc_to_us_floor32(capture_cycles);
		send_us = k_cyc_to_us_floor32(k_cycle_get_32() - t1);
		last_publish = k_uptime_get();
		broadcast_status();
	}
}

K_THREAD_DEFINE(scope_tid, SCOPE_STACK_SIZE, scope_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(10),
		0, 0);
