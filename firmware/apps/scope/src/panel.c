/*
 * The face panel: its LCD (lcd_display.vhd, a screen of text) and touch screen (touch_reader.vhd).
 *
 * The screen shows the board's address, the last touch, and the commits the firmware and the
 * FPGA image were built from, each on a line of its own; looked at every 200 ms, a line that
 * changed rewritten and the screen redrawn.
 *
 * The touch screen's calibration: its controller reads 0..4095 on two axes that need not be the
 * screen's (swapped, mirrored, offset), so screen pixels are an affine map of the raw readings,
 *   px = a rx + b ry + c,  py = d rx + e ry + f
 * fitted (least squares) to four touches on "+" targets near the corners. panel_calibrate()
 * starts it: each target held for half a second, then released; the fit kept in the web client's
 * littlefs (a dot file the file list leaves out, so upload_web.sh keeps it) and read at start.
 * No touch for 30 s gives up and keeps the old calibration.
 */
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include "scope.h"
#include "git_version.h"

LOG_MODULE_REGISTER(panel, LOG_LEVEL_INF);

#define LINE_MAX_COLS 24
#define ROWS          10

#define ADDRESS_ROW  2
#define TOUCH_ROW    4
#define FIRMWARE_ROW 6
#define FPGA_ROW     7
#define LINES        4

#define CALIBRATION_FILE "/web/.touch_calibration"

/* a "+" in text cell (col, row) is centred on this pixel: the glyph's centre is its column 6,
 * row 8, the font 10 x 16 at twice its size */
#define TARGET_X(col) (20 * (col) + 13)
#define TARGET_Y(row) (32 * (row) + 17)

static const struct {
	uint8_t col, row;
} targets[] = {{1, 1}, {22, 1}, {22, 8}, {1, 8}};

#define TARGETS     ARRAY_SIZE(targets)
#define HOLD_TICKS  10      /* of 50 ms: held this long counts */
#define SETTLE      3       /* the first ticks of a touch left out */
#define LIFTED      4       /* untouched ticks that end a touch (the controller's touch flickers) */
#define GIVE_UP_MS  30000

static struct {
	bool valid;
	double m[6];        /* a b c d e f */
} calibration;

static struct {
	bool active;
	int target;
	int ticks;          /* touched ticks of this target */
	bool captured;      /* held long enough: waiting for the release */
	int untouched;      /* ticks since the touch was last seen */
	double sum_x, sum_y;
	int samples;
	double raw[TARGETS][2];
	int64_t last_touch;
	int64_t message_until;  /* the result shown until then */
} cal;

static char shown[LINES][LINE_MAX_COLS + 1];

static void panel_update(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(panel_work, panel_update);

static uint32_t screen_cols(void)
{
	return MIN(capture_lcd_cols(), LINE_MAX_COLS);
}

/* a row of text padded with spaces to the screen's width */
static void write_row(uint32_t row, const char *text)
{
	char line[LINE_MAX_COLS + 1];
	uint32_t cols = screen_cols();

	snprintf(line, sizeof(line), "%-*.*s", (int)cols, (int)cols, text);
	line[cols] = '\0';
	capture_lcd_text(row * cols, line);
}

static void clear_screen(void)
{
	for (uint32_t row = 0; row < ROWS; row++) {
		write_row(row, "");
	}
	memset(shown, 0, sizeof(shown));    /* every line written again */
}

/* a line of the normal screen, written when it is not what is shown */
static bool normal_line(int which, uint32_t row, const char *text)
{
	char line[LINE_MAX_COLS + 1];
	uint32_t cols = screen_cols();

	snprintf(line, sizeof(line), "%-*s", (int)cols, text);
	line[cols] = '\0';
	if (strcmp(line, shown[which]) == 0) {
		return false;
	}
	capture_lcd_text(row * cols, line);
	memcpy(shown[which], line, sizeof(line));
	return true;
}

bool panel_touch_pixels(const uint32_t touch[4], int *x, int *y)
{
	if (!calibration.valid) {
		return false;
	}
	*x = (int)lround(calibration.m[0] * touch[1] + calibration.m[1] * touch[2] + calibration.m[2]);
	*y = (int)lround(calibration.m[3] * touch[1] + calibration.m[4] * touch[2] + calibration.m[5]);
	return true;
}

static void show_normal(void)
{
	char address[NET_IPV4_ADDR_LEN] = "...";
	char text[LINE_MAX_COLS + 1];
	struct net_in_addr *ip = net_if_ipv4_get_global_addr(net_if_get_default(), NET_ADDR_PREFERRED);
	uint32_t touch[4];
	uint32_t fpga = capture_fpga_version();
	bool changed = false;
	int x, y;

	if (ip != NULL) {
		net_addr_ntop(NET_AF_INET, ip, address, sizeof(address));
	}
	snprintf(text, sizeof(text), "IP %s", address);
	changed |= normal_line(0, ADDRESS_ROW, text);

	capture_touch_state(touch);
	if (!touch[0]) {
		snprintf(text, sizeof(text), "touch -");
	} else if (panel_touch_pixels(touch, &x, &y)) {
		snprintf(text, sizeof(text), "touch %d %d", x, y);
	} else {
		snprintf(text, sizeof(text), "touch raw %u %u", touch[1], touch[2]);
	}
	changed |= normal_line(1, TOUCH_ROW, text);

	snprintf(text, sizeof(text), "fw   %s%s", GIT_HASH, GIT_DIRTY ? "+" : "");
	changed |= normal_line(2, FIRMWARE_ROW, text);
	snprintf(text, sizeof(text), "fpga %07x%s", fpga & 0x0fffffff, fpga & BIT(31) ? "+" : "");
	changed |= normal_line(3, FPGA_ROW, text);

	if (changed) {
		capture_lcd_draw();
	}
}

/* ------------------------------------------------------------------ calibration */

static void show_target(void)
{
	char line[48];      /* write_row cuts it to the screen's width */
	uint32_t cols = screen_cols();

	clear_screen();
	write_row(4, "   touch the + and hold");
	snprintf(line, sizeof(line), "   until it moves (%d/%d)", cal.target + 1, (int)TARGETS);
	write_row(5, line);
	capture_lcd_text(targets[cal.target].row * cols + targets[cal.target].col, "+");
	capture_lcd_draw();
}

/* the 3 x 3 system A v = b by Gaussian elimination with partial pivoting */
static bool solve3(double a[3][3], double b[3], double v[3])
{
	for (int c = 0; c < 3; c++) {
		int p = c;

		for (int r = c + 1; r < 3; r++) {
			if (fabs(a[r][c]) > fabs(a[p][c])) {
				p = r;
			}
		}
		if (fabs(a[p][c]) < 1e-12) {
			return false;
		}
		for (int k = 0; k < 3; k++) {
			double t = a[c][k];

			a[c][k] = a[p][k];
			a[p][k] = t;
		}
		double t = b[c];

		b[c] = b[p];
		b[p] = t;
		for (int r = c + 1; r < 3; r++) {
			double f = a[r][c] / a[c][c];

			for (int k = c; k < 3; k++) {
				a[r][k] -= f * a[c][k];
			}
			b[r] -= f * b[c];
		}
	}
	for (int r = 2; r >= 0; r--) {
		double s = b[r];

		for (int k = r + 1; k < 3; k++) {
			s -= a[r][k] * v[k];
		}
		v[r] = s / a[r][r];
	}
	return true;
}

/* the affine map from the targets' raw readings to their pixels; the fit's rms error, pixels */
static bool fit(double m[6], double *error)
{
	double a[3][3] = {0}, bx[3] = {0}, by[3] = {0};
	double a2[3][3];
	double sum = 0;

	for (size_t i = 0; i < TARGETS; i++) {
		double row[3] = {cal.raw[i][0], cal.raw[i][1], 1.0};

		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				a[j][k] += row[j] * row[k];
			}
			bx[j] += row[j] * TARGET_X(targets[i].col);
			by[j] += row[j] * TARGET_Y(targets[i].row);
		}
	}
	memcpy(a2, a, sizeof(a));
	if (!solve3(a, bx, &m[0]) || !solve3(a2, by, &m[3])) {
		return false;
	}
	for (size_t i = 0; i < TARGETS; i++) {
		double dx = m[0] * cal.raw[i][0] + m[1] * cal.raw[i][1] + m[2] - TARGET_X(targets[i].col);
		double dy = m[3] * cal.raw[i][0] + m[4] * cal.raw[i][1] + m[5] - TARGET_Y(targets[i].row);

		sum += dx * dx + dy * dy;
	}
	*error = sqrt(sum / TARGETS);
	return true;
}

static void save_calibration(void)
{
	struct fs_file_t file;
	char text[200];
	int n = snprintf(text, sizeof(text), "%.9g %.9g %.9g %.9g %.9g %.9g\n", calibration.m[0],
			 calibration.m[1], calibration.m[2], calibration.m[3], calibration.m[4],
			 calibration.m[5]);
	int ret;

	fs_file_t_init(&file);
	ret = fs_open(&file, CALIBRATION_FILE, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret == 0) {
		ret = fs_write(&file, text, n) == n ? 0 : -EIO;
		fs_close(&file);
	}
	if (ret != 0) {
		LOG_WRN("touch calibration not kept: %d", ret);
	}
}

static void load_calibration(void)
{
	struct fs_file_t file;
	char text[200];
	char *p = text;
	ssize_t n;

	fs_file_t_init(&file);
	if (fs_open(&file, CALIBRATION_FILE, FS_O_READ) != 0) {
		LOG_INF("touch screen not calibrated");
		return;
	}
	n = fs_read(&file, text, sizeof(text) - 1);
	fs_close(&file);
	if (n <= 0) {
		return;
	}
	text[n] = '\0';
	for (int i = 0; i < 6; i++) {
		char *end;

		calibration.m[i] = strtod(p, &end);
		if (end == p) {
			return;
		}
		p = end;
	}
	calibration.valid = true;
	LOG_INF("touch calibration read");
}

static void finish(const char *message)
{
	cal.active = false;
	clear_screen();
	write_row(4, message);
	capture_lcd_draw();
	cal.message_until = k_uptime_get() + 3000;
}

/* a tick of the calibration: the touch held on the target, then released */
static void calibration_tick(void)
{
	uint32_t touch[4];
	int64_t now = k_uptime_get();

	capture_touch_state(touch);
	if (touch[0]) {
		cal.last_touch = now;
		cal.untouched = 0;
		if (!cal.captured) {
			cal.ticks++;
			if (cal.ticks > SETTLE) {
				cal.sum_x += touch[1];
				cal.sum_y += touch[2];
				cal.samples++;
			}
			if (cal.ticks >= HOLD_TICKS) {
				cal.raw[cal.target][0] = cal.sum_x / cal.samples;
				cal.raw[cal.target][1] = cal.sum_y / cal.samples;
				cal.captured = true;
				LOG_INF("touch target %d: raw %.0f %.0f", cal.target + 1, cal.raw[cal.target][0],
					cal.raw[cal.target][1]);
			}
		}
		return;
	}
	if (++cal.untouched < LIFTED) {
		return;     /* a flicker, or not yet lifted for long */
	}
	if (!cal.captured) {
		/* lifted too early: start this target again */
		cal.ticks = 0;
		cal.samples = 0;
		cal.sum_x = cal.sum_y = 0;
		if (now - cal.last_touch > GIVE_UP_MS) {
			finish("calibration given up");
		}
		return;
	}
	/* released: the next target, or the fit */
	cal.target++;
	cal.ticks = 0;
	cal.samples = 0;
	cal.sum_x = cal.sum_y = 0;
	cal.captured = false;
	cal.untouched = 0;
	if (cal.target < (int)TARGETS) {
		show_target();
		return;
	}

	double m[6], error;
	char line[48];

	if (!fit(m, &error)) {
		finish("calibration failed");
		return;
	}
	memcpy(calibration.m, m, sizeof(m));
	calibration.valid = true;
	save_calibration();
	LOG_WRN("touch calibrated: %.5f %.5f %.1f / %.5f %.5f %.1f, error %.1f px", m[0], m[1], m[2],
		m[3], m[4], m[5], error);
	snprintf(line, sizeof(line), "calibrated: %.1f px", error);
	finish(line);
}

static atomic_t calibrate_requested;

/* from any thread: the calibration starts in the panel's work */
void panel_calibrate(void)
{
	atomic_set(&calibrate_requested, 1);
	k_work_reschedule(&panel_work, K_NO_WAIT);
}

static void start_calibration(void)
{
	memset(&cal, 0, sizeof(cal));
	cal.active = true;
	cal.last_touch = k_uptime_get();
	show_target();
}

bool panel_calibrating(void)
{
	return cal.active;
}

/* ------------------------------------------------------------------ */

static void panel_update(struct k_work *work)
{
	if (atomic_cas(&calibrate_requested, 1, 0)) {
		start_calibration();
	}
	if (cal.active) {
		calibration_tick();
		k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(50));
		return;
	}
	if (cal.message_until != 0) {
		if (k_uptime_get() < cal.message_until) {
			k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(200));
			return;
		}
		cal.message_until = 0;
		clear_screen();
		write_row(0, "ti60 AC PSU");
	}
	show_normal();
	k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(200));
}

void panel_start(void)
{
	load_calibration();
	k_work_reschedule(&panel_work, K_NO_WAIT);
}
