/*
 * Software stand-in for ti60_ac_psu's scope_capture and test_signal_generator.vhd: the same
 * sixteen sources computed the same way, sampled every divider+1 clocks of a 125 MHz clock.
 *
 * The generator runs on between captures as if it never stopped: before each capture it is
 * advanced by the samples that would have passed since the last one. A capture is taken from
 * twice the record length of fresh samples, placed so that the first trigger crossing that
 * leaves room for the pre-trigger samples lands at sample pre.
 */
#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "scope.h"

const char *scope_source_names[SCOPE_SOURCES] = {
	"sine", "cosine", "triangle", "square", "sawtooth", "noise", "sine + noise", "counter",
	"probe 0: clock 15:0", "probe 1: clock 31:16", "probe 2", "probe 3",
	"probe 4", "probe 5", "probe 6", "probe 7",
};

#define SINE_POINTS 1024
#define PI          3.14159265358979323846

static int16_t sine_table[SINE_POINTS];

/* generator state, kept between captures */
static uint32_t phase;
static uint32_t lfsr = 0xACE11234;
static uint16_t counter;
static uint32_t clock_count;
static int64_t last_uptime_us;

static int16_t window[2 * SCOPE_MAX_LENGTH][SCOPE_CHANNELS];

static void make_sine_table(void)
{
	/* round(32000*sin(2*pi*i/1024)), as sine_table_pkg.vhd */
	for (int i = 0; i < SINE_POINTS; i++) {
		sine_table[i] = (int16_t)lround(32000.0 * sin(2.0 * PI * i / SINE_POINTS));
	}
}

static int16_t source_value(uint8_t source)
{
	uint32_t index = phase >> 22;
	int16_t noise = (int16_t)(lfsr & 0xffff);
	int32_t triangle;

	switch (source) {
	case 0:
		return sine_table[index];
	case 1:
		return sine_table[(index + 256) % SINE_POINTS];
	case 2:
		triangle = (phase >> 15) & 0xffff;
		return (int16_t)((phase & BIT(31)) ? 32767 - triangle : triangle - 32768);
	case 3:
		return (phase & BIT(31)) ? -24000 : 24000;
	case 4:
		return (int16_t)((phase >> 16) ^ 0x8000);
	case 5:
		return noise;
	case 6:
		return (int16_t)(sine_table[index] / 2 + (noise >> 4));
	case 7:
		return (int16_t)counter;
	case 8:
		return (int16_t)(clock_count & 0xffff);
	case 9:
		return (int16_t)(clock_count >> 16);
	default:
		return 0;
	}
}

static void generator_step(uint32_t increment, uint32_t clocks)
{
	phase += increment;
	lfsr = (lfsr << 1) | (((lfsr >> 31) ^ (lfsr >> 21) ^ (lfsr >> 1) ^ lfsr) & 1);
	counter++;
	clock_count += clocks;
}

static bool crossed(int16_t before, int16_t now, int32_t level, enum scope_edge edge)
{
	bool rising = before < level && now >= level;
	bool falling = before >= level && now < level;

	return (edge == SCOPE_EDGE_RISING && rising) || (edge == SCOPE_EDGE_FALLING && falling) ||
	       (edge == SCOPE_EDGE_EITHER && (rising || falling));
}

int capture_run(const struct scope_config *config, bool force, int16_t *samples,
		struct scope_capture *capture)
{
	double sample_rate = SCOPE_CLOCK_HZ / (config->divider + 1);
	uint32_t increment = (uint32_t)(int64_t)(config->generator_hz / sample_rate * 4294967296.0);
	uint32_t clocks = config->divider + 1;
	uint32_t length = config->length;
	uint32_t pre = MIN(length * config->pre_percent / 100, length - 1);
	int64_t now = k_ticks_to_us_floor64(k_uptime_ticks());
	uint64_t passed;
	uint32_t start = 0;
	bool found = false;

	if (sine_table[SINE_POINTS / 4] == 0) {
		make_sine_table();
	}

	/* the samples since the last capture, only the phase and the counters matter */
	passed = (uint64_t)((now - last_uptime_us) * 1e-6 * sample_rate);
	phase += (uint32_t)(passed * increment);
	counter += (uint16_t)passed;
	clock_count += (uint32_t)(passed * clocks);
	last_uptime_us = now;

	for (uint32_t s = 0; s < 2 * length; s++) {
		for (int c = 0; c < SCOPE_CHANNELS; c++) {
			window[s][c] = source_value(config->sources[c]);
		}
		generator_step(increment, clocks);
	}

	if (config->mode != SCOPE_MODE_FREE) {
		uint32_t channel = config->trigger_channel % SCOPE_CHANNELS;

		for (uint32_t s = MAX(pre, 1); s + length - pre <= 2 * length; s++) {
			if (crossed(window[s - 1][channel], window[s][channel],
				    config->trigger_level, config->trigger_edge)) {
				start = s - pre;
				found = true;
				break;
			}
		}
		if (!found && !force) {
			return -EAGAIN;
		}
	}

	memcpy(samples, window[start], length * sizeof(window[0]));
	capture->length = length;
	capture->pre = pre;
	capture->forced = config->mode != SCOPE_MODE_FREE && !found;
	capture->lost = 0;
	return 0;
}

int capture_init(void)
{
	for (int s = 0; s < SCOPE_SOURCES; s++) {
		if (scope_source_names[s] == NULL) {
			scope_source_names[s] = "-";
		}
	}
	return 0;
}

/* no ADCs here */
void capture_set_adc_masks(const uint8_t masks[SCOPE_ADCS])
{
	ARG_UNUSED(masks);
}

void capture_set_adc_settle(const uint32_t settle_ns[SCOPE_ADCS])
{
	ARG_UNUSED(settle_ns);
}

uint32_t capture_adc_conversions(int adc)
{
	ARG_UNUSED(adc);
	return 0;
}

void capture_set_sdm(uint32_t log2, uint32_t phase)
{
	ARG_UNUSED(log2);
	ARG_UNUSED(phase);
}

uint32_t capture_sdm_results(void)
{
	return 0;
}

void capture_set_gate_power(uint32_t enables)
{
	ARG_UNUSED(enables);
}

uint32_t capture_gate_power_on(void)
{
	return 0;
}

void capture_dhb_test(uint32_t periods, uint32_t pulse_clocks, uint32_t ramp_periods,
		      uint32_t run_periods, bool inverted)
{
	ARG_UNUSED(periods);
	ARG_UNUSED(pulse_clocks);
	ARG_UNUSED(ramp_periods);
	ARG_UNUSED(run_periods);
	ARG_UNUSED(inverted);
}

void capture_dhb_stop(void)
{
}

uint32_t capture_dhb_status(void)
{
	return 0;
}

uint32_t capture_dhb_periods(void)
{
	return 0;
}

void capture_set_scaling(int channel, double zero, double per_unit)
{
	ARG_UNUSED(channel);
	ARG_UNUSED(zero);
	ARG_UNUSED(per_unit);
}

void capture_scaler_init(void)
{
}

double capture_scaled(int channel)
{
	ARG_UNUSED(channel);
	return 0.0;
}

void capture_set_dhb_control(bool enable, double kp, double ki, double limit_high, double limit_low)
{
	ARG_UNUSED(enable);
	ARG_UNUSED(kp);
	ARG_UNUSED(ki);
	ARG_UNUSED(limit_high);
	ARG_UNUSED(limit_low);
}

double capture_dhb_command(void)
{
	return 0.0;
}

int capture_dhb_phase(void)
{
	return 0;
}

uint32_t capture_dhb_updates(void)
{
	return 0;
}

uint32_t capture_dhb_run_periods(void)
{
	return 0;
}

void capture_set_dhb_protection(double overcurrent, double overvoltage, uint32_t readings)
{
	ARG_UNUSED(overcurrent);
	ARG_UNUSED(overvoltage);
	ARG_UNUSED(readings);
}

void capture_set_dhb_filter(uint32_t n)
{
	ARG_UNUSED(n);
}

double capture_dhb_filtered(int secondary)
{
	ARG_UNUSED(secondary);
	return 0.0;
}

uint32_t capture_dhb_trips(void)
{
	return 0;
}

double capture_dhb_trip_current(void)
{
	return 0.0;
}

double capture_dhb_trip_voltage(void)
{
	return 0.0;
}

void capture_dhb_clear_trips(void)
{
}

void capture_output_test(double before, double after, uint32_t step_periods, uint32_t periods)
{
	ARG_UNUSED(before);
	ARG_UNUSED(after);
	ARG_UNUSED(step_periods);
	ARG_UNUSED(periods);
}

void capture_output_voltage_test(double before, double after, uint32_t step_periods, uint32_t periods)
{
	ARG_UNUSED(before);
	ARG_UNUSED(after);
	ARG_UNUSED(step_periods);
	ARG_UNUSED(periods);
}

void capture_output_stop(void)
{
}

void capture_output_kick(void)
{
}

void capture_lcd_text(uint32_t place, const char *text)
{
	ARG_UNUSED(place);
	ARG_UNUSED(text);
}

void capture_lcd_draw(void)
{
}

uint32_t capture_lcd_cols(void)
{
	return 24;
}

uint32_t capture_fpga_version(void)
{
	return 0;
}

void capture_touch_button(uint32_t x_from, uint32_t x_to, uint32_t y_from, uint32_t y_to)
{
	ARG_UNUSED(x_from);
	ARG_UNUSED(x_to);
	ARG_UNUSED(y_from);
	ARG_UNUSED(y_to);
}

uint32_t capture_touch_presses(void)
{
	return 0;
}

void capture_touch_state(uint32_t state[4])
{
	state[0] = state[1] = state[2] = state[3] = 0;
}

void capture_boost_set(int reg, double value)
{
	ARG_UNUSED(reg);
	ARG_UNUSED(value);
}

void capture_boost_start(void)
{
}

void capture_boost_stop(void)
{
}

void capture_boost_kick(void)
{
}

void capture_boost_state(uint32_t state[2])
{
	state[0] = state[1] = 0;
}

double capture_boost_value(int which)
{
	ARG_UNUSED(which);
	return 0.0;
}

uint32_t capture_waveform_capacity(void)
{
	return 0; /* no waveform player without the board */
}

void capture_waveform_load(const float *volts, const uint32_t *periods, uint32_t count, uint32_t passes)
{
	ARG_UNUSED(volts);
	ARG_UNUSED(periods);
	ARG_UNUSED(count);
	ARG_UNUSED(passes);
}

void capture_output_waveform_test(uint32_t periods)
{
	ARG_UNUSED(periods);
}

uint32_t capture_waveform_status(void)
{
	return 0;
}

uint32_t capture_output_status(void)
{
	return 0;
}

uint32_t capture_output_periods(void)
{
	return 0;
}

void capture_set_observer(bool enable, double l, double c, double r, double current_hz, double voltage_hz)
{
	ARG_UNUSED(enable);
	ARG_UNUSED(l);
	ARG_UNUSED(c);
	ARG_UNUSED(r);
	ARG_UNUSED(current_hz);
	ARG_UNUSED(voltage_hz);
}

double capture_observer(int estimate)
{
	ARG_UNUSED(estimate);
	return 0.0;
}

uint32_t capture_observer_steps(void)
{
	return 0;
}

void capture_set_control(bool enable, double l, double c, double r, double current_hz, double voltage_hz,
			 double b, double voltage_zero, double du_hz, double i_max, double vl_max, double u_max,
			 double prefilter_hz, double feedforward)
{
	ARG_UNUSED(prefilter_hz);
	ARG_UNUSED(feedforward);
	ARG_UNUSED(voltage_zero);
	ARG_UNUSED(enable);
	ARG_UNUSED(l);
	ARG_UNUSED(c);
	ARG_UNUSED(r);
	ARG_UNUSED(current_hz);
	ARG_UNUSED(voltage_hz);
	ARG_UNUSED(b);
	ARG_UNUSED(du_hz);
	ARG_UNUSED(i_max);
	ARG_UNUSED(vl_max);
	ARG_UNUSED(u_max);
}

double capture_control(int value)
{
	ARG_UNUSED(value);
	return 0.0;
}

uint32_t capture_control_flags(void)
{
	return 0;
}

uint32_t capture_control_steps(void)
{
	return 0;
}

void capture_output_control_test(double amplitude, uint32_t half_periods, uint32_t periods)
{
	ARG_UNUSED(amplitude);
	ARG_UNUSED(half_periods);
	ARG_UNUSED(periods);
}
