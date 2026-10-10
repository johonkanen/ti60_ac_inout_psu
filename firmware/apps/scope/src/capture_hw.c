/*
 * The FPGA's scope_capture (fpga/source/scope) as the capture backend, in
 * fpga/tsemac_soc: its registers on an APB window at their board_test addresses,
 * its deep capture streamed by a DMA channel into a circular buffer in this RAM.
 *
 * Every capture is a deep capture. Before arming, the DMA is restarted at the start of
 * the ring; the scope numbers the records it sends from there, and register 12 gives the
 * record of sample 0, so sample s is ring record (start + s) mod ring. Each record is
 * eight 16 bit channels in order (scope_axis_writer). The DMA writes the HyperRAM
 * directly, so the CPU's write-through data cache is invalidated before the samples are
 * read.
 *
 * The stream (stream.c) uses a ring of its own: the capture continuous (mode bit 1), the DMA
 * going round a ring of whole segments, register 22 saying how many records it has taken.
 * The stream starts from the network thread while the acquisition thread may be in
 * capture_run(): hw_lock keeps the two apart, and no triggered capture arms while the
 * stream has the scope.
 */
#include <errno.h>
#include <math.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/sys_io.h>
#include "scope.h"

LOG_MODULE_REGISTER(capture_hw, LOG_LEVEL_INF);

#define SCOPE_NODE DT_INST(0, hvhdl_scope_capture)
#define REGISTERS  DT_REG_ADDR_BY_NAME(SCOPE_NODE, registers)
#define DMA        DT_REG_ADDR_BY_NAME(SCOPE_NODE, dma)
#define BASE       DT_PROP(SCOPE_NODE, base_address)

/* scope_capture registers, offsets from its base address */
#define REG_COMMAND      0
#define REG_DIVIDER      1
#define REG_TRIGGER      2
#define REG_LEVEL        3
#define REG_PRE          4
#define REG_LENGTH       5
#define REG_SOURCES_LO   6
#define REG_PHASE_STEP   7
#define REG_INFORMATION  8
#define REG_MODE         10
#define REG_OVERFLOWS    11
#define REG_START_RECORD 12
#define REG_SOURCES_HI   14
#define REG_PROBE_INDEX  16
#define REG_PROBE_NAME   17
#define REG_WRITTEN      22

#define COMMAND_ARM   BIT(0)
#define COMMAND_FORCE BIT(1)
#define COMMAND_STOP  BIT(2)
#define STATE_MASK    GENMASK(2, 0)
#define STATE_DONE    4
#define STATUS_FORCED BIT(8)
#define TRIGGER_ENABLE BIT(8)
#define MODE_DEEP     BIT(0)
#define MODE_CONTINUOUS BIT(1)

/* Efinix DMA channel 0, direct mode, see dmasg.h of the Sapphire BSP */
#define DMA_OUTPUT_ADDRESS 0x10
#define DMA_OUTPUT_CONFIG  0x1c
#define DMA_INPUT_STREAM   0x08
#define DMA_INPUT_CONFIG   0x0c
#define DMA_DIRECT_BYTES   0x20
#define DMA_STATUS         0x2c
#define DMA_STATUS_BUSY    BIT(0)
#define DMA_STATUS_START   BIT(0)
#define DMA_STATUS_SELF_RESTART BIT(1)
#define DMA_STATUS_STOP    BIT(2)
#define DMA_CONFIG_MEMORY  BIT(12)
#define DMA_BURST_BYTES    256

/* scope_axis_writer: padding after a capture, at most min_pad + a burst of records */
#define PAD_RECORDS (16 + 16)

#define RING_RECORDS CONFIG_SCOPE_RING_RECORDS
BUILD_ASSERT((RING_RECORDS & (RING_RECORDS - 1)) == 0, "the ring is a power of two");
BUILD_ASSERT(SCOPE_MAX_LENGTH + PAD_RECORDS <= RING_RECORDS, "a capture fits the ring");

static int16_t ring[RING_RECORDS][SCOPE_CHANNELS] __aligned(DMA_BURST_BYTES) __noinit;

#define STREAM_RECORDS (SCOPE_STREAM_RING_SEGMENTS * SCOPE_STREAM_SEGMENT_RECORDS)
static int16_t stream_ring[STREAM_RECORDS][SCOPE_CHANNELS] __aligned(DMA_BURST_BYTES) __noinit;
BUILD_ASSERT(sizeof(stream_ring) % DMA_BURST_BYTES == 0, "the stream ring is whole bursts");

const char *scope_source_names[SCOPE_SOURCES];
static char names[SCOPE_SOURCES][17];

static struct scope_config armed_config;
static bool armed;
static bool streaming;
static K_MUTEX_DEFINE(hw_lock);

static uint32_t reg_read(int r)
{
	return sys_read32(REGISTERS + 4 * (BASE + r));
}

static void reg_write(int r, uint32_t value)
{
	sys_write32(value, REGISTERS + 4 * (BASE + r));
}

static void dcache_invalidate(void)
{
	__asm__ volatile(".word 0x500f" ::: "memory");
}

static void dma_restart(void *buffer, size_t bytes)
{
	sys_write32(DMA_STATUS_STOP, DMA + DMA_STATUS);
	for (int i = 0; i < 100000 && (sys_read32(DMA + DMA_STATUS) & DMA_STATUS_BUSY); i++) {
	}
	sys_write32(0, DMA + DMA_INPUT_STREAM);
	sys_write32(0, DMA + DMA_INPUT_CONFIG);
	sys_write32((uintptr_t)buffer, DMA + DMA_OUTPUT_ADDRESS);
	sys_write32(DMA_CONFIG_MEMORY | (DMA_BURST_BYTES - 1), DMA + DMA_OUTPUT_CONFIG);
	sys_write32(bytes - 1, DMA + DMA_DIRECT_BYTES);
	sys_write32(DMA_STATUS_START | DMA_STATUS_SELF_RESTART, DMA + DMA_STATUS);
}

static void set_sources(const struct scope_config *config)
{
	uint32_t sources_lo = 0, sources_hi = 0;

	for (int c = 0; c < 4; c++) {
		sources_lo |= (uint32_t)config->sources[c] << (8 * c);
		sources_hi |= (uint32_t)config->sources[c + 4] << (8 * c);
	}
	reg_write(REG_SOURCES_LO, sources_lo);
	reg_write(REG_SOURCES_HI, sources_hi);
}

static void set_generator(const struct scope_config *config)
{
	double sample_rate = SCOPE_CLOCK_HZ / (config->divider + 1);

	reg_write(REG_PHASE_STEP, (uint32_t)(int64_t)(config->generator_hz / sample_rate * 4294967296.0));
}

static void arm(const struct scope_config *config)
{
	uint32_t pre = MIN(config->length * config->pre_percent / 100, config->length - 1);
	uint32_t trigger = config->trigger_channel % SCOPE_CHANNELS;

	switch (config->trigger_edge) {
	case SCOPE_EDGE_FALLING:
		trigger |= 1 << 4;
		break;
	case SCOPE_EDGE_EITHER:
		trigger |= 2 << 4;
		break;
	default:
		break;
	}
	if (config->mode != SCOPE_MODE_FREE) {
		trigger |= TRIGGER_ENABLE;
	}

	reg_write(REG_COMMAND, COMMAND_STOP);
	dma_restart(ring, sizeof(ring));
	reg_write(REG_DIVIDER, config->divider);
	reg_write(REG_TRIGGER, trigger);
	reg_write(REG_LEVEL, (uint16_t)config->trigger_level);
	reg_write(REG_PRE, pre);
	reg_write(REG_LENGTH, config->length);
	set_sources(config);
	set_generator(config);
	reg_write(REG_MODE, MODE_DEEP);
	reg_write(REG_COMMAND, COMMAND_ARM);
	armed_config = *config;
	armed = true;
}

const int16_t *capture_stream_start(const struct scope_config *config)
{
	k_mutex_lock(&hw_lock, K_FOREVER);
	streaming = true;
	reg_write(REG_COMMAND, COMMAND_STOP);
	dma_restart(stream_ring, sizeof(stream_ring));
	reg_write(REG_DIVIDER, config->divider);
	reg_write(REG_TRIGGER, 0);              /* no trigger: from the start */
	reg_write(REG_PRE, 0);
	reg_write(REG_LENGTH, 1);
	set_sources(config);
	set_generator(config);
	reg_write(REG_MODE, MODE_DEEP | MODE_CONTINUOUS);
	reg_write(REG_COMMAND, COMMAND_ARM);
	armed = false;                          /* a triggered capture arms again after this */
	k_mutex_unlock(&hw_lock);
	return &stream_ring[0][0];
}

void capture_stream_stop(void)
{
	k_mutex_lock(&hw_lock, K_FOREVER);
	reg_write(REG_COMMAND, COMMAND_STOP);
	armed = false;
	streaming = false;
	k_mutex_unlock(&hw_lock);
}

uint32_t capture_stream_written(void)
{
	return reg_read(REG_WRITTEN);
}

/* board_adc.vhd's registers at 700 (sec) and 720 (pri): 0 control (bit0 run, bits 15:8 the
 * mux positions), 3 conversions */
#define ADC_REGISTER(adc, r) (REGISTERS + 4 * ((adc) == 0 ? 700 : 720) + 4 * (r))
#define ADC_CONTROL     0
#define ADC_SETTLE      2   /* adc_clock clocks, 5 ns */
#define ADC_CONVERSIONS 3
#define ADC_CLOCK_NS    5

void capture_set_adc_masks(const uint8_t masks[SCOPE_ADCS])
{
	for (int adc = 0; adc < SCOPE_ADCS; adc++) {
		uint32_t control = sys_read32(ADC_REGISTER(adc, ADC_CONTROL));

		control = (control & ~(0xffu << 8) & ~BIT(1)) | ((uint32_t)masks[adc] << 8);
		sys_write32(control, ADC_REGISTER(adc, ADC_CONTROL));
	}
}

void capture_set_adc_settle(const uint32_t settle_ns[SCOPE_ADCS])
{
	for (int adc = 0; adc < SCOPE_ADCS; adc++) {
		sys_write32(MIN(settle_ns[adc] / ADC_CLOCK_NS, 0xffff), ADC_REGISTER(adc, ADC_SETTLE));
	}
}

uint32_t capture_adc_conversions(int adc)
{
	return sys_read32(ADC_REGISTER(adc, ADC_CONVERSIONS));
}

/* sdm_adc.vhd's registers at 740: 0 control (bit0 run, bits 11:8 log2 of the output
 * decimation, bits 15:12 the bit rate filter's k), 2 read phase, 3 results, 7 the period mean's
 * window in bits (the output bridge's period: 500, 25 us) */
#define SDM_REGISTER(r) (REGISTERS + 4 * 740 + 4 * (r))
#define SDM_FILTER_K 4

void capture_set_sdm(uint32_t log2, uint32_t phase)
{
	sys_write32(BIT(0) | (log2 << 8) | (SDM_FILTER_K << 12), SDM_REGISTER(0));
	sys_write32(500, SDM_REGISTER(7));   /* the period mean's window: the output bridge's period */
	sys_write32(phase, SDM_REGISTER(2));
}

uint32_t capture_sdm_results(void)
{
	return sys_read32(SDM_REGISTER(3));
}

/* gate_power.vhd's registers at 760: 0 enables, 5 the supplies on now. The FPGA brings the
 * supplies up one at a time and phase shifts their PWMs; nothing here needs to. */
#define GATE_POWER_REGISTER(r) (REGISTERS + 4 * 760 + 4 * (r))

void capture_set_gate_power(uint32_t enables)
{
	sys_write32(enables & 0x3f, GATE_POWER_REGISTER(0));
}

uint32_t capture_gate_power_on(void)
{
	return sys_read32(GATE_POWER_REGISTER(5)) & 0x3f;
}

/* dhb_test.vhd's registers at 780: 0 W bit0 start, bit1 stop / R status, 1 burst periods,
 * 2 pulse width, 3 periods of the last burst, 4 periods a ramp step */
#define DHB_REGISTER(r) (REGISTERS + 4 * 780 + 4 * (r))

void capture_dhb_test(uint32_t periods, uint32_t pulse_clocks, uint32_t ramp_periods,
		      uint32_t run_periods, bool inverted)
{
	sys_write32(periods, DHB_REGISTER(1));
	sys_write32(pulse_clocks, DHB_REGISTER(2));
	sys_write32(ramp_periods, DHB_REGISTER(4));
	sys_write32(run_periods, DHB_REGISTER(6));
	sys_write32(inverted ? 1 : 0, DHB_REGISTER(7));
	sys_write32(BIT(0), DHB_REGISTER(0));
}

void capture_dhb_stop(void)
{
	sys_write32(BIT(1), DHB_REGISTER(0));
}

uint32_t capture_dhb_status(void)
{
	return sys_read32(DHB_REGISTER(0));
}

uint32_t capture_dhb_periods(void)
{
	return sys_read32(DHB_REGISTER(3));
}

uint32_t capture_dhb_run_periods(void)
{
	return sys_read32(DHB_REGISTER(8));
}

/* measurement_scaler.vhd's registers at 900: i the slope of channel i (30 fractional bits),
 * 32 + i its offset (20), 64 + i its scaled value (20) */
#define SCALER_REGISTER(r) (REGISTERS + 4 * 900 + 4 * (r))

void capture_set_scaling(int channel, double zero, double per_unit)
{
	double slope = 1.0 / per_unit;

	sys_write32((uint32_t)(int32_t)lround(slope * (1 << 30)), SCALER_REGISTER(channel));
	sys_write32((uint32_t)(int32_t)lround(-zero * slope * (1 << 20)), SCALER_REGISTER(32 + channel));
}

void capture_scaler_init(void)
{
	/* the scope pages' default scaling (web/plots.js) */
	static const struct {
		int channel;
		double zero, per_unit;
	} calibrations[] = {
		{SCALER_SEC_VDC_OUT, -15.197, 16.857480},       /* sec 0 vdc_out, the primary's divider */
		{SCALER_SEC_VFILT_OUT, 8153.273, -16.327273},   /* sec 2 vfilt_out, as vfilt_in */
		{SCALER_PRI_VDC_IN, -15.197, 16.857480},        /* pri 1 vdc_in */
		{SCALER_SDM_IAC_OUT, -331.6, -256.0},           /* sdm iac_out, 0.5 mOhm, + into the filter */
		{SCALER_SDM_I_DAB, -197.4, -256.0},             /* sdm i_dab */
		{SCALER_IAC_OUT_SAMPLED, -331.6, -256.0},       /* iac_out's period mean, sampled */
	};

	for (size_t i = 0; i < ARRAY_SIZE(calibrations); i++) {
		capture_set_scaling(calibrations[i].channel, calibrations[i].zero,
				    calibrations[i].per_unit);
	}
}

double capture_scaled(int channel)
{
	return (int32_t)sys_read32(SCALER_REGISTER(64 + channel)) / (double)(1 << 20);
}

/* dhb_voltage_controller.vhd's registers at 1100: 0 enable, 1 kp, 2 ki, 3 upper limit, 4 lower
 * limit, 5 scale (20 fractional bits), 6 the phase, 7 updates, 8 the command */
#define DHB_CONTROL_REGISTER(r) (REGISTERS + 4 * 1100 + 4 * (r))

static uint32_t to_fixed(double v)
{
	return (uint32_t)(int32_t)lround(v * (1 << 20));
}

void capture_set_dhb_control(bool enable, double kp, double ki, double limit_high, double limit_low)
{
	sys_write32(to_fixed(kp), DHB_CONTROL_REGISTER(1));
	sys_write32(to_fixed(ki), DHB_CONTROL_REGISTER(2));
	sys_write32(to_fixed(limit_high), DHB_CONTROL_REGISTER(3));
	sys_write32(to_fixed(limit_low), DHB_CONTROL_REGISTER(4));
	sys_write32(enable ? 1 : 0, DHB_CONTROL_REGISTER(0));
}

double capture_dhb_command(void)
{
	return (int32_t)sys_read32(DHB_CONTROL_REGISTER(8)) / (double)(1 << 20);
}

int capture_dhb_phase(void)
{
	return (int32_t)sys_read32(DHB_CONTROL_REGISTER(6));
}

uint32_t capture_dhb_updates(void)
{
	return sys_read32(DHB_CONTROL_REGISTER(7));
}

/* dhb_protection.vhd's registers at 1120: 0 overcurrent limit, 1 overvoltage limit, 2 trips /
 * clear, 3 the current, 4 the voltage that tripped (20 fractional bits) */
#define PROTECTION_REGISTER(r) (REGISTERS + 4 * 1120 + 4 * (r))

void capture_set_dhb_protection(double overcurrent, double overvoltage, uint32_t readings)
{
	sys_write32(to_fixed(overcurrent), PROTECTION_REGISTER(0));
	sys_write32(to_fixed(overvoltage), PROTECTION_REGISTER(1));
	sys_write32(readings, PROTECTION_REGISTER(5));
}

void capture_set_dhb_filter(uint32_t n)
{
	sys_write32(n, DHB_CONTROL_REGISTER(9));
}

double capture_dhb_filtered(int secondary)
{
	return (int32_t)sys_read32(DHB_CONTROL_REGISTER(secondary ? 11 : 10)) / (double)(1 << 20);
}

uint32_t capture_dhb_trips(void)
{
	return sys_read32(PROTECTION_REGISTER(2)) & 3;
}

double capture_dhb_trip_current(void)
{
	return (int32_t)sys_read32(PROTECTION_REGISTER(3)) / (double)(1 << 20);
}

double capture_dhb_trip_voltage(void)
{
	return (int32_t)sys_read32(PROTECTION_REGISTER(4)) / (double)(1 << 20);
}

void capture_dhb_clear_trips(void)
{
	sys_write32(1, PROTECTION_REGISTER(2));
}

/* output_observer.vhd's registers at 1160: 0 enable, 1..12 the coefficients (24 fractional
 * bits): phi11 phi12 phi21 phi22, gamma11 gamma12 gamma21 gamma22, kp_i, ki_i dt, kp_v, ki_v dt;
 * 13..16 the estimates i, v, du, iload (20 fractional bits), 17 steps. The coefficients as
 * output_observer_pkg's observer_design and observer_coefficients compute them: the filter's
 * zero order hold over a step, its matrix exponential and integral by their series */
#define OBSERVER_REGISTER(r) (REGISTERS + 4 * 1160 + 4 * (r))
/* the output bridge's period (output_bridge_pkg.vhd): 500 sigma-delta bits, 25 us, 40 kHz */
#define OUTPUT_PERIOD_BITS 500
#define OUTPUT_PERIOD (OUTPUT_PERIOD_BITS * 50.0e-9)
#define OBSERVER_STEP (OUTPUT_PERIOD / 2.0)
#define OBSERVER_TWO_PI 6.283185307179586

static void matrix_multiply(const double a[2][2], const double b[2][2], double out[2][2])
{
	double r[2][2];

	for (int m = 0; m < 2; m++) {
		for (int n = 0; n < 2; n++) {
			r[m][n] = a[m][0] * b[0][n] + a[m][1] * b[1][n];
		}
	}
	memcpy(out, r, sizeof(r));
}

void capture_set_observer(bool enable, double l, double c, double r, double current_hz, double voltage_hz)
{
	const double dt = OBSERVER_STEP, zeta = 0.7;
	const double wi = OBSERVER_TWO_PI * current_hz, wv = OBSERVER_TWO_PI * voltage_hz;
	const double a[2][2] = {{-r / l * dt, -1.0 / l * dt}, {1.0 / c * dt, 0.0}};
	const double b[2][2] = {{1.0 / l, 0.0}, {0.0, -1.0 / c}};
	double term[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
	double phi[2][2] = {{1.0, 0.0}, {0.0, 1.0}};
	double integral[2][2] = {{dt, 0.0}, {0.0, dt}};
	double gamma[2][2];

	for (int k = 1; k <= 30; k++) {
		matrix_multiply(term, a, term);
		for (int m = 0; m < 2; m++) {
			for (int n = 0; n < 2; n++) {
				term[m][n] /= k;
				phi[m][n] += term[m][n];
				integral[m][n] += term[m][n] * dt / (k + 1);
			}
		}
	}
	matrix_multiply(integral, b, gamma);

	const double coefficients[12] = {
		phi[0][0], phi[0][1], phi[1][0], phi[1][1],
		gamma[0][0], gamma[0][1], gamma[1][0], gamma[1][1],
		2.0 * zeta * wi * l - r, l * wi * wi * dt,   /* the current's PI */
		2.0 * zeta * wv * c, c * wv * wv * dt,       /* the voltage's */
	};

	/* written while it runs: a config message must not reset it (its integrals hold the
	 * bridge's error between runs) */
	for (int n = 0; n < 12; n++) {
		sys_write32((uint32_t)(int32_t)lround(coefficients[n] * (1 << 24)), OBSERVER_REGISTER(1 + n));
	}
	sys_write32(enable ? 1 : 0, OBSERVER_REGISTER(0));
}

double capture_observer(int estimate)
{
	return (int32_t)sys_read32(OBSERVER_REGISTER(13 + estimate)) / (double)(1 << 20);
}

uint32_t capture_observer_steps(void)
{
	return sys_read32(OBSERVER_REGISTER(17));
}

/* output_voltage_control.vhd's registers at 1180: 0 enable, 1 kp_v b, 2 kp_v, 3 ki_v period,
 * 4 kp_i, 5 ki_i period, 6 R, 7 du's low pass (24 fractional bits), 8..10 the limits (20);
 * 11 i_ref, 12 vl_ref, 13 u_ref, 14 the predicted voltage, 15 the limits' flags, 16 steps,
 * 17 the reference's prefilter, 18 its slope's feedforward (24 fractional bits).
 * The gains as output_voltage_control_pkg's voltage_control_design: the current PI's zero a
 * fifth of its bandwidth, the voltage PI's voltage_zero of its (the observer feeds the load
 * current forward: a small integral, no slow tail after a step) */
#define CONTROL_REGISTER(r) (REGISTERS + 4 * 1180 + 4 * (r))
#define CONTROL_PERIOD OUTPUT_PERIOD

void capture_set_control(bool enable, double l, double c, double r, double current_hz, double voltage_hz,
			 double b, double voltage_zero, double du_hz, double i_max, double vl_max, double u_max,
			 double prefilter_hz, double feedforward)
{
	/* the prefilter's stages a period, its slope's feedforward C alpha / period (none at 0 Hz) */
	const double alpha = prefilter_hz > 0.0 ? 1.0 - exp(-OBSERVER_TWO_PI * prefilter_hz * CONTROL_PERIOD) : 1.0;
	const double gain = prefilter_hz > 0.0 ? feedforward * c * alpha / CONTROL_PERIOD : 0.0;
	const double wi = OBSERVER_TWO_PI * current_hz, wv = OBSERVER_TWO_PI * voltage_hz;
	const double kp_i = l * wi, ki_i = kp_i * wi / 5.0;
	const double kp_v = c * wv, ki_v = kp_v * wv * voltage_zero;
	const double gains[7] = {kp_v * b, kp_v, ki_v * CONTROL_PERIOD, kp_i, ki_i * CONTROL_PERIOD, r,
				 OBSERVER_TWO_PI * du_hz * CONTROL_PERIOD};

	for (int n = 0; n < 7; n++) {
		sys_write32((uint32_t)(int32_t)lround(gains[n] * (1 << 24)), CONTROL_REGISTER(1 + n));
	}
	sys_write32(to_fixed(i_max), CONTROL_REGISTER(8));
	sys_write32(to_fixed(vl_max), CONTROL_REGISTER(9));
	sys_write32(to_fixed(u_max), CONTROL_REGISTER(10));
	sys_write32((uint32_t)(int32_t)lround(alpha * (1 << 24)), CONTROL_REGISTER(17));
	sys_write32((uint32_t)(int32_t)lround(gain * (1 << 24)), CONTROL_REGISTER(18));
	sys_write32(enable ? 1 : 0, CONTROL_REGISTER(0));
}

double capture_control(int value)
{
	return (int32_t)sys_read32(CONTROL_REGISTER(11 + value)) / (double)(1 << 20);
}

uint32_t capture_control_flags(void)
{
	return sys_read32(CONTROL_REGISTER(15)) & 7;
}

uint32_t capture_control_steps(void)
{
	return sys_read32(CONTROL_REGISTER(16));
}

/* output_bridge_test.vhd's registers at 1140: 0 W bit0 arm, bit1 stop / R state, 1 duty before,
 * 2 duty after (20 fractional bits), 3 periods before the step, 4 periods in all, 5 dead time,
 * 6 periods run */
#define OUTPUT_REGISTER(r) (REGISTERS + 4 * 1140 + 4 * (r))

void capture_output_test(double before, double after, uint32_t step_periods, uint32_t periods)
{
	sys_write32(0, OUTPUT_REGISTER(7));
	sys_write32(to_fixed(before), OUTPUT_REGISTER(1));
	sys_write32(to_fixed(after), OUTPUT_REGISTER(2));
	sys_write32(step_periods, OUTPUT_REGISTER(3));
	sys_write32(periods, OUTPUT_REGISTER(4));
	sys_write32(BIT(0), OUTPUT_REGISTER(0));
}

/* registers 7 bit0 voltage mode, 8 and 9 the voltages before and after the step */
void capture_output_voltage_test(double before, double after, uint32_t step_periods, uint32_t periods)
{
	sys_write32(BIT(0), OUTPUT_REGISTER(7));
	sys_write32(to_fixed(before), OUTPUT_REGISTER(8));
	sys_write32(to_fixed(after), OUTPUT_REGISTER(9));
	sys_write32(step_periods, OUTPUT_REGISTER(3));
	sys_write32(periods, OUTPUT_REGISTER(4));
	sys_write32(BIT(0), OUTPUT_REGISTER(0));
}

/* registers 7 bit1 closed loop, 10 the square wave's amplitude, 11 its half cycle */
void capture_output_control_test(double amplitude, uint32_t half_periods, uint32_t periods)
{
	sys_write32(BIT(1), OUTPUT_REGISTER(7));
	sys_write32(to_fixed(amplitude), OUTPUT_REGISTER(10));
	sys_write32(half_periods, OUTPUT_REGISTER(11));
	sys_write32(periods, OUTPUT_REGISTER(4));
	sys_write32(BIT(0), OUTPUT_REGISTER(0));
}

/* waveform_player.vhd's registers at 1200: 0 the points, 1 the passes, 2 the index the next
 * entry goes to, 3 its voltage (20 fractional bits), 4 its periods (writes the entry, the index
 * on), 5 the state, 6 the passes done */
#define WAVEFORM_REGISTER(r) (REGISTERS + 4 * 1200 + 4 * (r))
#define WAVEFORM_POINTS      512

uint32_t capture_waveform_capacity(void)
{
	return WAVEFORM_POINTS;
}

void capture_waveform_load(const float *volts, const uint32_t *periods, uint32_t count, uint32_t passes)
{
	count = MIN(count, WAVEFORM_POINTS);
	sys_write32(MAX(count, 1), WAVEFORM_REGISTER(0));
	sys_write32(passes, WAVEFORM_REGISTER(1));
	sys_write32(0, WAVEFORM_REGISTER(2));
	for (uint32_t n = 0; n < count; n++) {
		sys_write32(to_fixed(volts[n]), WAVEFORM_REGISTER(3));
		sys_write32(MAX(periods[n], 1), WAVEFORM_REGISTER(4));
	}
}

uint32_t capture_waveform_status(void)
{
	return sys_read32(WAVEFORM_REGISTER(5));
}

/* registers 7 bit1 closed loop and bit2 its reference the waveform's */
void capture_output_waveform_test(uint32_t periods)
{
	sys_write32(BIT(1) | BIT(2), OUTPUT_REGISTER(7));
	sys_write32(periods, OUTPUT_REGISTER(4));
	sys_write32(BIT(0), OUTPUT_REGISTER(0));
}

void capture_output_stop(void)
{
	sys_write32(BIT(1), OUTPUT_REGISTER(0));
}

uint32_t capture_output_status(void)
{
	return sys_read32(OUTPUT_REGISTER(0)) & 15;
}

/* input_bridge_test.vhd's registers at 1240: 0 start/stop and state, 1 the switch, 2 the duty,
 * 3 the burst's periods, 4 the overvoltage stop, 5 periods run, 6 and 7 the link at the start and
 * its highest */
#define INPUT_REGISTER(r) (REGISTERS + 4 * 1240 + 4 * (r))

void capture_input_test(uint32_t which, double duty, uint32_t periods, double limit)
{
	sys_write32(which & 3, INPUT_REGISTER(1));
	sys_write32(to_fixed(duty), INPUT_REGISTER(2));
	sys_write32(periods, INPUT_REGISTER(3));
	sys_write32(to_fixed(limit), INPUT_REGISTER(4));
	sys_write32(BIT(0), INPUT_REGISTER(0));
}

void capture_input_stop(void)
{
	sys_write32(BIT(1), INPUT_REGISTER(0));
}

void capture_input_state(uint32_t state[4])
{
	state[0] = sys_read32(INPUT_REGISTER(0)) & 7;
	state[1] = sys_read32(INPUT_REGISTER(5));
	state[2] = sys_read32(INPUT_REGISTER(6));
	state[3] = sys_read32(INPUT_REGISTER(7));
}

/* register 12: the output's watchdog kicked (13 its time, 1 s after a reset) */
void capture_output_kick(void)
{
	sys_write32(1, OUTPUT_REGISTER(12));
}

uint32_t capture_output_periods(void)
{
	return sys_read32(OUTPUT_REGISTER(6));
}

uint32_t capture_overflows(void)
{
	return reg_read(REG_OVERFLOWS);
}

static int run_locked(const struct scope_config *config, bool force, int16_t *samples,
		      struct scope_capture *capture);

/* -EBUSY while the stream has the scope */
int capture_run(const struct scope_config *config, bool force, int16_t *samples,
		struct scope_capture *capture)
{
	int ret = -EBUSY;

	k_mutex_lock(&hw_lock, K_FOREVER);
	if (!streaming) {
		ret = run_locked(config, force, samples, capture);
	}
	k_mutex_unlock(&hw_lock);
	return ret;
}

static int run_locked(const struct scope_config *config, bool force, int16_t *samples,
		      struct scope_capture *capture)
{
	uint32_t status, start, length;
	int64_t until;

	if (!armed || memcmp(&armed_config, config, sizeof(*config)) != 0) {
		arm(config);
	}
	if (force) {
		reg_write(REG_COMMAND, COMMAND_FORCE);
	}

	/* a short wait for the trigger, the caller comes back */
	until = k_uptime_get() + 20;
	do {
		status = reg_read(REG_COMMAND);
		if ((status & STATE_MASK) == STATE_DONE) {
			break;
		}
		k_msleep(1);
	} while (k_uptime_get() < until);
	if ((status & STATE_MASK) != STATE_DONE) {
		return -EAGAIN;
	}
	armed = false;

	capture->lost = reg_read(REG_OVERFLOWS);

	/* the last records are in the memory microseconds after the scope is done */
	k_busy_wait(20);
	dcache_invalidate();
	start = reg_read(REG_START_RECORD) & (RING_RECORDS - 1);
	length = MIN(config->length, SCOPE_MAX_LENGTH);
	/* the capture in at most two blocks: the ring wraps at most once */
	uint32_t first = MIN(length, RING_RECORDS - start);

	memcpy(samples, ring[start], first * sizeof(ring[0]));
	memcpy(&samples[first * SCOPE_CHANNELS], ring[0], (length - first) * sizeof(ring[0]));
	capture->length = length;
	capture->pre = MIN(length * config->pre_percent / 100, length - 1);
	capture->forced = (status & STATUS_FORCED) != 0;
	return 0;
}

/* the source names from the scope's probe table */
int capture_init(void)
{
	uint32_t information = reg_read(REG_INFORMATION);
	uint32_t count = MIN(information >> 24, SCOPE_SOURCES);

	for (uint32_t s = 0; s < SCOPE_SOURCES; s++) {
		char *name = names[s];

		name[0] = '\0';
		if (s < count) {
			reg_write(REG_PROBE_INDEX, s);
			for (int w = 0; w < 4; w++) {
				uint32_t word = reg_read(REG_PROBE_NAME + w);

				for (int b = 0; b < 4; b++) {
					name[4 * w + b] = (char)(word >> (24 - 8 * b));
				}
			}
			name[16] = '\0';
		}
		scope_source_names[s] = name[0] ? name : "-";
	}
	LOG_INF("scope_capture: %u sources, %u channels, %u samples deep in block RAM",
		information >> 24, (information >> 16) & 0xff, information & 0xffff);
	return 0;
}
