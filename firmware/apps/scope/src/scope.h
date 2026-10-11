/*
 * Oscilloscope served to a browser: the web client of fpga/scripts/scope talks to
 * this app over the /ws WebSocket exactly as it talks to scope_server.py.
 *
 * The capture itself is behind capture_run(). On the Ti60 kit it is emulated in software
 * (capture_sim.c) with the test signals of ti60_ac_psu's test_signal_generator.vhd; on
 * the PSU board's tsemac_soc design it is the FPGA's scope_capture (capture_hw.c).
 */
#ifndef SCOPE_H
#define SCOPE_H

#include <stdbool.h>
#include <stdint.h>

#define SCOPE_CHANNELS   8
#define SCOPE_SOURCES    64
#define SCOPE_MAX_LENGTH 32768
#define SCOPE_CLOCK_HZ   125000000.0

enum scope_mode {
	SCOPE_MODE_AUTO,   /* trigger, forced when none comes in time */
	SCOPE_MODE_NORMAL, /* wait for the trigger */
	SCOPE_MODE_FREE,   /* no trigger */
};

enum scope_edge {
	SCOPE_EDGE_RISING,
	SCOPE_EDGE_FALLING,
	SCOPE_EDGE_EITHER,
};

#define SCOPE_ADCS 2

struct scope_config {
	uint32_t divider; /* a sample every divider+1 clocks */
	uint32_t length;  /* samples per capture */
	uint32_t pre_percent;
	uint32_t trigger_channel;
	enum scope_edge trigger_edge;
	int32_t trigger_level;
	enum scope_mode mode;
	uint8_t sources[SCOPE_CHANNELS];
	double generator_hz;
	/* the mux positions each ADC (sec, pri) converts in turn: the fewer, the more often */
	uint8_t adc_masks[SCOPE_ADCS];
	/* how long each ADC's mux settles after it switches, before the next sample */
	uint32_t adc_settle_ns[SCOPE_ADCS];
	/* the sigma-delta modulators' filters: decimation 2^sdm_log2 (5..8), and the phase of
	 * the modulator clock period at which their bits are read (0..9 at 20 MHz) */
	uint32_t sdm_log2;
	uint32_t sdm_phase;
	/* the gate driver supplies asked on, bit n-1 for supply n (they also power the
	 * sigma-delta modulators); the FPGA brings them up one at a time */
	uint32_t gate_power;
	/* the DHB's voltage control (dhb_voltage_controller.vhd): gains of the command (+-1.0 a
	 * quarter period of phase) per volt, its limits, and whether it runs */
	double dhb_kp;
	double dhb_ki;
	double dhb_limit_high;
	double dhb_limit_low;
	bool dhb_control;
	/* the DHB's trips (dhb_protection.vhd): the current's and the DC links' limits */
	double dhb_overcurrent;
	double dhb_overvoltage;
	uint32_t dhb_overvoltage_readings; /* in a row over the limit to trip */
	uint32_t dhb_filter;               /* the DC links' filters: 2^n readings of time constant */
	/* the output filter's observer (output_observer.vhd): the filter's L (H), C (F), R (ohm)
	 * and its PIs' bandwidths (Hz) */
	bool observer;
	double observer_l, observer_c, observer_r;
	double observer_current_hz, observer_voltage_hz;
	/* the output voltage's control (output_voltage_control.vhd): the loops' bandwidths (Hz),
	 * the voltage PI's setpoint weight and its zero (a share of the voltage loop's bandwidth),
	 * du's low pass (Hz), the limits: the inductor current (A), its voltage and the bridge's
	 * (V) */
	bool control;
	double control_current_hz, control_voltage_hz, control_b, control_voltage_zero, control_du_hz;
	/* the reference's prefilter before the voltage PI (Hz, two stages; 0: none) and the share
	 * of its slope fed forward as the capacitor's current (0..1) */
	double control_prefilter_hz, control_feedforward;
	double control_i_max, control_vl_max, control_u_max;
};

struct scope_capture {
	uint32_t length;
	uint32_t pre; /* samples before the trigger */
	bool forced;
	uint32_t lost;    /* samples the memory did not take since the arm: broken if not 0 */
};

/* names of the sources, set by capture_init() */
extern const char *scope_source_names[SCOPE_SOURCES];

int capture_init(void);

/*
 * Take one capture with the given configuration into samples, sample-major: all channels
 * of sample 0, then sample 1, ... Returns 0 when captured, -EAGAIN when the trigger has
 * not come yet; force takes the capture without it.
 */
int capture_run(const struct scope_config *config, bool force, int16_t *samples,
		struct scope_capture *capture);

/* WebSocket connection accepted by the HTTP server */
int scope_client_add(int ws_socket);

/* the settings the browser has made */
void scope_get_config(struct scope_config *config);

/* the settings kept in flash from before a restart, if any; after capture_init() */
void scope_settings_load(void);

/*
 * The stream (stream.c, FPGA capture only): the FPGA captures without end into a ring of
 * SCOPE_STREAM_RING_SEGMENTS segments of SCOPE_STREAM_SEGMENT_RECORDS records, which a
 * WebSocket on port SCOPE_STREAM_PORT sends as it fills, SCOPE_STREAM_SEGMENTS to a message.
 * Triggered captures pause while a client takes it.
 */
#define SCOPE_STREAM_PORT            5004
#define SCOPE_STREAM_SEGMENT_RECORDS 91     /* 1456 bytes: with a frame header, one TCP segment */
#define SCOPE_STREAM_SEGMENTS        720    /* a message: 65520 records, 1 MB */
#define SCOPE_STREAM_RING_SEGMENTS   (10 * SCOPE_STREAM_SEGMENTS)  /* 10 MB, 168 ms at 3.9 MS/s */

int scope_stream_init(void);
bool scope_stream_active(void);
uint32_t scope_stream_lost(void);

/* capture_hw.c: the continuous capture behind the stream */
const int16_t *capture_stream_start(const struct scope_config *config);
void capture_stream_stop(void);
uint32_t capture_stream_written(void);  /* records in the ring since the start, 32 bits */
uint32_t capture_overflows(void);       /* records the DMA did not take in time */

/* the control board's ADCs (ad_subsystem.vhd): the mux positions they convert, and their
 * conversions since power up, 32 bits; none (0) without them */
void capture_set_adc_masks(const uint8_t masks[SCOPE_ADCS]);
void capture_set_adc_settle(const uint32_t settle_ns[SCOPE_ADCS]);
uint32_t capture_adc_conversions(int adc);
void capture_set_sdm(uint32_t log2, uint32_t phase);
uint32_t capture_sdm_results(void);     /* filter results since power up, 32 bits */
void capture_set_gate_power(uint32_t enables);
uint32_t capture_gate_power_on(void);   /* the supplies on now */

/* the DHB's precharge test (dhb_test.vhd): a burst of the primary leg alone, periods of 150 kHz
 * long, which the FPGA ends itself, its pulses pulse_clocks of 200 MHz wide at the start and a
 * clock wider every ramp_periods (0 no ramp; 124 reaches nominal in 500 ms); the FPGA limits
 * them all */
void capture_dhb_test(uint32_t periods, uint32_t pulse_clocks, uint32_t ramp_periods,
		      uint32_t run_periods, bool inverted);
uint32_t capture_dhb_run_periods(void); /* run in the last burst */
void capture_dhb_stop(void);
/* bits 1:0 the bridge's mode (0 off, 1 precharging, 2 running, 3 failed), bit 8 a burst */
uint32_t capture_dhb_status(void);
uint32_t capture_dhb_periods(void);     /* of the last burst */

/* the FPGA's measurement scaler (measurement_scaler.vhd): every ADC measurement to fixed point
 * (20 fractional bits) as value = (counts - zero) / per_unit. Channels 0..7 the sec mux
 * positions, 8..15 the pri ones, 16..18 the sigma-delta currents. capture_scaler_init writes
 * the board's calibrations, the scope pages' defaults; the others read 0. */
/* the measurement scaler's channels (scaler_channels_pkg.vhd): only those the FPGA's controls and
 * protections use */
#define SCALER_SEC_VDC_OUT       0
#define SCALER_SEC_VFILT_OUT     1
#define SCALER_PRI_VDC_IN        2
#define SCALER_SDM_IAC_OUT       3
#define SCALER_SDM_I_DAB         4
#define SCALER_IAC_OUT_SAMPLED   5
#define SCALER_IAC_IN_MEAN       6
#define SCALER_PRI_VFILT_IN      7
#define SCALER_PRI_VAC_IN        8
void capture_set_scaling(int channel, double zero, double per_unit);
void capture_scaler_init(void);
double capture_scaled(int channel);

/* the DHB's voltage control: enable, gains and limits; its command (-1.0 .. 1.0), the phase in
 * carrier counts and its updates */
void capture_set_dhb_control(bool enable, double kp, double ki, double limit_high, double limit_low);
double capture_dhb_command(void);
int capture_dhb_phase(void);
uint32_t capture_dhb_updates(void);

/* the DHB's trips: limits, bit0 overcurrent / bit1 overvoltage tripped, the values that
 * tripped, and clearing them */
void capture_set_dhb_protection(double overcurrent, double overvoltage, uint32_t readings);
void capture_set_dhb_filter(uint32_t n);
/* the observer: its coefficients from the filter and the bandwidths (a step of half the
 * output bridge's period, 12.5 us), and its estimates: 0 i (A), 1 v (V), 2 du (V), 3 iload (A) */
void capture_set_observer(bool enable, double l, double c, double r, double current_hz, double voltage_hz);
double capture_observer(int estimate);
uint32_t capture_observer_steps(void);
/* the output voltage's control: its gains from the filter (the observer's L, C, R) and the
 * loops' bandwidths; its status: 0 i_ref (A), 1 vl_ref (V), 2 u_ref (V), 3 the predicted
 * voltage (V); its limits' flags (bit0 current, 1 inductor voltage, 2 bridge) and steps */
void capture_set_control(bool enable, double l, double c, double r, double current_hz, double voltage_hz,
			 double b, double voltage_zero, double du_hz, double i_max, double vl_max, double u_max,
			 double prefilter_hz, double feedforward);
double capture_control(int value);
uint32_t capture_control_flags(void);
uint32_t capture_control_steps(void);
/* the output bridge in closed loop: the voltage controlled to a square wave of +-amplitude
 * (V), half_periods switching periods each half, periods in all; armed as capture_output_test */
void capture_output_control_test(double amplitude, uint32_t half_periods, uint32_t periods);
/* the output voltage's waveform (waveform_player.vhd): its table's points, 0 without one; a
 * table of count points loaded (volts, periods of 25 us each), played passes times (0: until
 * stopped); the closed loop on it for at most periods, 0 until stopped (it ends when the
 * waveform does), armed as capture_output_test (the DHB runs on while the output runs); the
 * player's state: bit0 playing, bit1 finished, bits 31:16 the point held */
uint32_t capture_waveform_capacity(void);
void capture_waveform_load(const float *volts, const uint32_t *periods, uint32_t count, uint32_t passes);
void capture_output_waveform_test(uint32_t periods);
uint32_t capture_waveform_status(void);
double capture_dhb_filtered(int secondary); /* the filtered DC link, 0 primary, 1 secondary */
uint32_t capture_dhb_trips(void);
double capture_dhb_trip_current(void);
double capture_dhb_trip_voltage(void);
void capture_dhb_clear_trips(void);

/* the output bridge's open loop step test (output_bridge_test.vhd): duties before and after the
 * step (0..1, 0.5 is 0 V), periods of 40 kHz before the step and in all; armed, it starts when
 * the DHB runs. Its state: bit0 running, bit1 stepped, bit2 armed */
void capture_output_test(double before, double after, uint32_t step_periods, uint32_t periods);
/* the same in voltage mode: output voltages before and after the step (V), the duty from them and
 * the secondary DC link (voltage_modulator.vhd) */
void capture_output_voltage_test(double before, double after, uint32_t step_periods, uint32_t periods);
void capture_output_stop(void);
/* the output's closed loop on the waveform set up (mode and periods, 0 until stopped), not
 * armed: the supply sequencer arms it */
void capture_output_waveform_prepare(uint32_t periods);
/* the DHB test's settings as capture_dhb_test's, not started: the supply sequencer starts it */
void capture_dhb_prepare(uint32_t periods, uint32_t pulse_clocks, uint32_t ramp_periods,
			 uint32_t run_periods, bool inverted);
/* supply_sequencer.vhd (registers 1340): start (the supplies, the DHB, the output), stop,
 * power off; its register 0 (bits 3:0 its state, bit 8 the supplies held, 15:12 why the last
 * start ended) */
void capture_sequencer_start(void);
void capture_sequencer_stop(void);
void capture_sequencer_power_off(void);
uint32_t capture_sequencer_state(void);
/* the input bridge as a boost converter (input_boost_control.vhd, registers at 1260): a setting
 * by its register (1 v_ref, 2 kp_v, 3 ki_v, 4 kp_i, 5 ki_i, 6 i_max, 7 d_max, 8 the overvoltage
 * stop, 16 the ramp V/ms), start (when stopped), stop, the watchdog's kick; its state: [0] the
 * status (bit0 running, bit1..4 stopped by the overvoltage, a trip, the watchdog, inhibit), [1]
 * periods run */
void capture_boost_set(int reg, double value);
void capture_boost_start(void);
void capture_boost_stop(void);
void capture_boost_kick(void);
void capture_boost_state(uint32_t state[2]);
/* the face panel's LCD (lcd_display.vhd, registers at 1300): a screen of text, cols x rows;
 * a string into it at a place (row * cols + col), and a redraw */
void capture_lcd_text(uint32_t place, const char *text);
void capture_lcd_draw(void);
uint32_t capture_lcd_cols(void);
/* the touch screen (touch_reader.vhd, registers at 1320): [0] touched, [1] X and [2] Y of the
 * last touch (0..4095), [3] the pressure's Z1 */
void capture_touch_state(uint32_t state[4]);
/* the stop button's area in raw touch readings (touch_reader.vhd's registers 7..10), and its
 * presses so far */
void capture_touch_button(uint32_t x_from, uint32_t x_to, uint32_t y_from, uint32_t y_to);
uint32_t capture_touch_presses(void);
/* the commit the FPGA image was built from: bits 27..0 its hash's first 7 hex digits, bit 31 the
 * tree had changes */
uint32_t capture_fpga_version(void);

/* the face panel (panel.c): its screen kept up to date; the touch screen's calibration started
 * (from any thread) and whether it runs; a touch's screen pixels, false when not calibrated */
void panel_start(void);
void panel_calibrate(void);
bool panel_calibrating(void);
bool panel_touch_pixels(const uint32_t touch[4], int *x, int *y);

/* the network interface's state (the Ethernet driver's statistics; zeros without it) */
struct scope_net_state {
	uint32_t rx_frames, rx_dropped, rx_restarts, tx_frames, tx_errors;
	int64_t last_rx_ms;
	uint32_t mac_rx_ok, mac_tx_ok, mac_errors;
	uint32_t dma_rx_status, dma_tx_status;
	bool link_up;
	uint16_t link_mbps;
};
void capture_net_state(struct scope_net_state *state);
/* the /ws clients connected now */
int scope_client_count(void);
/* the network watchdog's restarts so far (main.c) */
uint32_t scope_net_resets(void);
/* 0 the current reference, 1 the duty, 2 the input current, 3 the link, 4 the bridge's output
 * current, 5 the input's voltage, 6 v_ref */
double capture_boost_value(int which);
/* the output's watchdog: a run is stopped (brought down) when this is not called for 1 s */
void capture_output_kick(void);
/* bit0 running, bit1 stepped, bit2 armed, bit3 the last run stopped by the watchdog */
uint32_t capture_output_status(void);
uint32_t capture_output_periods(void);

#endif
