# Open items

Known problems, unfinished work and plans, newest findings first in each section. Remove an
item when it is done; the commit that does it says so.

## Input bridge (boost from a DC source)

- **The boost does not regulate on the board.** `input_boost_control.vhd` (registers 1260)
  passes its testbench, but on the board iac_in's period mean (about 0.1 A at 4 % duty) does
  not follow the current that charges the primary DC link (its rise needs about 0.5 A or more
  at the estimated 1.3 mF), so the current loop winds the duty up to d_max and the link
  overshoots to the overvoltage stop. Likely cause: the capacitor that compensates the
  iac_in shunt's stray inductance is missing. Blocked until it is fitted.
- When resuming:
  1. check that iac_in's period mean matches C dv/dt with a short 1 MS/s capture at low duty;
  2. add a peak current cutoff on the raw sigma-delta current, and start with d_max about 0.05;
  3. move the control to the microprogram processor with observers, as the output voltage
     control is, to deal with the measurement noise. Keep the current loop on the bridge's DC
     side current, iac_in * (vac_in / vdc_in), and the voltage loop slow (well below 100 Hz) so
     an AC input's 100 Hz ripple needs no new voltage controller.
- iac_in's sign: a 1 MS/s capture shows the boost's current pulses as positive counts (the
  firmware calibrates +256 counts/A), while the polarity test (`input_polarity_test.py`)
  reported its period mean negative (-1.72 A). Unexplained; recheck after the capacitor.
- The input inductance is not measured: about 10-15 uH from the current pulses' slopes. The
  primary DC link's 1.3 mF is an estimate from the polarity test's current reading, which is
  in doubt (above).
- The primary bypass relay stays open for the input tests.

## Output voltage control

- First half cycle after a start: the output holds about -18.6 V while the gate signals average
  about -2 V and the current stays about -0.5 A, decaying over about 9 ms. Physically
  inconsistent: either vfilt_out reads wrong then, or something between the gate signals and
  the output adds about -17 V. Same with fresh gate supplies, a later arm, or any leftover
  output voltage.
- The real output settles over about 15 ms at each run's start even with du held; the
  observer's slow voltage loop is the suspect. The observer also drifts after a stop.
- iac_out's period mean has an offset while switching (about -0.18 A at duty 0.6, up to
  +-0.3 A in closed loop), set by where the PWM edges fall against the ADuM7702's own clock.
  It repeats run to run; calibrate it per duty or pick the least disturbing start alignment.
- du's fast part swings; consider a 1-1.5 kHz observer current loop. Spikes in the
  sigma-delta stream.
- The grid generator's 5th harmonic comes out low (5.1-5.4 % for 6 %), possibly amplitude
  dependent. The loop response table must be remeasured whenever the tuning changes
  (`firmware/apps/scope/measure_loop_response.py --update`, or the waveform page's Bode
  section).

## Measurement noise and the shared carrier

- Plan: drive the DHB and the output bridge from one sawtooth carrier, both switching
  symmetrically, so the DC links and other ADC readings can be sampled where neither bridge
  switches. vfilt_out and vdc_out carry +-20-30 V of switching noise now.
- The FPGA is about 90 % full (XLRs); make room before adding much more logic.

## Waveform generator

- The point table holds 512 points; about 670 kbit of block RAM is free for more (about 10 k
  points). Loading 10 k points as JSON is about 120 kB, so the load path may need HTTP
  instead of the WebSocket.
- Status messages sometimes arrive seconds late over the network: timestamps taken from them
  are unreliable.

## Face panel LCD

- `lcd_display.vhd` (registers 1300) shows a screen of text: the IP address, the last touch,
  and the firmware's and the FPGA image's commits ("+": built from a changed tree).
  `touch_reader.vhd` (registers 1320) polls the touch controller (no pen interrupt connected;
  its data out on GPIOR_22). The firmware's panel.c calibrates it to screen pixels
  (`{"type": "touch_calibrate"}`: four "+" targets, an affine fit kept in flash). Next: a stop
  button that works without the network (in the FPGA, straight from the touch), then start
  buttons and a system state indicator.
- Efinity numbers the bits of an unconstrained record field (the memory library's RAM output)
  the other way from the simulator: indexing a bit of it directly mirrored each character on
  the board while the testbench passed. Copy such a vector whole into a constrained one before
  indexing bits.

## Board, images and network

- The golden image in flash is old: it still has the Efinix Ethernet and drives GPIOR_21 (the
  output leg 2 high side flywire) weakly high, so keep the gate supplies off on it. Rebuilding
  and flashing it needs JTAG, with the power stage off.
- A network FPGA reload from the application image sometimes falls back to golden; power
  cycling and reinstalling the same file has always worked.
- The PHY sometimes stops answering MDIO; replugging the cable helps.
- A page refresh sometimes takes about 10 s. Suspected: the HTTP server's 6 client limit with
  its 10 s inactivity timeout. Not reproduced.
