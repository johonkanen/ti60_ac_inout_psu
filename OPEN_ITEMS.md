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
     control is, to deal with the measurement noise (decided: together, once the capacitor is
     in). Sharing the output's core: it is ~11 % busy (observer + control 270 clocks of the
     period's 3125); the boost's step ~150-250 clocks; 1/vdc by a Newton step a period instead
     of the lut_divider; program RAM to 1024 words; the PWM and every stop stay in logic;
     ~3k logic cells freed. Keep the current loop on the bridge's DC
     side current, iac_in * (vac_in / vdc_in), and the voltage loop slow (well below 100 Hz) so
     an AC input's 100 Hz ripple needs no new voltage controller.
- iac_in's sign: a 1 MS/s capture shows the boost's current pulses as positive counts (the
  firmware calibrates +256 counts/A), while the polarity test (`input_polarity_test.py`, removed
  to make room in the FPGA: it is in the history up to 877ce86) reported its period mean
  negative (-1.72 A). Unexplained; recheck after the capacitor.
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
- The FPGA is about 92 % full (XLRs) after removing the polarity test; make room before adding
  much more logic. Where it goes (the build's hier_util report): the Sapphire SoC, Ethernet,
  DMAs and HyperRAM ~44 %, the ADCs ~12 %, the scope ~9 % (nearly all its 8 x 34 source
  multiplexer, kept for full sample rates), the power control ~25 %.
- The scope's source names are a block RAM (4 blocks, saving only ~70 logic cells over the
  logic ROM they replaced): if RAM runs out, put them back in logic to free those blocks.

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
  (`{"type": "touch_calibrate"}`: four "+" targets, an affine fit kept in flash).
- The STOP button works in the FPGA alone (touch_reader's area, lcd_display's red rectangle):
  the output brought down and stopped, the DHB after it, the input boost off.
  Tested on the board in closed loop. Next: start buttons and a system state indicator.
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
- The PHY sometimes stops answering MDIO (every address reads 0xffff, the PHY driver fails at
  start). After a power cycle with the serial adapter (FT4232H) plugged in it stayed so: the
  adapter's TX line seems to keep the board partly powered. Power-cycle with the adapter's USB
  unplugged; replugging the Ethernet cable helped before.
- Quick page changes used to stop the network after 3..62 of them (every receive packet held,
  the board unreachable; the old "page refresh takes 10 s" was likely the same): Zephyr's
  websocket_unregister() writes to the websocket context after freeing it, and the HTTP server
  gives that context to the next page meanwhile. Fixed in scope.c (the sockets closed without
  it, a receive buffer for each client). Two more in scope.c: a client slot was reused before
  its thread had ended (creating a thread the kernel still listed corrupted its thread list: the
  HTTP server stopped, once the whole firmware hung), and a page gone without closing held its
  slot; now a slot waits for its thread to end, a client silent for 10 s is closed, and there
  are 6 slots. 503 page loads in the stress test since, none slow. The network watchdog
  (main.c) restarts the firmware should the receive packets stay used up for 10 s; its restarts
  show on the face panel ("r"). The websocket race is worth reporting to Zephyr.
- Every page asks for scaling.json, which the board does not have until the data collection
  page saves one: a 404 and a logged error each load, harmless.
