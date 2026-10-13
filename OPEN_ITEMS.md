# Open items

Known problems, unfinished work and plans, newest findings first in each section. Remove an
item when it is done; the commit that does it says so.

## Input bridge (boost from a DC source)

- **The boost is a voltage control alone, tuned on the board to a 5 Hz crossover.** The
  current loop could not work: iac_in's period mean (about 0.1 A at 4 % duty) did not follow
  the current that charges the primary DC link, too noisy and likely missing the shunt's
  compensation capacitor. `input_boost_control.vhd` (registers 1260) runs a voltage PI from
  the link's period mean straight to the duty, its step a program on the output control's
  microprogram core (`output_control_program_pkg`'s boost programs); the current is only
  shown. The duty is capped at 0.4 in the FPGA whatever d_max says (0.2 until 2026-10-13); the overvoltage stop
  needs 8 readings in a row (single readings carry the switching's noise). The reference
  goes through a two-stage prefilter at the PI's zero (alpha = ki/kp, register 19).
  On the board (2026-10-13, 60 V DC in, `input_boost_test.py --v-ref 64 --step 67 --d-max
  0.2 --limit 75`): with the old gains (kp 0.005, ki 1.5e-6) no overshoot but slow, a step
  1.4 s. A first order fit to that capture: the link 55.2 V + 62.7 V a unit of duty, 42 ms
  (to 0.06 V rms). On it kp 0.022, ki 1.16e-5 (the PI's zero 3.3 Hz) cross over at 5 Hz,
  phase margin 93 deg (the user: about 5 Hz, so the input current has no spikes); with the
  DHB online its capacitance doubles the link's: 3.2 Hz, 74 deg. On the board with them a
  3 V step settles in 0.3 s, 0.14..0.20 V over, the input current as steady (0.82 A at most
  in 5 ms means): now the defaults. 80 V from 56 V was out of reach at the 0.2 cap (69 V):
  the cap raised to 0.4 (the user's choice). The full system to 80 V (2026-10-13,
  supply_raise_test.py, -56 V in): Start to idle at 62 V, then 66 .. 80 V in 4 V steps, the DAB
  on: the link 62.1 / 66.1 / 70.2 / 73.8 / 78.0 / 80.1 V, the duty 0.11 / 0.17 / 0.20 / 0.25 /
  0.26 / 0.25 (it stops rising above 74 V: not the fitted model's regime there), the input
  current 1.1 A at most, the secondary 1..4 V behind in the steps, 80.9 V held.
  Either polarity of the source: the leg its positive terminal is on switches (leg 1 when
  the input's voltage reads negative at the start; status bit 5, scope lane bit 6). On the
  board with the leads swapped (2026-10-13, -56 V in, 60 -> 63 -> 60 V): leg 1 switched, the
  link held at 60.01 / 63.00 V. Its readings are noisier then (the link 0.48 V rms against
  0.17 V, lines at 534 Hz and 1 kHz; the input's voltage 12.6 V rms against 5.5 V): leg 1's
  switching couples into the measurements more than leg 2's; in 5 ms means 0.13 V, the
  control unaffected. The input current reads with the same sign either way.
  With the DAB online (2026-10-13, boost_dab_step_test.py, one capture, -56 V in): the step
  58 -> 60 V as with it off (90 % at 0.24 s either way, 0.13 V over instead of 0.07 V); the
  DHB's control does not tie the secondary's capacitance to the primary rigidly, so the
  doubling the model expects barely shows. Next:
  1. a peak current cutoff on the raw sigma-delta current is still missing; the duty cap is
     the guard.
- **The supply starts in the user's order (supply_sequencer.vhd, 2026-10-13):** the gate driver
  supplies, the input boost to its reference (config boost_v_ref / boost_d_max / boost_limit,
  the scope page's "Input boost at Start"; within 1 V for 50 ms, 3 s at most), then the DHB
  (precharge, then held running): idle. The output on and off from idle (output_on /
  output_off, the banner's Output button); stop, the panel's STOP, a trip, or the boost or the
  DHB stopping by itself takes all of it down (why: 6 the boost not at its reference, 7 the
  boost stopped, 8 the DHB stopped). On the board: Start to idle in 1.6 s at 58 V, the output
  on and off, stop, power off.
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
  Tested on the board in closed loop. START and OUTPUT next to it (lcd_display's buttons 1
  and 2, registers 1360..; panel.c: START to idle, OUTPUT on from idle / off when running,
  grey when they would do nothing; the supply's state and DC links on row 3) are the
  firmware's; a start from the panel keeps the watchdogs kicked until the supply stops. On
  the board (2026-10-13): START to idle, OUTPUT on, STOP. They need the touch calibration.
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
  are 6 slots. Once (in some 1500 loads) a client's TCP socket fd had been closed by something
  else before the client closed it, and closing it crashed the firmware: the close is now
  guarded (only when the fd still refers to the same socket, else a warning "was closed
  already"); who closed it is not known. 503 page loads in the stress test since, none slow. The network watchdog
  (main.c) restarts the firmware should the receive packets stay used up for 10 s; its restarts
  show on the face panel ("r"). The websocket race is worth reporting to Zephyr (a draft
  issue was written, not filed).
- **The golden image still has Efinix's HyperRAM controller**, which corrupts DMA transfers on
  the HyperRAM's second die (0x02000000 up; 2026-10-11). The application image has ours
  (hVHDL_memory_library's hyperram_core.vhd, native, behind hyperram_axi.vhd's AXI3 port
  since 2026-10-13) since then, and the scope stream's
  ring is on the second die (32 MB, half a second at 3.9 MS/s). On the golden image the
  firmware refuses the stream (capture_stream_ram_usable(): register 1336, our controller's
  calibration status). Rebuilding and flashing the golden image needs JTAG (with the power stage off).
- hyperram_axi's state is readable at registers 1336 (calibration: phases that passed, the
  one chosen), 1337 (read timeouts: the calibration's failing phases give 2..4), 1338..1339
  (core state, queue, buffers). Under JTAG, with the CPU stuck on the memory, `reset halt`
  resets only the SoC and the registers can still be read from the on-chip RAM.
- The FPGA is about 94 % full (logic): timing closes with a little margin (placement seed 2);
  adding logic may need another seed or freeing room.
