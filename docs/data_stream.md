# The 62.5 MB/s data stream

The PSU board streams the oscilloscope's continuous capture to a browser or to a Python client:
eight int16 channels at up to 3.906 MS/s (125 MHz / 32), 16 bytes a sample, **62.5 MB/s**,
every sample delivered. The stream is a WebSocket on TCP port 5004
(`ws://<board>:5004/`). The scope page's Stream button and the collect page use it, and so
does `firmware/apps/scope/test_stream.py`.

This document records what the FPGA design and the firmware needed to get there, in the
order the work was done. Zephyr's own TCP stack carried about 1.7 MB/s on this hardware.

## The data path

```
 ADCs, sigma-delta, control signals
        |
 scope_capture (continuous mode)  --8 x int16 records-->  scope_axis_writer (64 bit AXI stream)
        |                                                        |
        |                                            dma_scope (DDR master port 1, write only)
        |                                                        |
        |                                  hyperram_axi -> HyperRAM, 32 MB ring on the second die
        |                                                        |
        |                         TSEMAC scatter-gather DMA (DDR master port 0) reads the payload
        |                                                        |
        |                                  tx_checksum_inserter fills in the IPv4/TCP checksums
        |                                                        |
        +---- register 22: records taken ----> CPU (tcp_stream)  TSEMAC -> RGMII -> 1000BASE-T
```

The CPU does not touch the sample data. For each 1460 byte TCP segment it writes three
words into a header that already exists and gives the DMA the payload's address in the
ring.

## Why the stack was slow

Profiling with JTAG PC sampling showed where the CPU's time went. The data cache is
write-through, so every store goes to the HyperRAM, about 28 clocks each. Zephyr's TCP
handles every byte several times:

- the application builds the data;
- `tcp_pkt_append` copies it into the send queue;
- `net_pkt_copy` copies it again into each segment;
- `calc_chksum` sums it (4 % of the time).

In the SDK's picolibc, which is built for size, `memcpy` and `memmove` copy one byte per
loop, and they took 41 % of the CPU. A faster stack could never reach tens of MB/s, so the
data needed a path that never goes through the CPU.

## FPGA

### 1. A HyperRAM controller that carries two DMAs at once

The SoC's main memory is a W959D8NFYA HyperRAM: x8 DDR, two 32 MB dies. During a stream it
carries 62.5 MB/s of capture writes, 62.5 MB/s of Ethernet DMA reads, and the CPU's own
traffic.

- **200 MHz, one CK cycle per clock, through DDIO registers** (`hyperram_ddr_controller`,
  then `hyperram_axi`). CK comes from a 90° clock that is inverted at the pin and gated
  around CS#.
- **Read calibration at power-up.** The read clock's phase is moved at run time through the
  PLL's dynamic phase shift. The middle of the widest passing window is selected, and both
  dies must pass. Register 1336 reads 0x0DF30001 after a passed calibration.
- **Our own controller instead of Efinix's** (`hyperram_axi`, from hVHDL_memory_library).
  Efinix's controller corrupted DMA transfers on the second die: the Ethernet DMA's reads
  came back 2 bytes off now and then, and the board crashed within minutes. Our controller
  is an AXI3 slave on the SoC's DDR port and serves all 64 MB:
  - It runs on the same clock as the SoC's memory side.
  - It takes INCR bursts of up to 256 beats.
  - It splits bursts into HyperRAM transactions at 1 KB boundaries: at most 512 words,
    2.6 µs of CS# low, under tCSM = 4 µs. A transaction never crosses the die boundary.
  - Write strobes become RWDS masks.
  - Responses come back in order, with their ids.
  - The write and read buffers are memory-library block RAMs.
  - CK stops after a read's last word.
- **Timing fixes found on the board:**
  - The input DDIO registers (`ram_clk_cal`) go straight into a register on `ram_clk`. The
    constraints time that path; no FIFO or synchroniser remains.
  - The next transaction is prepared a clock ahead, because scheduling was the critical
    path.
  - The calibration's window search sets one setting per clock.
  - The padding of a timed-out read had compared two free-running 32-bit counters as
    signed. After 2³¹ words it padded forever and the controller deadlocked. It now counts
    out the missing words.
- **Native core and AXI layer split** (2026-10-13). `hyperram_core.vhd` is the 16-bit word
  interface; `hyperram_axi.vhd` translates AXI3 onto it. A 2-minute stream at 58.8 MB/s
  showed no errors after the split.

### 2. The capture into memory without the CPU

- `scope_capture` has a continuous mode (mode bit 1). Register 22 counts the records the
  DMA has taken.
- `scope_axis_writer` turns the 128-bit capture port into a 64-bit AXI stream: two beats per
  record. In memory, 16-bit word `8*s + c` is channel `c` of sample `s`.
- `dma_scope` is a second Efinix DMA on DDR master port 1, write only. It writes the stream
  around a ring in the HyperRAM in whole bursts of 16 records. It counts a record when it
  takes it into its 2 KB buffer (128 records), not when the record reaches memory, so the
  firmware keeps a margin (see below).
- All of this runs on the SoC's peripheral clock (125 MHz), which is also the sample clock.

### 3. Transmit checksums in hardware

The TSEMAC computes no checksums: its "checksum enable" registers at +0x210/+0x214 have no
logic behind them. `fpga/tsemac_soc/ethernet/tx_checksum_inserter.vhd` sits between the
transmit DMA and the MAC:

- It stores each frame in a 4 KB ring while summing it, then plays it out with the IPv4
  header, TCP, UDP and ICMP checksums filled in.
- It handles Ethernet II IPv4 only, and L4 checksums only for unfragmented datagrams that
  the frame holds whole.
- It runs one byte per clock, with at most 5 clocks between waiting frames.
- It uses 4 RAM blocks and under 5 % of the logic; io_tseClk fmax is 190 MHz.

The testbench (`ethernet/sim/tx_checksum_inserter_tb.vhd`) sends 400 frames with garbage
checksums, random gaps and backpressure, and checks every byte.

This is what makes the zero-copy send possible. The DMA reads the payload straight from the
ring, and nothing would be there to sum it otherwise.

## Firmware

### 4. Build for speed

`ti60_psu_eth_defconfig` sets `CONFIG_SPEED_OPTIMIZATIONS=y` and
`CONFIG_PICOLIBC_USE_MODULE=y`. Picolibc picks its word-wise copies only when
`CONFIG_SIZE_OPTIMIZATIONS` is off. This mattered for the stack's path: triggered captures
went from 7.7 to 10.0 per second. The fast stream does not depend on it.

### 5. Driver fast path (`module/include/eth_efinix_tsemac.h`)

The TSEMAC driver got an interface beside Zephyr's stack:

- **A receive hook** that sees every frame in the driver's interrupt before the stack and
  takes the stream port's frames (handshake, ACKs, the client's requests).
- **Batches:** a run of frames the transmit DMA sends in one go, two descriptors per frame
  (header and payload, each read where it is). The call starts the DMA and returns. The
  stack's own frames wait until a batch has gone. The batch calls block on the driver's
  mutex instead of retrying it, because retrying starved the stack's lower-priority sender.
- **Transmit slots:** frames that all look alike. Each slot's header buffer and its
  descriptors are written once; a send only says where each slot's payload is. There are
  twice as many slots as frames in a batch, so one half is filled while the DMA sends the
  other. The transmit channel's completion interrupt (IRQ 17) says when a half is free.
- `tx-checksum-offload` in the devicetree tells the driver that the design fills in the
  checksums. The driver then reports `ETHERNET_HW_TX_CHKSUM_OFFLOAD`.

### 6. `tcp_stream`: our own TCP sender for one port (`module/lib/tcp_stream`)

- One connection at a time on one port, data in one direction. The stack never sees it.
- It handles the handshake, acknowledgements and the receive window, and passes the
  client's data to the source in order.
- The stream is a run of equal segments. Segment *i* is a prefix written into the slot's
  header (here the WebSocket fragment header) followed by a payload the source keeps in
  memory. A segment costs the CPU three word stores into its header slot.
- **Retransmission rebuilds a segment from its index**, after three duplicate ACKs or after
  200 ms. Nothing is copied or queued; the source only has to keep a segment until it is
  acknowledged.
- It sends 32 segments per batch (`CONFIG_TCP_STREAM_BATCH`), about 380 µs at line rate.
- Closing resets the connection at the first unacknowledged byte, the only reset a client
  accepts (RFC 5961).
- A client that stops acknowledging for 3 s is dropped.
- A new client takes over only once it has sent a complete `GET` request. A browser's idle
  extra connection had taken the stream over before.
- `ws_stream` makes it a WebSocket. Each 1456-byte segment is one fragment of a 1 MB binary
  message.

From a buffer in memory (`apps/stream_test`) this path sends **87 MB/s**, raw or as a
WebSocket. There the DMA is the limit: 32 frames take 466 µs, against 393 µs at line rate.

### 7. The stream source (`apps/scope/src/stream.c`)

- A connecting client starts a continuous capture with the settings the page has made
  (sample rate and the eight sources).
- **Segment size:** 91 records = 1456 bytes, which fits the 1460-byte MSS with the 4-byte
  WebSocket header.
- **The ring:** 23 040 segments (32 MB) on the HyperRAM's second die (`stream_ram` in the
  devicetree, 0x02000000 up, clear of the controller's calibration area at the top). That
  is 537 ms at 3.9 MS/s, so the client may fall half a second behind and catch up: a pause
  of the PC's or a slow ACK is no loss.
  - The ring grew from 1 MB (17 ms, too short for any PC pause) to 8 MB to 32 MB.
  - It could only move to the second die once our controller replaced Efinix's.
- **MARGIN_RECORDS = 256:** a segment goes out only once the DMA has taken 256 records past
  it. The DMA's count runs ahead of what is in memory. Sent sooner, a burst could still
  hold the previous lap.
- **Overrun check:** the count of records written is read again for every segment. If the
  capture gets a ring ahead of a segment that has not been sent yet, the board resets the
  connection rather than send stale data, and counts the overrun (the scope's status shows
  why the stream ended). An earlier version used a stale count and sent overwritten data
  before it noticed.
- Triggered captures pause while a stream runs.
- A reboot stops the scope DMA. Otherwise it kept writing into the old image's ring.

### 8. The clients

- **`collect.html` / `collect_worker.js`:** the WebSocket is read in a Web Worker, so
  drawing does not stall the receive. The page draws a min/max envelope per pixel column
  and records raw int16 data. It sends heartbeats, because the board drops a silent client
  after 10 s.
- The scope page offers only the stream rates the board carries (up to 3.906 MS/s). A
  stream started at a faster rate first falls back to that rate.
- **`test_stream.py <board> <seconds> <divider>`** checks every sample against the counter
  on channel 0. It changes the scope's settings.

## Where the limit is

The Ethernet link carries about 118 MB/s of payload, and `tcp_stream` 87 MB/s from a buffer in
memory. **The HyperRAM sets the 62.5 MB/s ceiling.** It carries the capture's writes and the
Ethernet DMA's reads at the same time: a batch of 32 frames takes 507 µs alone and 733 µs
beside 80 MB/s of writes. At 5 MS/s (80 MB/s) the ring overruns in a fraction of a second,
and the board resets the connection.

## Measured results

`test_stream.py`, divider 31 = 3.906 MS/s = 62.5 MB/s produced:

| Run | Received | Result |
|---|---|---|
| 30 s (117 M samples) | 62.6 MB/s | 0 errors, no overrun |
| 5–8 s runs, repeated | 62.5–62.7 MB/s | 0 errors |
| 89 s (349 M samples) | 61.9 MB/s | 0 errors, then an overrun: board reset the connection |
| divider 33 (58.8 MB/s), 2 min, after the HyperRAM core split | — | 0 errors |
| divider 24 (5 MS/s, 80 MB/s) | — | overrun within a fraction of a second |

At 62.5 MB/s the stream runs with little margin: it holds for tens of seconds, and a long
PC-side stall can still overrun the half-second ring. Rates of 58.8 MB/s and below have
been clean in every run.

## Going faster

More than 62.5 MB/s needs fewer bytes through the HyperRAM:

- fewer channels, or 14-bit packing of the records;
- or a full hardware send: an FPGA frame generator in the TSEMAC transmit stream (muxed in
  at frame boundaries) builds the segments from data that never goes to the HyperRAM, and
  the CPU only updates window and sequence from the ACKs (about 110 MB/s). Retransmission
  then needs the data kept in the FPGA or the HyperRAM for one round trip.

## Files

| Part | File |
|---|---|
| HyperRAM core / AXI3 slave | `fpga/source/hVHDL_memory_library/hyperram/hyperram_core.vhd`, `hyperram_axi.vhd` |
| Scope capture | `fpga/source/fpga_scope/source/scope_capture.vhd` (submodule fpga_scope) |
| Capture to AXI stream | `fpga/source/fpga_scope/source/scope_axis_writer.vhd`, `fpga/tsemac_soc/scope/scope_subsystem.vhd` |
| Checksum inserter | `fpga/tsemac_soc/ethernet/tx_checksum_inserter.vhd` |
| SoC wiring (DMAs, DDR ports) | `fpga/tsemac_soc/sapphire_subsystem.vhd` |
| Driver fast path | `firmware/module/drivers/ethernet/eth_efinix_tsemac.c`, `module/include/eth_efinix_tsemac.h` |
| TCP sender, WebSocket | `firmware/module/lib/tcp_stream/tcp_stream.c`, `ws_stream.c` |
| Stream source | `firmware/apps/scope/src/stream.c` |
| Clients | `firmware/apps/scope/web/collect*.js`, `firmware/apps/scope/test_stream.py` |
| Devicetree (ring, offload) | `firmware/module/boards/efinix/ti60_psu_eth/ti60_psu_eth.dts` |
