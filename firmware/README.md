# Zephyr on the Efinix Sapphire SoC

Zephyr RTOS on Efinix's Sapphire RISC-V SoC, proved on the Titanium Ti60F225 development
kit before porting to the ti60_ac_psu board. The kit runs Efinix's unchanged TSEMAC Linux
example bitstream (`~/dev/ti60f225_evmlinux/fpga/ti60-tsemac-linux`): two rv32imac harts
at 200 MHz, 32 MB HyperRAM, and the TSEMAC Ethernet MAC behind a scatter-gather DMA.

## Layout

| Path | What |
|---|---|
| `module/` | Zephyr module: boards `ti60_evm_eth`, `ti60_psu_eth` (ti60_ac_psu, tsemac_soc design), `ti60_psu` (board_test_cpu design), drivers, libraries |
| `module/drivers/ethernet/` | TSEMAC Ethernet (MAC + DMA) and its MDIO |
| `module/drivers/spi/` | Sapphire SPI master, used for the configuration flash |
| `module/lib/fw_update/` | firmware update over HTTP (`/update`) |
| `module/lib/http_files/` | list, write and delete files over HTTP (`/files/`) |
| `module/lib/word_memcpy/` | memcpy and memmove that copy words at any alignment |
| `module/lib/tcp_stream/` | one fast TCP stream beside Zephyr's stack, and a WebSocket over it |
| `loader/` | second-stage loader, bare metal, and `mkslot.py` |
| `apps/scope/` | oscilloscope web app (the main app) |
| `apps/net_test/` | network shell |
| `apps/flash_test/` | SPI flash checks |
| `apps/psu_test/` | register bus checks on the board_test_cpu design |
| `apps/tcp_tx/` | raw TCP send benchmark, with `tcp_tx_client.py` |
| `apps/stream_test/` | tcp_stream from a buffer in memory, with `stream_client.py` |
| `fpga/ti60_evm/` | the first bring-up: own Sapphire SoC, no Ethernet |
| `fpga/ti60_evm_eth/` | scripts for the kit with the TSEMAC design |
| `fpga/ti60_psu/` | scripts for the ti60_ac_psu board (JTAG load, flash, console) |

## Setup

See the top-level `README.md` and `setup.sh`: the west workspace (Zephyr v4.4.2 and the
modules in `west.yml`, beside the repository), `.venv` here with Python 3.12, and the tools.
`source env.sh` before building: it also clears the `PYTHONHOME`/`PYTHONPATH` that
Efinity's `setup.sh` sets, which break other Pythons.

## Build and run

```
source env.sh
west build -b ti60_evm_eth -d build/scope apps/scope
fpga/ti60_evm_eth/update.sh build/scope/zephyr/zephyr.bin   # over the network
apps/scope/upload_web.sh                                    # the web client, once
```

Then open `http://<address>/` (DHCP, printed on the console; currently 192.168.1.207).
The pages link to each other: the scope, data collection (the stream in 1 ms points, PSU
board) and Update, which installs firmware (zephyr.bin or loader.bin) and replaces the
web pages from the browser, the same as `update.sh` and `upload_web.sh` without JTAG or
a script.

On the PSU board the FPGA updates the same way (`module/include/fpga_update.h`): the flash
holds a golden image at 0, which the FPGA loads at power-up and which only JTAG writes, and
an application image at 1 MB that the golden image switches to through the Titanium's
internal reconfiguration. ti60_ac_psu's `tsemac_soc/build.sh golden`, `build.sh` and
`build.sh images` make both and `images/application_update.bin`, which the Update page
installs. A new image is tried once and confirms itself after 10 s; one that fails its CRC
makes the FPGA fall back to the golden image (in about a minute), one that hangs is left
after the next power cycle, and either way the board stays reachable for another upload.
After a fallback the FPGA loads no other image before a power cycle. `flash.sh` writes
`images/flash.hex`, once, with the power supply off.

During any FPGA reconfiguration, at power-up included, the FPGA's pins are released with
weak pull-ups for up to about a minute; the board's hardware keeps the power stage safe then,
so FPGA updates over the network are safe with it running. JTAG is not: use it only with the
power stage off.

| Script | Does |
|---|---|
| `fpga/ti60_evm_eth/update.sh <zephyr.bin\|loader.bin> [address]` | network update; waits for the reboot |
| `apps/scope/upload_web.sh [address]` | gzips `apps/scope/web/` into the board's littlefs |
| `fpga/ti60_evm_eth/flash.sh <zephyr.bin>` | USB: writes bitstream, loader and slot 0 (about 2 min); for a new board or recovery |
| `fpga/ti60_evm_eth/load.sh <zephyr.elf>` | JTAG: loads into HyperRAM and runs, nothing written to flash |
| `fpga/ti60_evm_eth/program.sh` | JTAG: loads the bitstream (volatile) |
| `fpga/ti60_evm_eth/shell.py "<cmd>" ...` | runs Zephyr shell commands on the console |
| `apps/scope/test_scope.py [address]` | end-to-end test of the scope app |
| `apps/scope/test_stream.py [address] [seconds] [divider ...]` | the stream on port 5004, every sample checked (PSU) |

The kit's console is FT4232H interface 2 (`fpga/ti60_evm_eth/console_tty.sh`). Its USB
connector is unreliable (it drops out when the board is touched), so the network
update is the normal path.

## Boot and flash layout

1. The FPGA's built-in bootloader copies flash 0x600000 to HyperRAM 0x01000000 and
   starts both harts there. That is our loader.
2. The loader moves itself to 0x01FF0000, parks hart 1, and loads the newest slot whose
   header and image CRC check out to 0x01000000, falling back to the other slot.
3. Zephyr starts. Its image region is 2 MB at 0x01000000 (built XIP style); RAM is 14 MB
   from 0x01200000.

| Flash range | Use |
|---|---|
| 0x000000–0x200000 | bitstream (about 850 KB) |
| 0x200000–0x400000 | slot 0 |
| 0x400000–0x600000 | slot 1 |
| 0x600000–0x610000 | loader |
| 0x610000–0x800000 | littlefs, mounted at `/web` |

A slot is a header (`module/include/sapphire_loader.h`) and the image 4 KB in, up to
2 MB - 4 KB. An update writes the slot that is not running and its header last, so a
failed or interrupted update leaves the old image booting. `GET /update` shows the slots.

## Status

The Zephyr changes behind quick page changes and several clients at once are in
[../docs/zephyr_web_connections.md](../docs/zephyr_web_connections.md), the fast stream in
[../docs/data_stream.md](../docs/data_stream.md).

Working and tested on the kit:

- Ethernet at 1000 Mb/s with DHCP.
- SPI flash: reads about 1.47 MB/s (limited by the CPU's register accesses, not the SPI
  clock), writes about 0.5 MB/s.
- Network firmware update: about 2.3 s to write, back up about 12 s later. A corrupted
  newest slot and an interrupted upload were both tested.
- Loader replacement over the network.
- Web client served gzipped from littlefs.
- Scope app: 8 channels, triggering, run/single/stop; `test_scope.py` passes.
- `flash.sh` recovery.

Known limits:

- The scope's triggered captures over Zephyr's stack are CPU bound: about 12 captures/s
  (0.8 MB/s) on both boards, raw TCP about 1.7 MB/s. Every store to HyperRAM goes through
  the write-through data cache (about 28 clocks), and Zephyr's TCP copies the data twice.
  The PSU board's stream (port 5004) does not go through the stack: up to 62.5 MB/s.
- No rollback: an image that boots but cannot take network updates stays the one that
  boots. Recovery is JTAG or `flash.sh`.
- Replacing the loader has a window of about 30 ms in which a power cut leaves the
  board unbootable until `flash.sh`.
- No authentication on `/update` or `/files/`.
- Zephyr's file serving sends 4 extra bytes after each file, so clients do not reuse the
  connection. Only `.gz` files are stored, so clients must accept gzip.
- The design has no reset the CPU can trigger. A reboot jumps hart 0 back to the
  bootloader after stopping the Ethernet DMA; hart 1 stays parked.
- The upstream Sapphire SoC port leaves the PLIC unusable; the board defconfig fixes
  it (`CONFIG_2ND_LVL_ISR_TBL_OFFSET`, `CONFIG_MAX_IRQ_PER_AGGREGATOR`).
- The TSEMAC's address filter passes no frames, so the driver runs the MAC promiscuous.

## Next: the FPGA data stream at high speed

The page stays in Zephyr: served from littlefs, with the WebSocket and the control
messages. The data needs a faster path. A browser receives only over TCP (HTTP or a
WebSocket); raw UDP is not possible, and WebRTC and WebTransport need encryption that
is impractical in the FPGA. So the fast path is TCP to the browser.

The scope DMA already writes captures into the HyperRAM ring without the CPU. From
there the CPU handles every byte several times: the app builds the capture,
`tcp_pkt_append` copies it into the connection's send queue, `net_pkt_copy` into each
segment, and `calc_chksum` sums it (4%).

1. **Checksum in hardware: done on the PSU board.** The TSEMAC computes no checksums
   (the control registers at +0x210/+0x214 are plain storage), so tsemac_soc has a
   `tx_checksum_inserter` between the transmit DMA and the MAC that fills in the IPv4
   header, TCP, UDP and ICMP checksums, and the board's devicetree declares
   `tx-checksum-offload`. Raw TCP 1.72 -> 1.76 MB/s (2.00 -> 2.11 to Windows).
2. **Decide the rate the page needs.** Decided: as fast as possible. The ADC is 2.5 MS/s x 2 bytes = 5 MB/s, and a
   browser cannot draw 50 MB/s. Decimating in the FPGA (min/max per pixel column), or
   sending only the selected channels, may be enough with step 1 alone.
3. **Own send path for the data connection: done on the PSU board.** `tcp_stream` owns
   port 5004 beside Zephyr's stack: the driver's receive hook gives it the port's frames,
   and its segments go out in batches of transmit slots, two DMA descriptors each (a
   header slot written once per connection and patched with three word stores, and the
   payload where it is), two sets so one fills while the DMA sends the other.
   `ws_stream` makes it a WebSocket: every segment one fragment of a 1 MB binary message.
   The scope app streams the FPGA's continuous capture (scope_capture mode bit 1,
   register 22 for its progress) from a ring of 720 segments; the web client's Stream
   button shows it and Record saves it.
   - From memory (`stream_test`): 87 MB/s, raw or as a WebSocket; Zephyr's stack 1.7.
   - From the FPGA: every sample arrives up to 3.9 MS/s of eight channels, 62.5 MB/s.
     The HyperRAM is the limit there, carrying the capture's writes and the Ethernet
     DMA's reads (a batch of 32 frames 507 us alone, 733 us beside 80 MB/s of writes).
     Faster sample rates overrun the ring; the board then resets the connection rather
     than send stale data.
4. **More than 62 MB/s from the FPGA** needs fewer bytes through the HyperRAM: fewer
   channels or 14 bit packing in the records, or the full hardware send, an FPGA frame
   generator in the TSEMAC transmit stream (muxed in at frame boundaries) building the
   segments from data that never goes to the HyperRAM, the CPU only updating window and
   sequence from the ACKs (about 110 MB/s). Retransmission then needs the data kept in
   the FPGA or the HyperRAM for a round trip.

Other open items: confirm-on-boot rollback.
