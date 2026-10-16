# Zephyr: fast page changes and several clients at once

The board's web pages (scope, collect, waveform, update, and the banner on each of them)
come from Zephyr v4.4.2's HTTP server and its WebSocket support. The same goes for the
Python scripts that talk to `/ws`.

Out of the box this setup had problems:

- Changing pages quickly slowed the board down, and after 3 to 62 page changes its network
  stopped.
- Firefox pages sometimes never drew their plots.
- Two pages, or a page and a script, could not share the scope reliably.

This document collects the Zephyr-side changes that fixed this. Most of them work around
behaviour in Zephyr v4.4.2 itself, so **check them again when Zephyr is upgraded**
(`west.yml` pins v4.4.2, unpatched).

The fast data stream on port 5004 is a separate path beside the stack; see
[data_stream.md](data_stream.md).

## Summary

| Problem | Cause | Fix | Where |
|---|---|---|---|
| Network stops after quick page changes | `websocket_unregister()` uses its context after freeing it | close the WebSocket and its TCP socket ourselves | `scope.c` client_thread |
| Two pages' messages mixed | every WebSocket of a resource shares one receive buffer | a receive buffer per client | `scope.c` scope_client_add |
| Kernel thread list corrupted, HTTP server stops | client slot reused before its thread ended | `k_thread_join()` before reuse | `scope.c` scope_client_add |
| Slots held by pages that left | no close frame from a vanished page | 10 s without a message closes the client; pages send a heartbeat each second | `scope.c`, `web/*.js` |
| Crash in about 1 of 1500 page loads | TCP socket fd closed twice | close it only if the fd still refers to the same socket | `scope.c` client_thread |
| Firefox pages not drawing | the static FS resource sends 4 bytes past `Content-Length` | our own dynamic file resource, chunked | `main.c` |
| Every request answered 409 after a broken transfer | the server's fallback resource stayed held | serve files as a `/*` resource, not the fallback | `main.c` |
| Second tab or browser finds no connection | 6 HTTP clients, 1 WebSocket context by default | 12 HTTP clients, 6 WebSocket contexts, contexts, fds and poll entries to match | `prj.conf` |
| HTTP server did not start next to telnet | only one eventfd | `CONFIG_ZVFS_EVENTFD_MAX=2` | `prj.conf` |
| A page shows other settings than the board runs | settings sent only in the hello | config broadcast to every client after a change | `scope.c` |
| A reopened page is empty until the next capture | — | the last capture kept and sent after the hello | `scope.c` |
| Network dead, nothing to recover it | — | network watchdog and restart on a fatal error | `main.c` |
| Captures slow over the stack | byte copies, a copied transmit buffer, a 64 KB window losing segments | word copies, DMA from the packet's fragments, 32 KB send window, captures built in place | driver, board defconfig, `prj.conf`, `scope.c` |

## 1. The WebSocket close race (network stops on quick page changes)

**Symptom.** After a few to a few dozen quick page changes the board stopped answering:
no ping and no pages until it was restarted.

**How it was found.** The serial shell showed `net mem` with all 32 receive packets held.
`devmem` dumps of the fd table, the WebSocket contexts and the sockets' receive queues then
showed the cause:

1. In Zephyr v4.4.2, `websocket_unregister()` closes the WebSocket fd, which frees its
   context. After that it reads and writes that context's `real_sock` and `sock` fields.
2. The HTTP server thread is cooperative (priority −1), and the scope's client threads are
   preemptible (priority 8). The server can therefore run in between and hand the freed
   context to the next page's WebSocket.
3. That WebSocket is then left with `sock = real_sock = -1`. Its client thread polls
   nothing, its TCP connection stays open, and the browser's data fills every receive
   packet. With no packets left, the stack drops everything else too.

**Fix** (`firmware/apps/scope/src/scope.c`):

- `scope_client_add()` runs when the HTTP server has registered the WebSocket, before
  anything is received on it. It finds the WebSocket's context with
  `websocket_context_foreach()` and notes the TCP socket (`real_sock`) together with the
  object its fd refers to. This needs Zephyr's internal header `websocket_internal.h`; its
  include path is added in `apps/scope/CMakeLists.txt`.
- On disconnect the client thread does not call `websocket_unregister()`. It closes the
  WebSocket fd, which sends the close frame and frees the context, and then the TCP socket
  from what it noted. It does both under `send_lock`.
- **Each client has its own receive buffer** (`ws_buffers[MAX_CLIENTS][1024]`) set in its
  context. The HTTP server registers every WebSocket of a resource with that resource's
  single buffer, so two pages' messages would be read into the same memory.

**Follow-up fix.** About once in 1500 stress-test page loads the TCP socket's fd had
already been closed, by something not identified. Closing it again crashed the firmware,
or would have closed another connection that had reused the number. The close now checks
with `zvfs_get_fd_obj()` that the fd still refers to the noted socket, and logs a warning
if it does not.

## 2. Client slots

`MAX_CLIENTS` is 6: the pages (each page has its own connection, or shares its banner's)
plus a script.

- **A slot is free only once its thread has ended.** The client thread used to mark its
  slot free and then return. The HTTP server, running above it, could call
  `scope_client_add()` in between and create a new thread on a `struct k_thread` that the
  kernel still listed. That corrupted the kernel's thread list: the HTTP server stopped,
  and once the whole firmware hung. `scope_client_add()` now skips a slot whose thread
  `k_thread_join(..., K_NO_WAIT)` says has not ended.
- **Pages that vanish without closing do not hold a slot.** A client that has sent nothing
  for `CLIENT_GONE_MS` (10 s) is closed. Every page sends a heartbeat each second, and the
  collect page sends one too: it had none at first and was dropped every 10 s. Scripts
  send heartbeats through `measure_loop_response.Board`.
- The output's watchdog uses the same notion of presence: a run stops when no client has
  sent anything for a while.

## 3. Serving the web files ourselves (`firmware/apps/scope/src/main.c`)

**Problem.** Zephyr v4.4.2's static filesystem resource sends `"\r\n\r\n"` after each file,
four bytes that its `Content-Length` does not count. On a kept-alive connection the next
response then appears to start with those four bytes. Firefox took the next file for one
without headers, scripts failed to parse, and the pages never drew. Chromium let it pass.

**Fix.** A dynamic resource serves the littlefs files:

- chunked, 4 KB at a time, gzipped as stored, the content type taken from the name, and
  404 for files that are not there;
- registered as a resource for `"/*"` and `"/"`, **not** as the service's fallback. The
  server releases a dynamic resource whose transfer broke off (for example a reset
  connection) only if it is one of the service's resources. As the fallback it stayed held
  and answered every later request with 409.
- Its name sorts last, so the other wildcard resources (`/files/*`) match first.

`upload_web.sh` also puts an empty `scaling.json` on the board when there is none, so no
page load asks for a missing file.

With this in place, a page change is ready in 0.2 to 0.3 s in Firefox and in Chromium.

## 4. Room for several browsers (`firmware/apps/scope/prj.conf`)

A browser keeps up to 6 idle connections open per page after loading it; the server closes
them after 10 s. A second tab or browser needs room on top of that.

```
CONFIG_HTTP_SERVER_MAX_CLIENTS=12     # was 6
CONFIG_WEBSOCKET_MAX_CONTEXTS=6       # Zephyr's default is 1; = scope.c MAX_CLIENTS
CONFIG_NET_MAX_CONTEXTS=28            # 12 HTTP, 6 WebSockets, the stream, the listeners
CONFIG_NET_MAX_CONN=28
CONFIG_ZVFS_OPEN_MAX=48               # two fds per WebSocket (its own and its TCP socket's)
CONFIG_ZVFS_POLL_MAX=24
CONFIG_ZVFS_EVENTFD_MAX=2             # the HTTP server's and the telnet shell's
CONFIG_NET_TCP_TIME_WAIT_DELAY=0      # closed connections free their context at once
CONFIG_HTTP_SERVER_STACK_SIZE=8192
```

With only one eventfd, the telnet shell took it and the HTTP server, and with it the
firmware update, did not start.

## 5. Captures for every client

- **One capture, sent to all clients.** The acquisition thread builds each capture once and
  sends the same message to every connected client under `send_lock`, with a 2 s send
  timeout each.
- **Captures built in place.** The samples are copied from the HyperRAM ring straight into
  the message, in at most two blocks, behind a JSON header padded to 512 bytes. Copying
  32768 samples takes 25 ms.
- **The last capture is kept for pages that connect later.** Two message buffers take
  turns: one is filled while the other holds the last capture sent. A new client gets the
  hello, then that capture, then the status. A scope page reopened after the collect page,
  or opened in another browser, therefore shows what the scope last took, even when it is
  stopped.
- **Long captures do not go through `/ws`.** A capture longer than 32768 samples is
  announced on `/ws` without its samples. The page downloads it on the stream port
  (`ws://<board>:5004/capture?id=<n>`): 16 MB in about 0.25 s. The scope arms again once
  the capture is downloaded, the settings change, or 5 s pass.
- **Shared settings.** After a config command that changes something, the board sends
  `{"type": "config", "version", "config"}` to every client. Each page then shows what the
  board runs, not its own idea of it. A command that changes nothing (a script's `{}` status
  poll) does not bump the version. Before this fix it restarted the capture in progress
  every 0.3 s.
- **New settings arm at once.** The acquisition's waits are 10 ms slices that end on new
  settings, a stop or a force. `armed_version` in the status tells a script when its
  settings are in effect (`Board.wait_armed()`). With a 10 s record running, a short
  capture now arms in 0.45 s and arrives in 1.45 s. Before, it took 4.3 s, or never came.
- **One stream at a time, with a deliberate takeover.** A connection to port 5004 while a
  stream runs goes through the handshake beside the stream. It takes over only with a
  complete `GET` request; the old client is then closed in a way it accepts. A connection
  that asks for nothing is reset after 2 s and the stream goes on. Before this, a browser's
  idle extra connection took the stream over.

## 6. Recovery: network watchdog and restart on a crash (`main.c`)

- If the stack has no free receive packet for 10 s (`NET_STUCK_S`), the firmware restarts.
- A fatal error restarts the firmware instead of halting it.
- Both kinds of restart are counted in memory that survives the reboot, and the face
  panel's network line shows the count (`r`), along with the link and the number of `/ws`
  clients.
- `eth_efinix_get_stats()` gives the driver's and the MAC's counters for diagnosis.

Since the fixes above, the restart count has stayed at 0 in the stress tests.

## 7. Sending speed through the stack

These changes made captures and page loads over Zephyr's TCP faster. The data cache is
write-through, so every store goes to the HyperRAM at about 28 clocks.

- **Build for speed** (`ti60_psu_eth_defconfig`: `CONFIG_SPEED_OPTIMIZATIONS=y`,
  `CONFIG_PICOLIBC_USE_MODULE=y`). Picolibc built for size copies one byte per loop, and
  `memcpy`/`memmove` were 41 % of the CPU. Captures went from 7.7 to 10.0 per second.
- **`word_memcpy`.** Picolibc still copies bytes when source and destination are misaligned
  within a word, and the network stack copies at a 2-byte misalignment all the time. The
  replacement aligns the destination and builds each word from two aligned reads. A copy
  misaligned by 2 went from 6.5 to 22.2 MB/s, and raw TCP from 1.3 to 1.7 MB/s.
- **Transmit from the packet's fragments.** The TSEMAC's DMA reads each `net_pkt`
  fragment where it is, one descriptor each, instead of from a copy. The driver's copy went
  from 9.6 % to 0.1 % of the CPU, and a 1472-byte ping from 1.42 to 1.04 ms.
- **Checksums in the FPGA** (`tx-checksum-offload`): raw TCP 1.72 to 1.76 MB/s.
- **32 KB TCP send window** (`CONFIG_NET_TCP_MAX_SEND_WINDOW_SIZE=32768`). A 64 KB window
  lost segments on the way out and waited for their retransmits: a 512 KB capture took
  516 ms, and takes 326 ms at 32 KB.

The result: about 1.6 MB/s through the stack. That is a 32768-sample capture every 459 ms,
or a 4096-sample capture every 88 ms.

## Testing

- **Browser stress test** (seeded, headless Firefox and Chromium): rapid page changes and
  loads at up to full speed and at 3 s intervals. After the fixes, 503 page loads ran with
  none slow, no failed requests and no restarts, and afterwards only the listening socket
  was open. Separate runs of 151 (Firefox) and 201 (Chromium) loads were also clean.
- The pages' own browser tests: waveform, grid, views and banner.
- Settings shared between clients: one client's change reaches another, and five polls
  leave the version unchanged (firmware 118).

## Where to look after a Zephyr upgrade

- `websocket_unregister()`: whether upstream still uses its context after closing it. If it
  no longer does, `scope.c` could go back to calling it.
- The layout of `struct websocket_context` in `subsys/net/lib/websocket/websocket_internal.h`
  (`recv_buf`, `sock`, `real_sock`), and `websocket_context_foreach()`.
- Whether the HTTP server still gives all WebSockets of a resource one receive buffer.
- The static filesystem resource's extra `"\r\n\r\n"`, and how the fallback resource is
  released after a broken transfer.
