# recv→transmit spike instrumentation (firmware v1.4)

Tooling to pinpoint the occasional `recv→transmit` spike (typical ~16 µs, sticky
max seen ~40 µs) on the core-0 fast loop, even at broadband-only. **Hypothesis:**
the spike lives inside `udp_sendto(...)` — the GEM TX path reaps a *variable*
number of completed TX descriptors via `xemacps_process_sent_bds` (a `while(1)`
loop), so the send time is bursty. This instrumentation *measures* that rather
than guessing: it splits the timed recv→transmit window, captures the worst
packet's breakdown, and builds a distribution.

All times are stored as raw 333.3 MHz global-timer **ticks** and converted to
microseconds host-side in `net.py print_status` (the "store the measurement,
derive the display" convention already used by `loop_ticks`/`dma_ticks`).

## What gets measured (in `process_packet_from_bram`, `main.c`)

The recv→transmit window (`loop_ticks`, the 33.3 µs-budget metric) is split:

```
loop_ticks  =  cdma  +  send  +  other
                │        │        └─ everything else: magic peek, pbuf_alloc/free,
                │        │           pointer update, instrumentation
                │        └─ udp_sendto()  ← suspected spike source
                └─ pl_dma_read_bram() (CDMA BRAM→DDR)
```

`send` is timed by a new `XTime` pair straddling the single `udp_sendto` call.

## New `get_status` fields ("Performance" section)

| Field | Meaning |
|-------|---------|
| `UDP send: last/max` | `send_ticks_last` / `send_ticks_max` — the `udp_sendto` time, isolated. **The new headline number.** |
| `Worst pkt #N: cdma=… send=… other=…` | Snapshot of the worst recv→transmit packet's breakdown, captured the instant a new `loop_ticks_max` is set. `N` = `packets_received_count` at that packet. Shows *what dominated* the worst case. `other` is clamped ≥0 (the three sub-timers are sampled at slightly different instants). |
| `Recv->transmit histogram` | 6 buckets by microseconds: `[<16, 16–25, 25–33, 33–50, 50–100, ≥100]`, counts + % of packets. Shape of the tail. |
| `Over budget (>=33 us): N` | `over_budget_count` — packets whose recv→transmit exceeded the 33.3 µs budget. Frequency of the spike. |

The existing `CDMA transfer last/max` and `Recv->transmit last/max` lines are
unchanged.

## How to read it — is the spike the send path?

1. `perf_reset` to clear sticky maxes + histogram + counts (starts a fresh window).
2. Stream for a while at the config of interest.
3. `get_status`.

**Confirms the hypothesis (spike = GEM TX reaping) if:**
- `send_max` ≫ `cdma_max`, **and**
- the worst-packet breakdown shows `send` dominating (`send` ≫ `cdma`, `other`),
  **and**
- the histogram has a tail (non-zero counts in the `33–50` / `50–100` buckets)
  while the bulk sits in `<16`.

That pattern means recv→transmit is fast almost always but the rare slow packet
is spent inside `udp_sendto` — i.e. the variable TX-BD reap. The fix would then
be in the GEM TX path (e.g. cap/offload the reap, or pace sends), not the CDMA.

**Refutes it if** the worst-packet breakdown shows `cdma` or `other` dominating,
or `send_max ≈ cdma_max` with no histogram tail — then look elsewhere (CDMA
contention on S_AXI_HP0, or the `other` bucket = pbuf/lwIP bookkeeping).

## Control / wire contract

- New command **`CMD_PERF_RESET = 0x91`** (firmware `network.c`, host `net.py`
  `perf_reset`). Clears the sticky maxes, the worst-case snapshot, the histogram,
  and `over_budget_count` — but **not** the per-sample `last` fields or the
  lifetime `dma_errors`. An explicit reset (rather than reset-on-START) keeps the
  measurement window under user control.
- `status_response_t` grew **168 → 220 bytes** (appended a 52-byte block: 7×u32
  then 6×u32 histogram). The `_Static_assert(sizeof(status_response_t)==220)` in
  `main.c`, `collect_status_data` in `network.c`, and `net.py get_status`
  (length check + `data[168:196]`/`data[196:220]` unpack) were updated together.
- Firmware version bumped to **v1.4**.

## Notes

- No RTL/BD change — firmware + `net.py` only. The PL is unchanged, so `BOOT.bin`
  reuses the existing timing-closed bitstream.
- `scripts/check_dma.sh firmware` still PASSes: the instrumentation adds only
  `XTime_GetTime` calls and arithmetic — no single-beat BRAM/staging reads.
- `dma_ticks_last` (and thus `worst_cdma_ticks`) is only meaningful on the default
  `BRAM_READ_DMA` path; the compile-time `BRAM_READ_SINGLE` fallback doesn't set it.
