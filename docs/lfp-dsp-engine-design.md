# LFP / on-PL DSP engine — design

Status: **design** (branch `claude/pl-dsp-engine`). This is the agreed plan, not yet
implemented. It captures the first module of an on-fabric **preprocessing engine** aimed
at closed-loop work: a decimating low-pass filter that extracts an **LFP band** and streams
it as a second, independent data product.

LFP = anti-alias low-pass (~600 Hz) + downsample 30 kHz → **2 kHz** (decimation `R = 15`).

## 1. Why these choices (the short version)

| Decision | Choice | Why |
|----------|--------|-----|
| Where the filter runs | **PL fabric** | closed-loop needs µs latency; host round-trip is ms |
| Filter structure | **one time-shared polyphase FIR decimator** | ~22 PL clocks per input sample → 1 datapath serves all 256 ch |
| Arithmetic | 1–2 DSP48, per-channel state in BRAM | `taps × out_rate × ch` = `200×2000×256` ≈ 102 MMAC/s ≈ 1.2 DSP48 @ 84 MHz |
| Coefficients | **shared coef RAM, host-loadable via an indirect window** | one design for all channels; not 200+ registers |
| Transport | **separate UDP stream on its own port** | broadband path stays byte-identical; mirrors the Neuropixels AP/LFP split |
| PL→PS | **dedicated LFP BRAM, reuse the existing CDMA** | proven path; CDMA idle 14/15 of the time |
| Packetization | **core 0, one LFP frame per packet (no batching)** | system already sustains 30k pps; batching only enlarges the per-cycle spike and hits the MTU at K=3 |

Bandwidth: LFP is `256 × 2k × 2 B` = **1.0 MB/s** = +6.7% on top of the ~18 MB/s broadband
(the ratio is just `1/R`, invariant of channel count). The real saving comes from *not*
streaming broadband when an experiment only needs LFP (set broadband mask small / 0 →
~18× less data), which the independent-stream design makes a one-register change.

**Data fidelity:** LFP is a *derived* band on a separate wire. The raw broadband samples
are never altered — consistent with the lab's data-fidelity rule.

## 2. Datapath (PL)

```
per-channel sample stream ─► [delay-line BRAM, 256×N] ─► time-shared MAC ─► [decimate ÷R] ─► LFP frame
   (from acquisition FSM)            ▲                         ▲
                                     │                  [coef RAM, 1×N, shared]
                              write @ input rate         read 1 tap/clk
```

- **Delay lines** (the real memory cost): `256 ch × N taps × 16-bit`. For `N=256`:
  256·256·2 = 128 KB ≈ ~28 BRAM36 of 140 on the 7020 (~20%). For `N=200`: ~100 KB. If
  this ever gets tight, switch to **multi-stage** decimation (CIC ÷15 + short comp-FIR, or
  cascaded halfbands) to shrink both the tap count and the delay lines. Single-stage is the
  v1 because it is the simplest to reason about and fits.
- **Coefficient RAM**: `1 × N × 18-bit`, shared across all channels. `N≤512` is half a
  BRAM18. Dual-port: AXI-clock write side, 84-MHz read side (the BRAM *is* the CDC).
- **MAC**: one DSP48E1 (25×18 signed mult + 48-bit acc), time-shared. Sequences
  `(channel, tap)`; emits an output once per `R` input samples per channel.
- **Group delay**: linear-phase FIR → fixed `(N-1)/2 / Fs` ≈ **3.3 ms** at `N=200`. Fine
  for streaming/recording and amplitude/band-power features; a *phase-targeted* closed loop
  (theta/gamma phase-locked stim) would instead want a low-order IIR or a phase predictor —
  noted as a future module, not v1.

## 3. Coefficients — the indirect window (NOT 200 registers)

One filter design is shared by all channels, so there is exactly one coefficient table.
It lives in the coef RAM and is uploaded through a 2-register window, reusing the
**aux-bank strobe/toggle CDC pattern** already in the codebase (`CTRL_REG_AUX_WRITE` +
`CTRL_REG_AUX_STROBE`):

- write pointer auto-increments on each coefficient write;
- the host streams `N` 18-bit signed taps (one-time setup, like the COPI/aux uploads —
  trivial over TCP);
- **coefficients latch only while streaming is stopped** (matches the existing "safe
  control register" convention), so a live pass never mixes old/new taps. A ping-pong
  double-buffer for live retune is a later option, not v1.

The host owns the design: `scipy.signal.firwin`/`remez` → quantize to **Q1.17** (18-bit
signed) → upload. Thin firmware pipe, host computes the physical units — same split as
everywhere else.

## 4. Contract additions (the three layers)

### 4a. Commands (host → PS), new `0x80` block

| ID | Name | params | notes |
|----|------|--------|-------|
| `0x80` | `LFP_ENABLE` | p1 = 0/1 | enable/disable the LFP engine + stream |
| `0x81` | `LFP_SET_PARAMS` | p1 = decimation `R`; p2 = `num_taps` | latched while stopped |
| `0x82` | `LFP_SET_CHANNELS` | p1 = 8-bit stream mask | independent of the broadband mask |
| `0x83` | `LFP_WRITE_COEF` | p1 = index (or auto-inc); p2 = 18-bit signed tap | maps to the coef window + strobe |
| `0x84` | `LFP_SET_UDP_PORT` | p1 = port | default 5001 |
| `0x85` | `LFP_DUMP_COEF` | — | optional: read back the loaded table for verification |

### 4b. Control registers (PS → PL), extend past the aux regs (reg 25+)

`PL_N_CTRL_REGS` grows from 25. New regs (indices fixed at implementation):

- `CTRL_REG_LFP_PARAMS` — `[0]` lfp_en, `[15:8]` lfp_stream_mask, `[23:16]` decimation `R`,
  `[31:24]` num_taps (latched while inactive).
- `CTRL_REG_LFP_COEF_ADDR` — write pointer into the coef RAM (read back = current pointer).
- `CTRL_REG_LFP_COEF_DATA` — `[17:0]` tap value; a write stores `coef[ptr]`, `ptr++`
  (crosses to the 84-MHz domain via the aux-style strobe toggle).

> Note: adding control regs moves `STATUS_REG_BASE` (`= PL_N_CTRL_REGS*4`). The binary
> `GET_STATUS` path is index-agnostic so it is unaffected; only raw register peeks shift.

### 4c. get_status (PL/PS → host) — "report everything configurable"

Add to `status_response_t` (bump the `_Static_assert` + net.py length together, as usual):

- `lfp_enable`, `lfp_decimation_R`, `lfp_stream_mask`, `lfp_num_taps`, `lfp_udp_port`
- `lfp_coef_crc` — a checksum/CRC of the loaded coefficient table, so the host can verify
  *which* filter is live without dumping all N taps (honors the rule's intent without bloat;
  full read-back is the optional `LFP_DUMP_COEF` command).

### 4d. LFP UDP packet (board → host, port 5001)

Compact, self-describing, its own magic so a misrouted packet is caught. Proposed layout
(values marked *proposal* are arbitrary and finalized at implementation):

| words | field | notes |
|------:|-------|-------|
| 0–1 | magic `0xCAFEBABE_1F1FBEEF` *(proposal)* | distinct low word vs broadband's `0xDEADBEEF` |
| 2–3 | 64-bit timestamp | master count at the decimation instant (a subset of broadband stamps; every `R`-th) |
| 4 | `[7:0]` stream mask · `[15:8]` decimation `R` · `[31:16]` flags | self-describing rate |
| 5 | 32-bit sequence number | LFP-stream drop detection |
| 6… | payload: enabled streams × 32 ch × 16-bit (2 per word) | `0xFF` → 256 ch = 128 words |

Host alignment: LFP sample at timestamp `T` ↔ broadband frame `T`; the FIR group delay is a
fixed documented offset. The plugin exposes this as a second 2 kHz DataStream
(`A_LFP1…`, `B_LFP1…`), alongside the 30 kHz broadband stream.

## 5. PL plumbing summary

Reuses proven parts; the only new RTL is the filter:

1. **+1 BRAM** — small LFP ring (a few KB), written by the DSP engine (kept separate from
   the `0x80000000` capture BRAM to isolate the two write paths).
2. **+1 `axi_bram_ctrl`** at a new address (e.g. `0x84000000`), added as a 3rd slave on
   `smartconnect_1` (2×2 → 2×3) — a copy of the existing pattern.
3. **Reuse the one CDMA** — broadband transfer every 33.3 µs cycle (as today) + an LFP
   transfer on the 1-in-`R` cycle (different `SRCADDR`). Single engine, sequenced, no
   contention (LFP duty 1/15, each transfer a few µs).
4. **+1 `XAxiCdma_SimpleTransfer`** call + LFP packetize/send in the core-0 loop.

## 6. Out of scope for v1 (future engine modules)

- Multi-stage (CIC + comp-FIR) decimation if BRAM/taps get tight or more bands are added.
- IIR / phase-predictor path for phase-targeted closed loop.
- Live coefficient retune (ping-pong double-buffer).
- Core-1 as a data engine — only worth it bundled with moving serial debug to a **soft
  core** (the EMAC is a single core-0-owned peripheral and bare-metal lwIP is single-
  instance, so core 1 can't independently transmit; parked).
- On-fabric feature extraction (band power, threshold crossings, phase) feeding a control
  output — the actual closed-loop endgame this engine is built toward.
