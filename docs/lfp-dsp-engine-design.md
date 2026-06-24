# LFP / on-PL DSP engine — design

Status: **design** (branch `claude/pl-dsp-engine`). This is the agreed plan, not yet
implemented. It captures the first module of an on-fabric **preprocessing engine** aimed
at closed-loop work: a decimating low-pass filter that extracts an **LFP band** and streams
it as a second, independent data product.

LFP = anti-alias low-pass + downsample 30 kHz → **3 kHz** (decimation `R = 10`,
Phase A; was 2 kHz / `R = 15`). See `docs/PHASE_A_SUMMARY.md` for the 3 kHz
anti-alias design (a ~131-tap Kaiser run on a **dual-MAC** time-shared engine,
passband ~1 kHz, ≥46 dB alias rejection) and the analytic-chirp debug signal.
The single-stage budget at R=10 is ~109 taps/256ch per MAC, so the engine now
processes `N_MAC=2` taps/clock (DSP48 is otherwise free); the delay-line BRAM is
unchanged. A CIC÷5 + halfband÷2 variant (`cic_decimator.sv`/`lfp_halfband.sv`)
that would cut the delay-line BRAM ~5× is the documented alternative.

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

## 3a. Sample representation — offset binary (critical)

Intan amplifier samples are **offset binary**: mid-scale `0x8000` = baseline (0 µV),
`V = 0.195 µV × (code − 32768)`. (This repo's debug `sine_lut` confirms it — centered at
`+32767`, "unsigned-16-bit".) The FIR engine is a **pure two's-complement signed** DSP
block, so the offset must be removed at its input and re-applied at its output, otherwise
a quiet baseline sits at the signed saturation rail and the low-pass output is a flat,
saturated line.

Conversion is a symmetric **MSB invert** (`^ 0x8000`), done at the integration boundary so
the engine stays generic and already-proven:
- **in:** `engine_sample = raw_sample ^ 0x8000`  (offset binary → signed, centered at 0)
- **out:** `lfp_sample = engine_out ^ 0x8000`     (signed → offset binary)

LFP is therefore shipped in the **same offset-binary format as broadband**, so the host
de-offsets both identically. The raw broadband stream is untouched (data fidelity).

**Channels filtered = the 32 amplifier converts only** (8 lanes × 32 = 256 ch). The
integration tap drops the 3 aux slots *and* removes the 2-cycle SPI readback offset:
`convert_cmd_sequence` issues CONVERT(0..31) at command-slots 0–31, so with the +2 readback
delay the amplifier samples land at `cycle_counter` 2..33 → fed as engine slot 0..31
(slots 0/1/34 carry aux/wrap and are not fed). This makes the delay line exactly 8×32×N
(not 8×35×N), the ~128 KB / ~32 BRAM36 in §2.

Bonus: debug-mode sine (58–469 Hz, all < 600 Hz) passes the LP filter, so it is a clean
on-bench LFP test signal through this path.

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

`status_response_t` carries (see `firmware/include/main.h`):

- `lfp_enable`, `lfp_decim_R`, `lfp_lane_mask`, `lfp_num_taps`, `lfp_packets_sent`, `lfp_overrun`.
- `lfp_lane_mask` is reported as the **broadband `channel_enable` mask** (the single source of
  truth) — `network.c::collect_status_data` fills it from `pl_get_current_channel_enable()`,
  not from a separate LFP mask register.

### 4d. LFP UDP packet (board → host, port 5001) — **PL-built, DMA'd into the pbuf**

Compact, self-describing, its own magic so a misrouted packet is caught. **The PL builds the
complete wire packet (6-word header + decimated samples) in the LFP output BRAM** — exactly
the broadband pattern where `data_generator_core.sv` writes the 10-word header into the
capture BRAM. The PS then just **CDMAs the whole frame straight into a pbuf and sends it**
(`network.c::lfp_stream_service` → `pl_dma_read_addr` from `0x84000000` into a non-cacheable
LFP staging buffer → `udp_sendto` a `PBUF_REF`). **No PS-side header construction, no
`Xil_In32` sample loop** (`scripts/check_dma.sh` enforces this — the LFP single-beat loop is
gone). Implemented layout (one frame = one wire packet):

| words | field | notes |
|------:|-------|-------|
| 0 | magic low `0x1F1FBEEF` | distinct low word vs broadband's `0xDEADBEEF` |
| 1 | magic high `0xCAFEBABE` | |
| 2–3 | 64-bit master **timestamp** | the master count of the **LAST (most-recent) broadband sample** that contributed to this decimated frame — the *same* counter the broadband header stamps (header word 1), latched in `lfp_dsp_block` on the engine's decimation tick (`frame_tick`/`start_pass`). It is an **absolute master timestamp**, not a frame index. |
| 4 | `[7:0]` lane_mask · `[15:8]` decim_R · `[23:16]` num_taps · `[31:24]` overrun | `lane_mask` = the **broadband `channel_enable` mask** (single source of truth; see below) |
| 5 | 32-bit sequence number | PL-maintained LFP frame counter (++ per emitted frame), for drop detection |
| 6… | payload: enabled lanes × 32 ch × 16-bit, offset-binary (2 per word) | `0xFF` → 256 ch = 128 words |

**Lane mask = broadband mask.** The LFP engine's `lane_mask` is driven from the broadband
`channel_enable_reg` (`data_generator_core.sv` → `dsp_channel_enable` → `lfp_dsp_block`), **not**
from `lfp_cfg[15:8]`. The LFP filters exactly the broadband-enabled lanes — one place to pick
streams (`set_channels`). `CMD_LFP_SET_CHANNELS` is deprecated (firmware accepts-and-ignores).

Host alignment: an LFP sample stamped `T` ↔ broadband sample `T`; the filter group delay is a
fixed documented offset. The plugin exposes this as a second ~3 kHz DataStream
(`A_LFP1…`, `B_LFP1…`), alongside the 30 kHz broadband stream.

## 5. PL plumbing summary

Reuses proven parts; the only new RTL is the filter:

1. **+1 BRAM** — small LFP ring (a few KB), written by the DSP engine (kept separate from
   the `0x80000000` capture BRAM to isolate the two write paths).
2. **+1 `axi_bram_ctrl`** at a new address (e.g. `0x84000000`), added as a 3rd slave on
   `smartconnect_1` (2×2 → 2×3) — a copy of the existing pattern.
3. **Reuse the one CDMA** — broadband transfer every 33.3 µs cycle (as today) + an LFP
   transfer on the 1-in-`R` cycle (different `SRCADDR`). Single engine, sequenced, no
   contention (LFP duty 1/decim, each transfer a few µs). The LFP output BRAM
   (`0x84000000` / `axi_bram_ctrl_1`) is in the `axi_cdma_0/Data` address space
   (`design_1_bd.tcl`) so the CDMA can read it; `pl_dma_read_addr(dst, src_addr, n)`
   generalizes the read to an arbitrary BRAM source.
4. **+1 `XAxiCdma_SimpleTransfer`** call (the whole PL-built packet, header + samples) →
   zero-copy `udp_sendto` in the core-0 loop. The PS adds **no** header and copies **no**
   samples on the CPU.

## 6. Out of scope for v1 (future engine modules)

- Multi-stage (CIC + comp-FIR) decimation if BRAM/taps get tight or more bands are added.
- IIR / phase-predictor path for phase-targeted closed loop.
- Live coefficient retune (ping-pong double-buffer).
- Core-1 as a data engine — only worth it bundled with moving serial debug to a **soft
  core** (the EMAC is a single core-0-owned peripheral and bare-metal lwIP is single-
  instance, so core 1 can't independently transmit; parked).
- On-fabric feature extraction (band power, threshold crossings, phase) feeding a control
  output — the actual closed-loop endgame this engine is built toward.
