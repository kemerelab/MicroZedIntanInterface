# Phase A summary — LFP 2 kHz → 3 kHz + analytic chirp

Branch `claude/lfp-3khz` (off `main`). Goal: raise the LFP decimation from
**R=15 (2 kHz) → R=10 (3 kHz)** for all 256 channels, broadband 30 kHz unchanged
(byte-identical), and add a memory-free analytic chirp debug signal. No hardware
available — taken through **RTL sim + a timing-closed PL build**. Not pushed, not
merged to `main`.

## TL;DR

- **#1 deliverable met:** a **timing-closed R=10 PL build** at 84 MHz, **WNS = +0.413 ns**,
  0 failing endpoints. Bitstream generated (`design_1_wrapper.bit`).
- **Filter approach shipped: dual-MAC single-stage FIR** (the plan's endorsed
  fallback), not CIC. A 131-tap Kaiser 3 kHz anti-alias (passband flat to ~1 kHz,
  ≥46 dB — measured **−70 dB** worst alias-into-passband). The single-MAC budget
  at R=10/256ch is ~109 taps; adding a 2nd MAC lane (DSP48 is ~free) doubles it to
  ~218, so a real ~130-tap anti-alias fits with margin.
- **Analytic chirp NCO** implemented, **bit-exact in sim**, gated by a new
  `chirp_mode` bit in `CTRL_REG_3`, reusing the existing 512-entry sine LUT (no BRAM).
- **CIC ÷5 + halfband ÷2 (the plan's primary, BRAM-optimizing approach):**
  `lfp_halfband.sv` (the ÷2 block Phase B reuses) is **bit-exact in sim**;
  `cic_decimator.sv` is implemented + sim-harnessed but **not yet bit-exact**
  (a known read-latency bug in the comb pass — details below). Per the plan's
  "if CIC bring-up stalls, fall back" guidance, the dual-MAC FIR is the shipped,
  timing-closed datapath; the CIC is the documented BRAM-win left for follow-up.

## What I implemented

### 1. Dual-MAC decimating FIR (`programmable_logic/src/lfp_fir_decimator.sv`)
- Parameterized `N_MAC` (default 2). The proven time-shared ring+MAC engine now
  processes `N_MAC` adjacent taps/clock: the per-lane delay mems and the coef RAM
  are replicated `N_MAC` ways for `N_MAC` single-cycle reads/clock; two products
  sum into one accumulator. Handles odd `num_taps` via per-lane tap-valid gating
  (partial last group). The MAC datapath stays the registered 3-cycle pipeline.
- **Budget @ R=10:** window = 10×~2800 = 28 000 clk. Cost = 256ch × ceil(131/2) =
  ~16 768 clk < 28 000 → no overrun, with margin. (Single MAC: 256×131 = 33 536 →
  would overrun.)
- Fixed a real bug surfaced by the generalization: a bare `a*b` inside a ternary
  takes a self-determined width of `max(|a|,|b|)` (18 b) and **truncates the
  product**; both operands are now widened to `PROD_W` first.

### 2. Analytic chirp NCO (`data_generator_core.sv`)
- Dual accumulator advanced once per packet: `freq_acc` triangles 0↔f_max by
  `sweep_rate`; `phase_acc += freq_acc`; top 9 bits of (phase_acc + per-channel
  phase) index the existing 512-entry sine LUT — **no BRAM** (frees the playback
  BRAM). Host-configurable per-channel **phase stride** (slot×stride) plus an
  8-way per-lane ⅛-period fan-out so every (lane,slot) channel is distinguishable.
- Config packed into the previously-reserved **`CTRL_REG_3`** (kept clear of the
  STFT regs 28–30 / playback reg 31 so the eventual Phase B merge stays
  mechanical): `[0]` chirp_mode, `[7:2]` stride, `[19:8]` f_span (→f_max), `[31:20]`
  sweep_rate. Gated by `chirp_mode` (independent of `debug_mode`, which must also
  be on). With chirp/aux off the datapath is **bit-identical** to the legacy core.

### 3. CIC ÷5 + halfband ÷2 (BRAM-win path, partial)
- `lfp_halfband.sv` — clean, parameterized time-shared **÷2** decimating FIR with
  host coefficients (true halfband OR CIC droop-comp). **Bit-exact in sim.** This
  is the ÷2 building block Phase B's octave cascade reuses.
- `cic_decimator.sv` — CIC^4 ÷5, per-channel integrator/comb state in BRAM (no
  multipliers, no FIR delay line). **Not yet bit-exact** (see Known issues).

### 4. Contract (PL / firmware / host kept in sync)
- Default `decim_R`: firmware `lfp_cfg_decim_R` 15→**10**; net.py `configure_lfp`
  / `lfp_config` defaults R=10, 131 taps, 1250 Hz Kaiser cutoff.
- `net.py` filter designer: new `_kaiser_window` + `design_lfp_lowpass(window,
  beta)`; default = Kaiser β=6.5 (sharper than the legacy Hamming, which is still
  selectable for the 2 kHz design).
- Chirp: `CMD_SET_CHIRP` (0x77) → `pl_set_chirp` (writes `CTRL_REG_3`), tracked
  and surfaced in `status_response_t` (+8 B → **168 B**; `_Static_assert` + fw
  version → 1.3 + net.py length/offsets all bumped together). net.py
  `configure_chirp` / `chirp_fmax_to_fspan` / `chirp_sweep_to_rate` + interactive
  `chirp` / `chirp_off`. The LFP packet's self-describing R field already carries
  decim_R (firmware `lfp_pktbuf[4]`), so the host/plugin auto-tracks the new rate.
- The LFP timestamp alignment is unchanged in mechanism: LFP stamp = frame_seq ×
  decim_R (every R-th broadband packet); the **FIR group delay is now (131−1)/2 /
  3000 ≈ 21.7 ms** (vs ~2 kHz before) — the documented offset the ephys-socket
  plugin should use.

## Filter design (pure-Python analysis; no numpy on this host)

Target: passband flat to ~1.0 kHz (scalogram headroom), alias-free above 1.5 kHz
Nyquist, ≥46 dB. Worst case = any input band that folds onto the **0–1 kHz**
passband after ÷10.

**Shipped — single-stage Kaiser FIR (N=131, fc=1250, β=6.5), Q1.17 quantized:**
| f (Hz) | 0.1 | 500 | 800 | 1000 | 1200 | 1500 | worst-alias |
|--------|-----|-----|-----|------|------|------|-------------|
| |H| dB | −0.00 | −0.00 | −0.02 | **−0.80** | −4.4 | −21.2 | **−70.6 @ 2085 Hz** |

Comfortably exceeds 46 dB. (The 1.0–1.5 kHz output region is the intended
transition/“don’t-care” band per the plan.)

**Analyzed — CIC^4 ÷5 + comp-FIR ÷2** (if the CIC is finished): −56 dB worst
alias-into-passband, ~1 dB droop @ 800 Hz, ~1.9 dB @ 1 kHz (comp-correctable).
CIC order 3 was only −42 dB (below target); order 4 is the right choice. Scripts:
`/scratchpad/filter_design.py`, `/cic_design.py` (kept out of the repo; pure stdlib).

## Sim results (all under xsim, bit-exact vs Python references)

| TB | config | result |
|----|--------|--------|
| `lfp_fir_decimator_tb` | R=10, **131 taps** (odd), dual-MAC, lanes 0/2/5/7 | **PASS** 4096 outs, 32 frames, no overrun |
| `lfp_dsp_block_tb` | R=10, 131 taps, full integration (offset-binary, slot gate, BRAM pack) | **PASS** 768 BRAM words |
| `chirp_tb` | debug+chirp, sweep + per-ch/per-lane phase | **PASS** 1400 words |
| `data_generator_aux_tb` | aux/chirp **off** → legacy bit-identity | **PASS** 26 006 checks, 0 errors |
| `lfp_halfband_tb` | ÷2 generic FIR, 23 taps | **PASS** 3840 outs, 40 frames |
| `cic_decimator_tb` | CIC^4 ÷5 | **FAIL** (comb reads X on alternating channels — WIP) |

New sim files: `gen_lfp_fir_vectors.py`/`lfp_fir_decimator_tb.sv` (updated to
R=10/131), `gen_lfp_block_vectors.py`/`lfp_dsp_block_tb.sv` (updated), plus
`gen_chirp_vectors.py`+`chirp_tb.sv`+`run_chirp_tb.sh`,
`gen_halfband_vectors.py`+`lfp_halfband_tb.sv`+`run_halfband_tb.sh`,
`gen_cic_vectors.py`+`cic_decimator_tb.sv`+`run_cic_tb.sh`.

## Build — timing closure + utilization (the headline result)

Clean build from `scripts/create_vivado_project.tcl` + `build_bitstream.tcl`
(`xc7z020clg400-1`, ~12 min). **Routed**, bitstream written.

**Timing @ 84 MHz data clock (post-route):**
- **WNS = +0.413 ns**, TNS = 0, **0 / 71 006 failing setup endpoints**.
- WHS = +0.018 ns, 0 failing hold. → **TIMING CLOSED.** (Comparable to the
  documented ~0.45 ns baseline slack — the dual-MAC + chirp preserved it.)

**Utilization (post-route):**
| resource | this build (dual-MAC R=10) | of device | note |
|----------|---------------------------|-----------|------|
| DSP48E1 | **3** / 220 | 1.4% | +2 vs single-MAC (the 2nd lane); still trivial |
| Block RAM tiles | **98.5** / 140 | 70% | RAMB36 96 + RAMB18 5 |
| Slice LUTs | 13 912 / 53 200 | 26% | |
| Slice FFs | 20 647 / 106 400 | 19% | |

**LFP engine (`lfp_dsp_inst`) hierarchical: 64 RAMB36 + 2 RAMB18 + 3 DSP48.**

**Firmware:** `scripts/create_vitis_project.py` rebuilt the platform from the new
`.xsa` and compiled **both cores at −O3 → "Build Finished successfully"** (core0
`klab-firmware.elf` + core1 + FSBL). This confirms the `_Static_assert(sizeof(
status_response_t)==168)` passes and all firmware changes (chirp fns, CMD_SET_CHIRP,
status fields) compile clean.

**256-channel realistic-cadence check:** a focused TB ran the dual-MAC engine at
the true ~2800-clk/packet rate, lane_mask=0xFF (all 256 ch), 131 taps, R=10 →
**PASS, no compute_overrun**, 7168 outputs bit-exact. This proves the compute pass
fits the real-time budget for the *full* configuration, not just the padded TBs.

**Before vs after BRAM (the dual-MAC tradeoff):**
- Single-MAC baseline (R=15) LFP engine: **~32 RAMB36** (delay lines = 8 lanes ×
  N_SLOTS×RING_DEPTH×16 b, coef RAM small).
- Dual-MAC (R=10) LFP engine: **64 RAMB36** — the 2nd MAC lane **replicated the
  delay-line + coef RAM (×2)**. This is the deliberate cost of the fallback: it
  buys tap budget, not memory. Device total 98.5/140 tiles (70%) → fits, with
  headroom, but the dual-MAC is BRAM-heavy.
- **The CIC ÷5 + halfband ÷2 path would instead cut the delay-line BRAM ~5× to
  ≈6 RAMB36**, freeing ~25+ RAMB36 for Phase B. That BRAM win is the reason to
  finish the CIC (below).

## Known issues / decisions left for the user

1. **CIC bit-exactness (the one real bug).** `cic_decimator.sv`’s integrator
   pipeline-forwarding was fixed (combinational cascade input + reset of the
   addr/x registers), but the **comb pass still reads `X` on alternating
   channels**. Root cause is a 1-cycle **read-latency misalignment**: the 2-phase
   per-stage schedule issues the BRAM read address at phase 0 and consumes the
   registered read at phase 1, but a registered-address BRAM needs the data one
   cycle *after* the address settles — so the value used is one cycle early. The
   fix is to drive `integ_addr`/`comb_addr` **combinationally** from the FSM state
   (so the registered read captures the right address at the phase edge) rather
   than registering them; a 3-phase schedule would also work but 256×4×3 = 3072 >
   2800 clk/packet → would overrun the integrate budget, so the combinational-
   address fix is the right one. `lfp_halfband.sv` (which Phase B needs) is already
   bit-exact, so this is isolated to the CIC integrator/comb engine.
   **Decision:** ship dual-MAC now; finish CIC as a follow-up to reclaim ~25 BRAM36
   for Phase B (recommended before the 256-ch wavelet build, which is BRAM-bound).

2. **BRAM headroom for Phase B.** With the dual-MAC FIR at 70% BRAM, Phase B’s
   256-ch scalogram + DDR staging is tighter than the plan assumed (which counted
   on the CIC freeing ~25 RAMB36). Either finish the CIC (issue 1) or build Phase B
   K=32-first (its modest BRAM) and revisit. Not a blocker for Phase A.

3. **No hardware validation.** Everything here is sim + build. The on-bench checks
   the plan lists (256-ch LFP @ 3 kHz, clean passband to ~1 kHz, swept-generator
   anti-alias rolloff, 1.5 MB/s sustained, ephys-socket 3 kHz in Open Ephys) remain
   to be run on the board, and the ephys-socket plugin rate bump (2 kHz→3 kHz,
   derive from the packet R field; update the group-delay offset to ~21.7 ms) is a
   separate repo not touched here.

4. **Chirp scaling knobs.** f_span step = `<<16` (≈0.46 Hz/step, full ≈1.9 kHz);
   sweep_rate step = `<<9`. net.py `configure_chirp(f_max, period_s, stride)`
   converts Hz/seconds to fields. Defaults: sweep 0→~1.4 kHz. Reasonable, but the
   exact f_min (currently ~0) and triangle-vs-one-shot behavior are easy to retune
   if the bench wants a different sweep shape (e.g. an exponential/log sweep for the
   constant-Q scalogram — a `[1]` reserved bit in CTRL_REG_3 is parked for it).

## Files touched
- RTL: `lfp_fir_decimator.sv` (dual-MAC), `data_generator_core.sv` (chirp NCO),
  `lfp_halfband.sv` (new), `cic_decimator.sv` (new, WIP).
- Firmware: `include/main.h`, `src-core0/pl_control.c`, `network.c`, `main.c`.
- Host: `remote/net.py`.
- Sim: see the table above.
- Docs: this file + `docs/lfp-dsp-engine-design.md` (rate note).
