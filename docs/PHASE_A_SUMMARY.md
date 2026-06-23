# Phase A summary — LFP 2 kHz → 3 kHz + analytic chirp

Branch `claude/lfp-3khz` (off `main`). Goal: raise the LFP decimation from
**R=15 (2 kHz) → R=10 (3 kHz)** for all 256 channels, broadband 30 kHz unchanged
(byte-identical), and add a memory-free analytic chirp debug signal. No hardware
available — taken through **RTL sim + a timing-closed PL build**. Not pushed, not
merged to `main`.

## TL;DR

- **Shipped datapath = CIC^4(÷5) → comp-FIR halfband(÷2) = ÷10** (the plan's
  primary, BRAM-optimizing approach). It is **timing-closed** at 84 MHz
  (**WNS = +0.417 ns**, 0 failing endpoints) and **bit-exact end-to-end in sim**.
  The whole point — **BRAM dropped 98.5 → 46.5 tiles (RAMB36 96 → 44, −52)** vs the
  dual-MAC FIR, freeing ~52 RAMB36 (~37% of the device) for the BRAM-bound Phase B
  256-ch scalogram. DSP 3 → 1.
- **Selectable via `USE_CIC` (default 1)** in `lfp_dsp_block.sv`. The **dual-MAC
  single-stage FIR** (the earlier fallback) is retained at `USE_CIC=0` — also
  timing-closed (WNS +0.413) and bit-exact — and stays in git history.
- **Combined ÷10 anti-alias** (quantized Q1.17): **flat to 1 kHz (≤0.01 dB)**,
  −32 dB @ 1.5 kHz Nyquist, **−54 dB worst alias-into-passband** (≫ 46 dB target).
  Droop-compensated comp-FIR (frequency sampling) flattens the CIC^4 passband
  droop; combined DC gain unity.
- **Analytic chirp NCO** implemented, **bit-exact in sim**, gated by a new
  `chirp_mode` bit in `CTRL_REG_3`, reusing the existing 512-entry sine LUT (no BRAM).
- **`lfp_halfband.sv`** — the clean parameterized ÷2 block — is reused as Phase B's
  octave-cascade building block.
- No hardware available; everything is RTL sim + timing-closed builds + a clean
  Vitis −O3 firmware build. Not pushed, not merged.

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

**Shipped — CIC^4 ÷5 + droop-comp halfband ÷2** (Q1.17, the default datapath):
flat to 1 kHz **≤0.01 dB**, −32 dB @ 1.5 kHz Nyquist, **−54 dB** worst
alias-into-passband. CIC order 3 was only −42 dB (below target) → order 4. The
comp-FIR (N=43, fc=1300, Kaiser β=6.0) is designed by frequency sampling with the
target = 1/CIC-droop over the passband, so the combined chain is flat and unity
DC gain. CIC bit-growth: ACC_W=32 ≥ 16 + ⌈4·log2(5)⌉ = 26; GAIN_SHIFT=10
(DC gain (R^N)/2^10 = 0.61, restored to unity by the comp-FIR). Host designer
`design_cic_comp_fir` in net.py matches the sim reference bit-for-bit (coef[mid]=
103019). Scripts: `/scratchpad/filter_design.py`, `cic_design.py`, `comp_design.py`.

## Sim results (all under xsim, bit-exact vs Python references)

| TB | config | result |
|----|--------|--------|
| `cic_decimator_tb` | CIC^4 ÷5, lanes 0/2/5/7 | **PASS** 3072 outs, 24 frames, no overrun |
| `lfp_halfband_tb` | ÷2 comp-FIR, 23 taps | **PASS** 3840 outs, 40 frames |
| `cic_chain_tb` | **CIC ÷5 → glue → halfband ÷2 = ÷10**, realistic cadence | **PASS** 2560 outs, no overrun |
| `lfp_dsp_block_tb` | full integration (offset-binary, slot gate, BRAM pack); FIR path | **PASS** 768 BRAM words |
| `lfp_fir_decimator_tb` | R=10, 131 taps (odd), dual-MAC fallback | **PASS** 4096 outs, 32 frames, no overrun |
| `chirp_tb` | debug+chirp, sweep + per-ch/per-lane phase | **PASS** 1400 words |
| `data_generator_aux_tb` | aux/chirp **off** → legacy bit-identity | **PASS** 26 006 checks, 0 errors |

New sim files: `gen_cic_vectors.py`+`cic_decimator_tb.sv`+`run_cic_tb.sh`,
`gen_halfband_vectors.py`+`lfp_halfband_tb.sv`+`run_halfband_tb.sh`,
`gen_cic_chain_vectors.py`+`cic_chain_tb.sv`+`run_cic_chain_tb.sh`,
`gen_chirp_vectors.py`+`chirp_tb.sv`+`run_chirp_tb.sh`, plus the updated
`lfp_fir_decimator_tb`/`lfp_dsp_block_tb` (R=10).

## Build — timing closure + utilization (the headline result)

Clean build from `scripts/create_vivado_project.tcl` + `build_bitstream.tcl`
(`xc7z020clg400-1`). **Routed**, bitstream written, **`USE_CIC=1` (CIC chain)**.

**Timing @ 84 MHz data clock (post-route):**
- **WNS = +0.417 ns**, TNS = 0, **0 / 95 685 failing setup endpoints**.
- WHS = +0.032 ns, 0 failing hold. → **TIMING CLOSED.**
- Took 3 build iterations: build 1 missed by −0.237 ns on the comb cascade→output
  path (16 levels, 11 CARRY4) → split the comb into 3 phases (register `k_res_r`
  between cascade and saturate); build 2 missed by −0.553 ns on the chirp
  `cycle_counter → stride-multiply → LUT → fifo_write_data` path → pipelined the
  chirp per-slot phase (`chirp_ch_phase` registered, multiply in its own stage);
  build 3 closed.

**Utilization — CIC (after) vs dual-MAC FIR (before):**
| resource | dual-MAC FIR (USE_CIC=0) | **CIC chain (USE_CIC=1, shipped)** | Δ |
|----------|------------------------|-----------------------------------|---|
| Block RAM tiles | 98.5 / 140 (70%) | **46.5 / 140 (33%)** | **−52 tiles** |
| RAMB36 | 96 | **44** | **−52** |
| RAMB18 | 5 | 5 | 0 |
| DSP48E1 | 3 | **1** | −2 |
| Slice LUTs | 13 912 (26%) | 16 559 (31%) | +2 647 |
| Slice FFs | 20 647 (19%) | 33 439 (31%) | +12 792 |
| WNS @ 84 MHz | +0.413 ns | **+0.417 ns** | both closed |

**The BRAM win (the whole point): −52 RAMB36 (≈37% of the 140-tile device
freed).** The CIC replaces the FIR's `8×32×256×16-bit` delay lines (≈64 RAMB36 in
the dual-MAC build) with per-channel integrator/comb register state in BRAM (the
LFP engine's BRAM is now dominated by the halfband's short delay line + the
CIC/comb state ≈ a handful of RAMB36). The cost is FFs (per-channel CIC state +
pipeline regs), 19% → 31% used — ample. This directly gives Phase B's BRAM-bound
256-ch scalogram + DDR staging its headroom.

**Firmware:** `scripts/create_vitis_project.py` rebuilds the platform from the new
`.xsa` and compiles **both cores at −O3** (`_Static_assert(sizeof(status_response_t)
==168)` passes; chirp fns / CMD_SET_CHIRP / status fields clean). The firmware is
datapath-agnostic — the host (`configure_lfp(datapath="cic")`) uploads the 43-tap
comp-FIR; the engine's ÷10 is hardwired.

## Known issues / decisions left for the user

1. **(RESOLVED) CIC bit-exactness.** The earlier comb read-latency bug is fixed:
   the per-channel state is now stored as ONE wide word (read → combinational
   cascade → write-back), the BRAM read address is **combinational** (so the
   registered read is valid the same cycle the FSM consumes it), and the packet is
   **snapshotted** into `in_snap` at `packet_tick` (the prior single-buffered
   `in_buf` was overwritten by the next packet mid-integrate-pass — a real HW bug
   the realistic-cadence chain TB caught). CIC + chain are bit-exact. Historical
   detail of the original bug:
   the fix is to drive `integ_addr`/`comb_addr` **combinationally** from the FSM state
   (so the registered read captures the right address at the phase edge) rather
   than registering them; a 3-phase schedule would also work but 256×4×3 = 3072 >
   2800 clk/packet → would overrun the integrate budget, so the combinational-
   address fix is the right one — and that is exactly what shipped. The CIC and the
   ÷10 chain are now bit-exact; **no open CIC correctness issue.**

2. **BRAM headroom for Phase B — solved.** The shipped CIC build uses **46.5/140
   tiles (33%)**, freeing ~52 RAMB36 vs the dual-MAC FIR. Phase B's 256-ch
   scalogram + DDR staging now has comfortable headroom (and the `lfp_halfband.sv`
   ÷2 block is ready to reuse). FFs rose to 31% (CIC per-channel state) — still
   ample.

   *Latency note:* the CIC group delay differs from a 131-tap FIR. The shipped
   chain's group delay = the comp-FIR (43 taps @ 6 kHz ≈ 3.5 ms) + the CIC^4 (~4
   half-lengths at the intermediate rate). The ephys-socket plugin's documented
   FIR-alignment offset must be re-derived for the CIC chain (it is NOT the
   131-tap/3 kHz ≈ 21.7 ms figure quoted for the FIR fallback). Compute/measure on
   HW and update the plugin.

3. **No hardware validation.** Everything here is sim + timing-closed builds. The
   on-bench checks the plan lists (256-ch LFP @ 3 kHz, clean passband to ~1 kHz via
   the chirp/swept generator, anti-alias rolloff, 1.5 MB/s sustained, ephys-socket
   3 kHz in Open Ephys) remain to be run on the board, plus the ephys-socket plugin
   rate bump (2 kHz→3 kHz, derive from the packet R field; re-derive the CIC-chain
   group-delay offset — see note in issue 2) in a separate repo not touched here.

   *256-ch real-time budget:* verified in sim for the dual-MAC FIR (7168 outs at
   the true ~2800-clk/packet cadence, no overrun). The CIC chain's per-tick
   integrate (2·256=512 clk) + per-frame comb (3·256=768 clk) both fit far inside
   their windows; `cic_chain_tb` runs at realistic cadence with no overrun.

4. **Chirp scaling knobs.** f_span step = `<<16` (≈0.46 Hz/step, full ≈1.9 kHz);
   sweep_rate step = `<<9`. net.py `configure_chirp(f_max, period_s, stride)`
   converts Hz/seconds to fields. Defaults: sweep 0→~1.4 kHz. Reasonable, but the
   exact f_min (currently ~0) and triangle-vs-one-shot behavior are easy to retune
   if the bench wants a different sweep shape (e.g. an exponential/log sweep for the
   constant-Q scalogram — a `[1]` reserved bit in CTRL_REG_3 is parked for it).

## Datapath select
`lfp_dsp_block.sv` parameter **`USE_CIC` (default 1)**:
- `USE_CIC=1` (shipped): `cic_decimator(÷5)` → `cic_to_halfband` glue →
  `lfp_halfband(÷2)`. Host: `configure_lfp(datapath="cic")` uploads the 43-tap
  comp-FIR; ÷10 hardwired.
- `USE_CIC=0` (fallback, in history): `lfp_fir_decimator` dual-MAC 131-tap FIR.
  Host: `configure_lfp(datapath="fir")`.

## Files touched
- RTL: `cic_decimator.sv` (new, shipped), `lfp_halfband.sv` (new, shipped),
  `cic_to_halfband.sv` (new, glue), `lfp_dsp_block.sv` (USE_CIC select),
  `lfp_fir_decimator.sv` (dual-MAC fallback), `data_generator_core.sv` (chirp NCO).
- Firmware: `include/main.h`, `src-core0/pl_control.c`, `network.c`, `main.c`.
- Host: `remote/net.py` (CIC comp-FIR designer + datapath select + chirp).
- Sim: see the table above.
- Docs: this file + `docs/lfp-dsp-engine-design.md` (rate note).
