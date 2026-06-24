# Phase B — Tier-3 multirate wavelet scalogram engine — summary

Branch: `claude/tier3-wavelet` (off `main`). No push / no merge (per the task). Status below.

## TL;DR

- **Sim verification (the #1 deliverable): PASS, bit-exact.** The PL scalogram matches the
  pure-Python generalized-Morse (γ=3) multirate reference to **zero tolerance** — 128/128
  complex (re,im) bins, no overrun, all 256 frames processed.
- **Full PL integration done** (RTL wrapper, AXI regs, block design, firmware, net.py) and
  the **bitstream builds with timing CLOSED: WNS = +0.286 ns @ 84 MHz, 0 failing endpoints**
  (after pipelining one wavelet path — see §4).
- IP-free / pure fixed-point throughout. **4 DSP48 / 220, BRAM 64.6 % / 140** for the whole
  design at K=32.

## 1. What is implemented (done)

### RTL (`programmable_logic/src/`)
- **`wavelet_cqt_engine.sv`** — the engine. One time-shared MAC computes both the octave
  cascade's halfband ÷2 decimations and the V complex Morse voices/octave. Complex Q1.17
  coefficients (re/im interleaved coef RAM); per-(lane,octave) sample rings in BRAM; dyadic
  à-trous schedule (octave o advances every 2^o base frames); per-octave programmable output
  gain (1/f dynamic range); `overrun` guard (late frame dropped, never corrupted). Registered
  3-stage MAC pipeline + a pipelined emit (see §4) for 84 MHz. Results packed (re,im)/(lane,scale)
  to a results BRAM @ 0x90000000. Build params: K=32, N_OCTAVES=8, V=4, N_TAPS=24, HB_TAPS=7.
- **`wavelet_halfband.sv`** — the reusable decimate-by-2 anti-alias FIR primitive (the octave
  building block), with its own unit test.
- **`wavelet_dsp_block.sv`** — integration wrapper: host upload sequencer (strobe/toggle +
  2-bit target: voice coef / halfband / selector) + control decode + results BRAM port.
- **`lfp_dsp_block.sv`** — exposed the decimated signed LFP output tap
  (`lfp_out_valid/channel/data/frame_start`) for the wavelet consumer (mirrors the
  `claude/tier2-stft` lfp tap).

### Sim (`programmable_logic/sim/`)
- **`gen_wavelet_vectors.py`** — pure-Python (no numpy) generalized-Morse (γ=3) complex voice
  designer + halfband + multirate CQT reference on synthetic LFP (per-lane multi-tone +
  injected ripple, à la `ripple_detect_prototype.synth_lfp`). Emits bit-exact `.hex` vectors.
- **`wavelet_engine_tb.sv`** — drives K=4 lanes over 256 frames, snoops the results BRAM,
  compares (re,im) per (lane,octave,voice) to the reference **bit-exact**. **RESULT: PASS.**
- **`wavelet_halfband_tb.sv`** — the ÷2 primitive unit test. **RESULT: PASS.**
- Runners: `run_wavelet_tb.sh`, `run_wavelet_halfband_tb.sh` (xsim, like the LFP/STFT TBs).

### Integration (the 3-layer contract, kept in sync)
- **PL**: control regs 28(cfg)/29(gain)/30(data)/31(strobe); status reg 14; results BRAM
  `0x90000000` via `axi_bram_ctrl_2` on `smartconnect_1` (NUM_MI 3→4, M03);
  `axi_lite_registers` N_CTRL 28→32 / N_STATUS 14→15; wrapper wiring + WAV_BRAM port.
- **Firmware**: `CMD_WAV_*` = 0x88–0x8C; `pl_wav_*` upload helpers; `wav_stream_service`
  (UDP 5004); `status_response_t` +20 bytes (`_Static_assert` 160→180); main.c hooks.
- **Host (`remote/net.py`)**: `design_wavelet_bank(V,n_octaves,fs=3000,gamma=3,beta=...)`,
  `configure_wavelet`, `wavelet_enable`, `receive_wavelet`; `get_status` decode/print;
  interactive menu `wav_config`/`wav_on`/`wav_off`/`wav_recv`.

### Host↔sim coefficient contract — verified identical
`net.py:design_wavelet_bank` was checked to produce **byte-identical** coefficients to the
sim reference `gen_wavelet_vectors.py` (voices and halfband). So the board runs exactly the
coefficients the testbench proved bit-exact.

## 2. Sim PASS — the bin-match result

```
WAVELET engine TB: checked=128 errors=0 overrun=0 frame_seq=256
RESULT: PASS
```
- K=4 lanes × 4 octaves × 4 voices = 16 scales × 2 (re,im) = 128 bins, **all match to 0 LSB**.
- The comparison tolerance is literally `d != 0` (exact), not a fuzzy float tolerance — the
  Python reference uses the identical integer MAC, round-to-nearest, arithmetic shift, and
  saturation as the RTL.
- `overrun=0`, `frame_seq=256` (every frame processed; the dyadic schedule + decimation
  alignment + complex accumulation all verified end-to-end through the 8-octave-capable
  engine at K=4 / 4 octaves).
- Halfband ÷2 primitive: `RESULT: PASS (halfband out=26)`.

Two real bugs were found and fixed by sim (both would have silently corrupted hardware):
1. **coef-select pipeline misalignment** — the product stage selected the coef RAM with the
   s2-stage `is_hb1` marker (one cycle too late vs the s1 ring/coef reads); the halfband used
   the voice coef RAM. Fixed by selecting with the s1 marker `ag_is_hb`.
2. **decimation phase** — the Python batch halfband used a group-delay-centered window while
   the RTL is a causal incremental polyphase decimator (output m consumes source index 2m).
   Reconciled the reference to the RTL's exact causal semantics.

## 3. Build — resources

Full-design utilization (xc7z020clg400-1, the whole board incl. broadband + LFP + wavelet):

| Resource | Used | Avail | % |
|---|---|---|---|
| Slice LUTs | 16906 | 53200 | 31.8 |
| Slice Registers | 25202 | 106400 | 23.7 |
| **Block RAM Tile** | **90.5** | 140 | **64.6** |
| **DSP48E1** | **4** | 220 | **1.8** |

The wavelet engine adds ~2–3 DSP48 (the complex MAC + halfband, time-shared) and the
per-(lane,octave) sample rings dominate the BRAM rise. DSP is right on the plan's estimate
(~1–3 for K=32). **BRAM is the resource to watch** (the plan's prediction) but is comfortable.
No Xilinx IP in the engine (pure fixed-point).

## 4. Build — timing closure

- **First build:** bitstream wrote successfully; timing **missed by 6 endpoints**,
  **WNS = −0.070 ns** on the 84 MHz clock. All 6 failing paths were the wavelet
  `im1_reg → v_emit_im` path: the last-tap beat did DSP-accumulate → round → variable
  barrel-shift → saturate (an 18-level path with an 11-deep CARRY4) in a single cycle — the
  known ~0.45 ns 84 MHz margin couldn't absorb it.
- **Fix:** pipelined the emit — on the last tap, register the *raw* accumulators + gain +
  routing; apply the round/variable-shift/saturate the *next* cycle. The FSM already waits on
  the emit pulse, so the extra latency is free in the per-frame budget. Sim re-verified
  bit-exact PASS after the change.
- **Final build: TIMING CLOSED. WNS = +0.286 ns, WHS = +0.050 ns, 0 failing endpoints**
  (84972 total) on the 84 MHz path. The worst remaining path is a reset synchronizer (MET,
  +7.3 ns) — no wavelet path is critical anymore. Bitstream + `.xsa` written.

> **Build gotcha (note for the user):** `scripts/build_bitstream.tcl`'s `reset_run synth_1`
> resets only the *top* run, NOT the per-module OOC synth run for `data_generator` (it is a
> BD module-reference). An incremental `build_bitstream.tcl` after editing
> `wavelet_cqt_engine.sv` therefore re-implemented the STALE pre-pipeline netlist (the second
> build still showed −0.070). The fix is to also `reset_run design_1_data_generator_0_synth_1`
> (see `build_logs/rebuild_ooc.tcl`) OR follow the canonical CLAUDE.md flow: re-run
> `create_vivado_project.tcl` (regenerates all OOC runs) after a PL change. The closed-timing
> result above is from a clean OOC re-synth.

> Reproduce: `source /opt/Xilinx/2025.1/Vivado/settings64.sh`, then
> `vivado -mode batch -source scripts/create_vivado_project.tcl` and
> `vivado -mode batch -source scripts/build_bitstream.tcl`. Timing summary at
> `vivado_project/klab_project.runs/impl_1/design_1_wrapper_timing_summary_routed.rpt`.

## 5. Monitor read path — now AXI CDMA (the STFT "hang" was a BD bug, root-caused)

> **UPDATE (branch `claude/tier3-wavelet`, AXI-CDMA switch):** the read path below has been
> replaced by AXI CDMA. See `docs/WAVELET_DMA_SUMMARY.md` for the full change. The original
> text is kept for history.

`wav_stream_service` now AXI-CDMAs the full results surface (0x90000000) into a non-cacheable
DDR staging buffer, then repacks the active sub-region into the UDP packet — the same CDMA path
the broadband capture uses. The earlier "CDMA read from a results BRAM HANGS" finding was **not**
a CDMA limitation: the results BRAM (`axi_bram_ctrl_2`) was reachable through the `smartconnect_1`
crossbar but its address (0x90000000) was never assigned into the CDMA's address space
(`axi_cdma_0/Data`), so the read decoded to nothing and `XAxiCdma_IsBusy` spun forever.
`design_1_bd.tcl` now assigns 0x90000000 into `axi_cdma_0/Data`. (HW-unvalidated — no board.)

_Original text:_ `wav_stream_service` reads the results BRAM with rate-limited single-beat
`Xil_In32`, NOT CDMA. The `claude/tier2-stft` branch found that a CDMA read from a results BRAM
hangs on real hardware (it reverted to single-beat). The wavelet monitor polls the column counter
(STATUS_REG_14), single-beat-reads the active surface, guards against a torn surface (re-read the
counter), and ships a self-describing packet on UDP 5004.

## 6. Phase-A reconciliation needed (developed independently)

Phase A (LFP 2→3 kHz) is **not merged**. This engine was developed against `fs=3000` and the
existing `lfp_dsp_block` per-channel interface, as instructed. Reconciliation when Phase A lands:
- Phase A introduces its own `lfp_halfband.sv` (the CIC + comp-FIR / halfband ÷2 for the
  decimation chain). My `wavelet_halfband.sv` is a *separate* ÷2 used only inside the octave
  cascade. They can coexist, but **consider sharing one halfband module** if the coefficient
  shape and width match — purely a cleanup, not a correctness issue.
- The wavelet engine consumes the `lfp_dsp_block` decimated output tap. Phase A may change the
  LFP tap (CIC chain) — verify `lfp_out_valid/channel/data/frame_start` semantics still hold
  (one signed sample per selected channel per LFP frame, frame-start on the first channel).
  Phase A raising R 15→10 changes only the LFP *rate* (2→3 kHz), which the wavelet engine
  already assumes; no engine change needed, but re-confirm the frame cadence.
- If Phase A also touches `data_generator_wrapper.v` / `axi_lite_registers.v` /
  `design_1_bd.tcl`, the merge is mechanical (Phase A edits the LFP regions; this branch adds
  regs 28–31 / status 14 / BRAM 0x90000000 — non-overlapping).

## 7. STFT-merge note

This branch is off `main`, so the STFT engine is *not* present here. The wavelet uses a fresh
non-colliding allocation (regs 28–31, BRAM 0x90000000, UDP 5004, cmd 0x88) — but **control
regs 28–30 overlap the STFT branch's reg block (28–30)**. A future 3-way merge must re-slot
one engine's control regs (free regs exist above 31 if N_CTRL grows); BRAM/UDP/cmd don't
collide. The 0x84–0x87 command block and 0x88000000 BRAM were deliberately left free for STFT.

## 8. What is stubbed / v2 (honest scope)

- **256 channels** — v2 (2 MAC lanes + lazy work-spread over each octave's 2^o-frame slack).
  K=32 is the first build, single MAC, naive per-frame scheduling (the plan's v1).
- **DDR-resident full-resolution surface** for the soft-core consumer — v2 (the UDP monitor
  is the consumer now; CDMA path blocked by the HW hang, see §5).
- **θ-phase predictor** pairing — future module.
- **Live coefficient retune** (ping-pong double buffer) — config latches while disabled.
- **HW validation** — no hardware available; everything above is sim + build only. The chirp /
  swept-generator HW checks in the plan's staging §3 are not run.

## 9. Decisions left for the user

1. **Default voice center frequencies.** The host designer's default `fc_top=0.34` leans the
   bands high (octave 0 ≈ 600–1020 Hz) to exercise the new fast-ripple/HFO octave. The plan's
   table peaks at 512 Hz. Lower `fc_top` (or pass per-voice β) to hit the exact table — it's a
   re-upload, no rebuild. Pick the production grid.
2. **Per-octave gains.** Default ramp `[3,2,1,0,0,0,0,0]` (boost high octaves). Tune for the
   real 1/f spectrum of the rig, or switch to a float output if fixed-point Q proves tight in
   the δ/θ bands (the plan's fallback).
3. **Monitor decimation / packet size.** The K=32 surface is 32×32 complex int32 ≈ 8 KB/packet
   (jumbo). If the host path isn't jumbo-capable, decimate the monitor (fewer scales/lanes) or
   raise the column-skip in `wav_stream_service`.
4. **HW bring-up** when a board is available: `wav_config` → `wav_on` → `wav_recv`, cross-check
   against the analytic chirp (log sweep traces a diagonal across scales) and a swept generator.
