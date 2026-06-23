# Tier-3 multirate wavelet (constant-Q / Morse) scalogram engine — design

Status: **implemented + sim-verified** (branch `claude/tier3-wavelet`). This is Phase B of
the LFP→3 kHz→scalogram plan. It builds an on-PL, IP-free, pure fixed-point bank of complex
constant-Q bandpass voices, producing a real-time scalogram of K selected channels.

## 1. What it is

A **multirate octave cascade (à trous / dyadic)**: per channel, `V` complex bandpass voices
per octave, with a halfband ÷2 feeding the next octave. The cost is ≈2× the top octave
regardless of octave count. The engine is **shape-agnostic** — complex FIR voices with
**host-uploaded Q1.17 complex coefficients**; the default coefficients are a generalized
**Morse wavelet (γ=3)** designed on the host (`remote/net.py:design_wavelet_bank`).

```
3 kHz LFP ─► [selector: K of 256] ─► OCTAVE CASCADE (per channel):
   octave o @ 3 kHz/2^o:  V complex voices ─► (re,im)/scale ;  halfband ÷2 ─► octave o+1
   ─► pack (re,im)/(lane,scale) ─► results BRAM @ 0x90000000
   ─► core 0: rate-limited single-beat read ─► UDP monitor (5004)
```

First build: **K=32 host-selected channels, N_OCTAVES=8, V=4, N_TAPS=24** → 32 scales,
~2–512 Hz at fs=3000 (the plan's frequency table). One time-shared MAC (complex = 2 real
MACs/tap, but the LFP input is real so it is literally `acc_re += cre·x`, `acc_im += cim·x`).

## 2. Frequency grid (V=4, fs=3000, fc_top=0.34·octave-Nyquist-band)

The same V normalized voice shapes are reused at every octave (constant-Q); the center
frequency in Hz halves each octave because the octave sample rate halves. The default
designer's centers (Hz):

| octave | rate (Hz) | center freqs (Hz) |
|---|---|---|
| 0 | 3000 | 1020 858 721 607 |
| 1 | 1500 | 510 429 361 303 |
| 2 | 750  | 255 214 180 152 |
| 3 | 375  | 128 107 90 76 |
| 4 | 187.5| 64 54 45 38 |
| 5 | 93.75| 32 27 23 19 |
| 6 | 46.9 | 16 13 11 9.5 |
| 7 | 23.4 | 8 6.7 5.7 4.8 |

(`fc_top` and per-voice β/Q are host knobs; this default leans high to exercise the new
fast-ripple/HFO octave. Retune to the plan's exact table by lowering `fc_top` — no rebuild,
just re-upload coefficients.)

## 3. RTL

| File | Role |
|---|---|
| `wavelet_cqt_engine.sv` | the engine: one time-shared MAC for both the halfband ÷2 cascade and the V complex voices; per-(lane,octave) sample rings; dyadic schedule; per-octave output gain; `overrun` guard; results pack. |
| `wavelet_halfband.sv` | the reusable ÷2 anti-alias FIR primitive (the octave building block). |
| `wavelet_dsp_block.sv` | integration wrapper: host upload sequencer (strobe/toggle/target) + control decode + results BRAM port. |

### MAC discipline
One ring read port, one voice-coef read port, one halfband-coef read port, one MAC pipeline.
The FSM sequences (a) **halfband passes** — `HB_TAPS` real MACs producing one ÷2 sample
stored back into the next octave's ring, and (b) **voice passes** — `2·n_taps` real MACs
(re then im phase, the ring slot read once per re/im pair) producing one complex bin.
Registered 3-stage pipeline (s0 addr-gen, s1 read+product, s2 accumulate/emit), same shape
as `lfp_fir_decimator.sv`, for 84 MHz timing.

### Dyadic schedule
On every base-rate LFP frame the engine commits each lane's new octave-0 sample, then
cascades the halfband for every octave that advances this frame (octave `o` advances iff
`fcount mod 2^o == 0`), then runs the V voices for every advanced octave. Worst case
(octave 0, every frame): `K·V·2·N_TAPS` MAC cycles + cascade ≈ a few thousand clocks — tiny
vs the ~28 000-clock base-frame budget at 3 kHz. `overrun` latches (sticky) if a fresh frame
arrives mid-pass — the late frame is dropped, never corrupted (lfp_fir_decimator's guarantee).

### Fixed-point
- input samples: int16 signed (offset-binary removed upstream by `lfp_dsp_block`).
- coeffs: signed Q1.17 (18-bit field), `re,im` interleaved in the coef RAM.
- complex MAC accumulators are wide signed (ACC_W=48).
- voice output: round-to-nearest then `>> (COEF_FRAC − gain[octave])` then saturate to
  OUT_W=18. Per-octave `gain` is a left-shift to recover 1/f dynamic range.
- halfband ÷2: round-to-nearest `>> COEF_FRAC`, saturate to int16, decimation-aligned so
  output `m` consumes the source's newest sample at index `2m` (the natural causal polyphase
  decimator).

### Results BRAM layout (@ 0x90000000, 32-bit words)
Per lane: `N_OCTAVES·V` complex bins, each `{re word, im word}`.
`word(lane,scale) = (lane·N_OCTAVES·V + scale)·2`; `+0`=re, `+1`=im; `scale = octave·V +
voice`. Values signed OUT_W sign-extended to 32. It is a **snapshot** (overwritten each
column), not a ring. Octaves that did not advance this frame keep their previous column.

## 4. Integration contract (3 layers, kept in sync)

| Layer | What |
|---|---|
| PL | control regs 28(cfg) / 29(gain) / 30(data) / 31(strobe); status reg 14; results BRAM `0x90000000` via `axi_bram_ctrl_2` on `smartconnect_1` (NUM_MI 3→4, M03). |
| Firmware | `CMD_WAV_*` = 0x88–0x8C (NOT 0x90 = UDP bench; 0x84–0x87 left free for a future STFT merge); `pl_wav_*` upload helpers; `wav_stream_service` (UDP 5004); `status_response_t` +20 bytes (`_Static_assert` 180). |
| Host (net.py) | `design_wavelet_bank(V,n_octaves,fs=3000,gamma=3,beta=...)`, `configure_wavelet`, `wavelet_enable`, `receive_wavelet`; `get_status` decode/print; menu `wav_config`/`wav_on`/`wav_off`/`wav_recv`. |

**Coexists with STFT** (a future merge): distinct reg block (28–31), BRAM 0x90000000, UDP
5004, cmd 0x88. The STFT's 0x84–0x87 / reg 28–30 / 0x88000000 / 5003 are deliberately not
reused — but note reg 28–30 *do* overlap STFT's reg block, so a 3-way merge must re-slot one
engine's control regs (there are free regs above 31 if N_CTRL grows).

### Upload protocol
Host clears the pointer (with the target selector held), then streams, per target:
- voice coef (target 0): `2·V·N_TAPS` signed Q1.17 words, `re,im` interleaved.
- halfband (target 1): `HB_TAPS` signed Q1.17 words.
- selector (target 2): K channel indices.

The host `design_wavelet_bank` is verified **byte-identical** to the sim reference
`gen_wavelet_vectors.py`, so the board runs exactly the coefficients the testbench proved.

## 5. Monitor read path (important caveat)

The `claude/tier2-stft` branch found that a **CDMA read from a results BRAM HANGS on real
hardware** (it reverted to a rate-limited single-beat `Xil_In32` read). So `wav_stream_service`
uses a **single-beat read** of the results BRAM, polling the column counter (STATUS_REG_14)
and shipping a self-describing surface packet on UDP 5004, with a torn-surface re-read guard.
The full DDR-resident path (CDMA → DDR ring → soft-core) is **v2**.

## 6. Verification

- `programmable_logic/sim/gen_wavelet_vectors.py` — pure-Python (no numpy) Morse designer +
  multirate CQT reference on synthetic multi-tone + injected ripple + per-lane LFP; emits
  bit-exact `.hex` vectors.
- `programmable_logic/sim/wavelet_engine_tb.sv` — compares the PL (re,im) per
  (lane,octave,voice) to the Python reference **bit-exact (zero tolerance)**. PASS:
  128/128 bins (K=4, 4 octaves, 4 voices), `overrun=0`, `frame_seq=256`.
- `programmable_logic/sim/wavelet_halfband_tb.sv` — the ÷2 primitive unit test. PASS.
- Run: `source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_wavelet_tb.sh`.

## 7. Out of scope for v1 (future)

- 256 channels (2 MAC lanes + lazy work-spread over each octave's 2^o-frame slack).
- DDR-resident full-resolution surface for the soft-core consumer.
- θ-phase predictor pairing for phase-targeted stim.
- Live coefficient retune (ping-pong double-buffer).
- Reconciliation with Phase A's `lfp_halfband.sv` (see PHASE_B_SUMMARY.md).
