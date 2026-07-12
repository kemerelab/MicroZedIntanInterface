# LFP extraction (Tier-1 on-PL DSP engine)

How the board turns the 30 kHz broadband neural stream into a decimated **local field
potential (LFP)** band, streamed as an independent data product. Part 1 is the signal-
processing theory (with references); Part 2 is the as-built PL implementation.

Companion doc: [`lfp-dsp-engine-design.md`](lfp-dsp-engine-design.md) (design rationale /
decision log). This doc is the reference for *what the engine is and why*.

---

## Part 1 — Signal-processing theory

### 1.1 What the LFP is

The extracellular potential recorded by a neural electrode is conventionally split into a
high-frequency band — spikes / multi-unit activity (~300 Hz–7 kHz) — and a low-frequency
band, the **local field potential** (~0.5–300 Hz), which reflects summed synaptic and
transmembrane currents in the local population [1][2]. Many experiments (theta/gamma
oscillations, sharp-wave ripples ~150–250 Hz, phase-targeted stimulation) only need the LFP
band, so it is worth extracting it as a compact, lower-rate stream rather than shipping the
full 30 kHz broadband.

Extracting the LFP is a **sample-rate conversion** problem: low-pass filter to the band of
interest, then **decimate** (downsample) to a rate just above its Nyquist requirement. Here:
30 kHz broadband → **3 kHz LFP** (decimation factor **R = 10**; Nyquist 1.5 kHz, usable band
to ~1 kHz). The raw broadband stream is never altered — LFP is a *derived* product on a
separate wire (data-fidelity rule).

### 1.2 Decimation and aliasing

Downsampling by R keeps every R-th sample, which folds (aliases) every input component above
the new Nyquist `f_s,out/2` back into the baseband [3][5][7]. To avoid corrupting the band of
interest you **must** low-pass *before* discarding samples, attenuating everything that would
alias onto `[0, f_pass]`. Concretely, with `f_s,out = 3 kHz`, any input energy near
`k·3 kHz ± [0,1 kHz]` would land in the passband after ÷10, so the anti-alias filter must be
down by the target stop attenuation across those folding bands. The design target here is
**flat to ~1 kHz, ≥46 dB alias rejection into the 0–1 kHz passband**.

### 1.3 Why multistage (CIC + compensation), not one big FIR

A single-stage FIR sharp enough for a ÷10 anti-alias at 30 kHz needs ~130–200 taps, and a
**time-shared** engine serving 256 channels then needs a large per-channel delay line (the
dominant memory cost). The classic, far cheaper approach for large integer decimation is
**multistage** [4][5][6]: do most of the rate reduction in a **cascaded integrator–comb
(CIC)** stage (no multipliers, tiny state), then a short FIR at the lower rate.

**CIC decimator (Hogenauer, 1981) [4].** A CIC of order `N` and rate-change `R` (differential
delay `M`) is `N` cascaded integrators at the *input* rate, a ÷R downsampler, then `N`
cascaded combs at the *output* rate. Its transfer function is

```
H(z) = [ (1 - z^-RM) / (1 - z^-1) ]^N      → magnitude ≈ | sin(π R M f / f_s) / sin(π f / f_s) |^N
```

i.e. a `sinc^N` low-pass with nulls at multiples of `f_s/(RM)` (exactly the images that fold
on decimation). Key properties:

- **No multipliers, no coefficient memory, no long delay line** — only `N` integrator and `N`
  comb accumulators per channel. This is the ~5× memory saving over the single-stage FIR.
- **DC gain `(R·M)^N`**, removed by an arithmetic right-shift (here `≈ R^N`).
- **Passband droop**: the `sinc^N` shape sags toward the band edge — corrected downstream.
- **Register bit-growth**: the accumulators must be wide enough that *modular* two's-complement
  wraparound is exact: `ACC_W ≥ input_W + ⌈N·log2(R·M)⌉` [4]. The integrators may overflow/wrap;
  the comb differences recover the correct result modulo `2^ACC_W` as long as the true output
  fits.

**Compensation / half-band FIR.** A short FIR at the CIC output rate (a) **flattens the CIC
passband droop** (its target is ≈ `1/sinc^N` over the passband) and (b) provides the final
sharp transition + a ÷2 decimation [5][6][7]. A half-band design is a natural fit for the ÷2.

### 1.4 The shipped filter: CIC⁴(÷5) → droop-comp half-band(÷2) = ÷10

| stage | rate in → out | role |
|------|---------------|------|
| CIC, order N=4, R=5, M=1 | 30 kHz → 6 kHz | bulk anti-alias + ÷5, multiplier-free |
| comp-FIR / half-band, 43 taps | 6 kHz → 3 kHz | flatten CIC droop + sharp edge + ÷2 |

**Combined response (as quantized to Q1.17):** flat to **~1 kHz (≤0.01 dB)**, −3 dB ≈ 1.25 kHz,
−6 dB ≈ 1.30 kHz, **−32 dB at the 1.5 kHz Nyquist**, **−54 dB worst alias-into-passband** —
comfortably past the 46 dB target. CIC order 3 only reached −42 dB, hence order 4. DC gain is
restored to unity by the comp-FIR (the CIC's `R^N/2^GAIN_SHIFT = 625/1024 = 0.61` is cancelled
by the comp-FIR's design-time normalization).

The comp-FIR is designed on the host (`design_cic_comp_fir` in `remote/net.py`) by
**frequency sampling** [7] with the desired passband response set to `1/CIC-droop`, windowed
(Kaiser β=6.0 [9]), normalized to unity combined DC gain, and quantized to Q1.17. The host
owns the filter design; the PL is a fixed-point engine.

> **Group delay.** Linear-phase FIR + CIC give a fixed, documented group delay; it changed
> with the CIC chain vs the legacy single-stage FIR (recompute it for absolute LFP↔broadband
> alignment). A *phase-targeted* closed loop (e.g. theta-phase stim) wants a low-order IIR or
> a phase predictor instead — noted as a future module.

---

## Part 2 — PL implementation

### 2.1 Datapath

```
data_generator_core (84 MHz, acquisition FSM)
   │  per-amplifier-channel sample tap (offset-binary), 1 word/slot @ packet_tick (30 kHz)
   ▼  offset-binary → two's-complement signed (^0x8000)   [lfp_dsp_block.sv]
cic_decimator.sv      CIC^4, ÷5   (integrators @ 30 kHz, combs @ 6 kHz)   → 6 kHz
   ▼
cic_to_halfband.sv    glue / handoff
   ▼
lfp_halfband.sv       droop-comp half-band FIR, ÷2                         → 3 kHz
   ▼  signed → offset-binary (^0x8000) ; write LFP output ring
LFP output BRAM @ 0x84000000  (64 KB, 2nd axi_bram_ctrl)
   ▼  AXI CDMA (BRAM→DDR over HP0), reusing the broadband path
PS core 0  → UDP stream, port 5001 (one LFP frame per packet)
```

All of this is wired in **`lfp_dsp_block.sv`**, selected by the `USE_CIC` parameter
(default **1** = the CIC chain above; **0** = the single-stage FIR fallback, §2.5).

### 2.2 Time-shared architecture

One datapath serves **all 256 channels** (8 lanes × 32 amplifier slots). Every channel
advances together — exactly one new sample per 30 kHz `packet_tick` — so a single schedule
walks `(lane, slot)` and **per-channel state lives in BRAM**, not in replicated logic. The
budget is generous: ~2800 PL clocks per 30 kHz tick (and ~14 000 between the ÷5 comb passes).
A `compute_overrun` flag latches (and the late frame is dropped, never corrupted) if a pass
can't finish in its window.

### 2.3 CIC engine — `cic_decimator.sv`

- Parameters: `R=5, N_ORDER=4, M=1, ACC_W=32, GAIN_SHIFT=10, OUT_W=16`. The bit-growth check
  is `ACC_W ≥ 16 + ⌈4·log2(5)⌉ = 26`, so 32 is comfortable [4].
- **State packing:** a channel's `N` integrator accumulators (and `N` comb accumulators) are
  packed into one wide BRAM word at `address = channel`. Each channel is a clean single read →
  combinational `N`-stage cascade → single write-back (no per-stage RAM addressing, no
  read-latency hazards). Read addresses are *combinational*; writes are registered.
- **Schedule:** the INTEGRATE pass runs every tick (per channel: read packed integrators,
  cascade-add the input, write back); on the R-th tick the COMB pass runs after it (read the
  top integrator = decimated value + packed comb state, cascade-difference, write back, emit).
- **Packet snapshot:** the integrate pass reads `in_snap` — a per-`packet_tick` freeze of the
  arriving sample word — not the live `in_buf`. On real hardware the *next* packet's slots
  start arriving ~80 clk after `packet_tick`, mid-pass, and would otherwise corrupt late-lane
  reads. (This is a general rule for any multi-packet compute pass; see `CLAUDE.md`.)
- **Output:** the final comb value `>>> GAIN_SHIFT` (≈ unity DC after comp-FIR), saturated to
  `OUT_W`. Integers wrap modulo `ACC_W`; the comb differences recover the exact result [4].

### 2.4 Compensation half-band + coefficients

`cic_to_halfband.sv` hands the 6 kHz CIC output to **`lfp_halfband.sv`** (the 43-tap
droop-comp FIR, ÷2 → 3 kHz). The taps are **host-designed and uploaded** — the PL holds no
filter design:

- Host: `design_cic_comp_fir(num_taps=43, fc=1300, beta=6.0, R_cic=5, n_order=4,
  gain_shift=10)` in `remote/net.py` → Q1.17 signed taps. **Must match**
  `programmable_logic/sim/gen_cic_chain_vectors.py` bit-for-bit.
- Upload: the **indirect-window** pattern — write taps through `CTRL_REG_LFP_COEF` (reg 26,
  `[17:0]` Q1.17) with a `CTRL_REG_LFP_STROBE` (reg 27) toggle/clear, streamed over TCP
  (`CMD_LFP_WRITE_COEF`). Coefficients latch only while streaming is stopped.

### 2.5 Fallback — single-stage dual-MAC FIR (`USE_CIC=0`)

`lfp_fir_decimator.sv` is a time-shared **dual-MAC** decimating FIR (131-tap Kaiser, fc=1250,
direct ÷10). It is the BRAM-heavy fallback (delay lines push BRAM to ~70% vs ~33% for the CIC)
and is timing-closed; kept for comparison and as a known-good reference. The host uploads its
131 taps with `configure_lfp(datapath="fir")`.

### 2.6 Sample format (offset-binary)

Intan amplifier samples are **offset-binary** (mid-scale `0x8000` = 0 µV). The DSP engine is
pure two's-complement, so the integration boundary converts in/out with a symmetric MSB
invert: `engine_in = raw ^ 0x8000`, `lfp_out = engine ^ 0x8000`. LFP therefore ships in the
**same offset-binary format as broadband**, so the host de-offsets both identically. Only the
32 amplifier converts per lane are filtered (aux slots dropped, 2-cycle SPI readback offset
removed upstream).

### 2.7 Control / status contract (keep the 3 layers in sync)

| reg | field |
|----|-------|
| `CTRL_REG_LFP_CFG` (25) | `[0]` enable · `[15:8]` lane_mask · `[23:16]` decim_R (=10) · `[31:24]` num_taps |
| `CTRL_REG_LFP_COEF` (26) | `[17:0]` signed Q1.17 coefficient |
| `CTRL_REG_LFP_STROBE` (27) | `[0]` coef write toggle · `[1]` pointer clear |
| `STATUS_REG_13` | `[15:0]` LFP BRAM write byte-addr · `[16]` overrun |

Commands `CMD_LFP_ENABLE/SET_PARAMS/SET_CHANNELS/WRITE_COEF` (`0x80–0x83`). `get_status`
mirrors the full LFP config (rule: everything configurable is read-back-able). The three
encodings — `axi_lite_registers.v` + `data_generator_core.sv` (PL), `firmware/include/main.h`
(PS), `remote/net.py` (host) — must change together.

### 2.8 Transport — UDP port 5001

LFP BRAM @ `0x84000000` (64 KB ring) → CDMA → DDR → lwIP UDP, **one LFP frame per packet**.
Packet = 6-word header then offset-binary 16-bit samples (enabled lanes × 32 ch, 2 per word):

| words | field |
|------:|-------|
| 0–1 | magic `0xCAFEBABE_1F1FBEEF` (distinct from broadband's `…DEADBEEF`) |
| 2–3 | 64-bit timestamp (master count at the decimation instant; every R-th broadband tick) |
| 4 | `[7:0]` lane_mask · `[15:8]` decim_R · `[23:16]` num_taps · `[24]` overrun |
| 5 | 32-bit sequence number (LFP-stream drop detection) |
| 6… | payload: offset-binary 16-bit samples (subtract `0x8000` for signed) |

The packet is **self-describing** (`decim_R` rides in word 4), so a host/plugin derives the
rate as `30 kHz / decim_R` and auto-tracks it (the Open Ephys `ephys-socket` plugin does this).

### 2.9 Resources & timing

CIC chain @ `xc7z020clg400-1`: **BRAM ≈ 33%** (vs ~70% for the FIR fallback — the CIC's ~5×
delay-line saving), **1 DSP48**, **WNS ≈ +0.42 ns @ 84 MHz** (0 failing endpoints). LFP stream
bandwidth = `256 ch × 3 kHz × 2 B = 1.5 MB/s` (+8.3 % over the ~18 MB/s broadband).

---

## Part 3 — Validating the filter from the host

The analytic **chirp** debug NCO (`data_generator_core.sv`, `chirp_mode` in `CTRL_REG_3`,
`CMD_SET_CHIRP`) replaces the live data with a swept sine, and **`lfp_sweep`** in `net.py`
measures the realized magnitude response: it drives the chirp across `[0, f_max]`, captures one
LFP channel off UDP 5001, and per short window estimates the dominant frequency (Goertzel peak)
+ amplitude → bins amplitude vs frequency = measured `|H(f)|`, printed as a dB table with the
measured −3/−6/−20 dB crossings next to the predicted values.

```
lfp_sweep                # f_max=1490 (just under the 1.5 kHz LFP Nyquist), period=3 s
```

Reading the **raw** LFP stream this way isolates the anti-alias filter from any display-side
high-pass in the viewer. Expect: flat to ~1 kHz, −3 dB ≈ 1.25 kHz, rolling into the stop by
1.5 kHz.

---

## References

1. Buzsáki G., Anastassiou C. A., Koch C. (2012). *The origin of extracellular fields and
   currents — EEG, ECoG, LFP and spikes.* Nat Rev Neurosci 13:407–420.
   https://doi.org/10.1038/nrn3241
2. Einevoll G. T., Kayser C., Logothetis N. K., Panzeri S. (2013). *Modelling and analysis of
   local field potentials for studying the function of cortical circuits.* Nat Rev Neurosci
   14:770–785. https://doi.org/10.1038/nrn3599
3. Shannon C. E. (1949). *Communication in the presence of noise.* Proc. IRE 37(1):10–21.
   https://doi.org/10.1109/JRPROC.1949.232969  (Nyquist–Shannon sampling)
4. Hogenauer E. B. (1981). *An economical class of digital filters for decimation and
   interpolation.* IEEE Trans. ASSP 29(2):155–162.
   https://doi.org/10.1109/TASSP.1981.1163535  (the CIC filter)
5. Crochiere R. E., Rabiner L. R. (1983). *Multirate Digital Signal Processing.* Prentice-Hall.
6. Lyons R. G. (2010). *Understanding Digital Signal Processing,* 3rd ed., ch. 10
   (multirate / CIC). Prentice-Hall.
7. Oppenheim A. V., Schafer R. W. (2009). *Discrete-Time Signal Processing,* 3rd ed.
   (FIR design, frequency sampling, Kaiser window). Pearson.
8. Donadio M. (2000). *CIC Filter Introduction.* dspGuru application note (dspguru.com) —
   a widely-cited practical CIC derivation.
9. Harris F. J. (1978). *On the use of windows for harmonic analysis with the discrete Fourier
   transform.* Proc. IEEE 66(1):51–83. https://doi.org/10.1109/PROC.1978.10837
10. AMD/Xilinx, *CIC Compiler LogiCORE IP Product Guide (PG140)* — practical FPGA CIC reference.

## File map

| File | Role |
|------|------|
| `programmable_logic/src/lfp_dsp_block.sv` | top wrapper; `USE_CIC` select; offset conv; BRAM writer |
| `programmable_logic/src/cic_decimator.sv` | CIC⁴ ÷5, time-shared, packed per-channel state |
| `programmable_logic/src/cic_to_halfband.sv` | CIC → half-band handoff |
| `programmable_logic/src/lfp_halfband.sv` | droop-comp half-band FIR ÷2 |
| `programmable_logic/src/lfp_fir_decimator.sv` | single-stage dual-MAC FIR (`USE_CIC=0` fallback) |
| `remote/net.py` | `design_cic_comp_fir`, `configure_lfp`, `receive_lfp`, `lfp_sweep` |
| `firmware/src-core0/pl_control.c`, `network.c` | `pl_lfp_set_config`, coef upload, UDP 5001 stream |
| `firmware/include/main.h` | LFP registers, packet layout, status struct |
| `programmable_logic/sim/gen_cic_chain_vectors.py` + `*_tb.sv` | bit-exact reference + testbenches |
| `docs/lfp-dsp-engine-design.md` | design rationale / decision log |
