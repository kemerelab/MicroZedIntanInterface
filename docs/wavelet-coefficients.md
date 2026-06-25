# Wavelet bank coefficients — `claude/tier3-wavelet-fine` (finer-grid build)

This branch streams a complex wavelet scalogram (UDP 5004) computed on the
3 kHz CIC LFP. It is the **finer-grid / sharper** variant: more voices per
octave (`V=6`) and longer voice FIRs (`N_TAPS=40`) than the standard build, for
denser frequency coverage and tighter passbands. The voices are **true
generalized-Morse wavelets** — exactly
[ghostipy](https://github.com/kemerelab/ghostipy)'s `MorseWavelet(gamma=3,
beta=3)` — so out-of-band / offline analysis with ghostipy lines up with the
hardware. `docs/wavelet_coeffs.py` regenerates **the exact Q1.17 taps this
branch uploads to the PL**.

```bash
python3 docs/wavelet_coeffs.py            # print Q1.17 ints, float taps, center freqs
python3 docs/wavelet_coeffs.py --validate # assert bit-exact vs the sim reference
python3 docs/validate_against_ghostipy.py # assert the taps == ghostipy Morse (needs ghostipy)
```
```python
import wavelet_coeffs as w
bank = w.coefficients()
bank['voices'][v][n]['c']   # complex float tap = silicon value (int / 2**17)
bank['halfband'][n]['c']    # halfband /2 tap
bank['centers_hz'][o][v]    # analysis center frequency (Hz) of voice v, octave o
```

`--validate` cross-checks the generator against
`programmable_logic/sim/gen_wavelet_vectors.py` — the same bit-exact reference
the wavelet-engine testbench is checked against, and which net.py's
`design_wavelet_bank()` (the actual uploader) mirrors. All three agree
voice-for-voice and tap-for-tap (**passes**). The bit-exact engine TB
(`run_wavelet_tb.sh`) passes with this bank (`errors=0`).

> **Coefficients are uploaded at runtime, not baked into the bitstream.** The PL
> engine is shape-agnostic; switching the wavelet family is a `net.py` re-upload
> (`configure_wavelet`), **no PL/firmware rebuild**. The same bitstream runs any
> bank `wavelet_coeffs.py` describes.

## The wavelet family — generalized Morse (ghostipy)

Frequency domain (ghostipy convention, reproduced bit-for-bit):

```
Psi(w) = a · w^beta · exp(-w^gamma),   w > 0        (analytic: zero for w <= 0)
log a  = ln2 + (beta/gamma)·(1 + ln gamma - ln beta)   (peak value 2, "bandpass")
wp     = (beta/gamma)^(1/gamma)                         (peak angular frequency)
```

Each voice is the inverse FT of `Psi(s·w)` at scale `s = wp / (2π·fc)`, sampled to
`N_TAPS` complex taps, zero-meaned (DC rejection — true Morse already has
`Psi(0)=0`), L1-normalized, and Q1.17-quantized. Voice `v` centers at
`fc = fc_top·2^(-v/V)` cycles/sample.

`docs/validate_against_ghostipy.py` confirms, per voice: our analytic formula
equals `ghostipy.spectral.MorseWavelet.freq_domain()` to **0.0**, and the finite
40-tap FIR's frequency response matches ghostipy's wavelet in shape (same peak
frequency; passband magnitude error **< 6e-5**; correlation 1.00000). The longer
`N_TAPS=40` voices of this fine build truncate less than the standard 24-tap
build, so the passband error is ~3× smaller.

**`gamma`/`beta`.** `gamma=3` is the locked default (γ=3 Morse is exactly analytic
even at short durations). `beta` is the **Q / bandwidth knob**: ghostipy's default
is `beta=20` (narrowband, many oscillations); this bank uses **`beta=3`** — broad,
so a true Morse atom fits `N_TAPS=40` taps without truncation artifacts. Raising
`beta` sharpens frequency resolution but needs more taps / longer latency.

### Matching outputs in ghostipy

To analyze data with **the identical filter the hardware applies**, use the taps
from `wavelet_coeffs.py` directly as a complex FIR (with the à trous cascade
below) — that is a *bit-exact* match. If instead you run `ghostipy.cwt(...)` with
`MorseWavelet(gamma=3, beta=3)`, you get the **same wavelet**, but ghostipy
normalizes to peak-2 in frequency while the FIR is L1-normalized in time — so the
two agree **up to a constant per-voice gain** (and the FIR's finite-length
truncation). Same shapes, same center frequencies; only an amplitude scale differs.

## This branch's configuration

| param | value | meaning |
|------|------|---------|
| `V` | 6 | voices (sub-bands) per octave |
| `N_TAPS` | 40 | complex FIR taps per voice |
| `N_OCTAVES` | 8 | octaves in the cascade |
| `HB_TAPS` | 7 | halfband /2 anti-alias taps (unchanged) |
| `fs` | 3000 Hz | LFP rate into octave 0 |
| `fc_top` | 0.34 | top-voice center (cycles/sample within an octave) |
| `gamma, beta` | 3, 3 | Morse shape params (β = Q knob) |
| coeff format | Q1.17 | signed 18-bit, value = `int / 2**17`, saturating |

These mirror `firmware/include/main.h` (`WAV_*`) and `remote/net.py`
(`design_wavelet_bank` defaults) for this branch.

### Analysis center frequencies (Hz) — `centers[octave][voice]`

`fc_top = 0.34` leans the grid **high** (top voice = 1020 Hz, into the
fast-ripple/HFO range), not the nominal ≤512 Hz table. Lower `fc_top` to retune.
The `V=6` grid puts 6 voices in each octave (vs 4 in the standard build), so the
spacing between adjacent center frequencies is a finer `2^(-1/6)` step.

| octave | rate (Hz) | v0 | v1 | v2 | v3 | v4 | v5 |
|---|---|---|---|---|---|---|---|
| 0 | 3000.0 | 1020.00 | 908.72 | 809.57 | 721.25 | 642.56 | 572.46 |
| 1 | 1500.0 | 510.00 | 454.36 | 404.79 | 360.62 | 321.28 | 286.23 |
| 2 | 750.0 | 255.00 | 227.18 | 202.39 | 180.31 | 160.64 | 143.11 |
| 3 | 375.0 | 127.50 | 113.59 | 101.20 | 90.16 | 80.32 | 71.56 |
| 4 | 187.5 | 63.75 | 56.79 | 50.60 | 45.08 | 40.16 | 35.78 |
| 5 | 93.75 | 31.88 | 28.40 | 25.30 | 22.54 | 20.08 | 17.89 |
| 6 | 46.88 | 15.94 | 14.20 | 12.65 | 11.27 | 10.04 | 8.94 |
| 7 | 23.44 | 7.97 | 7.10 | 6.32 | 5.63 | 5.02 | 4.47 |

## How the hardware applies these (à trous octave cascade)

To reproduce the on-chip scalogram offline, match the **structure**, not just the
taps. The PL runs an à trous / dyadic cascade — the *same* `V` voice shapes are
reused at every octave, applied to a progressively halfband-decimated copy of the
LFP:

```
x0 = LFP @ 3 kHz
octave o:  for each voice v:  X[o][v] = complex_FIR(voice[v], x_o)   # at rate fs/2^o
           x_{o+1} = decimate_by_2(halfband, x_o)                    # /2 anti-alias
```

So octave `o` analyzes `x_o = LFP` decimated by `2**o` via the 7-tap halfband
applied `o` times, then convolved with the `N_TAPS` complex voice FIR. The voice
output is the complex (re, im) scalogram coefficient for that (octave, voice).
Per-octave output gains (host-configurable, default unity) are applied on top for
fixed-point dynamic range; `coefficients()` reports the unscaled taps.

### Citing ghostipy

> Chu, J. P. et al. & Kemere, C. *ghostipy: an efficient signal processing and
> spectral analysis toolbox for large data.* eNeuro (2021).
> Code: https://github.com/kemerelab/ghostipy

(Confirm the exact reference with the ghostipy authors.)
