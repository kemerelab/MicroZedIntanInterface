# Wavelet bank coefficients — `claude/tier3-wavelet-v2` (K=176, 4-MAC engine)

This branch streams a complex wavelet scalogram (UDP 5004) computed on the
3 kHz CIC LFP. The voices are **true generalized-Morse wavelets** — exactly
[ghostipy](https://github.com/kemerelab/ghostipy)'s `MorseWavelet(gamma=3,
beta=3)` — so out-of-band / offline analysis with ghostipy lines up with the
hardware. `docs/wavelet_coeffs.py` regenerates **the exact Q1.17 taps this
branch uploads to the PL**.

> **v2 engine.** This branch uses the **4-MAC work-spread** scalogram engine
> (two voices/cycle, lazy per-octave work-spread) — real-time-clean to **K=176**
> channels. The coefficient bank below is **identical** to the base
> `claude/tier3-wavelet` branch (same `V`/`N_TAPS`/`N_OCTAVES`/`fc_top`); only
> the engine's channel throughput differs, so the scalogram values are unchanged.

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
24-tap FIR's frequency response matches ghostipy's wavelet in shape (same peak
frequency; passband magnitude error **< 2.2e-4**; correlation 1.00000).

**`gamma`/`beta`.** `gamma=3` is the locked default (γ=3 Morse is exactly analytic
even at short durations). `beta` is the **Q / bandwidth knob**: ghostipy's default
is `beta=20` (narrowband, many oscillations); this bank uses **`beta=3`** — broad,
so a true Morse atom fits `N_TAPS=24` taps without truncation artifacts. Raising
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
| `V` | 4 | voices (sub-bands) per octave |
| `N_TAPS` | 24 | complex FIR taps per voice |
| `N_OCTAVES` | 8 | octaves in the cascade |
| `HB_TAPS` | 7 | halfband /2 anti-alias taps (unchanged) |
| `K` | 176 | channels (lanes), 4-MAC work-spread engine (real-time-clean) |
| `fs` | 3000 Hz | LFP rate into octave 0 |
| `fc_top` | 0.34 | top-voice center (cycles/sample within an octave) |
| `gamma, beta` | 3, 3 | Morse shape params (β = Q knob) |
| coeff format | Q1.17 | signed 18-bit, value = `int / 2**17`, saturating |

These mirror `firmware/include/main.h` (`WAV_*`) and `remote/net.py`
(`design_wavelet_bank` defaults) for this branch. The coefficient values are
identical to the base `claude/tier3-wavelet` branch (`V`/`N_TAPS`/`N_OCTAVES`
match); only `K` (the 4-MAC work-spread engine's channel count) differs.

### Analysis center frequencies (Hz) — `centers[octave][voice]`

`fc_top = 0.34` leans the grid **high** (top voice = 1020 Hz, into the
fast-ripple/HFO range), not the nominal ≤512 Hz table. Lower `fc_top` to retune.

| octave | rate (Hz) | v0 | v1 | v2 | v3 |
|---|---|---|---|---|---|
| 0 | 3000.0 | 1020.00 | 857.71 | 721.25 | 606.50 |
| 1 | 1500.0 | 510.00 | 428.86 | 360.62 | 303.25 |
| 2 | 750.0 | 255.00 | 214.43 | 180.31 | 151.62 |
| 3 | 375.0 | 127.50 | 107.21 | 90.16 | 75.81 |
| 4 | 187.5 | 63.75 | 53.61 | 45.08 | 37.91 |
| 5 | 93.75 | 31.88 | 26.80 | 22.54 | 18.95 |
| 6 | 46.88 | 15.94 | 13.40 | 11.27 | 9.48 |
| 7 | 23.44 | 7.97 | 6.70 | 5.63 | 4.74 |

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
