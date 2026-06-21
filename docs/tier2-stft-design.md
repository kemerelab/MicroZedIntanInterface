# Tier-2 — on-PL STFT spectral-estimation engine

Status: **build in progress** (branch `claude/tier2-stft`). This is module 2 of the on-PL
processing engine: a sliding-window STFT that turns the Tier-1 LFP stream into per-channel
spectra for band power, spectral shape, coherence/phase-synchronization, and closed-loop
detection. Tier-1 (LFP decimation) ships on `main`; this is the exploratory Tier-2 layer.

## Why STFT (vs. filter banks)

We measured it (`remote/stft_vs_filter_latency.py`): STFT and FIR/IIR filters ride the
**same time-bandwidth curve** — `filter ΣgroupDelay ≈ STFT W/2` — so there's *no latency
penalty* for the STFT. You choose it for **capability**: one transform per channel gives
all bands, spectral shape (artifact rejection), and — the standout — **inter-channel
coherence / phase-locking** from cross-spectra `X_i·X_j*`, which filter banks can't do
cleanly. Compute is cheap (one shared FFT IP for K channels) and bandwidth fits (below).

## Defaults (all host-reconfigurable, no rebuild)

| knob | default | how |
|------|---------|-----|
| **N** (transform length) | **64** (32 ms @ 2 kHz, ~31 Hz bins) | xfft runtime-configurable length |
| **H** (hop) | **1** (per-sample, max overlap) | register; larger H for larger N to bound load |
| **K** (channels) | **32** of 256 | channel-selector table |
| output | **float32** complex | xfft IEEE-754 |
| window | Hann | host-loaded coefficient RAM |
| bands / pairs | host/software | derived from the streamed spectra |

## Architecture (inline PL, software decides)

```
Tier-1 LFP out (256ch @ 2kHz)
   → [channel selector: K of 256]
   → [per-channel sliding buffer, K×maxN ring]   ← push one sample/channel per LFP frame
   → every H samples, per channel:  × Hann (fixed) → fixed→float32
   → [xfft IP, IEEE-754, runtime-N]              (time-shared across K channels)
   → capture float32 complex bins → [results BRAM]
   → core 0: jumbo UDP stream (monitoring)   |   core 1: band power / coherence / threshold / trigger
```

The heavy, regular FFT runs **inline in the PL** (a streaming stage, no CPU round-trip,
like the LFP FIR). The **flexible measures** — band selection, ratios, coherence/PLV,
decisions — run in **software** on the spectral frames (small data). Cross-spectral
coupling math is pure software, so new measures never touch the bitstream.

## Latency

`detection latency ≈ W/2` for the *peak* estimate, **but** with H=1 you threshold the
*rising* band power and fire before the window fills — so detection latency `< W/2` for a
suprathreshold event, traded against frequency specificity / SNR (fewer cycles in the
window = can't yet differentiate 175 vs 100 Hz). The same rising-edge trick applies to
filters. A causal Hann weights *recent* samples low, so for lowest-latency single-band
detection a recent-weighted window (→ a recursive filter) wins; the STFT's edge is
multi-band economy at one window.

## Bandwidth & packetization (jumbo)

Full Hermitian spectrum, **float32**, N=64: `64 real values/ch` (DC+Nyquist real) →
`32 ch × 64 × 4 B = 8 KB/frame`. At one spectrum per 2 kHz LFP sample = **2k pps**, ~16 MB/s.
That's **within** the box's demonstrated ~18 MB/s (and the gigabit line / true large-packet
ceiling is far higher — being measured by the `udp_bench` tool on main). 8 KB/frame fits one
**jumbo** frame → 2k pps, one packet per spectrum (the chosen approach). On non-jumbo paths
you'd MTU-split (~6 packets/frame); jumbo keeps it one-packet-clean.

## Spectrum UDP packet (port 5003, proposed)

| words | field |
|------:|-------|
| 0–1 | magic `0xCAFEBABE_5DEC7A00` *(proposal)* |
| 2–3 | 64-bit timestamp (≈ broadband packet index) |
| 4 | `[7:0]` N_log2 · `[15:8]` K · `[23:16]` hop · `[31:24]` flags |
| 5 | frame sequence number |
| 6 | channel-select map ref / chunk index |
| 7… | payload: per channel, `N/2+1` complex **float32** bins (Hermitian half) |

## Phase caveat

STFT bin phase is window-averaged → laggy/coarse for *instantaneous* phase-locked stim. For
that, add a quadrature/Hilbert filter path (lower latency). For windowed coherence/PLV the
STFT phase is fine.

## Resources (xfft float32 — being measured)

Fixed-point xfft was ~9 DSP / ~1.6k LUT / ~0 BRAM (N=128). Float32 is heavier (~2–3×) but
still small vs. the 99% DSP / 53% BRAM free after Tier-1. One shared IP serves all K
channels (load = `K·N·Fs/H`; N=64/H=1/32ch @ 2 kHz ≈ 4 M samp/s ≈ 4% of a 100 MHz FFT).

## Build plan

xfft IP (float32, runtime-N) → `stft_engine.sv` (selector + sliding buffer + window +
fixed→float + xfft + capture) → sim vs Python FFT reference → BD integrate (+ results BRAM,
control regs) → full PL build → firmware jumbo streaming + net.py receiver. Quadrature-phase
path and software coherence/PLV are follow-ons.
