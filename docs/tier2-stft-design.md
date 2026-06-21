# Tier-2 — on-PL STFT spectral-estimation engine

Status: **implemented on `claude/tier2-stft`** — engine sim-verified (264/264 vs a Python
float-DFT reference), build path synthesizes clean (xfft float32 + fix2float, ~17-24 DSP),
integrated into the BD (results BRAM @ 0x88000000, control regs 28-30, status reg 14), and
the PS + net.py jumbo-streaming prototype is in place. On-hardware validation pending.

This is module 2 of the on-PL processing engine: a sliding-window STFT that turns the Tier-1
LFP stream into per-channel spectra for band power, spectral shape, coherence/phase-
synchronization, and closed-loop detection. Tier-1 (LFP decimation) ships on `main`.

### As-built contract (keep the 3 layers in sync)

| layer | what |
|-------|------|
| PL | `stft_engine.sv` (selector+ring+window+capture) + `stft_fft.v` (xfft float32 + fix2float) in `stft_dsp_block.sv`; ctrl regs 28 (cfg: `[0]en [7:4]nfft_log2 [31:16]hop`), 29 (data), 30 (strobe: `[0]toggle [1]ptr_clr [2]target 0=win/1=sel`); status reg 14 (`[29:0]frame_seq [30]busy [31]overflow`); results BRAM @ `0x88000000` |
| PS | `CTRL_REG_STFT_*`, `STFT_BRAM_BASE 0x88000000`, `STFT_UDP_PORT 5003`, `STFT_K=32`, `STFT_MAX_N=64`; `pl_stft_*` upload; `stft_stream_service` (poll frame_seq → one jumbo packet) |
| host | `CMD_STFT_*` (0x84-0x87), `configure_stft` (Hann Q15 + selector upload), `receive_stft` (jumbo float32 decode), `get_status` (176 B) |

Window N is runtime-selectable up to **64** this build (xfft `transform_length=64`); raise the IP
param + keep `RES_AW`=16 for up to 256. The engine ring is mod-MAX_N(256) so streaming frames
never overwrite the window mid-pass.

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

## Spectrum UDP packet (port 5003, as built)

| words | field |
|------:|-------|
| 0–1 | magic `0xCAFEBABE_5DEC7A00` (low word 0 = `0x5DEC7A00`, high word 1 = `0xCAFEBABE`) |
| 2–3 | 64-bit timestamp (`frame_seq × hop` ≈ LFP-frame index) |
| 4 | `[7:0]` N_log2 · `[15:8]` K · `[31:24]` flags (bit0 = overflow) |
| 5 | frame sequence number (`frame_seq`, 30-bit) |
| 6 | `[15:0]` nbins (N/2+1) · `[31:16]` hop |
| 7 | reserved |
| 8… | payload: per lane (lane-major), `N/2+1` complex **float32** bins `(re,im)` |

One jumbo frame/spectrum (N=64/K=32 → 8 hdr + 2112 payload words = 8480 B). The firmware
re-reads `frame_seq` after the copy and drops any frame a new pass tore (cheap integrity guard).

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
