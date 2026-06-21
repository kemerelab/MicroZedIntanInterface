# Tier-2 STFT engine — overnight build summary

Branch **`claude/tier2-stft`**. Goal (your ask): move Tier-2/STFT off `main`, then go
ahead with the **jumbo-packet, full-hardware float32** approach and implement the **PS +
net.py prototype**. Done end-to-end and built; on-hardware validation is the remaining step.

## What got built (and how it's verified)

| piece | state | evidence |
|------|-------|----------|
| `stft_engine.sv` — selector + ring + Hann window + feed/capture FSMs | **sim-verified** | `sim/stft_engine_tb.sv` vs Python float-DFT (`sim/gen_stft_vectors.py`): **264/264, 0 errors** |
| Frame protocol (frame-start + deep mod-256 ring) | **sim-verified** | matches real `lfp_fir_decimator.out_frame_start`; no window overwrite under streaming |
| `stft_fft.v` — xfft float32 + fix2float (int32→float32) | **synth-verified** | OOC `synth_design`: 0 errors, ~17–24 DSP / ~5 BRAM |
| BD integration — results BRAM @ `0x88000000`, ctrl regs 28–30, status 14 | **built** | wrapper elaborates clean; full PL build below |
| **Full PL build** | **DONE, timing CLOSED** | **WNS +0.285 ns, WHS +0.041 ns**, bitstream written |
| Firmware v1.3 (PS prototype) | _see Firmware build below_ | mirrors LFP; `_Static_assert` guards 176-byte wire |
| net.py (host prototype) | **syntax-checked** | `configure_stft`, `receive_stft`, `get_status` |

## The 3-layer contract (all in sync)

- **Ctrl regs**: 28 = cfg `[0]en [7:4]nfft_log2 [31:16]hop`; 29 = data (window Q15 / sel channel);
  30 = strobe `[0]toggle [1]ptr_clr [2]target(0=win,1=sel)`. **Status reg 14** = `[29:0]frame_seq [30]busy [31]overflow`.
- **STFT results BRAM** @ `0x88000000` (64 KB). **UDP port 5003**, magic `0xCAFEBABE_5DEC7A00`.
- **Packet**: 8-word header (magic, 64-bit ts, `nfft_log2|K<<8|flags<<24`, seq, `nbins|hop<<16`,
  resv) + per-lane `(N/2+1)` complex float32 `(re,im)`. N=64/K=32 → one ~8.5 KB jumbo frame/spectrum.
- **Commands** `CMD_STFT_*` 0x84–0x87 (enable / set-params / set-channels / write-window).

## How to test on hardware

```python
# remote/net.py (after flashing the new BOOT.bin)
configure_stft(sock, channels=list(range(32)), nfft_log2=6, hop=1)  # N=64, Hann, 32 ch
stft_enable(sock, True)
receive_stft(100)        # jumbo float32 spectra on UDP 5003 (host needs jumbo MTU)
get_status(sock)         # shows STFT enable/N/hop/passes/overflow
```
LFP (Tier-1) must be enabled/streaming first — the STFT taps the decimated LFP output stream.

## Pending / risks to validate on hardware

1. **On-HW correctness** — never run on the board; sim + synth only here.
2. **xfft config word** — `stft_fft.v` builds the 16-bit config as `{NFFT@[12:8], FWD@[0]}`. If a
   frame mis-sizes, confirm the field layout against PG109 for this exact IP config.
3. **Feed backpressure** — the feed FSM gates on `fft_in_tready` and assumes the xfft holds tready
   for a whole frame (true for the behavioral model). If the real xfft backpressures mid-frame, add
   a 2-deep skid buffer (noted in `stft_engine.sv`).
4. **Window length** — this build caps N at 64 (`xfft transform_length=64`). For N up to 256, raise
   that IP param (`scripts/create_stft_ip.tcl`); `RES_AW=16` already fits the larger spectrum.
5. **hop=1 read race** — passes (~61 µs) ≪ LFP-frame period (~500 µs) and the PS read is guarded by
   a re-check of `frame_seq`, so torn frames are dropped rather than sent.

## Commits (this branch)
engine sim-verify → frame-start model → build path (stft_fft + IP gen) → PL integration →
firmware+net.py → docs. See `git log claude/tier2-stft`.
