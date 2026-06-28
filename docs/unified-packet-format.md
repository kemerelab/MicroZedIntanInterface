# Unified single-port packet format (no-loss, no-MTU-framer)

Status: **design contract** for the `claude/unified-ports` (broadband + LFP) and the
follow-on `claude/unified-wavelet` branches, and the two matching `ephys-socket` branches.
This is the single source of truth — PL, firmware, `net.py`, and the Open Ephys plugin all
implement exactly this.

## Principles (from the CLAUDE.md hard rule)

1. **One UDP port** (default **5000**) for ALL PL→host streams. The host demuxes by
   `stream_type` in the header. (Board-side this is TX-neutral; host-side we drain
   promiscuously so broadband is never blocked.)
2. **NO DATA LOSS.** Every packet fits **one standard datagram** (≤ 1472 B payload → no IP
   fragmentation, no jumbo, **no MTU framer**). We *specify* the data so it inherently fits;
   if a config wouldn't fit, we **reduce the spec**, never chunk-with-loss.
3. **Loss is provably zero**, not assumed: every packet carries a **per-stream monotonic
   sequence number**; the host flags any gap. Broadband's gap count must stay 0.

## Common header — 8 × 32-bit little-endian words (32 bytes), identical for every stream

| word | name | contents |
|------|------|----------|
| 0 | `MAGIC` | `0xCAFEBABE` (all PL packets) |
| 1 | `TYPE_VER` | `[7:0]` stream_type · `[15:8]` version (=1) · `[31:16]` flags |
| 2 | `TS_LO` | 64-bit master timestamp, low word |
| 3 | `TS_HI` | 64-bit master timestamp, high word |
| 4 | `SEQ` | per-stream packet sequence, +1 each packet of that stream (wraps 32-bit) |
| 5 | `AUX0` | stream-specific (below) |
| 6 | `AUX1` | stream-specific (below) |
| 7 | `RSVD` | 0 (reserved; candidate for a future CRC32 of the packet) |

`stream_type`: **1 = BROADBAND, 2 = LFP, 3 = WAVELET.**

The host demuxes on `TYPE_VER[7:0]`. **Per-stream `SEQ` continuity = the loss check.** Keep
each stream's `SEQ` independent so broadband's integrity is unaffected by the others.

## Per-stream payloads

### BROADBAND (type 1) — unchanged content, re-framed
- `AUX0` = `channel_enable[7:0]` · `num_data_words[23:8]`
- `AUX1` = digital-in / metadata (preserve today's fields)
- Payload = the existing per-packet fields that don't fit the header (the 8 external-ADC
  values, any remaining metadata) **followed by** the data words. **Map ALL of today's
  10-word-header content into the new header + a small broadband sub-block — lose nothing.**
- Already ≤ 1 datagram (≤140 data words + header ≈ 600 B). Fits trivially.

### LFP (type 2)
- `AUX0` = `lane_mask[7:0]` · `decim_R[15:8]` · `num_taps[23:16]` · `overrun[24]`
- `AUX1` = `num_samples`
- Payload = the decimated samples (int16, as today). One frame ≤ 1 datagram. Fits.

### WAVELET (type 3) — one octave per packet, rate-aligned
- **One packet = one octave.** `AUX0` = `octave[3:0]` · `n_octaves[7:4]` · `n_voices[11:8]`
  · `overrun[24]`. `AUX1` = `n_channels[7:0]` · `lane_start[23:8]`.
- Payload = `n_channels × n_voices` complex coefficients (re,im as int32 each) for **this
  octave only**, lane-major then voice-minor:
  `[(ch0,v0.re),(ch0,v0.im),(ch0,v1.re),(ch0,v1.im)...,(ch1,v0.re)...]`.
- **Rate-aligned emission:** emit octave *o*'s packet **only on the frames where it updates**
  (the à trous engine advances octave *o* when `fcount mod 2^o == 0` — the work-spread
  scheduler already computes this flag). So octave 0 streams at 3 kHz, octave 7 at ~23 Hz —
  **no redundant re-sending of slow bands** (≈4× less traffic than sending all octaves every
  frame). The host holds each octave's last value between its updates (the truthful
  representation of a multirate scalogram).
- **Fits one datagram by construction (no MTU framer):** `32 + n_channels·n_voices·8 ≤ 1472`
  ⇒ `n_channels·n_voices ≤ 180`. The monitored channel count is a **spec we bound** (e.g.
  ≤ 40 channels at V=4) so a one-octave packet always fits. Want more channels than fit?
  **Reduce the channel count** (the no-loss rule) — do not split/fragment. (Full 256-ch
  resolution is the future DDR/soft-core path, not this UDP monitor.)

## Host (net.py + Open Ephys)

- **One socket, port 5000, promiscuous drain:** a tight `recvfrom → ring` loop that never
  blocks on processing; demux + per-stream handling happen downstream. Big `SO_RCVBUF`.
- Demux by `TYPE_VER[7:0]`; verify per-stream `SEQ` continuity (the loss check).
- Wavelet: place each packet's `(octave, lane_start..+n_channels)` block into the surface;
  hold each octave between its rate-aligned updates.

## Branch plan

1. `claude/unified-ports` (off `main`): broadband + LFP on port 5000 with this header. PL
   (both packet builders) + firmware (single send path/port) + `net.py` (one socket, demux)
   + sim. **No wavelet.**
2. `claude/unified-wavelet` (off `unified-ports`): port the v2 wavelet engine on top, add the
   octave-split rate-aligned WAVELET packets.
3. `ephys-socket` branch 1: consume the unified port (broadband + LFP demux).
4. `ephys-socket` branch 2 (off branch 1): add the WAVELET scalogram consumer.
