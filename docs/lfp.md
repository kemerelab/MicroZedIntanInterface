# The LFP band — on-PL decimation to 3 kHz

The board extracts a **local field potential** band on the fabric: the 30 kHz broadband
sample stream is anti-alias filtered and decimated to **3 kHz** and shipped as a second,
independent data product. The broadband stream is never altered — LFP is *derived*, on its
own `stream_type`, so a recording keeps both the raw samples and the cheap low-rate band.

Everything below lives in the PL. `lfp_dsp_block.sv` is the top level: it taps the
acquisition core, runs the filter cascade, and leaves a **complete UDP packet** in the LFP
output BRAM for the PS to DMA out. The PS builds no header and copies no samples.

```
data_generator_core ──► lfp_halfband_dec2 ──► lfp_poly_dec5 ──► packet builder ──► LFP BRAM
    30 kHz tap              11 taps, /2        ≤120 taps, /5       (in lfp_dsp_block)
                              15 kHz              3 kHz                  0x84000000
```

The band the chain promises is **flat to 1.2 kHz**. The first frequency that can fold back
onto that edge after the final /5 is `3000 − 1200 = 1800 Hz`, so 1.2/1.8 kHz is the
transition the sharp filter has to hit.

---

## Why two stages

A filter's transition width is a fraction of **its own** sample rate, so the same sharpness
gets cheaper every time the rate comes down. The 1.2/1.8 kHz transition costs roughly

| built at | taps | delay line |
|----------|-----:|-----------:|
| 30 kHz (one /10 stage) | ~245 | 2× |
| 15 kHz (after a /2)    | ~120 | 1× |

Halving the rate first therefore buys the sharp filter for half the multiplies *and* half
the per-channel delay-line BRAM. The only job of the first stage is to make that halving
safe: it must suppress anything that would fold onto the final passband.

Total decimation is **structural — /2 then /5**. It is not a runtime parameter; the output
is always 3 kHz. Only the stage-2 tap count and the coefficients are configurable.

---

## Stage 1 — `lfp_halfband_dec2.sv`, 11 taps, 30 → 15 kHz

One time-shared MAC (a single DSP48) serves every one of the 256 `(lane × slot)` channels.

The default coefficients are a **halfband**: an equiripple design whose bands are symmetric
about `fs/4` with equal ripple weights already has every second tap zero, and the design
script snaps those to exact zero and pins the centre tap to 0.5. Halfband symmetry forces
the transition to be symmetric about `fs/4`, so choosing the passband edge also chooses the
stopband edge (`f_stop = 15 kHz − f_pass`, i.e. 13 kHz at the shipped 2 kHz edge). The only
free choice is how *wide* to make that transition, and widening it buys attenuation for free
until stage 2 becomes the limiting stage: a 3 kHz edge reaches 63 dB, **2 kHz** reaches
82 dB, and below 2 kHz there is no further gain. Passband droop at 1.2 kHz stays under
0.01 dB across that whole range, so the wide transition costs nothing worth having.

**The engine MACs all 11 taps, including the four structural zeros.** This is deliberate.
Skipping them would save 4 of 11 cycles the budget does not need, and — because the host is
allowed to upload *any* coefficients through the same port — a tap-skipping engine would
silently compute the wrong answer for every non-halfband filter someone loads.

**Throughput.** One output per channel every 2 broadband packets, so a pass has
`2 × (84 MHz / 30 kHz) = 5600` clocks. It costs `256 channels × 11 taps = 2816` — about half
the budget. Lanes the mask disables are skipped whole and spend no MAC cycles.

---

## Stage 2 — `lfp_poly_dec5.sv`, ≤120 taps, 15 → 3 kHz

Up to 120 taps (the shipped filters are 120 = 5 phases × 24, so the polyphase decomposition
is exact), with **4 MACs running in parallel**.

### On "polyphase"

A decimating FIR only has to produce the outputs it keeps, so each output costs `num_taps`
MACs however the sum is organised. An explicit M-branch polyphase decomposition computes
the same products in a different order for the same cost; its advantage is over the naive
*filter everything, throw away 4 of 5*, which this engine never does. So this is the
polyphase-efficient form written directly: on the decimation tick, walk the taps once per
channel.

### Why the MACs are spread across LANES

This is the structural insight of the block. The delay line is **one memory per lane**, and
the read address depends only on `(slot, tap)` — **not** on lane.

So 4 MACs working on 4 *different lanes* issue **one** address, read **four separate**
memories, and all want the **same** coefficient in the same cycle: one address generator,
one coefficient RAM read broadcast four ways, zero memory replication.

Parallelising across taps or slots instead would put 4 different addresses into the **same**
memory in the same cycle, forcing that memory to be replicated 4× — roughly 64 block RAMs
here instead of 16, and 4 coefficient RAMs instead of 1.

The cost is that 4 lanes finish together, so **outputs do not emerge in wire order**. That
is handled downstream (see *Sample placement*), not by buffering and reordering a frame.

**Throughput.** One output per channel every 5 stage-1 frames, so a pass has
`5 × (84 MHz / 15 kHz) = 28000` clocks. At 120 taps it costs 8194 — about 3.4× inside the
budget. That headroom is deliberate: a minimum-phase filter is **not symmetric**, so unlike
a linear-phase design it cannot be folded to half the multiplies, and the engine has to fit
the worst case a host may upload.

---

## Sample placement

Because stage 2 emits group-then-lane rather than in wire order, the packet builder does not
pack samples in arrival order. Each sample carries its true channel index, and is written
straight to the BRAM address that index implies: two 16-bit samples share a 32-bit word, and
the **byte-write enables** select the half. Arrival order stops mattering entirely, and no
reorder buffer or sequential packing state is needed.

A channel's position in the payload depends on how many *enabled* lanes precede it, not on
its raw channel number, so the payload stays tightly packed for any lane mask.

The frame is published to the PS (`lfp_wr_addr` advances) only after its last sample has
landed, so the PS can never DMA a half-written frame.

---

## Fixed point

```
16-bit in  ──►  18-bit intermediate  ──►  16-bit out
   (wire)      (2 extra fractional bits)      (wire)
```

Coefficients are **Q1.17** signed, 18-bit. Stage 1's accumulator carries the coefficients'
17 fractional bits; on the way out it drops 15 of them, keeping **2 fractional bits below
the wire LSB**. Stage 2 sheds the rest on its own way out to the 16-bit wire format. Both
stages saturate rather than wrap at the output.

Two reasons the intermediate is 18 bits rather than 16:

- **It is free.** A stage-2 lane delay line is 32 slots × 128 ring = 4096 deep. At 4K deep,
  both a 16-bit and an 18-bit memory occupy two block RAMs per lane — the extra two bits
  cost nothing in the BRAM shape.
- **It matches the DSP48 B port** (18-bit signed), so nothing is truncated on the way into
  the multiplier.

And one reason it is not 16: rounding straight back to the wire width between the stages
would inject quantisation noise **ahead of** the narrowband filter that then has to live
with it, right where the passband is narrowest.

Everything in both MAC pipelines is explicitly signed. A single unsigned operand would make
the whole expression unsigned and turn negative partial sums into large positives.

---

## The two stage-2 filters

Two 120-tap designs ship with identical magnitude responses. Both are loaded from
`programmable_logic/sim/`, and either can be selected at runtime by uploading its
coefficients — no rebuild. The board powers up with **linear phase**.

| | passband ripple (0–1.2 kHz) | worst-case alias | cascade latency | group delay across the passband |
|---|---:|---:|---:|---|
| **linear phase** | 0.007 dB | 82.4 dB | 4.13 ms | 4.13 ms flat, by construction |
| **minimum phase** | 0.001 dB | 80.0 dB | 0.90 ms | 0.75–1.41 ms (0.656 ms spread) |

**Worst-case alias** is the number that matters. It is not a per-stage stopband figure: it
is the largest gain *any* out-of-band input frequency has after passing through **both**
filters and folding through **both** decimations into the 0–1.2 kHz output band. Per-stage
stopband numbers flatter you; this one does not.

**Which to pick.**

- **Linear phase** — the default. Group delay is exactly constant across the band, so every
  frequency component is delayed equally and **waveform shape is preserved**. Choose it for
  recording, for anything where sharp-wave/ripple morphology or cross-frequency phase
  relationships matter, and whenever 4 ms of delay is not in a control loop.
- **Minimum phase** — same magnitude response, **4.6× less latency**, built by spectral
  factorisation (design a linear-phase prototype of twice the length with double the
  stopband dB, then take its minimum-phase factor). It front-loads its impulse response,
  which is exactly where the latency saving comes from. The price is that group delay varies
  by 0.66 ms across the passband: low and high components of the same event arrive at
  different times, so **waveform shape disperses**. Choose it when closed-loop latency
  matters more than shape fidelity — and not for phase-targeted work, where the dispersion
  is precisely the quantity being measured.

---

## Coefficient upload

Both stages share one indirect write window, with a stage-select bit:

| `lfp_strobe` | effect |
|---|---|
| `[0]` toggle | write one coefficient at the auto-incrementing pointer |
| `[1]` clear | reset the write pointer — do this before each upload |
| `[2]` stage | `0` = stage 1 (halfband), `1` = stage 2 (decimator) |

The stage bit must be **held across the whole upload, including the pointer clear**, so that
the reset and every subsequent write land in the same filter. Load while the engine is
disabled, so a live pass never mixes old and new taps.

The coefficient RAMs power up with the designed defaults compiled in
(`lfp_coef_pkg.sv`), so the board filters correctly with no host upload at all.

---

## Timestamps

Header words 2/3 carry the master sample count of the **newest broadband sample in this
output's decimation window**. These are FIR filters, so the newest input in an output's
support is a real, already-acquired sample: for frame *m* at total decimation R that is
broadband packet `R·m + (R−1)` — with R = 10, `10m+9` — the same count the broadband header
stamps for that packet. It is causal, monotonic, and exactly 10 apart per frame.

It marks the newest **input**, not the instant the filtered value represents. A host that
wants the latter subtracts the filter's group delay (the table above). A host that wants
"which input data has been folded in so far" uses the stamp directly.

---

## Overrun

If a new pass falls due while the previous one is still running, either stage latches a
sticky `compute_overrun` and the late frame is dropped **whole** — never emitted
half-summed. The flag rides out in header AUX0 and in status register 13, so a host can tell
a dropped frame from a corrupted one. With both stages at ~3× headroom this should
never fire; if it does, the configuration has outgrown the engine.

---

## Control surface

The LFP occupies control registers 25–27 and status register 13; `docs/register-map.md`
covers the rest of the AXI-Lite map, and `docs/protocol.md` the command frame format.

| reg | field |
|---|---|
| `CTRL_REG_LFP_CFG` (25) | `[0]` enable · `[15:8]` reserved · `[23:16]` decim_R (reads 10) · `[31:24]` stage-2 num_taps |
| `CTRL_REG_LFP_COEF` (26) | `[17:0]` signed Q1.17 coefficient |
| `CTRL_REG_LFP_STROBE` (27) | `[0]` write toggle · `[1]` pointer clear · `[2]` stage select |
| `STATUS_REG_13` | `[15:0]` LFP BRAM write byte-address · `[16]` overrun |

Two fields on reg 25 are **reported, not commanded**. `decim_R` always reads 10 because the
cascade is wired that way, and `[15:8]` is a reserved lane-mask field the engine ignores:

> **The LFP lane mask mirrors the broadband `channel_enable` mask.** It is driven straight
> from `data_generator_core`, so the LFP filters exactly the broadband-enabled lanes. There
> is one place to pick streams — `set_channels` — and `CMD_LFP_SET_CHANNELS` (`0x82`) is
> accepted and ignored rather than being given a mask of its own.

Commands `0x80`–`0x83` are `LFP_ENABLE`, `LFP_SET_PARAMS` (tap count; the decimation
parameter is ignored), `LFP_SET_CHANNELS` (no-op, above), and `LFP_WRITE_COEF`. `get_status`
reports the enable, lane mask, decimation, tap count, packets sent and overrun flag.

---

## On the wire

LFP frames go out on the **same UDP port as broadband** and are demuxed host-side by
`stream_type = 2`. There is no second data port. A frame is the 8-word common header —
defined once in `programmable_logic/src/unified_pkt_pkg.sv` — followed by the samples, with
one frame per datagram:

- `AUX0` (w5) = `lane_mask[7:0]` · `decim_R[15:8]` · `num_taps[23:16]` · `overrun[24]`
- `AUX1` (w6) = `num_samples` = `popcount(lane_mask) × 32`
- payload = 16-bit **offset-binary** samples, two per 32-bit word

Samples ship offset-binary, the same format as broadband, so a host de-offsets both
identically (subtract `0x8000`). The engine itself is pure two's-complement; the conversion
is a symmetric MSB invert at each boundary.

`docs/protocol.md` and `docs/unified-packet-format.md` are the wire-format references.

---

## Regenerating the filters

```
python3 programmable_logic/sim/design_lfp_filters.py [--plot]
```

This is the single source of the coefficients. It designs both stages, quantises to Q1.17,
reports the passband ripple, the end-to-end worst-case alias rejection and the latency each
design actually achieves, and writes:

| output | role |
|---|---|
| `lfp_hb11_coefs.hex` | stage 1 |
| `lfp_poly120_lin_coefs.hex` | stage 2, linear phase |
| `lfp_poly120_min_coefs.hex` | stage 2, minimum phase |
| `../src/lfp_coef_pkg.sv` | the power-on defaults the RTL compiles in |

Because the RTL defaults and the host's upload files come from the same run, the board and
the host cannot drift apart. `lfp_coef_pkg.sv` is generated — do not hand-edit it.

`--plot` additionally writes the magnitude response, and the impulse/phase/group-delay
comparison that shows the minimum-phase trade directly.

---

## File map

| file | role |
|---|---|
| `programmable_logic/src/lfp_dsp_block.sv` | top level: tap conditioning, cascade, coefficient routing, packet builder |
| `programmable_logic/src/lfp_halfband_dec2.sv` | stage 1: 11-tap halfband, /2 |
| `programmable_logic/src/lfp_poly_dec5.sv` | stage 2: ≤120-tap decimator, /5, lane-parallel MACs |
| `programmable_logic/src/lfp_coef_pkg.sv` | generated power-on coefficients |
| `programmable_logic/src/unified_pkt_pkg.sv` | the common packet header contract |
| `programmable_logic/sim/design_lfp_filters.py` | filter design + measured numbers |
| `programmable_logic/sim/lfp_halfband_dec2_tb.sv`, `lfp_poly_dec5_tb.sv` | per-stage bit-exact testbenches |
| `firmware/src-core0/pl_control.c` | `pl_lfp_set_config`, coefficient upload |
| `firmware/src-core0/stream.c` | drains the LFP BRAM to UDP |
| `remote/net.py` | `configure_lfp`, `lfp_enable`, `receive_lfp` |
