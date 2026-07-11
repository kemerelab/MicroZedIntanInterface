# Command & packet structure

The board exposes two network endpoints once its Ethernet link is up:

| Endpoint | Port | Direction | Purpose |
|----------|------|-----------|---------|
| **TCP control** | 0x6900 | host → board (with acks) | commands (start/stop, config, registers) |
| **UDP data** | 0x6800 | board → host | the acquisition stream |

Default board IP is `192.168.18.10` (put your host on the same `/24`).

`remote/net.py` and `firmware/include/main.h` are the **authoritative** sources for the
exact IDs, bit layouts, and offsets — this document is the readable summary. The firmware,
`net.py`, and the `ephys-socket` plugin are the **three consumers of the same contract**;
changing it means changing all three.

## TCP control commands

Every command is a fixed **20-byte** little-endian frame:

```
uint32  magic    = 0xDEADBEEF      (CMD_MAGIC; note this is the COMMAND magic, not the packet magic)
uint32  cmd_id
uint32  ack_id                     (echoed back in the ack so the host can match responses)
uint32  param1
uint32  param2
```

Most commands reply with a small ack; `GET_STATUS`, `READ_REGISTER`, etc. reply with data.

| cmd_id | Name | params | Notes |
|-------:|------|--------|-------|
| `0x01` | START | — | begin streaming |
| `0x02` | STOP | — | stop streaming |
| `0x03` | RESET_TIMESTAMP | — | zero the packet timestamp |
| `0x10` | SET_LOOP_COUNT | p1 = count | 0 = free-run; N = stream N packets then stop |
| `0x11` | SET_PHASE | p1 = phase0, p2 = phase1 | **port A** CIPO0/CIPO1 sampling phase (cable length) |
| `0x14` | SET_PHASE_B | p1 = phase2, p2 = phase3 | **port B** (2nd cable) CIPO0/CIPO1 phase |
| `0x12` | SET_DEBUG_MODE | p1 = 0/1 | synthetic sine instead of real CIPO (no chip needed) |
| `0x13` | SET_CHANNEL_ENABLE | p1 = mask | 8-bit stream mask (e.g. `0xFF` = all 8 streams / 256 ch) |
| `0x20` | LOAD_CONVERT | — | load the per-sample CONVERT command sequence |
| `0x21` | LOAD_INIT | — | load the RHD2000 init/config sequence |
| `0x22` | LOAD_CABLE_TEST | — | load the cable/phase test sequence |
| `0x30` | FULL_CABLE_TEST | — | run the full phase sweep |
| `0x40` | GET_STATUS | — | returns the `status_response_t` struct (see below) |
| `0x41` | DUMP_BRAM | p1 = word, p2 = count | debug: dump BRAM words to the serial console |
| `0x50` | SET_UDP_DEST | p1 = ip, p2 = port | set the UDP stream destination |
| `0x60` | PING | — | liveness check |
| `0x70` | AUX_WRITE_WORD | p1 = slot \| bank<<8 \| is_len<<16; p2 = addr<<16 \| data | write an aux command-bank word |
| `0x71` | AUX_BANK_SELECT | p1 = slot; p2 = bank | select the active aux bank for a slot |
| `0x72` | AUX_SEQ_EN | p1 = 0/1 | enable the aux command sequencer |
| `0x73` | READ_REGISTER | p1 = reg | → 4-byte `{cipo1, cipo0}` response |
| `0x74` | WRITE_REGISTER | p1 = reg, p2 = value | → 4-byte echo |
| `0x75` | SET_FAST_SETTLE | p1 = amp cfg, p2 = dsp cfg | `sw \| gpio_en<<1 \| pin<<4` per field |
| `0x76` | SET_DIGOUT | p1 = sw \| gpio_en<<1 \| pin<<4; p2 = reg3 static byte | digital-out control |
| `0x80` | LFP_ENABLE | p1 = 0/1 | enable the LFP/DSP engine + its UDP stream |
| `0x81` | LFP_SET_PARAMS | p1 = decim_R, p2 = num_taps | decimation factor + active FIR length |
| `0x82` | LFP_SET_CHANNELS | *(deprecated)* | **DEPRECATED** — the LFP lane mask now mirrors the broadband `channel_enable` mask (use `SET_CHANNEL_ENABLE` `0x13`); firmware accepts-and-ignores |
| `0x83` | LFP_WRITE_COEF | p1 = [0] clear-ptr-first, p2 = 18-bit signed coef | upload one Q1.17 tap |

Configure LFP while disabled (params → coefficients via repeated `0x83`, the first with the
clear bit), then `LFP_ENABLE`. The **lane mask = the broadband `channel_enable` mask** (single
source of truth) — pick which streams to LFP-filter with `SET_CHANNEL_ENABLE` (`0x13`); the
LFP filters exactly the broadband-enabled lanes. (See `remote/net.py` `configure_lfp()`
+ `design_cic_comp_fir()` / `design_lfp_lowpass()`.)

(See `remote/net.py` for the exact per-command parameter packing and the interactive
command names like `start`, `set_channels`, `auto_cable_detect`, `verify_sine`.)

## UDP data packet (port 0x6800)

Each UDP datagram is **one 30 kHz sample frame**: `10` header words + `N` data words, all
32-bit little-endian. The 64-bit timestamp (header words 2–3) increments once per datagram.

**Header (10 words / 40 bytes):**

| words | field | notes |
|------:|-------|-------|
| 0–1 | magic `0xCAFEBABE_DEADBEEF` | word0 = `0xDEADBEEF`, word1 = `0xCAFEBABE` |
| 2–3 | 64-bit timestamp | increments once per packet (per sample frame) |
| 4–5 | digital-in (8 bits) + metadata | |
| 6–9 | 8 × 16-bit external ADC values | |

### Which channels get sampled — the channel-enable mask

`SET_CHANNEL_ENABLE` (`0x13`, param1) is an **8-bit mask selecting which of 8 streams** are
sent. Each set bit adds one stream of 32 amplifier channels; `0xFF` = all 8 = **256 channels**.
The bit order is **also the order the streams appear in every packet**:

| bit | stream | cable/port | CIPO line | phase |
|----:|--------|-----------|-----------|-------|
| 0 | `A_CIPO0_REG` | A (cable 1) | CIPO0 | regular |
| 1 | `A_CIPO0_DDR` | A | CIPO0 | DDR (2nd interleaved chip) |
| 2 | `A_CIPO1_REG` | A | CIPO1 | regular |
| 3 | `A_CIPO1_DDR` | A | CIPO1 | DDR |
| 4 | `B_CIPO0_REG` | B (cable 2) | CIPO0 | regular |
| 5 | `B_CIPO0_DDR` | B | CIPO0 | DDR |
| 6 | `B_CIPO1_REG` | B | CIPO1 | regular |
| 7 | `B_CIPO1_DDR` | B | CIPO1 | DDR |

Each stream = one RHD2000 chip's **32 amplifier channels** (the DDR streams are the second
chip interleaved on the same CIPO line via double-data-rate). 8 × 32 = 256.

### Data words — layout (this is what a receiver/plugin must de-interleave)

Each enabled stream contributes **35 16-bit samples per frame** (the 35 COPI commands of the
acquisition loop), packed **two samples per 32-bit word, low half first**:

```
data_word[i] = (sample[2*i+1] << 16) | sample[2*i]
```

Total = `35 × popcount(mask)` 16-bit samples = `ceil(that / 2)` 32-bit words. So the packet is
**28 words (min, 1 stream) … 150 words / 600 bytes (max, `0xFF`, 8 streams)**.

The samples are **cycle-major** — the outer loop is the 35 cycles, the inner loop is the
enabled streams in **ascending bit order**:

```
flat_index = 0
for cycle in 0..34:                      # 35 COPI commands
    for stream in enabled_streams:       # ascending mask bit
        sample[cycle][stream] = data_sample[flat_index];  flat_index += 1
```
So `sample(cycle, stream)` is at flat index `cycle * num_enabled + (rank of stream among enabled)`.

**Mapping cycle → amplifier channel.** Because of the 2-command SPI readback pipeline, a
`CONVERT(ch)` result lands two cycles later, so **amplifier channel `ch` (0–31) is at
`cycle = ch + 2`**. Cycles 0, 1, and 34 carry pipeline/aux readback (register reads, etc.),
**not** amplifier data — a displaying plugin should take cycles 2–33 as channels 0–31.

**Samples are offset binary** (mid-scale `0x8000` = 0 µV) — subtract `0x8000` for signed. This
layout is identical in debug mode (`SET_DEBUG_MODE`), which fills the streams with synthetic
sinewaves so a plugin can be tested with no chip attached.

**Reference parser:** `net.py` `unpack_data_segments()` (flatten words → 16-bit samples) +
`expected_debug_segments()` (the cycle×stream order) + `calculate_packet_size()` (size from
mask). The host/plugin **must** compute the expected size from the mask and check it — a size
mismatch is the classic dual-port "dropout" symptom; keep the plugin's size calc in lockstep.

## LFP UDP stream (port 5001) — the decimated band, a *separate* product

When the LFP/DSP engine is enabled, the board emits a second, independent UDP stream on
**port 5001** (broadband on 0x6800 is untouched). One datagram per decimation frame
(broadband rate ÷ effective decimation, e.g. 30 kHz ÷ 10 = 3 kHz).

**The PL builds the whole wire packet** (6-word header + samples) in the LFP output BRAM
(`0x84000000`) — just like `data_generator_core.sv` builds the 10-word broadband header in the
capture BRAM. The PS then **CDMAs the complete frame straight into a pbuf and sends it**
(`pl_dma_read_addr` → non-cacheable LFP staging → `udp_sendto` a `PBUF_REF`): no PS-side header
math and no `Xil_In32` sample loop (enforced by `scripts/check_dma.sh`). All 32-bit
little-endian:

| words | field | notes |
|------:|-------|-------|
| 0–1 | magic `0xCAFEBABE_1F1FBEEF` | word0 = `0x1F1FBEEF`, word1 = `0xCAFEBABE` (distinct from broadband) |
| 2–3 | 64-bit master **timestamp** | the master count of the **newest broadband sample in this output's decimation window** — the most recent broadband packet clocked into the (FIR) filter bank for this output. For frame `m` at total decimation `R` it is broadband packet `R·m + (R−1)` (R=10 → `10m+9`); the *same* counter the broadband header stamps (word 1). Causal, monotonic, `R` apart — an **absolute master timestamp**, not a frame index. NB: this is the newest *input* sample, not the instant the LFP value represents — subtract the filter group delay to align with broadband *content*. |
| 4 | `[7:0]` lane_mask · `[15:8]` decim_R · `[23:16]` num_taps · `[31:24]` overrun | self-describing config; `lane_mask` = the broadband `channel_enable` mask |
| 5 | 32-bit frame sequence number | PL-maintained (++ per emitted frame); LFP-stream drop detection |
| 6… | payload: per enabled lane, 32 decimated samples, **offset binary** 16-bit, 2 per word | `popcount(lane_mask) × 32` samples = `popcount × 16` words |

**Lane mask = broadband mask.** The LFP `lane_mask` is driven from the broadband
`channel_enable` mask in the PL (single source of truth) — the LFP filters exactly the
broadband-enabled lanes. **Samples are offset binary** (mid-scale `0x8000` = 0 µV), same as
broadband — subtract `0x8000` for signed. Lanes appear in ascending order; within each, the 32
amplifier channels in order. The host knows the frame size from `lane_mask`. Reference
receiver: `net.py` `receive_lfp()`.

## Register map (the source of truth is `firmware/include/main.h`)

| Region | Base | Notes |
|--------|------|-------|
| AXI-Lite control regs | `0x40000000` | 28 control regs (PS→PL): 0..21 legacy, 22..24 aux, **25..27 LFP** (cfg/coef/strobe) |
| AXI-Lite status regs | `0x40000000 + 28*4` | status regs (PL→PS): write pointer, fifo count, aux, **13 = LFP wr-ptr/overrun** (base = `PL_N_CTRL_REGS*4`) |
| Capture BRAM | `0x80000000` | 16384 × 32-bit words (64 KB) |
| **LFP output BRAM** | `0x84000000` | 16384 × 32-bit ring; PL builds the **whole wire packet** here (6-word header + samples 2×16-bit/word). In `axi_cdma_0/Data` space so the PS CDMAs each frame straight into a pbuf. |

`CTRL_REG_2` packs the four cable phases: `[3:0]` phase0 (A/cipo0), `[7:4]` phase1 (A/cipo1),
`[19:16]` phase2 (B/cipo0), `[23:20]` phase3 (B/cipo1).

### `get_status` response (`status_response_t`)
A packed **160-byte** little-endian struct returned by `GET_STATUS`. `main.h` is the
source of truth for the field layout; `net.py` `get_status()` mirrors the unpacking, and a
firmware `_Static_assert(sizeof(status_response_t) == 160)` keeps the two in sync. Contents,
in order:

- **version / IDs** — firmware version, board/build IDs.
- **PL hardware status** — 64-bit timestamp, BRAM write pointer, FIFO count, PL flags.
- **PS counters** — packets sent, error counters, PS flags.
- **current config** — `loop_count`, `phase0`/`phase1`, `channel_enable` mask, `debug_mode`.
- **UDP info** — destination IP/port, packet format, bytes sent.
- **aux-sequencer status** — `aux_read_result` (last injected command's `{cipo1, cipo0}`
  response), `aux_bank_active`, `aux_flags` (bit0 seq_en, bit1 fast-settle active, bit2
  digout, bit3 dsp, bit4 inject-ack), per-slot `aux_idx[3]`.
- **performance instrumentation** — per-packet CDMA and receive→transmit times as raw
  global-timer ticks (`dma_ticks_last/max`, `loop_ticks_last/max`), `dma_errors`, and
  `timer_hz` so the host converts ticks→µs (kept as ticks on the wire; converted only when
  printed).
- **`aux_ctrl` (`uint32`)** — `CTRL_REG_22` read back: the live fast-settle / DSP-reset /
  digout configuration (each as software-on / gpio-enable / gpio-pin-select fields), plus
  `seq_en`, active bank, and the static Reg-3 byte. Per the "get_status reports everything
  configurable" rule (CLAUDE.md).
- **`rhd_reg[22]` (`uint8` ×22)** — **RHD chip register mirror**: the firmware's view of the
  commanded state of RHD2000 registers 0..21. Seeded from the initialization sequence at
  boot (`pl_rhd_shadow_init`) and updated whenever a `WRITE_REGISTER` succeeds
  (`pl_write_rhd_register`). This is the *commanded* state, not a chip read-back — it lets a
  host/plugin recover the configured bandwidth, DSP cutoff, amplifier power-up mask, etc.
  without re-reading the chip. Registers 0 and 3 are owned by the PL override layer (amp
  fast-settle D5 and digout/temp); their *live* values are reported via `aux_ctrl`/
  `aux_flags`, so the mirror leaves those two as the static base.

  Useful registers in the mirror (see the RHD2000 datasheet for the full map):

  | reg | meaning |
  |-----|---------|
  | 0 | ADC config + amp fast-settle (bit D5 — overridden live) |
  | 3 | digital-out / temp-sensor (overridden live) |
  | 4 | DSP high-pass: enable (bit 4) + cutoff code (bits 3:0) |
  | 8, 10 | RH1/RH2 DACs — upper-bandwidth setting |
  | 12, 13 | RL DACs — lower-bandwidth setting |
  | 14–21 | per-channel amplifier power-up (8 bits each → 64 channels) |

- **LFP engine config + status (12 bytes)** — `lfp_enable`, `lfp_lane_mask`, `lfp_decim_R`,
  `lfp_num_taps`, `lfp_packets_sent` (UDP frames emitted), and `lfp_overrun` (sticky
  compute-overrun from `STATUS_REG_13`). `lfp_lane_mask` is reported as the **broadband
  `channel_enable` mask** (the single source of truth — the PL drives the LFP lane mask from
  `channel_enable_reg`), so the host can read back the full LFP configuration per the "report
  everything configurable" rule.
