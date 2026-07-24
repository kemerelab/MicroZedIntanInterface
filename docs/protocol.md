# Command & packet structure

The board exposes two network endpoints once its Ethernet link is up:

| Endpoint | Port | Direction | Purpose |
|----------|------|-----------|---------|
| **TCP control** | 0x6900 | host → board (with acks) | commands (start/stop, config, registers) |
| **UDP data** | 0x6800 | board → host | the acquisition stream |

Default board IP is `192.168.18.10` (put your host on the same `/24`). The board also
broadcasts a ~1 Hz discovery beacon so `net.py` can find it without a hard-coded IP.

`remote/net.py` and `firmware/include/main.h` are the **authoritative** sources for the exact
IDs, bit layouts, and offsets — this document is the readable summary. Firmware, `net.py`, and
the Open Ephys plugin are the **three consumers of the same contract**; changing it means
changing all three. This is the **v2** contract: aux command sequencing is on by default (the
firmware version's MAJOR field bumps when the contract breaks).

## TCP control commands

Every command is a fixed **20-byte** little-endian frame:

```
uint32  magic    = 0xDEADBEEF   (CMD_MAGIC — the command magic, distinct from the packet magic)
uint32  cmd_id
uint32  ack_id                  (echoed in the ack so the host can match responses)
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
| `0x13` | SET_CHANNEL_ENABLE | p1 = mask | 8-bit stream mask (`0xFF` = all 8 streams / 256 ch) |
| `0x20` | LOAD_CONVERT | — | load the per-sample CONVERT command sequence |
| `0x21` | LOAD_INIT | — | load the RHD2000 init/config sequence |
| `0x22` | LOAD_CABLE_TEST | — | load the cable/phase test sequence |
| `0x30` | FULL_CABLE_TEST | — | run the full phase sweep |
| `0x40` | GET_STATUS | — | returns the `status_response_t` struct (see below) |
| `0x41` | DUMP_BRAM | p1 = word, p2 = count | debug: dump BRAM words to the serial console |
| `0x50` | SET_UDP_DEST | p1 = ip, p2 = port | set the UDP stream destination |
| `0x60` | PING | — | liveness check |
| `0x70` | AUX_WRITE_WORD | p1 = target \| bank<<8 \| is_len<<16; p2 = addr<<16 \| data | program a slot-0 command word (or its loop length) |
| `0x71` | AUX_BANK_SELECT | p1 = slot; p2 = bank | select the active program bank for slot 0 |
| `0x73` | READ_REGISTER | p1 = reg | inject a one-shot RHD `READ(reg)` on aux slot 2 → 4-byte `{cipo1, cipo0}` response |
| `0x74` | WRITE_REGISTER | p1 = reg, p2 = value | inject a one-shot RHD `WRITE(reg,value)` on aux slot 2 → 4-byte echo |
| `0x75` | SET_FAST_SETTLE | p1 = amp cfg, p2 = dsp cfg | `sw \| gpio_en<<1 \| pin<<4` per field |
| `0x76` | SET_DIGOUT | p1 = sw \| gpio_en<<1 \| pin<<4; p2 = reg3 static byte | digital-out control |

> **Removed in v2:** `0x72 AUX_SEQ_EN`. The aux command engine is **always on** — there is no
> enable step — so the command was retired. (A v1 host that sends it will not interoperate;
> this is why the version's MAJOR field bumped.)

See `remote/net.py` for the exact per-command parameter packing and the interactive command
names (`start`, `set_channels`, `auto_cable_detect`, `verify_sine`, `aux_selftest`, …).

## The aux command slots (always on)

Every 30 kHz acquisition frame issues the 32 amplifier `CONVERT`s **plus three auxiliary
command slots** — the engine runs continuously, there is nothing to enable:

| slot | role | default | how the host uses it |
|-----:|------|---------|----------------------|
| 0 | **accelerometer sweep** | a 3-entry program cycling the on-headstage accel axes | its reply is de-interleaved **in-frame** (no round trip); reprogram with `AUX_WRITE_WORD` / `AUX_BANK_SELECT` |
| 1 | **fast-settle / status register** | `READ(40)` (the `'I'` of the `INTAN` ROM) | fast-settle / digout overrides ride here (`SET_FAST_SETTLE`, `SET_DIGOUT`) |
| 2 | **one-shot register inject** | on-chip temperature `CONVERT(49)` | `READ_REGISTER` / `WRITE_REGISTER` drop a single command here; the chip's reply returns in the next frame |

Because reading **any** RHD register goes through slot 2, the host can read or write any
register — and identify the headstage — inline with the data stream at single-sample latency.
`net.py aux_selftest` uses this to read the RHD `INTAN` ROM (registers 40–44) and the chip-ID
/ amplifier-count registers and validate them per CIPO lane: a deterministic pass/fail on the
aux path plus a "headstage connected and talking" check.

## UDP data packet (port 0x6800)

Each UDP datagram is **one 30 kHz sample frame**: a **14-word unified header** + `N` data
words, all 32-bit little-endian. The 64-bit timestamp increments once per datagram, and the
header carries a **per-stream monotonic sequence number** — the host flags any gap, so loss is
*proven* zero, not assumed (a clean broadband run shows 0 SEQ gaps). Packets are sized so each
fits one standard datagram (no IP fragmentation, no jumbo frames, no MTU framer).

### Which channels get sampled — the channel-enable mask

`SET_CHANNEL_ENABLE` (`0x13`, param1) is an **8-bit mask selecting which of 8 streams** are
sent; each set bit adds one stream of 32 amplifier channels, and the bit order is **also the
order the streams appear in every packet**:

| bit | stream | port | CIPO line | phase |
|----:|--------|------|-----------|-------|
| 0 | `A_CIPO0_REG` | A | CIPO0 | regular |
| 1 | `A_CIPO0_DDR` | A | CIPO0 | DDR (2nd interleaved chip) |
| 2 | `A_CIPO1_REG` | A | CIPO1 | regular |
| 3 | `A_CIPO1_DDR` | A | CIPO1 | DDR |
| 4 | `B_CIPO0_REG` | B | CIPO0 | regular |
| 5 | `B_CIPO0_DDR` | B | CIPO0 | DDR |
| 6 | `B_CIPO1_REG` | B | CIPO1 | regular |
| 7 | `B_CIPO1_DDR` | B | CIPO1 | DDR |

`0xFF` = all 8 streams = **256 amplifier channels** (the DDR streams are a second chip
interleaved on the same CIPO line via double-data-rate). The largest packet (`0xFF`) is
**154 words / 616 bytes**.

### Data words — layout (what a receiver must de-interleave)

Each enabled stream contributes **35 sixteen-bit samples per frame** (the 35 COPI commands of
the acquisition loop — 32 channel `CONVERT`s + the 3 aux slots), packed **two samples per
32-bit word, low half first**:

```
data_word[i] = (sample[2*i+1] << 16) | sample[2*i]
```

The samples are **cycle-major** — the outer loop is the 35 cycles, the inner loop is the
enabled streams in **ascending bit order**:

```
flat_index = 0
for cycle in 0..34:                    # 35 COPI commands
    for stream in enabled_streams:     # ascending mask bit
        sample[cycle][stream] = data_sample[flat_index];  flat_index += 1
```

**Mapping cycle → amplifier channel.** Because of the 2-command SPI readback pipeline a
`CONVERT(ch)` result lands two cycles later, so **amplifier channel `ch` (0–31) is at
`cycle = ch + 2`**; cycles 0–1 and the aux cycles carry pipeline / aux readback, not amplifier
data. **Samples are offset binary** (mid-scale `0x8000` = 0 µV — subtract `0x8000` for
signed). This layout is identical in debug mode (`SET_DEBUG_MODE`), which fills the streams
with synthetic sinewaves so a plugin can be tested with no chip attached.

**Reference parser:** `net.py` computes the expected packet size from the mask and checks it
(a size mismatch is the classic dual-port "dropout" symptom), then de-interleaves in the
cycle × stream order above.

## `get_status` response

`GET_STATUS` returns a packed little-endian `status_response_t`. `firmware/include/main.h` is
the source of truth for its exact layout — a firmware `_Static_assert` on `sizeof` keeps
`net.py`'s unpacking in sync. It reports the firmware version and IDs, PL/PS counters, the
current config (loop count, cable phases, channel-enable mask, debug mode), the UDP
destination, per-packet CDMA / receive timing instrumentation, the **aux status** (last
injected register's `{cipo1, cipo0}` result, per-slot program state, and the live
fast-settle / digout / DSP overrides), and the RHD register mirror (the firmware's commanded
view of RHD2000 registers 0–21).
