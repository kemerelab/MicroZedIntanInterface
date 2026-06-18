# Command & packet structure

The board exposes two network endpoints once its Ethernet link is up:

| Endpoint | Port | Direction | Purpose |
|----------|------|-----------|---------|
| **TCP control** | 6000 | host → board (with acks) | commands (start/stop, config, registers) |
| **UDP data** | 5000 | board → host | the acquisition stream |

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
| `0x13` | SET_CHANNEL_ENABLE | p1 = mask | 8-bit stream mask (e.g. `0xFF` = all 8 streams / 128 ch) |
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

(See `remote/net.py` for the exact per-command parameter packing and the interactive
command names like `start`, `set_channels`, `auto_cable_detect`, `verify_sine`.)

## UDP data packet

Each UDP datagram is one 30 kHz sample frame: **10 header words + N data words**, all 32-bit
little-endian. `N` depends on the channel mask (`35 × num_streams` 16-bit words, packed two
per 32-bit word), so the packet is **28 words (min) … 150 words / 600 bytes (max, `0xFF`,
128 ch)**.

**Header (10 words / 40 bytes):**

| words | field | notes |
|------:|-------|-------|
| 0–1 | magic `0xCAFEBABE_DEADBEEF` | word0 = `0xDEADBEEF`, word1 = `0xCAFEBABE` |
| 2–3 | 64-bit timestamp | increments once per packet (per sample frame) |
| 4–5 | digital-in (8 bits) + metadata | |
| 6–9 | 8 × 16-bit external ADC values | |

**Data words:** the de-interleaved CIPO samples for the enabled streams (regular/DDR ×
CIPO0/CIPO1 × port A/B). The host computes the expected size from the channel mask; a
size mismatch is the classic dual-port "dropout" symptom — keep `calculate_packet_size`
in `net.py`/the plugin in lockstep with the firmware.

## Register map (the source of truth is `firmware/include/main.h`)

| Region | Base | Notes |
|--------|------|-------|
| AXI-Lite control regs | `0x40000000` | 4 control regs (PS→PL): enable / loop / phase / channel + aux |
| AXI-Lite status regs | `0x40000000 + 22*4` | 13 status regs (PL→PS): write pointer, fifo count, phases, aux |
| Capture BRAM | `0x80000000` | 16384 × 32-bit words (64 KB) |

`CTRL_REG_2` packs the four cable phases: `[3:0]` phase0 (A/cipo0), `[7:4]` phase1 (A/cipo1),
`[19:16]` phase2 (B/cipo0), `[23:20]` phase3 (B/cipo1).

### `get_status` response (`status_response_t`)
A packed struct (122 bytes) returned by `GET_STATUS` — version/IDs, PL hardware status
(timestamp, write pointer, fifo count), PS counters (packets, errors), current config
(phases, channel mask, debug mode), UDP info, aux-sequencer status, and **performance
instrumentation** (per-packet CDMA + receive→transmit time as raw global-timer ticks, plus
`timer_hz` so the host converts to µs). See `main.h` for the field layout and `net.py`
`get_status()` for the unpacking; a firmware `_Static_assert` keeps the two in sync.
