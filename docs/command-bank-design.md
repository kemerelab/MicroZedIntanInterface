# Design — Aux Command Bank Rebuild + Fast-Settle Override

**Status:** Draft (captures the design discussed June 2026). Open decisions are listed at the
end. Confirm chip details against `docs/Intan_RHD2000_series_datasheet.pdf` and
`docs/Intan_RHD2164_datasheet.pdf` before implementing.

## Motivation

Today the streaming command stream is a flat, fixed 35-entry list replayed identically every
packet (`pl_control.c: convert_cmd_sequence`, played out by `data_generator_core.sv` over
`cycle_counter` 0..34). That makes the three "auxiliary" command positions static
(`CONVERT(32/33/34)` only) and leaves no room — without a firmware rebuild — for the temperature
sensor, supply-voltage monitoring, impedance testing, runtime link/health checks, or fast settle.

This rebuild replaces the static aux positions with **programmable, looping command banks** (the
Intan/OpenEphys model) and adds a **fast-settle override layer**, with an **index-tagged,
self-describing** data format so the host can decode aux results without mirroring the program.

## Background

### Current system (this repo)
- One packet = one ~30 ksps sample = **35 COPI commands**: 32 channel converts (RHD2164 reads
  64 ch via DDR) + **3 aux commands**, replayed identically every packet from host-uploaded
  `CTRL_REG_4..21`.
- Aux commands are fixed `CONVERT(32/33/34)`.
- Fast settle is **unimplemented** (TODOs at `data_generator_core.sv:273,312`).

### Intan / OpenEphys reference (github.com/open-ephys/rhythm)
- Same 32 converts + **3 independent programmable aux slots** (`AuxCmd1/2/3`), each a looping RAM
  bank with its own index / length / loop-point, selectable per port.
- Decode is **positional**: the host knows the uploaded program, computes `index = f(sample#)`,
  and accounts for the fixed **2-command pipeline delay** (result of command *i* returns 2
  commands later).
- Fast settle = edge-triggered `WRITE(0,…)` injected into `AuxCmd1` (the digout slot) +
  a bit-5 coherence override on every slot (`main.v:960-1013`). The RHD chip *latches* Register 0,
  so only two writes per pulse are needed (on at the rising edge, off at the falling edge).

## Design

### Slot map

Three aux slots, with roles assigned to enforce one **invariant: only the scratch slot is ever
command-*replaced*.**

| Slot | Role | Owns | Notes |
|------|------|------|-------|
| **S — scratch** | fast-settle home; filler `RegRead(63)` no-op | **Reg 0** writes | the *only* slot ever command-replaced. Deliberately does **not** touch Reg 3. |
| **M — measurement** | the looping "measurement cycle" | **Reg 3** (temperature `tempS1/S2`) | **PROTECTED** — never command-replaced. Its index is tagged into the packet. |
| **C — config / impedance** | bank-switched modes (normal / calibrate / Zcheck) | impedance regs 5/6/7 | only ever gets the bit-only reg-0 override, never replacement. |

One-owner-per-register avoids contention: Reg 3 holds both the digout bits *and* the temperature
`tempS1/tempS2`, so keeping Reg 3 ownership entirely in Slot M (and Reg 0 in Slot S) means the
slots never fight. (Intan put digout in the scratch slot and had to coordinate Reg 3 across slots;
we sidestep that by not doing real-time digout there.)

### Override layer (the invariant, structurally enforced)

Sits **between the bank outputs and the COPI pin**:

```
   CONVERT(0..31)            ← neural data (64ch via DDR) — untouched
   Slot S  (scratch)         ┐
   Slot M  (measurement)     ├─ bank outputs
   Slot C  (config/impedance)┘
                 │
                 ▼   override layer
   • amp settle  : on trigger EDGE, REPLACE Slot S's command with WRITE(0,0xFE/0xDE).
                   Wired to Slot S ONLY  →  invariant holds by construction.
   • reg-0 coher.: force bit5 on ANY reg-0 write in ANY slot (bit-only, never replaces).
   • DSP reset   : force bit H on CONVERT(0..31) (in-place; samples the channel anyway, drops nothing).
                 │
                 ▼
                COPI
```

- **Amp settle** (analog, Reg 0 bit 5): edge-following. Rising edge → `0x80FE`, falling edge →
  `0x80DE`, injected into Slot S for that one cycle; the chip holds the bit between edges. Trigger
  is a software-selectable `digital_in_0[*]` pin (plus a software force bit).
- **Reg-0 coherence:** any programmed `WRITE(0,…)` (e.g. Slot C config refresh) has bit 5 forced
  to the live fast-settle state, so it can't clobber an active settle.
- **DSP reset** (digital HPF, CONVERT LSB "bit H"): independently routable to software and/or a
  selectable pin. In-place modification of the channel converts — **drops no data**.

**Invariant test (xsim):** assert the trigger arbitrarily; prove `Slot M emitted == bank_M[index]`
and `CONVERT(c) channel field unchanged` on *every* cycle, including edges. Only Slot S may differ.

### Measurement-bank format

- A small **command RAM** per slot (propose depth 32–64; host-writable). Each entry: a 16-bit COPI
  command (optionally + a few "type" bits for self-documentation, though the index tag already
  identifies it).
- An **index counter** per slot advances **once per packet** (per sample), wrapping `max → loop`
  (a one-time preamble before the looping section, like Intan's `loopIndex/endIndex`).
- **Rate** of any item = `30 kHz / (loop_length / appearances_in_loop)`.

Proposed default measurement cycle (chip-side housekeeping; **the BNO055 IMU is NOT here** — it is
read by the PS over I²C in parallel and stamped into the same metadata region):

| Item | Command(s) | Rate / purpose |
|------|-----------|----------------|
| Temperature | `WRITE(3, tempS1/S2 …)` seq + `CONVERT(49)` | few Hz; multi-packet; owns Reg 3 |
| Supply voltage | `CONVERT(48)` | few Hz; health |
| Link / chip integrity | `CONVERT(63)` (chip ID) and/or `CONVERT(40–44)` ("INTAN" ROM) | continuous runtime link check (subsumes one-shot cable detection) |
| Aux analog in | `CONVERT(32/33/34)` | only if analog sensors are wired to aux1/2/3 |
| Spare / user | arbitrary `RegRead`/`RegWrite` | host-configurable entries |

### Packet metadata layout

Fill the header words currently hardwired to `0x0` (`data_generator_core.sv:352` breadcrumb). New
per-packet fields:

- `measurement_index` (8–12 bits) — **delay-adjusted**: the index of the command whose *result* is
  in this packet (PL captures it when it latches the result, so the host does a pure lookup with no
  pipeline math).
- `fast_settle_active` (1 bit) — amp-settle state this packet.
- `slotC_mode` (1–2 bits) — normal / calibrate / Zcheck (optional).
- room reserved for PS-side **IMU (BNO055) quaternion/accel** samples (Epic B/C).

### Decode contract (self-describing)

`meaning = measurement_map[tag]` — the host decodes whatever was loaded, without mirroring the bank
program. Drop a packet → lose one measurement, but **never desync** (unlike pure positional decode,
where a drop can silently misalign the aux stream). The 64-bit timestamp remains the global anchor.

### PL module boundaries (built for separable testing)

| Module | Responsibility | Pure function of | Testbench |
|--------|----------------|------------------|-----------|
| `aux_command_bank.sv` | hold bank RAM; advance per-slot indices; emit current command per slot + measurement index | host-loaded RAM, packet boundary | load known bank → clock N packets → assert emitted command sequence + index/wrap |
| `fast_settle_override.sv` | apply amp-settle replace, reg-0 bit5, DSP bit-H | command stream + trigger/software inputs | feed known commands + trigger edges → assert Slot-S-only replacement, bit5/bitH forcing, invariant |
| `data_generator_core.sv` | compose the above into the COPI assembly | — | integration TB |

Keeping the bank FSM and the override as separate modules is the whole point: each is a small,
deterministic unit that xsim can exhaustively exercise without a board.

## Three-layer contract changes (keep in sync — see CLAUDE.md)

- **PL:** new `aux_command_bank.sv` + `fast_settle_override.sv`; AXI regs/window to upload bank RAM
  and configure slot length/loop + fast-settle source (pin select, sw force, DSP enable); new
  packet metadata.
- **PS** (`main.h`, `pl_control.c`): bank-upload API; commands for fast-settle config and slot
  programming; `status_response_t` additions (fast-settle state, measurement index).
- **Host** (`net.py`): bank programming + `measurement_map`; packet decode of the index tag; new
  `CMD_*` IDs; `calculate_packet_size` update for the new metadata.

## Open decisions

1. **Decode:** positional vs tagged → **recommend tagged** (per-packet `measurement_index`).
2. **Slot count:** 3 (matches Intan; already what the packet has room for).
3. **Bank depth** and **upload path:** AXI register burst vs a dedicated BRAM window.
4. **Channel converts:** keep host-defined, or generate `CONVERT(0..31)` in the PL.
5. **Fast settle:** edge-following (confirmed); pin software-selectable (confirmed); DSP-reset also
   GPIO-pulsable, configurable together-with or separate-from amp settle (confirmed).
6. **Measurement cycle default contents** (table above) — finalize the must-haves.
