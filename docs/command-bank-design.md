# Design — Aux Command Bank Rebuild + Fast-Settle Override

**Status:** Draft (captures the design discussed June 2026). Open decisions are listed at the
end. Confirm chip details against `docs/Intan_RHD2000_series_datasheet.pdf` and
`docs/Intan_RHD2164_datasheet.pdf` before implementing.

## Motivation

Today the streaming command stream is a flat, fixed 35-entry list replayed identically every
packet (`pl_control.c: convert_cmd_sequence`, played out by `data_generator_core.sv` over
`cycle_counter` 0..34). The three "auxiliary" positions are static (`CONVERT(32/33/34)` only),
leaving no room — without a firmware rebuild — for the temperature sensor, supply-voltage
monitoring, runtime link/health checks, impedance testing, ad-hoc register access, or fast settle.

This rebuild replaces the static aux positions with **programmable, looping, swappable command
banks** (the Intan/OpenEphys model), adds a **fast-settle override layer**, and uses an
**index-tagged, self-describing** data format so the host decodes aux results without mirroring the
program.

## Background

### Current system (this repo)
- One packet = one ~30 ksps sample = **35 COPI commands**: 32 channel converts (RHD2164 reads
  64 ch via DDR) + **3 aux commands**, replayed identically every packet.
- Aux commands fixed at `CONVERT(32/33/34)`. Fast settle unimplemented (`data_generator_core.sv:273,312`).

### Intan / OpenEphys reference (github.com/open-ephys/rhythm)
- 32 converts + **3 independent programmable aux slots** (`AuxCmd1/2/3`), each a looping RAM bank
  with its own index/length/loop-point, **with multiple banks per slot selectable on the fly**.
- Decode is **positional**: host knows the program, computes `index = f(sample#)`, accounts for the
  fixed **2-command pipeline delay**.
- Fast settle = edge-triggered `WRITE(0,…)` injected into the digout slot + a bit-5 coherence
  override on every slot (`main.v:960-1013`). The RHD chip *latches* Register 0, so a pulse needs
  only two writes (on at the rising edge, off at the falling edge).

## Design

### What's in a bank: aux-only, multi-bank, swappable

- **Aux-only.** The PL **auto-generates** `CONVERT(0..31)` (invariant for the RHD2164 — those 32
  converts DDR-read all 64 channels). Banks hold **only the aux commands**, so the host can never
  corrupt the neural sampling, banks stay small, and the API is focused on what actually varies.
  Per-convert tweaks (the DSP-reset bit-H) are applied by the *override layer*, not by editing a bank.
- **Multiple banks per slot, preloaded and swapped** (Intan's safety model). Each slot has N banks
  (2–4). You build/validate a program offline, load it into an *unused* bank, then flip a small
  **bank-select** register; the swap is **atomic at a packet boundary**, the running bank is never
  mid-edit, and modes (calibrate↔normal, normal↔impedance) become a register change, not a re-upload.

### Slot map

Three aux slots (positions 33/34/35 of each packet; positions 1–32 are the auto-generated channel
converts). Roles enforce the invariant **only the scratch slot is ever command-*replaced*.**

| Slot | Role | Owns (reg) | Protected? |
|------|------|-----------|-----------|
| **1 — Aux ADC** | round-robin `CONVERT(32/33/34)` → each aux input at **10 kHz** | none | **yes** (bank-swappable to Zcheck DAC during impedance only) |
| **2 — Housekeeping** | looping low-rate measurement bank: temperature, supply, link/chip-ID, user periodic reads | **Reg 3** (temperature) | **yes** |
| **3 — Scratch / control** | fast-settle home; on-demand `READ/WRITE_REGISTER`; config bank-switching | **Reg 0** (fast settle) | no — the only command-replaced slot |

### Register ownership (why slots "own" registers)

An RHD register write sets **all 8 bits at once** — there is no per-bit masking on the chip. So two
slots writing the *same* register fight over the shared byte. We avoid this by giving each
contended register a single owner:

**Register 0 — ADC config + amplifier fast settle** (`WRITE(0,…)`):
```
D7:6 ADC reference BW (=3 always)
D5   amp fast settle   ← pulsed by 0x80FE (set) / 0x80DE (clear)
D4   amp Vref enable
D3:2 ADC comparator bias
D1:0 ADC comparator select
```
Owned by **Slot 3** (fast settle). Any `WRITE(0,…)` flowing through another slot gets bit 5 forced
to the live fast-settle state (the override below), so a config refresh can't clear an active settle.

**Register 3 — MUX load, temperature sensor, auxiliary digital output** (`WRITE(3,…)`):
```
D7:5 MUX load (=0 always)
D4   tempS2 ┐ temperature-sensor switch controls — toggled across samples,
D3   tempS1 ┘ then CONVERT(49) reads the result
D2   tempen  (temperature sensor enable)
D1   digout HiZ ┐ auxiliary digital output pin (auxout)
D0   digout     ┘
```
Owned by **Slot 2** (temperature). Because Reg 3 also holds the digout pin, we deliberately do
**not** do real-time digout in another slot (Intan put digout in the scratch slot and had to
coordinate Reg 3 across slots; we sidestep that). Caveat: `WRITE_REGISTER` (via Slot 3) *can* target
Reg 3 — document that ad-hoc writes during streaming must avoid registers an active slot owns.

### Override layer (invariant, structurally enforced)

```
   CONVERT(0..31)            ← neural data (64ch via DDR) — untouched
   Slot 1 (Aux ADC)          ┐
   Slot 2 (Housekeeping)     ├─ bank outputs
   Slot 3 (Scratch/control)  ┘
                 │  override layer
                 ▼
   • amp settle  : on trigger EDGE, REPLACE Slot 3's command with WRITE(0,0xFE/0xDE).
                   Wired to Slot 3 ONLY  →  invariant holds by construction.
   • reg-0 coher.: force bit5 on ANY reg-0 write in ANY slot (bit-only, never replaces).
   • DSP reset   : force bit H on CONVERT(0..31) (in-place; channel still sampled, drops nothing).
                 │
                 ▼  COPI
```

- **Amp settle** (analog, Reg 0 bit 5): edge-following; software-selectable `digital_in_0[*]` pin +
  software force bit. Chip latches Reg 0 → two writes per pulse.
- **DSP reset** (digital HPF, CONVERT bit H): independently routable to software and/or a selectable
  pin — usable together with amp settle or separately. In-place; drops no data.

**Invariant test (xsim):** assert the trigger arbitrarily; prove `Slot 1/2 emitted == bank[index]`
and `CONVERT(c) channel field unchanged` on *every* cycle, including edges. Only Slot 3 may differ.

### Default aux command cycle

One packet = one 30 kHz sample, so each slot emits one command per packet.

**Slot 1 — Aux ADC @ 10 kHz** (3-entry loop):
```
idx 0: CONVERT(32)  → AuxIn1
idx 1: CONVERT(33)  → AuxIn2
idx 2: CONVERT(34)  → AuxIn3       each input = 30 kHz / 3 = 10 kHz
```
Dedicating a whole slot to the aux ADC is *better than Intan* (theirs shares one slot, capping aux
at ¼ rate); here aux gets the full 10 kHz with nothing else interleaved.

**Slot 2 — Housekeeping** (looping; length tunes the rate — these signals are slow, so a long loop
is fine). Representative program:
```
idx 0: WRITE(3, tempen|tempS1)         // temp setup A
idx 1: WRITE(3, tempen|tempS1|tempS2)  // temp setup B
idx 2: CONVERT(49)                     // temperature
idx 3: CONVERT(48)                     // supply voltage
idx 4: CONVERT(63)                     // chip ID  ← continuous link/health check
idx 5: RegRead(40..44)                 // "INTAN" ROM ← link integrity (subsumes cable detection)
idx 6..N: user-defined periodic reads / RegRead(63) no-op padding
```
Rate of any item = `30 kHz × appearances / loop_length`.

**Slot 3 — Scratch / control** (default = `RegRead(63)` no-op every packet):
- fast-settle override replaces it on trigger edges (`WRITE(0,…)`),
- `READ_REGISTER` / `WRITE_REGISTER` inject here on demand (result returns 2 cmds later, tagged),
- hosts config/impedance bank-switching.

So: **one slot streams the aux ADC at 10 kHz; the other two carry all housekeeping and user-specified
I/O.**

### Bank format

- Per-slot **command RAM**, N banks deep × M entries (propose M = 32–64). Each entry: 16-bit COPI command.
- Per-slot **index** advances once per packet, wrapping `max → loop` (one-time preamble + looping
  section, like Intan's `loopIndex/endIndex`).
- Per-slot **bank-select** register for atomic swaps.

### Packet metadata + decode contract

Fill the header words currently `0x0` (`data_generator_core.sv:352`). Per packet add:
- `meas_index` per protected slot (Slots 1 & 2) — **delay-adjusted**: the index of the command whose
  *result* is in this packet, so the host does a pure `map[tag]` lookup with no pipeline math.
- `fast_settle_active` (1 bit); optional `slot3_tag` (what Slot 3 did: noop / settle / reg-read result).

**Self-describing:** drop a packet → lose one measurement, never desync (unlike pure positional
decode). The 64-bit timestamp stays the global anchor.

### Host API (new `CMD_*` in `net.py` + firmware)

| Command | Action |
|---|---|
| `AUX_BANK_WRITE(slot, bank, list)` | upload an aux program into a bank |
| `AUX_BANK_SELECT(slot, bank)` | atomic swap at next packet boundary |
| `AUX_BANK_LENGTH(slot, loop_idx, end_idx)` | set loop bounds |
| `READ_REGISTER(addr)` | inject `READ(addr)` into Slot 3; return result (tagged) |
| `WRITE_REGISTER(addr, val)` | inject `WRITE(addr,val)` into Slot 3 |
| `SET_FAST_SETTLE(pin, amp_en, dsp_en, mode)` | configure the override |

Default banks preloaded at init (Slot 1 = aux@10 kHz, Slot 2 = temp/supply/link, Slot 3 = no-op).
Fast settle and ad-hoc register access share Slot 3; on a collision **fast settle wins** and the
register op waits one cycle.

### PL module boundaries (built for separable testing)

| Module | Responsibility | Pure function of | Testbench |
|--------|----------------|------------------|-----------|
| `aux_command_bank.sv` | bank RAM + banks/slot; advance indices; emit per-slot command + meas index | host-loaded RAM, packet boundary, bank-select | load known banks → clock N packets → assert command sequence, index wrap, atomic swap |
| `fast_settle_override.sv` | amp-settle replace (Slot 3 only), reg-0 bit5, DSP bit-H | command stream + trigger/software | feed known commands + trigger edges → assert Slot-3-only replacement, bit forcing, invariant |
| `data_generator_core.sv` | compose into COPI assembly (auto-gen converts + 3 slots + override) | — | integration TB |

## Three-layer contract changes (keep in sync — see CLAUDE.md)

- **PL:** new `aux_command_bank.sv` + `fast_settle_override.sv`; auto-generate `CONVERT(0..31)`; AXI
  regs/window for bank RAM upload, per-slot length/loop + bank-select, fast-settle source; new packet metadata.
- **PS** (`main.h`, `pl_control.c`): bank-upload + select API; `READ/WRITE_REGISTER`; fast-settle
  config; `status_response_t` additions (fast-settle state, meas indices).
- **Host** (`net.py`): bank programming + `measurement_map`; index-tag decode; new `CMD_*`;
  `calculate_packet_size` for the new metadata.

## Open decisions

1. **Decode:** positional vs tagged → **tagged** (per-packet `meas_index`). ✅ decided.
2. **Slot count:** 3 — Slot 1 aux@10 kHz, Slots 2–3 everything else. ✅ decided.
3. **Bank contents:** aux-only (PL auto-generates converts); multi-bank swappable. ✅ decided.
4. **Bank depth (M)** and **upload path:** AXI register burst vs a dedicated BRAM window — open.
5. **Fast settle:** edge-following, pin software-selectable, DSP-reset also GPIO-pulsable
   (together/separate). ✅ decided.
6. **Housekeeping loop length** and final must-have entries — open (defaults proposed above).
7. **Impedance:** implement as a bank swapped into Slot 1 during a Z-check, or defer to its own epic — open.
