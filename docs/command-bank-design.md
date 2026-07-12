# Design — Aux Command Bank Rebuild + Fast-Settle/Digout Override

**Status:** Converged design (June 2026). Confirm chip details against
`docs/Intan_RHD2000_series_datasheet.pdf` / `docs/Intan_RHD2164_datasheet.pdf` before implementing.
Reference implementation studied: `github.com/open-ephys/rhythm` + `open-ephys-plugins`
(clones under `~/Code/Intan/`).

## Motivation

Today the streaming command stream is a flat, fixed 35-entry list replayed identically every
packet (`pl_control.c: convert_cmd_sequence`, played out by `data_generator_core.sv` over
`cycle_counter` 0..34). The three aux positions are static `CONVERT(32/33/34)`, leaving no room —
without a firmware rebuild — for temperature, supply, runtime link/health checks, impedance,
ad-hoc register access, real-time digital output, or fast settle.

This rebuild replaces the static aux positions with **programmable, looping, swappable per-slot
command banks**, adds a **real-time override layer** (fast settle + digital-out + a Register-3
shadow), and uses **PL-side frame alignment with command-echo identity** so the host receives
clean, self-contained UDP packets.

## Command source model

Per packet (= one ~30 kHz sample) the PL issues 35 commands: 32 channel converts + 3 aux.
A host-writable **full 35-entry command table** remains the base; two enable bits overlay the
segmented behavior:

```
if (cycle_counter < 32)
    cmd = auto_convert_en ? make_convert(cycle_counter)    // CONVERT(pos), fixed every packet
                          : full_cmd_table[cycle_counter]; // host table (init/cal/cable-test)
else begin
    slot = cycle_counter - 32;                              // 0,1,2
    cmd = aux_bank_en ? aux_slot[slot].read()              // looping per-slot bank
                      : full_cmd_table[cycle_counter];
end
// → override layer (below) may rewrite cmd
```

| Mode | `auto_convert_en` | `aux_bank_en` | `loop_count` | Behavior |
|---|---|---|---|---|
| Init / calibrate / cable-test | 0 | 0 | 1..N | full table replayed (today's exact pipeline) |
| Acquisition | 1 | 1 | 0 (∞) | auto converts + looping aux banks + overrides |

The segmentation is a **two-index-domain** FSM: positions 0–31 indexed by `cycle_counter` (fixed
each packet); positions 32–34 indexed by **per-slot, per-packet indices** that loop independently.

## Storage: 3 independent per-slot command BRAMs

One **dual-port BRAM per aux slot** (3 total) — *not* one shared 3×N array — so each slot's index,
length, and bank-select are fully independent (Slot 2 loops at 3, Slot 3 at ~30; trivial with
separate RAMs). Each BRAM is its own AXI **memory-mapped window** (PS writes as memory; FPGA reads
on the PL clock), separate from the control registers and the data BRAM.

- **≥2 banks per slot** (active + standby), so you write a standby bank while the FSM reads the
  active one — no torn program.
- **Length is bound to the bank.** Each bank carries its own `(loop_idx, end_idx)`, so a
  bank-select **atomically swaps the length too**. (Intan keeps length per-*slot* — a footgun we
  fix; their AuxCmd3 banks have different lengths and must be hand-synced on every swap.)
- **Per-slot index** advances once per packet, wrapping `end_idx → loop_idx`.
- **Atomic swap:** PS writes `aux_bank_select[slot]`; the FSM latches it only at a packet boundary.
- **Confirm-before-reuse handshake:** PS polls `aux_bank_active[slot]` (status reg) until it reads
  the new bank → the freed bank is now safe to write.
- **Read needs prefetch:** BRAM read has 1–2 cycle latency, so issue the `(bank,index)` address a
  state ahead inside the 80-state loop (plenty of slack).

**Multi-chip note:** one shared COPI broadcasts to both chips, so aux commands are **shared** — one
set of 3 banks, not per-chip. Each chip returns on its own CIPO line. So *identity = echoed command
(what) + CIPO line (which chip)*; no per-chip bank machinery (unlike Intan's per-port banks).

## Slot allocation (final)

| Slot | Role | Default contents | Reg writes |
|------|------|------------------|------------|
| **1 — real-time control** | digital-out (GPIO→`auxout` mirror) + fast settle | `WRITE(3,…)` every packet (digout); fast settle hijacks it on TTL edges | **sole Reg 0 + Reg 3 writer** (via shadow) |
| **2 — ADC only** | accelerometer sweep | `CONVERT(32) → CONVERT(33) → CONVERT(34)` looping → each aux input at **10 kHz** | none (pure reads) |
| **3 — config + measurements** | setup + low-rate housekeeping, banked | bank A = register config/calibration (run once); bank B = supply `CONVERT(48)`, temp read `CONVERT(49)`, link/chip-ID `CONVERT(63)`/`RegRead(40-44)`, arbitrary R/W, looping (length ~30) | many regs, time-exclusive |

This gives all goals at once: real-time digout, dedicated 10 kHz accel, and temp/supply/link +
config — no rate compromise. (Intan instead crams accel+temp+supply into one slot → 7.5 kHz accel;
dedicating Slot 2 beats that.)

## Override layer + the Register-3 shadow

Two RHD registers are shared across functions, and RHD writes set **all 8 bits at once**, so we
maintain coherent **shadows** and override any write to those registers — generalizing the trick
Intan already uses (override bit 5 on Reg-0 writes; `digout_override` on Reg-3 writes):

**Register 0 — ADC config + amp fast settle** (`D5` = amp fast settle):
- Force `D5` = live fast-settle state on *any* `WRITE(0,…)`.
- On a fast-settle TTL **edge**, replace **Slot 1's** command with `WRITE(0,0xFE)` (on) / `0x80DE`
  (off). Chip latches Reg 0 → two injections per pulse.

**Register 3 — temp sensor + auxiliary digital output** (`D0`=digout, `D2..D4`=tempen/tempS1/tempS2):
- Maintain `reg3_shadow = { static MUX/HiZ bits, tempen/tempS state, live digout bit }`.
- The **digout bit** = a software-selectable TTL/GPIO input (real-time mirror to the headstage
  `auxout` pin — this is why Intan hardcodes it: `auxout` follows a controller GPIO at ~1-sample
  latency, `main.v:1252` routes `TTL_in[channel]` → `digout_override`).
- Override *any* `WRITE(3,…)` (from any slot) with `reg3_shadow` → no slot can clobber another's
  Reg-3 bits; Reg 3 has a single coherent authority.

**Temperature accuracy (decision):**
- *Single-point* (default): fix `tempS` in the shadow, one `CONVERT(49)`. No sequencer.
- *Differential* (Intan-accurate): a small temp sequencer steps `tempS` in the shadow in sync with
  Slot 3's `CONVERT(49)`. Add only if calibrated temperature is needed.

**DSP reset** (optional digital fast settle): force the CONVERT **LSB ("bit H")** on `CONVERT(0..31)`
from software and/or a selectable pin — in-place, drops no data. Independent of amp settle.

**Invariant:** the override layer only ever *replaces* commands in **Slot 1** (the real-time-control
slot). Slots 2 (ADC) and 3 (measurements/config) are never command-replaced — only Reg-write *bits*
are coherently substituted. So the 10 kHz accel stream and the measurement stream are never perturbed.

## Data path — align in the PL, echo the command, self-contained packets

Intan resolves the SPI **2-command pipeline delay in the FPGA** (a tuned offset — `channel_MISO<=33`,
"*Bug fix: changed 2 to 33*" — plus fixed-order frame assembly), so the host reads clean, grouped
frames; it does **not** ship shifted data or read wrapped points. (The exact cycle-level grouping is
intricate and empirically tuned; verify ours by simulation rather than by replicating their magic
offset.) We do the same:

- **`aux_capture` unit:** latch each aux result, pair it with the **originating command** carried
  through the readback delay, and emit clean. *(As built, this dissolved into ~3 registers in
  the core rather than a separate module: the "intricate alignment" is just the fixed 2-command
  SPI readback pipeline — packet word 34 answers this packet's slot-1 command, words 0/1 answer
  the previous packet's slot-2/3 commands — so no generic delay-line module is needed.)*
- **Command-echo identity:** each packet's metadata carries the *originating command* per aux slot.
  The command is self-describing (`CONVERT(49)`→temp, `CONVERT(48)`→supply, `CONVERT(3x)`→accel
  axis, `READ(addr)`→that register). Host decodes with **zero** knowledge of the loaded program,
  and a dropped packet never desyncs. Robust to bank swaps too.
- **Self-contained UDP packets:** unlike Intan (continuous, reliable USB), we're on lossy UDP, so
  each packet must stand alone. Aligning in the PL gives this directly; if any result still wraps,
  pad with trailing dummy command(s) so only don't-care results cross the boundary.

## Packet metadata

Fill the header words currently `0x0` (`data_generator_core.sv:352`). Per packet:
- per protected slot: the **echoed originating command** (+ optional per-slot index for drop
  detection) — identity for the aux results on each CIPO line.
- `fast_settle_active` (1 bit), `digout_state` (1 bit), optional `slot3_bank`.
- reserved space for PS-side **BNO055 IMU** quaternion/accel (read over Zynq PS-I²C; see
  `[[openephys-accelerometer]]` — dedicated SDA/SCL on the cable, not in these slots).

## Aux command format (verified Intan interface → our adaptation)

Intan: 3 dual-port RAMs (`RAM_bank_1/2/3`, `main.v:937/972/994`), each 16 banks × 1024 × 16-bit
RHD command words; read at `RAM_addr_rd = aux_cmd_index_N`, `RAM_bank_sel_rd = bank`; written
word-by-word via WireIns `WireInCmdRamData/Addr/Bank` (0x07/0x05/0x06) + `TrigInRamWrite` (0x42)
bit 0/1/2 selecting the slot (`uploadCommandList`). Bank via `selectAuxCommandBank`, length via
`selectAuxCommandLength`.

Ours: same model, adapted — **memory-mapped BRAM window per slot** (cleaner than wire+trigger on
Zynq), **one bank-select per slot** (shared COPI, not per-port), **length bound to the bank**,
much smaller (2–4 banks, N≈30).

## Host API (new `CMD_*` in `net.py` + firmware)

| Command | Action |
|---|---|
| `AUX_BANK_WRITE(slot, bank, list)` | upload a program (incl. its length) into a standby bank |
| `AUX_BANK_SELECT(slot, bank)` | atomic swap at next packet boundary (carries the bank's length) |
| `READ_REGISTER(addr)` / `WRITE_REGISTER(addr,val)` | one-off, injected via Slot 3 (or Slot 1 for Reg 0/3 through the shadow); result tagged |
| `SET_FAST_SETTLE(pin, amp_en, dsp_en, mode)` | configure the Reg-0 override |
| `SET_DIGOUT(pin, enable)` | select the GPIO mirrored to `auxout` |

## PL module boundaries (separately testable in xsim)

| Module | Responsibility | Testbench |
|---|---|---|
| `aux_command_bank.sv` | per-slot BRAM + banks; advance index; emit command; atomic bank/length swap | load banks → clock N packets → assert command sequence, wrap, atomic swap+length |
| `override_layer.sv` | Reg-0 bit5 / Reg-3 shadow / digout / DSP bit-H; Slot-1-only replacement | feed commands + TTL edges → assert Slot-1-only replacement, shadow coherence, invariant |
| `aux_capture.sv` | pair results with originating command across the readback delay; emit labeled triples | synthetic delayed results → assert correct command↔result pairing incl. boundary |
| `data_generator_core.sv` | compose: auto-convert / table / banks → override → COPI; capture → frame | integration TB |

## Three-layer contract changes (keep in sync — CLAUDE.md)

- **PL:** new `aux_command_bank.sv`, `override_layer.sv`, `aux_capture.sv`; auto-generate
  `CONVERT(0..31)`; per-slot BRAM windows + bank-select/active status; fast-settle/digout config;
  Reg-3 shadow; command-echo metadata.
- **PS** (`main.h`, `pl_control.c`): bank upload/select + confirm handshake; `READ/WRITE_REGISTER`;
  fast-settle/digout config; `status_response_t` additions (active banks, fast-settle/digout state).
- **Host** (`net.py`): bank programming; command-echo decode; new `CMD_*`; packet-size update.

## Resolved / open

- ✅ Decode = **command-echo** (self-describing), align in PL, self-contained packets.
- ✅ Storage = **3 independent per-slot BRAMs**, ≥2 banks, **length bound to bank**, double-buffer swap.
- ✅ Slots: 1 = real-time digout+fast-settle (Reg-3 shadow), 2 = ADC-only accel @ 10 kHz,
  3 = config + supply/temp/link (banked).
- ✅ Reg 3 sharing resolved by a coherent **shadow + override**.
- ☐ Bank depth (banks/slot) and Slot-3 housekeeping length N (≈30) — finalize.
- ☐ Temperature: single-point (default) vs differential (needs a temp sequencer).
- ☐ Impedance: implement as a swap-in bank (Zcheck DAC) on a slot, or defer to its own epic.
