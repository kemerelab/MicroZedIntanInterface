# AXI-Lite register map (PS ↔ PL)

The control interface is a flat AXI-Lite register bank at **`0x40000000`**: **25 control
registers** (PS → PL, regs 0–24) immediately followed by **13 status registers** (PL → PS,
regs 0–12, so status reg *n* is at `0x40000000 + (25 + n)·4`).

`firmware/include/main.h` declares the layout; the PL **consumes** the control regs and
**produces** the status regs in `programmable_logic/src/data_generator_core.sv` (regs, phases,
command table, status 0–9, aux) and `data_generator_wrapper.v` (status 10–12). This document
is cross-referenced against both — every field below is confirmed by the PL line that uses it.
If you change the map, change `main.h`, the RTL, and this file together.

## Control registers — PS → PL (`0x40000000`, 25 regs)

| reg | offset | purpose |
|----:|--------|---------|
| 0 | `0x00` | run control |
| 1 | `0x04` | `loop_count` (32-bit; `0` = free-run) |
| 2 | `0x08` | phase select + channel enable |
| 3 | `0x0C` | analytic-chirp NCO config (debug signal gen) |
| 4–19 | `0x10`–`0x4C` | MOSI/COPI command table (32 channel RHD commands) |
| 20–21 | `0x50`–`0x54` | **unused** (freed when the aux commands moved to the engine) |
| 22 | `0x58` | aux control |
| 23 | `0x5C` | aux write payload |
| 24 | `0x60` | aux strobe |

**REG 0 — run control** · PL `data_generator_core.sv:55/100/101`

| bits | field |
|------|-------|
| `[0]` | `enable_transmission` |
| `[1]` | `reset_timestamp` |
| `[2]` | reserved |
| `[3]` | `debug_mode` (synthetic sine, no chip) |
| `[31:4]` | reserved |

**REG 2 — phase select + channel enable** · PL `:107–111`

| bits | field |
|------|-------|
| `[3:0]` | phase_a0 — cable A, line 0 |
| `[7:4]` | phase_a1 — cable A, line 1 |
| `[11:8]` | **phase_b0 (phase2)** — cable B, line 0 |
| `[15:12]` | **phase_b1 (phase3)** — cable B, line 1 |
| `[23:16]` | channel_enable — 8-bit stream mask (`[19:16]`=A, `[23:20]`=B) |
| `[31:24]` | reserved |

Each phase drives its own `CIPO_combined_phase_selector` (one per CIPO line) to compensate
cable delay.

**REG 3 — analytic-chirp NCO config** · PL `:223`

| bits | field |
|------|-------|
| `[0]` | chirp_mode |
| `[7:2]` | phase_stride (6b) |
| `[19:8]` | f_span (12b → f_max) |
| `[31:20]` | sweep_rate (12b → freq-acc step/pkt) |

**REG 4–19 — MOSI/COPI command table** · PL `:113–118`

`reg(4+j)` packs two 16-bit RHD SPI commands: `[15:0]` = `cmd[2j]`, `[31:16]` = `cmd[2j+1]`,
for `j = 0..15` (32 channel commands). The three aux cycles are supplied by the aux engine, not
here.

**REG 22 — aux control** · PL `:260`

| bits | field |
|------|-------|
| `[0]` | slot-0 program bank select (only slot 0 cycles) |
| `[3:1]` | reserved |
| `[4]` / `[5]` / `[8:6]` | fast-settle: sw / gpio_en / gpio_pin_sel |
| `[9]` / `[10]` / `[13:11]` | DSP-reset: sw / gpio_en / gpio_pin_sel |
| `[14]` / `[15]` / `[18:16]` | digout: sw / gpio_en / gpio_pin_sel |
| `[23:19]` | reserved |
| `[31:24]` | reg3_static — RHD Reg-3 bits D7..D1 (D0 = live digout) |

**REG 23 — aux write payload** · PL `:261`

| bits | field |
|------|-------|
| `[15:0]` | data (length record: `{end[5:0]<<8, loop[5:0]}` when `is_length`) |
| `[21:16]` | addr |
| `[23:22]` | target = slot index (0 = slot-0 program, 1 = slot-1 fs reg, 2 = slot-2 inject reg) |
| `[24]` | bank |
| `[25]` | is_length |
| `[31:26]` | reserved |

**REG 24 — aux strobe** · PL `:262`

| bits | field |
|------|-------|
| `[0]` | write_toggle (commit a REG 23 write) |
| `[1]` | inject_toggle (fire a slot-2 one-shot) |
| `[15:2]` | reserved |
| `[31:16]` | injected command (slot-2 one-shot, full 16-bit RHD command) |

## Status registers — PL → PS (`0x40000064`, 13 regs)

Core (`data_generator_core.sv`) drives regs 0–9; the wrapper (`data_generator_wrapper.v`)
appends 10–12.

| reg | offset | producer / content | PL |
|----:|--------|--------------------|----|
| 0 | `+0x00` | dynamic: `[0]` tx_active, `[1]` loop_limit, `[9:3]` state_counter, `[16:11]` cycle_counter | core `:654` |
| 1 | `+0x04` | reflected params (see below) — **port-A phases only** | core `:668` |
| 2 | `+0x08` | `packets_sent` | core `:681` |
| 3 / 4 | `+0x0C` / `+0x10` | timestamp low `[31:0]` / high `[63:32]` | core `:682–683` |
| 5 | `+0x14` | `loop_count` (registered) | core `:684` |
| 6 | `+0x18` | verbatim mirror of CTRL_REG_0 | core `:685` |
| 7 | `+0x1C` | verbatim mirror of CTRL_REG_1 | core `:686` |
| **8** | `+0x20` | **verbatim mirror of CTRL_REG_2** → phase_b0 `[11:8]`, phase_b1 `[15:12]`, channel_enable `[23:16]` | core `:687` |
| 9 | `+0x24` | verbatim mirror of CTRL_REG_3 | core `:688` |
| 10 | `+0x28` | `{9'd0, fifo_count, bram_write_address}` — `[13:0]` bram addr, `[22:14]` fifo count | wrapper `:192` |
| 11 | `+0x2C` | aux status (see below) | wrapper `:195` |
| 12 | `+0x30` | aux injected-command read result `{cipo_a1[31:16], cipo_a0[15:0]}` (**port A only**) | wrapper `:196` |

**REG 1 — reflected params** (`[0]` enable, `[1]` reset, `[3]` debug, `[15:12]` phase_a0,
`[19:16]` phase_a1, `[23:20]` channel_enable A, `[27:24]` channel_enable B). This register
carries **only the port-A phases**; read port B from the CTRL_REG_2 mirror (reg 8).

**REG 11 — aux status**: `[2:0]` active bank (only slot-0 moves), `[3]` engine_on (always 1),
`[4]` fast_settle_active, `[5]` digout_state, `[6]` dsp_force_h, `[7]` inject_ack toggle,
`[13:8]` slot-0 program index, `[21:16]` slot-1 index (always 0), `[29:24]` slot-2 index
(always 0).

## RHD2000 SPI command encodings

```
CONVERT(ch)      = (ch  & 0x3F) << 8
WRITE(reg, val)  = 0x8000 | ((reg & 0x3F) << 8) | (val & 0xFF)
READ(reg)        = 0xC000 | ((reg & 0x3F) << 8)
```

## Notes / gotchas

- **Two phase representations coexist in the status block.** Reg 1 (reflected params) exposes
  only the **port-A** phases at `[15:12]`/`[19:16]`; the full four-phase truth is the CTRL_REG_2
  mirror in **reg 8** (`[3:0]`/`[7:4]`/`[11:8]`/`[15:12]`). Read **port-B** phase from reg 8 at
  `[11:8]`/`[15:12]` — reading `[19:16]`/`[23:20]` there hits the channel-enable field. (This was
  the `get_status` port-B phase bug.)
- **Regs 20–21 (control) are unused** — free space that grows the map without shifting anything.
- **`aux_read_result` (status reg 12) reflects port A only** (`cipo_a0`/`cipo_a1`); a register
  read via the slot-2 inject returns the port-A headstage's response.
