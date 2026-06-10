# Night log — aux command sequencer (banked aux COPI source)

Date: 2026-06-09 → 06-10 overnight. Branch: `claude/roadmap`.
Goal: make the 3 aux COPI positions (cycles 32/33/34) sourceable from a
programmable banked sequencer, gated by a new `aux_seq_en` that defaults OFF so
the datapath stays bit-identical until enabled. Phased; verify each phase before
the next. Implements `docs/command-bank-design.md` (Epic A).

**Bottom line:** Phases 1, 2, 4 are done, verified, and committed to
`claude/roadmap`. Phase 3 (wiring the sequencer into the live datapath) is
functionally complete and passes whole-design synthesis, but the **full
place-and-route FAILS on routing congestion**, so per the task rules it was
**NOT committed to `claude/roadmap`** — it is preserved on a separate WIP branch.
`claude/roadmap` is left at a clean, synthesis-passing state.

---

## What is on `claude/roadmap` (committed, verified)

| Commit | Phase | What | How verified |
|--------|-------|------|--------------|
| `0b9c119` | 1 | `aux_command_sequencer.sv` (per-slot distributed-RAM banks, looping index, per-bank length, atomic packet-boundary swap) | Vivado OOC synth, `xc7z020clg400-1`, `-mode out_of_context` (below) |
| `bb39b0f` | 2 | `sim/aux_command_sequencer_tb.sv` + `sim/run_aux_seq_tb.sh` | xsim 2025.1: **68 checks, 0 errors, RESULT: PASS** |
| `60d0e89` | 4 | `override_layer.sv` + `aux_capture.sv` (un-integrated drafts) | elaborate + OOC synth clean (below) |

### Phase 1 — sequencer (VERIFIED)
Reviewed `aux_command_sequencer.sv` against `docs/command-bank-design.md`: 3
uniform per-slot engines, distributed-RAM command store, ≥2 banks/slot, length
record at addr 0 → per-bank `(loop,end)` regs, combinational read, atomic swap
latched at `packet_start`, `bank_active` status. Matches the spec.

OOC synth (`synth_design -top aux_command_sequencer -part xc7z020clg400-1 -mode out_of_context`):
- **Distributed RAM only** — `RAM32M ×9`, `RAM64M ×30`, `RAM64X1D ×6`;
  `LUT as Distributed RAM = 168`; **Block RAM Tile = 0** (no BRAM inferred).
- **263 Slice LUTs** (168 LUT-as-memory + ~95 logic), **21 FFs**.
- **0 latches** (`Register as Latch = 0`), **0 multidriver**, 0 errors,
  0 critical warnings. (Lone warning = benign "parallel synthesis criteria".)
- `xvlog`/`xelab` elaborate clean.

### Phase 2 — testbench (VERIFIED)
`programmable_logic/sim/aux_command_sequencer_tb.sv`, self-checking against an
independent shadow model. Reproduce: `source .../Vivado/settings64.sh && bash
programmable_logic/sim/run_aux_seq_tb.sh` (greps `RESULT: PASS`). xelab needs
`-timescale 1ns/1ps` because the production RTL carries no \`timescale (the run
script supplies it). Asserts, and all pass (**68/68, 0 errors**):
- A: index loops `loop_idx → end_idx → loop_idx`.
- B: 3 slots advance independently (incl. a loop point ≠ 1, and a length-1 slot).
- C: a bank swap lands ONLY at `packet_start`, never on an async `bank_select` change.
- D: length is bound to the bank (banks of length 3 vs 5; wrap uses the active bank).
- E: writing a standby bank never disturbs the active output/index.
- F: `aux_seq_cmd == mem[{active_bank, index}]` throughout (data-path check).

### Phase 4 — drafts (VERIFIED to elaborate/synth; behavior NOT verified)
Both **un-integrated** (not instantiated anywhere) and committed so the design
intent is captured. Bit positions taken from `firmware/src-core0/pl_control.c`
(`WRITE=10AAAAAA VVVVVVVV`, `READ=11…`, `CONVERT=00CCCCCC 0000000X`, fast-settle
amp = Reg-0 D5 `0x80FE`/`0x80DE`). **The exact bit positions and the readback
pipeline alignment must be reconfirmed against the RHD2000 datasheet and verified
in simulation before integration.**
- `override_layer.sv` — amp fast settle (Reg-0 D5 coherence + edge-injection into
  the RT slot only), Reg-3 shadow (digout/temp), DSP-reset bit-H on channel
  converts. OOC: **23 LUTs, 2 FFs, 0 latches**, 0 errors/critical.
- `aux_capture.sv` — command-echo identity: a PIPE-deep delay line pairs each aux
  result with its originating command across the readback pipeline + packet
  boundary; emits `{cmd, cipo0, cipo1}` per slot. OOC: **5 LUTs, 284 FFs, 0
  latches**, 0 errors/critical.

---

## Phase 3 — integration: FAILED the full build (NOT on `claude/roadmap`)

The integration itself is complete and is preserved on branch
**`claude/aux-seq-integration-wip`** (commit `639fd18`). It was deliberately not
merged because the gate ("commit only if BUILD_PASS and timing met") was not met.

### What the integration does (all default-OFF / additive)
- `data_generator_core.sv`: 3 new ports (`aux_seq_en`, `aux_seq_cmd[3*16]`,
  `packet_start` out); command-source mux at the COPI serializer — when
  `aux_seq_en && cycle_counter>=32`, source from `aux_seq_cmd[slot]`, else the
  legacy `copi_words_reg` (so `aux_seq_en=0` is **bit-identical** to today);
  `packet_start = transmission_active && is_first_cycle && state_counter==0`.
- `axi_lite_registers.v`: `N_CTRL 22→25`, `N_STATUS 11→12` (params only).
- `data_generator_wrapper.v`: instantiates the sequencer; decodes additive ctrl
  regs 22 (`aux_seq_en`+`bank_select`), 23 (write payload), 24 (CDC-safe write
  *toggle* edge-detected into a 1-cycle `wr_en` pulse); status reg 11 = `bank_active`.
- `led_status_controller.v`: status port width 11→12 (reads reg 0 only).
- `firmware/include/main.h`: additive `CTRL_REG_AUX_*` (22–24) + field macros;
  `STATUS_REG_*_OFFSET` shifted +3 (status space starts after N_CTRL=25) and new
  `STATUS_REG_11_OFFSET`. All firmware access goes through these macros (verified
  no raw offsets hardcoded), so PL + firmware stay consistent.

### What passed
- **Whole-design OOC synth** (`synth_design -top data_generator …`): 0 errors,
  0 critical warnings, **0 latches**; the sequencer is present and maps to
  **distributed RAM (0 BRAM tiles)**; `data_generator` = 7181 LUTs total. The
  315 warnings are all benign (reserved-bit constant drivers, `_reg removed`
  combinational temporaries, pre-existing `transmission_active` forward-ref).
- **Project + block-design generation** (`STEP1_OK`): the BD regenerated cleanly
  with the new `ctrl_regs_pl`/`status_regs_pl` widths across all three module
  references (axi_lite_registers, data_generator, led_status_controller) — so the
  bus-width + CDC parts of the integration are sound.

### What FAILED — routing congestion
`build_bitstream` → synthesis OK, placement OK, then **`route_design` FAILED**:
```
ERROR: [Route 35-2] Design is not legally routed. There are 8142 node overlaps.
CRITICAL WARNING: [Route 35-162] 7352 signals failed to route due to congestion.
```
(full impl log saved to `/tmp/phase3_impl_runme.log`; build ran 2h32m).

Diagnosis:
- **Global** routing utilization is only **~22–24%**, but there are **localized
  hotspots at 99–101%** — this is local congestion, not resource exhaustion.
- The top-10 overlapping nets are **almost entirely the PRE-EXISTING
  `fifo_bram_inst/write_fifo[*]` segment buffer** (indices up to [195]) and its
  replicated `fifo_write_data_reg`/`fifo_read_ptr`/`rstn` nets — **none** are the
  `aux_seq_inst` sequencer or the aux command path.
- Pre-route timing was already **WNS = −3.5 ns, TNS = −59,000 ns** (≈16k failing
  endpoints) at post-physopt — implausible for a hardware-proven 84 MHz design,
  which points to a pre-existing over-pessimistic timing/constraint situation.
  The router then abandoned timing to chase routing and still diverged
  (WNS drifted −3.5 → −9.3).

Interpretation: the congestion lives in the existing `fifo_bram_interface`, and
this otherwise-inert (default-OFF) integration added just enough global placement
pressure to push an already-marginal design past the routability cliff.

### Causation check (baseline) — <!--BASELINE_STATUS-->
A clean build of `claude/roadmap` HEAD (`60d0e89`) was launched to confirm the
branch builds and to isolate causation. At HEAD the 3 new `.sv` files are
**un-instantiated**, so its synthesized netlist and P&R are identical to the true
pre-integration baseline. **Result: _pending — see the final entry below._**

---

## State of the tree

- `claude/roadmap` @ `60d0e89`: clean, whole-design OOC-synth-passing; adds the
  sequencer + tb + Phase-4 drafts as files. The sequencer/override/capture are
  **not instantiated**, so the produced bitstream is identical to the baseline.
- `claude/aux-seq-integration-wip` @ `639fd18`: the full Phase-3 integration,
  preserved for follow-up. **Do not merge** until the congestion is resolved.

## Recommended next steps
1. **Resolve the congestion in `fifo_bram_interface`** (the actual bottleneck,
   independent of this work): run `report_design_analysis -congestion
   -complexity` on `impl_1/design_1_wrapper_routed_error.dcp`; try a
   congestion-oriented implementation strategy
   (`Performance_Explore`/`Congestion_SpreadLogic_high`), and/or a Pblock to
   spread the `write_fifo` segment buffer. This likely helps the baseline too.
2. **Fix the constraints** so the timing report reflects reality (the −59k TNS
   suggests unconstrained/false cross-clock paths — e.g. AXI↔PL and the 175 MHz
   domain). A design that times correctly routes far more easily.
3. Once the design routes with headroom, **re-apply `claude/aux-seq-integration-wip`**
   (`git cherry-pick 639fd18` or `git diff 60d0e89 claude/aux-seq-integration-wip`)
   and rebuild. The integration is default-OFF and bit-identical when disabled, so
   it is safe to carry once the platform routes.
4. Then proceed to host/firmware (`pl_control.c`, `net.py`) for bank upload +
   `aux_seq_en`, and to integrating the Phase-4 `override_layer`/`aux_capture`
   (after datasheet + sim verification of bit positions and pipeline alignment).

## How to reproduce
- Sequencer TB: `source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash
  programmable_logic/sim/run_aux_seq_tb.sh`
- OOC synth (any module): `read_verilog -sv …; synth_design -top <m> -part
  xc7z020clg400-1 -mode out_of_context; report_utilization`
- Full build: `bash scripts/clean_build_all.sh` (greps `BUILD_PASS`/`BUILD_FAIL`;
  toolchain at `/opt/Xilinx/2025.1`, not `~/Xilinx`).
