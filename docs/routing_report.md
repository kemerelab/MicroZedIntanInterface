# PL Routing / Efficiency Report — `main` baseline

**Build:** clean rebuild from `main` HEAD (`2be1513`), Vivado 2025.1, part `xc7z020clg400-1`.
Flow: `create_vivado_project.tcl` → `build_bitstream.tcl` (synth → impl → route → write_bitstream → XSA).
**Result:** synthesis, implementation, routing, and `write_bitstream` all completed; `klab_project.xsa`
+ `design_1_wrapper.bit` produced. **Design is fully routed (0 routing errors, 0 DRC), but FAILS timing.**

Authoritative reports in `routing_analysis/` (from the routed DCP):
`util_full.rpt`, `util_hier.rpt`, `clock_interaction.rpt`, `cdc.rpt`, `congestion.rpt`,
`worst_175_to_84.rpt`, `worst_84_intra.rpt`, `high_fanout.rpt`, `control_sets.rpt`.

---

## 0. UPDATE — fixes implemented & verified (branch `claude/routing-efficiency`)

Three changes were made and re-built clean (synth → impl → route → bitstream) on top of `main`:

- **A — reset domain rewire** (`programmable_logic/block_design/design_1_bd.tcl`): `data_generator/rstn`
  moved from `proc_sys_reset_175MHz/interconnect_aresetn` → `proc_sys_reset_0_84M/peripheral_aresetn`.
- **A2 — clock grouping** (`programmable_logic/constraints/zzz_clock_groups.xdc`, new):
  `set_clock_groups -asynchronous` between the 84 MHz and 175 MHz clk_wiz outputs (the CDC crossings
  were being timed against the 0.476 ns beat; the auto-exported `top.xdc` itself noted these clocks
  "are asynchronous, user should constrain them appropriately").
- **B — memory-backed FIFO** (`programmable_logic/src/fifo_bram_interface.sv`): added
  `(* ram_style = "distributed" *)` and removed the per-element reset loop, so `write_fifo` infers
  LUTRAM instead of ~17.7k flip-flops + a 256:1 read mux. (FSM logic unchanged; async read preserved.)
- **C — AXI fabric clock** (`programmable_logic/block_design/design_1_bd.tcl`): clk_wiz `clk_out2`
  lowered **175 → 131.25 MHz** (MMCM output divider 6→8; same 1050 MHz VCO, so the 84 MHz data clock
  is unchanged). The remaining 16 failing endpoints after A+A2+B were all inside the hardened PS7 on
  its AXI-GP port, which the Zynq-7020 ‑1 rates at ~150 MHz — 175 MHz was over-spec. 131.25 MHz is the
  next clean MMCM divider below the spec limit; BRAM read bandwidth (~525 MB/s) still dwarfs the
  ~9 MB/s stream. No firmware/host change (nothing depends on the AXI clock value).

### Verified results

| Metric | `main` baseline | + A | + A + A2 + B | **+ A + A2 + B + C (final)** |
|---|---|---|---|---|
| WNS | −4.551 ns | −2.147 ns | −1.214 ns | **+0.275 ns** |
| Failing endpoints | 23,429 | 1,041 | 16 | **0** |
| Timing verdict | not met | not met | not met | **ALL constraints met** |
| Data-path (intra-84 MHz) | −1.953 ns | −0.100 ns | +0.322 ns | **+0.450 ns** |
| AXI fabric (clk_out2) | −1.647 ns @175 | −0.782 | −1.214 | **+0.275 ns @131.25** |
| 84↔131 CDC crossings | timed → failing | failing | async | **async — not timed** |
| Congestion window | `fifo_bram_inst` 99% | — | none | **none** |
| Slice LUTs | 9,246 (17.4%) | — | 4,556 | **4,500 (8.5%)** |
| Flip-flops | 26,701 (25.1%) | — | 8,715 | **8,715 (8.2%)** |
| F7 / F8 muxes | 2,858 / 1,254 | — | 553 / 116 | **553 / 116** |
| `fifo_bram_inst` | 5,222 LUT / 18,209 FF | — | 542 / 237 | **542 LUT / 237 FF** (368 LUTRAM) |
| Block RAM | 16 / 140 | — | 16 | **16 / 140** |

**Outcome — timing fully closed (WNS +0.275 ns, 0 failing endpoints, "All user specified timing
constraints are met").** The sole congestion hotspot is gone and the design uses **~67% fewer
flip-flops and ~51% fewer LUTs** than the `main` baseline — large headroom for the planned features.

> Verified at build/route level only (no hardware on this machine). A is connectivity, A2/C are
> timing/clocking, and B keeps the FIFO FSM bit-identical (async read preserved) — low functional
> risk — but an on-hardware capture is the recommended final check. Production follow-up for B: a true
> block-RAM FIFO (synchronous read) to also free the LUTRAM — needs a small FSM refactor + testbench.

---

## 1. Headline numbers (original `main` baseline — for reference)

| Metric | Value | Notes |
|---|---|---|
| WNS / TNS | **−4.551 ns / −75,472 ns** | setup; **23,429 / 77,395 endpoints failing** |
| WHS | +0.013 ns | hold met |
| Routing | fully routed, 0 errors, 0 DRC | |
| Slice LUTs | 9,246 / 53,200 (**17.4%**) | |
| Slice Registers (FF) | 26,701 / 106,400 (**25.1%**) | |
| F7 / F8 muxes | 2,858 / 1,254 | wide-mux signature |
| Block RAM | **16 / 140 (11.4%)** | 124 tiles free |
| DSP | 0 / 220 | |
| Congestion windows | 1 (router est., level 5) | **`fifo_bram_inst` = 99%** of the window |

Device occupancy is *low* (17% LUT / 25% FF). The problem is **not** that the design is too big
for the part — it is **where** the logic is and **how** it is reset.

---

## 2. Two root causes (both in / around `fifo_bram_interface`)

### A. Timing failure — PL reset sourced from the wrong clock domain  *(1-line block-design bug)*

The worst path (−4.551 ns) and 22,208 of the 23,429 failing endpoints are all the **same shape**:

```
proc_sys_reset_175MHz/...FDRE_BSR_N_replica  (175 MHz domain)
      → fifo_bram_inst/bram_rst_INST_0 (LUT1) → net rstn_0  (fanout = 21,337)
      → write_fifo_reg[*][*]/R           (84 MHz domain, reset pin)
Requirement: 0.476 ns   (the 175↔84 beat)   Data path: 3.981 ns
```

`clock_interaction.rpt` shows the `clk_out2 (175) → clk_out1 (84)` pair as **"Clean / Timed"** with
**22,208 / 22,208 endpoints failing** at a 0.476 ns requirement — i.e. Vivado is timing these as
ordinary synchronous paths, not CDC. `report_cdc` recognises only 2 trivial reset synchronizers.

**Why:** in `design_1_bd.tcl` (lines 742–747) `data_generator/rstn` is connected to
`proc_sys_reset_175MHz/interconnect_aresetn`. But `data_generator` is clocked **entirely at 84 MHz**
(`clk_out1`). The correct 84 MHz reset (`proc_sys_reset_0_84M/peripheral_aresetn`) is wired only to
`axi_lite_registers/pl_rstn` and the LED controller. So all ~21k reset-pin paths in the data path are
forced across 175→84 at the impossible 0.476 ns beat.

**Fix (cheap, do first):** move `data_generator/rstn` to `proc_sys_reset_0_84M/peripheral_aresetn`.
Timed in-domain, that same net's requirement becomes 11.905 ns vs. ~4  ns delay → ~+7 ns slack.
This alone should clear essentially the entire −4.5 ns failure. (Belt-and-suspenders: also add a
`set_clock_groups -asynchronous` or `set_false_path` between the 84/175 MHz reset crossing.)

### B. Routing congestion + resource bloat — the register-array FIFO  *(the real efficiency win)*

`write_fifo` in `fifo_bram_interface.sv` is declared as a **register array**
(`logic [68:0] write_fifo [0:255]`, 256 × 69 = 17,664 bits) **with a reset loop that clears every
element**. A per-bit reset forces Vivado to infer **flip-flops** (not LUTRAM/BRAM), plus a
**256:1 × 69-bit read mux** (`write_fifo[fifo_read_ptr]`).

Hierarchical utilization (`util_hier.rpt`):

| Scope | LUTs | FFs | share |
|---|---|---|---|
| whole design | 9,246 | 26,701 | — |
| `data_generator` | 6,849 | 21,472 | |
| → **`fifo_bram_inst`** | **5,222** | **18,209** | **56% of LUTs, 68% of all FFs** |
| `data_gen_inst` (core) | 1,623 | 3,263 | |
| `axi_lite_registers` | 474 | 3,387 | |

A 256-deep FIFO that, per the RTL's own comments, only needs ~37 (max packet) entries — implemented as
registers — is **two-thirds of every flip-flop in the design**. Supporting evidence:
- `congestion.rpt`: the single estimated-router congestion window is **`fifo_bram_inst` (99%)** —
  Flop 60% / LUT 29% / **MUXF 27%** (the F7/F8 read mux).
- `high_fanout.rpt`: besides `rstn_0` (21,337), many `fifo_read_ptr_reg[*]_rep__N` / `fifo_write_ptr_reg[*]`
  nets at fanout ~126–140 — the tool **replicated the FIFO pointers** to drive the 256:1 mux.

**Fix:** rebuild `write_fifo` as a **BRAM- (or LUTRAM-) backed FIFO**: a `(* ram_style="block" *)`
memory written/read by pointers, **with no per-element reset** (reset only the pointers/counts — exactly
the pattern `bram.sv` already uses for the 16k×32 capture buffer). Optionally shrink depth to ~64.

Expected effect: FFs ~26,700 → **~9,000**; LUTs ~9,250 → **~4,000**; F7/F8 muxes largely gone;
the 99% congestion window gone; `rstn_0` fanout collapses; cost **~1 BRAM tile** (124 free).

> A and B reinforce each other: B also shrinks the fanout that A is timing. Do **both** — A is the
> correctness fix that's currently *presenting* as a "routing issue"; **B is the efficiency win** that
> buys headroom for the feature work.

---

## 3. Secondary findings (cheaper, do alongside)

1. **CIPO LVDS inputs are unconstrained.** Intra-84 MHz worst path (−1.953 ns) is
   `spi_lvds_0_cipo1_n → cipo1_4x_oversampled_reg`, **92% route delay (13.07 ns), 2 logic levels** — a
   placement/route artifact, not logic depth. `check_timing` flags **8 inputs with `no_input_delay`**.
   These CIPO lines are handled by the 4× oversample + programmable phase selector, so they are
   effectively source-synchronous — add `set_input_delay` or `set_false_path` so STA stops chasing them
   (and the router stops mis-prioritising them). Should clear once congestion (B) also lifts.

2. **`top.xdc` is a 3,211-line auto-generated dump** (a checked-in `write_xdc` of an elaborated design).
   It contains **duplicate `create_clock`** for `clk_fpga_0` (lines 46 & 328) and for `clk_in1`
   (282 & 694), scoped XPM CDC constraints for the smartconnect, and `set_bus_skew`/`src_gray_ff_reg*`
   lines that **match no objects**. This produces the `TIMING-4 / TIMING-27` *invalid primary clock*
   criticals and many `[Common 17-55] set_property expects at least one object` warnings. Replace with a
   small hand-written constraints file; let the BD auto-derive the MMCM output clocks (don't re-`create_clock`
   internal pins).

3. **`intan_io.xdc` / `digital_inputs.xdc` constraint errors** (CRITICAL):
   `DRIVE_STRENGTH`/`SLEW FAST` set on `LVDS_25` (unsupported), `Site cannot be assigned to more than one
   port`, and `Cannot change direction of connected port digital_in_0[*]`. These `set_property` calls are
   silently failing — clean them up so the I/O standard/drive actually applies.

4. **`axi_lite_registers` = 3,387 FFs** for 22 ctrl + 11 status 32-bit regs, because the full 704-bit ctrl
   / 352-bit status busses are double-synchronized and broadcast (`ctrl_sync2[21][31]` net fanout 1,717).
   Fine today; revisit only if the register file grows with features.

5. **Minor:** `sine_lut[0:511]` (debug mode) is read **4× with different shifted indices** → 4 replicated
   distributed ROMs (~hundreds of LUTs). Debug-only; share one port or accept it.

---

## 4. Suggested order of work (before adding features)

1. **A — reset rewire** in the block design (`data_generator/rstn → proc_sys_reset_0_84M`). Rebuild,
   confirm WNS ≥ 0. *(minutes; clears the timing failure)*
2. **B — BRAM/LUTRAM FIFO** in `fifo_bram_interface.sv` (drop the reset loop; optional depth 256→64).
   Rebuild, confirm `fifo_bram_inst` congestion gone and FF/LUT drop. *(the headroom win)*
3. **Constraint hygiene** — new minimal `top.xdc`; `set_input_delay`/`false_path` on CIPO + digital
   inputs; fix LVDS drive/slew + port-direction errors.
4. Re-check the residual intra-175 (−1.647, AXI fabric) and 84→175 (309 ep) violations — expected to
   clear once congestion lifts; if not, constrain the few real 84↔175 crossings explicitly.

**Bottom line:** the part is only ~17% LUT / 25% FF full, but ~68% of the flip-flops and the only
congestion hotspot live in one 256-deep register-array FIFO, and the timing failure is a mis-wired
reset domain on that same module. Fixing those two — both localized to `fifo_bram_interface` / its
reset — closes timing *and* frees roughly two-thirds of the fabric for the upcoming features.
