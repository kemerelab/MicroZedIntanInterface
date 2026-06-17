# Overnight autonomous plan (2026-06-16 → 17)

Goal for the morning: (1) the in-spec `ldmia` fix tested on `claude/dual-port`,
(2) a clear answer on whether to build our own `axi_bram_ctrl`, and (3) a
**DMA-based data path that compiles and simulates**, on its own branch, ready to
flash and test.

Hard rules held throughout: data fidelity (never alter sample values); commit only
to experiment branches; keep `claude/210mhz-single` (the known-working over-spec
fallback) untouched.

## Where we are (start of night)
- **Working full-bandwidth config exists:** `claude/210mhz-single` (210 MHz +
  single-word reads) — clean at 0xFF, but **over-spec** (WNS −1.354 ns; clk_out2
  also clocks the hardened M_AXI_GP0 master 40% past its ~150 MHz rating). Bench-only.
- **In-spec candidate building now:** `claude/dual-port` with the chunk read changed
  from `memcpy` to inline `ldmia` 10-word bursts (drops per-chunk call/alignment
  overhead; 10 = largest dump_bram-safe burst). If it sustains 0xFF at 131.25 MHz it
  beats 210-single (robust, in-spec).
- **Root-cause sharpening:** corrupt beat data == the byte-address being read →
  read-data pipeline **underrun** (RVALID a cycle ahead of the 1-cycle BRAM), only at
  ≥12-word back-to-back bursts over the AXI3 GP port (~512-byte seam). The BRAM RTL
  (`bram.sv`) is a clean 1-cycle SDP read — not the bug.

## Track 1 — verify the `ldmia` in-spec fix  [serial]
- Confirm the running build closes timing (expect WNS ≈ +0.1 ns at 131.25) and pushes
  `claude/dual-port`. Report. (You test it tomorrow vs 210-single.)

## Track 2 — sim: do we need our own `axi_bram_ctrl`?  [serial, after toolchain free]
- Write a custom **AXI4 read-only BRAM controller** (`axi_read_bram_ctrl.sv`) with a
  proper RVALID/backpressure pipeline (no underrun by construction) + a self-contained
  testbench: 150-beat back-to-back INCR bursts, random RREADY backpressure, BRAM filled
  with a **non-`i*4` pattern** (`i ^ 0xA5A5A5A5`) so any address-echo is unmistakable.
  Proves a correct controller handles the exact failing case in sim.
- Attempt the **vendor** path repro (`axi_bram_ctrl` behavioral sim, same stimulus).
- **Decision:** vendor repros → build the custom controller on `claude/axi-read-ctrl`.
  Vendor clean (expected — functional sim can't model the timing/GP-master effect) →
  document "controller logic is not the fault; it's timing/PS7-GP; go DMA," and do
  **not** sink the night into a custom controller.

## Track 3 — DMA data path (headline)  [serial; informed by research agents]
Architecture (primary = least-invasive, get it compiling first):

- **Option A — AXI CDMA (mem-to-mem), BRAM→DDR.** Add `axi_cdma`; its M_AXI reaches
  both the BRAM (through the existing `axi_bram_ctrl`) and DDR (through a newly enabled
  **S_AXI_HP0**). Firmware: `XAxiCdma_SimpleTransfer(src=BRAM packet, dst=DDR buf, len)`,
  poll done, then UDP-send. **The CPU leaves the BRAM read path entirely** — this is the
  fix regardless of whether the root cause was the GP master or the controller, because
  the BRAM is now read by a well-formed PL master, and the PS only touches cached DDR.
  Chosen for v1 because it reuses the BRAM + controller and the `XAxiCdma` API is small.
- **Option B — AXI DMA S2MM (stream).** Restructure `data_generator` to emit an
  AXI-Stream; AXI DMA writes it to DDR ring buffers; drop the capture BRAM. Cleaner
  long-term, bigger PL change. Note as the follow-on, not tonight.

Port / coherency for v1: **S_AXI_HP0** with a **non-cacheable DDR bounce buffer** (or
`Xil_DCacheInvalidateRange` before read) — simplest, well-exampled. ACP (coherent) and
**zero-copy straight into lwIP pbuf payloads** are the v2 optimization (saves the last
copy); design notes captured but v1 ships a bounce buffer first.

DDR-bandwidth contention with the EMAC is a non-issue (~18 MB/s vs ~4 GB/s).

Steps:
1. BD: add `axi_cdma` (+ interconnect, enable `S_AXI_HP0`), wire BRAM↔CDMA↔HP↔DDR,
   clocks/resets, address map. Build the bitstream. → branch `claude/dma-cdma`.
2. Firmware: `XAxiCdma` init + per-packet `SimpleTransfer` into a DDR buffer, cache
   invalidate, UDP-send; keep the magic/resync logic. Build firmware. **Compile clean.**
3. Sim/validate the CDMA datapath where feasible; document how to flash + test.

## Delegated (background) research/design agents
- **bare-metal Zynq DMA examples** (XAxiDma/XAxiCdma simple+SG, cache discipline,
  pbuf zero-copy, HP vs ACP) — running.
- **block-design CDMA-insertion design** (exact IPs/connections/Tcl to splice CDMA +
  HP0 into `design_1`) — spawned tonight.

## Branches you'll find in the morning
- `claude/dual-port` — in-spec `ldmia` read (test vs 210-single).
- `claude/210mhz-single` — known-working over-spec fallback (unchanged).
- `claude/axi-read-ctrl` — *only if* the sim says the controller is at fault.
- `claude/dma-cdma` — the DMA data path (compiles; sim notes + flash/test instructions).

## Risk / stop-conditions (don't burn the night looping)
- If a build errors, capture the log, fix once, retry once; if still broken, leave it on
  the branch with a NOTE and move to the next track.
- If the vendor-IP sim won't elaborate, skip it — the custom-controller sim + the
  hardware evidence are enough to make the call.
- Get **Option A compiling** before attempting zero-copy/ACP refinements.
