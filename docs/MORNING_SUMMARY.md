# Overnight results (read me first) — 2026-06-17

Everything below is built and pushed. Nothing here touches your known-good
`claude/210mhz-single` fallback.

## TL;DR
1. **In-spec `ldmia` read** is built on `claude/dual-port` — test whether it sustains 0xFF.
2. **The DMA data path is built, compiles, closes timing in-spec, and is pushed** on
   `claude/dma-cdma` — the real fix; needs its first hardware test.
3. **Custom `axi_bram_ctrl`: decided NO** (it's the PS GP master, not the controller).
   Backed by a from-scratch read-controller sim that PASSES the failing stimulus.

## Branches to flash + test (in priority order)

| Branch | What | Clock | Status |
|--------|------|-------|--------|
| `claude/dma-cdma` | **AXI CDMA** reads BRAM→DDR; CPU off the read path | 131.25 (in-spec) | **built, compiles, WNS +0.437; UNTESTED on HW** |
| `claude/dual-port` | inline-`ldmia` 10-word CPU bursts | 131.25 (in-spec) | built, WNS +0.133; UNTESTED on HW |
| `claude/210mhz-single` | single-word reads, faster clock | 210 (over-spec) | **verified working** (your fallback) |

Each branch's `blobs/BOOT.bin` is the matching image. Flash flow as always:
`git checkout <branch>` → copy `blobs/BOOT.bin` to the SD `Boot` partition → boot.

## Testing the DMA build (`claude/dma-cdma`) — the headline
1. Flash it. On the **serial console** you should see `CDMA: ready (ctrl 0x44A00000,
   staging 0x10000000)` at boot — that confirms `pl_dma_init()` found the CDMA.
2. `net.py`: `set_debug 1` → `set_channels 0xFF` → `verify_sine 0xFF`.
   - **Success looks like:** `real errors (|d|>1) = 0`, `PACKET LOSS: 0`, full packet
     count (~300/300) — clean 0xFF at full bandwidth, with the CPU no longer doing the
     BRAM read.
   - The bulk read is the CDMA; the magic/resync `Xil_In32` peeks still use the (clean)
     single-beat GP path.
3. If 0xFF is corrupt/garbage or the count is low: the CDMA path has a bug to chase
   (see "If the DMA misbehaves" below). Fall back to `claude/dual-port` or
   `claude/210mhz-single`.

### What the DMA proves
If the CDMA (a clean PL master) reads the BRAM through the *same* `axi_bram_ctrl`
cleanly at 0xFF, that is the final confirmation that the corruption was the PS7
M_AXI_GP **master**, not the controller/BRAM — i.e. a custom controller would not
have helped, and DMA is the right long-term architecture (and frees core 0).

## If the DMA misbehaves (debug pointers)
- `READ_VIA_DMA` in `firmware/src-core0/main.c` (set to 1). Set it to **0** to rebuild
  the *same bitstream* with the inline-`ldmia` CPU read — isolates "is it the CDMA or
  the rest?" (firmware-only rebuild: `vitis -s scripts/build_vitis_project.py`).
- `dma_errors` counter (main.c) increments on CDMA timeout/error. (Not yet surfaced in
  the status struct — quickest check is `verify_sine` correctness + the serial boot line.)
- The CDMA reads completed packets only (the existing guard band keeps the read pointer
  a full packet behind the PL write frontier — no read-during-write).
- Architecture + exact BD changes: `docs/dma_research_notes.md`. The block design edits
  are in `programmable_logic/block_design/design_1_bd.tcl` (commit `e1b3d63`).

## Custom `axi_bram_ctrl` decision
See `docs/custom_axi_bram_ctrl_decision.md`. Short version: the bug is in the hardened
PS7 GP master (long-burst corruption persists even in-spec at 131.25, where timing is
met and the controller is functionally correct), and a slave-side controller can't fix
a master-side problem. A from-scratch clean read controller
(`programmable_logic/sim/axi_read_bram_ctrl.sv`) + stress sim
(`run_axi_read_sim.sh`) PASSES the exact failing scenario (150-beat + back-to-back
bursts, backpressure, non-`i*4` fill, 0 errors), confirming the read logic is not the
fault. **Recommendation: ship DMA, don't build a custom controller.** The controller is
kept only as a verified contingency component.

## Next steps (for when you're back)
- Test the three images; tell me the DMA `verify_sine 0xFF` result.
- If DMA is clean: next is interrupt-mode CDMA (free core 0 fully) and/or zero-copy
  straight into lwIP pbufs (drops the last copy). Both are noted in the research.
- If DMA needs fixes: the most likely first issues are the CDMA address map / HP0
  reachability (forum's #1 failure) — easy to verify by checking the CDMA registers and
  a single test transfer before streaming.
