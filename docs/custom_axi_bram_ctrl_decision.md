# Should we build our own `axi_bram_ctrl`? — decision

**Short answer: No.** The 0xFF burst corruption is in the PS7 **M_AXI_GP master**,
not in the BRAM controller's read logic, and no slave-side controller can fix a
master-side problem. The robust fix is the **AXI DMA** path (PL master), which
also *proves* this. A from-scratch controller + stress sim
(`programmable_logic/sim/`) is included as supporting evidence and as a ready
component should the conclusion ever change.

## Why it is the master, not the controller (the evidence chain)

1. **In-spec, timing-met, still corrupts.** At 131.25 MHz the AXI fabric closes
   timing (WNS +0.13 ns; the BRAM read path itself is +2.36 ns) yet long bursts
   still corrupt. A timing-marginal controller would be fixed by meeting timing;
   this isn't.
2. **Single beats are always clean; only long bursts corrupt.** A functional bug
   in `axi_bram_ctrl` (or the BRAM) would corrupt single reads too. It doesn't.
   The variable is *burst length over the GP port*, not the controller.
3. **Reproducible on a STOPPED board.** `dump_bram` shows `memcpy` (AXI burst)
   garbage where `Xil_In32` (single beats) reads the same addresses correctly —
   no PL writing involved, so it is the read *transaction*, not RDW or the BRAM.
4. **The corrupt value equals the address being read** (`0x0404` @ word 257 =
   257*4) — a read-data pipeline underrun where the bus carries an address-phase
   value. The BRAM's `mem[i]=i*4` init masks this; the zero-init vendor-BRAM test
   still returned `0x0404`, confirming it is the address, not memory.
5. **The BRAM RTL is clean.** `bram.sv` is a textbook 1-cycle registered SDP read
   that matches `axi_bram_ctrl`'s expected latency — not the bug.

A custom `axi_bram_ctrl` is a **slave**. Items 1–5 point at the hardened PS7 GP
**master**'s long-burst behaviour (it is over its ~150 MHz spec only at 175/210,
but corrupts even at the in-spec 131.25). A new slave controller cannot change how
the master drives/over-drives bursts.

## The decisive experiment is the DMA build itself

`claude/dma-cdma` makes an **AXI CDMA (a PL master)** read the BRAM through the
*same* `axi_bram_ctrl` and write DDR over S_AXI_HP0. If that reads 0xFF cleanly,
it proves the controller + BRAM were always fine and the PS GP master was the
fault — i.e. a custom controller would not have helped. So the DMA work both
*fixes* the bug and *answers* this question; building a custom controller in
parallel would be redundant.

## The sim (supporting evidence)

`programmable_logic/sim/run_axi_read_sim.sh` drives a from-scratch clean AXI read
controller (`axi_read_bram_ctrl.sv`, RVALID gated by a backpressure FIFO so it can
never underrun) against the real `bram.sv`, with the exact failing stimulus:
150-beat and back-to-back 16-beat INCR bursts, random RREADY backpressure, BRAM
filled with a **non-`i*4` pattern** (`i ^ 0xA5A5A5A5`) so any address-echo is
caught. A clean controller passing this shows the read *logic* is not where the
corruption must come from.

**Sim result (2026-06-17):** `RESULT: PASS -- bursts=14 beats_checked=523
rlast_count=14 errors=0`. The clean controller returned every beat correctly:
the 150-beat 0xFF-sized burst, ten back-to-back 16-beat bursts (the AXI3-GP
chunk pattern), and assorted lengths (12-beat threshold, single beat, 200-beat),
all under random RREADY backpressure, against the non-`i*4` fill. Zero
mismatches — no address-echo, no dropped/duplicated beats. So correct read logic
handles the exact failing stimulus; the corruption must originate upstream of the
controller (the PS7 GP master). Re-run with `programmable_logic/sim/run_axi_read_sim.sh`.

## Recommendation
Ship the DMA path. Do **not** invest in a custom `axi_bram_ctrl`. Keep
`axi_read_bram_ctrl.sv` as a verified component only as a contingency (e.g. if a
future need arises to own the read path for a non-DMA reason).
