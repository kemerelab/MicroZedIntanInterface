---
name: check-dma
description: Verify PL→PS data moves by DMA, not single-beat Xil_In32/staging loops. Run before declaring any PL↔PS data-transfer task done in the MicroZedIntanInterface firmware.
---

# check-dma

Guardrail for the PL→PS data path. **Rule:** bulk PL→PS data MUST move by AXI CDMA,
landed straight into the pbuf payload. The cleanest form is: the **PL builds the whole
wire packet (header + payload) in its result BRAM**, and the PS just DMAs it into the pbuf
and sends — exactly as the broadband path does (the PL writes the 10-word header in
`data_generator_core.sv`). The PS must **never** loop over BRAM (`Xil_In32`) or over the
DMA staging buffer to move bulk samples.

## How to run
```
bash scripts/check_dma.sh [firmware_dir]      # default: firmware
```
It flags two single-beat patterns and exits non-zero on any un-justified one:
- **A — single-beat BRAM:** `Xil_In32`/`Xil_Out32` of a `*BRAM*` address.
- **B — staging CPU read:** `= ...staging...[...]` (the PS reading the DMA'd copy word-by-word).

Isolated control/status-register accesses (`PL_CTRL_BASE_ADDR + CTRL_REG/STATUS_REG`) are
**not** flagged — those are small config, not bulk data.

## Resolving a VIOLATION
1. **Preferred:** convert it to DMA-into-pbuf (or have the PL build the packet in BRAM and
   just DMA+send it). Then it stops matching and the check passes.
2. **If genuinely justified** (e.g. a 2-word magic/resync peek, or the single-beat reference
   reader behind `BRAM_READ_METHOD`): annotate the **same line** with
   `// DMA-EXEMPT: <reason>`. The check then reports it as `exempt` and passes.

## When to run
**Before telling the user any PL↔PS data-transfer change is done.** If you added a new
single-beat site, you must convert it to DMA or justify it with a `DMA-EXEMPT` annotation —
don't ship an un-justified single-beat bulk transfer.
