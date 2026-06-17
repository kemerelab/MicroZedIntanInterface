# DMA implementation notes (distilled from research, 2026-06-16)

Actionable guide for the `claude/dma-cdma` build. Full brief was produced by the
research agent; this is the implementation-critical subset.

## Decision: AXI CDMA, simple polled, BRAM→DDR over S_AXI_HP0
- **AXI CDMA** is the memory-to-memory engine: takes two plain addresses
  (src = BRAM `0x80000000`, dst = DDR buffer), so it drops into the existing
  memory-mapped BRAM path with **no PL restructuring**. (AXI DMA S2MM is better
  only if we convert the PL to AXI-Stream — that's the v2/follow-on.)
- **Port: S_AXI_HP0** (not ACP). HP = highest BW, needs manual cache ops; ACP
  pollutes L1/L2 with single-use stream data and needs custom `AxCACHE=0xF/AxUSER=1`
  (stock DMA ties AxCACHE=0x3 which *corrupts* on ACP). HP is the right call.
- **Polled first** (`XAxiCdma_IsBusy`) to avoid the dual-core GIC distributor
  gotcha (only core0 may init the Distributor). Move to interrupt later if needed.

## Firmware call sequence (CDMA simple, the template is xaxicdma_example_simple_poll.c)
```c
XAxiCdma_Config *cfg = XAxiCdma_LookupConfig(BASEADDR);   // SDT: pass base addr
XAxiCdma_CfgInitialize(&Cdma, cfg, cfg->BaseAddress);
// per packet:
//   src = BRAM_BASE_ADDR + ps_read_address*4  (PL slave, uncached -> no flush)
//   dst = 32B-aligned DDR buffer
XAxiCdma_SimpleTransfer(&Cdma, (UINTPTR)src, (UINTPTR)dst, nbytes, NULL, NULL); // NULL=poll
while (XAxiCdma_IsBusy(&Cdma)) { }
if (XAxiCdma_GetError(&Cdma)) { XAxiCdma_Reset(&Cdma); /* wait reset done */ }
Xil_DCacheInvalidateRange((UINTPTR)dst, nbytes);   // BEFORE the CPU/lwIP reads dst
// then udp_send(dst ...)
```
- `XAxiCdma_SimpleTransfer(cdma, SrcAddr, DstAddr, Length_BYTES, cb, cbRef)`.
- Max BTT per simple xfer = 0x7FFFFF (8 MB-1); our 600 B is trivial.
- CDMA has **no SelfTest** (that's an AXI DMA API only).

## Cache rules (Cortex-A9, 32-byte line) — the #1 corruption source
- Mem-to-mem: **flush source** (only if CPU wrote it; BRAM is a PL slave/uncached
  so skip), **invalidate destination AFTER "not busy", BEFORE reading**.
- Buffers MUST be **32-byte aligned and size a multiple of 32** (`aligned(64)` is a
  safe superset). Never let a DMA buffer share a 32-byte line with other live data
  (partial-line invalidate does clean-then-invalidate and can write stale CPU data
  over DMA'd bytes -> silent corruption).
- **Shortcut:** mark a 1 MB-aligned DDR staging section `NORM_NONCACHE` (0x11DE2)
  via `Xil_SetTlbAttributes(addr, NORM_NONCACHE)` and skip all flush/invalidate.
  `Xil_SetTlbAttributes` granularity = 1 MB. We already do this for SHARED_MEM and
  the BRAM in main.c (NORM_NONCACHE_SHARED). Reuse that pattern for the DMA dest.

## Block-design changes needed (design_1_bd.tcl)
1. Add `xilinx.com:ip:axi_cdma:4.1` (Enable Simple-only / disable SG to start;
   data width 32 to match the BRAM, or 64 for HP; address width >= 32 so it can
   address both 0x80000000 BRAM and 0x00xxxxxx DDR).
2. Enable PS7 **S_AXI_HP0**: `set_property CONFIG.PCW_USE_S_AXI_HP0 {1}` on
   processing_system7_0; connect its `S_AXI_HP0_ACLK` to clk_out2.
3. CDMA **M_AXI** -> a smartconnect -> two slaves: `axi_bram_ctrl_0/S_AXI`
   (BRAM @0x80000000) AND `processing_system7_0/S_AXI_HP0` (DDR). NOTE the BRAM
   controller's S_AXI must be reachable by the CDMA master (shared interconnect) --
   "CDMA can't reach BRAM" is the top forum failure. The PS GP0 read path into
   axi_bram_ctrl can be removed or kept; CDMA becomes the BRAM reader.
4. CDMA **S_AXI_LITE** (control regs) <- PS `M_AXI_GP0` via the existing
   smartconnect_0 (alongside axi_lite_registers). Assign it an address in the GP0
   space.
5. Clocks: CDMA `m_axi_aclk`/`s_axi_lite_aclk` = clk_out2; resets from the matching
   proc_sys_reset. (Watch the reset-domain rule: data path resets from 84 MHz; the
   CDMA/AXI fabric resets from the AXI-domain reset.)
6. Address map: CDMA M_AXI must see DDR (HP0, e.g. 0x00000000-0x3FFFFFFF) and BRAM
   (0x80000000). Assign segments with assign_bd_address.

## Landmines (from the brief)
- BRAM controller S_AXI must share the interconnect the CDMA masters into.
- 32B alignment + size-multiple-of-32 for DMA buffers; don't share cache lines.
- Invalidate dest only AFTER not-busy; never mid-transfer.
- HP AFI: just width/FIFO/QoS, no coherency — manual cache ops required.
- lwIP BSP must be lwip211+ (pbuf cache-invalidate fix). Zero-copy-into-pbuf is a
  v2 optimization (marginal at our rate) -- v1 = DMA into a non-cacheable DDR buffer
  then udp_send.

## v1 plan (get it compiling)
- DMA each packet BRAM -> a non-cacheable DDR staging buffer -> udp_send. CPU never
  touches the BRAM read path => the GP-burst corruption is structurally gone, AND it
  doubles as the diagnostic: if CDMA reads the BRAM through the SAME axi_bram_ctrl
  cleanly, that proves the corruption was the PS GP *master*, not the controller/BRAM
  (=> a custom axi_bram_ctrl would NOT have helped).

## Key example URLs
- xaxicdma_example_simple_poll.c (Xilinx/embeddedsw, axicdma/examples)
- EDT "Using the HP Slave Port with AXI CDMA" (Zynq-7000)
- Hamid-R-Tanhaei/ZYNQ_ADC_DMA_LWIP (PL->DMA->DDR->bare-metal lwIP UDP, closest match)
- AXI CDMA Standalone Driver wiki (xilinx-wiki 18842291)
