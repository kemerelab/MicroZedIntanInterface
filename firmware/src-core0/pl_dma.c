// AXI CDMA capture-BRAM read path. See pl_dma.h for the rationale.
//
// Topology (design_1_bd.tcl): axi_cdma_0/M_AXI -> smartconnect_1 -> {
//   axi_bram_ctrl_0 (BRAM @0x80000000, read source),
//   processing_system7/S_AXI_HP0 (DDR, write dest) }.
// Control regs (S_AXI_LITE) at CDMA_BASEADDR via M_AXI_GP0/smartconnect_0.

#include "pl_dma.h"
#include "xaxicdma.h"
#include "xil_mmu.h"
#include "xil_cache.h"
#include "xil_types.h"
#include "main.h"          // BRAM_BASE_ADDR
#include "shared_print.h"

// Must match the S_AXI_LITE address assigned in design_1_bd.tcl.
#define CDMA_BASEADDR  0x44A00000U

// OCM staging buffers live at FIXED low-OCM addresses (DMA_BUF_ADDR /
// LFP_DMA_BUF_ADDR in pl_dma.h), not C arrays: the linker leaves low OCM unused
// so no reservation is needed, and fixed addresses are reproducible regardless of
// how the Vitis-generated linker script is laid out. See pl_dma.h for the full
// OCM rationale (kills the CDMA-vs-GEM DDR contention -- the recv->transmit spike).

static XAxiCdma cdma;
static int      cdma_ready = 0;

int pl_dma_init(void) {
    // OCM staging buffers at DMA_BUF_ADDR (0x1000) / LFP_DMA_BUF_ADDR (0x2000),
    // both inside the 1 MB TLB section at 0x0. Flush any cache lines covering the
    // range (the BootROM/FSBL may have touched OCM), THEN mark the section
    // non-cacheable so no stale line can later evict over DMA'd data. One marking
    // covers both buffers (same 1 MB section). After this the DMA path needs no
    // per-packet cache ops.
    Xil_DCacheFlushRange(DMA_BUF_ADDR, 0x2000U);   // 0x1000..0x2FFF: both buffers
    Xil_SetTlbAttributes(DMA_BUF_ADDR, NORM_NONCACHE_SHARED);

    XAxiCdma_Config *cfg = XAxiCdma_LookupConfig(CDMA_BASEADDR);
    if (cfg == NULL) {
        send_message("CDMA: LookupConfig(0x%08lX) failed\r\n",
                     (unsigned long)CDMA_BASEADDR);
        return -1;
    }
    if (XAxiCdma_CfgInitialize(&cdma, cfg, cfg->BaseAddress) != XST_SUCCESS) {
        send_message("CDMA: CfgInitialize failed\r\n");
        return -2;
    }
    // Polled mode (avoids the dual-core GIC distributor init gotcha).
    XAxiCdma_IntrDisable(&cdma, XAXICDMA_XR_IRQ_ALL_MASK);
    cdma_ready = 1;
    // Print both staging addresses so the bench can confirm they landed in OCM
    // (< 0x00030000); if they read as DDR the .ocm_staging placement failed.
    send_message("CDMA: ready (ctrl 0x%08lX, OCM staging bb=0x%08lX lfp=0x%08lX)\r\n",
                 (unsigned long)CDMA_BASEADDR, (unsigned long)DMA_BUF_ADDR,
                 (unsigned long)LFP_DMA_BUF_ADDR);

    // Smoke test: one CDMA transfer BRAM[0]->staging exercises the control path
    // (GP0 -> CDMA regs) and the data path (CDMA -> BRAM read, CDMA -> HP0/DDR
    // write). We only check it completes without error -- it confirms at boot
    // whether the CDMA datapath is wired/addressable before the stream relies on
    // it (the BRAM contents at boot are don't-care).
    {
        int rc = pl_dma_read_bram((uint32_t *)DMA_BUF_ADDR, 0, 8);
        if (rc == 0) send_message("CDMA: self-test OK\r\n");
        else         send_message("CDMA: self-test FAILED (rc=%d)\r\n", rc);
    }
    return 0;
}

// Generic CDMA read: copy n_words from an ARBITRARY PL/BRAM source byte address
// (e.g. the capture BRAM at 0x80000000 or the LFP output BRAM at 0x84000000) to
// dst (must be inside the non-cacheable DMA_BUF_ADDR section). Both source BRAMs
// are in the axi_cdma_0/Data address space (see design_1_bd.tcl).
int pl_dma_read_addr(uint32_t *dst, uintptr_t src_addr, uint32_t n_words) {
    if (!cdma_ready) return -1;

    int     nbytes = (int)(n_words * 4U);
    UINTPTR src    = (UINTPTR)src_addr;

    // BRAM is a PL slave (uncached) and dst is non-cacheable -> no cache ops.
    if (XAxiCdma_SimpleTransfer(&cdma, src, (UINTPTR)dst, nbytes, NULL, NULL)
            != XST_SUCCESS) {
        return -2;
    }

    // Poll for completion (sub-kB transfer is microseconds; guard bounds it).
    uint32_t guard = 0;
    while (XAxiCdma_IsBusy(&cdma)) {
        if (++guard > 100000000U) return -3;   // timeout
    }

    if (XAxiCdma_GetError(&cdma) != 0x0) {
        XAxiCdma_Reset(&cdma);
        while (!XAxiCdma_ResetIsDone(&cdma)) { }
        return -4;
    }
    return 0;
}

int pl_dma_read_bram(uint32_t *dst, uint32_t bram_word_addr, uint32_t n_words) {
    return pl_dma_read_addr(dst, (uintptr_t)(BRAM_BASE_ADDR + bram_word_addr * 4U),
                            n_words);
}
