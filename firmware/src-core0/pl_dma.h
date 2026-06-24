#ifndef PL_DMA_H
#define PL_DMA_H

#include <stdint.h>

// AXI CDMA based capture-BRAM read path.
//
// The PS M_AXI_GP master corrupts long burst reads of the capture BRAM (the
// 0xFF dual-port dropout). The CDMA reads the BRAM as a well-formed PL master
// and writes each packet to a DDR staging buffer over S_AXI_HP0, so the CPU GP
// port leaves the bulk-read path entirely.
//
// OCM staging buffers at FIXED low-OCM addresses. The CDMA writes each packet
// here and the GEM TX DMA then reads it (zero-copy PBUF_REF). The linker leaves
// the low 192 KB OCM (region ps7_ram_0 @ 0x0) entirely unused -- every firmware
// section maps to DDR -- so these fixed addresses are collision-free WITHOUT a
// linker reservation, board-independent (OCM-low is 0x0..0x2FFFF on every
// Zynq-7000 part), and reproducible regardless of how the Vitis-generated linker
// script is laid out. We deliberately avoid 0x0 (a NULL pointer -O3 may treat as
// unreachable); the CPU never dereferences these anyway -- the CDMA writes them
// over S_AXI_HP0 (the HP0_DDR_LOWOCM segment reaches low OCM) and the GEM TX DMA
// reads them.
//
// Keeping BOTH the CDMA write dest and the Ethernet read source in OCM keeps the
// whole PL->PS->wire data path OFF the DDR controller, so CDMA writes and GEM
// reads no longer contend on DDR (the measured recv->transmit spike: the
// broadband CDMA stalling 4 -> 44 us once a 2nd stream loads DDR). OCM has no
// refresh and far lower latency, so the two ~20 MB/s flows no longer collide.
// pl_dma_init() marks the 0x0 1 MB TLB section non-cacheable -> no per-packet
// cache ops. Each buffer holds one packet (broadband <= 600 B, LFP <= 536 B);
// they sit 4 KB apart inside that single non-cacheable section.
#define DMA_BUF_ADDR       0x00001000U   // broadband staging (low OCM)
#define LFP_DMA_BUF_ADDR   0x00002000U   // LFP staging (low OCM)

// Initialize the AXI CDMA (polled mode) and mark the staging buffer
// non-cacheable. Returns 0 on success, negative on failure.
int pl_dma_init(void);

// Copy n_words 32-bit words from the capture BRAM (word offset bram_word_addr)
// to dst (must be inside the non-cacheable DMA_BUF_ADDR section) via the CDMA.
// Blocks until the transfer completes. Returns 0 on success, negative on
// error/timeout.
int pl_dma_read_bram(uint32_t *dst, uint32_t bram_word_addr, uint32_t n_words);

// Generic CDMA read from an ARBITRARY PL/BRAM source byte address (e.g. the LFP
// output BRAM at 0x84000000) to dst (must be inside DMA_BUF_ADDR). Same blocking
// semantics/return codes as pl_dma_read_bram. The source must be in the
// axi_cdma_0/Data address space (see design_1_bd.tcl).
int pl_dma_read_addr(uint32_t *dst, uintptr_t src_addr, uint32_t n_words);

#endif // PL_DMA_H
