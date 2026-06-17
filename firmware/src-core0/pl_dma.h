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
// DMA_BUF_ADDR: a 1 MB-aligned DDR staging buffer the CDMA writes into. It sits
// far above the core-0 image (0x100000), core-1 (0x20000000) and the shared
// region (0x3F000000). pl_dma_init() marks its 1 MB section NORM_NONCACHE, so no
// cache flush/invalidate is needed and the EMAC TX (which flushes the pbuf
// payload) reads the freshly DMA'd bytes rather than a stale cache line.
#define DMA_BUF_ADDR   0x10000000U

// Initialize the AXI CDMA (polled mode) and mark the staging buffer
// non-cacheable. Returns 0 on success, negative on failure.
int pl_dma_init(void);

// Copy n_words 32-bit words from the capture BRAM (word offset bram_word_addr)
// to dst (must be inside the non-cacheable DMA_BUF_ADDR section) via the CDMA.
// Blocks until the transfer completes. Returns 0 on success, negative on
// error/timeout.
int pl_dma_read_bram(uint32_t *dst, uint32_t bram_word_addr, uint32_t n_words);

#endif // PL_DMA_H
