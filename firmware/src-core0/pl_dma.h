// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

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
// pl_dma_staging: the DDR staging buffer the CDMA writes each packet into. It is
// a LINKER-RESERVED static array (1 MB, 1 MB-aligned), NOT a hardcoded address --
// so the linker places it inside core-0's own DDR region. That means it can never
// collide with core-0 code/data/heap/stack, and it is portable across boards with
// different DDR sizes (no assumption about where DDR ends -- important for, e.g.,
// 7010 parts). pl_dma_init() marks its 1 MB section NORM_NONCACHE (the cache
// attribute granularity is 1 MB, hence the 1 MB size + alignment so the buffer
// owns its whole section); the DMA path then needs no per-packet cache ops and
// the EMAC TX transmits the freshly DMA'd bytes directly.
extern uint8_t pl_dma_staging[];
#define DMA_BUF_ADDR   ((uintptr_t)pl_dma_staging)

// Initialize the AXI CDMA (polled mode) and mark the staging buffer
// non-cacheable. Returns 0 on success, negative on failure.
int pl_dma_init(void);

// Copy n_words 32-bit words from the capture BRAM (word offset bram_word_addr)
// to dst (must be inside the non-cacheable DMA_BUF_ADDR section) via the CDMA.
// Blocks until the transfer completes. Returns 0 on success, negative on
// error/timeout.
int pl_dma_read_bram(uint32_t *dst, uint32_t bram_word_addr, uint32_t n_words);

// Generic CDMA read from a PL/BRAM source byte address to dst (must be inside
// DMA_BUF_ADDR). Same blocking semantics/return codes as pl_dma_read_bram. The
// source must be in the axi_cdma_0/Data address space (see design_1_bd.tcl).
int pl_dma_read_addr(uint32_t *dst, uintptr_t src_addr, uint32_t n_words);

#endif // PL_DMA_H
