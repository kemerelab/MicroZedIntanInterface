// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// stream.c
//
// Every PL -> host data stream lives here: how a packet the PL has already
// assembled in a BRAM ring becomes a UDP datagram.
//
// The mechanism is identical for all streams, and is shared below:
//   1. rotate to a fresh staging slot -- the send is zero-copy (PBUF_REF), so
//      reusing a slot whose TX descriptor is still pending would transmit stale
//      bytes and silently lose the earlier packet
//   2. CDMA the packet from the BRAM ring into that slot, split in two if it
//      straddles the ring wrap
//   3. wrap it in a PBUF_REF and udp_sendto, counting the two ways that fails
//      (the shared pbuf pool was momentarily empty, or the TX ring rejected it)
//
// What legitimately differs is the RETRY POLICY, and that stays per stream:
//
//   broadband is the latency-critical archival stream on a ~33 us budget, so it
//   retries IN PLACE, spinning briefly to let the GEM reap descriptors, and only
//   gives up after TX_MAX_ATTEMPTS. It would rather burn cycles than drop.
//
//   LFP is a low-rate background drain, so it does the opposite: on any failure
//   it returns immediately WITHOUT advancing its read pointer and retries the
//   same frame on the next service call. It also caps how many frames it sends
//   per call, so a backlog can never starve the broadband loop.
//
// Both leave the read pointer alone until a packet is actually on the wire, so
// a transient shortage costs latency, never data.

#include "main.h"
#include "pl_dma.h"        // pl_dma_read_addr, the staging buffers
#include "shared_print.h"  // send_message
#include "lwip/udp.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Shared mechanism
// ---------------------------------------------------------------------------

// Copy one packet out of a PL BRAM ring into a staging slot, splitting the
// transfer if it wraps. Returns non-zero on a CDMA error.
int pl_stream_dma_frame(uint32_t *dst, uintptr_t bram_base,
                        uint32_t ring_words, uint32_t read_word,
                        uint32_t n_words)
{
    if ((read_word + n_words) <= ring_words) {
        return pl_dma_read_addr(dst, bram_base + (read_word << 2), n_words);
    }
    uint32_t first = ring_words - read_word;
    int rc  = pl_dma_read_addr(dst,         bram_base + (read_word << 2), first);
    rc     |= pl_dma_read_addr(dst + first, bram_base,                    n_words - first);
    return rc;
}

// Send one already-staged packet zero-copy. Returns ERR_OK, or the lwIP error;
// *alloc_fail is incremented instead if the pbuf pool was empty.
err_t pl_stream_send_frame(struct udp_pcb *pcb, const uint32_t *buf,
                           uint32_t n_bytes, uint32_t *alloc_fail)
{
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, n_bytes, PBUF_REF);
    if (p == NULL) { (*alloc_fail)++; return ERR_MEM; }
    p->payload = (void *)buf;
    ip_addr_t dst; dst.addr = udp_dest_ip;
    err_t e = udp_sendto(pcb, p, &dst, udp_dest_port);
    pbuf_free(p);       // PBUF_REF: releases the reference, not the staging slot
    return e;
}

// ---------------------------------------------------------------------------
// LFP stream (stream_type = 2, 3 kHz)
//
// The PL builds the complete wire packet -- header and samples -- in the LFP
// BRAM and publishes a frame only once its last sample has landed, so anything
// up to lfp_wr_addr is a whole frame and can be sent as-is.
// ---------------------------------------------------------------------------
static struct udp_pcb *lfp_pcb;
static uint32_t        lfp_read_word;
static uint32_t        lfp_staging_slot;
uint32_t               lfp_udp_packets_sent = 0;

// 64 slots x 1 KB, comfortably more than the ~16 sends that can be in flight,
// so a slot is only reused long after its TX-done.
#define LFP_STAGING_SLOT_BYTES 1024u
#define N_LFP_STAGING_SLOTS    64u
#define LFP_FRAMES_PER_CALL    8      // yield to the broadband loop

void lfp_stream_init(void)
{
    lfp_pcb = udp_new();
    lfp_read_word = 0;
    lfp_staging_slot = 0;
    lfp_udp_packets_sent = 0;
    if (lfp_pcb == NULL) send_message("ERROR: Could not create LFP UDP PCB\r\n");
}

void lfp_stream_service(void)
{
    if (!lfp_cfg_enable || lfp_pcb == NULL) return;

    // The lane mask mirrors the broadband channel-enable (single source of
    // truth), so the frame size follows from it.
    int nlanes = __builtin_popcount(pl_get_current_channel_enable() & 0xFF);
    if (nlanes == 0) return;
    uint32_t sample_words = (uint32_t)nlanes * 16;    // popcount*32 samples, 2 per word
    uint32_t frame_words  = UNIFIED_HEADER_WORDS + sample_words;
    const uint32_t mask   = LFP_BRAM_SIZE_WORDS - 1;

    uint32_t wr_word = (pl_lfp_read_status() & 0xFFFF) >> 2;   // byte addr -> word index

    for (int budget = LFP_FRAMES_PER_CALL; budget > 0; budget--) {
        if (((wr_word - lfp_read_word) & mask) < frame_words) break;   // no whole frame yet

        uint32_t *pkt = (uint32_t *)(LFP_DMA_BUF_ADDR
                          + (uintptr_t)lfp_staging_slot * LFP_STAGING_SLOT_BYTES);

        if (pl_stream_dma_frame(pkt, LFP_BRAM_BASE_ADDR, LFP_BRAM_SIZE_WORDS,
                                lfp_read_word, frame_words)) {
            dma_errors++;
            break;                      // retry the SAME frame next call
        }

        err_t e = pl_stream_send_frame(lfp_pcb, pkt, frame_words * 4,
                                       &lfp_pbuf_alloc_fail);
        if (e != ERR_OK) {
            // Do NOT advance: the ring holds 100+ frames of slack, so retrying
            // this frame next call is lossless. (Advancing unconditionally here
            // is what used to turn a transient pool shortage into a dropped
            // frame and a +1 SEQ gap on the host.)
            if (e != ERR_MEM || lfp_pbuf_alloc_fail == 0) {
                lfp_send_err++; lfp_last_send_err = (int32_t)e;
            }
            break;
        }

        lfp_udp_packets_sent++;
        lfp_staging_slot = (lfp_staging_slot + 1u) % N_LFP_STAGING_SLOTS;
        lfp_read_word    = (lfp_read_word + frame_words) & mask;   // commit
    }
}
