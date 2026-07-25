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

// Send one already-staged packet zero-copy, LFP policy: a pool shortage and a
// rejected send are both simply "not now, retry the same frame next call".
static err_t lfp_send_frame(struct udp_pcb *pcb, const uint32_t *buf,
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


// The capture-BRAM read method is selected in main.h (both this reader and
// main()'s CDMA init depend on it).

static int n_words_available;

// ---------------------------------------------------------------------------
// Broadband stream (stream_type = 1, 30 kHz)
//
// The latency-critical one: one packet per 30 kHz sample, on a ~33 us budget.
// It retries a rejected send IN PLACE rather than dropping, because the stream
// is archival and a dropped packet cannot be recovered, and it carries the
// timing instrumentation that measures that budget.
//
// It does NOT share the send path with the LFP stream. The two count failures
// differently on purpose: broadband separates "the pbuf pool was momentarily
// empty" from "the TX ring rejected it", because they call for different
// responses, and a shared helper that returned one error would throw that away.
// ---------------------------------------------------------------------------
// Pre-allocated packet buffer for UDP (sized for maximum packet)
// Use __attribute__((aligned(64))) to align to cache line boundary for optimal performance
// Used as the packet buffer only on the BRAM_READ_SINGLE path; unused under DMA.
static uint32_t udp_packet_buffer[MAX_WORDS_PER_PACKET] __attribute__((aligned(64), unused));

static int packets_available(void) {
  uint32_t pl_write_addr = pl_get_bram_write_address();

  if (pl_write_addr >= ps_read_address) {
    n_words_available = pl_write_addr - ps_read_address;
  } else {
    // Handle wrap-around
    n_words_available = (BRAM_SIZE_WORDS - ps_read_address) + pl_write_addr;
  }

  // No guard band: the exposed write pointer (packet_boundary_address in
  // fifo_bram_interface.sv) advances ONLY at packet boundaries, so every packet
  // in [ps_read_address, pl_write_addr) is already fully committed -- the
  // in-progress packet is excluded by construction, so there is no
  // read-during-write to guard against. The CDC on that pointer is handled by the
  // read-twice deglitch in pl_get_bram_write_address(), and the per-packet magic
  // check is the safety net. (A former one-packet guard band here was a
  // misattributed band-aid for the M_AXI_GP burst corruption that the DMA fixed;
  // it also held back the last packet of any finite loop_count, so loop_count=1
  // streamed nothing.)
  return n_words_available / current_packet_size;  // complete packets available
}


// Record a broadband TX drop (pbuf-alloc fail or udp_sendto error). packets_
// received_count is the count BEFORE this packet's increment, so the dropped
// packet is ~that index. first/last bracket the span; the ring shows clustering.
static void record_bb_drop(void) {
  uint32_t idx = packets_received_count;
  if (first_drop_pkt == 0) first_drop_pkt = idx;
  last_drop_pkt = idx;
  drop_ring[drop_ring_idx & 7u] = idx;
  drop_ring_idx++;
}

// Read and validate one packet directly from BRAM with UDP transmission
static int process_packet_from_bram(void) {
  XTime t_loop0; XTime_GetTime(&t_loop0);   // perf: receive->transmit timer
  // Unified packet format: header word 0 = MAGIC (0xCAFEBABE), word 1 = TYPE_VER
  // with stream_type=1 (broadband), version=1 in the low 16 bits. The capture
  // BRAM only ever holds broadband packets (the LFP stream lives in its own
  // BRAM), so we validate both the magic AND the broadband stream_type/version.
  uint32_t magic_offset    = ps_read_address; // should always be < BRAM_SIZE_WORDS
  uint32_t typever_offset  = (ps_read_address + 1) % BRAM_SIZE_WORDS;

  uint32_t magic_word   = Xil_In32(BRAM_BASE_ADDR + (magic_offset * 4));   // DMA-EXEMPT: 2-word header peek (clean 1-beat reads; bulk payload moves by CDMA below)
  uint32_t typever_word = Xil_In32(BRAM_BASE_ADDR + (typever_offset * 4)); // DMA-EXEMPT: 2-word header peek (clean 1-beat reads; bulk payload moves by CDMA below)

  uint32_t expected_typever =
      (uint32_t)STREAM_TYPE_BROADBAND | ((uint32_t)UNIFIED_VERSION << 8);

  // Validate the unified header (magic + broadband type/version, low 16 bits)
  if (magic_word != UNIFIED_MAGIC ||
      (typever_word & 0xFFFFu) != (expected_typever & 0xFFFFu)) {
    // Invalid header - could be BRAM overflow, corruption, or misalignment.
    // Jump directly to write pointer to sync with fresh data.
    uint32_t pl_write_addr = pl_get_bram_write_address();
    ps_read_address = pl_write_addr;
    error_count++; // ERROR TO TRACK
    send_message("Header validation failed (magic=0x%08X type_ver=0x%08X), jumping to write position %u\r\n",
                 magic_word, typever_word, pl_write_addr);
    return 0; // Packet validation failed, now synced to fresh data
  }

  // TODO: If we are in an error state, we could track how long we stay there
  //    by measuring the timestamp gap when we recover.

  // UDP transmission (always enabled) - zero-copy with pre-allocated buffer.
  //
  // Read the packet out of the capture BRAM into pkt_buf (see "read method"
  // above). DMA: the CDMA copies BRAM -> a non-cacheable DDR buffer, split at
  // the BRAM wrap into two contiguous transfers. SINGLE: clean but slow
  // word-by-word Xil_In32 (the conceptual reference / 210 MHz fallback).
  uint32_t *pkt_buf;
#if BRAM_READ_METHOD == BRAM_READ_DMA
  // Staging RING: the send is zero-copy (PBUF_REF), so rotate the staging slot to
  // avoid clobbering a slot whose TX BD is still pending (see the broadband no-loss
  // notes). 128 * 2 KB = 256 KB inside the 1 MB pl_dma_staging.
  #define STAGING_SLOT_BYTES 2048u
  #define N_STAGING_SLOTS    128u
  pkt_buf = (uint32_t *)(DMA_BUF_ADDR + (uintptr_t)staging_slot * STAGING_SLOT_BYTES);
  staging_slot = (staging_slot + 1u) % N_STAGING_SLOTS;
  int derr;
  XTime t_dma0; XTime_GetTime(&t_dma0);     // perf: CDMA transfer timer
  derr = pl_stream_dma_frame(pkt_buf, BRAM_BASE_ADDR, BRAM_SIZE_WORDS,
                             ps_read_address, current_packet_size);
  XTime t_dma1; XTime_GetTime(&t_dma1);
  dma_ticks_last = (uint32_t)(t_dma1 - t_dma0);
  if (dma_ticks_last > dma_ticks_max) dma_ticks_max = dma_ticks_last;
  if (derr) dma_errors++;
#else  // BRAM_READ_SINGLE -- clean 1-beat reads, but too slow for 0xFF at 131 MHz
  pkt_buf = udp_packet_buffer;
  for (uint32_t i = 0; i < current_packet_size; i++) {
    uint32_t src = (ps_read_address + i) % BRAM_SIZE_WORDS;
    pkt_buf[i] = Xil_In32(BRAM_BASE_ADDR + src * 4);  // DMA-EXEMPT: BRAM_READ_SINGLE reference reader (compile-time fallback, not the default DMA path)
  }
#endif

  // NO-LOSS bounded retry (broadband is archival): retry the send instead of
  // dropping. udp_sendto returns ERR_MEM on a transient TX-BD-ring-full (the GEM
  // reaps lazily, no TX-done ISR); the ring drains autonomously and each udp_sendto
  // reaps completed BDs, so a fresh attempt recovers the packet. Bounded so a
  // sustained stall degrades to a drop; the staging ring + ~100-packet PL BRAM
  // absorb the backlog. (PAUSErx=0/TXSR=TXGO confirmed these drops are benign
  // transient ring-full, not flow control or a TX error.)
  #define TX_MAX_ATTEMPTS 64
  uint32_t packet_bytes = current_packet_size * BYTES_PER_WORD;
  ip_addr_t dest_ip;
  dest_ip.addr = udp_dest_ip;

  XTime t_send0; XTime_GetTime(&t_send0);
  err_t result = ERR_MEM;
  uint32_t attempt = 0;
  for (; attempt < TX_MAX_ATTEMPTS; attempt++) {
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, packet_bytes, PBUF_REF);
    if (p != NULL) {
      p->payload = (void*)pkt_buf;
      result = udp_sendto(udp, p, &dest_ip, udp_dest_port);
      pbuf_free(p);
      if (result == ERR_OK) break;
      bb_send_retries++;
    } else {
      bb_pbuf_retries++;
    }
    for (volatile int s = 0; s < 120; s++) { }   // let the GEM make TX progress
  }
  XTime t_send1; XTime_GetTime(&t_send1);
  send_ticks_last = (uint32_t)(t_send1 - t_send0);
  if (send_ticks_last > send_ticks_max) send_ticks_max = send_ticks_last;

  if (result == ERR_OK) {
    udp_packets_sent++;
    if (attempt > 0) bb_send_recovered++;   // needed >=1 retry but got through (no loss)
  } else {
    send_message("UDP Send Error: %d (after %u retries)\r\n", result, (unsigned)attempt);
    udp_send_errors++;
    bb_send_err++;
    bb_last_send_err = (int32_t)result;
    record_bb_drop();
  }

  // Update read pointer with variable packet size
  ps_read_address = (ps_read_address + current_packet_size) % BRAM_SIZE_WORDS;
  packets_received_count++;

  // perf: full receive->transmit time for this packet (the 33us-budget metric)
  XTime t_loop1; XTime_GetTime(&t_loop1);
  loop_ticks_last = (uint32_t)(t_loop1 - t_loop0);

  // perf: worst-case capture -- snapshot the breakdown the instant a new max is
  // set, so we see WHAT dominated the worst packet (cdma vs send vs other).
  if (loop_ticks_last > loop_ticks_max) {
    loop_ticks_max = loop_ticks_last;
    worst_pkt_index   = packets_received_count;
    worst_cdma_ticks  = dma_ticks_last;
    worst_send_ticks  = send_ticks_last;
    // other = loop - cdma - send (clamp; the three samples are taken at slightly
    // different instants so rounding can make the sum momentarily exceed loop)
    uint32_t accounted = dma_ticks_last + send_ticks_last;
    worst_other_ticks = (loop_ticks_last > accounted) ? (loop_ticks_last - accounted) : 0;
  }

  // perf: distribution + over-budget frequency. Convert this packet's
  // recv->transmit ticks to microseconds against the histogram edges. The 33.3 us
  // budget is one sample period at 30 kHz.
  if (perf_timer_hz) {
    uint32_t loop_us = (uint32_t)(((uint64_t)loop_ticks_last * 1000000ULL) / perf_timer_hz);
    int b;
    if      (loop_us <  16) b = 0;
    else if (loop_us <  25) b = 1;
    else if (loop_us <  33) b = 2;
    else if (loop_us <  50) b = 3;
    else if (loop_us < 100) b = 4;
    else                    b = 5;
    loop_hist[b]++;
    if (loop_us >= 33) over_budget_count++;   // 33.3 us budget; >=33 us is over
  }

  return 1;  // Success
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
// ONE frame per call. Eight was a burst of eight CDMA + udp_sendto pairs in a
// single visit -- comfortably longer than one 33 us broadband sample period, so
// it stalled the pump that must never stall. At 3 kHz the loop comes back around
// far faster than frames appear, so one per visit still drains it easily.
#define LFP_FRAMES_PER_CALL    1

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

        err_t e = lfp_send_frame(lfp_pcb, pkt, frame_words * 4,
                                 &lfp_pbuf_alloc_fail);
        if (e != ERR_OK) {
            // Do NOT advance: the ring holds 100+ frames of slack, so retrying
            // this frame next call is lossless. Advancing unconditionally would
            // turn a transient pool shortage into a dropped frame and a +1 SEQ
            // gap on the host.
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

// Drain every packet the PL has finished, then return to the caller's loop.
//
// The drain loop lives HERE, in the same translation unit as the two functions
// it calls, and that placement is load-bearing rather than cosmetic. Both are
// static, so the compiler can inline them into this loop and keep the read
// pointer and packet size in registers across iterations. When this loop sat in
// main.c and called across to here, every packet paid a real call plus a reload
// of each global the compiler could no longer reason about -- several
// microseconds out of a 33 us budget, enough that the PS fell behind the PL,
// back-pressured it through fifo_full, and dragged the acquisition rate itself
// below 30 kHz. Keeping the loop with its callees costs main() one call per
// outer iteration instead of one per packet.
void broadband_stream_service(void)
{
    if (!stream_enabled) return;
    while (packets_available() > 0) {
        process_packet_from_bram();
        if (packets_received_count % 30000 == 0) {
            send_message("Processed %u packets, %u errors, %u nwa, UDP: %u sent/%u errors\r\n",
                         packets_received_count, error_count, n_words_available,
                         udp_packets_sent, udp_send_errors);
        }
    }
}
