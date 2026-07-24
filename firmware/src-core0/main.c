// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#include "main.h"
#include "platform.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include "xil_io.h"
#include "xil_cache.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/etharp.h"   // etharp_gratuitous() -- proactively refresh host ARP
//#include "xuartps.h"
#include "shared_print.h"
#include "pl_dma.h"
#include "xiltimer.h"  // XTime_GetTime / COUNTS_PER_SECOND for perf instrumentation

// Forward declare eth_link_detect from xemacpsif adapter
// This function is provided by the LWIP library's Xilinx EMAC adapter
extern void eth_link_detect(struct netif *netif);


// Global variables
XTimer timer;
struct netif server_netif;
struct udp_pcb *udp;
volatile int stream_enabled = 0;
uint32_t packets_received_count = 0;


// Link state tracking for hotplug support
volatile int link_is_up = 0;

// Command flags for main loop processing
volatile int enable_streaming_flag = 0;
volatile int disable_streaming_flag = 0;
volatile int reset_timestamp_flag = 0;
volatile int cable_test_flag = 0;

// BRAM state tracking
uint32_t ps_read_address = 0;              // Current PS read position (word address)
uint32_t staging_slot = 0;                 // rotating DDR staging-ring slot (zero-copy TX aliasing fix)
uint32_t current_packet_size = 84;         // 14-word header + 70 data words (0x0F default); recomputed on start
uint32_t current_channel_enable = 0x0F;    // Current channel enable setting (default all channels)

// Packet validation tracking
uint32_t error_count = 0;
uint32_t dma_errors = 0;   // CDMA read failures (BRAM_READ_DMA path)

// Performance instrumentation, observable via get_status. We store raw global-
// timer TICKS at full resolution and let the host convert to microseconds using
// perf_timer_hz (sent in the status) -- store the measurement, derive the
// display. The 30 kHz sample rate gives a 33.3 us budget per packet; loop_ticks
// is the receive->transmit time and dma_ticks is the CDMA transfer within it.
uint32_t dma_ticks_last = 0, dma_ticks_max = 0;     // CDMA transfer (ticks)
uint32_t loop_ticks_last = 0, loop_ticks_max = 0;   // receive->transmit (ticks)
uint32_t perf_timer_hz = 0;                         // tick freq (set in main())
// recv->transmit spike instrumentation. Split the recv->transmit window into the
// CDMA, udp_sendto, and "other" components so the host can attribute the
// occasional ~40 us spike; capture the worst packet's breakdown and a histogram
// + over-budget count for the tail's shape/frequency. Cleared by perf_reset().
uint32_t send_ticks_last = 0, send_ticks_max = 0;   // udp_sendto() call (ticks)
uint32_t over_budget_count = 0;                     // packets over the 33.3 us budget
uint32_t worst_pkt_index = 0;                       // packet idx of the worst loop
uint32_t worst_cdma_ticks = 0, worst_send_ticks = 0, worst_other_ticks = 0;
uint32_t loop_hist[PERF_HIST_BUCKETS] = {0};        // recv->transmit time distribution

// TX drop diagnostics (v1.6): split udp_send_errors by failure mode and record
// WHEN drops happen. Each zero-copy PBUF_REF send holds one MEMP_PBUF entry
// (MEMP_NUM_PBUF) until the GEM TX-done reaps it; pbuf_alloc()==NULL => that pool
// is momentarily empty. Declared before perf_reset() so it can clear them.
// Cleared by CMD_PERF_RESET.
uint32_t bb_pbuf_alloc_fail = 0, bb_send_err = 0;
int32_t  bb_last_send_err = 0;
// NO-LOSS retry stats (bb_send_err now = drops after exhausting all retries).
uint32_t bb_send_retries = 0, bb_pbuf_retries = 0, bb_send_recovered = 0;
uint32_t first_drop_pkt = 0, last_drop_pkt = 0;
uint32_t drop_ring[8] = {0};
uint32_t drop_ring_idx = 0;
// If this fails, the wire layout changed -- update net.py get_status (the length
// check and the struct.unpack offsets) to match.
_Static_assert(sizeof(status_response_t) == 264, "status_response_t size must match net.py get_status");

// Clear the sticky maxes + worst-case snapshot + histogram + counts so the user
// controls the measurement window (CMD_PERF_RESET). Leaves the last-sample fields
// and dma_errors alone -- those self-refresh / are lifetime counters.
void perf_reset(void) {
  dma_ticks_max = 0;
  loop_ticks_max = 0;
  send_ticks_max = 0;
  over_budget_count = 0;
  worst_pkt_index = 0;
  worst_cdma_ticks = worst_send_ticks = worst_other_ticks = 0;
  for (int i = 0; i < PERF_HIST_BUCKETS; i++) loop_hist[i] = 0;
  // TX drop diagnostics
  bb_pbuf_alloc_fail = bb_send_err = 0; bb_last_send_err = 0;
  first_drop_pkt = last_drop_pkt = 0; drop_ring_idx = 0;
  for (int i = 0; i < 8; i++) drop_ring[i] = 0;
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

// UDP transmission
uint32_t udp_packets_sent = 0;
uint32_t udp_send_errors = 0;
// UDP configuration (can be changed via TCP command)
uint32_t udp_dest_ip = 0;      // Will be initialized in main()
uint16_t udp_dest_port = DEFAULT_UDP_DEST_PORT;

// Pre-allocated packet buffer for UDP (sized for maximum packet)
// Use __attribute__((aligned(64))) to align to cache line boundary for optimal performance
// Used as the packet buffer only on the BRAM_READ_SINGLE path; unused under DMA.
static uint32_t udp_packet_buffer[MAX_WORDS_PER_PACKET] __attribute__((aligned(64), unused));

// ---- Capture-BRAM read method ----------------------------------------------
// The PS M_AXI_GP master corrupts long *burst* reads of the capture BRAM (the
// 0xFF dual-port dropout). What we learned chasing it:
//   SINGLE - word-by-word Xil_In32. CLEAN by construction: each read is its own
//            1-beat AXI transaction, so it never issues the burst that the GP
//            master mishandles. But it is latency-bound and too slow to sustain
//            0xFF (256 ch, 150-word packets) at the 131.25 MHz AXI clock.
//            Raising the AXI clock to 210 MHz DID make single-beat fast enough --
//            but 210 MHz is over the -1 part's M_AXI_GP ~150 MHz spec
//            (clk_out2 also clocks the GP master), so it is bench-only, not
//            shippable. Kept here as the conceptual reference / fallback.
//   DMA    - an AXI CDMA (a PL master) copies each packet BRAM -> DDR over
//            S_AXI_HP0 (see pl_dma.c), taking the PS GP master off the bulk-read
//            path entirely. Clean at full bandwidth, in-spec at 131.25 MHz, and
//            it frees core 0. This is the fix. DEFAULT.
// (A removed third option, inline-`ldmia` chunked CPU bursts, was a burst over
//  the same broken GP master AND its asm scrambled word order -> wrong magic;
//  see git history. Don't reintroduce CPU bursts of the BRAM.)
#define BRAM_READ_DMA     0
#define BRAM_READ_SINGLE  1
#define BRAM_READ_METHOD  BRAM_READ_DMA

// ============================================================================
// PACKET SIZE CALCULATION FUNCTIONS
// ============================================================================

uint32_t calculate_data_words(int channel_enable) {
    int num_channels = 0;
    
    // Count enabled 16-bit streams across both ports (8-bit mask)
    for (int b = 0; b < 8; b++)
        if (channel_enable & (1 << b)) num_channels++;

    if (num_channels == 0) {
        send_message("WARNING: No channels enabled, defaulting to port-0 all channels\r\n");
        return 70; // Default to 4 streams (port 0) x 35 cycles / 2
    }
    
    // Calculate 32-bit words needed for the data
    // Each cycle produces num_channels × 16-bit words
    // Total 16-bit words = 35 × num_channels
    // Convert to 32-bit words with proper rounding up
    uint32_t total_16bit_words = 35 * num_channels;
    uint32_t data_32bit_words = (total_16bit_words + 1) / 2;  // Round up division
    
    return data_32bit_words;
}

uint32_t calculate_packet_size(int channel_enable) {
    return PACKET_HEADER_WORDS + calculate_data_words(channel_enable);
}

void update_current_packet_size(void) {
    uint32_t new_channel_enable = pl_get_current_channel_enable();
    
    if (new_channel_enable != current_channel_enable) {
        current_channel_enable = new_channel_enable;
        current_packet_size = calculate_packet_size(current_channel_enable);
        
        send_message("Updated packet size: channel_enable=0x%X, packet_size=%u words (%u bytes)\r\n",
                     current_channel_enable, current_packet_size, current_packet_size * 4);
    }
}

// ============================================================================
// BRAM ACCESS FUNCTIONS
// ============================================================================

int n_words_available;

// Check how many complete packets are available to read
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

// Read and validate one packet directly from BRAM with UDP transmission
static int process_packet_from_bram(void) {
  XTime t_loop0; XTime_GetTime(&t_loop0);   // perf: receive->transmit timer
  // Unified packet format: header word 0 = MAGIC (0xCAFEBABE), word 1 = TYPE_VER
  // with stream_type=1 (broadband), version=1 in the low 16 bits. The capture
  // BRAM only ever holds broadband packets, so we validate both the magic AND
  // the broadband stream_type/version.
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
  if ((ps_read_address + current_packet_size) <= BRAM_SIZE_WORDS) {
    derr = pl_dma_read_bram(pkt_buf, ps_read_address, current_packet_size);
  } else {
    uint32_t first = BRAM_SIZE_WORDS - ps_read_address;
    derr  = pl_dma_read_bram(pkt_buf, ps_read_address, first);
    derr |= pl_dma_read_bram(pkt_buf + first, 0, current_packet_size - first);
  }
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

// ============================================================================
// STREAMING CONTROL
// ============================================================================

void handle_enable_streaming(void) {
  if (stream_enabled) {
    send_message("Streaming already enabled\r\n");
    return;
  }

    // Update packet size before starting streaming
    update_current_packet_size();
  
  // Reset state
  packets_received_count = 0;
  error_count = 0;
  udp_packets_sent = 0;
  udp_send_errors = 0;
  
  // Reset PL
  pl_set_transmission(0);
  usleep(100);
  pl_reset_timestamp();
  usleep(1000);

  // Enable streaming
  stream_enabled = 1;
  pl_set_transmission(1);

  // Re-sync ps_read to a REAL packet boundary by scanning for the header.
  // A stop can interrupt the datapath mid-packet; since write_address, the
  // packet boundary, and the (intentionally unreset) FIFO are only cleared by
  // the hardware reset -- not by stop/restart -- the fresh header on restart can
  // land a few words off packet_boundary_address. Setting ps_read to the
  // pointer then mis-aligns and the header check loops forever. So: let the PL
  // write several packets, then walk back from the write pointer to the nearest
  // unified header (word0=MAGIC, word1=broadband TYPE_VER) and align to it
  // (single Xil_In32 reads are clean).
  usleep(3000);  // ~90 packets @30ksps -- guarantees fresh complete packets
  uint32_t wp = pl_get_bram_write_address();
  uint32_t expected_typever =
      (uint32_t)STREAM_TYPE_BROADBAND | ((uint32_t)UNIFIED_VERSION << 8);
  int synced = 0;
  for (uint32_t back = 0; back < 2 * current_packet_size + 16; back++) {
    uint32_t a = (wp + BRAM_SIZE_WORDS - back) % BRAM_SIZE_WORDS;
    uint32_t b = (a + 1) % BRAM_SIZE_WORDS;
    if (Xil_In32(BRAM_BASE_ADDR + a * 4) == UNIFIED_MAGIC &&                       // word0 MAGIC  // DMA-EXEMPT: 2-word resync header peek (clean 1-beat reads while re-aligning ps_read)
        (Xil_In32(BRAM_BASE_ADDR + b * 4) & 0xFFFFu) == (expected_typever & 0xFFFFu)) { // word1 TYPE_VER // DMA-EXEMPT: 2-word resync header peek (clean 1-beat reads while re-aligning ps_read)
      ps_read_address = a;
      synced = 1;
      break;
    }
  }
  if (!synced) {
    ps_read_address = wp;   // fallback; the magic-fail recovery will retry
    send_message("Restart: no magic found near wp=%u, using write ptr\r\n", wp);
  } else {
    send_message("Restart: ps_read re-synced to magic at %u (wp=%u)\r\n",
                 ps_read_address, wp);
  }

  send_message("BRAM streaming STARTED (packet size: %u words)\r\n", current_packet_size);
}

void handle_disable_streaming(void) {
  if (!stream_enabled) {
    send_message("Streaming already disabled\r\n");
    return;
  }
  
  stream_enabled = 0;
  pl_set_transmission(0);
  
  send_message("BRAM streaming STOPPED\r\n");
  send_message("Summary: %u packets processed, %u errors\r\n",
       packets_received_count, error_count);
  send_message("UDP: %u packets sent, %u errors\r\n", udp_packets_sent, udp_send_errors);
}

void handle_reset_timestamp(void) {
  packets_received_count = 0;
  error_count = 0;
  udp_packets_sent = 0;
  udp_send_errors = 0;
  pl_reset_timestamp();
  send_message("Timestamp and counters RESET\r\n");
}

void process_command_flags(void) {
  if (command_flags->enable_streaming_flag) {
    command_flags->enable_streaming_flag = 0;
    handle_enable_streaming();
    command_flags->lock = 0;
  }
  
  if (command_flags->disable_streaming_flag) {
    command_flags->disable_streaming_flag = 0;
    handle_disable_streaming();
    command_flags->lock = 0;
  }
  
  if (command_flags->reset_timestamp_flag) {
    command_flags->reset_timestamp_flag = 0;
    handle_reset_timestamp();
    command_flags->lock = 0;
  }

  if (command_flags->pl_print_flag) {
    command_flags->pl_print_flag = 0;
    pl_print_status();
    command_flags->lock = 0;
  }

  if (command_flags->dump_bram_flag) {
    command_flags->dump_bram_flag = 0;
    pl_dump_bram_data(command_flags->start_bram_addr, command_flags->word_count);
    command_flags->lock = 0;
  }

  if (command_flags->cable_test_flag) {
    command_flags->cable_test_flag = 0;
    pl_run_full_cable_test();
    handle_enable_streaming();
    command_flags->lock = 0;
  }
}

// Network maintenance loop
// Publish a binary status snapshot to shared memory for core 1 to format/print.
// Cheap, bounded, non-blocking: ~15 PL register reads + plain stores, no string
// formatting and no print ring. seqlock (odd while writing) lets core 1 read a
// consistent snapshot. This is what replaces the old core-0 console flood.
static void publish_status_snapshot(void) {
  uint32_t s0 = psmon->seq;
  psmon->seq = s0 | 1u;          // mark odd: update in progress
  dsb();

  uint64_t ts = pl_get_timestamp();
  psmon->timestamp_lo   = (uint32_t)ts;
  psmon->timestamp_hi   = (uint32_t)(ts >> 32);
  psmon->packets_sent   = pl_get_packets_sent();
  psmon->bram_write_addr= pl_get_bram_write_address();
  psmon->fifo_count     = (Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET) >> 14) & 0x1FF;
  psmon->state_counter  = pl_get_state_counter();
  psmon->cycle_counter  = pl_get_cycle_counter();
  psmon->channel_enable = pl_get_current_channel_enable();
  int p0, p1;
  pl_get_current_phase_select(&p0, &p1);
  // port-B phase2/phase3 read from the CTRL_REG_2 mirror (status reg 8): reg 8
  // mirrors CTRL_REG_2 verbatim, so phase_b0 is at [11:8], phase_b1 at [15:12].
  uint32_t cr2 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_8_OFFSET);
  psmon->phase          = (p0 & 0xF) | ((p1 & 0xF) << 4)
                        | (((cr2 >> 8) & 0xF) << 8) | (((cr2 >> 12) & 0xF) << 12);
  psmon->flags_pl       = (pl_is_transmission_active() ? PSMON_FLAG_TX_ACTIVE : 0)
                        | (pl_is_loop_limit_reached()  ? PSMON_FLAG_LOOP_LIMIT : 0)
                        | (pl_get_current_debug_mode() ? PSMON_FLAG_DEBUG_MODE : 0);

  psmon->packets_received = packets_received_count;
  psmon->error_count      = error_count;
  psmon->udp_packets_sent = udp_packets_sent;
  psmon->udp_send_errors  = udp_send_errors;
  psmon->ps_read_addr     = ps_read_address;
  psmon->packet_size      = current_packet_size;
  psmon->stream_enabled   = stream_enabled;

  dsb();
  psmon->seq = (s0 | 1u) + 1u;   // even again: snapshot complete
}

// Pump the lwIP RX path once. Called continuously from the moment RX goes live
// (right after xemac_add) through ALL of init so an early client's SYN/ARP can't
// pile up in an un-serviced window and wedge the GEM RX. Belt-and-suspenders with
// the beacon-gated connect (the client shouldn't connect until it hears us) --
// but if a client DOES poke us during boot, draining here keeps the RX healthy.

// ---- GEM RX-hang self-heal (Zynq-7000 SI#692601) ---------------------------
// The GEM RX can latch up ("used-bit hang"): RXSR sets BUFFNA (b0) and/or RXOVR
// (b2), the RX DMA stops, and the MAC receives nothing (even though TX keeps
// working) until reset. This was the "connect during boot -> unreachable until
// power-cycle" failure. Recover by toggling RXEN (vendor SI#692601 workaround) and
// clearing the sticky RX status bits, GATED on the actual hang bits so a healthy/
// idle RX is never touched (the old resetrx toggled on merely-idle RX and regressed
// normal boots). Cheap: one register read per call when there's no hang.
#define GEM_BASE          XPAR_XEMACPS_0_BASEADDR
#define GEM_NWCTRL_OFF    0x000u
#define GEM_RXSR_OFF      0x020u
#define GEM_NWCTRL_RXEN   0x00000004u
#define GEM_RXSR_BUFFNA   0x00000001u
#define GEM_RXSR_RXOVR    0x00000004u
#define GEM_RXSR_HRESPNOK 0x00000008u

volatile uint32_t rx_hang_recoveries = 0;

static void rx_hang_recover(void) {
  uint32_t rxsr = Xil_In32(GEM_BASE + GEM_RXSR_OFF);
  if (rxsr & (GEM_RXSR_BUFFNA | GEM_RXSR_RXOVR | GEM_RXSR_HRESPNOK)) {
    uint32_t nwctrl = Xil_In32(GEM_BASE + GEM_NWCTRL_OFF);
    Xil_Out32(GEM_BASE + GEM_NWCTRL_OFF, nwctrl & ~GEM_NWCTRL_RXEN);  // RXEN off (TXEN preserved)
    Xil_Out32(GEM_BASE + GEM_NWCTRL_OFF, nwctrl);                     // RXEN on
    Xil_Out32(GEM_BASE + GEM_RXSR_OFF, rxsr);                         // W1C sticky bits
    rx_hang_recoveries++;
    send_message("GEM RX hang RXSR=0x%02x -> RXEN toggled to recover [#%u]\r\n",
                 (unsigned)(rxsr & 0xffu), (unsigned)rx_hang_recoveries);
  }
}

static inline void service_network(void) {
  xemacif_input(&server_netif);
  sys_check_timeouts();
  rx_hang_recover();   // self-heal a GEM RX hang during the boot/init window too
}

void network_maintenance_loop(void) {
  static uint32_t counter = 0;
  static uint32_t last_link_check_time = 0;
  static uint32_t last_psmon_time = 0;
  counter++;

  xemacif_input(&server_netif);
  sys_check_timeouts();
  rx_hang_recover();   // steady-state GEM RX-hang self-heal (gated on the hang bits)
  process_command_flags();

  // Refresh the shared status snapshot at ~200 Hz (every 5 ms). Cheap and
  // non-blocking; core 1 reads it on demand or for its ~1 Hz monitor.
  uint32_t now_ms = sys_now();
  if (now_ms - last_psmon_time >= 5) {
    last_psmon_time = now_ms;
    publish_status_snapshot();
  }

  // Discovery beacon: broadcast our identity ~1 Hz while the link is up so a
  // client can auto-discover us and gate its connect on hearing us. TX-only and
  // tiny; harmless during streaming.
  static uint32_t last_beacon_time = 0;
  if (link_is_up && (now_ms - last_beacon_time >= 1000)) {
    last_beacon_time = now_ms;
    beacon_send();
  }

  // Poll network link state every 500ms for hotplug detection
  uint32_t current_time = sys_now();
  if (current_time - last_link_check_time >= 500) {
    last_link_check_time = current_time;

    // Update PHY link status
    eth_link_detect(&server_netif);
    int current_link_state = netif_is_link_up(&server_netif) ? 1 : 0;

    // Detect link state transitions
    if (link_is_up && !current_link_state) {
      // Link went DOWN
      link_is_up = 0;
      send_message("Network link DOWN - cable disconnected\r\n");

      // Abort TCP connections immediately
      abort_tcp_connections();
      stop_tcp_server();

      // Stop UDP stream
      stop_udp_stream();

      // Disable streaming if active
      if (stream_enabled) {
        handle_disable_streaming();
        send_message("Streaming automatically stopped due to link down\r\n");
      }
    } else if (!link_is_up && current_link_state) {
      // Link came UP
      link_is_up = 1;
      send_message("Network link UP - cable reconnected\r\n");

      // Restart TCP server
      start_tcp_server();

      // Restart UDP stream
      udp_stream_init();

      // Announce ourselves so a reconnecting host re-learns our MAC immediately
      // (same stale-ARP antidote as at boot, for the hotplug path).
      etharp_gratuitous(&server_netif);

      send_message("Network ready. Send START command to resume streaming.\r\n");
    }
  }
}

// ============================================================================
// MAIN APPLICATION
// ============================================================================

int main() {
  ip_addr_t ipaddr, netmask, gw;
  unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

  init_platform();
  XilTickTimer_Init(&timer);
  perf_timer_hz = COUNTS_PER_SECOND;   // global-timer freq; host converts ticks->us

  // ========================================================================
  // NOTE: This applies to 1M of memory (see TRM - UG585)
  Xil_SetTlbAttributes(SHARED_MEM_BASE, NORM_NONCACHE_SHARED); // Critical for coherency!
  // The capture BRAM (0x80000000) is written by the PL behind the data cache.
  // Reading it cached lets the A9 prefetcher pull lines while the PL is mid-write,
  // and there is no invalidate in the streaming read path -> stale/garbage words
  // surface in the UDP stream (seen as out-of-range values near the tail cycles
  // of each packet, worse at the higher dual-port data rate). Map it
  // non-cacheable so every read goes to the PL's committed data.
  Xil_SetTlbAttributes(BRAM_BASE_ADDR, NORM_NONCACHE_SHARED); // PL writes behind the cache
  // Xil_SetTlbAttributes(PL_CTRL_BASE_ADDR, NORM_NONCACHE_SHARED);
  // Prepare for second core by initializing shared structures
  init_print_buffer();
  memset((void *)command_flags, 0, sizeof(command_flags_t));
  psmon_init();   // zero the status snapshot before core 1 reads it
  pl_rhd_shadow_init();   // seed the RHD register mirror from the init defaults
#if BRAM_READ_METHOD == BRAM_READ_DMA
  pl_dma_init();  // AXI CDMA + non-cacheable DDR staging buffer for the read path
#endif
  // ========================================================================

  // ========================================================================
  // Clean cache to make sure memory is visible to CPU1
  // Write the memory space base address in the Zynq's DDR (PS7 DDR) for ARM Core 1 to 
  //  0xFFFFFFF0 (which is 0x10080000 in this project).
  Xil_Out32(ARM1_BASEADDR, ARM1_STARTADR);
  // Flush the cache line containing the register write
  Xil_DCacheFlushRange(ARM1_BASEADDR, 4);
  // Full memory barriers to ensure ordering
  dmb();  // Data Memory Barrier
  dsb();  // Data Synchronization Barrier
  isb();  // Instruction Synchronization Barrier
  // ========================================================================

  xil_printf("GLANCE v%d.%d.%d.%d by the Kemere Lab\n\r\n\r\n\r",
            FIRMWARE_VERSION_MAJOR,
            FIRMWARE_VERSION_MINOR,
            FIRMWARE_VERSION_PATCH,
            FIRMWARE_VERSION_BUILD);

  // Initialize network
  IP4_ADDR(&ipaddr, 192, 168, 18, 10);
  IP4_ADDR(&netmask, 255, 255, 255, 0);
  IP4_ADDR(&gw, 192, 168, 18, 1);
  
  // NOTE: Something is suspect about our shared memory setup, in the sense
  //       that LWIP does something that breaks if we let the other core start
  //       before we call lwip_init. Is it LWIP's fault? Ours???

  // TODO: Figure out how to make this work with hotplug
  // TODO: Ideally, we'd allow for a DHCP option with some sort of discovery protocol
  lwip_init();
  
  netif_add(&server_netif, &ipaddr, &netmask, &gw, NULL, NULL, NULL);
  netif_set_default(&server_netif);
  xemac_add(&server_netif, &ipaddr, &netmask, &gw,
       mac_ethernet_address, XPAR_XEMACPS_0_BASEADDR);
  netif_set_up(&server_netif);
  service_network();   // RX is live now -- start draining it through the rest of init

  // Start second core
  xil_printf("ARM0: sending the SEV to wake up ARM1\n\r");
  sev(); // Send event to wake up ARM1
  usleep(5000);

  send_message("Debug server up and running.\r\n");
    
  // Interrogate PHY to detect initial link state right after xemac_add completes
  // We'll only start TCP and UDP if we're connected
  eth_link_detect(&server_netif);
  if (netif_is_link_up(&server_netif)) {
    link_is_up = 1;
    send_message("Network link UP at boot\r\n");
  } else {
    link_is_up = 0;
    send_message("Network link DOWN at boot - waiting for cable connection...\r\n");
  }

  while (!link_is_up) {
    service_network();   // keep draining RX while waiting for PHY link-up
    eth_link_detect(&server_netif);
    if (netif_is_link_up(&server_netif)) {
      link_is_up = 1;
      send_message("Network link UP\r\n");
    }
  }

  start_tcp_server();
  service_network();   // answer anything already waiting on the listener

  // Initialize UDP (always enabled)
  udp_stream_init();

  send_message("Network initialized. IP: %s\r\n", ip4addr_ntoa(&ipaddr));
  
  // Initialize PL
  pl_set_transmission(0);
  pl_set_loop_count(0);
    
  // Initialize packet size based on current channel_enable setting
  update_current_packet_size();

  pl_set_copi_commands(initialization_cmd_sequence);
  service_network();   // keep the RX pool drained across PL init
  
  send_message("System ready. Commands: start, stop, reset_timestamp, status\r\n");
  send_message("debug> ");

  // Board is fully up now. Proactively announce our IP->MAC with a gratuitous
  // ARP so the host's ARP cache and the switch CAM learn us immediately -- the
  // direct antidote to the stale/failed-ARP "[Errno 64] Host is down" the host
  // reports when it connects before it has heard from us. Then emit ONE
  // unambiguous readiness line, distinct from the earlier "link UP"/"server up"
  // notices that fire >20 s before the board can actually service a connection.
  etharp_gratuitous(&server_netif);
  send_message("READY: safe to connect now (TCP command port %d up)\r\n", TCP_PORT);

  // Start the discovery beacon: from here we broadcast our identity ~1 Hz (in
  // network_maintenance_loop) so a client can auto-discover our IP and know we're
  // up, without sending us anything during boot. See beacon_send().
  beacon_init();

  // Main event loop
  while (1) {
    network_maintenance_loop();
    
    if (stream_enabled) {
      // Process all available packets with direct BRAM access and UDP transmission
      while (packets_available() > 0) { 
        process_packet_from_bram();
        
        // Periodic status (every 30k packets)
        if (packets_received_count % 30000 == 0) {
          send_message("Processed %u packets, %u errors, %u nwa, UDP: %u sent/%u errors\r\n",
               packets_received_count, error_count, n_words_available,
               udp_packets_sent, udp_send_errors);
        }
      }
    }
  }
  
  cleanup_platform();
  return 0;
}
