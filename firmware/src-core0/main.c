#include "main.h"
#include "platform.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include "xil_io.h"
#include "xil_cache.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
//#include "xuartps.h"
#include "shared_print.h"

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
uint32_t current_packet_size = 74;         // Current expected packet size in 32-bit words (default to max)
uint32_t current_channel_enable = 0x0F;    // Current channel enable setting (default all channels)

// Packet validation tracking
uint32_t error_count = 0;

// UDP transmission
uint32_t udp_packets_sent = 0;
uint32_t udp_send_errors = 0;
// UDP configuration (can be changed via TCP command)
uint32_t udp_dest_ip = 0;      // Will be initialized in main()
uint16_t udp_dest_port = DEFAULT_UDP_DEST_PORT;

// Pre-allocated packet buffer for UDP (sized for maximum packet)
// Use __attribute__((aligned(64))) to align to cache line boundary for optimal performance
static uint32_t udp_packet_buffer[MAX_WORDS_PER_PACKET] __attribute__((aligned(64)));

// ---- Capture-BRAM burst read -----------------------------------------------
// A single long memcpy over M_AXI_GP corrupts a run of words; dump_bram showed
// <=10-word reads always clean and >=~12 corrupting. So read the packet in
// bursts of BRAM_BURST_WORDS, each below the corruption length. We issue each
// burst as ONE inline `ldmia` (a single AXI INCR burst) instead of calling
// memcpy() per chunk: at -O3 memcpy is `b memcpy` (a call + alignment branches)
// paid ~19x/packet, and chunk-8 memcpy could not sustain 0xFF. ldmia drops the
// per-chunk software setup AND lets us use the largest safe burst (10), which
// also minimizes the number of high-latency GP address phases. (BRAM is mapped
// NORM_NONCACHE_SHARED, so ldmia to it issues a real burst.)
#define BRAM_BURST_WORDS  10

// Copy exactly 10 words src->dst as one 10-beat ldmia burst (no call/setup).
static inline void bram_burst10(uint32_t *d, const uint32_t *s) {
    uint32_t a, b, c, e, f, g, h, i, j, k;
    __asm__ volatile(
        "ldmia %10, {%0,%1,%2,%3,%4,%5,%6,%7,%8,%9}"
        : "=r"(a), "=r"(b), "=r"(c), "=r"(e), "=r"(f),
          "=r"(g), "=r"(h), "=r"(i), "=r"(j), "=r"(k)
        : "r"(s) : "memory");
    d[0]=a; d[1]=b; d[2]=c; d[3]=e; d[4]=f;
    d[5]=g; d[6]=h; d[7]=i; d[8]=j; d[9]=k;
}

// Copy a CONTIGUOUS run of n words from BRAM (word-aligned src) into dst using
// 10-word ldmia bursts, with single-beat loads for the <10-word tail (volatile
// forces each tail word to a separate 1-beat read, never merged into a burst).
static inline void bram_copy_run(uint32_t *d, const uint32_t *s, uint32_t n) {
    while (n >= BRAM_BURST_WORDS) { bram_burst10(d, s); d += BRAM_BURST_WORDS; s += BRAM_BURST_WORDS; n -= BRAM_BURST_WORDS; }
    volatile const uint32_t *v = (volatile const uint32_t *)s;
    while (n--) *d++ = *v++;
}

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

  // GUARD BAND: keep the read pointer one full packet behind the PL write
  // frontier. The capture BRAM is a dual-CLOCK simple-dual-port RAM (PL writes
  // port A @84MHz, PS reads port B via AXI @131MHz). If the PS reads a word the
  // PL is committing in the SAME packet, the cross-clock same-address
  // read-during-write returns stale data -- observed on hardware as the BRAM's
  // i*4 power-on init pattern (0x0404=mem[257], 0x0624=mem[393], ...) leaking
  // into the packet tail (cyc ~26-29). The boundary pointer itself is correct;
  // the problem is margin, so hold a whole packet back -- the read region is
  // then never the region the PL is actively writing. Costs one packet (~33us)
  // of latency and one packet of the 109-packet BRAM buffer.
  if (n_words_available >= (int)current_packet_size)
    n_words_available -= current_packet_size;
  else
    n_words_available = 0;

  return n_words_available / current_packet_size;  // Use variable packet size
}

// Read and validate one packet directly from BRAM with UDP transmission
static int process_packet_from_bram(void) {
  // Calculate BRAM address (no copying - read directly)
  uint32_t magic_low_offset = ps_read_address; // should always be smaller than BRAM_SIZE_WORDS!!!
  uint32_t magic_high_offset = (ps_read_address + 1) % BRAM_SIZE_WORDS;

  // Read packet header from BRAM
  uint32_t magic_low = Xil_In32(BRAM_BASE_ADDR + (magic_low_offset * 4));
  uint32_t magic_high = Xil_In32(BRAM_BASE_ADDR + (magic_high_offset * 4));

  // Reconstruct 64-bit magic number
  uint64_t magic = ((uint64_t)magic_high << 32) | magic_low;

  // Validate magic number
  if (magic != 0xCAFEBABEDEADBEEF) {
    // Invalid magic - could be BRAM overflow, corruption, or misalignment
    // Jump directly to write pointer to sync with fresh data
    uint32_t pl_write_addr = pl_get_bram_write_address();
    ps_read_address = pl_write_addr;
    error_count++; // ERROR TO TRACK
    send_message("Magic validation failed (0x%016llX), jumping to write position %u\r\n",
                 magic, pl_write_addr);
    return 0; // Packet validation failed, now synced to fresh data
  }

  // TODO: If we are in an error state, we could track how long we stay there
  //    by measuring the timestamp gap when we recover.

  // UDP transmission (always enabled) - zero-copy with pre-allocated buffer.
  //
  // Read the packet out of the capture BRAM in BRAM_BURST_WORDS-word ldmia
  // bursts (see bram_copy_run / bram_burst10 above) -- short enough to stay
  // below the GP-port burst length that corrupts, but issued without memcpy's
  // per-chunk call/alignment overhead. Split at the BRAM wrap so each
  // bram_copy_run sees a contiguous, word-aligned run.
  if ((ps_read_address + current_packet_size) <= BRAM_SIZE_WORDS) {
    bram_copy_run(udp_packet_buffer,
                  (const uint32_t *)(BRAM_BASE_ADDR + ps_read_address * 4),
                  current_packet_size);
  } else {
    uint32_t first = BRAM_SIZE_WORDS - ps_read_address;
    bram_copy_run(udp_packet_buffer,
                  (const uint32_t *)(BRAM_BASE_ADDR + ps_read_address * 4), first);
    bram_copy_run(&udp_packet_buffer[first],
                  (const uint32_t *)BRAM_BASE_ADDR, current_packet_size - first);
  }

  // Create pbuf that references our buffer directly (zero-copy!)
  uint32_t packet_bytes = current_packet_size * BYTES_PER_WORD;
  struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, packet_bytes, PBUF_REF);
  if (p != NULL) {
    // Point pbuf payload directly to our buffer (zero-copy!)
    p->payload = (void*)udp_packet_buffer;

    // Send using udp_sendto (no connect required)
    ip_addr_t dest_ip;
    dest_ip.addr = udp_dest_ip;
    err_t result = udp_sendto(udp, p, &dest_ip, udp_dest_port);
    // err_t result = udp_send(udp, p);
    
    if (result == ERR_OK) {
      udp_packets_sent++;
    } else {
      send_message("UDP Send Error: %d\r\n", result);
      udp_send_errors++; // ERROR TO TRACK
    }
    
    // Free pbuf (this won't free our buffer since it's PBUF_REF)
    pbuf_free(p);
  } else {
    udp_send_errors++;
  }
  
  // Update read pointer with variable packet size
  ps_read_address = (ps_read_address + current_packet_size) % BRAM_SIZE_WORDS;
  packets_received_count++;
  
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

  // Re-sync ps_read to a REAL packet boundary by scanning for the magic.
  // A stop can interrupt the datapath mid-packet; since write_address, the
  // packet boundary, and the (intentionally unreset) FIFO are only cleared by
  // the hardware reset -- not by stop/restart -- the fresh magic on restart can
  // land a few words off packet_boundary_address. Setting ps_read to the
  // pointer then mis-aligns and the magic check loops forever. So: let the PL
  // write several packets, then walk back from the write pointer to the nearest
  // 0xDEADBEEF/0xCAFEBABE and align to it (single Xil_In32 reads are clean).
  usleep(3000);  // ~90 packets @30ksps -- guarantees fresh complete packets
  uint32_t wp = pl_get_bram_write_address();
  int synced = 0;
  for (uint32_t back = 0; back < 2 * current_packet_size + 16; back++) {
    uint32_t a = (wp + BRAM_SIZE_WORDS - back) % BRAM_SIZE_WORDS;
    uint32_t b = (a + 1) % BRAM_SIZE_WORDS;
    if (Xil_In32(BRAM_BASE_ADDR + a * 4) == 0xDEADBEEF &&     // magic low
        Xil_In32(BRAM_BASE_ADDR + b * 4) == 0xCAFEBABE) {     // magic high
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

  if (command_flags->bram_benchmark_flag) {
    command_flags->bram_benchmark_flag = 0;
    benchmark_bram_reads();
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
  // port-B phase2/phase3 read from the CTRL_REG_2 mirror (status reg 8)
  uint32_t cr2 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_8_OFFSET);
  psmon->phase          = (p0 & 0xF) | ((p1 & 0xF) << 4)
                        | (((cr2 >> 16) & 0xF) << 8) | (((cr2 >> 20) & 0xF) << 12);
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

void network_maintenance_loop(void) {
  static uint32_t counter = 0;
  static uint32_t last_link_check_time = 0;
  static uint32_t last_psmon_time = 0;
  counter++;

  xemacif_input(&server_netif);
  sys_check_timeouts();
  process_command_flags();

  // Refresh the shared status snapshot at ~200 Hz (every 5 ms). Cheap and
  // non-blocking; core 1 reads it on demand or for its ~1 Hz monitor.
  uint32_t now_ms = sys_now();
  if (now_ms - last_psmon_time >= 5) {
    last_psmon_time = now_ms;
    publish_status_snapshot();
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

  xil_printf("Kemere Lab Intan Interface v%d.%d.%d.%d\n\r\n\r\n\r",
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
    eth_link_detect(&server_netif);
    if (netif_is_link_up(&server_netif)) {
      link_is_up = 1;
      send_message("Network link UP\r\n");    
    }
  }

  start_tcp_server();
  
  // Initialize UDP (always enabled)
  udp_stream_init();

  send_message("Network initialized. IP: %s\r\n", ip4addr_ntoa(&ipaddr));
  
  // Initialize PL
  pl_set_transmission(0);
  pl_set_loop_count(0);
    
  // Initialize packet size based on current channel_enable setting
  update_current_packet_size();

  // benchmark_bram_reads();

  pl_set_copi_commands(initialization_cmd_sequence);
  
  send_message("System ready. Commands: start, stop, reset_timestamp, status\r\n");
  send_message("debug> ");
  
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
