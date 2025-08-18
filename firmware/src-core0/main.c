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

// Global variables
XTimer timer;
struct netif server_netif;
struct udp_pcb *udp;
volatile int stream_enabled = 0;
u32_t packets_received_count = 0;

// Command flags for main loop processing
volatile int enable_streaming_flag = 0;
volatile int disable_streaming_flag = 0;
volatile int reset_timestamp_flag = 0;

// BRAM state tracking
u32 ps_read_address = 0;              // Current PS read position (word address)

// Packet validation tracking
u32 error_count = 0;

// UDP transmission
u32 udp_packets_sent = 0;
u32 udp_send_errors = 0;

// Pre-allocated packet buffer for UDP 
// Use __attribute__((aligned(64))) to align to cache line boundary for optimal performance
static u32 udp_packet_buffer[WORDS_PER_PACKET] __attribute__((aligned(64)));

// Serial debug command buffer
#define SERIAL_CMD_BUFFER_SIZE 64
static char serial_cmd_buffer[SERIAL_CMD_BUFFER_SIZE];
static int serial_cmd_index = 0;


// ============================================================================
// BRAM ACCESS FUNCTIONS
// ============================================================================

int n_words_available;

// Check how many complete packets are available to read
static int packets_available(void) {
    u32 pl_write_addr = pl_get_bram_write_address();
    
    if (pl_write_addr >= ps_read_address) {
        n_words_available = pl_write_addr - ps_read_address;
    } else {
        // Handle wrap-around
        n_words_available = (BRAM_SIZE_WORDS - ps_read_address) + pl_write_addr;
    }

    return n_words_available / WORDS_PER_PACKET;
}


// Read and validate one packet directly from BRAM with UDP transmission
static int process_packet_from_bram(void) {
    // Calculate BRAM address (no copying - read directly)
    u32 magic_low_offset = ps_read_address; // should always be smaller than BRAM_SIZE_WORDS!!!
    u32 magic_high_offset = (ps_read_address + 1) % BRAM_SIZE_WORDS;
    // Read packet header from BRAM
    u32 magic_low = Xil_In32(BRAM_BASE_ADDR + (magic_low_offset * 4));
    u32 magic_high = Xil_In32(BRAM_BASE_ADDR + (magic_high_offset * 4));
    // Reconstruct 64-bit values
    u64 magic = ((u64)magic_high << 32) | magic_low;
    // Validate magic number
    if (magic != 0xCAFEBABEDEADBEEF) {
        // The only way that this should happen is if we've overflowed our BRAM
        // To try to recover, we need to keep moving ASAP. So we won't send
        //  this packet over the network, but we'll try to fast forward.
        ps_read_address = (ps_read_address + WORDS_PER_PACKET) % BRAM_SIZE_WORDS;
    
        error_count++;  // ERROR TO TRACK

        return 0;  // Packet validation failed
    }

    // TODO: If we are in an error state, we could track how long we stay there
    //       by measuring the timestamp gap when we recover.

    // UDP transmission (always enabled) - zero-copy with pre-allocated buffer
    // Copy packet data to pre-allocated buffer with safe address wrapping.
    // TODO: Consider replacing with memcpy
    
    for (int i = 0; i < WORDS_PER_PACKET; i++) {
        u32 word_offset = (ps_read_address + i) % BRAM_SIZE_WORDS;
        u32 safe_addr = BRAM_BASE_ADDR + (word_offset * 4);
        udp_packet_buffer[i] = Xil_In32(safe_addr);
    }
    
    // Create pbuf that references our buffer directly (zero-copy!)
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, BYTES_PER_PACKET, PBUF_REF);
    if (p != NULL) {
        // Point pbuf payload directly to our buffer (no copying!)
        p->payload = (void*)udp_packet_buffer;
        
        // Send UDP packet
        err_t result = udp_send(udp, p);
        if (result == ERR_OK) {
            udp_packets_sent++;
        } else {
            udp_send_errors++;  // ERROR TO TRACK
        }
        
        // Free pbuf (this won't free our buffer since it's PBUF_REF)
        pbuf_free(p);
    } else {
        udp_send_errors++;
    }
    
    // Update read pointer
    ps_read_address = (ps_read_address + WORDS_PER_PACKET) % BRAM_SIZE_WORDS;
    
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
    
    // Reset state
    packets_received_count = 0;
    ps_read_address = 0;
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
    
    send_message("BRAM streaming STARTED\r\n");
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
    ps_read_address = 0;
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
    if( command_flags->pl_print_flag) {
        command_flags->pl_print_flag = 0;
        pl_print_status();
        command_flags->lock = 0;
    }
    if( command_flags->bram_benchmark_flag) {
        command_flags->bram_benchmark_flag = 0;
        benchmark_bram_reads();
        command_flags->lock = 0;
    }
    if(command_flags->dump_bram_flag) {
        command_flags->dump_bram_flag = 0;
        pl_dump_bram_data(command_flags->start_bram_addr, command_flags->word_count);
        command_flags->lock = 0;
    }
}

// Network maintenance loop
void network_maintenance_loop(void) {
    static u32 counter = 0;
    counter++;
    
    xemacif_input(&server_netif);
    sys_check_timeouts();
    process_command_flags();
}


// ====================================================

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
    // Prepare for second core by initializing shared structures
    init_print_buffer();
    memset((void *)command_flags, 0, sizeof(command_flags_t));
    // ========================================================================

    // ========================================================================
    // Clean cache to make sure memory is visible to CPU1
    // Write the memory space base address in the Zynq's DDR (PS7 DDR) for ARM Core 1 to 
    //    0xFFFFFFF0 (which is 0x10080000 in this project).
    Xil_Out32(ARM1_BASEADDR, ARM1_STARTADR);
    // Flush the cache line containing the register write
    Xil_DCacheFlushRange(ARM1_BASEADDR, 4);
    // Full memory barriers to ensure ordering
    dmb();  // Data Memory Barrier
    dsb();  // Data Synchronization Barrier
    isb();  // Instruction Synchronization Barrier
    // ========================================================================

    xil_printf("Kemere Lab Intan Interface v1.0.0.0\n\r\n\r\n\r");

    // Initialize network
    IP4_ADDR(&ipaddr, 192, 168, 18, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 18, 1);
    
    // TODO: Figure out how to make this work with hotplug
    // TODO: Ideally, we'd allow for a DHCP option with some sort of discovery protocol
    lwip_init();
    
    netif_add(&server_netif, &ipaddr, &netmask, &gw, NULL, NULL, NULL);
    netif_set_default(&server_netif);
    xemac_add(&server_netif, &ipaddr, &netmask, &gw,
              mac_ethernet_address, XPAR_XEMACPS_0_BASEADDR);
    netif_set_up(&server_netif);

    xil_printf("ARM0: sending the SEV to wake up ARM1\n\r");
    sev(); // Send event to wake up ARM1

    usleep(5000);

    send_message("Debug server up and running. \r\n");
    send_message("Network initialized. IP: %s\r\n", ip4addr_ntoa(&ipaddr));
    send_message("System ready. Commands: start, stop, reset_timestamp, status, dump_bram\r\n");
    send_message("Serial debug: Type 'help' for commands\r\n");
    
    // Initialize PL
    pl_set_transmission(0);
    pl_set_loop_count(0);
    
    start_tcp_server();
    
    // Initialize UDP (always enabled)
    udp_stream_init();

    // benchmark_bram_reads();

    pl_set_copi_commands(mosi_test_pattern);
    
    send_message("debug> ");
    
    // Main event loop - minimal copying, direct BRAM access with UDP transmission
    while (1) {
        network_maintenance_loop();
        
        if (stream_enabled) {
            // Process all available packets with direct BRAM access and UDP transmission
            while (packets_available() > 1) {  // Keep 1 packet buffer for safety
                process_packet_from_bram();
                /*
                if (!process_packet_from_bram()) {
                    // Validation failed - stop streaming
                    handle_disable_streaming();
                    break;
                }*/
                
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
