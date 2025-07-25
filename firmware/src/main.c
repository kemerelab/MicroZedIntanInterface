#include "main.h"
#include "platform.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include "xil_io.h"
#include "xil_cache.h"
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "xuartps.h"

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
u64 expected_timestamp = 0;
u32 error_count = 0;
u32 timestamp_gaps = 0;

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
// SERIAL DEBUG COMMAND HANDLING
// ============================================================================

void process_serial_command(const char* cmd) {
    // Trim whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    if (strncmp(cmd, "start", 5) == 0) {
        xil_printf("Serial command: Starting transmission\r\n");
        enable_streaming_flag = 1;
        
    } else if (strncmp(cmd, "stop", 4) == 0) {
        xil_printf("Serial command: Stopping transmission\r\n");
        disable_streaming_flag = 1;
        
    } else if (strncmp(cmd, "reset", 5) == 0) {
        xil_printf("Serial command: Resetting timestamp\r\n");
        reset_timestamp_flag = 1;
        
    } else if (strncmp(cmd, "status", 6) == 0) {
        xil_printf("Serial command: Status\r\n");
        pl_print_status();
        
    } else if (strncmp(cmd, "benchmark", 9) == 0) {
        xil_printf("Serial command: Running BRAM benchmark\r\n");
        benchmark_bram_reads();
        
    } else if (strncmp(cmd, "dump", 4) == 0) {
        // Parse dump command: "dump [start] [count]"
        u32 start_addr = 0;
        u32 word_count = 16;  // Default
        
        sscanf(cmd, "dump %u %u", &start_addr, &word_count);
        
        xil_printf("Serial command: Dumping BRAM from %u, count %u\r\n", start_addr, word_count);
        dump_bram_data(start_addr, word_count);
        
    } else if (strncmp(cmd, "help", 4) == 0 || strlen(cmd) == 0) {
        xil_printf("\r\nSerial Debug Commands:\r\n");
        xil_printf("  start    - Start data transmission\r\n");
        xil_printf("  stop     - Stop data transmission\r\n");
        xil_printf("  reset    - Reset timestamp and counters\r\n");
        xil_printf("  status   - Show system status\r\n");
        xil_printf("  benchmark - Run BRAM read performance test\r\n");
        xil_printf("  dump [start] [count] - Dump BRAM contents\r\n");
        xil_printf("  help     - Show this help\r\n");
        
    } else {
        xil_printf("Unknown command: '%s'. Type 'help' for commands.\r\n", cmd);
    }
}

void check_serial_input(void) {
    // Check if UART has data available
    if (XUartPs_IsReceiveData(STDIN_BASEADDRESS)) {
        char ch = XUartPs_RecvByte(STDIN_BASEADDRESS);
        
        // Handle different line endings and backspace
        if (ch == '\r' || ch == '\n') {
            if (serial_cmd_index > 0) {
                // Null terminate the command
                serial_cmd_buffer[serial_cmd_index] = '\0';
                
                xil_printf("\r\n");  // Echo newline
                
                // Process the command
                process_serial_command(serial_cmd_buffer);
                
                // Reset buffer
                serial_cmd_index = 0;
                
                // Print prompt
                xil_printf("debug> ");
            }
        } else if (ch == '\b' || ch == 127) {  // Backspace or DEL
            if (serial_cmd_index > 0) {
                serial_cmd_index--;
                xil_printf("\b \b");  // Erase character on terminal
            }
        } else if (ch >= 32 && ch <= 126) {  // Printable characters
            if (serial_cmd_index < SERIAL_CMD_BUFFER_SIZE - 1) {
                serial_cmd_buffer[serial_cmd_index++] = ch;
                XUartPs_SendByte(STDIN_BASEADDRESS, ch);  // Echo character
            }
        }
        // Ignore other characters (like additional \n after \r)
    }
}

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
    // u32 magic_low_offset = (ps_read_address + 1) % BRAM_SIZE_WORDS;
    u32 magic_low_offset = ps_read_address; // should always be smaller than BRAM_SIZE_WORDS!!!
    u32 magic_high_offset = (ps_read_address + 1) % BRAM_SIZE_WORDS;
    u32 timestamp_low_offset = (ps_read_address + 2) % BRAM_SIZE_WORDS;
    u32 timestamp_high_offset = (ps_read_address + 3) % BRAM_SIZE_WORDS;
    
    // Read packet header directly from BRAM
    u32 magic_low = Xil_In32(BRAM_BASE_ADDR + (magic_low_offset * 4));
    u32 magic_high = Xil_In32(BRAM_BASE_ADDR + (magic_high_offset * 4));
    u32 timestamp_low = Xil_In32(BRAM_BASE_ADDR + (timestamp_low_offset * 4));
    u32 timestamp_high = Xil_In32(BRAM_BASE_ADDR + (timestamp_high_offset * 4));
    u32 packets_sent = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_3_OFFSET);

    // Reconstruct 64-bit values
    u64 magic = ((u64)magic_high << 32) | magic_low;
    u64 timestamp = ((u64)timestamp_high << 32) | timestamp_low;
    
    // Validate magic number
    if (magic != 0xCAFEBABEDEADBEEF) {
        xil_printf("\r\n*** MAGIC ERROR at packet %u (%u) ***\r\n", packets_received_count, n_words_available);
        xil_printf("Expected: 0xCAFEBABEDEADBEEF\r\n");
        xil_printf("Got:      0x%016llX\r\n", magic);
        xil_printf("Raw words: 0x%08X, 0x%08X\r\n", magic_low, magic_high);
        xil_printf("Raw words: 0x%08X, 0x%08X\r\n", timestamp_low, timestamp_high);

        u32 back_1_offset = (ps_read_address > 0) ? (ps_read_address - 1) : (BRAM_SIZE_WORDS - 1);
        u32 back_2_offset = (ps_read_address > 1) ? (ps_read_address - 2) : 
                           ((ps_read_address == 0) ? (BRAM_SIZE_WORDS - 2) : (BRAM_SIZE_WORDS - 1));
        
        u32 back_1_addr = BRAM_BASE_ADDR + (back_1_offset * 4);
        u32 back_2_addr = BRAM_BASE_ADDR + (back_2_offset * 4);
            
        u32 back_1 = Xil_In32(back_1_addr);
        u32 back_2 = Xil_In32(back_2_addr);
        xil_printf("Previous words: 0x%08X, 0x%08X\r\n", back_2, back_1);

        xil_printf("PS read addr: %u\r\n", ps_read_address);
        xil_printf("Packets Sent: %u\r\n", packets_sent);

        xil_printf("**********************\r\n\r\n");
        
        error_count++;
        return 0;  // Packet validation failed
    }
    
    // Check for timestamp continuity
    if (expected_timestamp != 0 && timestamp != expected_timestamp) {
        xil_printf("\r\n*** TIMESTAMP GAP at packet %u ***\r\n", packets_received_count);
        xil_printf("Expected: %llu, Got: %llu (gap: %lld)\r\n",
                  expected_timestamp, timestamp, (long long)(timestamp - expected_timestamp));
        xil_printf("*****************************\r\n\r\n");
        timestamp_gaps++;
    }
    
    // UDP transmission (always enabled) - zero-copy with pre-allocated buffer
    // Copy packet data to pre-allocated buffer with safe address wrapping
    
    for (int i = 0; i < WORDS_PER_PACKET; i++) {
        u32 word_offset = (ps_read_address + i) % BRAM_SIZE_WORDS;
        u32 safe_addr = BRAM_BASE_ADDR + (word_offset * 4);
        udp_packet_buffer[i] = Xil_In32(safe_addr);
    }
    
    /*
    // Use fast memcpy instead of slow Xil_In32 loop
    if ((ps_read_address + WORDS_PER_PACKET) <= BRAM_SIZE_WORDS) {
        // Packet doesn't wrap - single fast memcpy
        u32 packet_start_addr = BRAM_BASE_ADDR + (ps_read_address * 4);
        memcpy(udp_packet_buffer, (void*)packet_start_addr, BYTES_PER_PACKET);
    } else {
        // Packet wraps around BRAM boundary - two memcpy calls
        u32 words_before_wrap = BRAM_SIZE_WORDS - ps_read_address;
        u32 words_after_wrap = WORDS_PER_PACKET - words_before_wrap;
        
        // Copy first chunk (before wrap)
        u32 first_chunk_addr = BRAM_BASE_ADDR + (ps_read_address * 4);
        memcpy(udp_packet_buffer, (void*)first_chunk_addr, words_before_wrap * 4);
        
        // Copy second chunk (after wrap)
        memcpy(&udp_packet_buffer[words_before_wrap], (void*)BRAM_BASE_ADDR, words_after_wrap * 4);
    }
    */
    
    // Ensure cache coherency before sending to network hardware
    // Xil_DCacheFlushRange((UINTPTR)udp_packet_buffer, BYTES_PER_PACKET);
    
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
            udp_send_errors++;
            // Only print occasional UDP errors to avoid spam
            if (udp_send_errors % 1000 == 1) {
                xil_printf("UDP send error %u (code %d)\r\n", udp_send_errors, result);
            }
        }
        
        // Free pbuf (this won't free our buffer since it's PBUF_REF)
        pbuf_free(p);
    } else {
        udp_send_errors++;
    }
    
    // Update read pointer
    ps_read_address = (ps_read_address + WORDS_PER_PACKET) % BRAM_SIZE_WORDS;
    
    expected_timestamp = timestamp + 1;
    packets_received_count++;
    
    return 1;  // Success
}

// Initialize BRAM interface
int init_bram_interface(void) {
    xil_printf("Initializing BRAM interface...\r\n");
    
    ps_read_address = 0;
    expected_timestamp = 0;
    error_count = 0;
    timestamp_gaps = 0;
    udp_packets_sent = 0;
    udp_send_errors = 0;
    
    // Test BRAM connectivity
    u32 test_data = Xil_In32(BRAM_BASE_ADDR);
    xil_printf("BRAM[0]: 0x%08X\r\n", test_data);
    
    xil_printf("BRAM interface initialization complete\r\n");
    xil_printf("  BRAM base address: 0x%08X\r\n", BRAM_BASE_ADDR);
    xil_printf("  BRAM size: %u words (%u bytes)\r\n", BRAM_SIZE_WORDS, BRAM_SIZE_BYTES);
    xil_printf("  Packet size: %u words (%u bytes)\r\n", WORDS_PER_PACKET, BYTES_PER_PACKET);
    xil_printf("  Max packets: %u\r\n", MAX_PACKETS_IN_BRAM);
    
    return XST_SUCCESS;
}

// ============================================================================
// STREAMING CONTROL
// ============================================================================

void handle_enable_streaming_bram(void) {
    if (stream_enabled) {
        xil_printf("Streaming already enabled\r\n");
        return;
    }
    
    // Reset state
    packets_received_count = 0;
    ps_read_address = 0;
    expected_timestamp = 0;
    error_count = 0;
    timestamp_gaps = 0;
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
    
    xil_printf("BRAM streaming STARTED\r\n");
}

void handle_disable_streaming_bram(void) {
    if (!stream_enabled) {
        xil_printf("Streaming already disabled\r\n");
        return;
    }
    
    stream_enabled = 0;
    pl_set_transmission(0);
    
    xil_printf("BRAM streaming STOPPED\r\n");
    xil_printf("Summary: %u packets processed, %u errors, %u timestamp gaps\r\n",
              packets_received_count, error_count, timestamp_gaps);
    xil_printf("UDP: %u packets sent, %u errors\r\n", udp_packets_sent, udp_send_errors);
}

void handle_reset_timestamp_bram(void) {
    packets_received_count = 0;
    ps_read_address = 0;
    expected_timestamp = 0;
    error_count = 0;
    timestamp_gaps = 0;
    udp_packets_sent = 0;
    udp_send_errors = 0;
    pl_reset_timestamp();
    xil_printf("Timestamp and counters RESET\r\n");
}

void process_command_flags_bram(void) {
    if (enable_streaming_flag) {
        enable_streaming_flag = 0;
        handle_enable_streaming_bram();
    }
    
    if (disable_streaming_flag) {
        disable_streaming_flag = 0;
        handle_disable_streaming_bram();
    }
    
    if (reset_timestamp_flag) {
        reset_timestamp_flag = 0;
        handle_reset_timestamp_bram();
    }
}

// Network maintenance loop
void network_maintenance_loop(void) {
    static u32 counter = 0;
    counter++;
    
    xemacif_input(&server_netif);
    sys_check_timeouts();
    process_command_flags_bram();
}

// Simple BRAM dump for debugging
void dump_bram_data(u32 start_addr, u32 word_count) {
    xil_printf("BRAM dump starting at address %u:\r\n", start_addr);
    for (u32 i = 0; i < word_count; i++) {
        u32 addr = (start_addr + i) % BRAM_SIZE_WORDS;
        u32 data = Xil_In32(BRAM_BASE_ADDR + addr * 4);
        xil_printf("%u: 0x%08X - 0x%08X\r\n", i, BRAM_BASE_ADDR + addr * 4, data);
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
    
    // Initialize BRAM interface
    if (init_bram_interface() != XST_SUCCESS) {
        xil_printf("FATAL: BRAM interface initialization failed\r\n");
        return -1;
    }
    
    // Initialize network
    IP4_ADDR(&ipaddr, 192, 168, 1, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);
    
    lwip_init();
    
    netif_add(&server_netif, &ipaddr, &netmask, &gw, NULL, NULL, NULL);
    netif_set_default(&server_netif);
    xemac_add(&server_netif, &ipaddr, &netmask, &gw,
              mac_ethernet_address, XPAR_XEMACPS_0_BASEADDR);
    netif_set_up(&server_netif);
    
    xil_printf("Network initialized. IP: %s\r\n", ip4addr_ntoa(&ipaddr));
    xil_printf("System ready. Commands: start, stop, reset_timestamp, status, dump_bram\r\n");
    xil_printf("Serial debug: Type 'help' for commands\r\n");
    
    // Initialize PL
    pl_set_transmission(0);
    pl_set_loop_count(0);
    
    start_tcp_server();
    
    // Initialize UDP (always enabled)
    udp_stream_init();

    benchmark_bram_reads();
    
    xil_printf("debug> ");
    
    // Main event loop - minimal copying, direct BRAM access with UDP transmission
    while (1) {
        // Check for serial debug commands
        check_serial_input();
        
        network_maintenance_loop();
        
        if (stream_enabled) {
            // Process all available packets with direct BRAM access and UDP transmission
            while (packets_available() > 1) {  // Keep 1 packet buffer for safety
                if (!process_packet_from_bram()) {
                    // Validation failed - stop streaming
                    handle_disable_streaming_bram();
                    break;
                }
                
                // Periodic status (every 30k packets)
                if (packets_received_count % 30000 == 0) {
                    xil_printf("Processed %u packets, %u errors, %u gaps, %u nwa, UDP: %u sent/%u errors\r\n",
                              packets_received_count, error_count, timestamp_gaps, n_words_available,
                              udp_packets_sent, udp_send_errors);
                }
            }
        }
    }
    
    cleanup_platform();
    return 0;
}