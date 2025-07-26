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
        
        sscanf(cmd, "dump %lu %lu", &start_addr, &word_count);
        
        pl_dump_bram_data(start_addr, word_count);
        
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
        xil_printf("Streaming already enabled\r\n");
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
    
    xil_printf("BRAM streaming STARTED\r\n");
}

void handle_disable_streaming(void) {
    if (!stream_enabled) {
        xil_printf("Streaming already disabled\r\n");
        return;
    }
    
    stream_enabled = 0;
    pl_set_transmission(0);
    
    xil_printf("BRAM streaming STOPPED\r\n");
    xil_printf("Summary: %u packets processed, %u errors\r\n",
              packets_received_count, error_count);
    xil_printf("UDP: %u packets sent, %u errors\r\n", udp_packets_sent, udp_send_errors);
}

void handle_reset_timestamp(void) {
    packets_received_count = 0;
    ps_read_address = 0;
    error_count = 0;
    udp_packets_sent = 0;
    udp_send_errors = 0;
    pl_reset_timestamp();
    xil_printf("Timestamp and counters RESET\r\n");
}

void process_command_flags(void) {
    if (enable_streaming_flag) {
        enable_streaming_flag = 0;
        handle_enable_streaming();
    }
    
    if (disable_streaming_flag) {
        disable_streaming_flag = 0;
        handle_disable_streaming();
    }
    
    if (reset_timestamp_flag) {
        reset_timestamp_flag = 0;
        handle_reset_timestamp();
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

// ============================================================================
// MAIN APPLICATION
// ============================================================================

int main() {
    ip_addr_t ipaddr, netmask, gw;
    unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };
    
    init_platform();
    XilTickTimer_Init(&timer);
    
    // Initialize network
    IP4_ADDR(&ipaddr, 192, 168, 1, 10);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 192, 168, 1, 1);
    
    // TODO: Figure out how to make this work with hotplug
    // TODO: Ideally, we'd allow for a DHCP option with some sort of discovery protocol
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

    pl_set_copi_commands(mosi_test_pattern);
    
    xil_printf("debug> ");
    
    // Main event loop - minimal copying, direct BRAM access with UDP transmission
    while (1) {
        // Check for serial debug commands
        check_serial_input();
        
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
                    xil_printf("Processed %u packets, %u errors, %u nwa, UDP: %u sent/%u errors\r\n",
                              packets_received_count, error_count, n_words_available,
                              udp_packets_sent, udp_send_errors);
                }
            }
        }
    }
    
    cleanup_platform();
    return 0;
}