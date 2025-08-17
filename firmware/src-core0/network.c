#include "main.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/timeouts.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include "xil_io.h"
#include "shared_print.h"


u32_t sys_now(void) {
    XTime now;
    XTime_GetTime(&now);
    return (u32_t)(now / (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 1000U));
}

// Helper function to parse command arguments
static int parse_two_ints(const char* payload, u32* first, u32* second) {
    char* space1 = strchr(payload, ' ');
    if (!space1) return 0;
    
    *first = atoi(space1 + 1);
    
    char* space2 = strchr(space1 + 1, ' ');
    if (space2) {
        *second = atoi(space2 + 1);
        return 2;  // Found both
    }
    return 1;  // Found only first
}

err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    (void)arg;
    (void)err;
    
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }
    
    if (p->len > 0) {
        send_message("TCP Command: %s\r\n", (char *)p->payload);
        
        // Basic streaming control
        if (strncmp(p->payload, "start", 5) == 0) {
            command_flags->enable_streaming_flag = 1;
            send_message("TCP Command: Enable streaming requested\r\n");
            
        } else if (strncmp(p->payload, "stop", 4) == 0) {
            command_flags->disable_streaming_flag = 1;
            send_message("TCP Command: Disable streaming requested\r\n");
            
        } else if (strncmp(p->payload, "reset_timestamp", 15) == 0) {
            command_flags->reset_timestamp_flag = 1;
            send_message("TCP Command: Reset timestamp requested\r\n");
            
        // PL configuration
        } else if (strncmp(p->payload, "loop", 4) == 0) {
            char* space_ptr = strchr(p->payload, ' ');
            if (space_ptr) {
                u32_t loop_count = atoi(space_ptr + 1);
                pl_set_loop_count(loop_count);
            } else {
                send_message("Usage: loop <count> (0=infinite)\r\n");
            }
            
        // COPI sequence commands - NEW
        } else if (strncmp(p->payload, "convert", 7) == 0) {
            pl_set_convert_sequence();
            
        } else if (strncmp(p->payload, "init", 4) == 0) {
            pl_set_initialization_sequence();
            
        } else if (strncmp(p->payload, "cable_test", 10) == 0) {
            pl_set_cable_length_sequence();
            
        } else if (strncmp(p->payload, "test_pattern", 12) == 0) {
            pl_set_test_pattern_sequence();
            
        // Status commands
        } else if (strncmp(p->payload, "status", 6) == 0) {
            pl_print_status();
            
        // BRAM dump with optional parameters
        } else if (strncmp(p->payload, "dump_bram", 9) == 0) {
            u32 start_addr = 0;
            u32 word_count = 10;  // Default: show 10 words
            
            int args = parse_two_ints(p->payload, &start_addr, &word_count);
            if (args == 0) {
                // Use defaults
            } else if (args == 1) {
                word_count = 10;  // Keep default count
            }
            
            pl_dump_bram_data(start_addr, word_count);
        } else if (strncmp(p->payload, "set_phase", 9) == 0) {
            u32 phase0 = 0, phase1 = 0;
            int args = parse_two_ints(p->payload, &phase0, &phase1);
            if (args == 2) {
                pl_set_phase_select(phase0, phase1);
                send_message("Phase set to %u and %u\r\n", phase0, phase1);
            } else {
                send_message("Usage: set_phase <phase0> <phase1>\r\n");
            }
        } else if (strncmp(p->payload, "set_debug", 9) == 0) {
            char* space_ptr = strchr(p->payload, ' ');
            if (space_ptr) {
                u32_t enable = atoi(space_ptr + 1);
                pl_set_debug_mode(enable);
            } else {
                send_message("Usage: set_debug <0|1>\r\n");
            }   
        // Help command
        } else if (strncmp(p->payload, "help", 4) == 0) {
            send_message("=== AVAILABLE TCP COMMANDS ===\r\n");
            send_message("Basic Control:\r\n");
            send_message("  start                - Begin streaming\r\n");
            send_message("  stop                 - Stop streaming\r\n");
            send_message("  reset_timestamp      - Reset timestamp\r\n");
            send_message("  loop <count>         - Set loop count (0=infinite)\r\n");
            send_message("\r\nCOPI Command Sequences:\r\n");
            send_message("  convert              - Set normal data acquisition sequence\r\n");
            send_message("  init                 - Set chip initialization sequence\r\n");
            send_message("  cable_test           - Set cable length test sequence\r\n");
            send_message("  test_pattern         - Set COPI test pattern\r\n");
            send_message("\r\nConfiguration:\r\n");
            send_message("  set_phase <p0> <p1>  - Set phase delay for CIPO cables\r\n");
            send_message("  set_debug <0|1>      - Send dummy data vs real CIPO data\r\n");
            send_message("\r\nStatus Commands:\r\n");
            send_message("  status               - Show PL status\r\n");
            send_message("  dump_bram [start] [count] - Show BRAM contents\r\n");
            send_message("  help                 - Show this help\r\n");
            send_message("==============================\r\n");
            
        } else {
            send_message("Unknown command. Type 'help' for available commands.\r\n");
        }
    }
    
    tcp_recved(tpcb, p->len);
    pbuf_free(p);
    return ERR_OK;
}

err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;
    (void)err;
    tcp_recv(newpcb, tcp_recv_cb);
    return ERR_OK;
}

void start_tcp_server() {
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) {
        send_message("ERROR: Could not create TCP PCB\r\n");
        return;
    }
    
    tcp_bind(pcb, IP_ADDR_ANY, TCP_PORT);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_accept_cb);
    send_message("TCP command server started on port %d\r\n", TCP_PORT);
}

void udp_stream_init() {
    ip_addr_t dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 18, 100);
    
    udp = udp_new();
    if (udp == NULL) {
        send_message("ERROR: Could not create UDP PCB\r\n");
        return;
    }
    
    udp_connect(udp, &dest_ip, UDP_PORT);
    send_message("UDP streaming initialized to %s:%d\r\n", ip4addr_ntoa(&dest_ip), UDP_PORT);
}
