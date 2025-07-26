#include "main.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/timeouts.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include "xil_io.h"


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
        xil_printf("TCP Command: %s\r\n", (char *)p->payload);
        
        // Basic streaming control
        if (strncmp(p->payload, "start", 5) == 0) {
            enable_streaming_flag = 1;
            xil_printf("TCP Command: Enable streaming requested\r\n");
            
        } else if (strncmp(p->payload, "stop", 4) == 0) {
            disable_streaming_flag = 1;
            xil_printf("TCP Command: Disable streaming requested\r\n");
            
        } else if (strncmp(p->payload, "reset_timestamp", 15) == 0) {
            reset_timestamp_flag = 1;
            xil_printf("TCP Command: Reset timestamp requested\r\n");
            
        // PL configuration
        } else if (strncmp(p->payload, "loop", 4) == 0) {
            char* space_ptr = strchr(p->payload, ' ');
            if (space_ptr) {
                u32_t loop_count = atoi(space_ptr + 1);
                pl_set_loop_count(loop_count);
            } else {
                xil_printf("Usage: loop <count> (0=infinite)\r\n");
            }
            
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
            
        // Help command
        } else if (strncmp(p->payload, "help", 4) == 0) {
            xil_printf("=== AVAILABLE TCP COMMANDS ===\r\n");
            xil_printf("Basic Control:\r\n");
            xil_printf("  start                - Begin streaming\r\n");
            xil_printf("  stop                 - Stop streaming\r\n");
            xil_printf("  reset_timestamp      - Reset timestamp\r\n");
            xil_printf("  loop <count>         - Set loop count (0=infinite)\r\n");
            xil_printf("\r\nStatus Commands:\r\n");
            xil_printf("  status               - Show PL status\r\n");
            xil_printf("  dump_bram [start] [count] - Show BRAM contents\r\n");
            xil_printf("  help                 - Show this help\r\n");
            xil_printf("==============================\r\n");
            
        } else {
            xil_printf("Unknown command. Type 'help' for available commands.\r\n");
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
        xil_printf("ERROR: Could not create TCP PCB\r\n");
        return;
    }
    
    tcp_bind(pcb, IP_ADDR_ANY, TCP_PORT);
    pcb = tcp_listen(pcb);
    tcp_accept(pcb, tcp_accept_cb);
    xil_printf("TCP command server started on port %d\r\n", TCP_PORT);
}

void udp_stream_init() {
    ip_addr_t dest_ip;
    IP4_ADDR(&dest_ip, 192, 168, 1, 100);
    
    udp = udp_new();
    if (udp == NULL) {
        xil_printf("ERROR: Could not create UDP PCB\r\n");
        return;
    }
    
    udp_connect(udp, &dest_ip, UDP_PORT);
    xil_printf("UDP streaming initialized to %s:%d\r\n", ip4addr_ntoa(&dest_ip), UDP_PORT);
}