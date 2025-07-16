#include "xparameters.h"
#include "netif/xadapter.h"
#include "lwip/init.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/timeouts.h"
#include "platform.h"
#include "sleep.h"
#include <string.h>
#include <stdio.h>
#include "xiltimer.h"
#include "xil_io.h"
#include "xaxidma.h"
#include "xil_cache.h"

#define UDP_PORT 5000
#define TCP_PORT 6000

// DMA configuration - you'll need to check xparameters.h for the actual name
// Common DMA parameter names (check which one exists in your xparameters.h):
// XPAR_AXIDMA_0_DEVICE_ID
// XPAR_AXI_DMA_0_DEVICE_ID  
// XPAR_XAXIDMA_0_DEVICE_ID
#ifdef XPAR_AXIDMA_0_DEVICE_ID
    #define DMA_DEVICE_ID XPAR_AXIDMA_0_DEVICE_ID
#elif defined(XPAR_AXI_DMA_0_DEVICE_ID)
    #define DMA_DEVICE_ID XPAR_AXI_DMA_0_DEVICE_ID
#elif defined(XPAR_XAXIDMA_0_DEVICE_ID)
    #define DMA_DEVICE_ID XPAR_XAXIDMA_0_DEVICE_ID
#else
    #error "DMA device ID not found. Check xparameters.h and add AXI DMA to block design"
#endif

#define DMA_BUFFER_SIZE (37 * 8)  // 37 x 64-bit words = 296 bytes per packet
#define DMA_BUFFER_ALIGN 64       // Cache line alignment

// AXI Lite control interface base address
#define PL_CTRL_BASE_ADDR 0x60000000

// Control register offsets
#define CTRL_REG_0_OFFSET   (0 * 4)   // Enable transmission, reset timestamp
#define CTRL_REG_1_OFFSET   (1 * 4)   // Loop count

// Status register offsets  
#define STATUS_REG_0_OFFSET  (22 * 4)  // Transmission status
#define STATUS_REG_1_OFFSET  (23 * 4)  // State counter
#define STATUS_REG_2_OFFSET  (24 * 4)  // Cycle counter  
#define STATUS_REG_3_OFFSET  (25 * 4)  // Packets sent
#define STATUS_REG_4_OFFSET  (26 * 4)  // Timestamp low [31:0]
#define STATUS_REG_5_OFFSET  (27 * 4)  // Timestamp high [63:32]
#define STATUS_REG_6_OFFSET  (28 * 4)  // packet_complete, loop_limit_reached

// Control register bits
#define CTRL_ENABLE_TRANSMISSION (1 << 0)
#define CTRL_RESET_TIMESTAMP     (1 << 1)

// Status register bits  
#define STATUS_PACKET_COMPLETE   (1 << 0)
#define STATUS_LOOP_LIMIT_REACHED (1 << 1)

static struct netif server_netif;
static struct udp_pcb *udp;
volatile int stream_enabled = 0;
static XTimer timer;
static XAxiDma axi_dma;

// DMA buffer (cache-aligned)
static u8 dma_buffer[DMA_BUFFER_SIZE] __attribute__((aligned(DMA_BUFFER_ALIGN)));
static u32_t packets_received_count = 0;

u32_t sys_now(void) {
    XTime now;
    XTime_GetTime(&now);
    return (u32_t)(now / (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 1000U));
}

// Initialize DMA
int init_dma(void) {
    XAxiDma_Config *config;
    int status;
    
    config = XAxiDma_LookupConfig(DMA_DEVICE_ID);
    if (!config) {
        xil_printf("ERROR: DMA config lookup failed\r\n");
        return XST_FAILURE;
    }
    
    status = XAxiDma_CfgInitialize(&axi_dma, config);
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: DMA initialization failed\r\n");
        return XST_FAILURE;
    }
    
    // Check for scatter gather mode (we want simple mode)
    if (XAxiDma_HasSg(&axi_dma)) {
        xil_printf("ERROR: DMA is in scatter gather mode, need simple mode\r\n");
        return XST_FAILURE;
    }
    
    xil_printf("DMA initialized successfully (simple mode)\r\n");
    return XST_SUCCESS;
}

// Control the PL data generator via AXI Lite
void pl_set_transmission(int enable) {
    u32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    if (enable) {
        ctrl_reg |= CTRL_ENABLE_TRANSMISSION;
        xil_printf("PL transmission ENABLED\r\n");
    } else {
        ctrl_reg &= ~CTRL_ENABLE_TRANSMISSION;
        xil_printf("PL transmission DISABLED\r\n");
    }
    
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
}

void pl_reset_timestamp(void) {
    u32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    ctrl_reg |= CTRL_RESET_TIMESTAMP;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
    
    usleep(1000);
    
    ctrl_reg &= ~CTRL_RESET_TIMESTAMP;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
    
    xil_printf("PL timestamp RESET\r\n");
}

void pl_set_loop_count(u32_t loop_count) {
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_1_OFFSET, loop_count);
    xil_printf("PL loop count set to %u\r\n", loop_count);
}

// Read PL status
void pl_print_status(void) {
    u32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    u32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);
    u32_t status2 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_2_OFFSET);
    u32_t status3 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_3_OFFSET);
    u32_t status4 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_4_OFFSET);
    u32_t status5 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_5_OFFSET);
    u32_t status6 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET);
    
    xil_printf("=== PL STATUS ===\r\n");
    xil_printf("Status0 (transmission_active): 0x%08X\r\n", status0);
    xil_printf("Status1 (state_counter): %u\r\n", status1 & 0x7F);
    xil_printf("Status2 (cycle_counter): %u\r\n", status2 & 0x3F);
    xil_printf("Status3 (packets_sent): %u\r\n", status3 & 0xFFFF);
    
    u64_t pl_timestamp = ((u64_t)status5 << 32) | status4;
    xil_printf("PL Timestamp: %llu\r\n", pl_timestamp);
    xil_printf("PS packets received: %u\r\n", packets_received_count);
    
    xil_printf("Status6: 0x%08X", status6);
    if (status6 & STATUS_PACKET_COMPLETE) xil_printf(" PACKET_COMPLETE");
    if (status6 & STATUS_LOOP_LIMIT_REACHED) xil_printf(" LOOP_LIMIT");
    xil_printf("\r\n");
}

// Start DMA transfer to receive data
void start_dma_transfer(void) {
    int status;
    
    // Invalidate cache before DMA transfer
    Xil_DCacheInvalidateRange((UINTPTR)dma_buffer, DMA_BUFFER_SIZE);
    
    // Start DMA transfer from stream to memory (S2MM)
    status = XAxiDma_SimpleTransfer(&axi_dma, (UINTPTR)dma_buffer, 
                                   DMA_BUFFER_SIZE, XAXIDMA_DEVICE_TO_DMA);
    
    if (status != XST_SUCCESS) {
        xil_printf("ERROR: DMA transfer start failed\r\n");
    }
}

// Check if DMA transfer is complete
int is_dma_transfer_complete(void) {
    return XAxiDma_Busy(&axi_dma, XAXIDMA_DEVICE_TO_DMA) ? 0 : 1;
}

// Process received DMA data and send via UDP
void process_dma_data(void) {
    // Ensure cache coherency
    Xil_DCacheInvalidateRange((UINTPTR)dma_buffer, DMA_BUFFER_SIZE);
    
    // Allocate UDP packet
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, DMA_BUFFER_SIZE, PBUF_RAM);
    if (!p) {
        xil_printf("ERROR: Could not allocate pbuf\r\n");
        return;
    }
    
    // Copy DMA data to UDP packet
    memcpy(p->payload, dma_buffer, DMA_BUFFER_SIZE);
    
    // Send the packet
    err_t result = udp_send(udp, p);
    if (result != ERR_OK) {
        xil_printf("ERROR: UDP send failed with code %d\r\n", result);
    } else {
        packets_received_count++;
        
        // Extract timestamp from received data for verification
        u64_t *words = (u64_t*)dma_buffer;
        u64_t magic = words[0];
        u64_t timestamp = words[1];
        
        if (packets_received_count % 1000 == 0) {
            xil_printf("Received packet %u: magic=0x%016llX, timestamp=%llu\r\n", 
                      packets_received_count, magic, timestamp);
        }
    }
    
    pbuf_free(p);
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
        if (strncmp(p->payload, "start", 5) == 0) {
            stream_enabled = 1;
            pl_set_transmission(1);
            xil_printf("Data streaming STARTED\r\n");
        } else if (strncmp(p->payload, "stop", 4) == 0) {
            stream_enabled = 0;
            pl_set_transmission(0);
            xil_printf("Data streaming STOPPED\r\n");
        } else if (strncmp(p->payload, "reset_timestamp", 15) == 0) {
            packets_received_count = 0;
            pl_reset_timestamp();
            xil_printf("Timestamp RESET (packet counter also reset)\r\n");
        } else if (strncmp(p->payload, "status", 6) == 0) {
            pl_print_status();
        } else if (strncmp(p->payload, "loop", 4) == 0) {
            char* space_ptr = strchr(p->payload, ' ');
            if (space_ptr) {
                u32_t loop_count = atoi(space_ptr + 1);
                pl_set_loop_count(loop_count);
            }
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

int main() {
    ip_addr_t ipaddr, netmask, gw;
    unsigned char mac_ethernet_address[] = { 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };
    
    init_platform();
    XilTickTimer_Init(&timer);
    
    // Initialize DMA
    if (init_dma() != XST_SUCCESS) {
        xil_printf("FATAL: DMA initialization failed\r\n");
        return -1;
    }
    
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
    xil_printf("DMA buffer size: %d bytes\r\n", DMA_BUFFER_SIZE);
    xil_printf("PL control base address: 0x%08X\r\n", PL_CTRL_BASE_ADDR);
    
    // Initialize PL
    pl_set_transmission(0);
    pl_set_loop_count(0);
    
    start_tcp_server();
    udp_stream_init();
    
    xil_printf("System ready. TCP commands:\r\n");
    xil_printf("  start - Begin streaming\r\n");
    xil_printf("  stop - Stop streaming\r\n");
    xil_printf("  reset_timestamp - Reset timestamp\r\n");
    xil_printf("  status - Show PL status\r\n");
    xil_printf("  loop <count> - Set loop count (0=infinite)\r\n");
    
    // Start initial DMA transfer
    start_dma_transfer();
    
    while (1) {
        xemacif_input(&server_netif);
        sys_check_timeouts();
        
        if (stream_enabled) {
            // Check if DMA transfer completed
            if (is_dma_transfer_complete()) {
                // Process the received data
                process_dma_data();
                
                // Start next DMA transfer
                start_dma_transfer();
            }
            
            // Print status every 3000 packets
            if (packets_received_count > 0 && packets_received_count % 3000 == 0) {
                static u32_t last_status_count = 0;
                if (packets_received_count != last_status_count) {
                    xil_printf("\r\n=== AUTO STATUS (after %u DMA packets) ===\r\n", packets_received_count);
                    pl_print_status();
                    xil_printf("=== END AUTO STATUS ===\r\n\r\n");
                    last_status_count = packets_received_count;
                }
            }
        }
        
        usleep(100); // 100 microseconds
    }
    
    cleanup_platform();
    return 0;
}