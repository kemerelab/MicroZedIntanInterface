#ifndef MAIN_H
#define MAIN_H

#include "xparameters.h"
#include "xiltimer.h"
#include "lwip/udp.h"
#include "netif/xadapter.h"

// Network configuration
#define UDP_PORT 5000
#define TCP_PORT 6000

// ============================================================================
// MULTICORE CONFIGURATION
// ============================================================================
#define ARM1_BASEADDR 0xFFFFFFF0
#define ARM1_STARTADR 0x20000000
#define sev() __asm__("sev")

// ============================================================================
// BRAM CONFIGURATION
// ============================================================================

// BRAM base address (connected to M_AXI_GP1)
#define BRAM_BASE_ADDR          0x80000000

// BRAM layout - matches FPGA configuration
#define WORDS_PER_PACKET        144         // 4 header + (35 cycles * 4 data words)
#define BYTES_PER_WORD          4           // 32-bit words
#define BYTES_PER_PACKET        (WORDS_PER_PACKET * BYTES_PER_WORD)  // 576 bytes
#define BRAM_SIZE_WORDS         16384       // 16384 x 32-bit words (64KB)
#define BRAM_SIZE_BYTES         (BRAM_SIZE_WORDS * BYTES_PER_WORD)   // 64KB
#define MAX_PACKETS_IN_BRAM     (BRAM_SIZE_WORDS / WORDS_PER_PACKET) // ~113 packets

// ============================================================================
// AXI LITE CONTROL INTERFACE
// ============================================================================

// AXI Lite control interface base address
#define PL_CTRL_BASE_ADDR 0x40000000

// Control register offsets
#define CTRL_REG_0_OFFSET   (0 * 4)   // Enable transmission, reset timestamp
#define CTRL_REG_1_OFFSET   (1 * 4)   // Loop count
#define CTRL_REG_MOSI_START_OFFSET  (CTRL_REG_0_OFFSET + (4 * 4)) // Offset for MOSI control words

// Status register offsets
#define STATUS_REG_0_OFFSET  (22 * 4)  // Transmission status
#define STATUS_REG_1_OFFSET  (23 * 4)  // State counter
#define STATUS_REG_2_OFFSET  (24 * 4)  // Cycle counter  
#define STATUS_REG_3_OFFSET  (25 * 4)  // Packets sent
#define STATUS_REG_4_OFFSET  (26 * 4)  // Timestamp low [31:0]
#define STATUS_REG_5_OFFSET  (27 * 4)  // Timestamp high [63:32]
#define STATUS_REG_6_OFFSET  (28 * 4)  // BRAM write address + FIFO count

// Control register bits
#define CTRL_ENABLE_TRANSMISSION (1 << 0)
#define CTRL_RESET_TIMESTAMP     (1 << 1)

// Status register bits  
#define STATUS_TRANSMISSION_ACTIVE   (1 << 0)
#define STATUS_LOOP_LIMIT_REACHED    (1 << 1)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System state
extern XTimer timer;
extern struct netif server_netif;
extern struct udp_pcb *udp;
extern volatile int stream_enabled;
extern u32_t packets_received_count;

// Command flags for main loop processing
extern volatile int enable_streaming_flag;
extern volatile int disable_streaming_flag;
extern volatile int reset_timestamp_flag;

// BRAM state tracking
extern u32 ps_read_address;              // Current PS read position (word address)

// Packet validation tracking
extern u64 expected_timestamp;
extern u32 error_count;
extern u32 timestamp_gaps;

// UDP transmission
extern u32 udp_packets_sent;
extern u32 udp_send_errors;

// ============================================================================
// CORE FUNCTIONS
// ============================================================================

// Streaming control
void handle_enable_streaming(void);
void handle_disable_streaming(void);
void handle_reset_timestamp(void);
void process_command_flags(void);

// Main loop
void network_maintenance_loop(void);

// ============================================================================
// PL CONTROL FUNCTIONSvoid pl_dump_bram_data(u32 start_addr, u32 word_count) {

// ============================================================================

// Basic PL control
void pl_set_transmission(int enable);
void pl_reset_timestamp(void);
void pl_set_loop_count(u32_t loop_count);

// Status reading
u64_t pl_get_timestamp(void);
int pl_is_transmission_active(void);
u32_t pl_get_packets_sent(void);
int pl_is_loop_limit_reached(void);
u32 pl_get_bram_write_address(void);

// Status display
void pl_print_status(void);

// Debug
void pl_dump_bram_data(u32 start_addr, u32 word_count);


void pl_set_mosi_channel_sequence(void);
void pl_set_copi_commands(const u16 copi_array[35]);

extern const u16 convert_cmd_sequence[35];
extern const u16 initialization_cmd_sequence[35];
extern const u16 cable_length_cmd_sequence[35];
extern const u16 mosi_test_pattern[35];

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

// BRAM benchmark function
void benchmark_bram_reads(void);

// ============================================================================
// NETWORK FUNCTIONS
// ============================================================================

// Network functions (implemented in network.c)
u32_t sys_now(void);
void start_tcp_server(void);
void udp_stream_init(void);

#endif // MAIN_H