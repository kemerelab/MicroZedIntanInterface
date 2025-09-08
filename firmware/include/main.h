#ifndef MAIN_H
#define MAIN_H
#include <stdint.h>
#include "xparameters.h"
#include "xiltimer.h"
#include "lwip/udp.h"
#include "netif/xadapter.h"

// ============================================================================
// NETWORK CONFIGURATION
// ============================================================================
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
#define BYTES_PER_WORD          4           // 32-bit words
#define BRAM_SIZE_WORDS         16384       // 16384 x 32-bit words (64KB)
#define BRAM_SIZE_BYTES         (BRAM_SIZE_WORDS * BYTES_PER_WORD)   // 64KB

// Packet size calculation based on channel_enable bits
#define PACKET_HEADER_WORDS     4           // Magic number + timestamp
#define MAX_PACKET_DATA_WORDS   70          // Maximum data words (all 4 channels enabled)
#define MIN_PACKET_DATA_WORDS   18          // Minimum data words (1 channel enabled)
#define MAX_WORDS_PER_PACKET    (PACKET_HEADER_WORDS + MAX_PACKET_DATA_WORDS) // 74 words
#define MIN_WORDS_PER_PACKET    (PACKET_HEADER_WORDS + MIN_PACKET_DATA_WORDS) // 22 words

// ============================================================================
// AXI LITE CONTROL INTERFACE
// ============================================================================

// AXI Lite control interface base address
#define PL_CTRL_BASE_ADDR 0x40000000

// Control register offsets
#define CTRL_REG_0_OFFSET   (0 * 4)   // Enable transmission, reset timestamp, debug mode
#define CTRL_REG_1_OFFSET   (1 * 4)   // Loop count
#define CTRL_REG_2_OFFSET   (2 * 4)   // Phase select, channel enable
#define CTRL_REG_MOSI_START_OFFSET  (CTRL_REG_0_OFFSET + (4 * 4)) // Offset for MOSI control words

// Status register offsets
#define STATUS_REG_0_OFFSET  (22 * 4)  // Dynamic status + counters
#define STATUS_REG_1_OFFSET  (23 * 4)  // Reflected control parameters
#define STATUS_REG_2_OFFSET  (24 * 4)  // Packets sent
#define STATUS_REG_3_OFFSET  (25 * 4)  // Timestamp low [31:0]
#define STATUS_REG_4_OFFSET  (26 * 4)  // Timestamp high [63:32]
#define STATUS_REG_5_OFFSET  (27 * 4)  // Loop count (registered)
// Mirrored control registers in status space
#define STATUS_REG_6_OFFSET  (28 * 4)  // Mirror of CTRL_REG_0 (enable, reset, etc.)
#define STATUS_REG_7_OFFSET  (29 * 4)  // Mirror of CTRL_REG_1 (loop count)
#define STATUS_REG_8_OFFSET  (30 * 4)  // Mirror of CTRL_REG_2 (phase select, debug mode)
#define STATUS_REG_9_OFFSET  (31 * 4)  // Mirror of CTRL_REG_3 (reserved)
#define STATUS_REG_10_OFFSET (32 * 4)  // BRAM write address + FIFO count (added by wrapper)

// Control register bits
#define CTRL_ENABLE_TRANSMISSION (1 << 0)
#define CTRL_RESET_TIMESTAMP     (1 << 1)
#define CTRL_DEBUG_MODE          (1 << 3)   // Debug mode (send dummy data) [3]
#define CTRL_PHASE0_MASK         (0xF << 0) // phase0 [3:0] in CTRL_REG_2
#define CTRL_PHASE1_MASK         (0xF << 4) // phase1 [7:4] in CTRL_REG_2
#define CTRL_CHANNEL_ENABLE_MASK (0xF << 8) // channel_enable [11:8] in CTRL_REG_2

// Status register 0 bits (dynamic status + counters)
#define STATUS_TRANSMISSION_ACTIVE   (1 << 0)
#define STATUS_LOOP_LIMIT_REACHED    (1 << 1)
#define STATUS_STATE_COUNTER_MASK    (0x7F << 3)  // [9:3] - 7 bits
#define STATUS_STATE_COUNTER_SHIFT   3
#define STATUS_CYCLE_COUNTER_MASK    (0x3F << 11) // [16:11] - 6 bits  
#define STATUS_CYCLE_COUNTER_SHIFT   11

// Status register 1 bits (reflected control parameters)
#define STATUS_ENABLE_TRANSMISSION_REG  (1 << 0)
#define STATUS_RESET_TIMESTAMP_REG      (1 << 1)
#define STATUS_DEBUG_MODE_REG           (1 << 3)
#define STATUS_PHASE0_REG_MASK          (0xF << 12) // [15:12] - 4 bits
#define STATUS_PHASE0_REG_SHIFT         12
#define STATUS_PHASE1_REG_MASK          (0xF << 16) // [19:16] - 4 bits
#define STATUS_PHASE1_REG_SHIFT         16
#define STATUS_CHANNEL_ENABLE_REG_MASK  (0xF << 20) // [23:20] - 4 bits
#define STATUS_CHANNEL_ENABLE_REG_SHIFT 20

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// System state
extern XTimer timer;
extern struct netif server_netif;
extern struct udp_pcb *udp;
extern volatile int stream_enabled;
extern uint32_t packets_received_count;

// Command flags for main loop processing
extern volatile int enable_streaming_flag;
extern volatile int disable_streaming_flag;
extern volatile int reset_timestamp_flag;
extern volatile int cable_test_flag;

// BRAM state tracking
extern uint32_t ps_read_address;              // Current PS read position (word address)
extern uint32_t current_packet_size;          // Current expected packet size in 32-bit words
extern uint32_t current_channel_enable;       // Current channel enable setting

// Packet validation tracking
extern uint64_t expected_timestamp;
extern uint32_t error_count;
extern uint32_t timestamp_gaps;

// UDP transmission
extern uint32_t udp_packets_sent;
extern uint32_t udp_send_errors;

// ============================================================================
// CORE FUNCTIONS
// ============================================================================

// Streaming control
void handle_enable_streaming(void);
void handle_disable_streaming(void);
void handle_reset_timestamp(void);
void process_command_flags(void);

// Packet size calculation based on channel_enable
uint32_t calculate_packet_size(int channel_enable);
uint32_t calculate_data_words(int channel_enable);
void update_current_packet_size(void);

// Main loop
void network_maintenance_loop(void);

// ============================================================================
// PL CONTROL FUNCTIONS
// ============================================================================

// Basic PL control
void pl_set_transmission(int enable);
void pl_reset_timestamp(void);
void pl_set_loop_count(uint32_t loop_count);
void pl_set_phase_select(int phase0, int phase1);
void pl_set_debug_mode(int enable);
void pl_set_channel_enable(int channel_enable);

// Status reading
uint64_t pl_get_timestamp(void);
int pl_is_transmission_active(void);
uint32_t pl_get_packets_sent(void);
int pl_is_loop_limit_reached(void);
uint32_t pl_get_bram_write_address(void);
uint32_t pl_get_state_counter(void);
uint32_t pl_get_cycle_counter(void);

// Reflected control parameter reading
uint32_t pl_get_current_loop_count(void);
int pl_get_current_phase_select(int *phase0, int *phase1);
int pl_get_current_debug_mode(void);
int pl_get_current_channel_enable(void);
uint32_t pl_get_current_control_flags(void);

// Status display
void pl_print_status(void);

// Debug
void pl_dump_bram_data(uint32_t start_addr, uint32_t word_count);

// COPI command management
void pl_set_copi_commands(const uint16_t copi_array[35]);
int pl_set_copi_commands_safe(const uint16_t copi_array[35], const char* sequence_name);

// COPI sequence selection functions
void pl_set_convert_sequence(void);
void pl_set_initialization_sequence(void);
void pl_set_cable_length_sequence(void);

// Command to go through all possible cable lengths for cable optimization
void pl_run_full_cable_test(void);

extern const uint16_t convert_cmd_sequence[35];
extern const uint16_t initialization_cmd_sequence[35];
extern const uint16_t cable_length_cmd_sequence[35];

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

// BRAM benchmark function
void benchmark_bram_reads(void);

// ============================================================================
// NETWORK FUNCTIONS
// ============================================================================

// Network functions (implemented in network.c)
uint32_t sys_now(void);
void start_tcp_server(void);
void udp_stream_init(void);

#endif // MAIN_H