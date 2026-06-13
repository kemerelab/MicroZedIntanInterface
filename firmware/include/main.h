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

// Default UDP destination (can be changed via TCP command)
#define DEFAULT_UDP_DEST_IP_A   192
#define DEFAULT_UDP_DEST_IP_B   168
#define DEFAULT_UDP_DEST_IP_C   18
#define DEFAULT_UDP_DEST_IP_D   100
#define DEFAULT_UDP_DEST_PORT   5000

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
#define PACKET_HEADER_WORDS     10           // Magic number + timestamp
#define MAX_PACKET_DATA_WORDS   70          // Maximum data words (all 4 channels enabled)
#define MIN_PACKET_DATA_WORDS   18          // Minimum data words (1 channel enabled)
#define MAX_WORDS_PER_PACKET    (PACKET_HEADER_WORDS + MAX_PACKET_DATA_WORDS) // 74 words
#define MIN_WORDS_PER_PACKET    (PACKET_HEADER_WORDS + MIN_PACKET_DATA_WORDS) // 22 words

// ============================================================================
// AXI LITE CONTROL INTERFACE
// ============================================================================

// AXI Lite control interface base address
#define PL_CTRL_BASE_ADDR 0x40000000

// Number of PL control registers (must match axi_lite_registers N_CTRL --
// the status registers are read back starting right after the control block)
#define PL_N_CTRL_REGS      25

// Control register offsets
#define CTRL_REG_0_OFFSET   (0 * 4)   // Enable transmission, reset timestamp, debug mode
#define CTRL_REG_1_OFFSET   (1 * 4)   // Loop count
#define CTRL_REG_2_OFFSET   (2 * 4)   // Phase select, channel enable
#define CTRL_REG_MOSI_START_OFFSET  (CTRL_REG_0_OFFSET + (4 * 4)) // Offset for MOSI control words

// Aux command sequencer / override layer control registers (PL regs 22..24)
#define CTRL_REG_AUX_CTRL_OFFSET    (22 * 4)  // enable, bank select, fast settle/digout/dsp config
#define CTRL_REG_AUX_WRITE_OFFSET   (23 * 4)  // bank write port payload
#define CTRL_REG_AUX_STROBE_OFFSET  (24 * 4)  // write/inject toggles + inject command

// CTRL_REG_AUX_CTRL bit fields
#define AUX_CTRL_SEQ_EN             (1u << 0)
#define AUX_CTRL_BANK_SEL_SHIFT     1          // [3:1] bank select, 1 bit per slot
#define AUX_CTRL_BANK_SEL_MASK      (0x7u << 1)
#define AUX_CTRL_FS_SW              (1u << 4)  // software amp fast settle level
#define AUX_CTRL_FS_GPIO_EN         (1u << 5)
#define AUX_CTRL_FS_GPIO_SEL_SHIFT  6          // [8:6] digital_in pin select
#define AUX_CTRL_FS_GPIO_SEL_MASK   (0x7u << 6)
#define AUX_CTRL_DSP_SW             (1u << 9)  // software DSP-reset (CONVERT bit H) level
#define AUX_CTRL_DSP_GPIO_EN        (1u << 10)
#define AUX_CTRL_DSP_GPIO_SEL_SHIFT 11         // [13:11]
#define AUX_CTRL_DSP_GPIO_SEL_MASK  (0x7u << 11)
#define AUX_CTRL_DIGOUT_SW          (1u << 14) // software digout level
#define AUX_CTRL_DIGOUT_GPIO_EN     (1u << 15)
#define AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT 16      // [18:16]
#define AUX_CTRL_DIGOUT_GPIO_SEL_MASK  (0x7u << 16)
#define AUX_CTRL_REG3_STATIC_SHIFT  24         // [31:24] RHD Reg-3 bits D7..D1 (D0 = live digout)
#define AUX_CTRL_REG3_STATIC_MASK   (0xFFu << 24)

// CTRL_REG_AUX_WRITE packing: [15:0] data, [21:16] addr, [23:22] slot,
// [24] bank, [25] is_length (length record data = {2'b0,end[5:0],2'b0,loop[5:0]})
#define AUX_WRITE_PACK(slot, bank, is_len, addr, data) \
    ( ((uint32_t)(data) & 0xFFFFu)            | \
      (((uint32_t)(addr) & 0x3Fu)   << 16)    | \
      (((uint32_t)(slot) & 0x3u)    << 22)    | \
      (((uint32_t)(bank) & 0x1u)    << 24)    | \
      (((uint32_t)(is_len) & 0x1u)  << 25) )
#define AUX_LENGTH_DATA(loop_idx, end_idx) \
    ( ((uint32_t)(loop_idx) & 0x3Fu) | (((uint32_t)(end_idx) & 0x3Fu) << 8) )

// CTRL_REG_AUX_STROBE bits
#define AUX_STROBE_WRITE_TOGGLE     (1u << 0)
#define AUX_STROBE_INJECT_TOGGLE    (1u << 1)
#define AUX_STROBE_INJECT_CMD_SHIFT 16         // [31:16] injected command

// Bank size (entries per bank; matches aux_command_sequencer ADDR_W=6)
#define AUX_BANK_ENTRIES            64

// Status register offsets (status block starts after the control block)
#define STATUS_REG_BASE      (PL_N_CTRL_REGS * 4)
#define STATUS_REG_0_OFFSET  (STATUS_REG_BASE + 0 * 4)   // Dynamic status + counters
#define STATUS_REG_1_OFFSET  (STATUS_REG_BASE + 1 * 4)   // Reflected control parameters
#define STATUS_REG_2_OFFSET  (STATUS_REG_BASE + 2 * 4)   // Packets sent
#define STATUS_REG_3_OFFSET  (STATUS_REG_BASE + 3 * 4)   // Timestamp low [31:0]
#define STATUS_REG_4_OFFSET  (STATUS_REG_BASE + 4 * 4)   // Timestamp high [63:32]
#define STATUS_REG_5_OFFSET  (STATUS_REG_BASE + 5 * 4)   // Loop count (registered)
// Mirrored control registers in status space
#define STATUS_REG_6_OFFSET  (STATUS_REG_BASE + 6 * 4)   // Mirror of CTRL_REG_0 (enable, reset, etc.)
#define STATUS_REG_7_OFFSET  (STATUS_REG_BASE + 7 * 4)   // Mirror of CTRL_REG_1 (loop count)
#define STATUS_REG_8_OFFSET  (STATUS_REG_BASE + 8 * 4)   // Mirror of CTRL_REG_2 (phase select, debug mode)
#define STATUS_REG_9_OFFSET  (STATUS_REG_BASE + 9 * 4)   // Mirror of CTRL_REG_3 (reserved)
#define STATUS_REG_10_OFFSET (STATUS_REG_BASE + 10 * 4)  // BRAM write address + FIFO count (added by wrapper)
#define STATUS_REG_11_OFFSET (STATUS_REG_BASE + 11 * 4)  // Aux sequencer status
#define STATUS_REG_12_OFFSET (STATUS_REG_BASE + 12 * 4)  // Aux injected-command read result

// STATUS_REG_11 bit fields
#define AUX_STATUS_BANK_ACTIVE_MASK  0x7u      // [2:0] active bank per slot
#define AUX_STATUS_SEQ_EN            (1u << 3) // per-packet latched aux_seq_en
#define AUX_STATUS_FS_ACTIVE         (1u << 4)
#define AUX_STATUS_DIGOUT            (1u << 5)
#define AUX_STATUS_DSP_ACTIVE        (1u << 6)
#define AUX_STATUS_INJECT_ACK        (1u << 7) // toggles when an injection result lands
#define AUX_STATUS_IDX0_SHIFT        8         // [13:8]  slot-0 index
#define AUX_STATUS_IDX1_SHIFT        16        // [21:16] slot-1 index
#define AUX_STATUS_IDX2_SHIFT        24        // [29:24] slot-2 index
#define AUX_STATUS_IDX_MASK          0x3Fu

// RHD2000 SPI command encodings (datasheet-confirmed)
#define RHD_CMD_CONVERT(ch)     ((uint16_t)(((ch) & 0x3F) << 8))
#define RHD_CMD_WRITE(reg, val) ((uint16_t)(0x8000 | (((reg) & 0x3F) << 8) | ((val) & 0xFF)))
#define RHD_CMD_READ(reg)       ((uint16_t)(0xC000 | (((reg) & 0x3F) << 8)))

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
// TCP RESPONSE PROTOCOL
// ============================================================================

// Device type constants
#define DEVICE_TYPE_INTAN_INTERFACE    0x1000

// UDP packet format constants
#define UDP_PACKET_FORMAT_V1           0x0001

// Protocol version
#define PROTOCOL_VERSION               1
#define FIRMWARE_VERSION_MAJOR         1
#define FIRMWARE_VERSION_MINOR         0
#define FIRMWARE_VERSION_PATCH         0
#define FIRMWARE_VERSION_BUILD         0
#define FIRMWARE_VERSION_WORD          ((FIRMWARE_VERSION_MAJOR << 24) | \
                                       (FIRMWARE_VERSION_MINOR << 16) | \
                                       (FIRMWARE_VERSION_PATCH << 8) | \
                                       FIRMWARE_VERSION_BUILD)

// Response status codes
#define ACK_SUCCESS         0x06
#define ACK_ERROR           0x15

// Status response structure (98 bytes total)
typedef struct __attribute__((packed)) {
    // Version and identification (8 bytes)
    uint16_t version;
    uint16_t device_type;
    uint32_t firmware_version;
    
    // PL Hardware Status (22 bytes)
    uint64_t timestamp;
    uint32_t packets_sent;
    uint32_t bram_write_addr;
    uint16_t fifo_count;
    uint8_t  state_counter;
    uint8_t  cycle_counter;
    uint8_t  flags_pl;
    uint8_t  reserved1;
    
    // PS Software Status (28 bytes)
    uint32_t packets_received;
    uint32_t error_count;
    uint32_t udp_packets_sent;
    uint32_t udp_send_errors;
    uint32_t ps_read_addr;
    uint32_t packet_size;
    uint8_t  flags_ps;
    uint8_t  reserved2[3];
    
    // Current Configuration (16 bytes)
    uint32_t loop_count;
    uint8_t  phase0;
    uint8_t  phase1;
    uint8_t  channel_enable;
    uint8_t  debug_mode;
    uint32_t reserved3[2];
    
    // UDP Stream Information (12 bytes)
    uint32_t udp_dest_ip;
    uint16_t udp_dest_port;
    uint16_t udp_packet_format;
    uint32_t udp_bytes_sent;

    // Aux command sequencer status (12 bytes; appended -- keep net.py in sync)
    uint32_t aux_read_result;   // last injected command's response {cipo1, cipo0}
    uint8_t  aux_bank_active;   // [2:0] active bank per slot
    uint8_t  aux_flags;         // bit0 seq_en, bit1 fs_active, bit2 digout, bit3 dsp, bit4 inject_ack
    uint8_t  aux_idx[3];        // per-slot sequence index
    uint8_t  reserved5[3];

} status_response_t;

// Flag definitions
#define STATUS_PL_TRANSMISSION_ACTIVE  (1 << 0)
#define STATUS_PL_LOOP_LIMIT_REACHED   (1 << 1)
#define STATUS_PS_STREAM_ENABLED       (1 << 0)

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

// UDP configuration (can be changed via TCP command)
extern uint32_t udp_dest_ip;      // Network byte order
extern uint16_t udp_dest_port;

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

// COPI sequence selection
void pl_set_convert_sequence(void);
void pl_set_initialization_sequence(void);
void pl_set_cable_length_sequence(void);

// Aux command sequencer control (bank upload works DURING acquisition:
// write the standby bank, select it, then confirm the swap)
void pl_aux_write_word(int slot, int bank, int addr, uint16_t data);
void pl_aux_write_length(int slot, int bank, int loop_idx, int end_idx);
int  pl_aux_upload_bank(int slot, int bank, const uint16_t *cmds, int n, int loop_idx);
void pl_aux_select_bank(int slot, int bank);
int  pl_aux_confirm_bank(int slot, int bank, int timeout_ms);
void pl_aux_seq_enable(int enable);
int  pl_aux_seq_is_enabled(void);
void pl_aux_set_fast_settle(uint32_t cfg);   // AUX_CTRL fs/dsp bit fields [13:4]
void pl_aux_set_digout(uint32_t cfg);        // AUX_CTRL digout fields [18:14] + reg3_static [31:24]
int  pl_aux_inject(uint16_t cmd, uint32_t *result, int timeout_ms);
int  pl_read_rhd_register(int reg, uint32_t *result);
int  pl_write_rhd_register(int reg, uint8_t value, uint32_t *result);

// Command to go through all possible cable lengths for cable optimization
void pl_run_full_cable_test(void);

extern const uint16_t convert_cmd_sequence[35];
extern const uint16_t initialization_cmd_sequence[35];
extern const uint16_t cable_length_cmd_sequence[35];

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================

void benchmark_bram_reads(void);

// ============================================================================
// NETWORK FUNCTIONS
// ============================================================================

// Network functions (implemented in network.c)
uint32_t sys_now(void);
void start_tcp_server(void);
void udp_stream_init(void);

// UDP destination configuration
int udp_reconfigure_destination(uint32_t new_ip, uint16_t new_port);
int is_valid_udp_dest(uint32_t ip, uint16_t port);

// Status data collection
void collect_status_data(status_response_t* status);

// Hotplug support functions
void abort_tcp_connections(void);
void stop_tcp_server(void);
void stop_udp_stream(void);

#endif // MAIN_H