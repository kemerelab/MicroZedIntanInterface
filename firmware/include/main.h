// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

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
// Ports chosen to avoid common OS conflicts (macOS AirPlay 5000/7000, X11 6000,
// Windows Hyper-V reserved ranges) and to sit below every ephemeral-port floor.
#define UDP_PORT 0x6800   // 26624 -- unified data stream (broadband)
#define TCP_PORT 0x6900   // 26880 -- control channel

// Default UDP destination (can be changed via TCP command)
#define DEFAULT_UDP_DEST_IP_A   192
#define DEFAULT_UDP_DEST_IP_B   168
#define DEFAULT_UDP_DEST_IP_C   18
#define DEFAULT_UDP_DEST_IP_D   100
#define DEFAULT_UDP_DEST_PORT   0x6800   // 26624 (== UDP_PORT)

// ---- Device discovery beacon (subnet broadcast) ----------------------------
// Once fully initialized, the board broadcasts this fixed little-endian struct
// to <subnet>.255:BEACON_PORT ~1 Hz. A client uses it to (a) DISCOVER the board's
// IP (from the datagram source address / the ip field), (b) know the board is UP
// (readiness gate -- it only beacons after init), and (c) stay fully passive
// until it arrives (zero packets to the board during its fragile boot window).
// CONTRACT -- keep in sync across: network.c (build), remote/net.py (decode),
// and the ephys-socket plugin (decode). All fields naturally aligned; no padding.
#define BEACON_PORT     0x6880   // 26752 -- discovery beacon (subnet broadcast)
#define BEACON_MAGIC    0x4B4C4231u   // distinctive discovery magic ("KLB1")
#define BEACON_VERSION  1

typedef struct {
    uint32_t magic;       // BEACON_MAGIC
    uint32_t version;     // BEACON_VERSION
    uint32_t ip;          // board IPv4, network byte order (== datagram source)
    uint16_t tcp_port;    // control port (TCP_PORT, 0x6900)
    uint16_t udp_port;    // unified data port (UDP_PORT, 0x6800)
    uint32_t fw_version;  // FIRMWARE_VERSION_WORD (maj<<24|min<<16|patch<<8|build)
    uint8_t  mac[6];      // board MAC = unique device id
    uint16_t reserved;    // pad to 28 bytes / 4-byte multiple
} device_beacon_t;

void beacon_init(void);   // create the beacon PCB + compute the broadcast addr
void beacon_send(void);   // broadcast one beacon (call ~1 Hz while link is up)

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

// ============================================================================
// UNIFIED PACKET FORMAT (docs/unified-packet-format.md)
// ----------------------------------------------------------------------------
// The PL stream emits an 8 x 32-bit little-endian common header, then a
// stream-specific payload, on UDP_PORT. The host demuxes by stream_type (this
// broadband-only build produces stream_type=1 only). The PL builds the whole
// header in its BRAM; the PS does NO header math (DMA-into-pbuf rule). Keep this
// in sync with the PL builder (data_generator_core.sv) and net.py.
//
//   word 0  MAGIC      = 0xCAFEBABE
//   word 1  TYPE_VER   = stream_type[7:0] | version[15:8] | flags[31:16]
//   word 2  TS_LO      | word 3 TS_HI  (64-bit master timestamp)
//   word 4  SEQ        per-stream packet sequence (+1/packet; the loss check)
//   word 5  AUX0       stream-specific
//   word 6  AUX1       stream-specific
//   word 7  RSVD       0
#define UNIFIED_MAGIC           0xCAFEBABE
#define UNIFIED_VERSION         1
#define UNIFIED_HEADER_WORDS    8
#define STREAM_TYPE_BROADBAND   1

// Packet size calculation based on channel_enable bits.
// channel_enable is now 8 bits: [3:0] = port 0 streams, [7:4] = port 1 (dual
// cable). Max = 8 streams x 35 cycles / 2 = 140 data words.
//
// BROADBAND framing (stream_type=1): the 8-word common header + a 6-word
// broadband sub-block = 14 header words ahead of the data. The sub-block
// carries the previous-packet aux echoes + the 8 external-ADC breadcrumbs
// (see docs/unified-packet-format.md, NO DATA LOSS).
//   AUX0 (w5) = channel_enable[7:0] | num_data_words[23:8]
//   AUX1 (w6) = digital_in[7:0] | aux_flags[15:8] | echo0[31:16]
#define BB_SUBBLOCK_WORDS       6
#define PACKET_HEADER_WORDS     (UNIFIED_HEADER_WORDS + BB_SUBBLOCK_WORDS) // 14
#define MAX_PACKET_DATA_WORDS   140         // Maximum data words (all 8 streams = both ports)
#define MIN_PACKET_DATA_WORDS   18          // Minimum data words (1 stream enabled)
#define MAX_WORDS_PER_PACKET    (PACKET_HEADER_WORDS + MAX_PACKET_DATA_WORDS) // 154 words
#define MIN_WORDS_PER_PACKET    (PACKET_HEADER_WORDS + MIN_PACKET_DATA_WORDS) // 32 words

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
#define CTRL_REG_3_OFFSET   (3 * 4)   // Analytic chirp NCO config (see data_generator_core.sv)
#define CTRL_REG_MOSI_START_OFFSET  (CTRL_REG_0_OFFSET + (4 * 4)) // Offset for MOSI control words

// CTRL_REG_3 analytic-chirp config packing:
//   [0] chirp_mode, [1] reserved, [7:2] phase_stride (6b),
//   [19:8] f_span (12b -> f_max), [31:20] sweep_rate (12b -> freq_acc step/pkt)
#define CTRL_CHIRP_MODE          (1u << 0)
#define CTRL_CHIRP_STRIDE_SHIFT  2
#define CTRL_CHIRP_STRIDE_MASK   (0x3Fu << 2)
#define CTRL_CHIRP_FSPAN_SHIFT   8
#define CTRL_CHIRP_FSPAN_MASK    (0xFFFu << 8)
#define CTRL_CHIRP_RATE_SHIFT    20
#define CTRL_CHIRP_RATE_MASK     (0xFFFu << 20)

// Aux command engine / override control registers (PL regs 22..24)
#define CTRL_REG_AUX_CTRL_OFFSET    (22 * 4)  // prog bank select + fast settle/digout/dsp config
#define CTRL_REG_AUX_WRITE_OFFSET   (23 * 4)  // write port payload (RT reg / program)
#define CTRL_REG_AUX_STROBE_OFFSET  (24 * 4)  // write/inject toggles + inject command

// CTRL_REG_AUX_CTRL bit fields
// Program live-bank select: only slot 0 (the sole cycling program -- the accel
// sweep) has a bank, at reg22 bit 0. Slots 1 and 2 are fixed command registers
// (no bank).
#define AUX_CTRL_BANK_BIT(slot)     (1u << (slot))   // slot == 0 -> reg22 bit 0
#define AUX_CTRL_BANK_SEL_MASK      (0x1u << 0)       // [0] only
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

// CTRL_REG_AUX_WRITE packing: [15:0] data, [21:16] addr, [23:22] target,
// [24] bank, [25] is_length (length record data = {2'b0,end[5:0],2'b0,loop[5:0]}).
// target IS the slot index: 0 = slot-0 program (accel sweep; banked), 1 = slot-1
// fs command register, 2 = slot-2 inject command register (registers are single
// words; their bank/addr/is_length are ignored).
#define AUX_WRITE_PACK(target, bank, is_len, addr, data) \
    ( ((uint32_t)(data) & 0xFFFFu)            | \
      (((uint32_t)(addr) & 0x3Fu)   << 16)    | \
      (((uint32_t)(target) & 0x3u)  << 22)    | \
      (((uint32_t)(bank) & 0x1u)    << 24)    | \
      (((uint32_t)(is_len) & 0x1u)  << 25) )
#define AUX_LENGTH_DATA(loop_idx, end_idx) \
    ( ((uint32_t)(loop_idx) & 0x3Fu) | (((uint32_t)(end_idx) & 0x3Fu) << 8) )

// CTRL_REG_AUX_STROBE bits
#define AUX_STROBE_WRITE_TOGGLE     (1u << 0)
#define AUX_STROBE_INJECT_TOGGLE    (1u << 1)
#define AUX_STROBE_INJECT_CMD_SHIFT 16         // [31:16] injected command (slot-2 one-shot)

// Program size (entries per bank; matches aux_program ADDR_W=6)
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

// STATUS_REG_11 bit fields. Only slot 0 (the accel program) cycles: its bank bit
// and index are the only ones that move. Slots 1 and 2 are fixed registers -- their
// bank bits and index fields always read 0.
#define AUX_STATUS_BANK_ACTIVE_MASK  0x7u      // [2:0] active bank per slot (bit0=slot0 program)
#define AUX_STATUS_ENGINE_ON         (1u << 3) // always 1 (aux engine always on)
#define AUX_STATUS_FS_ACTIVE         (1u << 4)
#define AUX_STATUS_DIGOUT            (1u << 5)
#define AUX_STATUS_DSP_ACTIVE        (1u << 6)
#define AUX_STATUS_INJECT_ACK        (1u << 7) // toggles when an injection result lands
#define AUX_STATUS_IDX0_SHIFT        8         // [13:8]  slot-0 program index
#define AUX_STATUS_IDX1_SHIFT        16        // [21:16] slot-1 index (always 0: fs register)
#define AUX_STATUS_IDX2_SHIFT        24        // [29:24] slot-2 index (always 0: inject register)
#define AUX_STATUS_IDX_MASK          0x3Fu

// RHD2000 SPI command encodings (datasheet-confirmed)
#define RHD_CMD_CONVERT(ch)     ((uint16_t)(((ch) & 0x3F) << 8))
#define RHD_CMD_WRITE(reg, val) ((uint16_t)(0x8000 | (((reg) & 0x3F) << 8) | ((val) & 0xFF)))
#define RHD_CMD_READ(reg)       ((uint16_t)(0xC000 | (((reg) & 0x3F) << 8)))

// Control register bits
#define CTRL_ENABLE_TRANSMISSION (1 << 0)
#define CTRL_RESET_TIMESTAMP     (1 << 1)
#define CTRL_DEBUG_MODE          (1 << 3)   // Debug mode (send dummy data) [3]
// CTRL_REG_2 layout. All four CIPO cable-delay phases are adjacent in the low
// 16 bits; channel_enable follows:
//   [3:0] phase_a0, [7:4] phase_a1, [11:8] phase_b0, [15:12] phase_b1,
//   [23:16] channel_enable (8-bit: [19:16]=cable A, [23:20]=cable B)
#define CTRL_PHASE0_MASK         (0xF << 0)   // phase_a0 [3:0]   (cable A, line 0)
#define CTRL_PHASE1_MASK         (0xF << 4)   // phase_a1 [7:4]   (cable A, line 1)
#define CTRL_PHASE2_MASK         (0xF << 8)   // phase_b0 [11:8]  (cable B, line 0)
#define CTRL_PHASE3_MASK         (0xF << 12)  // phase_b1 [15:12] (cable B, line 1)
#define CTRL_CHANNEL_ENABLE_MASK (0xFF << 16) // channel_enable [23:16]
#define CTRL_CHANNEL_ENABLE_SHIFT 16

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
#define STATUS_CHANNEL_ENABLE_REG_MASK  (0xF << 20) // [23:20] - port-0 channel enable
#define STATUS_CHANNEL_ENABLE_REG_SHIFT 20
#define STATUS_CHANNEL_ENABLE_HI_REG_MASK  (0xF << 24) // [27:24] - port-1 channel enable
#define STATUS_CHANNEL_ENABLE_HI_REG_SHIFT 24

// ============================================================================
// TCP RESPONSE PROTOCOL
// ============================================================================

// Device type constants
#define DEVICE_TYPE_INTAN_INTERFACE    0x1000

// UDP packet format constants
#define UDP_PACKET_FORMAT_V1           0x0001

// Protocol version
#define PROTOCOL_VERSION               1
#define FIRMWARE_VERSION_MAJOR         2
#define FIRMWARE_VERSION_MINOR         0   // 2.0.0.0: aux command-path re-architecture -- MAJOR bump because the
                                           //      aux wire contract broke: the aux_seq_en path is retired
                                           //      (CMD_AUX_SEQ_EN 0x72 gone), the new aux_command_engine changes
                                           //      the reg22 semantics, and the accel sweep is always-on on slot 0
                                           //      (intra-packet reply @ data word 34). A 1.x host/plugin will not
                                           //      interoperate -- PL + firmware + plugin move together. slot 1 =
                                           //      fs register (default 'I' = READ(40)), slot 2 = inject register
                                           //      (default temp = CONVERT(49)); program bank-select -> reg22[0].
                                           //      No on-PL LFP/DSP engine: PL N_CTRL 25, N_STATUS 13,
                                           //      status_response_t wire size 264 B; keep net.py get_status +
                                           //      the _Static_assert in sync. Unified 8-word header UNCHANGED
                                           //      (broadband = stream_type=1).
                                           // 1.0.0.0: GLANCE broadband-only release (LFP/DSP stripped).
                                           //      --- prior internal history ---
                                           // 1.7: lwIP TX headroom -- n_tx_descriptors 64->256, mem_size
                                           //      128K->256K (BSP lwip220 config) to eliminate the rare
                                           //      udp_sendto ERR_MEM drops under ISR-stall catch-up bursts.
                                           //      NOT the pbuf pool (pbuf_alloc never failed). Firmware
                                           //      sources + struct unchanged (still 288 B); BSP regen only.
                                           // 1.6: OCM staging REVERTED (back to DDR; OCM didn't help --
                                           //      the recv->transmit tail is EMAC TX-done ISR preemption,
                                           //      not DDR contention). Adds TX-drop instrumentation: split
                                           //      udp_send_errors into bb/lfp pbuf-alloc-fail vs sendto-err
                                           //      (+ err code), first/last drop packet index, and an 8-deep
                                           //      drop-index ring; reports MEMP_NUM_PBUF. get_status -> 288 B.
                                           // 1.4: recv->transmit spike instrumentation -- split the
                                           //      timed window into CDMA / udp_sendto / other, capture
                                           //      the worst packet's breakdown, a 6-bucket recv->transmit
                                           //      histogram + over-budget count, and CMD_PERF_RESET to
                                           //      clear the window. get_status grows to 220 bytes.
                                           // 1.3: LFP default R=10 (3 kHz) + dual-MAC engine;
                                           //      analytic chirp NCO (CTRL_REG_3). get_status adds
                                           //      chirp config. Status wire = 168 bytes.
                                           // 1.2: AXI-CDMA read path; get_status config tracking
                                           //      (aux_ctrl + RHD register mirror); fast-settle/DSP/
                                           //      digout via TTL/GPIO.
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

    // DMA / performance instrumentation (24 bytes; appended -- keep net.py in sync)
    // Raw global-timer ticks; host converts to us with timer_hz.
    uint32_t dma_errors;        // CDMA read failures since boot
    uint32_t dma_ticks_last;    // last CDMA transfer (ticks)
    uint32_t dma_ticks_max;     // worst CDMA transfer (ticks)
    uint32_t loop_ticks_last;   // last receive->transmit (ticks)
    uint32_t loop_ticks_max;    // worst receive->transmit (ticks; vs 33us budget)
    uint32_t timer_hz;          // tick frequency, for host ticks->us conversion

    // Aux control register (CTRL_REG_22) read back -- the live fast-settle / DSP /
    // digout configuration (sw/gpio_en/pin per field), seq_en, bank, reg3_static.
    // Per the "get_status reports everything configurable" rule (see CLAUDE.md).
    uint32_t aux_ctrl;

    // RHD chip register mirror (commanded state of regs 0..21). Seeded from the
    // init sequence, updated on write_reg; regs 0/3 are the override-owned base
    // (live D5/digout are in aux_ctrl/aux_flags). 22 bytes.
    uint8_t  rhd_reg[22];

    // Analytic chirp NCO config (CTRL_REG_3 read-back). Per the "get_status
    // reports everything configurable" rule. 8 bytes.
    uint8_t  chirp_mode;        // 1 = chirp debug signal enabled
    uint8_t  chirp_stride;      // per-channel phase stride (6-bit)
    uint16_t chirp_fspan;       // f_max field (12-bit)
    uint16_t chirp_rate;        // sweep_rate field (12-bit)
    uint8_t  chirp_reserved[2];

    // recv->transmit spike instrumentation (52 bytes; appended -- keep net.py in
    // sync). All times are raw global-timer ticks (host converts with timer_hz).
    // The recv->transmit window (loop_ticks) is split into CDMA / udp_sendto /
    // other so the host can attribute the occasional ~40 us spike. The worst-case
    // snapshot is captured the instant a new loop_ticks_max is set, so we see WHAT
    // dominated that packet; the histogram + over_budget_count give the frequency
    // and shape of the tail. Cleared by CMD_PERF_RESET (see network.c).
    uint32_t send_ticks_last;   // last udp_sendto() call (ticks)
    uint32_t send_ticks_max;    // worst udp_sendto() call (ticks)
    uint32_t over_budget_count; // packets whose loop_ticks exceeded the 33.3 us budget
    uint32_t worst_pkt_index;   // packets_received_count at the worst-loop packet
    uint32_t worst_cdma_ticks;  // that packet's CDMA time (ticks)
    uint32_t worst_send_ticks;  // that packet's udp_sendto time (ticks)
    uint32_t worst_other_ticks; // that packet's loop - cdma - send (ticks)
    // recv->transmit histogram, microsecond bucket edges [<16,16-25,25-33,33-50,50-100,>=100]
    uint32_t loop_hist[6];      // counts per bucket

    // TX drop diagnostics (v1.6): split udp_send_errors by failure mode, and
    // record WHEN drops happen. Each zero-copy PBUF_REF send holds one MEMP_PBUF
    // entry (MEMP_NUM_PBUF) until the GEM TX-done reaps it; pbuf_alloc()==NULL =>
    // that pool was momentarily empty, a udp_sendto err (ERR_MEM) => no TX BD/mem.
    // Cleared by CMD_PERF_RESET. 56 bytes (keep net.py + _Static_assert in sync).
    uint32_t bb_pbuf_alloc_fail;  // broadband: pbuf_alloc returned NULL
    uint32_t bb_send_err;         // broadband: udp_sendto() != ERR_OK
    int32_t  bb_last_send_err;    // broadband: last err_t (ERR_MEM = -1, ...)
    uint32_t first_drop_pkt;      // packets_received_count at the first broadband drop
    uint32_t last_drop_pkt;       // ... at the most recent broadband drop
    uint32_t memp_num_pbuf;       // = MEMP_NUM_PBUF (shared zero-copy pool size)
    uint32_t drop_ring[8];        // last 8 broadband drop packet indices (clustering)

} status_response_t;

// recv->transmit histogram bucket count (keep in sync with loop_hist[] + net.py)
#define PERF_HIST_BUCKETS   6

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

// DMA + performance instrumentation (raw global-timer ticks; surfaced via
// get_status, converted to us host-side using perf_timer_hz)
extern uint32_t dma_errors;
extern uint32_t dma_ticks_last, dma_ticks_max;
extern uint32_t loop_ticks_last, loop_ticks_max;
extern uint32_t perf_timer_hz;
// recv->transmit spike instrumentation (raw ticks; see status_response_t)
extern uint32_t send_ticks_last, send_ticks_max;
extern uint32_t over_budget_count;
extern uint32_t worst_pkt_index, worst_cdma_ticks, worst_send_ticks, worst_other_ticks;
extern uint32_t loop_hist[PERF_HIST_BUCKETS];
// TX drop diagnostics (v1.6)
extern uint32_t bb_pbuf_alloc_fail, bb_send_err;
extern int32_t  bb_last_send_err;
extern uint32_t first_drop_pkt, last_drop_pkt;
extern uint32_t drop_ring[8];
extern uint32_t drop_ring_idx;
void perf_reset(void);   // clear sticky maxes + histogram + counts (CMD_PERF_RESET)

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
void pl_set_phase_select_b(int phase2, int phase3);  // port B (second cable)
void pl_set_debug_mode(int enable);
void pl_set_channel_enable(int channel_enable);
void pl_set_chirp(uint8_t mode, uint8_t stride, uint16_t fspan, uint16_t rate);  // CTRL_REG_3
uint32_t pl_get_chirp_cfg(void);  // raw CTRL_REG_3 read-back

// Tracked chirp config (mirrored into status_response_t)
extern uint8_t  chirp_cfg_mode, chirp_cfg_stride;
extern uint16_t chirp_cfg_fspan, chirp_cfg_rate;

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

// COPI command management. The table is 32 words -- one command per channel cycle
// (0..31). The 3 aux cycles (32..34) are sourced by the aux command engine.
void pl_set_copi_commands(const uint16_t copi_array[32]);
int pl_set_copi_commands_safe(const uint16_t copi_array[32], const char* sequence_name);

// COPI sequence selection
void pl_set_convert_sequence(void);
void pl_set_initialization_sequence(void);
void pl_rhd_shadow_init(void);          // seed the RHD register shadow from the init sequence
extern uint8_t rhd_reg_shadow[22];      // mirror of RHD regs 0..21 (commanded state)
void pl_set_cable_length_sequence(void);

// Aux command sequencer control (bank upload works DURING acquisition:
// write the standby bank, select it, then confirm the swap)
void pl_aux_write_word(int slot, int bank, int addr, uint16_t data);
void pl_aux_write_length(int slot, int bank, int loop_idx, int end_idx);
int  pl_aux_upload_bank(int slot, int bank, const uint16_t *cmds, int n, int loop_idx);
void pl_aux_select_bank(int slot, int bank);
int  pl_aux_confirm_bank(int slot, int bank, int timeout_ms);
void pl_aux_set_fast_settle(uint32_t cfg);   // AUX_CTRL fs/dsp bit fields [13:4]
void pl_aux_set_digout(uint32_t cfg);        // AUX_CTRL digout fields [18:14] + reg3_static [31:24]
int  pl_aux_inject(uint16_t cmd, uint32_t *result, int timeout_ms);
int  pl_read_rhd_register(int reg, uint32_t *result);
int  pl_write_rhd_register(int reg, uint8_t value, uint32_t *result);

// Command to go through all possible cable lengths for cable optimization
void pl_run_full_cable_test(void);

extern const uint16_t convert_cmd_sequence[32];
extern const uint16_t initialization_cmd_sequence[32];
extern const uint16_t cable_length_cmd_sequence[32];

// ============================================================================
// DEBUG FUNCTIONS
// ============================================================================


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