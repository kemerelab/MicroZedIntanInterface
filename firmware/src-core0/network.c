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
#include "pl_dma.h"     // CDMA read for the STFT spectrum (pl_dma_staging, pl_dma_read_addr)

/*
Binary Command Protocol:
Magic: 0xDEADBEEF
Format: [magic:u32][cmd_id:u32][ack_id:u32][param1:u32][param2:u32] = 20 bytes

Command Table:
ID   | Command          | Param1              | Param2
-----|------------------|---------------------|-------------------
0x01 | START            | unused              | unused  
0x02 | STOP             | unused              | unused
0x03 | RESET_TIMESTAMP  | unused              | unused
0x10 | SET_LOOP_COUNT   | loop_count          | unused
0x11 | SET_PHASE        | phase0              | phase1
0x12 | SET_DEBUG_MODE   | enable (0/1)        | unused
0x13 | SET_CHANNEL_ENABLE | 4 bits            | unused
0x20 | LOAD_CONVERT     | unused              | unused
0x21 | LOAD_INIT        | unused              | unused  
0x22 | LOAD_CABLE_TEST  | unused              | unused
0x30 | FULL_CABLE_TEST  | unused              | unused
0x40 | GET_STATUS       | unused              | unused
0x41 | DUMP_BRAM        | start_addr          | word_count
0x50 | SET_UDP_DEST     | ip_addr             | port
0x60 | PING             | unused              | unused
*/

#define CMD_MAGIC           0xDEADBEEF
#define CMD_PACKET_SIZE     20

#define CMD_START           0x01
#define CMD_STOP            0x02
#define CMD_RESET_TIMESTAMP 0x03
#define CMD_SET_LOOP_COUNT  0x10
#define CMD_SET_PHASE       0x11
#define CMD_SET_DEBUG_MODE  0x12
#define CMD_SET_CHANNEL_ENABLE 0x13
#define CMD_SET_PHASE_B     0x14   // port B (second cable) CIPO phase
#define CMD_LOAD_CONVERT    0x20
#define CMD_LOAD_INIT       0x21
#define CMD_LOAD_CABLE_TEST 0x22
#define CMD_FULL_CABLE_TEST 0x30
#define CMD_GET_STATUS      0x40
#define CMD_DUMP_BRAM       0x41
#define CMD_SET_UDP_DEST    0x50
#define CMD_PING            0x60
// Aux command sequencer / override layer (Epic A)
#define CMD_AUX_WRITE_WORD  0x70   // param1 = slot | bank<<8 | is_len<<16; param2 = addr<<16 | data
#define CMD_AUX_BANK_SELECT 0x71   // param1 = slot; param2 = bank (confirms swap before ACK)
#define CMD_AUX_SEQ_EN      0x72   // param1 = 0/1
#define CMD_READ_REGISTER   0x73   // param1 = reg; responds 4-byte {cipo1,cipo0} result
#define CMD_WRITE_REGISTER  0x74   // param1 = reg; param2 = value; responds 4-byte echo
#define CMD_SET_FAST_SETTLE 0x75   // param1 = amp: sw | gpio_en<<1 | pin<<4; param2 = dsp: same layout
#define CMD_SET_DIGOUT      0x76   // param1 = sw | gpio_en<<1 | pin<<4; param2 = reg3_static byte

// LFP/DSP engine (Tier-1). Set params + lane mask + coefficients while disabled,
// then enable. Coefficients stream one tap per CMD_LFP_WRITE_COEF.
#define CMD_LFP_ENABLE       0x80  // param1 = 0/1
#define CMD_LFP_SET_PARAMS   0x81  // param1 = decim_R, param2 = num_taps
#define CMD_LFP_SET_CHANNELS 0x82  // param1 = 8-bit lane mask
#define CMD_LFP_WRITE_COEF   0x83  // param1 = [0] clear-ptr-first; param2 = 18-bit signed coef

// STFT/Tier-2 engine: set params + channels + window while disabled, then enable.
#define CMD_STFT_ENABLE       0x84  // param1 = 0/1
#define CMD_STFT_SET_PARAMS   0x85  // param1 = nfft_log2, param2 = hop
#define CMD_STFT_SET_CHANNELS 0x86  // param1 = [0] clear-ptr-first; param2 = channel index (one lane)
#define CMD_STFT_WRITE_WINDOW 0x87  // param1 = [0] clear-ptr-first; param2 = Q15 Hann coeff (one tap)
#define CMD_UDP_BENCH        0x90  // param1 = payload bytes, param2 = n_packets (throughput test)
// Synthetic-data playback
#define CMD_PLAYBACK_LOAD    0x91  // param1 = byte offset, param2 = byte length; raw bytes follow
#define CMD_PLAYBACK_EN      0x92  // param1 = 0/1, param2 = loop length (samples)

#define ACK_SUCCESS         0x06
#define ACK_ERROR           0x15


typedef struct {
    uint32_t magic;
    uint32_t cmd_id;
    uint32_t ack_id;
    uint32_t param1;
    uint32_t param2;
} cmd_packet_t;

// Static receive buffer for handling partial commands.
// Explicitly word-aligned: it is cast to cmd_packet_t*, and the TCP payload
// it is filled from is NOT word-aligned (14-byte Ethernet header), so the
// alignment must come from this buffer itself.
static uint8_t recv_buffer[CMD_PACKET_SIZE] __attribute__((aligned(8)));
static uint16_t recv_buffer_pos = 0;

static void send_ack(struct tcp_pcb *tpcb, uint32_t ack_id, uint8_t status);  // fwd decl

// Bulk playback load: after CMD_PLAYBACK_LOAD, the next pb_load_remaining bytes of
// the TCP stream are written straight into the playback BRAM (not parsed as
// commands), and the load's ACK is sent once they all arrive.
static uint32_t pb_load_remaining = 0;   // bytes still expected
static uint32_t pb_load_addr      = 0;   // next BRAM byte address (word writes)
static uint32_t pb_load_word      = 0;   // partial-word accumulator
static uint8_t  pb_load_phase     = 0;   // 0..3 byte within the word
static uint32_t pb_load_ack_id    = 0;
static struct tcp_pcb *pb_load_pcb = NULL;

void pl_playback_load_arm(uint32_t byte_offset, uint32_t byte_len, uint32_t ack_id, void *tpcb) {
    if (byte_offset + byte_len > PLAYBACK_BRAM_SIZE_BYTES) byte_len = 0;  // bounds guard
    pb_load_addr      = PLAYBACK_BRAM_BASE_ADDR + byte_offset;
    pb_load_remaining = byte_len;
    pb_load_word = 0; pb_load_phase = 0;
    pb_load_ack_id = ack_id; pb_load_pcb = (struct tcp_pcb *)tpcb;
}

// Drain raw bytes from a pbuf into the playback BRAM; returns the new position.
static uint16_t pb_drain(const uint8_t *data, uint16_t pos, uint16_t len) {
    while (pos < len && pb_load_remaining > 0) {
        pb_load_word |= ((uint32_t)data[pos]) << (8 * pb_load_phase);
        pos++; pb_load_remaining--; pb_load_phase++;
        if (pb_load_phase == 4) {
            Xil_Out32(pb_load_addr, pb_load_word);
            pb_load_addr += 4; pb_load_word = 0; pb_load_phase = 0;
        }
    }
    if (pb_load_remaining == 0) {
        if (pb_load_phase != 0) { Xil_Out32(pb_load_addr, pb_load_word); pb_load_word = 0; pb_load_phase = 0; }
        if (pb_load_pcb) send_ack(pb_load_pcb, pb_load_ack_id, ACK_SUCCESS);
    }
    return pos;
}

// TCP connection tracking for hotplug support
static struct tcp_pcb *tcp_server_pcb = NULL;  // Listening PCB
static struct tcp_pcb *tcp_client_pcb = NULL;  // Active client connection

uint32_t sys_now(void) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(now / (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 1000U));
}

// ============================================================================
// UDP DESTINATION CONFIGURATION
// ============================================================================

int is_valid_udp_dest(uint32_t ip, uint16_t port) {
    if (ip == 0x00000000) return 0;  // 0.0.0.0
    if (ip == 0xFFFFFFFF) return 0;  // 255.255.255.255
    if (port == 0) return 0;
    
    uint8_t first_octet = (ip & 0xFF);
    if (first_octet == 127) return 0;  // Loopback
    
    return 1;
}

int udp_reconfigure_destination(uint32_t new_ip, uint16_t new_port) {
    if (!is_valid_udp_dest(new_ip, new_port)) {
        send_message("ERROR: Invalid UDP destination\r\n");
        return 0;
    }
    
    udp_dest_ip = new_ip;
    udp_dest_port = new_port;
    
    ip_addr_t dest_ip;
    dest_ip.addr = new_ip;
    send_message("UDP destination updated to %s:%d\r\n",
                 ip4addr_ntoa(&dest_ip), new_port);
    
    return 1;
}

void udp_stream_init() {
    ip_addr_t dest_ip;
    dest_ip.addr = udp_dest_ip;
    
    udp = udp_new();
    if (udp == NULL) {
        send_message("ERROR: Could not create UDP PCB\r\n");
        return;
    }
    
    send_message("UDP initialized (destination: %s:%d)\r\n",
                 ip4addr_ntoa(&dest_ip), udp_dest_port);
}

// ============================================================================
// UDP THROUGHPUT BENCHMARK: blast n_packets of `bytes` to udp_dest_ip:5002 as
// fast as the EMAC drains (zero-copy PBUF_REF, like the real stream path). The
// host (net.py udp_bench) counts received bytes/packets over wall-clock for the
// sustained MB/s + pps + loss. Measures the real large-packet ceiling vs the
// small-packet (broadband) operating point.
// ============================================================================
extern struct netif server_netif;
static struct udp_pcb *bench_pcb = NULL;
static uint8_t bench_buf[UDP_BENCH_MAX_BYTES] __attribute__((aligned(8)));

static void udp_bench_blast(uint32_t bytes, uint32_t npk) {
    if (bytes < 8) bytes = 8;
    if (bytes > UDP_BENCH_MAX_BYTES) bytes = UDP_BENCH_MAX_BYTES;
    if (bench_pcb == NULL) bench_pcb = udp_new();
    if (bench_pcb == NULL) return;
    ip_addr_t dst; dst.addr = udp_dest_ip;
    uint32_t sent = 0;
    for (uint32_t i = 0; i < npk; i++) {
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, bytes, PBUF_REF);
        if (p == NULL) { xemacif_input(&server_netif); continue; }
        p->payload = bench_buf;
        err_t e = udp_sendto(bench_pcb, p, &dst, UDP_BENCH_PORT);
        pbuf_free(p);
        if (e == ERR_OK) sent++;
        else xemacif_input(&server_netif);          // TX BDs full -> drain
        if ((i & 0x3F) == 0) xemacif_input(&server_netif);
    }
    send_message("UDP_BENCH: sent %u/%u x %u bytes\r\n", sent, npk, bytes);
}

// ============================================================================
// STATUS DATA COLLECTION
// ============================================================================

void collect_status_data(status_response_t* status) {
    memset(status, 0, sizeof(status_response_t));
    
    // Version and identification
    status->version = PROTOCOL_VERSION;
    status->device_type = DEVICE_TYPE_INTAN_INTERFACE;
    status->firmware_version = FIRMWARE_VERSION_WORD;
    
    // PL Hardware Status
    status->timestamp = pl_get_timestamp();
    status->packets_sent = pl_get_packets_sent();
    status->bram_write_addr = pl_get_bram_write_address();
    status->state_counter = pl_get_state_counter();
    status->cycle_counter = pl_get_cycle_counter();
    
    // PL Flags
    status->flags_pl = 0;
    if (pl_is_transmission_active()) {
        status->flags_pl |= STATUS_PL_TRANSMISSION_ACTIVE;
    }
    if (pl_is_loop_limit_reached()) {
        status->flags_pl |= STATUS_PL_LOOP_LIMIT_REACHED;
    }
    
    // PS Software Status
    status->packets_received = packets_received_count;
    status->error_count = error_count;
    status->udp_packets_sent = udp_packets_sent;
    status->udp_send_errors = udp_send_errors;
    status->ps_read_addr = ps_read_address;
    status->packet_size = current_packet_size;
    
    // PS Flags
    status->flags_ps = 0;
    if (stream_enabled) {
        status->flags_ps |= STATUS_PS_STREAM_ENABLED;
    }
    
    // Current Configuration
    status->loop_count = pl_get_current_loop_count();
    int phase0, phase1;
    pl_get_current_phase_select(&phase0, &phase1);
    status->phase0 = phase0;
    status->phase1 = phase1;
    status->channel_enable = pl_get_current_channel_enable();
    status->debug_mode = pl_get_current_debug_mode();
    
    // UDP Stream Information
    status->udp_dest_ip = udp_dest_ip;
    status->udp_dest_port = udp_dest_port;
    status->udp_packet_format = UDP_PACKET_FORMAT_V1;
    status->udp_bytes_sent = udp_packets_sent * current_packet_size * 4;
    
    // Get FIFO count
    uint32_t status10 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET);
    status->fifo_count = (status10 >> 14) & 0x1FF;

    // Aux command sequencer status
    uint32_t s11 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_11_OFFSET);
    status->aux_read_result = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_12_OFFSET);
    status->aux_bank_active = s11 & AUX_STATUS_BANK_ACTIVE_MASK;
    status->aux_flags       = (s11 >> 3) & 0x1F;
    status->aux_idx[0]      = (s11 >> AUX_STATUS_IDX0_SHIFT) & AUX_STATUS_IDX_MASK;
    status->aux_idx[1]      = (s11 >> AUX_STATUS_IDX1_SHIFT) & AUX_STATUS_IDX_MASK;
    status->aux_idx[2]      = (s11 >> AUX_STATUS_IDX2_SHIFT) & AUX_STATUS_IDX_MASK;

    // DMA / performance instrumentation (raw ticks + tick frequency)
    status->dma_errors      = dma_errors;
    status->dma_ticks_last  = dma_ticks_last;
    status->dma_ticks_max   = dma_ticks_max;
    status->loop_ticks_last = loop_ticks_last;
    status->loop_ticks_max  = loop_ticks_max;
    status->timer_hz        = perf_timer_hz;

    // Aux config read-back (fast-settle / DSP / digout settings live in CTRL_REG_22)
    status->aux_ctrl = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET);

    // RHD chip register mirror (commanded state of regs 0..21)
    memcpy(status->rhd_reg, rhd_reg_shadow, sizeof(status->rhd_reg));

    // LFP/DSP engine config (host-set) + live status
    status->lfp_enable       = lfp_cfg_enable;
    status->lfp_lane_mask    = lfp_cfg_lane_mask;
    status->lfp_decim_R      = lfp_cfg_decim_R;
    status->lfp_num_taps     = lfp_cfg_num_taps;
    status->lfp_packets_sent = lfp_udp_packets_sent;
    status->lfp_overrun      = (pl_lfp_read_status() >> 16) & 1;

    // STFT/Tier-2 engine config (host-set) + live status
    uint32_t stft_st         = pl_stft_read_status();
    status->stft_enable      = stft_cfg_enable;
    status->stft_nfft_log2   = stft_cfg_nfft_log2;
    status->stft_K           = STFT_K;
    status->stft_overflow    = (stft_st >> 31) & 1;
    status->stft_hop         = stft_cfg_hop;
    status->stft_reserved    = 0;
    status->stft_frame_seq   = stft_st & 0x3FFFFFFF;
    status->stft_packets_sent = stft_udp_packets_sent;

    // Synthetic-data playback config (host-set)
    status->playback_enable = playback_cfg_enable;
    status->playback_length = playback_cfg_length;
}

// ============================================================================
// TCP RESPONSE FUNCTIONS
// ============================================================================

static void send_ack(struct tcp_pcb *tpcb, uint32_t ack_id, uint8_t status) {
    uint8_t response[3];
    response[0] = (ack_id >> 8) & 0xFF;  // High byte
    response[1] = ack_id & 0xFF;         // Low byte  
    response[2] = status;
    tcp_write(tpcb, response, 3, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
}

static void send_response(struct tcp_pcb *tpcb, uint32_t ack_id, uint8_t status,
                         const void* data, uint16_t data_len) {
    uint8_t header[5];
    header[0] = (ack_id >> 8) & 0xFF;
    header[1] = ack_id & 0xFF;
    header[2] = status;
    header[3] = (data_len >> 8) & 0xFF;
    header[4] = data_len & 0xFF;
    
    tcp_write(tpcb, header, 5, TCP_WRITE_FLAG_COPY);
    
    if (data && data_len > 0) {
        tcp_write(tpcb, data, data_len, TCP_WRITE_FLAG_COPY);
    }
    
    tcp_output(tpcb);
}

// ============================================================================
// TCP COMMAND PROCESSING
// ============================================================================

static void process_command(struct tcp_pcb *tpcb, cmd_packet_t *cmd) {
    uint8_t status = ACK_SUCCESS;

    switch (cmd->cmd_id) {
        case CMD_START:
            command_flags->enable_streaming_flag = 1;
            send_message("Binary Command: START\r\n");
            break;
            
        case CMD_STOP:
            command_flags->disable_streaming_flag = 1;
            send_message("Binary Command: STOP\r\n");
            break;
            
        case CMD_RESET_TIMESTAMP:
            command_flags->reset_timestamp_flag = 1;
            send_message("Binary Command: RESET_TIMESTAMP\r\n");
            break;
            
        case CMD_SET_LOOP_COUNT:
            pl_set_loop_count(cmd->param1);
            send_message("Binary Command: SET_LOOP_COUNT %u\r\n", cmd->param1);
            break;
            
        case CMD_SET_PHASE:
            pl_set_phase_select(cmd->param1 & 0xFF, cmd->param2 & 0xFF);
            send_message("Binary Command: SET_PHASE %u %u\r\n",
                        cmd->param1 & 0xFF, cmd->param2 & 0xFF);
            break;

        case CMD_SET_PHASE_B:   // port B (second cable) CIPO phase
            pl_set_phase_select_b(cmd->param1 & 0xFF, cmd->param2 & 0xFF);
            send_message("Binary Command: SET_PHASE_B %u %u\r\n",
                        cmd->param1 & 0xFF, cmd->param2 & 0xFF);
            break;

        case CMD_SET_CHANNEL_ENABLE:
            pl_set_channel_enable(cmd->param1 & 0xFF);
            send_message("Binary Command: SET_CHANNEL_ENABLE 0x%02X\r\n", cmd->param1 & 0xFF);
            break;

        case CMD_SET_DEBUG_MODE:
            pl_set_debug_mode(cmd->param1 ? 1 : 0);
            send_message("Binary Command: SET_DEBUG_MODE %u\r\n", cmd->param1 ? 1 : 0);
            break;
            
        case CMD_LOAD_CONVERT:
            pl_set_convert_sequence();
            send_message("Binary Command: LOAD_CONVERT\r\n");
            break;
            
        case CMD_LOAD_INIT:
            pl_set_initialization_sequence();
            send_message("Binary Command: LOAD_INIT\r\n");
            break;
            
        case CMD_LOAD_CABLE_TEST:
            pl_set_cable_length_sequence();
            send_message("Binary Command: LOAD_CABLE_TEST\r\n");
            break;
            
        case CMD_FULL_CABLE_TEST:
            command_flags->cable_test_flag = 1;
            send_message("Binary Command: FULL_CABLE_TEST\r\n");
            break;

        case CMD_SET_UDP_DEST: {
            uint32_t new_ip = cmd->param1;
            uint16_t new_port = cmd->param2 & 0xFFFF;

            // Convert from little-endian (host) to network byte order
            new_ip = htonl(new_ip);

            if (udp_reconfigure_destination(new_ip, new_port)) {
                ip_addr_t dest_ip;
                dest_ip.addr = new_ip;
                send_message("Binary Command: SET_UDP_DEST %s:%u\r\n",
                            ip4addr_ntoa(&dest_ip), new_port);
            } else {
                status = ACK_ERROR;
                send_message("Binary Command: SET_UDP_DEST FAILED\r\n");
            }
            break;
        }

        case CMD_PING:
            // Lightweight link check - no send_message() to avoid UDP streaming lag
            // Just ACK immediately
            break;

        case CMD_GET_STATUS: {
            // NOTE: pl_print_status() (a ~16-line console flood) is deliberately
            // NOT called here -- it stalled the data pump on the print ring. The
            // host gets everything from the binary collect_status_data() response
            // below; the serial console reads the shared snapshot on core 1.
            status_response_t status_data;
            collect_status_data(&status_data);
            send_response(tpcb, cmd->ack_id, ACK_SUCCESS,
                         &status_data, sizeof(status_data));
            send_message("Binary Command: GET_STATUS (sent %d bytes)\r\n",
                        sizeof(status_data));
            return;  // Early return - don't call send_ack
        }
            
        case CMD_AUX_WRITE_WORD: {
            int slot   = cmd->param1 & 0x3;
            int bank   = (cmd->param1 >> 8) & 0x1;
            int is_len = (cmd->param1 >> 16) & 0x1;
            int addr   = (cmd->param2 >> 16) & 0x3F;
            uint16_t data = cmd->param2 & 0xFFFF;
            if (is_len)
                pl_aux_write_length(slot, bank, data & 0x3F, (data >> 8) & 0x3F);
            else
                pl_aux_write_word(slot, bank, addr, data);
            break;
        }

        case CMD_AUX_BANK_SELECT: {
            int slot = cmd->param1 & 0x3;
            int bank = cmd->param2 & 0x1;
            pl_aux_select_bank(slot, bank);
            if (!pl_aux_confirm_bank(slot, bank, 50))
                status = ACK_ERROR;
            send_message("Binary Command: AUX_BANK_SELECT slot=%d bank=%d %s\r\n",
                         slot, bank, status == ACK_SUCCESS ? "OK" : "TIMEOUT");
            break;
        }

        case CMD_AUX_SEQ_EN:
            pl_aux_seq_enable(cmd->param1 ? 1 : 0);
            send_message("Binary Command: AUX_SEQ_EN %u\r\n", cmd->param1 ? 1 : 0);
            break;

        case CMD_READ_REGISTER: {
            uint32_t result = 0;
            if (pl_read_rhd_register(cmd->param1 & 0x3F, &result)) {
                send_response(tpcb, cmd->ack_id, ACK_SUCCESS, &result, sizeof(result));
                send_message("Binary Command: READ_REGISTER %u -> 0x%08X\r\n",
                             cmd->param1 & 0x3F, result);
                return;  // response already sent
            }
            status = ACK_ERROR;
            send_message("Binary Command: READ_REGISTER %u FAILED\r\n", cmd->param1 & 0x3F);
            break;
        }

        case CMD_WRITE_REGISTER: {
            uint32_t result = 0;
            if (pl_write_rhd_register(cmd->param1 & 0x3F, cmd->param2 & 0xFF, &result)) {
                send_response(tpcb, cmd->ack_id, ACK_SUCCESS, &result, sizeof(result));
                send_message("Binary Command: WRITE_REGISTER %u 0x%02X -> 0x%08X\r\n",
                             cmd->param1 & 0x3F, cmd->param2 & 0xFF, result);
                return;
            }
            status = ACK_ERROR;
            send_message("Binary Command: WRITE_REGISTER %u FAILED\r\n", cmd->param1 & 0x3F);
            break;
        }

        case CMD_SET_FAST_SETTLE: {
            uint32_t cfg = 0;
            if (cmd->param1 & 0x1) cfg |= AUX_CTRL_FS_SW;
            if (cmd->param1 & 0x2) cfg |= AUX_CTRL_FS_GPIO_EN;
            cfg |= ((cmd->param1 >> 4) & 0x7) << AUX_CTRL_FS_GPIO_SEL_SHIFT;
            if (cmd->param2 & 0x1) cfg |= AUX_CTRL_DSP_SW;
            if (cmd->param2 & 0x2) cfg |= AUX_CTRL_DSP_GPIO_EN;
            cfg |= ((cmd->param2 >> 4) & 0x7) << AUX_CTRL_DSP_GPIO_SEL_SHIFT;
            pl_aux_set_fast_settle(cfg);
            send_message("Binary Command: SET_FAST_SETTLE 0x%X 0x%X\r\n",
                         cmd->param1, cmd->param2);
            break;
        }

        case CMD_SET_DIGOUT: {
            uint32_t cfg = 0;
            if (cmd->param1 & 0x1) cfg |= AUX_CTRL_DIGOUT_SW;
            if (cmd->param1 & 0x2) cfg |= AUX_CTRL_DIGOUT_GPIO_EN;
            cfg |= ((cmd->param1 >> 4) & 0x7) << AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT;
            cfg |= (cmd->param2 & 0xFF) << AUX_CTRL_REG3_STATIC_SHIFT;
            pl_aux_set_digout(cfg);
            send_message("Binary Command: SET_DIGOUT 0x%X reg3=0x%02X\r\n",
                         cmd->param1, cmd->param2 & 0xFF);
            break;
        }

        case CMD_DUMP_BRAM:
            command_flags->dump_bram_flag = 1;
            command_flags->start_bram_addr = cmd->param1;
            command_flags->word_count = cmd->param2;
            send_message("Binary Command: DUMP_BRAM %u %u\r\n",
                        cmd->param1, cmd->param2);
            break;

        case CMD_LFP_ENABLE:
            pl_lfp_set_config(cmd->param1 ? 1 : 0, lfp_cfg_lane_mask,
                              lfp_cfg_decim_R, lfp_cfg_num_taps);
            send_message("Binary Command: LFP_ENABLE %u\r\n", cmd->param1 ? 1 : 0);
            break;

        case CMD_LFP_SET_PARAMS:
            pl_lfp_set_config(lfp_cfg_enable, lfp_cfg_lane_mask,
                              cmd->param1 & 0xFF, cmd->param2 & 0xFF);
            send_message("Binary Command: LFP_SET_PARAMS decimR=%u num_taps=%u\r\n",
                         cmd->param1 & 0xFF, cmd->param2 & 0xFF);
            break;

        case CMD_LFP_SET_CHANNELS:
            pl_lfp_set_config(lfp_cfg_enable, cmd->param1 & 0xFF,
                              lfp_cfg_decim_R, lfp_cfg_num_taps);
            send_message("Binary Command: LFP_SET_CHANNELS 0x%02X\r\n", cmd->param1 & 0xFF);
            break;

        case CMD_LFP_WRITE_COEF:
            if (cmd->param1 & 0x1) pl_lfp_coef_begin();
            pl_lfp_coef_push((int32_t)(cmd->param2 << 14) >> 14);  // sign-extend 18-bit
            break;

        case CMD_STFT_ENABLE:
            pl_stft_set_config(cmd->param1 ? 1 : 0, stft_cfg_nfft_log2, stft_cfg_hop);
            send_message("Binary Command: STFT_ENABLE %u\r\n", cmd->param1 ? 1 : 0);
            break;

        case CMD_STFT_SET_PARAMS:
            pl_stft_set_config(stft_cfg_enable, cmd->param1 & 0xF, cmd->param2 & 0xFFFF);
            send_message("Binary Command: STFT_SET_PARAMS nfft_log2=%u hop=%u\r\n",
                         cmd->param1 & 0xF, cmd->param2 & 0xFFFF);
            break;

        case CMD_STFT_SET_CHANNELS:
            if (cmd->param1 & 0x1) pl_stft_sel_begin();
            pl_stft_sel_push(cmd->param2 & 0xFF);
            break;

        case CMD_STFT_WRITE_WINDOW:
            if (cmd->param1 & 0x1) pl_stft_window_begin();
            pl_stft_window_push((int16_t)(cmd->param2 & 0xFFFF));
            break;

        case CMD_UDP_BENCH:
            udp_bench_blast(cmd->param1, cmd->param2);
            break;

        case CMD_PLAYBACK_LOAD:
            // param1 = byte offset, param2 = byte length; the raw waveform bytes
            // follow in the stream. ACK is deferred until the bulk load completes.
            pl_playback_load_arm(cmd->param1, cmd->param2, cmd->ack_id, tpcb);
            return;   // do NOT send_ack here

        case CMD_PLAYBACK_EN:
            pl_playback_set_config(cmd->param1 ? 1 : 0, cmd->param2);
            send_message("Binary Command: PLAYBACK_EN %u len=%u\r\n",
                         cmd->param1 ? 1 : 0, cmd->param2);
            break;

        default:
            status = ACK_ERROR;
            send_message("Binary Command: UNKNOWN (0x%08X)\r\n", cmd->cmd_id);
            break;
    }
    
    send_ack(tpcb, cmd->ack_id, status);
}

// ============================================================================
// TCP CALLBACKS
// ============================================================================

static void tcp_err_cb(void *arg, err_t err) {
    (void)arg;
    (void)err;

    // Connection error or abort - clear client tracking
    tcp_client_pcb = NULL;
    recv_buffer_pos = 0;
    send_message("TCP connection error/closed\r\n");
}

err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    (void)arg;
    (void)err;

    if (!p) {
        // Client closed connection gracefully
        tcp_close(tpcb);
        tcp_client_pcb = NULL;
        recv_buffer_pos = 0;
        send_message("TCP connection closed by client\r\n");
        return ERR_OK;
    }
    
    uint8_t *data = (uint8_t *)p->payload;
    uint16_t data_len = p->len;
    uint16_t data_pos = 0;
    
    // 0) Bulk playback load armed in a prior callback: drain raw bytes into BRAM.
    if (pb_load_remaining > 0) {
        data_pos = pb_drain(data, data_pos, data_len);
    }

    // First, handle any incomplete command from previous packet (not while loading)
    if (pb_load_remaining == 0 && recv_buffer_pos > 0) {
        uint16_t bytes_needed = CMD_PACKET_SIZE - recv_buffer_pos;
        uint16_t bytes_available = data_len - data_pos < bytes_needed ? data_len - data_pos : bytes_needed;

        memcpy(&recv_buffer[recv_buffer_pos], &data[data_pos], bytes_available);
        recv_buffer_pos += bytes_available;
        data_pos += bytes_available;

        // Check if we now have a complete command
        if (recv_buffer_pos == CMD_PACKET_SIZE) {
            cmd_packet_t *cmd = (cmd_packet_t *)recv_buffer;
            if (cmd->magic == CMD_MAGIC) {
                process_command(tpcb, cmd);
            }
            recv_buffer_pos = 0;  // Reset for next incomplete command
        }
    }

    // Process complete commands from the TCP buffer.
    //
    // IMPORTANT: copy each command into a word-aligned struct instead of
    // casting into the pbuf payload. The TCP payload sits at a halfword
    // boundary (14-byte Ethernet header), so a cmd_packet_t* into it is
    // misaligned. Plain LDRs tolerate that on the A9 (SCTLR.A=0), which is
    // why this "worked" for years -- but the compiler is allowed to merge
    // adjacent field reads into LDRD, which ALIGNMENT-FAULTS on non-word
    // addresses regardless. -O3 did exactly that for one handler (the aux
    // bank-select case) and hard-wedged the CPU in the abort handler.
    //
    // CMD_PLAYBACK_LOAD arms a bulk transfer: when it does, the rest of the
    // stream is raw waveform bytes, so drain instead of parsing them as commands.
    while (pb_load_remaining == 0 && data_pos + CMD_PACKET_SIZE <= data_len) {
        cmd_packet_t cmd_aligned __attribute__((aligned(8)));
        memcpy(&cmd_aligned, &data[data_pos], CMD_PACKET_SIZE);
        if (cmd_aligned.magic == CMD_MAGIC) {
            process_command(tpcb, &cmd_aligned);
            data_pos += CMD_PACKET_SIZE;
            if (pb_load_remaining > 0) data_pos = pb_drain(data, data_pos, data_len);
        } else {
            // Skip bad data and look for next magic
            data_pos++;
        }
    }

    // Copy any remaining partial command to recv_buffer (not while loading)
    if (pb_load_remaining == 0) {
        uint16_t remaining_bytes = data_len - data_pos;
        if (remaining_bytes > 0) {
            memcpy(recv_buffer, &data[data_pos], remaining_bytes);
            recv_buffer_pos = remaining_bytes;
        }
    }

    tcp_recved(tpcb, p->len);
    pbuf_free(p);
    return ERR_OK;
}

err_t tcp_accept_cb(void *arg, struct tcp_pcb *newpcb, err_t err) {
    (void)arg;
    (void)err;

    // Reset receive buffer for new connection
    recv_buffer_pos = 0;

    // Track this client connection
    tcp_client_pcb = newpcb;

    // Set up callbacks
    tcp_recv(newpcb, tcp_recv_cb);
    tcp_err(newpcb, tcp_err_cb);

    send_message("Binary TCP connection established\r\n");
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

    // Store globally for hotplug support
    tcp_server_pcb = pcb;

    send_message("Binary TCP command server started on port %d\r\n", TCP_PORT);
    send_message("Commands use 20-byte binary format with magic 0xDEADBEEF\r\n");
}

// ============================================================================
// HOTPLUG SUPPORT FUNCTIONS
// ============================================================================

void abort_tcp_connections(void) {
    // Abort active client connection immediately
    if (tcp_client_pcb != NULL) {
        tcp_abort(tcp_client_pcb);
        tcp_client_pcb = NULL;
        recv_buffer_pos = 0;
        send_message("TCP client connection aborted\r\n");
    }
}

void stop_tcp_server(void) {
    // Abort any active client connections first
    abort_tcp_connections();

    // Close the listening server
    if (tcp_server_pcb != NULL) {
        tcp_close(tcp_server_pcb);
        tcp_server_pcb = NULL;
        send_message("TCP server stopped\r\n");
    }
}

void stop_udp_stream(void) {
    if (udp != NULL) {
        udp_remove(udp);
        udp = NULL;
        send_message("UDP stream stopped\r\n");
    }
}

// ============================================================================
// LFP/DSP STREAM (Tier-1): drain the LFP output BRAM -> UDP on LFP_UDP_PORT.
// Each decimation frame = popcount(lane_mask) streams x 32 ch x 16-bit, packed
// 2/word, written sequentially to the ring at 0x84000000. The PL exposes its
// write pointer in STATUS_REG_13; we drain whole frames and wrap each in a
// compact LFP packet (own magic, so a misrouted datagram is detectable).
// ============================================================================
static struct udp_pcb *lfp_pcb = NULL;
static uint32_t lfp_read_word = 0;
static uint32_t lfp_frame_seq = 0;
uint32_t lfp_udp_packets_sent = 0;
// 6-word header + max frame (8 streams x 32 ch / 2 = 128 words).
static uint32_t lfp_pktbuf[6 + 128] __attribute__((aligned(8)));

void lfp_stream_init(void) {
    lfp_pcb = udp_new();
    lfp_read_word = 0;
    lfp_frame_seq = 0;
    lfp_udp_packets_sent = 0;
    if (lfp_pcb == NULL) send_message("ERROR: Could not create LFP UDP PCB\r\n");
}

void lfp_stream_service(void) {
    if (!lfp_cfg_enable || lfp_pcb == NULL) return;
    int nlanes = __builtin_popcount(lfp_cfg_lane_mask);
    if (nlanes == 0) return;
    uint32_t frame_words = (uint32_t)nlanes * 16;       // popcount*32 samples / 2 per word
    const uint32_t mask = LFP_BRAM_SIZE_WORDS - 1;

    uint32_t st = pl_lfp_read_status();
    uint32_t wr_word = (st & 0xFFFF) >> 2;              // byte addr -> 32-bit word index
    uint8_t  overrun = (st >> 16) & 1;

    int budget = 8;   // cap frames per call so the broadband loop isn't starved
    while (budget-- > 0) {
        if (((wr_word - lfp_read_word) & mask) < frame_words) break;  // no full frame yet

        lfp_pktbuf[0] = 0x1F1FBEEF;                     // LFP magic (low)
        lfp_pktbuf[1] = 0xCAFEBABE;                     // magic (high)
        uint64_t ts = (uint64_t)lfp_frame_seq * lfp_cfg_decim_R;  // ~broadband packet index
        lfp_pktbuf[2] = (uint32_t)ts;
        lfp_pktbuf[3] = (uint32_t)(ts >> 32);
        lfp_pktbuf[4] = (uint32_t)lfp_cfg_lane_mask
                      | ((uint32_t)lfp_cfg_decim_R  << 8)
                      | ((uint32_t)lfp_cfg_num_taps << 16)
                      | ((uint32_t)overrun          << 24);
        lfp_pktbuf[5] = lfp_frame_seq;
        for (uint32_t i = 0; i < frame_words; i++)
            lfp_pktbuf[6 + i] = Xil_In32(LFP_BRAM_BASE_ADDR + (((lfp_read_word + i) & mask) << 2));

        uint32_t total = (6 + frame_words) * 4;
        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_REF);
        if (p != NULL) {
            p->payload = (void*)lfp_pktbuf;
            ip_addr_t dst; dst.addr = udp_dest_ip;
            udp_sendto(lfp_pcb, p, &dst, LFP_UDP_PORT);
            pbuf_free(p);
            lfp_udp_packets_sent++;
        }
        lfp_read_word = (lfp_read_word + frame_words) & mask;
        lfp_frame_seq++;
    }
}

// ============================================================================
// STFT/Tier-2 STREAM: poll the pass counter (STATUS_REG_14); when it advances,
// push the whole Hermitian spectrum (K x (N/2+1) complex float32, compact stride
// at 0x88000000) as ONE jumbo UDP packet on STFT_UDP_PORT. The engine rewrites
// the BRAM each pass; passes are >> the read time, but we re-check the counter
// after the copy and drop any frame torn by a new pass (cheap integrity guard).
// ============================================================================
static struct udp_pcb *stft_pcb = NULL;
static uint32_t stft_last_seq = 0xFFFFFFFF;
uint32_t stft_udp_packets_sent = 0;
// 8-word header + max payload (K x (MAX_N/2+1) complex float32).
// 8-word header + max payload (K x (MAX_N/2+1) complex float32).
static uint32_t stft_pktbuf[8 + STFT_K * (STFT_MAX_N/2 + 1) * 2] __attribute__((aligned(8)));

void stft_stream_init(void) {
    stft_pcb = udp_new();
    stft_last_seq = 0xFFFFFFFF;
    stft_udp_packets_sent = 0;
    if (stft_pcb == NULL) send_message("ERROR: Could not create STFT UDP PCB\r\n");
}

void stft_stream_service(void) {
    if (!stft_cfg_enable || stft_pcb == NULL) return;
    // Rate-limit the spectrum stream to ~30/s. The ~2100-word single-beat read +
    // 8.5 KB jumbo send is expensive, and at small hops (e.g. hop=1 -> 2 kHz) doing
    // it every loop iteration starves core-0 (broadband stutters, control commands
    // time out). 30 spectra/s is plenty for a heat-map display. (A CDMA read would
    // remove the read cost entirely, but the CDMA can't reach this BRAM on the
    // current bitstream -- it hangs -- so we read single-beat + rate-limit. TODO:
    // fix the CDMA->STFT-BRAM route to unlock hop=1.) Cheap to skip (XTime compare).
    static XTime stft_last_send_t = 0;
    XTime now_t; XTime_GetTime(&now_t);
    if (stft_last_send_t && perf_timer_hz &&
        (now_t - stft_last_send_t) < (perf_timer_hz / 30)) return;
    uint32_t st  = pl_stft_read_status();
    uint32_t seq = st & 0x3FFFFFFF;
    if (seq == stft_last_seq) return;              // no new spectrum
    stft_last_send_t = now_t;                       // commit -> reset the rate gate

    uint32_t N      = 1u << stft_cfg_nfft_log2;
    uint32_t nbins  = N / 2 + 1;
    uint32_t pwords = (uint32_t)STFT_K * nbins * 2; // complex float32 (re,im)
    if (pwords > (uint32_t)STFT_K * (STFT_MAX_N/2 + 1) * 2) return;  // config guard

    uint64_t ts = (uint64_t)seq * (stft_cfg_hop ? stft_cfg_hop : 1);
    stft_pktbuf[0] = 0x5DEC7A00;                    // STFT magic (low)
    stft_pktbuf[1] = 0xCAFEBABE;                    // magic (high)
    stft_pktbuf[2] = (uint32_t)ts;
    stft_pktbuf[3] = (uint32_t)(ts >> 32);
    stft_pktbuf[4] = (uint32_t)stft_cfg_nfft_log2
                   | ((uint32_t)STFT_K << 8)
                   | (((st >> 31) & 1u) << 24);     // [31:24] flags: bit0 = overflow
    stft_pktbuf[5] = seq;
    stft_pktbuf[6] = nbins | ((uint32_t)stft_cfg_hop << 16);
    stft_pktbuf[7] = 0;
    for (uint32_t i = 0; i < pwords; i++)
        stft_pktbuf[8 + i] = Xil_In32(STFT_BRAM_BASE_ADDR + (i << 2));

    // integrity: if a new pass started during the copy, the spectrum may be torn.
    if ((pl_stft_read_status() & 0x3FFFFFFF) != seq) return;
    stft_last_seq = seq;

    uint32_t total = (8 + pwords) * 4;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, total, PBUF_REF);
    if (p != NULL) {
        p->payload = (void*)stft_pktbuf;
        ip_addr_t dst; dst.addr = udp_dest_ip;
        udp_sendto(stft_pcb, p, &dst, STFT_UDP_PORT);
        pbuf_free(p);
        stft_udp_packets_sent++;
    }
}
