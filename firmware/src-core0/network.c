// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

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
// 0x72 retired (was CMD_AUX_SEQ_EN): the aux command engine is always on
#define CMD_READ_REGISTER   0x73   // param1 = reg; responds 4-byte {cipo1,cipo0} result
#define CMD_WRITE_REGISTER  0x74   // param1 = reg; param2 = value; responds 4-byte echo
#define CMD_SET_FAST_SETTLE 0x75   // param1 = amp: sw | gpio_en<<1 | pin<<4; param2 = dsp: same layout
#define CMD_SET_DIGOUT      0x76   // param1 = sw | gpio_en<<1 | pin<<4; param2 = reg3_static byte
#define CMD_SET_CHIRP       0x77   // param1 = mode | stride<<8; param2 = fspan | rate<<16 (CTRL_REG_3)

#define CMD_PERF_RESET       0x91  // clear recv->transmit sticky maxes + histogram + counts

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
// DEVICE DISCOVERY BEACON: broadcast device_beacon_t to <subnet>.255:BEACON_PORT
// ~1 Hz so a client can discover our IP + know we're up WITHOUT sending us
// anything during the fragile boot window. See the contract in main.h.
// IP_SOF_BROADCAST is off in this build, so a plain udp_sendto() to a broadcast
// address is permitted (no SOF_BROADCAST flag needed).
// ============================================================================
_Static_assert(sizeof(device_beacon_t) == 28, "device_beacon_t must be 28 bytes (net.py/ephys-socket decode)");

extern struct netif server_netif;
static struct udp_pcb *beacon_pcb = NULL;
static ip_addr_t beacon_bcast;      // subnet-directed broadcast address
static uint32_t  beacon_self_ip;    // our IPv4 (network order) for the payload

void beacon_init(void) {
    uint32_t ip   = netif_ip4_addr(&server_netif)->addr;
    uint32_t mask = netif_ip4_netmask(&server_netif)->addr;
    beacon_self_ip = ip;
    beacon_bcast.addr = (ip & mask) | (~mask);   // (subnet bits) | (host all-ones)
    beacon_pcb = udp_new();
    if (beacon_pcb == NULL) {
        send_message("ERROR: Could not create beacon UDP PCB\r\n");
        return;
    }
    send_message("Discovery beacon -> %s:%d (~1 Hz)\r\n",
                 ip4addr_ntoa(&beacon_bcast), BEACON_PORT);
}

void beacon_send(void) {
    if (beacon_pcb == NULL) return;
    static device_beacon_t b;   // static: the PBUF_REF references it until TX drains
    b.magic      = BEACON_MAGIC;
    b.version    = BEACON_VERSION;
    b.ip         = beacon_self_ip;
    b.tcp_port   = TCP_PORT;
    b.udp_port   = UDP_PORT;
    b.fw_version = FIRMWARE_VERSION_WORD;
    memcpy(b.mac, server_netif.hwaddr, 6);
    b.reserved   = 0;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(b), PBUF_REF);
    if (p == NULL) return;
    p->payload = &b;
    udp_sendto(beacon_pcb, p, &beacon_bcast, BEACON_PORT);
    pbuf_free(p);
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

    // recv->transmit spike instrumentation (raw ticks; cleared by CMD_PERF_RESET)
    status->send_ticks_last   = send_ticks_last;
    status->send_ticks_max    = send_ticks_max;
    status->over_budget_count = over_budget_count;
    status->worst_pkt_index   = worst_pkt_index;
    status->worst_cdma_ticks  = worst_cdma_ticks;
    status->worst_send_ticks  = worst_send_ticks;
    status->worst_other_ticks = worst_other_ticks;
    // TX drop diagnostics (v1.6)
    status->bb_pbuf_alloc_fail  = bb_pbuf_alloc_fail;
    status->bb_send_err         = bb_send_err;
    status->bb_last_send_err    = bb_last_send_err;
    status->first_drop_pkt      = first_drop_pkt;
    status->last_drop_pkt       = last_drop_pkt;
    status->memp_num_pbuf       = (uint32_t)MEMP_NUM_PBUF;
    for (int i = 0; i < 8; i++) status->drop_ring[i] = drop_ring[i];
    for (int i = 0; i < PERF_HIST_BUCKETS; i++)
        status->loop_hist[i] = loop_hist[i];

    // Aux config read-back (fast-settle / DSP / digout settings live in CTRL_REG_22)
    status->aux_ctrl = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET);

    // RHD chip register mirror (commanded state of regs 0..21)
    memcpy(status->rhd_reg, rhd_reg_shadow, sizeof(status->rhd_reg));

    // Analytic chirp NCO config (host-set, mirrored from CTRL_REG_3 tracking)
    status->chirp_mode   = chirp_cfg_mode;
    status->chirp_stride = chirp_cfg_stride;
    status->chirp_fspan  = chirp_cfg_fspan;
    status->chirp_rate   = chirp_cfg_rate;
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

        case CMD_SET_CHIRP:
            // param1 = mode(bit0) | stride<<8 ; param2 = fspan | rate<<16
            pl_set_chirp((uint8_t)(cmd->param1 & 0x1),
                         (uint8_t)((cmd->param1 >> 8) & 0x3F),
                         (uint16_t)(cmd->param2 & 0xFFF),
                         (uint16_t)((cmd->param2 >> 16) & 0xFFF));
            send_message("Binary Command: SET_CHIRP\r\n");
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

        case CMD_PERF_RESET:
            perf_reset();   // fresh recv->transmit measurement window
            send_message("Binary Command: PERF_RESET (maxes/histogram/counts cleared)\r\n");
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
    
    // First, handle any incomplete command from previous packet
    if (recv_buffer_pos > 0) {
        uint16_t bytes_needed = CMD_PACKET_SIZE - recv_buffer_pos;
        uint16_t bytes_available = data_len < bytes_needed ? data_len : bytes_needed;
        
        memcpy(&recv_buffer[recv_buffer_pos], data, bytes_available);
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
    while (data_pos + CMD_PACKET_SIZE <= data_len) {
        cmd_packet_t cmd_aligned __attribute__((aligned(8)));
        memcpy(&cmd_aligned, &data[data_pos], CMD_PACKET_SIZE);
        if (cmd_aligned.magic == CMD_MAGIC) {
            process_command(tpcb, &cmd_aligned);
            data_pos += CMD_PACKET_SIZE;
        } else {
            // Skip bad data and look for next magic
            data_pos++;
        }
    }
    
    // Copy any remaining partial command to recv_buffer
    uint16_t remaining_bytes = data_len - data_pos;
    if (remaining_bytes > 0) {
        memcpy(recv_buffer, &data[data_pos], remaining_bytes);
        recv_buffer_pos = remaining_bytes;
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
