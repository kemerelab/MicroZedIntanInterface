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
#define CMD_LOAD_CONVERT    0x20
#define CMD_LOAD_INIT       0x21
#define CMD_LOAD_CABLE_TEST 0x22
#define CMD_FULL_CABLE_TEST 0x30
#define CMD_GET_STATUS      0x40
#define CMD_DUMP_BRAM       0x41
#define CMD_SET_UDP_DEST    0x50


#define ACK_SUCCESS         0x06
#define ACK_ERROR           0x15

typedef struct {
    uint32_t magic;
    uint32_t cmd_id;
    uint32_t ack_id;
    uint32_t param1;
    uint32_t param2;
} cmd_packet_t;

// Static receive buffer for handling partial commands
static uint8_t recv_buffer[CMD_PACKET_SIZE];
static uint16_t recv_buffer_pos = 0;

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

        case CMD_SET_CHANNEL_ENABLE:
            pl_set_channel_enable(cmd->param1 & 0xF);
            send_message("Binary Command: SET_CHANNEL_ENABLE 0x%X\r\n", cmd->param1 & 0xF);
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
            
        case CMD_GET_STATUS: {
            pl_print_status();
            status_response_t status_data;
            collect_status_data(&status_data);
            send_response(tpcb, cmd->ack_id, ACK_SUCCESS,
                         &status_data, sizeof(status_data));
            send_message("Binary Command: GET_STATUS (sent %d bytes)\r\n",
                        sizeof(status_data));
            return;  // Early return - don't call send_ack
        }
            
        case CMD_DUMP_BRAM:
            command_flags->dump_bram_flag = 1;
            command_flags->start_bram_addr = cmd->param1;
            command_flags->word_count = cmd->param2;
            send_message("Binary Command: DUMP_BRAM %u %u\r\n",
                        cmd->param1, cmd->param2);
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

err_t tcp_recv_cb(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    (void)arg;
    (void)err;
    
    if (!p) {
        tcp_close(tpcb);
        recv_buffer_pos = 0;
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
    
    // Process complete commands directly from TCP buffer
    while (data_pos + CMD_PACKET_SIZE <= data_len) {
        cmd_packet_t *cmd = (cmd_packet_t *)&data[data_pos];
        if (cmd->magic == CMD_MAGIC) {
            process_command(tpcb, cmd);
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
    
    tcp_recv(newpcb, tcp_recv_cb);
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
    send_message("Binary TCP command server started on port %d\r\n", TCP_PORT);
    send_message("Commands use 20-byte binary format with magic 0xDEADBEEF\r\n");
}
