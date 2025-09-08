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
#define CMD_LOAD_TEST_PATTERN 0x23
#define CMD_FULL_CABLE_TEST 0x30
#define CMD_GET_STATUS      0x40
#define CMD_DUMP_BRAM       0x41

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

static void send_ack(struct tcp_pcb *tpcb, uint32_t ack_id, uint8_t status) {
    uint8_t response[3];
    response[0] = (ack_id >> 8) & 0xFF;  // High byte
    response[1] = ack_id & 0xFF;         // Low byte  
    response[2] = status;
    tcp_write(tpcb, response, 3, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
}

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
            
        case CMD_GET_STATUS:
            pl_print_status();
            send_message("Binary Command: GET_STATUS\r\n");
            break;
            
        case CMD_DUMP_BRAM:
            command_flags->dump_bram_flag = 1;
            command_flags->start_bram_addr = cmd->param1;
            command_flags->word_count = cmd->param2;
            send_message("Binary Command: DUMP_BRAM %u %u\r\n", cmd->param1, cmd->param2);
            break;
            
        default:
            status = ACK_ERROR;
            send_message("Binary Command: UNKNOWN (0x%08X)\r\n", cmd->cmd_id);
            break;
    }
    
    send_ack(tpcb, cmd->ack_id, status);
}

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
