// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#include "shared_print.h"
#include "sleep.h"      // For usleep
#include "xil_printf.h" // A common printf-like function for Xilinx embedded systems
#include "xuartps.h"
#include "xiltimer.h"   // XTime / XTime_GetTime (core-1 monitor cadence)
#include "xparameters.h"// XPAR_CPU_CORE_CLOCK_FREQ_HZ

// Milliseconds from the free-running timer (mirrors network.c sys_now()).
static uint32_t core1_now_ms(void) {
    XTime now;
    XTime_GetTime(&now);
    return (uint32_t)(now / (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 1000U));
}

#define SERIAL_CMD_BUFFER_SIZE 64
static char serial_cmd_buffer[SERIAL_CMD_BUFFER_SIZE];
static int serial_cmd_index = 0;

// Global pointer to the shared print buffer in the shared memory region

volatile command_flags_t *command_flags = (volatile command_flags_t *)SHARED_MEM_BASE;
#define ALIGN4(x) (((x) + 3) & ~0x3)  // align to next multiple of 4
// #define PRINT_BUFFER_ADDRESS sizeof(SHARED_MEM_BASE + ALIGN4(sizeof(command_flags_t)))
#define PRINT_BUFFER_ADDRESS (SHARED_MEM_BASE + ALIGN4(sizeof(command_flags_t)))

volatile print_buffer_t *print_buffer = (volatile print_buffer_t*)PRINT_BUFFER_ADDRESS;

// Status snapshot lives in the same non-cacheable shared region, after the
// print ring. Both cores map SHARED_MEM_BASE non-cacheable (1 MB section), and
// the ring (~17 KB) + this struct fit well inside it.
#define PSMON_ADDRESS (PRINT_BUFFER_ADDRESS + ALIGN4(sizeof(print_buffer_t)))
volatile psmon_t *psmon = (volatile psmon_t*)PSMON_ADDRESS;

void psmon_init(void) {
    // Core 0 zeroes the snapshot once before publishing.
    volatile uint32_t *p = (volatile uint32_t*)psmon;
    for (unsigned i = 0; i < sizeof(psmon_t)/4; i++) p[i] = 0;
}

volatile int monitor_enabled = 0;   // core-1 "mon" toggle (~1 Hz auto-status)

// Forward declaration: check_serial_input() (below) calls process_serial_command()
void process_serial_command(const char* cmd);

void init_command_flags(void) {
    command_flags->lock = 0;
    command_flags->enable_streaming_flag = 0;
    command_flags->disable_streaming_flag = 0;
    command_flags->reset_timestamp_flag = 0;
    command_flags->pl_print_flag = 0;
    command_flags->dump_bram_flag = 0;
    command_flags->start_bram_addr = 0;
    command_flags->word_count = 16;
}

void check_serial_input(void) {
    if((command_flags->debug_debouncer == 1) && (command_flags->lock == 0)) {
        command_flags->debug_debouncer = 0; // Reset debouncer
        // send_message("debug> ");
        xil_printf("debug> ");
    }
    // Check if UART has data available
    if (XUartPs_IsReceiveData(STDIN_BASEADDRESS)) {
        char ch = XUartPs_RecvByte(STDIN_BASEADDRESS);
        
        // Handle different line endings and backspace
        if (ch == '\r' || ch == '\n') {
            if (serial_cmd_index > 0) {
                command_flags->lock = 1;
                command_flags->debug_debouncer = 1;
                // Null terminate the command
                serial_cmd_buffer[serial_cmd_index] = '\0';
                
                xil_printf("\r\n");  // Echo newline
                
                // Process the command
                process_serial_command(serial_cmd_buffer);
                
                
                // Reset buffer
                serial_cmd_index = 0;
                
                // // Print prompt
                // xil_printf("debug> ");
            }
        } else if (ch == '\b' || ch == 127) {  // Backspace or DEL
            if (serial_cmd_index > 0) {
                serial_cmd_index--;
                xil_printf("\b \b");  // Erase character on terminal
            }
        } else if (ch >= 32 && ch <= 126) {  // Printable characters
            if (serial_cmd_index < SERIAL_CMD_BUFFER_SIZE - 1) {
                serial_cmd_buffer[serial_cmd_index++] = ch;
                //XUartPs_SendByte(STDIN_BASEADDRESS, ch);  // Echo character
            }
        }
        // Ignore other characters (like additional \n after \r)
    }
}
void process_serial_command(const char* cmd) {
    // Trim whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;
    
    if (strncmp(cmd, "start", 5) == 0) {
        xil_printf("Serial command: Starting transmission\r\n");
        command_flags->enable_streaming_flag = 1;
        
    } else if (strncmp(cmd, "stop", 4) == 0) {
        xil_printf("Serial command: Stopping transmission\r\n");
        command_flags->disable_streaming_flag = 1;
        
    } else if (strncmp(cmd, "reset", 5) == 0) {
        xil_printf("Serial command: Resetting timestamp\r\n");
        command_flags->reset_timestamp_flag = 1;
        
    } else if (strncmp(cmd, "status", 6) == 0) {
        // Handled entirely on core 1 from the shared snapshot -- never touches
        // the core-0 data pump. Reset lock since we complete synchronously here.
        print_status_local();
        command_flags->lock = 0;

    } else if (strncmp(cmd, "mon", 3) == 0) {
        // Toggle ~1 Hz auto-status (printed by print_handler_loop from psmon)
        monitor_enabled = !monitor_enabled;
        xil_printf("Auto-status monitor %s\r\n", monitor_enabled ? "ON" : "OFF");
        command_flags->lock = 0;

    } else if (strncmp(cmd, "dump", 4) == 0) {
        // Parse dump command: "dump [start] [count]"
        
        sscanf(cmd, "dump %lu %lu", &(command_flags->start_bram_addr), &(command_flags->word_count));
        
        xil_printf("Serial command: Dumping BRAM from %u, count %u\r\n", command_flags->start_bram_addr, command_flags->word_count);
        command_flags->dump_bram_flag = 1;
        //dump_bram_data(start_addr, word_count);
        
    } else if (strncmp(cmd, "help", 4) == 0 || strlen(cmd) == 0) {
        xil_printf("\r\nSerial Debug Commands:\r\n");
        xil_printf("  start    - Start data transmission\r\n");
        xil_printf("  stop     - Stop data transmission\r\n");
        xil_printf("  reset    - Reset timestamp and counters\r\n");
        xil_printf("  status   - Show system status (from shared snapshot)\r\n");
        xil_printf("  mon      - Toggle ~1 Hz auto-status\r\n");
        xil_printf("  dump [start] [count] - Dump BRAM contents\r\n");
        xil_printf("  help     - Show this help\r\n");
        command_flags->lock = 0;
        
    } else {
        xil_printf("Unknown command: '%s'. Type 'help' for commands.\r\n", cmd);
        command_flags->lock = 0;
    }
}


/**
 * @brief Initializes the shared print buffer.
 * This function should be called once by the designated core (typically Core 1).
 * The DDR is in an arbitrary state, so we can't assume anything about this structure
 * until this function is called.
 */
void init_print_buffer(void) {
    // if (!print_buffer->initialized) {
        print_buffer->write_idx = 0;
        print_buffer->read_idx = 0;
        for (int i = 0; i < MAX_PRINT_ENTRIES; i++) {
            print_buffer->entries[i].data_present = 0; // Clear data_present flags
        }
        print_buffer->initialized = 1;
        xil_printf("Shared print buffer initialized.\r\n");
    // }
}

/**
 * @brief Sends a formatted message to the shared print buffer.
 * This function is intended to be called by the main application core (Core 0).
 *
 * @param format The format string (e.g., "Hello, %s!").
 * @param ... Variable arguments matching the format string.
 */
void send_message(const char *format, ...) {
    char buffer[PRINT_MSG_SIZE];
    va_list args;

    // Format the message into a local buffer
    va_start(args, format);
    int len = vsnprintf(buffer, PRINT_MSG_SIZE - 1, format, args);
    va_end(args);

    if (len <= 0) return; // Handle empty or error cases
    buffer[PRINT_MSG_SIZE - 1] = '\0'; // Ensure null termination


    // NON-BLOCKING: if the target ring slot is still unread (ring full), DROP
    // the message and count it instead of spinning. The old code waited up to
    // 100*100us = 10 ms here, which stalled the core-0 data pump and corrupted
    // packets under load. Routine status no longer goes through this ring (core
    // 1 reads the psmon snapshot directly), so the ring carries only rare
    // ad-hoc event strings -- safe to drop the string under pressure; the
    // underlying counts/flags survive in psmon, and events_dropped is visible
    // in the status snapshot.
    uint32_t write_idx = print_buffer->write_idx;
    if (print_buffer->entries[write_idx].data_present == 1) {
        psmon->events_dropped++;
        return;
    }

    strcpy(print_buffer->entries[write_idx].message, buffer);
    print_buffer->entries[write_idx].length = len;
    dsb();  // Data Synchronization Barrier - make sure we write the message before we mark the buffer as ready
    print_buffer->entries[write_idx].data_present = 1;
    print_buffer->write_idx = (write_idx + 1) % MAX_PRINT_ENTRIES;
}

// ============================================================================
// CORE-1 STATUS PRINTER (reads the shared snapshot, not the PL or core 0)
// ============================================================================

// Read the seqlock-protected snapshot into a local copy (retry on a tear).
static void psmon_read(psmon_t *out) {
    for (int tries = 0; tries < 8; tries++) {
        uint32_t s1 = psmon->seq;
        if (s1 & 1u) continue;                 // writer mid-update
        // copy fields
        *out = *psmon;
        dsb();
        uint32_t s2 = psmon->seq;
        if (s1 == s2) return;                  // stable snapshot
    }
    // give up after retries: return whatever we last copied (status display only)
}

void print_status_local(void) {
    psmon_t s;
    psmon_read(&s);
    uint64_t ts = ((uint64_t)s.timestamp_hi << 32) | s.timestamp_lo;

    xil_printf("\r\n=== STATUS (core1 / shared snapshot) ===\r\n");
    xil_printf("Stream: %s   PL tx: %s   debug: %s\r\n",
               s.stream_enabled ? "ON" : "off",
               (s.flags_pl & PSMON_FLAG_TX_ACTIVE) ? "active" : "stopped",
               (s.flags_pl & PSMON_FLAG_DEBUG_MODE) ? "ON" : "off");
    xil_printf("channel_enable: 0x%02X (A=0x%X B=0x%X)   packet: %u words\r\n",
               s.channel_enable & 0xFF, s.channel_enable & 0xF,
               (s.channel_enable >> 4) & 0xF, s.packet_size);
    xil_printf("phase A:%u/%u  B:%u/%u\r\n",
               s.phase & 0xF, (s.phase >> 4) & 0xF,
               (s.phase >> 8) & 0xF, (s.phase >> 12) & 0xF);
    xil_printf("PL: timestamp=%llu  packets_sent=%u  bram_wr=%u  fifo=%u  st/cy=%u/%u\r\n",
               ts, s.packets_sent, s.bram_write_addr, s.fifo_count,
               s.state_counter, s.cycle_counter);
    xil_printf("PS: rx=%u  errors=%u  udp_sent=%u  udp_err=%u  ps_read=%u\r\n",
               s.packets_received, s.error_count, s.udp_packets_sent,
               s.udp_send_errors, s.ps_read_addr);
    xil_printf("log events dropped: %u\r\n", s.events_dropped);
    xil_printf("========================================\r\n");
}

/**
 * @brief Main loop for the print handler (core 1).
 *
 * Drains the print ring to the UART, services the serial debug console, and --
 * when enabled via "mon" -- prints a status snapshot ~1 Hz. All of this is on
 * core 1; none of it touches the core-0 data pump.
 */
void print_handler_loop(void) {
    xil_printf("Starting print_handler_loop.\r\n");
    uint32_t last_mon_ms = core1_now_ms();
    while (1) {
        check_serial_input();

        uint32_t read_idx = print_buffer->read_idx;
        if (print_buffer->entries[read_idx].data_present) {
            xil_printf("> %s", print_buffer->entries[read_idx].message);
            dsb();  // Data Synchronization Barrier - make sure we get the message before we mark the buffer as empty
            print_buffer->entries[read_idx].data_present = 0;
            print_buffer->read_idx = (read_idx + 1) % MAX_PRINT_ENTRIES;
        }

        if (monitor_enabled) {
            uint32_t now_ms = core1_now_ms();
            if ((now_ms - last_mon_ms) >= 1000) {   // ~1 Hz
                last_mon_ms = now_ms;
                print_status_local();
            }
        }
    }
}