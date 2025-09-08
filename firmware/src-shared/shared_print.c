#include "shared_print.h"
#include "sleep.h"      // For usleep
#include "xil_printf.h" // A common printf-like function for Xilinx embedded systems
#include "xuartps.h"

#define SERIAL_CMD_BUFFER_SIZE 64
static char serial_cmd_buffer[SERIAL_CMD_BUFFER_SIZE];
static int serial_cmd_index = 0;

// Global pointer to the shared print buffer in the shared memory region

volatile command_flags_t *command_flags = (volatile command_flags_t *)SHARED_MEM_BASE;
#define ALIGN4(x) (((x) + 3) & ~0x3)  // align to next multiple of 4
// #define PRINT_BUFFER_ADDRESS sizeof(SHARED_MEM_BASE + ALIGN4(sizeof(command_flags_t)))
#define PRINT_BUFFER_ADDRESS (SHARED_MEM_BASE + ALIGN4(sizeof(command_flags_t)))

volatile print_buffer_t *print_buffer = (volatile print_buffer_t*)PRINT_BUFFER_ADDRESS;
void init_command_flags(void) {
    command_flags->lock = 0;
    command_flags->enable_streaming_flag = 0;
    command_flags->disable_streaming_flag = 0;
    command_flags->reset_timestamp_flag = 0;
    command_flags->pl_print_flag = 0;
    command_flags->bram_benchmark_flag = 0;
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
        xil_printf("Serial command: Status\r\n");
        command_flags->pl_print_flag = 1;
        
    } else if (strncmp(cmd, "benchmark", 9) == 0) {
        xil_printf("Serial command: Running BRAM benchmark\r\n");
        command_flags->bram_benchmark_flag = 1;
        
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
        xil_printf("  status   - Show system status\r\n");
        xil_printf("  benchmark - Run BRAM read performance test\r\n");
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


    // TODO: This feels very problematic if we're in the middle of receiving data, we could have a 10 ms timeout?!
    // Wait if the buffer is full, with a timeout to prevent deadlock
    int timeout = 100;
    uint32_t write_idx = print_buffer->write_idx;
    while ((print_buffer->entries[write_idx].data_present == 1) && timeout -- > 0) { // Someone already wrote to this buffer!
        usleep(100); 
    }

    if (timeout <= 0) {
        return;
    }

    strcpy(print_buffer->entries[write_idx].message, buffer);
    print_buffer->entries[write_idx].length = len;
    dsb();  // Data Synchronization Barrier - make sure we write the message before we mark the buffer as ready
    print_buffer->entries[write_idx].data_present = 1;
    print_buffer->write_idx = (write_idx + 1) % MAX_PRINT_ENTRIES;
}

/**
 * @brief Main loop for the print handler.
 */
void print_handler_loop(void) {
    xil_printf("Starting print_handler_loop.\r\n");
    while (1) {
        //check_serial_input();
        uint32_t read_idx = print_buffer->read_idx;
        if (print_buffer->entries[read_idx].data_present) {
            xil_printf("> %s", print_buffer->entries[read_idx].message);
            dsb();  // Data Synchronization Barrier - make sure we get the message before we mark the buffer as empty
            print_buffer->entries[read_idx].data_present = 0;
            print_buffer->read_idx = (read_idx + 1) % MAX_PRINT_ENTRIES;
            // xil_printf("After update %d %d\r\n", print_buffer->write_idx, print_buffer->read_idx);
        }
    }
}