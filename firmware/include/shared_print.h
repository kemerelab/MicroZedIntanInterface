#ifndef SHARED_PRINT_H
#define SHARED_PRINT_H

#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <xil_types.h>
#include <xil_mmu.h>

#define MAX_PRINT_ENTRIES 64
#define PRINT_MSG_SIZE 256
// #define SHARED_MEM_BASE 0xFFFF0000UL
#define SHARED_MEM_BASE 0x3F000000UL

#define NORM_NONCACHE_SHARED    0x14de2

typedef struct {
    char message[PRINT_MSG_SIZE];
    volatile uint32_t data_present;
    volatile uint32_t length;
} print_entry_t;

void init_print_buffer(void);
void send_message(const char *format, ...);
void print_handler_loop(void);


typedef struct {
    print_entry_t entries[MAX_PRINT_ENTRIES];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
    volatile uint32_t initialized;
} print_buffer_t;


static inline int is_buffer_full(print_buffer_t *print_buffer) {
    uint32_t next_write = (print_buffer->write_idx + 1) % MAX_PRINT_ENTRIES;
    return (next_write == print_buffer->read_idx);
}

static inline int is_buffer_empty(print_buffer_t *print_buffer) {
    return (print_buffer->write_idx == print_buffer->read_idx);
}

typedef struct {
    volatile int debug_debouncer;
    volatile int lock;
    volatile int enable_streaming_flag;
    volatile int disable_streaming_flag;
    volatile int reset_timestamp_flag;
    volatile int pl_print_flag;
    volatile int bram_benchmark_flag;
    volatile int dump_bram_flag;
    volatile int cable_test_flag;
    volatile u32 start_bram_addr;
    volatile u32 word_count;
} command_flags_t;

extern volatile command_flags_t *command_flags;

#endif