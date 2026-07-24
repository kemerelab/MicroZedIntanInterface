// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

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
    volatile int dump_bram_flag;
    volatile int cable_test_flag;
    volatile uint32_t start_bram_addr;
    volatile uint32_t word_count;
} command_flags_t;

extern volatile command_flags_t *command_flags;

// ============================================================================
// SHARED STATUS SNAPSHOT (core 0 publishes -> core 1 reads, no print ring)
// ============================================================================
// Core 0 (the data pump) periodically copies a binary status snapshot here with
// cheap stores (no string formatting, no UART, no blocking). Core 1 reads it and
// does all the formatting/printing on its own time, so status reporting never
// stalls streaming. `seq` is a seqlock: core 0 makes it odd before writing the
// fields and even (== old+1) after; core 1 reads it before/after and retries on
// a tear. All scalars are 32-bit (atomic on the A9).
typedef struct {
    volatile uint32_t seq;             // seqlock; even = stable snapshot
    // --- PL hardware status ---
    volatile uint32_t timestamp_lo;
    volatile uint32_t timestamp_hi;
    volatile uint32_t packets_sent;
    volatile uint32_t bram_write_addr;
    volatile uint32_t fifo_count;
    volatile uint32_t state_counter;
    volatile uint32_t cycle_counter;
    volatile uint32_t channel_enable;  // 8-bit (both ports)
    volatile uint32_t phase;           // phase0 | phase1<<4 | phase2<<8 | phase3<<12
    volatile uint32_t flags_pl;        // bit0 tx_active, bit1 loop_limit, bit2 debug_mode
    // --- PS software status ---
    volatile uint32_t packets_received;
    volatile uint32_t error_count;
    volatile uint32_t udp_packets_sent;
    volatile uint32_t udp_send_errors;
    volatile uint32_t ps_read_addr;
    volatile uint32_t packet_size;
    volatile uint32_t stream_enabled;
    // --- logging health ---
    volatile uint32_t events_dropped;  // send_message drops (ring full under load)
} psmon_t;

extern volatile psmon_t *psmon;

// psmon flag bits
#define PSMON_FLAG_TX_ACTIVE   (1u << 0)
#define PSMON_FLAG_LOOP_LIMIT  (1u << 1)
#define PSMON_FLAG_DEBUG_MODE  (1u << 2)

void psmon_init(void);                 // core 0: zero the snapshot once at startup
void print_status_local(void);         // core 1: format + print from the snapshot

#endif