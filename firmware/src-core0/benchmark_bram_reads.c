#include "main.h"
#include "xil_io.h"
#include "xil_cache.h"
#include <string.h>
#include "shared_print.h"

#define BYTES_PER_PACKET (MAX_WORDS_PER_PACKET * BYTES_PER_WORD)

// BRAM read performance benchmark
void benchmark_bram_reads(void) {
    const u32 num_packets = 10;
    const u32 total_words = num_packets * MAX_WORDS_PER_PACKET;
    XTime start_time, end_time;
    
    send_message("\r\n=== BRAM READ BENCHMARK ===\r\n");
    send_message("Reading %u sequential packets (%u words, %u bytes total)\r\n", 
              num_packets, total_words, total_words * 4);
    
    // Test buffer to hold all 10 packets
    static u32 benchmark_buffer[10 * MAX_WORDS_PER_PACKET] __attribute__((aligned(64)));
    
    // Starting position for sequential reads
    u32 start_packet_addr = 0;  // Start at beginning of BRAM
    
    // Invalidate cache for entire test region
    send_message("Invalidating cache for test region...\r\n");
    Xil_DCacheInvalidateRange(BRAM_BASE_ADDR + (start_packet_addr * 4), total_words * 4);
    
    // Method 1: Xil_In32 word-by-word (current method)
    send_message("Method 1: Xil_In32 word-by-word (sequential packets)...\r\n");
    XTime_GetTime(&start_time);
    
    for (u32 packet = 0; packet < num_packets; packet++) {
        u32 packet_start_addr = start_packet_addr + (packet * MAX_WORDS_PER_PACKET);
        u32 *packet_buffer = &benchmark_buffer[packet * MAX_WORDS_PER_PACKET];
        
        for (int i = 0; i < MAX_WORDS_PER_PACKET; i++) {
            u32 word_offset = (packet_start_addr + i) % BRAM_SIZE_WORDS;
            u32 safe_addr = BRAM_BASE_ADDR + (word_offset * 4);
            packet_buffer[i] = Xil_In32(safe_addr);
        }
    }
    
    XTime_GetTime(&end_time);
    u32 xil_in32_time_us = (u32)((end_time - start_time) * 1000000 / XPAR_CPU_CORE_CLOCK_FREQ_HZ);
    send_message("  Time: %u microseconds\r\n", xil_in32_time_us);
    
    // Invalidate cache again before next test
    Xil_DCacheInvalidateRange(BRAM_BASE_ADDR + (start_packet_addr * 4), total_words * 4);
    
    // Method 2: memcpy bulk transfer (potentially faster)
    send_message("Method 2: memcpy bulk transfer (sequential packets)...\r\n");
    XTime_GetTime(&start_time);
    
    for (u32 packet = 0; packet < num_packets; packet++) {
        u32 packet_start_addr = start_packet_addr + (packet * MAX_WORDS_PER_PACKET);
        u32 bram_addr = BRAM_BASE_ADDR + (packet_start_addr * 4);
        u32 *packet_buffer = &benchmark_buffer[packet * MAX_WORDS_PER_PACKET];
        
        // Check if packet crosses BRAM boundary
        if ((packet_start_addr + MAX_WORDS_PER_PACKET) <= BRAM_SIZE_WORDS) {
            // Packet doesn't wrap - single memcpy
            memcpy(packet_buffer, (void*)bram_addr, BYTES_PER_PACKET);
        } else {
            // Packet wraps - two memcpy calls
            u32 words_before_wrap = BRAM_SIZE_WORDS - packet_start_addr;
            u32 words_after_wrap = MAX_WORDS_PER_PACKET - words_before_wrap;
            
            memcpy(packet_buffer, (void*)bram_addr, words_before_wrap * 4);
            memcpy(&packet_buffer[words_before_wrap], (void*)BRAM_BASE_ADDR, words_after_wrap * 4);
        }
    }
    
    XTime_GetTime(&end_time);
    u32 memcpy_time_us = (u32)((end_time - start_time) * 1000000 / XPAR_CPU_CORE_CLOCK_FREQ_HZ);
    send_message("  Time: %u microseconds\r\n", memcpy_time_us);
    
    // Invalidate cache again before next test
    Xil_DCacheInvalidateRange(BRAM_BASE_ADDR + (start_packet_addr * 4), total_words * 4);
    
    // Method 3: Optimized Xil_In32 (sequential addressing when possible)
    send_message("Method 3: Optimized Xil_In32 (sequential packets)...\r\n");
    XTime_GetTime(&start_time);
    
    for (u32 packet = 0; packet < num_packets; packet++) {
        u32 packet_start_addr = start_packet_addr + (packet * MAX_WORDS_PER_PACKET);
        u32 bram_addr = BRAM_BASE_ADDR + (packet_start_addr * 4);
        u32 *packet_buffer = &benchmark_buffer[packet * MAX_WORDS_PER_PACKET];
        
        // Sequential reads when possible (faster than modulo calculations)
        if ((packet_start_addr + MAX_WORDS_PER_PACKET) <= BRAM_SIZE_WORDS) {
            // No wrap - sequential reads (fastest)
            for (int i = 0; i < MAX_WORDS_PER_PACKET; i++) {
                packet_buffer[i] = Xil_In32(bram_addr + (i * 4));
            }
        } else {
            // Handle wrap case with modulo
            for (int i = 0; i < MAX_WORDS_PER_PACKET; i++) {
                u32 word_offset = (packet_start_addr + i) % BRAM_SIZE_WORDS;
                u32 safe_addr = BRAM_BASE_ADDR + (word_offset * 4);
                packet_buffer[i] = Xil_In32(safe_addr);
            }
        }
    }
    
    XTime_GetTime(&end_time);
    u32 optimized_time_us = (u32)((end_time - start_time) * 1000000 / XPAR_CPU_CORE_CLOCK_FREQ_HZ);
    send_message("  Time: %u microseconds\r\n", optimized_time_us);
    
    // Method 4: Single large memcpy (if no wrapping)
    if ((start_packet_addr + total_words) <= BRAM_SIZE_WORDS) {
        send_message("Method 4: Single large memcpy (all packets at once)...\r\n");
        
        // Invalidate cache for single large transfer test
        Xil_DCacheInvalidateRange(BRAM_BASE_ADDR + (start_packet_addr * 4), total_words * 4);
        
        XTime_GetTime(&start_time);
        
        // Single memcpy for all packets
        u32 start_bram_addr = BRAM_BASE_ADDR + (start_packet_addr * 4);
        memcpy(benchmark_buffer, (void*)start_bram_addr, total_words * 4);
        
        XTime_GetTime(&end_time);
        u32 single_memcpy_time_us = (u32)((end_time - start_time) * 1000000 / XPAR_CPU_CORE_CLOCK_FREQ_HZ);
        send_message("  Time: %u microseconds\r\n", single_memcpy_time_us);
        
        // Add to results summary
        send_message("\r\n--- BENCHMARK RESULTS ---\r\n");
        send_message("Xil_In32 (modulo):     %u us\r\n", xil_in32_time_us);
        send_message("memcpy (per packet):   %u us", memcpy_time_us);
        if (memcpy_time_us > 0) {
            u32 speedup = (xil_in32_time_us * 10) / memcpy_time_us;
            send_message(" (%u.%ux %s)\r\n", speedup/10, speedup%10,
                      memcpy_time_us < xil_in32_time_us ? "faster" : "slower");
        } else {
            send_message(" (too fast to measure)\r\n");
        }
        
        send_message("Xil_In32 (sequential): %u us", optimized_time_us);
        if (optimized_time_us > 0) {
            u32 speedup = (xil_in32_time_us * 10) / optimized_time_us;
            send_message(" (%u.%ux %s)\r\n", speedup/10, speedup%10,
                      optimized_time_us < xil_in32_time_us ? "faster" : "slower");
        } else {
            send_message(" (too fast to measure)\r\n");
        }
        
        send_message("Single large memcpy:   %u us", single_memcpy_time_us);
        if (single_memcpy_time_us > 0) {
            u32 speedup = (xil_in32_time_us * 10) / single_memcpy_time_us;
            send_message(" (%u.%ux %s)\r\n", speedup/10, speedup%10,
                      single_memcpy_time_us < xil_in32_time_us ? "faster" : "slower");
        } else {
            send_message(" (too fast to measure)\r\n");
        }
        
        // Throughput calculations
        u32 total_bytes = total_words * 4;
        send_message("\r\nThroughput:\r\n");
        if (xil_in32_time_us > 0) {
            u32 throughput1 = (total_bytes * 1000) / xil_in32_time_us;
            send_message("Xil_In32 (modulo):     %u KB/s\r\n", throughput1);
        }
        if (memcpy_time_us > 0) {
            u32 throughput2 = (total_bytes * 1000) / memcpy_time_us;
            send_message("memcpy (per packet):   %u KB/s\r\n", throughput2);
        }
        if (optimized_time_us > 0) {
            u32 throughput3 = (total_bytes * 1000) / optimized_time_us;
            send_message("Xil_In32 (sequential): %u KB/s\r\n", throughput3);
        }
        if (single_memcpy_time_us > 0) {
            u32 throughput4 = (total_bytes * 1000) / single_memcpy_time_us;
            send_message("Single large memcpy:   %u KB/s\r\n", throughput4);
        }
    } else {
        send_message("Method 4: Skipped (test data would wrap around BRAM)\r\n");
        
        // Results summary without Method 4
        send_message("\r\n--- BENCHMARK RESULTS ---\r\n");
        send_message("Xil_In32 (modulo):     %u us\r\n", xil_in32_time_us);
        send_message("memcpy (per packet):   %u us", memcpy_time_us);
        if (memcpy_time_us > 0) {
            u32 speedup = (xil_in32_time_us * 10) / memcpy_time_us;
            send_message(" (%u.%ux %s)\r\n", speedup/10, speedup%10,
                      memcpy_time_us < xil_in32_time_us ? "faster" : "slower");
        } else {
            send_message(" (too fast to measure)\r\n");
        }
        
        send_message("Xil_In32 (sequential): %u us", optimized_time_us);
        if (optimized_time_us > 0) {
            u32 speedup = (xil_in32_time_us * 10) / optimized_time_us;
            send_message(" (%u.%ux %s)\r\n", speedup/10, speedup%10,
                      optimized_time_us < xil_in32_time_us ? "faster" : "slower");
        } else {
            send_message(" (too fast to measure)\r\n");
        }
        
        // Throughput calculations
        u32 total_bytes = total_words * 4;
        send_message("\r\nThroughput:\r\n");
        if (xil_in32_time_us > 0) {
            u32 throughput1 = (total_bytes * 1000) / xil_in32_time_us;
            send_message("Xil_In32 (modulo):     %u KB/s\r\n", throughput1);
        }
        if (memcpy_time_us > 0) {
            u32 throughput2 = (total_bytes * 1000) / memcpy_time_us;
            send_message("memcpy (per packet):   %u KB/s\r\n", throughput2);
        }
        if (optimized_time_us > 0) {
            u32 throughput3 = (total_bytes * 1000) / optimized_time_us;
            send_message("Xil_In32 (sequential): %u KB/s\r\n", throughput3);
        }
    }
    
    send_message("\r\nNote: This represents reading 10 different packets\r\n");
    send_message("sequentially, storing each in separate buffer space.\r\n");
    send_message("=========================\r\n\r\n");
}
