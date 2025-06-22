#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>

// DMA Controller registers
#define DMA_BASE_ADDR 0x40400000
#define DMA_RANGE 0x10000
#define DMA_S2MM_DMACR 0x30
#define DMA_S2MM_DMASR 0x34
#define DMA_S2MM_DSTADDR 0x48
#define DMA_S2MM_LENGTH 0x58

// AXI Lite Control registers (you'll need to update this address)
#define CONTROL_BASE_ADDR 0x60000000  // Update with your actual base address
#define CONTROL_RANGE 0x1000
#define CONTROL_REG_OFFSET 0x0        // Control register at offset 0x0
#define STATUS_REG_OFFSET 0x4         // Status register at offset 0x4

// DMA Buffer
#define DMA_BUFFER_ADDR 0x1e000000
#define DMA_BUFFER_SIZE 0x1000 // 4 KB

// Data format
#define BATCH_WORDS 37*10 
#define BATCH_SIZE (BATCH_WORDS * sizeof(uint64_t))

// Control register bit definitions
#define CTRL_TRANSMIT_ENABLE    (1 << 0)
#define CTRL_RESET_TIMESTAMP    (1 << 1)
#define CTRL_PAUSE_TIMESTAMP    (1 << 2)

// Status register bit definitions
#define STATUS_TRANSMISSION_ACTIVE  (1 << 0)
#define STATUS_LAST_PACKET_SENT     (1 << 1)
#define STATUS_STATE_MASK           (0x7F << 2)
#define STATUS_CYCLE_MASK           (0x3F << 9)
#define STATUS_PACKET_COUNT_MASK    (0xFFFF << 16)

#define STATUS_STATE_SHIFT      2
#define STATUS_CYCLE_SHIFT      9
#define STATUS_PACKET_COUNT_SHIFT 16

void print_usage(const char *prog_name) {
    printf("Usage: %s [options]\n", prog_name);
    printf("Options:\n");
    printf("  -h, --help           Show this help message\n");
    printf("  -e, --enable         Enable transmission and start capture\n");
    printf("  -d, --disable        Disable transmission\n");
    printf("  -r, --reset-time     Reset timestamp to 0 (transmission must be disabled)\n");
    printf("  -s, --status         Read and display status\n");
    printf("  -c, --continuous     Continuous capture mode (default)\n");
    printf("  -p, --pause-time     Pause timestamp increment\n");
    printf("  -u, --unpause-time   Resume timestamp increment\n");
}

void print_status(volatile uint32_t *control_regs) {
    uint32_t status = control_regs[STATUS_REG_OFFSET / 4];
    uint32_t control = control_regs[CONTROL_REG_OFFSET / 4];
    
    printf("\n=== Data Generator Status ===\n");
    printf("Control Register: 0x%08X\n", control);
    printf("  Transmission Enabled: %s\n", (control & CTRL_TRANSMIT_ENABLE) ? "YES" : "NO");
    printf("  Reset Timestamp: %s\n", (control & CTRL_RESET_TIMESTAMP) ? "YES" : "NO");
    printf("  Pause Timestamp: %s\n", (control & CTRL_PAUSE_TIMESTAMP) ? "YES" : "NO");
    
    printf("\nStatus Register: 0x%08X\n", status);
    printf("  Transmission Active: %s\n", (status & STATUS_TRANSMISSION_ACTIVE) ? "YES" : "NO");
    printf("  Last Packet Sent: %s\n", (status & STATUS_LAST_PACKET_SENT) ? "YES" : "NO");
    printf("  Current State: %d\n", (status & STATUS_STATE_MASK) >> STATUS_STATE_SHIFT);
    printf("  Current Cycle: %d\n", (status & STATUS_CYCLE_MASK) >> STATUS_CYCLE_SHIFT);
    printf("  Packets Sent: %d\n", (status & STATUS_PACKET_COUNT_MASK) >> STATUS_PACKET_COUNT_SHIFT);
    printf("=============================\n\n");
}


int flush_pl_fifo(volatile uint32_t *dma_regs, volatile uint64_t *dma_buffer, 
                              volatile uint32_t *control_regs) {
    printf("Alternative FIFO flush: Large drain transfer...\n");
    
    // Disable PL transmission
    control_regs[CONTROL_REG_OFFSET / 4] = 0;
    usleep(10000);
    
    // Enable DMA
    dma_regs[DMA_S2MM_DMACR / 4] = 4; // Reset
    usleep(100);
    dma_regs[DMA_S2MM_DMACR / 4] = 0x1; // Enable
    dma_regs[DMA_S2MM_DMASR / 4] = 0xFFFFFFFF; // Clear status
    
    // Do one large transfer to drain everything at once
    // Use a generous size to catch all stale data
    int drain_size = 4096; // 4KB should be more than enough
    
    dma_regs[DMA_S2MM_DSTADDR / 4] = DMA_BUFFER_ADDR;
    dma_regs[DMA_S2MM_LENGTH / 4] = drain_size;
    
    // Wait with a reasonable timeout
    int timeout_ms = 500; // 500ms should be plenty
    int waited_ms = 0;
    
    while (!(dma_regs[DMA_S2MM_DMASR / 4] & (1 << 12)) && waited_ms < timeout_ms) {
        usleep(1000);
        waited_ms++;
    }
    
    uint32_t status = dma_regs[DMA_S2MM_DMASR / 4];
    
    if (status & (1 << 12)) {
        printf("  Large drain completed successfully\n");
    } else {
        printf("  Large drain timed out (this is expected if FIFO was empty)\n");
    }
    
    // Reset DMA to clean state
    dma_regs[DMA_S2MM_DMACR / 4] = 4;
    usleep(100);
    dma_regs[DMA_S2MM_DMACR / 4] = 0;
    dma_regs[DMA_S2MM_DMASR / 4] = 0xFFFFFFFF;
    
    return 0;
}


int wait_for_dma_idle(volatile uint32_t *dma_regs, int timeout_ms) {
    printf("Waiting for DMA to become idle...\n");
    
    for (int i = 0; i < timeout_ms; i++) {
        uint32_t status = dma_regs[DMA_S2MM_DMASR / 4];
        
        // Check if DMA is idle (bit 1) and not running
        if ((status & (1 << 1)) && !(status & (1 << 0))) {
            printf("DMA is idle\n");
            return 0;
        }
        
        usleep(1000); // Wait 1ms
    }
    
    printf("Warning: DMA did not become idle within timeout\n");
    return -1;
}

void reset_dma_controller(volatile uint32_t *dma_regs) {
    printf("Resetting DMA controller...\n");
    
    // Reset DMA
    dma_regs[DMA_S2MM_DMACR / 4] = 4; // Set reset bit
    usleep(100); // Wait 100µs
    
    // Clear reset and disable
    dma_regs[DMA_S2MM_DMACR / 4] = 0;
    usleep(100);
    
    // Clear all status bits
    dma_regs[DMA_S2MM_DMASR / 4] = 0xFFFFFFFF;
    
    printf("DMA controller reset complete\n");
}


int main(int argc, char *argv[]) {
    int uio_fd, mem_fd;
    volatile uint32_t *dma_regs;
    volatile uint32_t *control_regs;
    volatile uint64_t *dma_buffer;
    
    int enable_transmission = 0;
    int disable_transmission = 0;
    int reset_timestamp = 0;
    int show_status = 0;
    int continuous_mode = 0;
    int pause_timestamp = 0;
    int unpause_timestamp = 0;

    int magic_detected = 0;
    uint64_t current_timestamp, last_timestamp = 0;
    int received_packets = 0;
    int missed_timestamps = 0;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--enable") == 0) {
            enable_transmission = 1;
        } else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--disable") == 0) {
            disable_transmission = 1;
        } else if (strcmp(argv[i], "-r") == 0 || strcmp(argv[i], "--reset-time") == 0) {
            reset_timestamp = 1;
        } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--status") == 0) {
            show_status = 1;
            continuous_mode = 0;
        } else if (strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--continuous") == 0) {
            continuous_mode = 1;
        } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--pause-time") == 0) {
            pause_timestamp = 1;
        } else if (strcmp(argv[i], "-u") == 0 || strcmp(argv[i], "--unpause-time") == 0) {
            unpause_timestamp = 1;
        } else {
            printf("Unknown option: %s\n", argv[i]);
            print_usage(argv[0]);
            return 1;
        }
    }
    
    // Open device files
    uio_fd = open("/dev/uio0", O_RDWR);
    if (uio_fd < 0) {
        perror("Failed to open /dev/uio0");
        return 1;
    }
    
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0) {
        perror("Failed to open /dev/mem");
        close(uio_fd);
        return 1;
    }
    
    // Map memory regions
    dma_regs = (volatile uint32_t *) mmap(NULL, DMA_RANGE, PROT_READ | PROT_WRITE, 
                                          MAP_SHARED, mem_fd, DMA_BASE_ADDR);
    
    control_regs = (volatile uint32_t *) mmap(NULL, CONTROL_RANGE, PROT_READ | PROT_WRITE, 
                                              MAP_SHARED, mem_fd, CONTROL_BASE_ADDR);
    
    dma_buffer = (volatile uint64_t *) mmap(NULL, DMA_BUFFER_SIZE, PROT_READ | PROT_WRITE, 
                                            MAP_SHARED, mem_fd, DMA_BUFFER_ADDR);
    
    if (dma_regs == MAP_FAILED || control_regs == MAP_FAILED || dma_buffer == MAP_FAILED) {
        perror("Failed to mmap");
        close(uio_fd);
        close(mem_fd);
        return 1;
    }
    
    // Execute requested operations
    if (disable_transmission) {
        printf("Disabling transmission...\n");
        control_regs[CONTROL_REG_OFFSET / 4] = 0; // Clear all control bits
        usleep(10000); // Wait 10ms for change to take effect
    }
    
    if (reset_timestamp) {
        printf("Resetting timestamp...\n");
        // First ensure transmission is disabled
        uint32_t current_control = control_regs[CONTROL_REG_OFFSET / 4];
        if (current_control & CTRL_TRANSMIT_ENABLE) {
            printf("Warning: Transmission is enabled. Disabling first...\n");
            control_regs[CONTROL_REG_OFFSET / 4] = current_control & ~CTRL_TRANSMIT_ENABLE;
            usleep(10000);
        }
        // Reset timestamp
        control_regs[CONTROL_REG_OFFSET / 4] = CTRL_RESET_TIMESTAMP;
        usleep(1000);
        // Clear reset bit
        control_regs[CONTROL_REG_OFFSET / 4] = 0;
    }
    
    if (pause_timestamp) {
        printf("Pausing timestamp increment...\n");
        uint32_t current_control = control_regs[CONTROL_REG_OFFSET / 4];
        control_regs[CONTROL_REG_OFFSET / 4] = current_control | CTRL_PAUSE_TIMESTAMP;
    }
    
    if (unpause_timestamp) {
        printf("Resuming timestamp increment...\n");
        uint32_t current_control = control_regs[CONTROL_REG_OFFSET / 4];
        control_regs[CONTROL_REG_OFFSET / 4] = current_control & ~CTRL_PAUSE_TIMESTAMP;
    }
    
    if (enable_transmission) {
        printf("Enabling transmission...\n");
        uint32_t current_control = control_regs[CONTROL_REG_OFFSET / 4];
        control_regs[CONTROL_REG_OFFSET / 4] = (current_control & ~CTRL_RESET_TIMESTAMP) | CTRL_TRANSMIT_ENABLE;
        usleep(10000); // Wait 10ms for change to take effect
    }
    
    if (show_status) {
        print_status(control_regs);
    }
    
    // Continuous capture mode with improved buffer management
    if (continuous_mode) {
        printf("Starting continuous mode with complete cleanup...\n");
        
        // Step 1: Disable transmission and wait for DMA to be idle
        printf("Disabling transmission...\n");
        control_regs[CONTROL_REG_OFFSET / 4] = 0; // Clear all control bits
        usleep(10000); // Wait 10ms for change to take effect
        wait_for_dma_idle(dma_regs, 1000);
        
        // Step 2: Flush PL-side DMA FIFO to remove stale data
        flush_pl_fifo(dma_regs, dma_buffer, control_regs);
        
        // Step 3: Reset DMA controller
        reset_dma_controller(dma_regs);
        
        // Step 4: Clear the PS-side DMA buffer
        // clear_dma_buffer(dma_buffer, DMA_BUFFER_SIZE);
        
        // Step 5: Reset timestamp
        printf("Resetting timestamp...\n");
        control_regs[CONTROL_REG_OFFSET / 4] = CTRL_RESET_TIMESTAMP;

        // Step 6: Enable transmission
        printf("Enabling transmission for continuous mode...\n");
        uint32_t current_control = control_regs[CONTROL_REG_OFFSET / 4];
        control_regs[CONTROL_REG_OFFSET / 4] = (current_control & ~CTRL_RESET_TIMESTAMP) | CTRL_TRANSMIT_ENABLE;
        
        printf("Starting DMA continuous capture (expect 37 64-bit words per batch):\n");
        printf("Press Ctrl+C to stop\n\n");
        
        while (1) {
            // Reset and enable DMA
            dma_regs[DMA_S2MM_DMACR / 4] = 4; // Reset
            usleep(50);
            dma_regs[DMA_S2MM_DMACR / 4] = 0x1; // Enable
            dma_regs[DMA_S2MM_DMASR / 4] = 0xFFFFFFFF; // Clear status
            
            // Set destination and length
            dma_regs[DMA_S2MM_DSTADDR / 4] = DMA_BUFFER_ADDR;
            dma_regs[DMA_S2MM_LENGTH / 4] = BATCH_SIZE;
            
            // Wait for DMA completion (IOC bit 12)
            while (!(dma_regs[DMA_S2MM_DMASR / 4] & (1 << 12))) {
                usleep(50);
            }
            
            for (int i = 0; i < BATCH_WORDS; i++) {
                if (dma_buffer[i] == 0xDEADBEEFCAFEBABE) {
                    magic_detected = 1;
                }
                else {
                    if (magic_detected) {
                        current_timestamp = dma_buffer[i];
                        magic_detected = 0;
                        received_packets += 1;
                        if ((current_timestamp - last_timestamp) > 1) {
                            missed_timestamps += 1;
                        }
                        last_timestamp = current_timestamp;
                        if ((received_packets % 30000) == 1) {
                            printf("Received %d packets. Missed %d. Last timestamp %u\n",
                                   received_packets, missed_timestamps, (uint32_t)(current_timestamp & 0xFFFFFFFF));
                        }
                    }
                }
            }
        }
    }
    
    // Cleanup
    munmap((void *)dma_regs, DMA_RANGE);
    munmap((void *)control_regs, CONTROL_RANGE);
    munmap((void *)dma_buffer, DMA_BUFFER_SIZE);
    close(uio_fd);
    close(mem_fd);
    
    return 0;
}
