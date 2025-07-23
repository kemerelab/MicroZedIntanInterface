#include "main.h"
#include "sleep.h"
#include <stdio.h>
#include "xil_io.h"

// ============================================================================
// PL CONTROL FUNCTIONS
// ============================================================================

void pl_set_transmission(int enable) {
    u32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    if (enable) {
        ctrl_reg |= CTRL_ENABLE_TRANSMISSION;
        xil_printf("PL transmission ENABLED\r\n");
    } else {
        ctrl_reg &= ~CTRL_ENABLE_TRANSMISSION;
        xil_printf("PL transmission DISABLED\r\n");
    }
    
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
}

void pl_reset_timestamp(void) {
    u32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    ctrl_reg |= CTRL_RESET_TIMESTAMP;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
    
    usleep(1000);  // Hold reset for 1ms
    
    ctrl_reg &= ~CTRL_RESET_TIMESTAMP;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
    
    xil_printf("PL timestamp RESET\r\n");
}

void pl_set_loop_count(u32_t loop_count) {
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_1_OFFSET, loop_count);
    xil_printf("PL loop count set to %u\r\n", loop_count);
}

// ============================================================================
// PL STATUS READING FUNCTIONS
// ============================================================================

u64_t pl_get_timestamp(void) {
    u32_t status4 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_4_OFFSET);
    u32_t status5 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_5_OFFSET);
    return ((u64_t)status5 << 32) | status4;
}

int pl_is_transmission_active(void) {
    u32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    return (status0 & STATUS_TRANSMISSION_ACTIVE) ? 1 : 0;
}

u32_t pl_get_packets_sent(void) {
    return Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_3_OFFSET);
}

int pl_is_loop_limit_reached(void) {
    u32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    return (status0 & STATUS_LOOP_LIMIT_REACHED) ? 1 : 0;
}

u32 pl_get_bram_write_address(void) {
    u32_t status6 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET);
    return status6 & 0x3FFF;  // Extract 14-bit BRAM address (0 to 16383)
}

// Get FIFO count from status register 6
static u32 pl_get_fifo_count(void) {
    u32_t status6 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET);
    return (status6 >> 14) & 0x1FF;  // Extract 9-bit FIFO count
}

// ============================================================================
// STATUS DISPLAY FUNCTIONS
// ============================================================================

void pl_print_status(void) {
    u32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    u32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);
    u32_t status2 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_2_OFFSET);
    u32_t status3 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_3_OFFSET);
    
    xil_printf("=== PL STATUS ===\r\n");
    xil_printf("Transmission: %s\r\n", (status0 & STATUS_TRANSMISSION_ACTIVE) ? "ACTIVE" : "STOPPED");
    xil_printf("Loop limit reached: %s\r\n", (status0 & STATUS_LOOP_LIMIT_REACHED) ? "YES" : "NO");
    xil_printf("State counter: %u\r\n", status1 & 0x7F);
    xil_printf("Cycle counter: %u\r\n", status2 & 0x3F);
    xil_printf("Packets sent: %u\r\n", status3);
    xil_printf("Timestamp: %llu\r\n", pl_get_timestamp());
    xil_printf("BRAM write address: %u\r\n", pl_get_bram_write_address());
    xil_printf("FIFO count: %u\r\n", pl_get_fifo_count());
    xil_printf("==================\r\n");
}

