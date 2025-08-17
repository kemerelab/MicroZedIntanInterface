#include "main.h"
#include "sleep.h"
#include <stdio.h>
#include "xil_io.h"
#include "shared_print.h"

// ============================================================================
// PL CONTROL FUNCTIONS
// ============================================================================

void pl_set_transmission(int enable) {
    u32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    if (enable) {
        ctrl_reg |= CTRL_ENABLE_TRANSMISSION;
        send_message("PL transmission ENABLED\r\n");
    } else {
        ctrl_reg &= ~CTRL_ENABLE_TRANSMISSION;
        send_message("PL transmission DISABLED\r\n");
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
    
    send_message("PL timestamp RESET\r\n");
}

void pl_set_loop_count(u32_t loop_count) {
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_1_OFFSET, loop_count);
    send_message("PL loop count set to %u\r\n", loop_count);
}


void pl_set_phase_select(int phase0, int phase1) {
    u32_t ctrl_reg_2 = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET);
    
    ctrl_reg_2 &= ~(CTRL_PHASE0_MASK | CTRL_PHASE1_MASK); // Clear existing phase bits
    
    ctrl_reg_2 |= ((phase0 & 0xF) << 0); // Set phase0 bits
    ctrl_reg_2 |= ((phase1 & 0xF) << 4);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET, ctrl_reg_2);
    send_message("PL phase select set to phase0=%d, phase1=%d\r\n", phase0, phase1);
}

void pl_set_debug_mode(int enable) {
    u32_t ctrl_reg_2 = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET);
    
    if (enable) {
        ctrl_reg_2 |= CTRL_DEBUG_MODE;
        send_message("PL debug mode ENABLED\r\n");
    } else {
        ctrl_reg_2 &= ~CTRL_DEBUG_MODE;
        send_message("PL debug mode DISABLED\r\n");
    }
    
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET, ctrl_reg_2);
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
    u32_t status10 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET);  // Updated offset
    return status10 & 0x3FFF;  // Extract 14-bit BRAM address (0 to 16383)
}

static u32 pl_get_fifo_count(void) {
    u32_t status10 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET);  // Updated offset
    return (status10 >> 14) & 0x1FF;  // Extract 9-bit FIFO count
}

// ============================================================================
// CONTROL REGISTER READBACK FUNCTIONS (from mirrored status registers)
// ============================================================================

u32_t pl_get_current_loop_count(void) {
    return Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_7_OFFSET);
}

int pl_get_current_phase_select(int *phase0, int *phase1) {
    u32_t status8 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_8_OFFSET);
    *phase0 = (status8 & CTRL_PHASE0_MASK) >> 0;  // Bits [3:0]
    *phase1 = (status8 & CTRL_PHASE1_MASK) >> 4;  // Bits [7:4]
    return (status8 & CTRL_DEBUG_MODE) ? 1 : 0;   // Return debug mode status
}

int pl_get_current_debug_mode(void) {
    u32_t status8 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_8_OFFSET);
    return (status8 & CTRL_DEBUG_MODE) ? 1 : 0;
}

u32_t pl_get_current_control_flags(void) {
    return Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET);
}


// ============================================================================
// STATUS DISPLAY FUNCTIONS
// ============================================================================


void pl_print_status(void) {
    u32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    u32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);
    u32_t status2 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_2_OFFSET);
    u32_t status3 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_3_OFFSET);
    
    send_message("=== PL STATUS ===\r\n");
    send_message("Transmission: %s\r\n", (status0 & STATUS_TRANSMISSION_ACTIVE) ? "ACTIVE" : "STOPPED");
    send_message("Loop limit reached: %s\r\n", (status0 & STATUS_LOOP_LIMIT_REACHED) ? "YES" : "NO");
    send_message("State counter: %u\r\n", status1 & 0x7F);
    send_message("Cycle counter: %u\r\n", status2 & 0x3F);
    send_message("Packets sent: %u\r\n", status3);
    send_message("Timestamp: %llu\r\n", pl_get_timestamp());
    send_message("BRAM write address: %u\r\n", pl_get_bram_write_address());
    send_message("FIFO count: %u\r\n", pl_get_fifo_count());

    
    u32_t status6 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET);
    u32_t status7 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_7_OFFSET);
    u32_t status8 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_8_OFFSET);
    u32_t status9 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_9_OFFSET);
    send_message("Mirroed Control flags 0: \r\n0x%08X\r\n0x%08X\r\n0x%08X\r\n0x%08X\r\n", status6, status7, status8, status9);


    // NEW: Show current control settings from mirrored status registers
    send_message("=== CURRENT CONTROL SETTINGS ===\r\n");
    send_message("Loop count: %u\r\n", pl_get_current_loop_count());
    
    int phase0, phase1;
    int debug_mode = pl_get_current_phase_select(&phase0, &phase1);
    send_message("Phase select: CIPO0=%d, CIPO1=%d\r\n", phase0, phase1);
    send_message("Debug mode: %s\r\n", debug_mode ? "ENABLED (dummy data)" : "DISABLED (real CIPO)");
    
    u32_t ctrl_flags = pl_get_current_control_flags();
    send_message("Control flags: 0x%08X\r\n", ctrl_flags);
    send_message("  Enable transmission: %s\r\n", (ctrl_flags & CTRL_ENABLE_TRANSMISSION) ? "SET" : "CLEAR");
    send_message("  Reset timestamp: %s\r\n", (ctrl_flags & CTRL_RESET_TIMESTAMP) ? "SET" : "CLEAR");
    
    send_message("================================\r\n");
}



// Simple BRAM dump for debugging
void pl_dump_bram_data(u32 start_addr, u32 word_count) {
    send_message("BRAM dump starting at address %u:\r\n", start_addr);
    for (u32 i = 0; i < word_count; i++) {
        u32 addr = (start_addr + i) % BRAM_SIZE_WORDS;
        u32 data = Xil_In32(BRAM_BASE_ADDR + addr * 4);
        send_message("%u: 0x%08X - 0x%08X\r\n", i, BRAM_BASE_ADDR + addr * 4, data);
    }
}

// ============================================================================
// ============================================================================
// INTAN COMMAND CONFIGURATION SENT FROM FPGA TO CHIPs (COPI / MOSI)
// ============================================================================
// ============================================================================

// Our interface uses 35-element packets for both sending and receiving data.
// Each packet corresponds to a 35-command COPI sequence.

// Set all 35 COPI command words from an array of 16-bit values
void pl_set_copi_commands(const u16 copi_array[35]) {
    // MOSI commands are stored in control registers 4-21 (18 registers total)
    // Each 32-bit register holds two 16-bit MOSI words:
    // - Low 16 bits: even-indexed MOSI word (0, 2, 4, ...)
    // - High 16 bits: odd-indexed MOSI word (1, 3, 5, ...)
    
    for (int i = 0; i < 18; i++) {
        u32 reg_value = 0;
        
        // Pack two 16-bit MOSI words into one 32-bit register
        reg_value = (u32)copi_array[2*i];                    // Low 16 bits: even index
        if ((2*i + 1) < 35) {                               // Check bounds for odd index
            reg_value |= ((u32)copi_array[2*i + 1]) << 16;  // High 16 bits: odd index
        }
        
        // Write to control register (MOSI commands start at CTRL_REG_MOSI_START_OFFSET)
        u32 reg_offset = CTRL_REG_MOSI_START_OFFSET + (i * 4);
        Xil_Out32(PL_CTRL_BASE_ADDR + reg_offset, reg_value);
    }
    
    send_message("MOSI commands updated\r\n");
}


// ============================================================================
// SAFE COPI COMMAND UPDATING
// ============================================================================

// Safely update COPI commands only when transmission is disabled
int pl_set_copi_commands_safe(const u16 copi_array[35], const char* sequence_name) {
    // Check if transmission is currently active
    if (pl_is_transmission_active()) {
        send_message("ERROR: Cannot update COPI commands while transmission is active\r\n");
        send_message("       Stop transmission first with 'stop' command\r\n");
        return 0;  // Failure
    }
    
    // Safe to update - transmission is stopped
    pl_set_copi_commands(copi_array);
    send_message("COPI commands set to: %s\r\n", sequence_name);
    return 1;  // Success
}

// ============================================================================
// COPI SEQUENCE SELECTION FUNCTIONS
// ============================================================================

void pl_set_convert_sequence(void) {
    if (pl_set_copi_commands_safe(convert_cmd_sequence, "CONVERT sequence (channels 0-31)")) {
        send_message("Ready for normal data acquisition from channels 0-31\r\n");
    }
}

void pl_set_initialization_sequence(void) {
    if (pl_set_copi_commands_safe(initialization_cmd_sequence, "INITIALIZATION sequence")) {
        send_message("Ready for chip initialization - run this before first data acquisition\r\n");
    }
}

void pl_set_cable_length_sequence(void) {
    if (pl_set_copi_commands_safe(cable_length_cmd_sequence, "CABLE LENGTH test sequence")) {
        send_message("Ready for cable length calibration - look for 'INTAN' patterns in data\r\n");
    }
}

void pl_set_test_pattern_sequence(void) {
    if (pl_set_copi_commands_safe(mosi_test_pattern, "TEST PATTERN sequence")) {
        send_message("Ready for COPI test pattern - incrementing values 0x0000-0x0022\r\n");
    }
}


// ============================================================================
// PREDEFINED MOSI COMMAND ARRAYS
// ============================================================================
// Notes:
// Register WRITE is 10AA_AAAA VVVV_VVVV
// Register READ is  11AA_AAAA 0000_0000
// Convert is 00CC_CCCC 0000_000X, where X=1 is part of the fast-settle routine


// Channel conversion command sequence
const u16 convert_cmd_sequence[35] = {
    0x0000, 0x0100, 0x0200, 0x0300, 0x0400, 0x0500, 0x0600, 0x0700,  // Channels 0-7
    0x0800, 0x0900, 0x0A00, 0x0B00, 0x0C00, 0x0D00, 0x0E00, 0x0F00,  // Channels 8-15
    0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1500, 0x1600, 0x1700,  // Channels 16-23
    0x1800, 0x1900, 0x1A00, 0x1B00, 0x1C00, 0x1D00, 0x1E00, 0x1F00,  // Channels 24-31
    0x0000, 0x0000, 0x0000                                           // Last 3 commands are zeros
};

// Initialization  command sequence
const u16 initialization_cmd_sequence[35] = {
    0xFF00, 0xFF00, // Two dummy reads (read channel 63)
    0x80DE, // write register 0  - (fast settle off and other specified values)
    0x8142, // write register 1  - (Vdd sense enable + ADC buffer bias = 2)
    0x8204, // write register 2  - (Mux Bias = 4)
    0x8302, // write register 3  - (temperature sensor disabled, digital output in HiZ)
    0x849C, // write register 4  - (Weak MISO, not twos complement or abs mode, DSPen=True, Cutoff = 1.1658 Hz at 30kHz (cutoff freq=12))
    0x8500, // write register 5  - (Disable impedance check stuff)
    0x8680, // write register 6  - (Impedance DAC to middle value, anyway its disabled)
    0x8700, // write register 7  - (Zcheckp on channel 0, but anyway no Zcheck!)
    0x8811, // write register 8  - (RH1 is on chip, RH1 DAC1=17) (settings for 10 kHz upper filter)
    0x8980, // write register 9  - (Aux1 Enable, RH1 DAC2=0)
    0x8A10, // write register 10 - (RH2 is on chip, RH2 DAC1=16)
    0x8B80, // write register 11 - (Aux2 Enable, RH2 DAC2=0)
    0x8C2C, // write register 12 - (RL is on chip, RL DAC1=44) (settings for 1 Hz lower filter)
    0x8D86, // write register 13 - (Aux3 Enable, RL DAC3=0, RL DAC2=6)
    0x8EFF, // write register 14 - (All amplifiers on)
    0x8FFF, // write register 15 - (All amplifiers on)
    0x90FF, // write register 16 - (All amplifiers on)
    0x91FF, // write register 17 - (All amplifiers on)
    0x92FF, // write register 18 - (All amplifiers on RHD2164)
    0x93FF, // write register 19 - (All amplifiers on RHD2164)
    0x94FF, // write register 20 - (All amplifiers on RHD2164)
    0x95FF, // write register 21 - (All amplifiers on RHD2164)
    0x5500, // Calibrate (need 9 clocks)
    0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00,  // 5 dummy reads to accomplish calibration
    0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00   // 5 more dummy reads to accomplish calibration
};

// Channel conversion command sequence
const u16 cable_length_cmd_sequence[35] = {
    0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00,  // "Dummy reads" register 63 (chip id)   
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00,  // Read registers 40-44 ("INTAN")
    0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00,  // "Dummy reads" register 63 (chip id)   
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00,  // Read registers 40-44 ("INTAN")
    0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00,  // "Dummy reads" register 63 (chip id)   
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00,  // Read registers 40-44 ("INTAN")
    0xFF00, 0xFF00, 0xFF00, 0xFF00, 0xFF00   // "Dummy reads" register 63 (chip id)   
};

// Other interesting ROM registers:
//    63 - // chip id is 1 (RHD2132), 2 (RHD2216), or 4 (RHD2164)
//    62 - // number of amplifiers - 16, 32, or 64
//    61 - //unipolar or bipolar (should be 0x0001 = unipolar)
//    60 - // Die revision
//    59 - // MISO A/B (different data on A and B)
//    48, 49, 50, 51, 52, 53, 54, 55 - could be string version of chip name

// Test pattern with incrementing values
const u16 mosi_test_pattern[35] = {
    0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007,
    0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000E, 0x000F,
    0x0010, 0x0011, 0x0012, 0x0013, 0x0014, 0x0015, 0x0016, 0x0017,
    0x0018, 0x0019, 0x001A, 0x001B, 0x001C, 0x001D, 0x001E, 0x001F,
    0x0020, 0x0021, 0x0022
};
