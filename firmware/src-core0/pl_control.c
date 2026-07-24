// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

#include "main.h"
#include "sleep.h"
#include <stdio.h>
#include <string.h>
#include "xil_io.h"
#include "shared_print.h"

// ============================================================================
// PL CONTROL FUNCTIONS
// ============================================================================

void pl_set_transmission(int enable) {
    uint32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
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
    uint32_t ctrl_reg = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    ctrl_reg |= CTRL_RESET_TIMESTAMP;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
    
    usleep(1000);  // Hold reset for 1ms
    
    ctrl_reg &= ~CTRL_RESET_TIMESTAMP;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg);
    
    send_message("PL timestamp RESET\r\n");
}

void pl_set_loop_count(uint32_t loop_count) {
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_1_OFFSET, loop_count);
    send_message("PL loop count set to %u\r\n", loop_count);
}

void pl_set_phase_select(int phase0, int phase1) {
    uint32_t ctrl_reg_2 = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET);

    ctrl_reg_2 &= ~(CTRL_PHASE0_MASK | CTRL_PHASE1_MASK); // Clear existing phase bits

    ctrl_reg_2 |= ((phase0 & 0xF) << 0); // Set phase0 bits [3:0]
    ctrl_reg_2 |= ((phase1 & 0xF) << 4); // Set phase1 bits [7:4]
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET, ctrl_reg_2);
    send_message("PL phase select (port A) phase0=%d, phase1=%d\r\n", phase0, phase1);
}

// Port B (second cable) CIPO phase / cable-length compensation. Independent of
// port A because the two cables may be different lengths. In CTRL_REG_2:
// phase2 [11:8] (cable B, line 0), phase3 [15:12] (cable B, line 1).
void pl_set_phase_select_b(int phase2, int phase3) {
    uint32_t ctrl_reg_2 = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET);

    ctrl_reg_2 &= ~(CTRL_PHASE2_MASK | CTRL_PHASE3_MASK);
    ctrl_reg_2 |= ((phase2 & 0xF) << 8);
    ctrl_reg_2 |= ((phase3 & 0xF) << 12);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET, ctrl_reg_2);
    send_message("PL phase select (port B) phase2=%d, phase3=%d\r\n", phase2, phase3);
}

void pl_set_debug_mode(int enable) {
    uint32_t ctrl_reg_0 = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET);
    
    if (enable) {
        ctrl_reg_0 |= CTRL_DEBUG_MODE;
        send_message("PL debug mode ENABLED\r\n");
    } else {
        ctrl_reg_0 &= ~CTRL_DEBUG_MODE;
        send_message("PL debug mode DISABLED\r\n");
    }
    
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_0_OFFSET, ctrl_reg_0);
}

// Analytic chirp NCO config (CTRL_REG_3). Latched while transmission inactive
// (like debug_mode); chirp_mode also requires debug_mode to be set.
uint8_t  chirp_cfg_mode = 0, chirp_cfg_stride = 0;
uint16_t chirp_cfg_fspan = 0, chirp_cfg_rate = 0;

void pl_set_chirp(uint8_t mode, uint8_t stride, uint16_t fspan, uint16_t rate) {
    uint32_t cfg = (mode ? CTRL_CHIRP_MODE : 0u)
                 | (((uint32_t)stride << CTRL_CHIRP_STRIDE_SHIFT) & CTRL_CHIRP_STRIDE_MASK)
                 | (((uint32_t)fspan  << CTRL_CHIRP_FSPAN_SHIFT)  & CTRL_CHIRP_FSPAN_MASK)
                 | (((uint32_t)rate   << CTRL_CHIRP_RATE_SHIFT)   & CTRL_CHIRP_RATE_MASK);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_3_OFFSET, cfg);
    chirp_cfg_mode = mode ? 1 : 0;
    chirp_cfg_stride = stride & 0x3F;
    chirp_cfg_fspan = fspan & 0xFFF;
    chirp_cfg_rate = rate & 0xFFF;
    send_message("PL chirp: mode=%u stride=%u fspan=%u rate=%u\r\n",
                 chirp_cfg_mode, chirp_cfg_stride, chirp_cfg_fspan, chirp_cfg_rate);
}

uint32_t pl_get_chirp_cfg(void) {
    return Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_3_OFFSET);
}

void pl_set_channel_enable(int channel_enable) {
    uint32_t ctrl_reg_2 = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET);

    ctrl_reg_2 &= ~CTRL_CHANNEL_ENABLE_MASK; // Clear existing channel enable bits
    ctrl_reg_2 |= ((channel_enable & 0xFF) << CTRL_CHANNEL_ENABLE_SHIFT); // 8-bit, [23:16]

    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_2_OFFSET, ctrl_reg_2);
    send_message("PL channel enable set to 0x%02X (port0=0x%X port1=0x%X)\r\n",
                 channel_enable & 0xFF, channel_enable & 0xF, (channel_enable >> 4) & 0xF);
}

// ============================================================================
// PL STATUS READING FUNCTIONS
// ============================================================================

uint64_t pl_get_timestamp(void) {
    uint32_t status3 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_3_OFFSET);  // Low 32 bits
    uint32_t status4 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_4_OFFSET);  // High 32 bits
    return ((u64_t)status4 << 32) | status3;
}

int pl_is_transmission_active(void) {
    uint32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    return (status0 & STATUS_TRANSMISSION_ACTIVE) ? 1 : 0;
}

uint32_t pl_get_packets_sent(void) {
    return Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_2_OFFSET);  // Moved to register 2
}

int pl_is_loop_limit_reached(void) {
    uint32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    return (status0 & STATUS_LOOP_LIMIT_REACHED) ? 1 : 0;
}

uint32_t pl_get_state_counter(void) {
    uint32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    return (status0 & STATUS_STATE_COUNTER_MASK) >> STATUS_STATE_COUNTER_SHIFT;
}

uint32_t pl_get_cycle_counter(void) {
    uint32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    return (status0 & STATUS_CYCLE_COUNTER_MASK) >> STATUS_CYCLE_COUNTER_SHIFT;
}

uint32_t pl_get_bram_write_address(void) {
    // STATUS_REG_10 packs the PL-domain BRAM packet-boundary pointer (14 bits)
    // together with fifo_count, and the whole word crosses the PL->AXI clock
    // domain through a 2-stage synchronizer (axi_lite_registers.v). That is a
    // multi-bit BINARY CDC: when the pointer advances by one packet several bits
    // change at once, so a read landing in the ~1-2 cycle transition window can
    // return a mixed value that LEADS the data actually committed to BRAM. The
    // fast PS read then runs into the packet tail the PL is still writing, and
    // the cross-port read-during-write returns garbage (seen as out-of-range
    // words at cyc ~27-29, worse/shifting with read speed). Filter the transient
    // by sampling until two reads agree -- the glitch clears within a couple of
    // cycles, so consecutive AXI reads almost always already match.
    uint32_t a = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET) & 0x3FFF;
    for (int i = 0; i < 8; i++) {
        uint32_t b = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET) & 0x3FFF;
        if (b == a)
            return a;            // stable: synchronizer settled, value trustworthy
        a = b;
    }
    return a;                    // pathological churn (won't happen): use latest
}

static uint32_t pl_get_fifo_count(void) {
    uint32_t status10 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_10_OFFSET);
    return (status10 >> 14) & 0x1FF;  // Extract 9-bit FIFO count
}

// ============================================================================
// CONTROL REGISTER READBACK FUNCTIONS
// ============================================================================

uint32_t pl_get_current_loop_count(void) {
    return Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_5_OFFSET);
}

int pl_get_current_phase_select(int *phase0, int *phase1) {
    uint32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);
    *phase0 = (status1 & STATUS_PHASE0_REG_MASK) >> STATUS_PHASE0_REG_SHIFT;
    *phase1 = (status1 & STATUS_PHASE1_REG_MASK) >> STATUS_PHASE1_REG_SHIFT;
    return (status1 & STATUS_DEBUG_MODE_REG) ? 1 : 0;   // Return debug mode status for fun
}

int pl_get_current_debug_mode(void) {
    uint32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);
    return (status1 & STATUS_DEBUG_MODE_REG) ? 1 : 0;
}

int pl_get_current_channel_enable(void) {
    uint32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);
    int lo = (status1 & STATUS_CHANNEL_ENABLE_REG_MASK) >> STATUS_CHANNEL_ENABLE_REG_SHIFT;
    int hi = (status1 & STATUS_CHANNEL_ENABLE_HI_REG_MASK) >> STATUS_CHANNEL_ENABLE_HI_REG_SHIFT;
    return (hi << 4) | lo;   // full 8-bit channel enable (both ports)
}

// uint32_t pl_get_current_control_0_flags(void) {
//     return Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET); // Reflected
// }


// ============================================================================
// STATUS DISPLAY FUNCTIONS
// ============================================================================


void pl_print_status(void) {
    uint32_t status0 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_0_OFFSET);
    uint32_t status1 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_1_OFFSET);

    send_message("=== PL DYNAMIC STATUS ===\r\n");
    send_message("Transmission: %s\r\n", (status0 & STATUS_TRANSMISSION_ACTIVE) ? "ACTIVE" : "STOPPED");
    send_message("Loop limit reached: %s\r\n", (status0 & STATUS_LOOP_LIMIT_REACHED) ? "YES" : "NO");
    send_message("State counter: %u\r\n", pl_get_state_counter());
    send_message("Cycle counter: %u\r\n", pl_get_cycle_counter());
    send_message("Packets sent: %u\r\n", pl_get_packets_sent());
    send_message("Timestamp: %llu\r\n", pl_get_timestamp());
    send_message("BRAM write address: %u\r\n", pl_get_bram_write_address());
    send_message("FIFO count: %u\r\n", pl_get_fifo_count());

    
    uint32_t status6 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_6_OFFSET);
    uint32_t status7 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_7_OFFSET);
    uint32_t status8 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_8_OFFSET);
    uint32_t status9 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_9_OFFSET);
    send_message("Mirrored Control flags 0-3: \r\n0x%08X 0x%08X 0x%08X 0x%08X", status6, status7, status8, status9);

    send_message("\r\n=== REGISTERED CONTROL PARAMETERS ===\r\n");
    send_message("  Loop count: %u\r\n", pl_get_current_loop_count());
    
    int phase0, phase1;
    pl_get_current_phase_select(&phase0, &phase1);
    send_message("  Phase select: CIPO0=%d, CIPO1=%d\r\n", phase0, phase1);
    send_message("  Debug mode: %s\r\n", pl_get_current_debug_mode() ? "ENABLED (dummy data)" : "DISABLED (real CIPO)");
    send_message("  Channel enable: 0x%X\r\n", pl_get_current_channel_enable());    

    send_message("================================\r\n");
}

// Simple BRAM dump for debugging
void pl_dump_bram_data(uint32_t start_addr, uint32_t word_count) {
    // DIAGNOSTIC: read the SAME region two ways and compare --
    //   burst  = memcpy (AXI burst, exactly what the streaming path does)
    //   single = Xil_In32 (single-beat, what dump always used; known clean)
    // Run this while STOPPED. Any "<-- DIFF" line means the burst read itself
    // corrupted that word on a STATIC BRAM (no concurrent PL write) -> the bug
    // is the multi-burst read path, not read-during-write, so DMA/double-buffer
    // would NOT help. No DIFFs (but streaming is still corrupt) -> it IS
    // read-during-write and double-buffering is the fix.
    // Same characteristics as the streaming udp_packet_buffer (cacheable DDR,
    // 64-byte aligned) so the AXI burst shape matches the real read path.
    static uint32_t dump_burst_buf[MAX_WORDS_PER_PACKET] __attribute__((aligned(64)));
    uint32_t cap = word_count;
    if (cap > MAX_WORDS_PER_PACKET) cap = MAX_WORDS_PER_PACKET;  // memcpy into the local buffer
    memcpy(dump_burst_buf, (void*)(BRAM_BASE_ADDR + start_addr * 4), cap * 4);
    send_message("BRAM dump from %u (idx: burst | single | flag):\r\n", start_addr);
    uint32_t diffs = 0;
    for (uint32_t i = 0; i < word_count; i++) {
        uint32_t addr = (start_addr + i) % BRAM_SIZE_WORDS;
        uint32_t single = Xil_In32(BRAM_BASE_ADDR + addr * 4);  // DMA-EXEMPT: dump_bram debug -- single-beat reference reader compared against the burst memcpy; not a hot-path bulk transfer
        if (i < cap) {
            uint32_t burst = dump_burst_buf[i];
            if (burst != single) diffs++;
            send_message("%u: 0x%08X | 0x%08X %s\r\n", i, burst, single,
                         (burst != single) ? "<-- DIFF" : "");
        } else {
            send_message("%u: ---------- | 0x%08X\r\n", i, single);
        }
    }
    send_message("burst-vs-single diffs: %u (run STOPPED; DIFF => burst read bug, not RDW)\r\n", diffs);
}

// ============================================================================
// ============================================================================
// INTAN COMMAND CONFIGURATION SENT FROM FPGA TO CHIPs (COPI / MOSI)
// ============================================================================
// ============================================================================

// Each 35-command SPI frame is 32 channel-cycle commands (this table) plus the 3
// aux cycles, which are sourced by the aux command engine (not this table).

// Set the 32 channel-cycle COPI command words from an array of 16-bit values.
void pl_set_copi_commands(const uint16_t copi_array[32]) {
    // Stored in control registers 4-19 (16 registers), two 16-bit words each:
    // low 16 = even-indexed word (0,2,4,...), high 16 = odd-indexed word (1,3,5,...).
    for (int i = 0; i < 16; i++) {
        uint32_t reg_value = (uint32_t)copi_array[2*i]                // low: even index
                           | ((uint32_t)copi_array[2*i + 1] << 16);  // high: odd index
        uint32_t reg_offset = CTRL_REG_MOSI_START_OFFSET + (i * 4);
        Xil_Out32(PL_CTRL_BASE_ADDR + reg_offset, reg_value);
    }

    send_message("COPI commands updated\r\n");
}

// ============================================================================
// SAFE COPI COMMAND UPDATING
// ============================================================================

// Safely update COPI commands only when transmission is disabled
int pl_set_copi_commands_safe(const uint16_t copi_array[32], const char* sequence_name) {
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
    pl_rhd_shadow_init();   // re-seed the register shadow to the init defaults
}

void pl_set_cable_length_sequence(void) {
    if (pl_set_copi_commands_safe(cable_length_cmd_sequence, "CABLE LENGTH test sequence")) {
        send_message("Ready for cable length calibration - look for 'INTAN' patterns in data\r\n");
    }
}

// ============================================================================
// PREDEFINED COPI COMMAND ARRAYS
// ============================================================================
// Notes:
// Register WRITE is 10AA_AAAA VVVV_VVVV
// Register READ is  11AA_AAAA 0000_0000
// Convert is 00CC_CCCC 0000_000X, where X=1 is part of the fast-settle routine


// Channel conversion command sequence
const uint16_t convert_cmd_sequence[32] = {
    0x0000, 0x0100, 0x0200, 0x0300, 0x0400, 0x0500, 0x0600, 0x0700,  // Channels 0-7
    0x0800, 0x0900, 0x0A00, 0x0B00, 0x0C00, 0x0D00, 0x0E00, 0x0F00,  // Channels 8-15
    0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1500, 0x1600, 0x1700,  // Channels 16-23
    0x1800, 0x1900, 0x1A00, 0x1B00, 0x1C00, 0x1D00, 0x1E00, 0x1F00   // Channels 24-31
    // aux cycles 32-34 (the aux ADC reads) are sourced by the aux command engine
};

// Initialization command sequence
const uint16_t initialization_cmd_sequence[32] = {
    0xFF00, 0xFF00, // Two dummy reads (read channel 63)
    0x80DE, // write register 0  - (fast settle off and other specified values)
    0x8142, // write register 1  - (Vdd sense enable + ADC buffer bias = 2)
    0x8204, // write register 2  - (Mux Bias = 4)
    0x8302, // write register 3  - (temperature sensor disabled, digital output in HiZ)
    0x849C, // write register 4  - (Weak MISO, not twos complement or abs mode, DSPen=True, Cutoff = 1.1658 Hz at 30kHz (cutoff freq=12))
    0x8500, // write register 5  - (Disable impedance check stuff)
    0x8680, // write register 6  - (Impedance DAC to middle value, anyway its disabled)
    0x8700, // write register 7  - (Zcheckp on channel 0, but anyway no Zcheck!)
    0x8816, // write register 8  - (RH1 is on chip, RH1 DAC1=22) (settings for 7.5 kHz upper filter)
    0x8980, // write register 9  - (Aux1 Enable, RH1 DAC2=0)
    0x8A17, // write register 10 - (RH2 is on chip, RH2 DAC1=23)
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
    0xFF00, 0xFF00                           // + 2 = 7 here; the aux cycles 32-34 add 3 more -> 10 total
};

// Cable length test command sequence
const uint16_t cable_length_cmd_sequence[32] = {
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00, 0xFF00, 0xFB00,  // Read 40-44 ("INTAN"), chip ID, and MISO register
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00, 0xFF00, 0xFB00,  // Read 40-44 ("INTAN"), chip ID, and MISO register
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00, 0xFF00, 0xFB00,  // Read 40-44 ("INTAN"), chip ID, and MISO register
    0xE800, 0xE900, 0xEA00, 0xEB00, 0xEC00, 0xFF00, 0xFB00,  // Read 40-44 ("INTAN"), chip ID, and MISO register
    0xE800, 0xE900, 0xEA00, 0xEB00                           // Read 40-43 (cycles 32-34 are aux-sourced)
};

// Other interesting ROM registers:
//    63 - // chip id is 1 (RHD2132), 2 (RHD2216), or 4 (RHD2164)
//    62 - // number of amplifiers - 16, 32, or 64
//    61 - //unipolar or bipolar (should be 0x0001 = unipolar)
//    60 - // Die revision
//    59 - // MISO A/B (different data on A and B)
//    48, 49, 50, 51, 52, 53, 54, 55 - could be string version of chip name

// ============================================================================
// AUX COMMAND SEQUENCER CONTROL
// ============================================================================
// The banked aux command engine ALWAYS sources COPI cycles 32..34 (aux-default).
// Banks are double-buffered: upload to the standby bank (allowed DURING
// acquisition), select it, then confirm the swap landed (bank_active poll).

// One word through the bank write port: payload to reg 23, then flip the
// write toggle in reg 24 (the PL edge-detects the toggle into a 1-cycle
// strobe; the payload is long stable by the time the toggle crosses the CDC).
static void aux_strobe_write(uint32_t payload) {
    uint32_t strobe = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_STROBE_OFFSET);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_WRITE_OFFSET, payload);
    strobe ^= AUX_STROBE_WRITE_TOGGLE;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_STROBE_OFFSET, strobe);
    usleep(2);   // > a few PL clocks for the CDC + strobe
}

// slot is the write target (= slot index): 0 = RT command register, 1 = the
// cycling program, 2 = the inject command register. For the two registers (0, 2)
// bank/addr are ignored by the PL (single word).
void pl_aux_write_word(int slot, int bank, int addr, uint16_t data) {
    aux_strobe_write(AUX_WRITE_PACK(slot, bank, 0, addr, data));
}

void pl_aux_write_length(int slot, int bank, int loop_idx, int end_idx) {
    aux_strobe_write(AUX_WRITE_PACK(slot, bank, 1, 0, AUX_LENGTH_DATA(loop_idx, end_idx)));
}

// Upload a whole program (commands + its length record) into one bank.
// loop_idx < n allows a run-once preamble: entries 0..loop_idx-1 play once,
// then loop_idx..n-1 loop forever.
int pl_aux_upload_bank(int slot, int bank, const uint16_t *cmds, int n, int loop_idx) {
    if (n < 1 || n > AUX_BANK_ENTRIES || loop_idx < 0 || loop_idx >= n) {
        send_message("ERROR: aux bank upload: bad length %d / loop %d\r\n", n, loop_idx);
        return 0;
    }
    for (int i = 0; i < n; i++)
        pl_aux_write_word(slot, bank, i, cmds[i]);
    pl_aux_write_length(slot, bank, loop_idx, n - 1);
    send_message("Aux slot %d bank %d: %d commands loaded (loop at %d)\r\n",
                 slot, bank, n, loop_idx);
    return 1;
}

void pl_aux_select_bank(int slot, int bank) {
    if (slot != 0) return;                 // only slot 0 (the program) has a bank
    uint32_t ctrl = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET);
    uint32_t bit = AUX_CTRL_BANK_BIT(slot); // slot 0 -> reg22 bit 0
    if (bank) ctrl |= bit; else ctrl &= ~bit;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET, ctrl);
}

// Confirm-before-reuse handshake: the swap latches at a packet boundary
// (immediately when not streaming). Returns 1 once bank_active[slot]==bank.
int pl_aux_confirm_bank(int slot, int bank, int timeout_ms) {
    if (slot != 0) return 1;               // only slot 0 has a bank; others confirmed trivially
    for (int waited = 0; waited <= timeout_ms * 1000; waited += 100) {
        uint32_t s11 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_11_OFFSET);
        if (((s11 >> slot) & 1u) == (uint32_t)(bank ? 1 : 0))
            return 1;
        usleep(100);
    }
    send_message("ERROR: aux bank swap confirm timeout (slot %d bank %d)\r\n", slot, bank);
    return 0;
}

// pl_aux_seq_enable / pl_aux_seq_is_enabled retired: the aux engine is always on
// (aux-default). There is no enable/disable, and hence no OFF-injection ordering
// to manage -- clearing fast-settle now always reaches the chip.

// cfg carries the AUX_CTRL fast-settle + DSP fields (bits [13:4])
void pl_aux_set_fast_settle(uint32_t cfg) {
    const uint32_t mask = AUX_CTRL_FS_SW | AUX_CTRL_FS_GPIO_EN | AUX_CTRL_FS_GPIO_SEL_MASK |
                          AUX_CTRL_DSP_SW | AUX_CTRL_DSP_GPIO_EN | AUX_CTRL_DSP_GPIO_SEL_MASK;
    uint32_t ctrl = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET);
    ctrl = (ctrl & ~mask) | (cfg & mask);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET, ctrl);
    send_message("Fast settle config: sw=%d gpio_en=%d pin=%d dsp_sw=%d dsp_gpio=%d dsp_pin=%d\r\n",
                 !!(cfg & AUX_CTRL_FS_SW), !!(cfg & AUX_CTRL_FS_GPIO_EN),
                 (int)((cfg & AUX_CTRL_FS_GPIO_SEL_MASK) >> AUX_CTRL_FS_GPIO_SEL_SHIFT),
                 !!(cfg & AUX_CTRL_DSP_SW), !!(cfg & AUX_CTRL_DSP_GPIO_EN),
                 (int)((cfg & AUX_CTRL_DSP_GPIO_SEL_MASK) >> AUX_CTRL_DSP_GPIO_SEL_SHIFT));
}

// cfg carries the digout fields (bits [18:14]) + reg3_static (bits [31:24])
void pl_aux_set_digout(uint32_t cfg) {
    const uint32_t mask = AUX_CTRL_DIGOUT_SW | AUX_CTRL_DIGOUT_GPIO_EN |
                          AUX_CTRL_DIGOUT_GPIO_SEL_MASK | AUX_CTRL_REG3_STATIC_MASK;
    uint32_t ctrl = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET);
    ctrl = (ctrl & ~mask) | (cfg & mask);
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_CTRL_OFFSET, ctrl);
    send_message("Digout config: sw=%d gpio_en=%d pin=%d reg3_static=0x%02X\r\n",
                 !!(cfg & AUX_CTRL_DIGOUT_SW), !!(cfg & AUX_CTRL_DIGOUT_GPIO_EN),
                 (int)((cfg & AUX_CTRL_DIGOUT_GPIO_SEL_MASK) >> AUX_CTRL_DIGOUT_GPIO_SEL_SHIFT),
                 (unsigned)((cfg & AUX_CTRL_REG3_STATIC_MASK) >> AUX_CTRL_REG3_STATIC_SHIFT));
}

// One-shot command injection via the inject slot (the engine whole-replaces the
// slot-2 register for one packet). Requires streaming; the response returns two
// SPI commands later and is captured into STATUS_REG_12.
int pl_aux_inject(uint16_t cmd, uint32_t *result, int timeout_ms) {
    if (!pl_is_transmission_active()) {
        send_message("ERROR: inject requires streaming\r\n");
        return 0;
    }
    uint32_t ack_before = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_11_OFFSET) & AUX_STATUS_INJECT_ACK;
    uint32_t strobe = Xil_In32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_STROBE_OFFSET);
    strobe = (strobe & ~(0xFFFFu << AUX_STROBE_INJECT_CMD_SHIFT))
             | ((uint32_t)cmd << AUX_STROBE_INJECT_CMD_SHIFT);
    strobe ^= AUX_STROBE_INJECT_TOGGLE;
    Xil_Out32(PL_CTRL_BASE_ADDR + CTRL_REG_AUX_STROBE_OFFSET, strobe);

    // ack flips when the response lands (next packet, cycle 1): ~70 us typ.
    for (int waited = 0; waited <= timeout_ms * 1000; waited += 50) {
        uint32_t s11 = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_11_OFFSET);
        if ((s11 & AUX_STATUS_INJECT_ACK) != ack_before) {
            if (result)
                *result = Xil_In32(PL_CTRL_BASE_ADDR + STATUS_REG_12_OFFSET);
            return 1;
        }
        usleep(50);
    }
    send_message("ERROR: inject ack timeout (cmd 0x%04X)\r\n", cmd);
    return 0;
}

// ---- RHD register shadow (the chip's commanded register state) --------------
// Mirror of RHD registers 0..21 as commanded. Seeded from the init sequence
// (pl_rhd_shadow_init) and updated by pl_write_rhd_register. Regs 0 and 3 are
// OWNED by the override layer (D5 fast-settle / Reg-3 digout): their shadow keeps
// the init base, and the live override bits are reported separately via
// aux_ctrl/aux_flags. Reported in get_status ("report everything configurable").
uint8_t rhd_reg_shadow[22] = {0};

void pl_rhd_shadow_init(void) {
    // Record every WRITE in the init sequence. WRITE(reg,val) = 0x8000|reg<<8|val,
    // i.e. (cmd & 0xC000) == 0x8000 (CONVERT is 0x0xxx, READ is 0xCxxx).
    for (int i = 0; i < 32; i++) {
        uint16_t cmd = initialization_cmd_sequence[i];
        if ((cmd & 0xC000) == 0x8000) {
            int reg = (cmd >> 8) & 0x3F;
            if (reg < (int)sizeof(rhd_reg_shadow))
                rhd_reg_shadow[reg] = (uint8_t)(cmd & 0xFF);
        }
    }
}

int pl_read_rhd_register(int reg, uint32_t *result) {
    return pl_aux_inject(RHD_CMD_READ(reg), result, 10);
}

int pl_write_rhd_register(int reg, uint8_t value, uint32_t *result) {
    // Note: WRITE(0)/WRITE(3) pass through the override layer's coherence
    // rules (D5 forced to the live fast-settle state; Reg-3 data replaced by
    // the shadow) -- use pl_aux_set_fast_settle / pl_aux_set_digout for those.
    int rc = pl_aux_inject(RHD_CMD_WRITE(reg, value), result, 10);
    // Mirror the commanded value on a successful inject; skip regs 0/3 (those are
    // override-owned, so their shadow keeps the init base).
    if (rc && reg > 0 && reg != 3 && reg < (int)sizeof(rhd_reg_shadow))
        rhd_reg_shadow[reg] = value;
    return rc;
}

// ============================================================================
// CABLE TEST IMPLEMENTATION
// ============================================================================

void pl_run_full_cable_test(void) {
    send_message("=== STARTING FULL CABLE LENGTH TEST ===\r\n");
    
    // Check if streaming is active - must be stopped for this test
    if (pl_is_transmission_active()) {
        send_message("ERROR: Cannot run cable test while transmission is active\r\n");
        send_message("       Stop transmission first with 'stop' command\r\n");
        return;
    }
    
    // Set loop count to 1 for single packet acquisitions
    pl_set_loop_count(1);
    usleep(10000);  // 10ms delay
    
    // Step 1: Run initialization sequence (1 packet)
    send_message("Running initialization sequence...\r\n");
    pl_set_copi_commands(initialization_cmd_sequence);
    
    // Enable transmission for init
    pl_set_transmission(1);
    usleep(100000);  // Wait 100ms for init to complete
    pl_set_transmission(0);
    usleep(10000);  // 10ms delay to ensure packet is in BRAM
    
    // Step 2: Set cable test sequence
    send_message("Setting cable test sequence...\r\n");
    pl_set_copi_commands(cable_length_cmd_sequence);

    // Step 3: Generate cable test packets for all phase combinations
    send_message("Generating cable test packets...\r\n");
    
    for (int phase = 0; phase < 16; phase++) {  // Vary phase from 0-15
        send_message("Testing phase0=%d, phase1=%d\r\n", phase, phase);
        
        // Set phase values
        pl_set_phase_select(phase, phase);
        usleep(10000);  // 10ms delay for settings to take effect
        
        // Acquire one packet
        pl_set_transmission(1);
        usleep(10000);  // Wait 10ms for one packet
        pl_set_transmission(0);
        usleep(5000);   // 5ms delay between acquisitions
    }
    
    send_message("\r\n=== CABLE TEST DATA ACQUISITION COMPLETE ===\r\n");
    send_message("\r\nTo analyze results:\r\n");
    send_message("  1. Check received packets for 'INTAN' pattern\r\n");
    send_message("  2. Look for 0x0049 ('I') in word indices 8,9\r\n");
    send_message("  3. Use optimal phase settings found\r\n");
}
