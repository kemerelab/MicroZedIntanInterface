// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// acq_frame_pkg.sv
//
// Single source of truth for the RHD2000 acquisition-frame geometry and the
// chip's SPI command encoding. Every module that touches the COPI command frame
// imports this package, so each architectural fact is stated exactly ONCE and
// the compiler enforces consistency -- no `NSLOTS-1`, no magic `6'd1`, no bare
// `2'b10`, no `cycle_counter[1:0]` arithmetic dressed as a bit-select.
//
// RHD2000 SPI frame, issued once per acquisition packet (one 30 kHz sample):
//   cycles 0 .. N_CHAN_CMDS-1  : amplifier CONVERT commands (32 channels)
//   cycles AUX_CYC0 .. LAST_CYC : the 3 auxiliary commands
//
// The three aux positions are NOT interchangeable -- each has a role fixed by the
// command architecture and the SPI readback pipeline, not by "how many slots exist".

package acq_frame_pkg;

    // ---- frame geometry ----
    localparam int N_FRAME_CMDS = 35;                          // COPI commands per packet
    localparam int N_CHAN_CMDS  = 32;                          // amplifier CONVERTs, cycles 0..N_CHAN_CMDS-1
    localparam int N_AUX        = N_FRAME_CMDS - N_CHAN_CMDS;  // = 3 aux commands
    localparam int AUX_CYC0     = N_CHAN_CMDS;                 // first aux cycle (= 32)
    localparam int LAST_CYC     = N_FRAME_CMDS - 1;            // last cycle (= 34)

    // ---- fixed roles of the aux positions (index 0..N_AUX-1 within the aux group) ----
    // These are three DIFFERENT things, not a homogeneous array: only slot 0 cycles
    // (it is the one aux_program -- the accelerometer / aux-ADC sweep); slots 1 and 2
    // are fixed command registers. See aux_command_engine.sv.
    //
    // The sweep is on slot 0 (cycle 32) DELIBERATELY: with the +2 SPI readback its reply
    // lands in data word 34 of the SAME packet (AUX_SWEEP_REPLY_CYC), so the host pairs
    // each accel axis with its command echo INTRA-packet -- label and sample travel
    // together, and a dropped packet can never desync the axis. The two register slots
    // (cycles 33/34) answer in the next packet; only the injection needs cross-packet
    // tracking.
    localparam int AUX_SWEEP_SLOT  = 0;  // cycle 32: the one cycling program (accel/aux-ADC sweep)
    localparam int AUX_FS_SLOT     = 1;  // cycle 33: fixed register; fast-settle whole-replaces it
    localparam int AUX_INJECT_SLOT = 2;  // cycle 34: fixed register + one-shot inject target

    // ---- SPI readback pipeline ----
    // The chip's reply to the command at cycle C is captured at cycle C+SPI_READBACK_LAT.
    localparam int SPI_READBACK_LAT     = 2;
    // The sweep (cycle AUX_CYC0+AUX_SWEEP_SLOT = 32) is answered in the SAME packet at
    // (32 + 2) = data word 34 -- intra-packet, the whole point of putting it on slot 0.
    localparam int AUX_SWEEP_REPLY_CYC  = (AUX_CYC0 + AUX_SWEEP_SLOT + SPI_READBACK_LAT) % N_FRAME_CMDS;
    // The injected command (cycle AUX_CYC0+AUX_INJECT_SLOT = 34) is answered in the NEXT
    // packet at (34 + 2) mod 35 = cycle 1 -- derived here, never hardcoded downstream.
    localparam int AUX_INJECT_REPLY_CYC = (AUX_CYC0 + AUX_INJECT_SLOT + SPI_READBACK_LAT) % N_FRAME_CMDS;
    // Within that reply cycle the CIPO word[1] settles at SPI state 76 and is latched one
    // state later (a fixed detail of the 80-state serializer, not derivable from the above).
    localparam int AUX_INJECT_REPLY_STATE = 77;

    // ---- RHD2000 command-word encoding (datasheet: "SPI Command Words") ----
    localparam logic [1:0]   RHD_CMD_WRITE   = 2'b10;    // command[15:14] = WRITE
    localparam logic [5:0]   RHD_REG_FS      = 6'd0;     // Register 0: amplifier fast settle (D5) + config
    localparam int           RHD_FS_BIT      = 5;        // fast settle = Reg0 bit D5
    localparam logic [5:0]   RHD_REG_DIGOUT  = 6'd3;     // Register 3: aux digital output (D0)
    localparam logic [15:0]  RHD_WR0_FS_ON   = 16'h80FE; // WRITE(0, 0xFE): amp fast settle ON
    localparam logic [15:0]  RHD_WR0_FS_OFF  = 16'h80DE; // WRITE(0, 0xDE): amp fast settle OFF
    // DSP reset ("digital fast settle") forces the CONVERT LSB (bit H, = bit 0) high.

endpackage
