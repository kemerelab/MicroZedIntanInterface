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
    // These are three DIFFERENT things, not a homogeneous array: only slots 1 and
    // 2 cycle (each is one aux_program); slot 0 is a fixed register. See
    // aux_command_engine.sv.
    localparam int AUX_FS_SLOT     = 0;  // cycle 32: fixed RT register; fast-settle whole-replaces it
    localparam int AUX_PLAIN_SLOT  = 1;  // cycle 33: cycling program (ADC/accel sweep)
    localparam int AUX_INJECT_SLOT = 2;  // cycle 34: cycling program (housekeeping) + inject target

    // ---- SPI readback pipeline ----
    // The chip's reply to the command at cycle C is captured at cycle C+SPI_READBACK_LAT.
    localparam int SPI_READBACK_LAT     = 2;
    // So the injected command (cycle AUX_CYC0+AUX_INJECT_SLOT = 34) is answered in the NEXT
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
