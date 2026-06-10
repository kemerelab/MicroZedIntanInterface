// ============================================================================
// aux_capture.sv   --   PHASE 4 DRAFT, NOT YET INTEGRATED
// ----------------------------------------------------------------------------
// Command-echo identity for the aux results. The RHD SPI link has a fixed
// readback pipeline: the data the core latches for cycle C (cipo*_data[C]) is the
// *response to the command issued PIPE cycles earlier* (cipo_data[C] = resp(cmd
// C-PIPE); today PIPE = 2, see firmware/src-core0/pl_control.c:290 "2 sample
// delay"). This unit carries the originating command alongside the pipeline so
// each aux result can be emitted as a self-describing triple:
//
//     { originating_command , cipo0_result , cipo1_result }   per aux slot
//
// so the host decodes aux channels (temp / supply / accel axis / register read)
// from the echoed command with zero knowledge of the loaded program, and a
// dropped UDP packet never desyncs (docs/command-bank-design.md "command-echo").
//
// Boundary behavior: the aux commands at cycles 32/33/34 have their results at
// cycles 34 / 0(next) / 1(next). Slot-0's result lands in the same packet; slots
// 1 and 2 wrap into the next packet, which is exactly why the originating command
// must be carried across the boundary (this delay line does that for free).
//
// STATUS: elaborates + OOC-synthesizes clean. The exact PIPE depth and the
// cycle-to-slot alignment are the empirically-tuned part Intan calls out
// ("channel_MISO<=33, Bug fix: changed 2 to 33") and MUST be verified in
// simulation against the real datapath before use. See docs/NIGHT_LOG.md.
// ============================================================================
`timescale 1ns/1ps

module aux_capture #(
    parameter integer PIPE      = 2,    // readback pipeline depth: result[C] = resp(cmd C-PIPE)
    parameter integer FIRST_AUX = 32,   // first aux cycle (slots occupy FIRST_AUX .. FIRST_AUX+2)
    parameter integer N_SLOTS   = 3
)(
    input  wire        clk,
    input  wire        rstn,

    // Driven once per command cycle, when both the issued command and the result
    // latched for this cycle_counter are valid (e.g. pulse at the result-capture
    // state of the 80-state loop). cycle_counter is the cycle being captured.
    input  wire        capture_en,
    input  wire [5:0]  cycle_counter,

    input  wire [15:0] cmd_issued,     // command issued for this cycle (post-override)
    input  wire [31:0] cipo0_result,   // phase-selected CIPO0 readback latched this cycle
    input  wire [31:0] cipo1_result,   // phase-selected CIPO1 readback latched this cycle

    // Latched, labeled aux triples (slot i at the i-th field). Updated as each
    // aux result returns; read by the exfil FSM into the packet metadata words.
    output reg  [N_SLOTS*16-1:0] aux_cmd_echo,   // originating command per slot
    output reg  [N_SLOTS*32-1:0] aux_cipo0,      // CIPO0 result per slot
    output reg  [N_SLOTS*32-1:0] aux_cipo1       // CIPO1 result per slot
);

    // ------------------------------------------------------------------------
    // History of issued commands + their originating cycle. After the per-cycle
    // shift, entry [k] holds the command from k cycles ago. We pair the result
    // captured "now" with entry [PIPE] (the command issued PIPE cycles earlier),
    // using the PRE-shift (current) values so there is no read/write race.
    // ------------------------------------------------------------------------
    reg [15:0] cmd_hist    [1:PIPE];
    reg [5:0]  origin_hist [1:PIPE];

    integer k;
    // origin cycle of the result available this capture = entry [PIPE] (pre-shift)
    wire [5:0]  origin_now = origin_hist[PIPE];
    wire [15:0] cmd_now    = cmd_hist[PIPE];
    wire        origin_is_aux = (origin_now >= FIRST_AUX) &&
                                (origin_now <  FIRST_AUX + N_SLOTS);
    wire [5:0]  origin_slot   = origin_now - FIRST_AUX;

    always @(posedge clk) begin
        if (!rstn) begin
            for (k = 1; k <= PIPE; k = k + 1) begin
                cmd_hist[k]    <= 16'h0;
                origin_hist[k] <= 6'h3f;     // not an aux cycle
            end
            aux_cmd_echo <= {N_SLOTS*16{1'b0}};
            aux_cipo0    <= {N_SLOTS*32{1'b0}};
            aux_cipo1    <= {N_SLOTS*32{1'b0}};
        end else if (capture_en) begin
            // 1. Pair this cycle's result with its originating command (pre-shift).
            if (origin_is_aux) begin
                aux_cmd_echo[origin_slot*16 +: 16] <= cmd_now;
                aux_cipo0   [origin_slot*32 +: 32] <= cipo0_result;
                aux_cipo1   [origin_slot*32 +: 32] <= cipo1_result;
            end

            // 2. Shift the history and push this cycle's issued command.
            for (k = PIPE; k >= 2; k = k - 1) begin
                cmd_hist[k]    <= cmd_hist[k-1];
                origin_hist[k] <= origin_hist[k-1];
            end
            cmd_hist[1]    <= cmd_issued;
            origin_hist[1] <= cycle_counter;
        end
    end

endmodule
