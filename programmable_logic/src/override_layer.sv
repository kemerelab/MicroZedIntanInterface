// ============================================================================
// override_layer.sv   --   PHASE 4 DRAFT, NOT YET INTEGRATED
// ----------------------------------------------------------------------------
// Real-time override stage that sits AFTER the command-source mux (static table
// / aux_command_sequencer) and BEFORE the COPI serializer in data_generator_core.
// It implements the three real-time chip controls from docs/command-bank-design.md:
//
//   1. Amplifier fast settle  (RHD Reg 0, bit D5)
//   2. Auxiliary digital out  (RHD Reg 3, bit D0) via a coherent Reg-3 shadow
//   3. DSP reset / digital fast settle (the CONVERT LSB, "bit H")
//
// RHD2000 command encoding (authoritative, from firmware/src-core0/pl_control.c):
//   WRITE(addr,val) = 10AAAAAA VVVVVVVV   (WRITE reg0 = 0x80xx, reg3 = 0x83xx)
//   READ(addr)      = 11AAAAAA 00000000
//   CONVERT(ch)     = 00CCCCCC 0000000X   (X = bit0 = DSP-reset / fast-settle bit)
//   Fast settle amp = Reg-0 D5: 0x80FE (on) / 0x80DE (off)
//
// INVARIANT (docs/command-bank-design.md): the override layer only ever *replaces*
// a whole command in the real-time-control slot (cycle RT_SLOT_CYCLE). Slots 2/3
// are never command-replaced; only the *bits* of a WRITE(0)/WRITE(3) are coherently
// substituted wherever those writes appear, so no slot can clobber another's Reg-0
// fast-settle bit or Reg-3 digout/temp bits.
//
// STATUS: elaborates + OOC-synthesizes clean. Behavior is NOT verified against
// hardware or in simulation yet; bit positions must be reconfirmed against the
// RHD2000 datasheet before relying on it. See docs/NIGHT_LOG.md.
// ============================================================================
`timescale 1ns/1ps

module override_layer #(
    parameter integer RT_SLOT_CYCLE = 32   // real-time-control slot = aux cycle 32 (slot 0)
)(
    input  wire        clk,
    input  wire        rstn,

    // Position of the command currently being issued, and the per-packet edge.
    input  wire [5:0]  cycle_counter,       // 0..34
    input  wire        packet_start,        // 1-cycle pulse at the first state of a packet

    // Command chosen by the source mux for this cycle, and the overridden result.
    input  wire [15:0] cmd_in,
    output reg  [15:0] cmd_out,

    // ---- amplifier fast settle (Reg-0 D5) ----
    input  wire        fs_amp_en,           // master enable for the amp fast-settle override
    input  wire        fs_trigger,          // selected digital-in pin (already pin-muxed/synced)

    // ---- DSP reset / digital fast settle (CONVERT bit H) ----
    input  wire        dsp_reset_en,        // enable forcing the CONVERT LSB
    input  wire        dsp_reset_trigger,   // selected pin / software bit

    // ---- Reg-3 shadow (auxiliary digital out + temperature bits) ----
    input  wire [7:0]  reg3_static,         // host-owned static bits: MUX/HiZ + tempen/tempS (D1..D7)
    input  wire        digout_en,           // enable mirroring digout_src into Reg-3 D0
    input  wire        digout_src,          // GPIO/TTL mirrored to the headstage auxout pin

    // ---- status / packet metadata ----
    output wire        fast_settle_active,  // current latched amp fast-settle state (Reg-0 D5)
    output wire        digout_state         // current digout bit driven into Reg-3 D0
);

    // ------------------------------------------------------------------------
    // Per-packet fast-settle state. Latched once per packet (the chip latches
    // Reg 0 once; the doc requires fast settle change at most once per packet).
    // fs_inject fires on the packet where the trigger CHANGED, so a pulse yields
    // two injections (one on the rising edge packet, one on the falling edge).
    // ------------------------------------------------------------------------
    reg fs_state;     // desired Reg-0 D5 value (1 = fast settle on)
    reg fs_inject;    // replace the RT-slot command with a WRITE(0) this packet

    always @(posedge clk) begin
        if (!rstn) begin
            fs_state  <= 1'b0;
            fs_inject <= 1'b0;
        end else if (packet_start) begin
            if (fs_amp_en) begin
                fs_inject <= (fs_trigger != fs_state);  // edge this packet -> inject
                fs_state  <= fs_trigger;
            end else begin
                fs_inject <= 1'b0;
                fs_state  <= 1'b0;                       // disabled -> fast settle off
            end
        end else begin
            fs_inject <= 1'b0;                           // injection is a single-packet decision
        end
    end

    // ------------------------------------------------------------------------
    // Combinational command classification + override.
    // ------------------------------------------------------------------------
    wire is_write_reg0 = (cmd_in[15:14] == 2'b10) && (cmd_in[13:8] == 6'd0);
    wire is_write_reg3 = (cmd_in[15:14] == 2'b10) && (cmd_in[13:8] == 6'd3);
    wire is_convert    = (cmd_in[15:14] == 2'b00);
    wire is_chan_conv  = is_convert && (cmd_in[13:8] <= 6'd31);   // channel converts 0..31
    wire is_rt_slot    = (cycle_counter == RT_SLOT_CYCLE[5:0]);

    // Coherent Reg-3 shadow: keep host static bits, substitute live digout in D0.
    wire        digout_bit  = digout_en ? digout_src : reg3_static[0];
    wire [7:0]  reg3_shadow = {reg3_static[7:1], digout_bit};

    // The whole-command replacement used for fast-settle injection (RT slot only).
    wire [15:0] fs_cmd = fs_state ? 16'h80FE : 16'h80DE;

    always @(*) begin
        // Default: pass the command through unchanged.
        cmd_out = cmd_in;

        // ---- bit-level coherence (allowed in ANY slot; not a "replacement") ----
        if (is_write_reg0) begin
            cmd_out = {cmd_in[15:6], fs_state, cmd_in[4:0]};   // force D5 = live fast-settle
        end else if (is_write_reg3) begin
            cmd_out = {8'h83, reg3_shadow};                    // substitute Reg-3 shadow
        end else if (is_chan_conv && dsp_reset_en && dsp_reset_trigger) begin
            cmd_out = {cmd_in[15:1], 1'b1};                    // force CONVERT bit H (DSP reset)
        end

        // ---- whole-command replacement: RT slot fast-settle injection ----
        // Highest priority and structurally confined to the RT slot to honor the
        // "only Slot 1 is ever command-replaced" invariant.
        if (fs_amp_en && fs_inject && is_rt_slot) begin
            cmd_out = fs_cmd;
        end
    end

    assign fast_settle_active = fs_state;
    assign digout_state       = digout_bit;

endmodule
