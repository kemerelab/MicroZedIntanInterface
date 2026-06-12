// override_layer.sv
//
// Real-time override stage between the aux command sequencer and the COPI
// serializer. Implements the three real-time chip controls of
// docs/command-bank-design.md (bit positions confirmed against the RHD2000
// series datasheet, "SPI Command Words" + Register 0 / Register 3):
//
//   1. Amplifier fast settle - RHD Register 0 bit D5. On a change of the
//      settle level (software bit OR a selected digital-in pin), the SLOT-1
//      command of that packet is REPLACED with WRITE(0, 0xFE) / WRITE(0, 0xDE)
//      (0x80FE on / 0x80DE off; the other Reg-0 bits are the datasheet's
//      recommended values, matching the firmware init blob). Any passing
//      WRITE(0,...) from any slot also gets D5 forced to the live level, so
//      no slot can cancel an active fast settle.
//   2. Auxiliary digital output - RHD Register 3 bit D0, with a coherent
//      shadow: any WRITE(3,...) from any slot has its data byte replaced by
//      {reg3_static[7:1], digout_level}. The host owns the static bits
//      (MUX load D7:5, tempS2/tempS1/tempen D4:2, digout HiZ D1); the PL owns
//      the live D0 mirror. Mirroring works at one-packet latency when slot 1's
//      program contains a WRITE(3, x) every packet.
//   3. DSP reset ("digital fast settle") - the CONVERT LSB, bit H. While the
//      level (software OR pin) is high, dsp_force_h tells the serializer to
//      force bit 0 of every channel CONVERT (cycles 0..31).
//
// All trigger levels are sampled ONCE per packet at packet_start (the same
// instant digital_in is latched for the header), so a mid-packet pin change
// can never tear a serialized command, and fast settle changes at most once
// per packet. The command rewrite itself is combinational on those latched
// values: cmds_out is stable from one clock after packet_start until the next
// packet boundary -- valid both when the header echo is written (state 2) and
// when the aux commands serialize (cycles 32..34).
//
// INVARIANT (command-bank-design.md): only SLOT 1 (cycle 32, bundle field 0)
// is ever whole-command REPLACED. Slots 2 and 3 are never replaced; only the
// bits of a WRITE(0)/WRITE(3) are coherently substituted wherever they appear.
//
// When enable (aux_seq_en) is low every output is a pass-through and all
// status flags are 0 -- the datapath stays bit-identical to today.
//
// Note: clearing fs_sw / fs_gpio_en while enabled produces the final OFF
// injection (1->0 edge). Disabling `enable` itself, however, also reverts the
// core's command-source mux to the legacy path, so an OFF injection emitted
// here could never reach the chip -- firmware must clear the fast-settle
// config and wait one packet BEFORE dropping aux_seq_en (enforced in
// pl_control.c).

module override_layer (
    input  logic clk,
    input  logic rstn,

    input  logic        packet_start,   // 1-cycle pulse, first state of each packet
    input  logic        enable,         // aux_seq_en, latched per packet by the core
    input  logic [7:0]  digital_in,     // raw digital inputs (sampled here)

    // Quasi-static config (CDC-synced control register bits)
    input  logic        fs_sw,          // software fast-settle level
    input  logic        fs_gpio_en,
    input  logic [2:0]  fs_gpio_sel,
    input  logic        dsp_sw,         // software DSP-reset (bit H) level
    input  logic        dsp_gpio_en,
    input  logic [2:0]  dsp_gpio_sel,
    input  logic        digout_sw,      // software digout level
    input  logic        digout_gpio_en,
    input  logic [2:0]  digout_gpio_sel,
    input  logic [7:0]  reg3_static,    // host-owned Reg-3 bits D7..D1 (D0 ignored)

    // Aux command bundle (slot i at [i*16 +: 16]; slot 0 = cycle 32)
    input  logic [47:0] cmds_in,
    output logic [47:0] cmds_out,

    // To the serializer: force bit H on channel CONVERTs (cycles 0..31)
    output logic        dsp_force_h,

    // Status / packet metadata
    output logic        fast_settle_active,
    output logic        digout_state
);

// Live trigger levels (gated by enable so disable creates the OFF edge)
logic fs_level_now, dsp_level_now, digout_level_now;
assign fs_level_now     = enable && (fs_sw     || (fs_gpio_en     && digital_in[fs_gpio_sel]));
assign dsp_level_now    = enable && (dsp_sw    || (dsp_gpio_en    && digital_in[dsp_gpio_sel]));
assign digout_level_now = enable && (digout_sw || (digout_gpio_en && digital_in[digout_gpio_sel]));

// Per-packet latched state
logic fs_state;     // live Reg-0 D5 value
logic fs_inject;    // this packet: replace slot 1 with the WRITE(0) injection
logic dsp_state;
logic digout_level;

always_ff @(posedge clk) begin
    if (!rstn) begin
        fs_state     <= 1'b0;
        fs_inject    <= 1'b0;
        dsp_state    <= 1'b0;
        digout_level <= 1'b0;
    end else if (packet_start) begin
        fs_inject    <= (fs_level_now != fs_state);  // edge -> one injection packet
        fs_state     <= fs_level_now;
        dsp_state    <= dsp_level_now;
        digout_level <= digout_level_now;
    end
end

// Combinational rewrite on the latched per-packet state
logic [7:0] reg3_shadow;
assign reg3_shadow = {reg3_static[7:1], digout_level};

always_comb begin
    cmds_out = cmds_in;
    if (enable) begin
        for (int s = 0; s < 3; s++) begin
            logic [15:0] c;
            c = cmds_in[s*16 +: 16];
            if (c[15:14] == 2'b10 && c[13:8] == 6'd0)
                c[5] = fs_state;                       // WRITE(0,...): force D5
            else if (c[15:14] == 2'b10 && c[13:8] == 6'd3)
                c[7:0] = reg3_shadow;                  // WRITE(3,...): substitute shadow
            cmds_out[s*16 +: 16] = c;
        end
        // Whole-command replacement: slot 1 only (the invariant)
        if (fs_inject)
            cmds_out[15:0] = fs_state ? 16'h80FE : 16'h80DE;
    end
end

assign dsp_force_h        = dsp_state;
assign fast_settle_active = fs_state;
assign digout_state       = digout_level;

endmodule
