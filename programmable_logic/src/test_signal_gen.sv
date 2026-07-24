// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// test_signal_gen.sv
//
// Synthetic acquisition data for bring-up and loopback tests, split out of
// data_generator_core so the core is only about *framing* real CIPO data.
// Two synthetic sources share one 512-entry sine ROM:
//
//   * fixed-frequency debug sine  (chirp_mode = 0): 8 lanes at 1x/2x/4x/8x the
//     base sine, port-1 lanes phase-offset by half a period, marching one LUT
//     step per packet.
//   * analytic swept sine, "chirp" (chirp_mode = 1): a memory-free dual-
//     accumulator NCO -- freq triangles 0<->f_max, phase integrates it, and each
//     slot/lane gets a distinguishing phase offset so all 8 lanes x 32 slots are
//     visibly distinct.
//
// The module is purely a SOURCE and holds no framing/SPI state: pulse
// packet_advance once per emitted packet to march the sine index + NCO, drive
// cycle_counter with the slot currently being built, and read the combinational
// `lanes` at the data-word write. Output is byte-identical to the code this
// replaces (same lane order as fifo_write_data).

module test_signal_gen (
    input  logic         clk,
    input  logic         rstn,

    // March the sine index + chirp NCO once per emitted packet.
    input  logic         packet_advance,
    // Current frame slot (drives per-channel phase + the debug channel offset).
    input  logic [5:0]   cycle_counter,

    // Raw chirp config register (CTRL_REG_3). This module decodes AND latches it
    // itself (below), so the framing loop never touches chirp fields.
    input  logic [31:0]  chirp_cfg_reg,
    // Low => idle: the config is captured now and frozen for the whole stream.
    input  logic         transmission_active,

    // 8 x 16-bit synthetic lanes for the current slot (combinational; the core
    // samples it at the data-word write, same lane order as fifo_write_data).
    output logic [127:0] lanes
);

// ---- chirp NCO geometry -----------------------------------------------------
localparam int CHIRP_PHW          = 32;              // phase/freq accumulator width
localparam int CHIRP_LUT_IDX_HI   = CHIRP_PHW - 1;   // top 9 bits [31:23] index the LUT
localparam int CHIRP_LUT_IDX_LO   = CHIRP_PHW - 9;
localparam int CHIRP_FSPAN_SHIFT  = 16;              // f_span (12b) -> f_max in phase units
localparam int CHIRP_RATE_SHIFT   = 9;               // sweep_rate (12b) -> freq incr/packet
localparam int CHIRP_STRIDE_SHIFT = 24;              // per-channel phase-stride placement

// ---- CTRL_REG_3 decode + latch ----------------------------------------------
// Captured while idle, frozen during a stream (so a mid-stream host write can't
// tear the sweep). Encoding:
//   [0]     chirp_mode   (1 = swept chirp, 0 = fixed-frequency sine)
//   [1]     reserved
//   [7:2]   phase_stride (6-bit per-channel phase offset stride)
//   [19:8]  f_span       (12-bit; f_max     = f_span     << CHIRP_FSPAN_SHIFT)
//   [31:20] sweep_rate   (12-bit; incr/pkt  = sweep_rate << CHIRP_RATE_SHIFT)
logic        chirp_mode;
logic [5:0]  chirp_stride;
logic [11:0] chirp_fspan;
logic [11:0] chirp_rate;
always_ff @(posedge clk) begin
    if (!rstn) begin
        chirp_mode <= 1'b0;  chirp_stride <= 6'd0;  chirp_fspan <= 12'd0;  chirp_rate <= 12'd0;
    end else if (!transmission_active) begin
        chirp_mode   <= chirp_cfg_reg[0];
        chirp_stride <= chirp_cfg_reg[2  +: 6];
        chirp_fspan  <= chirp_cfg_reg[8  +: 12];
        chirp_rate   <= chirp_cfg_reg[20 +: 12];
    end
end

// ---- 512-entry sine ROM (unsigned offset-binary, +-1/16 full scale) ---------
logic [15:0] sine_lut [0:511];
initial begin
    for (int i = 0; i < 512; i++) begin
        real angle = 2.0 * 3.14159265359 * i / 512.0;
        real sine_real = 32767.0 / 16 * $sin(angle) + 32767.0;
        sine_lut[i] = $rtoi(sine_real);
    end
end

// ---- fixed-sine marching index (one LUT step per packet) --------------------
logic [8:0] sine_index;
always_ff @(posedge clk) begin
    if (!rstn)               sine_index <= 9'd0;
    else if (packet_advance) sine_index <= sine_index + 9'd1;
end

// ---- chirp dual-accumulator NCO (advances once per packet) ------------------
// freq_acc triangles between 0 and f_max stepping by rstep; phase_acc integrates
// freq_acc. Clamp at the turning points so the sweep reverses cleanly.
logic [CHIRP_PHW-1:0] chirp_freq_acc;
logic [CHIRP_PHW-1:0] chirp_phase_acc;
logic                 chirp_sweep_up;
wire  [CHIRP_PHW-1:0] chirp_fmax  = {chirp_fspan, {CHIRP_FSPAN_SHIFT{1'b0}}};
wire  [CHIRP_PHW-1:0] chirp_rstep = {{(CHIRP_PHW-12-CHIRP_RATE_SHIFT){1'b0}},
                                     chirp_rate, {CHIRP_RATE_SHIFT{1'b0}}};
always_ff @(posedge clk) begin
    if (!rstn) begin
        chirp_freq_acc  <= '0;
        chirp_phase_acc <= '0;
        chirp_sweep_up  <= 1'b1;
    end else if (packet_advance) begin
        if (chirp_sweep_up) begin
            if (chirp_freq_acc + chirp_rstep >= chirp_fmax) begin
                chirp_freq_acc <= chirp_fmax;
                chirp_sweep_up <= 1'b0;
            end else
                chirp_freq_acc <= chirp_freq_acc + chirp_rstep;
        end else begin
            if (chirp_freq_acc <= chirp_rstep) begin
                chirp_freq_acc <= '0;
                chirp_sweep_up <= 1'b1;
            end else
                chirp_freq_acc <= chirp_freq_acc - chirp_rstep;
        end
        chirp_phase_acc <= chirp_phase_acc + chirp_freq_acc;
    end
end

// Per-slot chirp phase, REGISTERED off cycle_counter so the channel*stride
// multiply lives in its own pipeline stage -- off the cycle->lanes path, which
// otherwise fails 84 MHz. cycle_counter is stable across a slot's 80 states and
// chirp_phase_acc only moves at the packet boundary, so this is settled well
// before the lanes are sampled at the data-word write.
logic [CHIRP_PHW-1:0] chirp_ch_phase;
always_ff @(posedge clk) begin
    logic [5:0]  c_off;
    logic [10:0] s_prod;
    c_off  = (cycle_counter >= 6'd2) ? (cycle_counter - 6'd2) : 6'd0;
    s_prod = c_off[4:0] * chirp_stride;            // 5b*6b, isolated stage
    chirp_ch_phase <= chirp_phase_acc +
                      ({{(CHIRP_PHW-11){1'b0}}, s_prod} <<< CHIRP_STRIDE_SHIFT);
end

// ---- combinational lane generation ------------------------------------------
logic [15:0] chirp_lanes [0:7];
logic [15:0] sine_lanes  [0:7];

always_comb begin
    // Chirp: fan the 8 lanes out by 1/8-period steps so one packet shows 8
    // distinct phases too.
    for (int l = 0; l < 8; l++) begin
        logic [CHIRP_PHW-1:0] lane_phase;
        lane_phase = chirp_ch_phase +
                     ({{(CHIRP_PHW-3){1'b0}}, l[2:0]} <<< (CHIRP_PHW-3));
        chirp_lanes[l] = sine_lut[lane_phase[CHIRP_LUT_IDX_HI:CHIRP_LUT_IDX_LO]];
    end
    // Fixed sine: base sine at 1x/2x/4x/8x; port-1 lanes offset half a period so
    // a doubled-size packet exercises the PS->UDP path with visibly distinct data.
    begin
        logic [5:0] channel_offset;
        logic [8:0] base_phase, base_phase_p1;
        channel_offset = (cycle_counter >= 6'd2) ? (cycle_counter - 6'd2) : 6'd0;
        base_phase     = sine_index + channel_offset;   // 9-bit wrap
        base_phase_p1  = base_phase + 9'd128;           // 90 deg
        sine_lanes[0] = sine_lut[base_phase];                    // 1x
        sine_lanes[1] = sine_lut[(base_phase << 1) & 9'h1FF];    // 2x
        sine_lanes[2] = sine_lut[(base_phase << 2) & 9'h1FF];    // 4x
        sine_lanes[3] = sine_lut[(base_phase << 3) & 9'h1FF];    // 8x
        sine_lanes[4] = sine_lut[base_phase_p1];                 // port1 1x
        sine_lanes[5] = sine_lut[(base_phase_p1 << 1) & 9'h1FF];
        sine_lanes[6] = sine_lut[(base_phase_p1 << 2) & 9'h1FF];
        sine_lanes[7] = sine_lut[(base_phase_p1 << 3) & 9'h1FF];
    end
end

// Lane order: lane 0 = bits[15:0] ... lane 7 = bits[127:112] (== fifo_write_data).
assign lanes = chirp_mode
    ? {chirp_lanes[7], chirp_lanes[6], chirp_lanes[5], chirp_lanes[4],
       chirp_lanes[3], chirp_lanes[2], chirp_lanes[1], chirp_lanes[0]}
    : {sine_lanes[7],  sine_lanes[6],  sine_lanes[5],  sine_lanes[4],
       sine_lanes[3],  sine_lanes[2],  sine_lanes[1],  sine_lanes[0]};

endmodule
