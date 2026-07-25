// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// =====================================================================
// lfp_dsp_block.sv
//
// The LFP band, end to end: it takes the acquisition core's sample tap and
// leaves a complete UDP packet in the LFP output BRAM for the PS to DMA out.
//
//   tap (30 kHz)  ->  lfp_halfband_dec2 (/2)  ->  lfp_poly_dec5 (/5)  ->  packet
//
// Two stages rather than one because a filter's transition width is a fraction
// of ITS sample rate: the sharp 1.2/1.8 kHz filter costs ~120 taps at 15 kHz
// instead of ~245 at 30 kHz, and half the delay-line BRAM. Total decimation is
// fixed at /10, so 30 kHz broadband yields a 3 kHz LFP stream.
//
// Responsibilities
//   * gate the tap to amplifier slots and convert offset-binary -> signed
//   * run the cascade, and route coefficient uploads to either stage
//   * BUILD THE COMPLETE WIRE PACKET in the output BRAM -- the shared 8-word
//     header (unified_pkt_pkg, stream_type = LFP) followed by the samples -- so
//     the PS just DMAs [header|samples] into a pbuf and sends it, with no
//     PS-side header maths and no Xil_In32 loop.
//
// Sample placement
// ----------------
// Stage 2 computes several lanes at once, so its outputs do NOT arrive in wire
// order. Rather than buffer and reorder a frame, each sample is written to the
// BRAM address implied by its channel: two 16-bit samples share a 32-bit word
// and the byte-write enables select the half. Arrival order stops mattering,
// and the old sequential pack state machine disappears.
//
// Timestamp convention
// --------------------
// w2/w3 carry the master sample count of the NEWEST broadband sample in this
// output's decimation window. These are FIR filters, so the newest input in an
// output's support is a real, already-acquired sample: for frame m at total
// decimation R that is broadband packet R*m+(R-1) (R=10 -> 10m+9), the same
// count the broadband header stamps. It marks the newest *input*, not the
// instant the filtered value represents -- a host that needs the latter
// subtracts the filter's group delay (reported by the design script).
//
// Stream-specific header words
//   w5 AUX0 = lane_mask | (decim_R<<8) | (num_taps<<16) | (overrun<<24)
//   w6 AUX1 = num_samples = popcount(lane_mask) * N_SLOTS
// =====================================================================

import unified_pkt_pkg::*;   // the 8-word common header shared by every PL stream

module lfp_dsp_block #(
    parameter int N_LANES        = 8,
    parameter int N_SLOTS        = 32,      // amplifier channels per lane
    parameter int DATA_W         = 16,      // broadband sample width
    parameter int MID_W          = 18,      // stage-1 -> stage-2 intermediate
    parameter int COEF_W         = 18,
    parameter int COEF_FRAC      = 17,
    parameter int OUT_W          = 16,      // wire sample width
    parameter int FIRST_AMP_SLOT = 2,       // cycle_counter of amplifier channel 0
    parameter int LFP_BRAM_AW    = 14,      // LFP output BRAM byte-address width (16 KB)
    parameter int HB_TAPS        = 11,      // stage-1 length
    parameter int HB_RING        = 16,      // stage-1 delay line, pow2 >= HB_TAPS
    parameter int MAX_POLY_TAPS  = 120,     // stage-2 maximum length
    parameter int POLY_RING      = 128,     // stage-2 delay line, pow2 >= MAX_POLY_TAPS
    parameter int N_PAR          = 4,       // stage-2 parallel MACs (one per lane)
    parameter int DECIM_1        = 2,
    parameter int DECIM_2        = 5
) (
    input  logic         clk,
    input  logic         rstn,

    // ---- DSP tap from data_generator_core ----
    input  logic         dsp_sample_valid,
    input  logic [N_LANES*DATA_W-1:0] dsp_sample_data,
    input  logic [5:0]   dsp_sample_slot,       // cycle_counter, 0..34
    input  logic         dsp_packet_tick,
    input  logic [63:0]  dsp_master_timestamp,  // live master sample count
    input  logic [7:0]   dsp_channel_enable,    // broadband mask -> LFP lane_mask

    // ---- LFP control register slice (CDC'd) ----
    input  logic [31:0]  lfp_cfg,
    input  logic [31:0]  lfp_coef,
    input  logic [31:0]  lfp_strobe,

    // ---- LFP output BRAM port (PL writes; PS reads via axi_bram_ctrl) ----
    output logic                   bram_clk,
    output logic                   bram_rst,
    output logic [LFP_BRAM_AW-1:0] bram_addr,
    output logic [31:0]            bram_din,
    input  logic [31:0]            bram_dout,    // unused (write-only side)
    output logic                   bram_en,
    output logic [3:0]             bram_we,

    // ---- status ----
    output logic [LFP_BRAM_AW-1:0] lfp_wr_addr,  // completed-frame write pointer (PS read limit)
    output logic                   lfp_overrun   // either stage overran (sticky)
);

    localparam int CH_W        = $clog2(N_LANES * N_SLOTS);
    localparam int SLOT_W      = (N_SLOTS <= 1) ? 1 : $clog2(N_SLOTS);
    localparam int LANE_W      = (N_LANES <= 1) ? 1 : $clog2(N_LANES);
    localparam int POLY_TAP_W  = $clog2(MAX_POLY_TAPS + 1);
    localparam int HB_TAP_W    = $clog2(HB_TAPS + 1);
    localparam int LFP_WORD_AW = LFP_BRAM_AW - 2;      // 32-bit word address width
    localparam int DECIM_TOTAL = DECIM_1 * DECIM_2;    // /10
    localparam logic [DATA_W-1:0] OFFSET = {1'b1, {(DATA_W-1){1'b0}}};  // 0x8000

    // -----------------------------------------------------------------
    // Control unpack. The lane mask MIRRORS the broadband channel-enable mask
    // (single source of truth), so the LFP filters exactly the broadband-enabled
    // lanes; lfp_cfg[15:8] is kept on the wire but no longer drives the engine.
    // The decimation is structural (/2 then /5), so only the tap count is
    // configurable here.
    // -----------------------------------------------------------------
    wire        lfp_en    = lfp_cfg[0];
    wire [7:0]  lane_mask = dsp_channel_enable;
    wire [7:0]  numtaps8  = lfp_cfg[31:24];
    wire [POLY_TAP_W-1:0] poly_taps =
         (numtaps8 == 0 || numtaps8 > MAX_POLY_TAPS) ? POLY_TAP_W'(MAX_POLY_TAPS)
                                                     : POLY_TAP_W'(numtaps8);

    // -----------------------------------------------------------------
    // Tap conditioning: amplifier-slot gate + offset-binary -> signed.
    // -----------------------------------------------------------------
    wire amp_slot = (dsp_sample_slot >= FIRST_AMP_SLOT) &&
                    (dsp_sample_slot <  FIRST_AMP_SLOT + N_SLOTS);
    wire              eng_valid = dsp_sample_valid & amp_slot;
    wire [SLOT_W-1:0] eng_slot  = SLOT_W'(dsp_sample_slot - FIRST_AMP_SLOT);

    logic [N_LANES*DATA_W-1:0] eng_data;
    genvar gl;
    generate
        for (gl = 0; gl < N_LANES; gl++) begin : g_xin
            // flip the MSB of each lane: offset-binary midpoint -> signed zero
            assign eng_data[gl*DATA_W +: DATA_W] =
                   dsp_sample_data[gl*DATA_W +: DATA_W] ^ OFFSET;
        end
    endgenerate

    // -----------------------------------------------------------------
    // Coefficient upload. One strobe-toggle window as before, plus a stage
    // select so the host can load either filter:
    //   lfp_strobe[0] toggle -> write one coefficient at the auto-incrementing ptr
    //   lfp_strobe[1] clear  -> reset the pointer (do this before each upload)
    //   lfp_strobe[2] stage  -> 0 = stage 1 (halfband), 1 = stage 2 (decimator)
    // -----------------------------------------------------------------
    logic                     coef_tog_d;
    logic [POLY_TAP_W-1:0]    coef_wr_ptr;
    logic                     coef_wr_en;
    logic                     coef_wr_stage;
    logic [POLY_TAP_W-1:0]    coef_wr_addr;
    logic [COEF_W-1:0]        coef_wr_data;
    wire                      coef_tog   = lfp_strobe[0];
    wire                      coef_clr   = lfp_strobe[1];
    wire                      coef_stage = lfp_strobe[2];

    always_ff @(posedge clk) begin
        if (!rstn) begin
            coef_tog_d <= 1'b0; coef_wr_ptr <= '0; coef_wr_en <= 1'b0;
            coef_wr_stage <= 1'b0;
        end else begin
            coef_tog_d <= coef_tog;
            coef_wr_en <= 1'b0;
            if (coef_clr) begin
                coef_wr_ptr <= '0;
            end else if (coef_tog ^ coef_tog_d) begin
                coef_wr_en    <= 1'b1;
                coef_wr_stage <= coef_stage;
                coef_wr_addr  <= coef_wr_ptr;
                coef_wr_data  <= lfp_coef[COEF_W-1:0];
                coef_wr_ptr   <= coef_wr_ptr + 1'b1;
            end
        end
    end

    wire hb_coef_we   = coef_wr_en & ~coef_wr_stage;
    wire poly_coef_we = coef_wr_en &  coef_wr_stage;

    // =================================================================
    // Stage 1: halfband, 30 kHz -> 15 kHz.
    // =================================================================
    logic                    hb_valid, hb_frame_start, hb_busy, hb_overrun;
    logic [CH_W-1:0]         hb_channel;
    logic signed [MID_W-1:0] hb_data;
    logic                    hb_frame_tick;

    lfp_halfband_dec2 #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W),
        .COEF_W(COEF_W), .COEF_FRAC(COEF_FRAC), .OUT_W(MID_W),
        .N_TAPS(HB_TAPS), .RING_DEPTH(HB_RING)
    ) u_stage1 (
        .clk(clk), .rstn(rstn),
        .sample_valid(eng_valid), .sample_data(eng_data),
        .sample_slot(eng_slot), .packet_tick(dsp_packet_tick),
        .lfp_en(lfp_en), .lane_mask(lane_mask),
        .coef_wr_en(hb_coef_we), .coef_wr_addr(coef_wr_addr[HB_TAP_W-1:0]),
        .coef_wr_data(coef_wr_data),
        .out_valid(hb_valid), .out_channel(hb_channel), .out_data(hb_data),
        .out_frame_start(hb_frame_start), .busy(hb_busy),
        .compute_overrun(hb_overrun)
    );

    // Stage 1's pass is finished when busy falls; that is stage 2's frame tick.
    logic hb_busy_d;
    always_ff @(posedge clk) begin
        if (!rstn) hb_busy_d <= 1'b0;
        else       hb_busy_d <= hb_busy;
    end
    assign hb_frame_tick = hb_busy_d & ~hb_busy;

    // =================================================================
    // Stage 2: decimate-by-5, 15 kHz -> 3 kHz.
    // =================================================================
    logic                    out_valid, frame_start, poly_busy, poly_overrun;
    logic [CH_W-1:0]         out_channel;
    logic signed [OUT_W-1:0] out_data;

    lfp_poly_dec5 #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .IN_W(MID_W),
        .COEF_W(COEF_W), .COEF_FRAC(COEF_FRAC), .OUT_W(OUT_W),
        .MAX_TAPS(MAX_POLY_TAPS), .RING_DEPTH(POLY_RING),
        .N_PAR(N_PAR), .DECIM(DECIM_2)
    ) u_stage2 (
        .clk(clk), .rstn(rstn),
        .sample_valid(hb_valid), .sample_channel(hb_channel),
        .sample_data(hb_data), .frame_tick(hb_frame_tick),
        .lfp_en(lfp_en), .lane_mask(lane_mask), .num_taps(poly_taps),
        .coef_wr_en(poly_coef_we), .coef_wr_addr(coef_wr_addr),
        .coef_wr_data(coef_wr_data),
        .out_valid(out_valid), .out_channel(out_channel), .out_data(out_data),
        .frame_start(frame_start), .busy(poly_busy),
        .compute_overrun(poly_overrun)
    );

    assign lfp_overrun = hb_overrun | poly_overrun;

    // -----------------------------------------------------------------
    // Master timestamp for the frame header. dsp_master_timestamp is the LIVE
    // counter and has already advanced past the packet whose samples were just
    // consumed, so the packet the engine just ingested is (timestamp - 1) --
    // exactly what the broadband header stamps for it.
    // -----------------------------------------------------------------
    logic [63:0] ts_ingest, ts_frame;
    logic [31:0] frame_seq;
    logic        ov_frame;

    always_ff @(posedge clk) begin
        if (!rstn)                   ts_ingest <= 64'd0;
        else if (dsp_packet_tick)    ts_ingest <= dsp_master_timestamp - 64'd1;
    end

    // -----------------------------------------------------------------
    // Packet geometry. Samples are laid out per ENABLED lane, so a channel's
    // position depends on how many enabled lanes precede it -- not on its raw
    // channel number.
    // -----------------------------------------------------------------
    logic [3:0] lane_popcount;
    always_comb begin
        lane_popcount = 4'd0;
        for (int b = 0; b < N_LANES; b++)
            lane_popcount = lane_popcount + {3'd0, lane_mask[b]};
    end
    wire [31:0] num_samples_word = {24'd0, lane_popcount} * N_SLOTS;
    wire [LFP_WORD_AW-1:0] sample_words =
         LFP_WORD_AW'((num_samples_word + 32'd1) >> 1);     // 2 samples per word

    wire [31:0] cfg_word = {ov_frame, 7'd0, {1'b0, poly_taps},
                            8'(DECIM_TOTAL), lane_mask};

    // Where this sample belongs in the payload.
    wire [LANE_W-1:0] out_lane = out_channel[CH_W-1 -: LANE_W];
    wire [SLOT_W-1:0] out_slot = out_channel[SLOT_W-1:0];
    logic [3:0] lanes_before;
    always_comb begin
        lanes_before = 4'd0;
        for (int b = 0; b < N_LANES; b++)
            if (LANE_W'(b) < out_lane && lane_mask[b])
                lanes_before = lanes_before + 4'd1;
    end
    wire [CH_W:0] out_rank = {4'd0, lanes_before} * N_SLOTS + out_slot;

    // -----------------------------------------------------------------
    // Packet builder. On frame_start the header is laid down one word per clock
    // at the frame base; samples then land at their own addresses as they are
    // produced. The frame is published to the PS only once every sample has
    // been written, so the PS never DMAs a partial frame.
    // -----------------------------------------------------------------
    logic [LFP_WORD_AW-1:0] frame_base;    // first word of the in-flight frame
    logic [LFP_WORD_AW-1:0] wr_word;       // next free word (after the last frame)
    logic [3:0]             hdr_idx;
    logic                   hdr_busy;
    logic [CH_W:0]          samp_count;    // samples written this frame
    logic [31:0]            bram_din_r;
    logic [LFP_WORD_AW-1:0] bram_word_r;
    logic [3:0]             bram_we_r;

    wire [OUT_W-1:0] out_offset = out_data ^ OFFSET;   // signed -> offset binary

    always_ff @(posedge clk) begin
        if (!rstn) begin
            frame_base <= '0; wr_word <= '0; hdr_idx <= 4'd0; hdr_busy <= 1'b0;
            samp_count <= '0; bram_we_r <= 4'h0; lfp_wr_addr <= '0;
            ts_frame <= 64'd0; ov_frame <= 1'b0; frame_seq <= 32'd0;
        end else begin
            bram_we_r <= 4'h0;

            if (frame_start) begin
                // Snapshot the header fields and start laying the header down at
                // the current free position.
                ts_frame   <= ts_ingest;
                ov_frame   <= lfp_overrun;
                frame_base <= wr_word;
                hdr_idx    <= 4'd0;
                hdr_busy   <= 1'b1;
                samp_count <= '0;
            end else if (hdr_busy) begin
                // One header word per clock, from the shared contract.
                bram_word_r <= frame_base + LFP_WORD_AW'(hdr_idx);
                bram_din_r  <= unified_hdr_word(hdr_idx[2:0], STREAM_TYPE_LFP,
                                                ts_frame, frame_seq,
                                                cfg_word, num_samples_word);
                bram_we_r   <= 4'hF;
                if (hdr_idx == 4'd7) begin
                    hdr_busy  <= 1'b0;
                    frame_seq <= frame_seq + 1'b1;   // one SEQ per emitted frame
                end else begin
                    hdr_idx <= hdr_idx + 1'b1;
                end
            end

            if (out_valid) begin
                // Place the sample by its rank; the byte enables pick the half of
                // the shared 32-bit word, so arrival order does not matter.
                bram_word_r <= frame_base + LFP_WORD_AW'(UNIFIED_HDR_WORDS)
                                          + LFP_WORD_AW'(out_rank >> 1);
                bram_din_r  <= {out_offset, out_offset};   // both halves; we mask
                bram_we_r   <= out_rank[0] ? 4'b1100 : 4'b0011;

                // Publish the frame once its last sample has landed.
                if (samp_count + 1'b1 >= num_samples_word[CH_W:0]) begin
                    wr_word     <= frame_base + LFP_WORD_AW'(UNIFIED_HDR_WORDS)
                                              + sample_words;
                    lfp_wr_addr <= {frame_base + LFP_WORD_AW'(UNIFIED_HDR_WORDS)
                                               + sample_words, 2'b00};
                    samp_count  <= '0;
                end else begin
                    samp_count <= samp_count + 1'b1;
                end
            end
        end
    end

    assign bram_clk  = clk;
    assign bram_rst  = ~rstn;
    assign bram_en   = 1'b1;
    assign bram_we   = bram_we_r;
    assign bram_addr = {bram_word_r, 2'b00};    // word -> byte address
    assign bram_din  = bram_din_r;

endmodule
