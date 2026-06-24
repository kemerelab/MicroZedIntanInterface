// =====================================================================
// lfp_dsp_block.sv
//
// Integration wrapper around lfp_fir_decimator. Sits between the acquisition
// core's DSP tap and the PS-readable LFP output BRAM. Responsibilities:
//   * gate the tap to the 32 amplifier slots and remove the +2 SPI readback
//     offset (cycle_counter 2..33 -> engine slot 0..31),
//   * convert Intan offset-binary -> two's-complement signed on the way IN
//     and signed -> offset-binary on the way OUT (symmetric ^0x8000),
//   * decode the host coefficient-upload window (aux-style strobe CDC),
//   * pack the decimated outputs 2x16-bit per 32-bit word and write them to a
//     ring in the LFP output BRAM (PS reads via a 2nd axi_bram_ctrl).
//
// Control-register slice (already CDC'd to clk by axi_lite_registers):
//   lfp_cfg    [0] lfp_en, [15:8] lane_mask, [23:16] decim_R, [31:24] num_taps
//   lfp_coef   [17:0] coefficient data (signed Q1.17)
//   lfp_strobe [0] coef write toggle (1 write per edge, ptr auto-increments),
//              [1] coef pointer clear (hold ptr at 0 while high)
//
// See docs/lfp-dsp-engine-design.md.
// =====================================================================

module lfp_dsp_block #(
    parameter int N_LANES        = 8,
    parameter int N_SLOTS        = 32,      // amplifier channels per lane
    parameter int DATA_W         = 16,
    parameter int COEF_W         = 18,
    parameter int COEF_FRAC      = 17,
    parameter int RING_DEPTH     = 256,     // FIR delay-line depth (USE_CIC=0 path)
    parameter int OUT_W          = 16,
    parameter int FIRST_AMP_SLOT = 2,       // cycle_counter of amplifier channel 0
    parameter int LFP_BRAM_AW    = 14,      // LFP output BRAM byte-address width (16 KB)
    // ---- datapath select ----
    // USE_CIC=1 (default): CIC^4(/5) -> comp-FIR halfband(/2) = /10 LFP @ 3 kHz.
    //   ~5x less delay-line BRAM than the single-stage FIR; the coef window loads
    //   the HB_TAPS comp-FIR taps; the engine's decimation is hardwired /10.
    // USE_CIC=0: the dual-MAC single-stage FIR (fallback; coef window loads the
    //   full FIR, decim_R/num_taps from lfp_cfg).
    parameter int USE_CIC        = 1,
    parameter int CIC_R          = 5,
    parameter int CIC_ORDER      = 4,
    parameter int CIC_ACC_W      = 32,
    parameter int CIC_GAIN_SHIFT = 10,
    parameter int HB_RING        = 64       // halfband delay-line depth (USE_CIC=1)
) (
    input  logic         clk,
    input  logic         rstn,

    // ---- DSP tap from data_generator_core ----
    input  logic         dsp_sample_valid,
    input  logic [N_LANES*DATA_W-1:0] dsp_sample_data,
    input  logic [5:0]   dsp_sample_slot,    // cycle_counter, 0..34
    input  logic         dsp_packet_tick,

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
    output logic [LFP_BRAM_AW-1:0] lfp_wr_addr,  // current write byte address (PS read ptr)
    output logic                   lfp_overrun   // engine compute overrun (sticky)
);

    localparam int RING_AW = $clog2(RING_DEPTH);
    localparam int TAPN_W  = $clog2(RING_DEPTH + 1);
    localparam int CH_W    = $clog2(N_LANES * N_SLOTS);
    localparam int SLOT_W  = (N_SLOTS <= 1) ? 1 : $clog2(N_SLOTS);
    localparam int LFP_WORD_AW = LFP_BRAM_AW - 2;          // 32-bit word address width
    localparam logic [DATA_W-1:0] OFFSET = {1'b1, {(DATA_W-1){1'b0}}};  // 0x8000

    // -----------------------------------------------------------------
    // Control unpack (with min-1 guards on the rate/length).
    // -----------------------------------------------------------------
    wire        lfp_en    = lfp_cfg[0];
    wire [7:0]  lane_mask = lfp_cfg[15:8];
    wire [7:0]  decim_r8  = lfp_cfg[23:16];
    wire [7:0]  numtaps8  = lfp_cfg[31:24];
    wire [7:0]  decim_R   = (decim_r8 == 0) ? 8'd1 : decim_r8;
    wire [TAPN_W-1:0] num_taps = (numtaps8 == 0) ? TAPN_W'(1) : TAPN_W'(numtaps8);

    // -----------------------------------------------------------------
    // Tap conditioning: amplifier-slot gate + offset-binary -> signed.
    // -----------------------------------------------------------------
    wire amp_slot = (dsp_sample_slot >= FIRST_AMP_SLOT) &&
                    (dsp_sample_slot <  FIRST_AMP_SLOT + N_SLOTS);
    wire                     eng_valid = dsp_sample_valid & amp_slot;
    wire [SLOT_W-1:0]        eng_slot  = SLOT_W'(dsp_sample_slot - FIRST_AMP_SLOT);

    logic [N_LANES*DATA_W-1:0] eng_data;
    genvar gl;
    generate
        for (gl = 0; gl < N_LANES; gl++) begin : g_xin
            // flip the MSB of each lane: offset-binary midpoint -> signed 0
            assign eng_data[gl*DATA_W +: DATA_W] =
                   dsp_sample_data[gl*DATA_W +: DATA_W] ^ OFFSET;
        end
    endgenerate

    // -----------------------------------------------------------------
    // Coefficient upload window (aux-style strobe toggle -> 1 write/edge).
    // -----------------------------------------------------------------
    logic              coef_tog_d;
    logic [RING_AW-1:0] coef_wr_ptr;
    logic              coef_wr_en;
    logic [RING_AW-1:0] coef_wr_addr;
    logic [COEF_W-1:0] coef_wr_data;
    wire               coef_tog = lfp_strobe[0];
    wire               coef_clr = lfp_strobe[1];

    always_ff @(posedge clk) begin
        if (!rstn) begin
            coef_tog_d  <= 1'b0;
            coef_wr_ptr <= '0;
            coef_wr_en  <= 1'b0;
        end else begin
            coef_tog_d <= coef_tog;
            coef_wr_en <= 1'b0;
            if (coef_clr) begin
                coef_wr_ptr <= '0;
            end else if (coef_tog ^ coef_tog_d) begin
                coef_wr_en   <= 1'b1;
                coef_wr_addr <= coef_wr_ptr;
                coef_wr_data <= lfp_coef[COEF_W-1:0];
                coef_wr_ptr  <= coef_wr_ptr + 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // The decimating engine: CIC(/5)->halfband(/2) (default) or single FIR.
    // -----------------------------------------------------------------
    localparam int HB_TAPN_W = $clog2(HB_RING + 1);
    localparam int HB_RING_AW = $clog2(HB_RING);
    logic               out_valid, out_frame_start, busy;
    logic [CH_W-1:0]    out_channel;
    logic [OUT_W-1:0]   out_data;

    generate
    if (USE_CIC) begin : g_cic_chain
        // ---- CIC^4 /5 ----
        logic               cic_valid, cic_fs, cic_busy, cic_ov;
        logic [CH_W-1:0]    cic_ch;
        logic [OUT_W-1:0]   cic_d;
        cic_decimator #(
            .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W), .R(CIC_R),
            .N_ORDER(CIC_ORDER), .ACC_W(CIC_ACC_W), .OUT_W(OUT_W),
            .GAIN_SHIFT(CIC_GAIN_SHIFT)
        ) u_cic (
            .clk(clk), .rstn(rstn),
            .sample_valid(eng_valid), .sample_data(eng_data),
            .sample_slot(eng_slot), .packet_tick(dsp_packet_tick),
            .en(lfp_en), .lane_mask(lane_mask),
            .out_valid(cic_valid), .out_channel(cic_ch), .out_data(cic_d),
            .out_frame_start(cic_fs), .busy(cic_busy), .compute_overrun(cic_ov)
        );
        // ---- glue: CIC frame -> per-slot 8-lane stream ----
        logic                      hb_v, hb_t;
        logic [N_LANES*DATA_W-1:0] hb_d;
        logic [SLOT_W-1:0]         hb_s;
        cic_to_halfband #(
            .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W)
        ) u_glue (
            .clk(clk), .rstn(rstn), .lane_mask(lane_mask),
            .cic_valid(cic_valid), .cic_channel(cic_ch), .cic_data(cic_d),
            .cic_frame_start(cic_fs),
            .hb_valid(hb_v), .hb_data(hb_d), .hb_slot(hb_s), .hb_tick(hb_t)
        );
        // ---- comp-FIR halfband /2 (loads the coef window) ----
        logic hb_ov;
        lfp_halfband #(
            .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W), .COEF_W(COEF_W),
            .COEF_FRAC(COEF_FRAC), .RING_DEPTH(HB_RING), .OUT_W(OUT_W)
        ) u_hb (
            .clk(clk), .rstn(rstn),
            .sample_valid(hb_v), .sample_data(hb_d), .sample_slot(hb_s),
            .packet_tick(hb_t), .en(lfp_en), .lane_mask(lane_mask),
            .num_taps(num_taps[HB_TAPN_W-1:0]),
            .coef_wr_en(coef_wr_en), .coef_wr_addr(coef_wr_addr[HB_RING_AW-1:0]),
            .coef_wr_data(coef_wr_data),
            .out_valid(out_valid), .out_channel(out_channel), .out_data(out_data),
            .out_frame_start(out_frame_start), .busy(busy), .compute_overrun(hb_ov)
        );
        assign lfp_overrun = cic_ov | hb_ov;
    end else begin : g_fir
        lfp_fir_decimator #(
            .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W), .COEF_W(COEF_W),
            .COEF_FRAC(COEF_FRAC), .RING_DEPTH(RING_DEPTH), .OUT_W(OUT_W)
        ) u_fir (
            .clk(clk), .rstn(rstn),
            .sample_valid(eng_valid), .sample_data(eng_data),
            .sample_slot(eng_slot), .packet_tick(dsp_packet_tick),
            .lfp_en(lfp_en), .lane_mask(lane_mask), .decim_R(decim_R), .num_taps(num_taps),
            .coef_wr_en(coef_wr_en), .coef_wr_addr(coef_wr_addr), .coef_wr_data(coef_wr_data),
            .out_valid(out_valid), .out_channel(out_channel), .out_data(out_data),
            .out_frame_start(out_frame_start), .busy(busy), .compute_overrun(lfp_overrun)
        );
    end
    endgenerate

    // -----------------------------------------------------------------
    // Output: signed -> offset-binary, pack 2x16-bit per 32-bit word, write
    // to the LFP output BRAM ring. The PS reads up to lfp_wr_addr.
    // -----------------------------------------------------------------
    wire [DATA_W-1:0] out_offset = out_data ^ OFFSET;

    logic                  pack_phase;     // 0 = expecting low half, 1 = high half
    logic [DATA_W-1:0]     pack_low;
    logic [LFP_WORD_AW-1:0] wr_word;
    logic                  bram_we_r;
    logic [31:0]           bram_din_r;
    logic [LFP_WORD_AW-1:0] bram_word_r;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            pack_phase <= 1'b0;
            pack_low   <= '0;
            wr_word    <= '0;
            bram_we_r  <= 1'b0;
        end else begin
            bram_we_r <= 1'b0;
            // Keep frames word-aligned: a fresh frame starts on the low half.
            if (out_valid && out_frame_start)
                pack_phase <= 1'b0;

            if (out_valid) begin
                if (pack_phase == 1'b0) begin
                    pack_low   <= out_offset;
                    pack_phase <= 1'b1;
                end else begin
                    bram_din_r  <= {out_offset, pack_low};   // {high, low}
                    bram_word_r <= wr_word;
                    bram_we_r   <= 1'b1;
                    wr_word     <= wr_word + 1'b1;            // wraps naturally
                    pack_phase  <= 1'b0;
                end
            end
        end
    end

    assign bram_clk    = clk;
    assign bram_rst    = ~rstn;
    assign bram_en     = 1'b1;
    assign bram_we     = bram_we_r ? 4'hF : 4'h0;
    assign bram_addr   = {bram_word_r, 2'b00};               // word -> byte address
    assign bram_din    = bram_din_r;
    assign lfp_wr_addr = {wr_word, 2'b00};

endmodule
