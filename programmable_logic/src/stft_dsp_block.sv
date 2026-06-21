// =====================================================================
// stft_dsp_block.sv
//
// Integration wrapper around stft_engine -- the Tier-2 sliding-window STFT.
// Mirrors lfp_dsp_block: it sits between the Tier-1 LFP output stream and the
// PS-readable STFT results BRAM, and decodes the host upload window. Unlike the
// LFP block it taps the LFP stream as a *consumer* (the LFP engine already
// produced signed, decimated samples), so no offset-binary conversion here.
//
// Control-register slice (already CDC'd to clk by axi_lite_registers):
//   stft_cfg    [0] enable, [7:4] nfft_log2 (6 -> N=64), [31:16] hop (>=1)
//   stft_data   upload payload: window coeff (signed Q15 [15:0]) OR
//               selector channel index ([CH_W-1:0]); the lane/window index is
//               the auto-incrementing pointer.
//   stft_strobe [0] write toggle (1 write/edge, ptr++), [1] ptr clear (hold 0),
//               [2] target: 0 = window RAM, 1 = channel-selector table.
//
// The host clears the pointer, then streams the window (MAX_N coeffs) or the
// selector (K channels), exactly like the LFP coefficient upload. See
// docs/tier2-stft-design.md.
// =====================================================================

module stft_dsp_block #(
    parameter int N_CH   = 256,
    parameter int K      = 32,
    parameter int MAX_N  = 256,
    parameter int DATA_W = 16,
    parameter int WIN_W  = 16,
    parameter int RES_AW = 14,
    // derived (do not override)
    localparam int CH_W   = (N_CH <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W = (K    <= 1) ? 1 : $clog2(K),
    localparam int NMAX_W = $clog2(MAX_N)
) (
    input  logic                     clk,
    input  logic                     rstn,

    // ---- Tier-1 LFP output stream tap (signed, from lfp_dsp_block) ----
    input  logic                     lfp_out_valid,
    input  logic [CH_W-1:0]          lfp_out_channel,
    input  logic signed [DATA_W-1:0] lfp_out_data,
    input  logic                     lfp_frame_start,

    // ---- STFT control register slice (CDC'd) ----
    input  logic [31:0]              stft_cfg,
    input  logic [31:0]              stft_data,
    input  logic [31:0]              stft_strobe,

    // ---- STFT results BRAM port (PL writes; PS reads via axi_bram_ctrl) ----
    output logic                     bram_clk,
    output logic                     bram_rst,
    output logic [RES_AW-1:0]        bram_addr,
    output logic [31:0]              bram_din,
    input  logic [31:0]              bram_dout,    // unused (write-only side)
    output logic                     bram_en,
    output logic [3:0]               bram_we,

    // ---- status ----
    output logic [31:0]              stft_frame_seq,  // completed STFT passes
    output logic                     stft_busy,
    output logic                     stft_overflow
);

    // -----------------------------------------------------------------
    // Control unpack.
    // -----------------------------------------------------------------
    wire        en        = stft_cfg[0];
    wire [3:0]  nfft_log2 = stft_cfg[7:4];
    wire [15:0] hop       = stft_cfg[31:16];

    wire        ld_tog = stft_strobe[0];
    wire        ld_clr = stft_strobe[1];
    wire        ld_tgt = stft_strobe[2];           // 0 = window, 1 = selector

    // -----------------------------------------------------------------
    // Upload window (aux-style strobe toggle -> 1 write/edge, ptr auto-inc).
    // Shared pointer; the host clears it before each window/selector stream.
    // -----------------------------------------------------------------
    logic              ld_tog_d;
    logic [NMAX_W-1:0] ld_ptr;
    logic              win_wr_en;  logic [NMAX_W-1:0] win_wr_addr;  logic signed [WIN_W-1:0] win_wr_data;
    logic              sel_wr_en;  logic [LANE_W-1:0] sel_wr_lane;  logic [CH_W-1:0] sel_wr_ch;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            ld_tog_d  <= 1'b0;
            ld_ptr    <= '0;
            win_wr_en <= 1'b0;
            sel_wr_en <= 1'b0;
        end else begin
            ld_tog_d  <= ld_tog;
            win_wr_en <= 1'b0;
            sel_wr_en <= 1'b0;
            if (ld_clr) begin
                ld_ptr <= '0;
            end else if (ld_tog ^ ld_tog_d) begin
                if (ld_tgt) begin
                    sel_wr_en   <= 1'b1;
                    sel_wr_lane <= ld_ptr[LANE_W-1:0];
                    sel_wr_ch   <= stft_data[CH_W-1:0];
                end else begin
                    win_wr_en   <= 1'b1;
                    win_wr_addr <= ld_ptr;
                    win_wr_data <= stft_data[WIN_W-1:0];
                end
                ld_ptr <= ld_ptr + 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // The STFT engine (selector + sliding buffer + window + FFT + capture).
    // -----------------------------------------------------------------
    stft_engine #(
        .N_CH(N_CH), .K(K), .MAX_N(MAX_N), .DATA_W(DATA_W), .WIN_W(WIN_W), .RES_AW(RES_AW)
    ) u_eng (
        .clk(clk), .rstn(rstn),
        .lfp_out_valid(lfp_out_valid), .lfp_out_channel(lfp_out_channel),
        .lfp_out_data(lfp_out_data), .lfp_frame_start(lfp_frame_start),
        .stft_en(en), .nfft_log2(nfft_log2), .hop(hop),
        .sel_wr_en(sel_wr_en), .sel_wr_lane(sel_wr_lane), .sel_wr_ch(sel_wr_ch),
        .win_wr_en(win_wr_en), .win_wr_addr(win_wr_addr), .win_wr_data(win_wr_data),
        .res_bram_clk(bram_clk), .res_bram_rst(bram_rst), .res_bram_addr(bram_addr),
        .res_bram_din(bram_din), .res_bram_dout(bram_dout), .res_bram_en(bram_en), .res_bram_we(bram_we),
        .frame_seq(stft_frame_seq), .busy(stft_busy), .overflow(stft_overflow)
    );

endmodule
