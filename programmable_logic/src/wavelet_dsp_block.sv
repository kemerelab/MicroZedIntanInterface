// =====================================================================
// wavelet_dsp_block.sv
//
// Integration wrapper around wavelet_cqt_engine -- the Tier-3 multirate
// wavelet scalogram. Mirrors stft_dsp_block / lfp_dsp_block: it taps the
// Tier-1 LFP output stream (already signed, decimated) as a CONSUMER (no
// offset-binary conversion), decodes the host upload window (aux-style
// strobe/toggle CDC with a target selector), and presents the engine's
// results BRAM port to a PS-readable axi_bram_ctrl.
//
// Control-register slice (already CDC'd to clk by axi_lite_registers):
//   wav_cfg    [0]      enable
//              [7:4]    n_octaves (1..N_OCTAVES)
//              [11:8]   n_voices  (1..V)
//              [19:12]  n_taps    (1..N_TAPS)
//   wav_gain   4 bits per octave: gain_cfg[4*o +: 4] = left-shift for octave o
//   wav_data   upload payload (target-dependent, see below)
//   wav_strobe [0]      write toggle (1 write/edge, ptr++)
//              [1]      ptr clear (hold 0)
//              [3:2]    target: 0 = voice coef RAM (re,im interleaved by ptr),
//                               1 = halfband coef RAM,
//                               2 = channel-selector table
//
// Upload protocol (host clears ptr, then streams), per target:
//   voice coef : ptr indexes the interleaved {re,im} coef RAM directly --
//                stream 2*V*N_TAPS signed Q1.17 words (re,im,re,im,...).
//   halfband   : ptr indexes hb taps -- stream HB_TAPS signed Q1.17 words.
//   selector   : ptr indexes lane 0..K-1 -- stream K channel indices.
// =====================================================================
module wavelet_dsp_block #(
    parameter int N_CH      = 256,
    parameter int K         = 96,   // v2 step2 real-time-clean ceiling (2 MAC + work-spread)
    parameter int N_OCTAVES = 8,
    parameter int V         = 4,
    parameter int N_TAPS    = 24,
    parameter int HB_TAPS   = 7,
    parameter int DATA_W    = 16,
    parameter int COEF_W    = 18,
    parameter int COEF_FRAC = 17,
    parameter int OUT_W     = 18,
    parameter int RING_DEPTH = 64,
    parameter int RES_AW    = 14,
    // derived
    localparam int CH_W   = (N_CH <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W = (K    <= 1) ? 1 : $clog2(K),
    localparam int OCT_W  = (N_OCTAVES <= 1) ? 1 : $clog2(N_OCTAVES),
    localparam int VOICE_W= (V    <= 1) ? 1 : $clog2(V),
    localparam int TAP_W  = (N_TAPS<= 1) ? 1 : $clog2(N_TAPS),
    localparam int HBTAP_W= (HB_TAPS<=1) ? 1 : $clog2(HB_TAPS),
    localparam int COEFN  = V * N_TAPS,
    localparam int COEF_AW= $clog2(2*COEFN),
    // upload pointer must address the largest target (the interleaved coef RAM)
    localparam int PTR_W  = COEF_AW
) (
    input  logic clk,
    input  logic rstn,

    // ---- Tier-1 LFP output stream tap (signed, from lfp_dsp_block) ----
    input  logic                 lfp_out_valid,
    input  logic [CH_W-1:0]      lfp_out_channel,
    input  logic signed [DATA_W-1:0] lfp_out_data,
    input  logic                 lfp_frame_start,

    // ---- wavelet control register slice (CDC'd) ----
    input  logic [31:0]          wav_cfg,
    input  logic [31:0]          wav_gain,
    input  logic [31:0]          wav_data,
    input  logic [31:0]          wav_strobe,

    // ---- results BRAM port (PL writes; PS reads via axi_bram_ctrl) ----
    output logic                 bram_clk,
    output logic                 bram_rst,
    output logic [RES_AW-1:0]    bram_addr,
    output logic [31:0]          bram_din,
    input  logic [31:0]          bram_dout,    // unused
    output logic                 bram_en,
    output logic [3:0]           bram_we,

    // ---- status ----
    output logic [31:0]          wav_frame_seq,
    output logic                 wav_busy,
    output logic                 wav_overrun
);
    // -----------------------------------------------------------------
    // Control unpack.
    // -----------------------------------------------------------------
    wire        en        = wav_cfg[0];
    wire [3:0]  n_oct4    = wav_cfg[7:4];
    wire [3:0]  n_voc4    = wav_cfg[11:8];
    wire [7:0]  n_tap8    = wav_cfg[19:12];

    wire [OCT_W:0]   n_octaves_cfg = (n_oct4 == 0) ? (OCT_W+1)'(N_OCTAVES) : (OCT_W+1)'(n_oct4);
    wire [VOICE_W:0] n_voices_cfg  = (n_voc4 == 0) ? (VOICE_W+1)'(V)       : (VOICE_W+1)'(n_voc4);
    wire [TAP_W:0]   n_taps_cfg    = (n_tap8 == 0) ? (TAP_W+1)'(N_TAPS)    : (TAP_W+1)'(n_tap8);

    wire        ld_tog = wav_strobe[0];
    wire        ld_clr = wav_strobe[1];
    wire [1:0]  ld_tgt = wav_strobe[3:2];   // 0=voice coef, 1=halfband, 2=selector

    // -----------------------------------------------------------------
    // Upload window (aux-style strobe toggle -> 1 write/edge, ptr auto-inc).
    // -----------------------------------------------------------------
    logic           ld_tog_d;
    logic [PTR_W-1:0] ld_ptr;
    logic           coef_wr_en;  logic [COEF_AW-1:0] coef_wr_addr; logic signed [COEF_W-1:0] coef_wr_data;
    logic           hb_wr_en;    logic [HBTAP_W-1:0] hb_wr_addr;   logic signed [COEF_W-1:0] hb_wr_data;
    logic           sel_wr_en;   logic [LANE_W-1:0]  sel_wr_lane;  logic [CH_W-1:0] sel_wr_ch;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            ld_tog_d <= 1'b0; ld_ptr <= '0;
            coef_wr_en <= 1'b0; hb_wr_en <= 1'b0; sel_wr_en <= 1'b0;
        end else begin
            ld_tog_d   <= ld_tog;
            coef_wr_en <= 1'b0; hb_wr_en <= 1'b0; sel_wr_en <= 1'b0;
            if (ld_clr) begin
                ld_ptr <= '0;
            end else if (ld_tog ^ ld_tog_d) begin
                case (ld_tgt)
                    2'd1: begin
                        hb_wr_en   <= 1'b1;
                        hb_wr_addr <= ld_ptr[HBTAP_W-1:0];
                        hb_wr_data <= wav_data[COEF_W-1:0];
                    end
                    2'd2: begin
                        sel_wr_en   <= 1'b1;
                        sel_wr_lane <= ld_ptr[LANE_W-1:0];
                        sel_wr_ch   <= wav_data[CH_W-1:0];
                    end
                    default: begin // 0 = voice coef RAM (interleaved re,im)
                        coef_wr_en   <= 1'b1;
                        coef_wr_addr <= ld_ptr[COEF_AW-1:0];
                        coef_wr_data <= wav_data[COEF_W-1:0];
                    end
                endcase
                ld_ptr <= ld_ptr + 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // The wavelet CQT engine.
    // -----------------------------------------------------------------
    wavelet_cqt_engine #(
        .N_CH(N_CH), .K(K), .N_OCTAVES(N_OCTAVES), .V(V), .N_TAPS(N_TAPS),
        .HB_TAPS(HB_TAPS), .DATA_W(DATA_W), .COEF_W(COEF_W), .COEF_FRAC(COEF_FRAC),
        .OUT_W(OUT_W), .RING_DEPTH(RING_DEPTH), .RES_AW(RES_AW)
    ) u_eng (
        .clk(clk), .rstn(rstn),
        .lfp_out_valid(lfp_out_valid), .lfp_out_channel(lfp_out_channel),
        .lfp_out_data(lfp_out_data), .lfp_frame_start(lfp_frame_start),
        .wav_en(en), .n_octaves_cfg(n_octaves_cfg), .n_voices_cfg(n_voices_cfg),
        .n_taps_cfg(n_taps_cfg), .gain_cfg(wav_gain[4*N_OCTAVES-1:0]),
        .sel_wr_en(sel_wr_en), .sel_wr_lane(sel_wr_lane), .sel_wr_ch(sel_wr_ch),
        .coef_wr_en(coef_wr_en), .coef_wr_addr(coef_wr_addr), .coef_wr_data(coef_wr_data),
        .hb_wr_en(hb_wr_en), .hb_wr_addr(hb_wr_addr), .hb_wr_data(hb_wr_data),
        .res_bram_clk(bram_clk), .res_bram_rst(bram_rst), .res_bram_addr(bram_addr),
        .res_bram_din(bram_din), .res_bram_dout(bram_dout), .res_bram_en(bram_en), .res_bram_we(bram_we),
        .frame_seq(wav_frame_seq), .busy(wav_busy), .overrun(wav_overrun)
    );

endmodule
