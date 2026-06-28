`timescale 1ns/1ps
// =====================================================================
// wav_spread_tb.sv -- REAL-TIME work-spread overrun test for the STEP-2
// wavelet_cqt_engine. Unlike wav_overrun_tb (which measures one ISOLATED
// worst-case pass), this drives K lanes/frame at the REAL 3 kHz @ 84 MHz
// spacing (FRAME_CLKS = 28000) for many frames so fcount sweeps through 0
// (all octaves coincide) repeatedly, exercising the deferred per-octave
// drain across frame windows.
//
// PASS = the engine's `overrun` flag NEVER asserts over the whole run AND
// frame_seq advances exactly once per frame (no dropped columns). That is
// the real-time-clean condition for the work-spread engine (busy may stay
// continuously high in steady state, so a busy-duration metric is NOT used).
//
// Configurable via plusargs: +FRAMECLKS= +NFRAMES=
// Sweep K via the `parameter int K` below.
// =====================================================================
module wav_spread_tb;
    parameter int N_CH      = 256;
    parameter int K         = 40;     // this branch's build cap (one octave packet
                                      // fits ONE datagram: K*V=160 <= 180). The v2
                                      // engine's compute ceiling is K~176; K=40 is
                                      // far below it, so this is real-time-clean.
    parameter int N_OCTAVES = 8;
    parameter int V         = 4;
    parameter int N_TAPS    = 24;
    parameter int HB_TAPS   = 7;
    parameter int DATA_W    = 16;
    parameter int COEF_W    = 18;
    parameter int COEF_FRAC = 17;
    parameter int OUT_W     = 18;
    parameter int RING_DEPTH= 64;
    parameter int RES_AW    = 17;

    int ACT_OCT  = N_OCTAVES;
    int ACT_VOC  = V;
    int ACT_TAP  = N_TAPS;
    int FRAME_CLKS = 28000;
    int N_FRAMES   = 800;     // > 512 so fcount sweeps through 0 twice

    localparam int CH_W   = $clog2(N_CH);
    localparam int LANE_W = $clog2(K);
    localparam int OCT_W  = $clog2(N_OCTAVES);
    localparam int VOICE_W= $clog2(V);
    localparam int TAP_W  = $clog2(N_TAPS);
    localparam int HBTAP_W= $clog2(HB_TAPS);
    localparam int COEFN  = V*N_TAPS;
    localparam int COEF_AW= $clog2(2*COEFN);

    logic clk=0, rstn=0;
    always #5 clk = ~clk;

    logic                 lfp_out_valid=0, lfp_frame_start=0;
    logic [CH_W-1:0]      lfp_out_channel=0;
    logic signed [DATA_W-1:0] lfp_out_data=0;
    logic [63:0]          master_ts=0;
    logic [LANE_W:0]      n_channels_cfg = K;
    logic                 wav_en=0;
    logic [OCT_W:0]       n_octaves_cfg;
    logic [VOICE_W:0]     n_voices_cfg;
    logic [TAP_W:0]       n_taps_cfg;
    logic [4*N_OCTAVES-1:0] gain_cfg = '0;
    logic                 sel_wr_en=0;   logic [LANE_W-1:0] sel_wr_lane=0; logic [CH_W-1:0] sel_wr_ch=0;
    logic                 coef_wr_en=0;  logic [COEF_AW-1:0] coef_wr_addr=0; logic signed [COEF_W-1:0] coef_wr_data=0;
    logic                 hb_wr_en=0;    logic [HBTAP_W-1:0] hb_wr_addr=0;   logic signed [COEF_W-1:0] hb_wr_data=0;
    logic                 res_clk, res_rst, res_en; logic [3:0] res_we;
    logic [RES_AW-1:0]    res_addr; logic [31:0] res_din; logic [31:0] res_dout=0;
    logic [31:0]          frame_seq; logic busy, overrun;

    wavelet_cqt_engine #(
        .N_CH(N_CH), .K(K), .N_OCTAVES(N_OCTAVES), .V(V), .N_TAPS(N_TAPS),
        .HB_TAPS(HB_TAPS), .DATA_W(DATA_W), .COEF_W(COEF_W), .COEF_FRAC(COEF_FRAC),
        .OUT_W(OUT_W), .RING_DEPTH(RING_DEPTH), .RES_AW(RES_AW)
    ) dut (
        .clk(clk), .rstn(rstn),
        .lfp_out_valid(lfp_out_valid), .lfp_out_channel(lfp_out_channel),
        .lfp_out_data(lfp_out_data), .lfp_frame_start(lfp_frame_start),
        .master_timestamp(master_ts), .n_channels_cfg(n_channels_cfg),
        .wav_en(wav_en), .n_octaves_cfg(n_octaves_cfg), .n_voices_cfg(n_voices_cfg),
        .n_taps_cfg(n_taps_cfg), .gain_cfg(gain_cfg),
        .sel_wr_en(sel_wr_en), .sel_wr_lane(sel_wr_lane), .sel_wr_ch(sel_wr_ch),
        .coef_wr_en(coef_wr_en), .coef_wr_addr(coef_wr_addr), .coef_wr_data(coef_wr_data),
        .hb_wr_en(hb_wr_en), .hb_wr_addr(hb_wr_addr), .hb_wr_data(hb_wr_data),
        .res_bram_clk(res_clk), .res_bram_rst(res_rst), .res_bram_addr(res_addr),
        .res_bram_din(res_din), .res_bram_dout(res_dout), .res_bram_en(res_en), .res_bram_we(res_we),
        .frame_seq(frame_seq), .busy(busy), .overrun(overrun)
    );

    // busy duty-cycle tracker (informational): fraction of time busy is high
    int unsigned busy_cyc, total_cyc;
    always @(posedge clk) begin
        total_cyc <= total_cyc + 1;
        if (busy) busy_cyc <= busy_cyc + 1;
    end

    task automatic neg; @(negedge clk); endtask

    int i, fr, l;
    initial begin
        if (!$value$plusargs("FRAMECLKS=%d", FRAME_CLKS)) FRAME_CLKS = 28000;
        if (!$value$plusargs("NFRAMES=%d",   N_FRAMES))   N_FRAMES   = 800;
        n_octaves_cfg = ACT_OCT; n_voices_cfg = ACT_VOC; n_taps_cfg = ACT_TAP;
        busy_cyc=0; total_cyc=0;

        rstn=0; repeat(5) neg(); rstn=1; neg();

        for (l=0;l<K;l++) begin neg(); sel_wr_en=1; sel_wr_lane=l[LANE_W-1:0]; sel_wr_ch=l[CH_W-1:0]; end
        neg(); sel_wr_en=0;

        for (i=0;i<2*COEFN;i++) begin neg(); coef_wr_en=1; coef_wr_addr=i[COEF_AW-1:0]; coef_wr_data=(i*7+1); end
        neg(); coef_wr_en=0;
        for (i=0;i<HB_TAPS;i++) begin neg(); hb_wr_en=1; hb_wr_addr=i[HBTAP_W-1:0]; hb_wr_data=(i*3+5); end
        neg(); hb_wr_en=0;

        neg(); wav_en=1;

        // Drive N_FRAMES real frames + 1 trailing trigger frame. The pass for
        // frame f is kicked by the frame-start of frame f+1 (staging-kick delay),
        // so the trailing frame flushes the last real frame's column -> frame_seq
        // reaches N_FRAMES.
        for (fr=0; fr<=N_FRAMES; fr++) begin
            for (l=0;l<K;l++) begin
                neg();
                lfp_out_valid   = 1;
                lfp_frame_start = (l==0);
                lfp_out_channel = l[CH_W-1:0];
                lfp_out_data    = $signed(16'(fr*131 + l*17));
            end
            neg(); lfp_out_valid=0; lfp_frame_start=0;
            repeat (FRAME_CLKS - (K+1)) neg();
        end

        repeat (FRAME_CLKS*2) neg();   // let the deferred queue drain

        $display("WAV SPREAD TB: K=%0d oct=%0d voc=%0d tap=%0d FRAME_CLKS=%0d NFRAMES=%0d",
                 K, ACT_OCT, ACT_VOC, ACT_TAP, FRAME_CLKS, N_FRAMES);
        $display("  final_frame_seq=%0d (expect ~%0d)  overrun=%0b  busy_duty=%0d%%",
                 frame_seq, N_FRAMES, overrun, (busy_cyc*100)/((total_cyc==0)?1:total_cyc));
        if (overrun)                       $display("RESULT: OVERRUN (flag asserted)");
        else if (frame_seq < N_FRAMES)     $display("RESULT: OVERRUN (dropped columns: seq=%0d < %0d)", frame_seq, N_FRAMES);
        else                               $display("RESULT: NO_OVERRUN");
        $finish;
    end

    initial begin
        #20000ms;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
