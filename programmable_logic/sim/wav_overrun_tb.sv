`timescale 1ns/1ps
// =====================================================================
// wav_overrun_tb.sv -- COMPUTE-BUDGET / OVERRUN measurement for the
// wavelet_cqt_engine at full build config. NOT a bit-accuracy test.
//
// Feeds K lanes/frame with frame-start on lane 0, spacing successive
// frames exactly FRAME_CLKS apart (the real 3 kHz @ 84 MHz budget =
// 84e6/3000 = 28000 clocks). Runs >= 256 base frames so fcount wraps
// through 0 (where ALL octaves coincide in one pass -- the true worst
// case). Reports the max measured busy-duration in clocks and whether
// the engine's overrun flag ever asserts.
//
// Configurable via plusargs: +K= +NOCT= +NVOC= +NTAP= +FRAMECLKS=
// =====================================================================
module wav_overrun_tb;
    parameter int N_CH      = 256;
    parameter int K         = 8;
    parameter int N_OCTAVES = 8;
    parameter int V         = 4;
    parameter int N_TAPS    = 24;
    parameter int HB_TAPS   = 7;
    parameter int DATA_W    = 16;
    parameter int COEF_W    = 18;
    parameter int COEF_FRAC = 17;
    parameter int OUT_W     = 18;
    parameter int RING_DEPTH= 64;
    parameter int RES_AW    = 17;        // 128 KB result BRAM (32768 words)

    // runtime active config = the build max (worst case)
    int ACT_OCT  = N_OCTAVES;
    int ACT_VOC  = V;
    int ACT_TAP  = N_TAPS;
    int FRAME_CLKS = 28000;              // 84 MHz / 3 kHz
    int N_FRAMES   = 600;                // > 512 so fcount sweeps through 0 twice

    localparam int CH_W   = $clog2(N_CH);
    localparam int LANE_W = $clog2(K);
    localparam int OCT_W  = $clog2(N_OCTAVES);
    localparam int VOICE_W= $clog2(V);
    localparam int TAP_W  = $clog2(N_TAPS);
    localparam int HBTAP_W= $clog2(HB_TAPS);
    localparam int COEFN  = V*N_TAPS;
    localparam int COEF_AW= $clog2(2*COEFN);

    logic clk=0, rstn=0;
    always #5 clk = ~clk;     // 100 MHz sim clock; cycle COUNT is what matters

    logic                 lfp_out_valid=0, lfp_frame_start=0;
    logic [CH_W-1:0]      lfp_out_channel=0;
    logic signed [DATA_W-1:0] lfp_out_data=0;
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
        .wav_en(wav_en), .n_octaves_cfg(n_octaves_cfg), .n_voices_cfg(n_voices_cfg),
        .n_taps_cfg(n_taps_cfg), .gain_cfg(gain_cfg),
        .sel_wr_en(sel_wr_en), .sel_wr_lane(sel_wr_lane), .sel_wr_ch(sel_wr_ch),
        .coef_wr_en(coef_wr_en), .coef_wr_addr(coef_wr_addr), .coef_wr_data(coef_wr_data),
        .hb_wr_en(hb_wr_en), .hb_wr_addr(hb_wr_addr), .hb_wr_data(hb_wr_data),
        .res_bram_clk(res_clk), .res_bram_rst(res_rst), .res_bram_addr(res_addr),
        .res_bram_din(res_din), .res_bram_dout(res_dout), .res_bram_en(res_en), .res_bram_we(res_we),
        .frame_seq(frame_seq), .busy(busy), .overrun(overrun)
    );

    // ---- busy-duration tracker ----
    int unsigned cyc;            // free-running cycle counter
    int unsigned busy_start;
    int unsigned max_busy;
    int unsigned max_busy_seq;
    bit          was_busy;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        if (busy && !was_busy) busy_start <= cyc;          // rising edge
        if (!busy && was_busy) begin                       // falling edge
            int unsigned dur; dur = cyc - busy_start;
            if (dur > max_busy) begin max_busy = dur; max_busy_seq = frame_seq; end
        end
        was_busy <= busy;
    end

    task automatic neg; @(negedge clk); endtask

    int i, fr, l;
    initial begin
        if (!$value$plusargs("FRAMECLKS=%d", FRAME_CLKS)) FRAME_CLKS = 28000;
        if (!$value$plusargs("NFRAMES=%d",   N_FRAMES))   N_FRAMES   = 600;
        n_octaves_cfg = ACT_OCT; n_voices_cfg = ACT_VOC; n_taps_cfg = ACT_TAP;
        cyc=0; max_busy=0; was_busy=0;

        rstn=0; repeat(5) neg(); rstn=1; neg();

        // channel selector: lane l -> channel l (distinct, in-range)
        for (l=0;l<K;l++) begin neg(); sel_wr_en=1; sel_wr_lane=l[LANE_W-1:0]; sel_wr_ch=l[CH_W-1:0]; end
        neg(); sel_wr_en=0;

        // load nonzero-ish voice + halfband coefs (values irrelevant for timing)
        for (i=0;i<2*COEFN;i++) begin neg(); coef_wr_en=1; coef_wr_addr=i[COEF_AW-1:0]; coef_wr_data=(i*7+1); end
        neg(); coef_wr_en=0;
        for (i=0;i<HB_TAPS;i++) begin neg(); hb_wr_en=1; hb_wr_addr=i[HBTAP_W-1:0]; hb_wr_data=(i*3+5); end
        neg(); hb_wr_en=0;

        neg(); wav_en=1;

        for (fr=0; fr<N_FRAMES; fr++) begin
            // drive K lanes back-to-back (mirrors lfp output: one chan/lane/cycle)
            for (l=0;l<K;l++) begin
                neg();
                lfp_out_valid   = 1;
                lfp_frame_start = (l==0);
                lfp_out_channel = l[CH_W-1:0];
                lfp_out_data    = $signed(16'(fr*131 + l*17));
            end
            neg(); lfp_out_valid=0; lfp_frame_start=0;
            // hold the rest of the frame interval idle: FRAME_CLKS total between
            // frame-starts. Already spent (K+1) cycles driving lanes.
            repeat (FRAME_CLKS - (K+1)) neg();
        end

        repeat (FRAME_CLKS) neg();   // let last pass finish

        $display("WAV OVERRUN TB: K=%0d oct=%0d voc=%0d tap=%0d FRAME_CLKS=%0d",
                 K, ACT_OCT, ACT_VOC, ACT_TAP, FRAME_CLKS);
        $display("  max_busy_cycles=%0d (at frame_seq~%0d)  budget=%0d  margin=%0d",
                 max_busy, max_busy_seq, FRAME_CLKS, FRAME_CLKS - max_busy);
        $display("  overrun_flag=%0b  final_frame_seq=%0d", overrun, frame_seq);
        if (overrun)              $display("RESULT: OVERRUN");
        else if (max_busy >= FRAME_CLKS) $display("RESULT: OVERRUN (busy>=budget, no fresh-frame collision in sim)");
        else                      $display("RESULT: NO_OVERRUN");
        $finish;
    end

    initial begin
        #5000ms;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
