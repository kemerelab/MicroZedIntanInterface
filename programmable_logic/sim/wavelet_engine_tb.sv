`timescale 1ns/1ps
// =====================================================================
// wavelet_engine_tb.sv  --  bit-accuracy testbench for wavelet_cqt_engine.
//
// Loads the V complex voice coefficients + the halfband ÷2 coeffs via the
// upload ports, loads the K-lane channel selector, streams N_FRAMES of
// int16 LFP samples (one selected channel per lane, frame-start on lane 0),
// snoops the results BRAM writes, and after the last frame compares the
// most-recent scalogram column (re,im) per (lane, octave, voice) against
// the pure-Python multirate Morse reference (wav_exp.hex).
//
// Config MUST match gen_wavelet_vectors.py.
// =====================================================================
module wavelet_engine_tb;
    localparam int N_CH      = 256;
    localparam int K         = 4;
    localparam int N_OCTAVES = 4;
    localparam int V         = 4;
    localparam int N_TAPS    = 16;
    localparam int HB_TAPS   = 7;
    localparam int DATA_W    = 16;
    localparam int COEF_W    = 18;
    localparam int COEF_FRAC = 17;
    localparam int OUT_W     = 18;
    localparam int RING_DEPTH= 64;
    localparam int RES_AW    = 14;
    localparam int N_FRAMES  = 256;

    // RUNTIME active config (< build maxes -> exercises the engine's COMPACTED
    // wire-packet build). MUST match gen_wavelet_vectors.py ACT_*.
    localparam int ACT_OCTAVES = 3;
    localparam int ACT_VOICES  = 3;
    localparam int ACT_TAPS    = N_TAPS;
    localparam int HDR_WORDS   = 8;
    localparam int N_SCALES_ACT= ACT_OCTAVES*ACT_VOICES;        // compacted stride
    localparam int WIRE_WORDS  = HDR_WORDS + K*N_SCALES_ACT*2;  // full packet words

    localparam int CH_W   = $clog2(N_CH);
    localparam int LANE_W = $clog2(K);
    localparam int OCT_W  = $clog2(N_OCTAVES);
    localparam int VOICE_W= $clog2(V);
    localparam int TAP_W  = $clog2(N_TAPS);
    localparam int HBTAP_W= $clog2(HB_TAPS);
    localparam int COEFN  = V*N_TAPS;
    localparam int COEF_AW= $clog2(2*COEFN);

    // selected source channels for lanes 0..K-1 (arbitrary, distinct)
    localparam int CHAN [0:3] = '{2, 50, 100, 200};

    logic clk=0, rstn=0;
    always #5 clk = ~clk;

    // engine I/O
    logic                 lfp_out_valid=0, lfp_frame_start=0;
    logic [CH_W-1:0]      lfp_out_channel=0;
    logic signed [DATA_W-1:0] lfp_out_data=0;
    logic                 wav_en=0;
    logic [OCT_W:0]       n_octaves_cfg = ACT_OCTAVES;
    logic [VOICE_W:0]     n_voices_cfg  = ACT_VOICES;
    logic [TAP_W:0]       n_taps_cfg    = ACT_TAPS;
    logic [4*N_OCTAVES-1:0] gain_cfg;
    logic                 sel_wr_en=0;   logic [LANE_W-1:0] sel_wr_lane=0; logic [CH_W-1:0] sel_wr_ch=0;
    logic                 coef_wr_en=0;  logic [COEF_AW-1:0] coef_wr_addr=0; logic signed [COEF_W-1:0] coef_wr_data=0;
    logic                 hb_wr_en=0;    logic [HBTAP_W-1:0] hb_wr_addr=0;   logic signed [COEF_W-1:0] hb_wr_data=0;
    logic                 res_clk, res_rst, res_en; logic [3:0] res_we;
    logic [RES_AW-1:0]    res_addr; logic [31:0] res_din; logic [31:0] res_dout=0;
    logic [31:0]          frame_seq; logic busy, overrun;

    // per-octave gain shifts must match the Python OUT_GAIN_SHIFT = [3,2,1,0]
    initial gain_cfg = {4'd0, 4'd1, 4'd2, 4'd3};  // oct0=3 (low nibble), oct1=2, oct2=1, oct3=0

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

    // vectors
    logic [COEF_W-1:0] coef_mem [0:2*COEFN-1];   // interleaved re,im
    logic [COEF_W-1:0] hb_mem   [0:HB_TAPS-1];
    logic [31:0]       samp_mem [0:N_FRAMES*K-1]; // one word/(frame,lane), int16 low
    logic [31:0]       exp_mem  [0:WIRE_WORDS-1];  // the COMPLETE wire packet

    // results capture: a shadow of the whole wire-packet region in the results
    // BRAM (header words 0..7 + compacted payload), last write wins. The engine
    // now writes the FULL wire packet here, so we snapshot it and compare it
    // word-for-word against the Python reference (header + compacted re,im).
    logic [31:0] got_val [0:WIRE_WORDS-1];
    logic        got_vld [0:WIRE_WORDS-1];
    always @(posedge clk) begin
        if (res_we != 0) begin
            int unsigned word;
            word = res_addr >> 2;             // 32-bit word index in the BRAM
            if (word < WIRE_WORDS) begin got_val[word] = res_din; got_vld[word] = 1; end
        end
    end

    task automatic neg; @(negedge clk); endtask

    int i, fr, l, errs, checked;
    integer got_s, exp_s, d;
    initial begin
        $readmemh("wav_coef.hex", coef_mem);
        $readmemh("wav_hb.hex",   hb_mem);
        $readmemh("wav_samp.hex", samp_mem);
        $readmemh("wav_exp.hex",  exp_mem);
        for (i=0;i<WIRE_WORDS;i++) got_vld[i]=0;

        rstn=0; repeat(5) neg(); rstn=1; neg();

        // load channel selector
        for (l=0;l<K;l++) begin neg(); sel_wr_en=1; sel_wr_lane=l[LANE_W-1:0]; sel_wr_ch=CHAN[l][CH_W-1:0]; end
        neg(); sel_wr_en=0;

        // load voice coefficients (interleaved re,im)
        for (i=0;i<2*COEFN;i++) begin neg(); coef_wr_en=1; coef_wr_addr=i[COEF_AW-1:0]; coef_wr_data=coef_mem[i]; end
        neg(); coef_wr_en=0;

        // load halfband coefficients
        for (i=0;i<HB_TAPS;i++) begin neg(); hb_wr_en=1; hb_wr_addr=i[HBTAP_W-1:0]; hb_wr_data=hb_mem[i]; end
        neg(); hb_wr_en=0;

        // enable
        neg(); wav_en=1;

        // stream N_FRAMES + 1 trigger frame. Each frame: drive K lanes, the
        // frame-start pulse on lane 0 (matches lfp_fir_decimator's out_frame_start).
        // The pass for frame f is kicked by the frame-start of frame f+1, so an
        // extra trailing frame flushes the last real frame's column.
        for (fr=0; fr<=N_FRAMES; fr++) begin
            for (l=0;l<K;l++) begin
                neg();
                lfp_out_valid   = 1;
                lfp_frame_start = (l==0);
                lfp_out_channel = CHAN[l][CH_W-1:0];
                lfp_out_data    = (fr<N_FRAMES) ? $signed(samp_mem[fr*K+l][DATA_W-1:0]) : 16'sd0;
            end
            neg(); lfp_out_valid=0; lfp_frame_start=0;
            // wait for the compute pass kicked by THIS frame-start to finish
            // (busy rises a couple cycles after frame-start, then falls). At
            // 3 kHz the real budget is ~28000 clocks; in sim we just gate on busy.
            repeat (4) neg();
            while (busy) neg();
            repeat (4) neg();
        end

        // let the final pass drain
        repeat (200) neg();

        // ---- compare the COMPLETE wire packet the PL built in the results
        //      BRAM (8-word header + compacted re,im payload) word-for-word
        //      against the Python reference (BIT-EXACT). This proves the PL now
        //      emits exactly what the PS used to repack -- so the PS just DMAs
        //      the BRAM frame and sends it (no CPU repack). ----
        errs=0; checked=0;
        for (i=0;i<WIRE_WORDS;i++) begin
            checked++;
            if (i < HDR_WORDS) begin
                // header words compare as raw 32-bit (magic/seq/cfg/gain)
                if (!got_vld[i] || got_val[i] !== exp_mem[i]) begin
                    errs++;
                    if (errs<=20)
                        $display("  HDR MISMATCH word=%0d got=%08x exp=%08x vld=%0b",
                                 i, got_val[i], exp_mem[i], got_vld[i]);
                end
            end else begin
                int unsigned pi, lane, scale;
                pi    = i - HDR_WORDS;            // payload word index
                lane  = pi / (N_SCALES_ACT*2);
                scale = (pi/2) % N_SCALES_ACT;
                got_s = $signed(got_vld[i] ? got_val[i] : 32'h0);
                exp_s = $signed(exp_mem[i]);
                d = (got_s > exp_s) ? (got_s-exp_s) : (exp_s-got_s);
                if (!got_vld[i] || d != 0) begin
                    errs++;
                    if (errs<=20)
                        $display("  PAY MISMATCH word=%0d (lane=%0d scale=%0d %s) got=%0d exp=%0d vld=%0b",
                                 i, lane, scale, (pi&1)?"IM":"RE", got_s, exp_s, got_vld[i]);
                end
            end
        end

        $display("WAVELET engine TB: wire_words=%0d (hdr=%0d) checked=%0d errors=%0d overrun=%0b frame_seq=%0d",
                 WIRE_WORDS, HDR_WORDS, checked, errs, overrun, frame_seq);
        if (errs==0 && !overrun) $display("RESULT: PASS");
        else                     $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #80ms;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
