`timescale 1ns/1ps
// =====================================================================
// wavelet_octave_tb.sv  --  STEP 2 bit-accuracy TB for the UNIFIED
// octave-split wavelet packetizer.
//
// Same compute as the v2 engine (bit-exact VALUES), but the engine now
// writes ONE unified per-octave wire packet into each octave's OWN region
// of the results BRAM (region(o) = o*OCT_STRIDE_WORDS). This TB:
//   * loads coeffs + selector, sets n_channels_cfg = K,
//   * streams N_FRAMES of int16 LFP (one channel/lane, frame-start on lane 0)
//     while driving a 64-bit master_timestamp counter,
//   * snoops the BRAM writes into a per-octave region shadow,
//   * after the last frame, compares each octave's region against the
//     per-octave Python reference (wav_oct{o}_exp.hex): MAGIC / TYPE_VER /
//     AUX0 / AUX1 and the K*V complex payload EXACTLY; SEQ is checked
//     advancing (>0) and the timestamp non-exact (runtime).
//
// Config MUST match gen_wavelet_vectors.py.
// =====================================================================
module wavelet_octave_tb;
    localparam int N_CH      = 256;
    localparam int K         = 4;
    localparam int N_OCTAVES = 4;
    localparam int V         = 4;
    localparam int N_TAPS    = 24;
    localparam int HB_TAPS   = 7;
    localparam int DATA_W    = 16;
    localparam int COEF_W    = 18;
    localparam int COEF_FRAC = 17;
    localparam int OUT_W     = 18;
    localparam int RING_DEPTH= 64;
    localparam int RES_AW    = 16;          // 64 KB; per-octave regions need >16 KB
    localparam int N_FRAMES  = 256;

    // RUNTIME active config (MUST match gen_wavelet_vectors.py ACT_*).
    localparam int ACT_OCTAVES = 3;
    localparam int ACT_VOICES  = 3;
    localparam int ACT_TAPS    = N_TAPS;
    localparam int HDR_WORDS   = 8;
    localparam int OCT_STRIDE_WORDS = 512;             // must match the engine
    localparam int PKT_WORDS   = HDR_WORDS + K*ACT_VOICES*2;  // one octave packet

    localparam int CH_W   = $clog2(N_CH);
    localparam int LANE_W = $clog2(K);
    localparam int OCT_W  = $clog2(N_OCTAVES);
    localparam int VOICE_W= $clog2(V);
    localparam int TAP_W  = $clog2(N_TAPS);
    localparam int HBTAP_W= $clog2(HB_TAPS);
    localparam int COEFN  = V*N_TAPS;
    localparam int COEF_AW= $clog2(2*COEFN);

    localparam int CHAN [0:3] = '{2, 50, 100, 200};

    logic clk=0, rstn=0;
    always #5 clk = ~clk;

    logic                 lfp_out_valid=0, lfp_frame_start=0;
    logic [CH_W-1:0]      lfp_out_channel=0;
    logic signed [DATA_W-1:0] lfp_out_data=0;
    logic [63:0]          master_ts=0;
    logic [LANE_W:0]      n_channels_cfg = K;
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

    initial gain_cfg = {4'd0, 4'd1, 4'd2, 4'd3};  // oct0=3, oct1=2, oct2=1, oct3=0

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

    // vectors
    logic [COEF_W-1:0] coef_mem [0:2*COEFN-1];
    logic [COEF_W-1:0] hb_mem   [0:HB_TAPS-1];
    logic [31:0]       samp_mem [0:N_FRAMES*K-1];
    logic [31:0]       exp_oct  [0:ACT_OCTAVES-1][0:PKT_WORDS-1];

    // per-octave region shadow (last write wins), one PKT_WORDS window per region
    logic [31:0] got_val [0:ACT_OCTAVES-1][0:PKT_WORDS-1];
    logic        got_vld [0:ACT_OCTAVES-1][0:PKT_WORDS-1];
    always @(posedge clk) begin
        if (res_we != 0) begin
            int unsigned word, oc, idx;
            word = res_addr >> 2;             // 32-bit word index
            oc   = word / OCT_STRIDE_WORDS;
            idx  = word % OCT_STRIDE_WORDS;
            if (oc < ACT_OCTAVES && idx < PKT_WORDS) begin
                got_val[oc][idx] = res_din; got_vld[oc][idx] = 1;
            end
        end
    end

    task automatic neg; @(negedge clk); endtask

    int i, fr, l, o, errs, checked;
    integer got_s, exp_s;
    initial begin
        $readmemh("wav_coef.hex", coef_mem);
        $readmemh("wav_hb.hex",   hb_mem);
        $readmemh("wav_samp.hex", samp_mem);
        $readmemh("wav_oct0_exp.hex", exp_oct[0]);
        $readmemh("wav_oct1_exp.hex", exp_oct[1]);
        $readmemh("wav_oct2_exp.hex", exp_oct[2]);
        for (o=0;o<ACT_OCTAVES;o++) for (i=0;i<PKT_WORDS;i++) got_vld[o][i]=0;

        rstn=0; repeat(5) neg(); rstn=1; neg();

        for (l=0;l<K;l++) begin neg(); sel_wr_en=1; sel_wr_lane=l[LANE_W-1:0]; sel_wr_ch=CHAN[l][CH_W-1:0]; end
        neg(); sel_wr_en=0;
        for (i=0;i<2*COEFN;i++) begin neg(); coef_wr_en=1; coef_wr_addr=i[COEF_AW-1:0]; coef_wr_data=coef_mem[i]; end
        neg(); coef_wr_en=0;
        for (i=0;i<HB_TAPS;i++) begin neg(); hb_wr_en=1; hb_wr_addr=i[HBTAP_W-1:0]; hb_wr_data=hb_mem[i]; end
        neg(); hb_wr_en=0;
        neg(); wav_en=1;

        for (fr=0; fr<=N_FRAMES; fr++) begin
            for (l=0;l<K;l++) begin
                neg();
                lfp_out_valid   = 1;
                lfp_frame_start = (l==0);
                lfp_out_channel = CHAN[l][CH_W-1:0];
                lfp_out_data    = (fr<N_FRAMES) ? $signed(samp_mem[fr*K+l][DATA_W-1:0]) : 16'sd0;
                if (l==0) master_ts = master_ts + 64'd1;   // tick the master timestamp/frame
            end
            neg(); lfp_out_valid=0; lfp_frame_start=0;
            repeat (4) neg();
            while (busy) neg();
            repeat (4) neg();
        end
        repeat (300) neg();

        // ---- compare each octave's per-octave packet ----
        errs=0; checked=0;
        for (o=0;o<ACT_OCTAVES;o++) begin
            for (i=0;i<PKT_WORDS;i++) begin
                checked++;
                if (i==2 || i==3 || i==4) begin
                    // w2/w3 = timestamp (runtime), w4 = SEQ (runtime). Don't compare
                    // exact; just require they were written (valid).
                    if (!got_vld[o][i]) begin
                        errs++;
                        if (errs<=20) $display("  oct%0d word%0d NOT WRITTEN (hdr ts/seq)", o, i);
                    end
                    // SEQ must be nonzero (the octave was emitted at least once)
                    if (i==4 && got_vld[o][i] && got_val[o][i]==0) begin
                        errs++;
                        if (errs<=20) $display("  oct%0d SEQ==0 (never emitted)", o);
                    end
                end else if (i < HDR_WORDS) begin
                    // exact: magic / type_ver / aux0 / aux1 / rsvd
                    if (!got_vld[o][i] || got_val[o][i] !== exp_oct[o][i]) begin
                        errs++;
                        if (errs<=20)
                            $display("  oct%0d HDR MISMATCH word=%0d got=%08x exp=%08x vld=%0b",
                                     o, i, got_val[o][i], exp_oct[o][i], got_vld[o][i]);
                    end
                end else begin
                    int unsigned pi, lane, voice;
                    pi    = i - HDR_WORDS;
                    lane  = pi / (ACT_VOICES*2);
                    voice = (pi/2) % ACT_VOICES;
                    got_s = $signed(got_vld[o][i] ? got_val[o][i] : 32'h0);
                    exp_s = $signed(exp_oct[o][i]);
                    if (!got_vld[o][i] || got_s != exp_s) begin
                        errs++;
                        if (errs<=20)
                            $display("  oct%0d PAY MISMATCH word=%0d (lane=%0d v=%0d %s) got=%0d exp=%0d vld=%0b",
                                     o, i, lane, voice, (pi&1)?"IM":"RE", got_s, exp_s, got_vld[o][i]);
                    end
                end
            end
        end

        $display("WAVELET octave TB: octaves=%0d pkt_words=%0d checked=%0d errors=%0d overrun=%0b frame_seq=%0d",
                 ACT_OCTAVES, PKT_WORDS, checked, errs, overrun, frame_seq);
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
