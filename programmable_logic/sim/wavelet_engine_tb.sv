`timescale 1ns/1ps
// =====================================================================
// wavelet_engine_tb.sv  --  VALUES-ONLY bit-accuracy TB for the wavelet
// engine. This is the "scalogram VALUES must not change" gate: it proves
// the engine's (re,im) coefficients are BIT-EXACT to the v2 reference,
// independent of the packet framing.
//
// The engine now writes ONE unified per-octave packet into each octave's
// OWN BRAM region (region(o) = o*OCT_STRIDE_WORDS). This TB snoops the
// region payloads (skipping the 8-word headers) and reconstructs the
// (lane, octave, voice) re/im VALUES, then compares them against the
// pure-Python multirate Morse reference wav_exp.hex (lane/octave/voice
// order -- the legacy values reference, unchanged numbers).
//
// wavelet_octave_tb.sv covers the per-octave packet FRAMING (header
// fields, region layout, SEQ). This TB isolates the numerical values.
//
// Config MUST match gen_wavelet_vectors.py.
// =====================================================================
module wavelet_engine_tb;
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

    // RUNTIME active config. MUST match gen_wavelet_vectors.py ACT_*.
    localparam int ACT_OCTAVES = 3;
    localparam int ACT_VOICES  = 3;
    localparam int ACT_TAPS    = N_TAPS;
    localparam int HDR_WORDS   = 8;
    localparam int OCT_STRIDE_WORDS = 512;            // must match the engine
    localparam int N_VALS      = K*ACT_OCTAVES*ACT_VOICES*2;  // total re/im values

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
    logic [31:0]       exp_mem  [0:N_VALS-1];   // values, lane/octave/voice order

    // per-octave region payload shadow: [octave][lane*ACT_VOICES*2 + voice*2 + ri]
    localparam int PAY_PER_OCT = K*ACT_VOICES*2;
    logic [31:0] got_pay [0:ACT_OCTAVES-1][0:PAY_PER_OCT-1];
    logic        got_vld [0:ACT_OCTAVES-1][0:PAY_PER_OCT-1];
    always @(posedge clk) begin
        if (res_we != 0) begin
            int unsigned word, oc, idx;
            word = res_addr >> 2;
            oc   = word / OCT_STRIDE_WORDS;
            idx  = word % OCT_STRIDE_WORDS;
            if (oc < ACT_OCTAVES && idx >= HDR_WORDS && (idx-HDR_WORDS) < PAY_PER_OCT) begin
                got_pay[oc][idx-HDR_WORDS] = res_din; got_vld[oc][idx-HDR_WORDS] = 1;
            end
        end
    end

    task automatic neg; @(negedge clk); endtask

    int i, fr, l, o, lane, voice, errs, checked, ei, pi;
    integer got_s, exp_s;
    initial begin
        $readmemh("wav_coef.hex", coef_mem);
        $readmemh("wav_hb.hex",   hb_mem);
        $readmemh("wav_samp.hex", samp_mem);
        $readmemh("wav_exp.hex",  exp_mem);
        for (o=0;o<ACT_OCTAVES;o++) for (i=0;i<PAY_PER_OCT;i++) got_vld[o][i]=0;

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
                if (l==0) master_ts = master_ts + 64'd1;
            end
            neg(); lfp_out_valid=0; lfp_frame_start=0;
            repeat (4) neg();
            while (busy) neg();
            repeat (4) neg();
        end
        repeat (300) neg();

        // ---- VALUES-ONLY compare: reconstruct (lane,octave,voice) re/im from the
        //      per-octave region payloads and compare against wav_exp.hex (which
        //      is ordered lane-major / octave / voice). ----
        errs=0; checked=0;
        ei = 0;   // index into exp_mem (lane/octave/voice/ri)
        for (l=0; l<K; l++) begin
            for (o=0; o<ACT_OCTAVES; o++) begin
                for (voice=0; voice<ACT_VOICES; voice++) begin
                    for (i=0; i<2; i++) begin                 // re then im
                        pi = l*ACT_VOICES*2 + voice*2 + i;    // index in octave region
                        checked++;
                        got_s = $signed(got_vld[o][pi] ? got_pay[o][pi] : 32'h0);
                        exp_s = $signed(exp_mem[ei]);
                        if (!got_vld[o][pi] || got_s != exp_s) begin
                            errs++;
                            if (errs<=20)
                                $display("  VAL MISMATCH lane=%0d oct=%0d v=%0d %s got=%0d exp=%0d vld=%0b",
                                         l, o, voice, i?"IM":"RE", got_s, exp_s, got_vld[o][pi]);
                        end
                        ei++;
                    end
                end
            end
        end

        $display("WAVELET engine TB (values-only): vals=%0d checked=%0d errors=%0d overrun=%0b frame_seq=%0d",
                 N_VALS, checked, errs, overrun, frame_seq);
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
