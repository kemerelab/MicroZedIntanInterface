// Testbench for stft_engine.sv (+ behavioral stft_fft). Feeds K=4 lanes of
// int16 samples over N=64 frames, triggers one STFT pass, snoops the results
// BRAM writes, and compares the captured complex float32 spectrum to the
// Python windowed-DFT reference (stft_exp.hex) with float tolerance.
`timescale 1ns/1ps
module stft_engine_tb;
    localparam int N_CH   = 256;
    localparam int K      = 4;
    localparam int MAX_N  = 256;
    localparam int N      = 64;
    localparam int NB     = N/2 + 1;          // 33 Hermitian bins
    localparam int CHAN0=2, CHAN1=50, CHAN2=100, CHAN3=200;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;                       // 100 MHz sim clock

    // engine I/O
    logic               lfp_out_valid=0, lfp_frame_start=0;
    logic [7:0]         lfp_out_channel=0;
    logic signed [15:0] lfp_out_data=0;
    logic               stft_en=0;
    logic [3:0]         nfft_log2=6;            // N=64
    logic [15:0]        hop=64;
    logic               sel_wr_en=0;  logic [1:0] sel_wr_lane=0;  logic [7:0] sel_wr_ch=0;
    logic               win_wr_en=0;  logic [7:0] win_wr_addr=0;  logic signed [15:0] win_wr_data=0;
    logic               res_clk, res_rst, res_en;  logic [3:0] res_we;
    logic [13:0]        res_addr;  logic [31:0] res_din;  logic [31:0] res_dout=0;
    logic [31:0]        frame_seq;  logic busy, overflow;

    stft_engine #(.N_CH(N_CH), .K(K), .MAX_N(MAX_N), .RES_AW(14)) dut (
        .clk(clk), .rstn(rstn),
        .lfp_out_valid(lfp_out_valid), .lfp_out_channel(lfp_out_channel),
        .lfp_out_data(lfp_out_data), .lfp_frame_start(lfp_frame_start),
        .stft_en(stft_en), .nfft_log2(nfft_log2), .hop(hop),
        .sel_wr_en(sel_wr_en), .sel_wr_lane(sel_wr_lane), .sel_wr_ch(sel_wr_ch),
        .win_wr_en(win_wr_en), .win_wr_addr(win_wr_addr), .win_wr_data(win_wr_data),
        .res_bram_clk(res_clk), .res_bram_rst(res_rst), .res_bram_addr(res_addr),
        .res_bram_din(res_din), .res_bram_dout(res_dout), .res_bram_en(res_en), .res_bram_we(res_we),
        .frame_seq(frame_seq), .busy(busy), .overflow(overflow)
    );

    // vectors
    logic [15:0] win_mem  [0:N-1];
    logic [63:0] samp_mem [0:N-1];              // K x 16-bit per frame
    logic [31:0] exp_mem  [0:K*NB*2-1];         // lane-major, bin, [re,im]
    int CHAN [0:3];

    // capture: idx = lane*(NB*2) + bin*2 + phase
    logic [31:0] got_val [0:K*NB*2-1];
    logic        got_vld [0:K*NB*2-1];

    always @(posedge clk) begin
        if (res_we != 0) begin
            int unsigned word, lane, bin, phase, idx;
            word  = res_addr >> 2;
            phase = word & 1;
            lane  = (word >> 1) / NB;
            bin   = (word >> 1) % NB;
            idx   = lane*(NB*2) + bin*2 + phase;
            if (idx < K*NB*2) begin got_val[idx] = res_din; got_vld[idx] = 1; end
        end
    end

    task automatic neg; @(negedge clk); endtask

    int i, f, l, errs, checked;
    real a, b, da, db, tol, floor_;
    initial begin
        $readmemh("stft_win.hex",  win_mem);
        $readmemh("stft_samp.hex", samp_mem);
        $readmemh("stft_exp.hex",  exp_mem);
        CHAN[0]=CHAN0; CHAN[1]=CHAN1; CHAN[2]=CHAN2; CHAN[3]=CHAN3;
        for (i=0;i<K*NB*2;i++) got_vld[i]=0;

        // reset
        rstn=0; repeat(4) neg(); rstn=1; neg();

        // load selector table
        for (l=0;l<K;l++) begin neg(); sel_wr_en=1; sel_wr_lane=l[1:0]; sel_wr_ch=CHAN[l][7:0]; end
        neg(); sel_wr_en=0;

        // load Hann window
        for (i=0;i<N;i++) begin neg(); win_wr_en=1; win_wr_addr=i[7:0]; win_wr_data=win_mem[i]; end
        neg(); win_wr_en=0;

        // config + enable. hop=N+1 so the single trigger lands on the (N+1)-th
        // frame-start, which completes frame N-1 -> window = frames 0..N-1.
        nfft_log2=6; hop=N+1; stft_en=1; neg();

        // feed N data frames + 1 trigger frame; the FIRST channel of each frame
        // carries the frame-start pulse (matches lfp_fir_decimator's out_frame_start).
        for (f=0; f<=N; f++) begin
            for (l=0;l<K;l++) begin
                neg();
                lfp_out_valid   = 1;
                lfp_frame_start = (l==0);                       // pulse on channel 0
                lfp_out_channel = CHAN[l][7:0];
                lfp_out_data    = (f<N) ? $signed(samp_mem[f][16*l +: 16]) : 16'sd0;
            end
            neg(); lfp_out_valid=0; lfp_frame_start=0;
            neg();                                              // gap between frames
        end

        // wait for the full pass (all K lanes captured -> frame_seq increments)
        fork : wd
            begin wait(frame_seq > 0); end
            begin repeat(40000) neg(); $display("TIMEOUT waiting for pass"); end
        join_any
        disable wd;
        repeat(8) neg();

        // compare
        errs=0; checked=0;
        tol=1.0e-3; floor_=1.0e5;
        for (l=0;l<K;l++) for (i=0;i<NB;i++) begin
            int ridx, iidx;
            ridx = l*(NB*2)+i*2+0;  iidx = l*(NB*2)+i*2+1;
            // real
            a=$bitstoshortreal(got_vld[ridx]?got_val[ridx]:32'h0);
            b=$bitstoshortreal(exp_mem[ridx]);
            da = (a>b)?(a-b):(b-a);  db=(b>0.0)?b:-b;
            checked++;
            if (!got_vld[ridx] || da > tol*db+floor_) begin
                errs++; if (errs<=12) $display("  MISMATCH lane%0d bin%0d RE got=%g exp=%g vld=%0b", l,i,a,b,got_vld[ridx]);
            end
            // imag
            a=$bitstoshortreal(got_vld[iidx]?got_val[iidx]:32'h0);
            b=$bitstoshortreal(exp_mem[iidx]);
            da=(a>b)?(a-b):(b-a);  db=(b>0.0)?b:-b;
            checked++;
            if (!got_vld[iidx] || da > tol*db+floor_) begin
                errs++; if (errs<=12) $display("  MISMATCH lane%0d bin%0d IM got=%g exp=%g vld=%0b", l,i,a,b,got_vld[iidx]);
            end
        end

        $display("STFT engine TB: checked=%0d errors=%0d overflow=%0b frame_seq=%0d", checked, errs, overflow, frame_seq);
        if (errs==0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        $finish;
    end
endmodule
