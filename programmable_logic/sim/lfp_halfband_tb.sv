`timescale 1ns/1ps
// Bit-accuracy TB for lfp_halfband (/2 decimating FIR) vs gen_halfband_vectors.py.
module lfp_halfband_tb;
    localparam int N_LANES=8, N_SLOTS=32, DATA_W=16, COEF_W=18, COEF_FRAC=17;
    localparam int RING_DEPTH=64, OUT_W=16, NUM_TAPS=23, K_PACKETS=80;
    localparam logic [7:0] LANE_MASK = 8'b0010_0110;
    localparam int TAPN_W=$clog2(RING_DEPTH+1), RING_AW=$clog2(RING_DEPTH);
    localparam int CH_W=$clog2(N_LANES*N_SLOTS), SLOT_W=$clog2(N_SLOTS), MAXOUT=4096;

    logic clk=0, rstn=0; always #5 clk=~clk;
    logic sample_valid, packet_tick, en;
    logic [N_LANES*DATA_W-1:0] sample_data;
    logic [SLOT_W-1:0] sample_slot;
    logic [N_LANES-1:0] lane_mask;
    logic [TAPN_W-1:0] num_taps;
    logic coef_wr_en; logic [RING_AW-1:0] coef_wr_addr; logic [COEF_W-1:0] coef_wr_data;
    logic out_valid, out_frame_start, busy, compute_overrun;
    logic [CH_W-1:0] out_channel; logic [OUT_W-1:0] out_data;

    lfp_halfband #(.N_LANES(N_LANES),.N_SLOTS(N_SLOTS),.DATA_W(DATA_W),.COEF_W(COEF_W),
        .COEF_FRAC(COEF_FRAC),.RING_DEPTH(RING_DEPTH),.OUT_W(OUT_W)) dut (.*);

    logic [COEF_W-1:0] coefs [0:NUM_TAPS-1];
    logic [127:0] samples [0:K_PACKETS*N_SLOTS-1];
    logic [OUT_W-1:0] exp_val [0:MAXOUT-1];
    logic [15:0] exp_chan [0:MAXOUT-1];
    logic [OUT_W-1:0] got_val [0:MAXOUT-1]; logic [CH_W-1:0] got_chan [0:MAXOUT-1];
    int n_got=0;
    always @(posedge clk) if (rstn && out_valid && n_got<MAXOUT) begin
        got_val[n_got]=out_data; got_chan[n_got]=out_channel; n_got++; end
    int n_expected, errors=0;

    initial begin
        $readmemh("hb_coefs.hex",coefs); $readmemh("hb_samples.hex",samples);
        $readmemh("hb_exp_val.hex",exp_val); $readmemh("hb_exp_chan.hex",exp_chan);
        n_expected=(K_PACKETS/2)*$countones(LANE_MASK)*N_SLOTS;
        sample_valid=0; sample_data=0; sample_slot=0; packet_tick=0; en=0;
        lane_mask=LANE_MASK; num_taps=NUM_TAPS[TAPN_W-1:0];
        coef_wr_en=0; coef_wr_addr=0; coef_wr_data=0;
        repeat (5) @(posedge clk); rstn=1; @(posedge clk);
        for (int j=0;j<NUM_TAPS;j++) begin
            @(negedge clk); coef_wr_en=1; coef_wr_addr=j[RING_AW-1:0]; coef_wr_data=coefs[j];
        end
        @(negedge clk); coef_wr_en=0; @(posedge clk); en=1;
        for (int p=0;p<K_PACKETS;p++) begin
            for (int s=0;s<N_SLOTS;s++) begin
                @(negedge clk); sample_valid=1; sample_slot=s[SLOT_W-1:0];
                sample_data=samples[p*N_SLOTS+s];
                @(negedge clk); sample_valid=0; repeat (3) @(negedge clk);
            end
            @(negedge clk); packet_tick=1; @(negedge clk); packet_tick=0;
            repeat (2600) @(negedge clk);
        end
        repeat (8000) @(posedge clk);
        if (n_got!=n_expected) begin $display("COUNT MISMATCH: got %0d exp %0d",n_got,n_expected); errors++; end
        for (int i=0;i<n_expected;i++)
            if (got_val[i]!==exp_val[i] || got_chan[i]!==exp_chan[i][CH_W-1:0]) begin
                if (errors<20) $display("MISMATCH i=%0d got c=%0d v=%04h exp c=%0d v=%04h",
                    i,got_chan[i],got_val[i],exp_chan[i],exp_val[i]); errors++; end
        if (compute_overrun) begin $display("OVERRUN"); errors++; end
        if (errors==0) $display("RESULT: PASS (%0d outputs, %0d frames)",n_expected,K_PACKETS/2);
        else $display("RESULT: FAIL (%0d errors)",errors);
        $finish;
    end
    initial begin #100ms; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
