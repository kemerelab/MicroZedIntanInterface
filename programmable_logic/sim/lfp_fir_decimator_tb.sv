`timescale 1ns/1ps
// Bit-accuracy testbench for lfp_fir_decimator. Loads coefficients via the
// indirect write port, streams K_PACKETS of samples, collects the decimated
// outputs, and compares against the Python fixed-point reference vectors.
// Config MUST match gen_lfp_fir_vectors.py.
module lfp_fir_decimator_tb;

    localparam int N_LANES   = 8;
    localparam int N_SLOTS   = 32;
    localparam int DATA_W    = 16;
    localparam int COEF_W    = 18;
    localparam int COEF_FRAC = 17;
    localparam int RING_DEPTH = 256;
    localparam int OUT_W     = 16;
    localparam int NUM_TAPS  = 25;
    localparam int DECIM_R   = 15;
    localparam int K_PACKETS = 300;
    localparam logic [7:0] LANE_MASK = 8'b1010_0101;

    localparam int TAPN_W = $clog2(RING_DEPTH + 1);
    localparam int RING_AW = $clog2(RING_DEPTH);
    localparam int CH_W   = $clog2(N_LANES * N_SLOTS);
    localparam int SLOT_W = $clog2(N_SLOTS);
    localparam int MAXOUT = 4096;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    logic                      sample_valid;
    logic [N_LANES*DATA_W-1:0] sample_data;
    logic [SLOT_W-1:0]         sample_slot;
    logic                      packet_tick;
    logic                      lfp_en;
    logic [N_LANES-1:0]        lane_mask;
    logic [7:0]                decim_R;
    logic [TAPN_W-1:0]         num_taps;
    logic                      coef_wr_en;
    logic [RING_AW-1:0]        coef_wr_addr;
    logic [COEF_W-1:0]         coef_wr_data;
    logic                      out_valid;
    logic [CH_W-1:0]           out_channel;
    logic [OUT_W-1:0]          out_data;
    logic                      out_frame_start, busy, compute_overrun;

    lfp_fir_decimator #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W), .COEF_W(COEF_W),
        .COEF_FRAC(COEF_FRAC), .RING_DEPTH(RING_DEPTH), .OUT_W(OUT_W)
    ) dut (.*);

    // ---- vectors ----
    logic [COEF_W-1:0] coefs   [0:NUM_TAPS-1];
    logic [127:0]      samples [0:K_PACKETS*N_SLOTS-1];
    logic [OUT_W-1:0]  exp_val [0:MAXOUT-1];
    logic [15:0]       exp_chan[0:MAXOUT-1];

    // ---- output collector ----
    logic [OUT_W-1:0] got_val  [0:MAXOUT-1];
    logic [CH_W-1:0]  got_chan [0:MAXOUT-1];
    int n_got = 0;
    always @(posedge clk) if (rstn && out_valid && n_got < MAXOUT) begin
        got_val[n_got]  = out_data;
        got_chan[n_got] = out_channel;
        n_got++;
    end

    int n_expected, errors = 0;

    initial begin
        $readmemh("lfp_coefs.hex",    coefs);
        $readmemh("lfp_samples.hex",  samples);
        $readmemh("lfp_exp_val.hex",  exp_val);
        $readmemh("lfp_exp_chan.hex", exp_chan);
        n_expected = (K_PACKETS / DECIM_R) * $countones(LANE_MASK) * N_SLOTS;

        sample_valid = 0; sample_data = 0; sample_slot = 0; packet_tick = 0;
        lfp_en = 0; lane_mask = LANE_MASK; decim_R = DECIM_R[7:0]; num_taps = NUM_TAPS[TAPN_W-1:0];
        coef_wr_en = 0; coef_wr_addr = 0; coef_wr_data = 0;
        repeat (5) @(posedge clk);
        rstn = 1;
        @(posedge clk);

        // load coefficients through the indirect write port
        for (int j = 0; j < NUM_TAPS; j++) begin
            @(negedge clk);
            coef_wr_en = 1; coef_wr_addr = j[RING_AW-1:0]; coef_wr_data = coefs[j];
        end
        @(negedge clk); coef_wr_en = 0;
        @(posedge clk); lfp_en = 1;

        // stream the packets
        for (int p = 0; p < K_PACKETS; p++) begin
            for (int s = 0; s < N_SLOTS; s++) begin
                @(negedge clk);
                sample_valid = 1; sample_slot = s[SLOT_W-1:0];
                sample_data  = samples[p*N_SLOTS + s];
                @(negedge clk); sample_valid = 0;
                repeat (7) @(negedge clk);          // spacing so the compute pass keeps up
            end
            @(negedge clk); packet_tick = 1;
            @(negedge clk); packet_tick = 0;
            repeat (3) @(negedge clk);
        end
        repeat (4000) @(posedge clk);                // let the last frame drain

        // ---- compare ----
        if (n_got != n_expected) begin
            $display("COUNT MISMATCH: got %0d expected %0d", n_got, n_expected);
            errors++;
        end
        for (int i = 0; i < n_expected; i++) begin
            if (got_val[i] !== exp_val[i] || got_chan[i] !== exp_chan[i][CH_W-1:0]) begin
                if (errors < 20)
                    $display("MISMATCH i=%0d: got chan=%0d val=%04h | exp chan=%0d val=%04h",
                             i, got_chan[i], got_val[i], exp_chan[i], exp_val[i]);
                errors++;
            end
        end
        if (compute_overrun) begin $display("COMPUTE_OVERRUN asserted"); errors++; end

        if (errors == 0) $display("RESULT: PASS (%0d outputs checked, %0d frames)",
                                  n_expected, K_PACKETS / DECIM_R);
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #50ms;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end
endmodule
