`timescale 1ns/1ps
// Bit-accuracy TB for cic_decimator vs the Python CIC reference
// (gen_cic_vectors.py). Streams K_PACKETS input frames (all N_SLOTS slots per
// tick) and compares the /R outputs (value + channel) frame-major.
module cic_decimator_tb;

    localparam int N_LANES   = 8;
    localparam int N_SLOTS   = 32;
    localparam int DATA_W    = 16;
    localparam int R         = 5;
    localparam int N_ORDER   = 4;
    localparam int ACC_W     = 32;
    localparam int OUT_W     = 16;
    localparam int GAIN_SHIFT = 10;
    localparam int K_PACKETS = 120;
    localparam logic [7:0] LANE_MASK = 8'b1010_0101;

    localparam int CH_W   = $clog2(N_LANES * N_SLOTS);
    localparam int SLOT_W = $clog2(N_SLOTS);
    localparam int MAXOUT = 4096;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    logic                      sample_valid;
    logic [N_LANES*DATA_W-1:0] sample_data;
    logic [SLOT_W-1:0]         sample_slot;
    logic                      packet_tick;
    logic                      en;
    logic [N_LANES-1:0]        lane_mask;
    logic                      out_valid, out_frame_start, busy, compute_overrun;
    logic [CH_W-1:0]           out_channel;
    logic [OUT_W-1:0]          out_data;

    cic_decimator #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W), .R(R),
        .N_ORDER(N_ORDER), .ACC_W(ACC_W), .OUT_W(OUT_W), .GAIN_SHIFT(GAIN_SHIFT)
    ) dut (
        .clk(clk), .rstn(rstn),
        .sample_valid(sample_valid), .sample_data(sample_data),
        .sample_slot(sample_slot), .packet_tick(packet_tick),
        .en(en), .lane_mask(lane_mask),
        .out_valid(out_valid), .out_channel(out_channel), .out_data(out_data),
        .out_frame_start(out_frame_start), .busy(busy), .compute_overrun(compute_overrun)
    );

    logic [127:0]      samples [0:K_PACKETS*N_SLOTS-1];
    logic [OUT_W-1:0]  exp_val [0:MAXOUT-1];
    logic [15:0]       exp_chan[0:MAXOUT-1];
    logic [OUT_W-1:0]  got_val [0:MAXOUT-1];
    logic [CH_W-1:0]   got_chan[0:MAXOUT-1];
    int n_got = 0;
    always @(posedge clk) if (rstn && out_valid && n_got < MAXOUT) begin
        got_val[n_got] = out_data; got_chan[n_got] = out_channel; n_got++;
    end

    int n_expected, errors = 0;

    initial begin
        $readmemh("cic_samples.hex",  samples);
        $readmemh("cic_exp_val.hex",  exp_val);
        $readmemh("cic_exp_chan.hex", exp_chan);
        n_expected = (K_PACKETS / R) * $countones(LANE_MASK) * N_SLOTS;

        sample_valid = 0; sample_data = 0; sample_slot = 0; packet_tick = 0;
        en = 0; lane_mask = LANE_MASK;
        repeat (5) @(posedge clk); rstn = 1; @(posedge clk);
        @(negedge clk); en = 1;

        for (int p = 0; p < K_PACKETS; p++) begin
            for (int s = 0; s < N_SLOTS; s++) begin
                @(negedge clk);
                sample_valid = 1; sample_slot = s[SLOT_W-1:0];
                sample_data  = samples[p*N_SLOTS + s];
                @(negedge clk); sample_valid = 0;
            end
            @(negedge clk); packet_tick = 1;
            @(negedge clk); packet_tick = 0;
            // pad to let the integrate (+ occasional comb) pass finish; real HW
            // has ~2800 clk/packet, this TB is far faster than real time.
            repeat (3000) @(negedge clk);
        end
        repeat (4000) @(posedge clk);

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
                                  n_expected, K_PACKETS / R);
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end
    initial begin #200ms; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
