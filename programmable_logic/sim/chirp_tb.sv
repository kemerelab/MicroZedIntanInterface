`timescale 1ns/1ps
// chirp_tb.sv -- bit-exact check of the analytic chirp NCO in
// data_generator_core. Drives the core in debug_mode + chirp_mode, captures each
// packet's 35 data words off the FIFO write interface (skipping the 7-word header),
// and compares the 8 lane sine values against the Python NCO reference
// (gen_chirp_vectors.py).
//
// Run: source .../settings64.sh && bash run_chirp_tb.sh
module chirp_tb;

    localparam int N_PACKETS   = 40;
    localparam int N_CYC       = 35;   // cycle_counter 0..34
    localparam int NW          = N_PACKETS * N_CYC;
    // chirp config (must match gen_chirp_vectors.py)
    localparam logic [11:0] FSPAN  = 12'h100;
    localparam logic [11:0] RATE   = 12'd4;
    localparam logic [5:0]  STRIDE = 6'd5;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    logic [32*25-1:0] ctrl = '0;
    logic [32*10-1:0] status;
    logic [31:0]      aux_status, aux_read_result;
    logic             fifo_we, pkt_end;
    logic [127:0]     fifo_wd;
    logic [7:0]       mask;
    logic             csn, sclk, copi;

    data_generator_core dut (
        .clk(clk), .rstn(rstn),
        .ctrl_regs_pl(ctrl), .status_regs_pl(status),
        .aux_status(aux_status), .aux_read_result(aux_read_result),
        .fifo_write_en(fifo_we), .fifo_write_data(fifo_wd),
        .fifo_channel_mask(mask), .fifo_full(1'b0), .fifo_count(9'd0),
        .fifo_packet_end_flag(pkt_end),
        .csn(csn), .sclk(sclk), .copi(copi),
        .cipo_a0(1'b0), .cipo_a1(1'b0), .cipo_b0(1'b0), .cipo_b1(1'b0),
        .digital_in(8'h00)
    );

    logic [127:0] exp_w [0:NW-1];

    // Capture the 35 data words per packet off the FIFO write interface. Each packet
    // is 7 header writes (cycle 0, states 0..6) then 35 data writes (state 77, one per
    // cycle); the last data write asserts fifo_packet_end_flag. So skip the first 7
    // writes of each packet and capture the next 35. (Replaces the removed DSP tap,
    // which gated exactly those state-77 data writes.)
    logic [127:0] got_w [0:NW-1];
    int n_got = 0;
    int widx   = 0;   // write index within the current packet
    always @(posedge clk) if (rstn && fifo_we) begin
        if (widx >= 7 && n_got < NW) begin
            got_w[n_got] = fifo_wd;
            n_got++;
        end
        widx = pkt_end ? 0 : widx + 1;
    end

    int errors = 0;

    initial begin
        $readmemh("chirp_exp.hex", exp_w);

        // load a convert-like COPI table (regs 4..21) -- value irrelevant in
        // debug mode but the core still serializes it.
        for (int j = 4; j < 22; j++) ctrl[j*32 +: 32] = 32'h0;

        // CTRL_REG_0: [0] enable, [3] debug_mode
        // CTRL_REG_3: [0] chirp_mode, [7:2] stride, [19:8] fspan, [31:20] rate
        ctrl[0*32 +: 32] = 32'h0;                  // start disabled
        ctrl[3*32 +: 32] = {RATE, FSPAN, STRIDE, 1'b0, 1'b1};  // chirp cfg, chirp_mode=1
        repeat (5) @(posedge clk); rstn = 1; @(posedge clk);

        // enable transmission + debug mode (latched while inactive, so set before
        // enabling). debug_mode bit3, enable bit0.
        ctrl[0*32 +: 32] = 32'h0000_0008;          // debug_mode=1, enable=0 (latch cfg)
        repeat (3) @(posedge clk);
        ctrl[0*32 +: 32] = 32'h0000_0009;          // enable=1, debug_mode=1

        // run enough packets
        repeat (N_PACKETS * 2900 + 5000) @(posedge clk);

        if (n_got < NW) begin
            $display("COUNT: got %0d words, expected %0d", n_got, NW);
            errors++;
        end
        for (int i = 0; i < NW; i++) begin
            if (got_w[i] !== exp_w[i]) begin
                if (errors < 20)
                    $display("MISMATCH word %0d (pkt %0d cyc %0d): got %032h | exp %032h",
                             i, i / N_CYC, i % N_CYC, got_w[i], exp_w[i]);
                errors++;
            end
        end
        if (errors == 0) $display("RESULT: PASS (%0d chirp words checked)", NW);
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #200ms; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
