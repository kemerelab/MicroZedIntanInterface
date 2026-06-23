`timescale 1ns/1ps
// Integration testbench for lfp_dsp_block: loads coefficients through the strobe
// window, streams ALL 35 cycle slots (aux slots {0,1,34} = junk that must be
// ignored) as offset-binary, captures the LFP output BRAM writes, and checks
// them against the Python reference (gen_lfp_block_vectors.py).
module lfp_dsp_block_tb;

    localparam int N_LANES   = 8;
    localparam int N_SLOTS   = 32;
    localparam int FIRST_AMP = 2;
    localparam int N_CYCLES  = 35;
    localparam int COEF_W    = 18;
    localparam int NUM_TAPS  = 131;   // Phase A 3 kHz anti-alias (odd -> partial group)
    localparam int DECIM_R   = 10;    // Phase A: 30 kHz / 10 = 3 kHz
    localparam int K_PACKETS = 160;
    localparam logic [7:0] LANE_MASK = 8'b0010_0101;
    localparam int LFP_BRAM_AW = 14;
    localparam int MAXW = 8192;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    logic        dsp_sample_valid;
    logic [127:0] dsp_sample_data;
    logic [5:0]  dsp_sample_slot;
    logic        dsp_packet_tick;
    logic [31:0] lfp_cfg, lfp_coef, lfp_strobe;
    logic                   bram_clk, bram_rst, bram_en;
    logic [LFP_BRAM_AW-1:0] bram_addr;
    logic [31:0]            bram_din;
    logic [3:0]             bram_we;
    logic [LFP_BRAM_AW-1:0] lfp_wr_addr;
    logic                   lfp_overrun;

    lfp_dsp_block #(.LFP_BRAM_AW(LFP_BRAM_AW)) dut (
        .clk(clk), .rstn(rstn),
        .dsp_sample_valid(dsp_sample_valid), .dsp_sample_data(dsp_sample_data),
        .dsp_sample_slot(dsp_sample_slot), .dsp_packet_tick(dsp_packet_tick),
        .lfp_cfg(lfp_cfg), .lfp_coef(lfp_coef), .lfp_strobe(lfp_strobe),
        .bram_clk(bram_clk), .bram_rst(bram_rst), .bram_addr(bram_addr),
        .bram_din(bram_din), .bram_dout(32'h0), .bram_en(bram_en), .bram_we(bram_we),
        .lfp_wr_addr(lfp_wr_addr), .lfp_overrun(lfp_overrun)
    );

    logic [COEF_W-1:0] coefs   [0:NUM_TAPS-1];
    logic [127:0]      samples [0:K_PACKETS*N_CYCLES-1];
    logic [31:0]       exp_w   [0:MAXW-1];

    logic [31:0] got_w [0:MAXW-1];
    logic [LFP_BRAM_AW-1:0] got_a [0:MAXW-1];
    int n_got = 0;
    always @(posedge clk) if (rstn && bram_we != 0 && n_got < MAXW) begin
        got_w[n_got] = bram_din; got_a[n_got] = bram_addr; n_got++;
    end

    int n_exp = 0, errors = 0;
    logic tog;

    initial begin
        $readmemh("lfp_blk_coefs.hex",     coefs);
        $readmemh("lfp_blk_samples.hex",   samples);
        $readmemh("lfp_blk_exp_words.hex", exp_w);
        // count expected words from the file (stop at first x)
        for (int i = 0; i < MAXW; i++)
            if (exp_w[i] !== 32'hxxxxxxxx) n_exp = i + 1;

        dsp_sample_valid = 0; dsp_sample_data = 0; dsp_sample_slot = 0; dsp_packet_tick = 0;
        lfp_cfg = 0; lfp_coef = 0; lfp_strobe = 0; tog = 0;
        repeat (5) @(posedge clk); rstn = 1; @(posedge clk);

        // ---- load coefficients (clear ptr, then toggle one write per coef) ----
        @(negedge clk); lfp_strobe = 32'h2;     // coef_ptr_clr = 1
        @(negedge clk); lfp_strobe = 32'h0;
        for (int j = 0; j < NUM_TAPS; j++) begin
            @(negedge clk); lfp_coef = {14'd0, coefs[j]};
            tog = ~tog; lfp_strobe = {31'd0, tog};
            @(negedge clk);
        end

        // ---- enable: en, lane_mask, decim_R, num_taps ----
        @(negedge clk);
        lfp_cfg = (NUM_TAPS << 24) | (DECIM_R << 16) | ({24'd0, LANE_MASK} << 8) | 32'd1;
        @(posedge clk);

        // ---- stream packets (all 35 cycle slots) ----
        for (int p = 0; p < K_PACKETS; p++) begin
            for (int c = 0; c < N_CYCLES; c++) begin
                @(negedge clk);
                dsp_sample_valid = 1; dsp_sample_slot = c[5:0];
                dsp_sample_data  = samples[p*N_CYCLES + c];
                @(negedge clk); dsp_sample_valid = 0;
                repeat (7) @(negedge clk);
            end
            @(negedge clk); dsp_packet_tick = 1;
            @(negedge clk); dsp_packet_tick = 0;
            // Pad the inter-packet gap to cover the worst-case compute pass
            // (real HW has ~2800 clk/packet*R; this TB streams far faster).
            repeat (1200) @(negedge clk);
        end
        repeat (12000) @(posedge clk);

        // ---- compare ----
        if (n_got != n_exp) begin
            $display("COUNT MISMATCH: got %0d words, expected %0d", n_got, n_exp);
            errors++;
        end
        for (int i = 0; i < n_exp; i++) begin
            if (got_w[i] !== exp_w[i] || got_a[i] !== (i*4)) begin
                if (errors < 20)
                    $display("MISMATCH word %0d: got %08h @%0d | exp %08h @%0d",
                             i, got_w[i], got_a[i], exp_w[i], i*4);
                errors++;
            end
        end
        if (lfp_overrun) begin $display("LFP_OVERRUN asserted"); errors++; end

        if (errors == 0) $display("RESULT: PASS (%0d BRAM words checked)", n_exp);
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin #80ms; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
