// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// dualport_dropout_tb.sv
//
// Reproduces the Phase-3 "neural channel dropout" report AND verifies the
// unified packet format (docs/unified-packet-format.md): in debug mode with
// channel_enable = 0xFF, specific SPI cycles' data come out stuck/wrong. Runs
// the FULL datapath (data_generator = core + fifo_bram_interface) and captures
// the BRAM write stream.
//
// Detection without a sine reference: in debug mode dummy_data_index advances
// by 1 every packet, so EVERY data word must differ between two consecutive
// steady-state packets. Any data-region BRAM word that is IDENTICAL across two
// packets is "stuck" -> the bug. Header words may legitimately repeat, so only
// the data region (offset >= HDR) is judged.
//
// UNIFIED HEADER + NO-DATA-LOSS assertions (this branch):
//   * the 14-word header decodes: w0 MAGIC=0xCAFEBABE, w1 stream_type=1/ver=1,
//     w2/w3 timestamp, w4 per-stream SEQ, w5 AUX0=channel_enable|num_data_words<<8,
//     w6 AUX1=digital_in/aux metadata, w7 RSVD=0.
//   * the per-stream SEQ advances by exactly 1 per packet (the loss check).
//   * BROADBAND CONTENT PRESERVED: every DATA word still matches the SAME sine
//     reference as the original format (the framing changed, the data did not),
//     and the timestamp advances by exactly 1 per packet -- so re-framing lost
//     nothing.
//
// Run: bash programmable_logic/sim/run_dualport_dropout_tb.sh  ("RESULT: PASS")

`timescale 1ns/1ps

module dualport_dropout_tb;

logic clk = 0, rstn = 0;
always #5 clk = ~clk;

logic [32*25-1:0] ctrl = '0;
wire  [32*13-1:0] status;
wire [15:0] bram_addr; wire [31:0] bram_din; wire bram_en; wire [3:0] bram_we;
wire bram_clk, bram_rst;

data_generator dut (
    .clk(clk), .rstn(rstn),
    .ctrl_regs_pl(ctrl), .status_regs_pl(status),
    .digital_in(8'h00),
    .bram_clk(bram_clk), .bram_rst(bram_rst),
    .bram_addr(bram_addr), .bram_din(bram_din), .bram_dout(32'h0),
    .bram_en(bram_en), .bram_we(bram_we),
    .csn(), .sclk(), .copi(), .cipo0(1'b0), .cipo1(1'b0),
    .csn_b(), .sclk_b(), .copi_b(), .cipo2(1'b0), .cipo3(1'b0)
);

// Capture the linear BRAM write sequence (writes are sequential).
logic [31:0] wseq [$];
always @(posedge clk) if (rstn && bram_en && bram_we == 4'hF) wseq.push_back(bram_din);

int n_checks = 0, n_errors = 0;
task automatic err(input string m); n_errors++; $display("ERROR: %s", m); endtask

// Unified header geometry: 8 common-header words + 6 broadband sub-block words
// = 14 words ahead of the data. At 0xFF: 140 data words -> PKT = 154.
localparam int HDR = 14;
localparam int PKT = HDR + 140;     // 154 words at 0xFF
localparam logic [31:0] UNIFIED_MAGIC = 32'hCAFEBABE;  // header word 0
localparam logic [7:0]  STREAM_TYPE_BB = 8'd1;
localparam logic [7:0]  VERSION        = 8'd1;

// Reference sine LUT, same formula as data_generator_core's initial block.
logic [15:0] ref_lut [0:511];
initial begin
    for (int i = 0; i < 512; i++) begin
        real angle = 2.0 * 3.14159265359 * i / 512.0;
        ref_lut[i] = $rtoi(32767.0/16.0 * $sin(angle) + 32767.0);
    end
end

// Expected 4 packed BRAM words for cycle `c` at debug index `ddi`. This is the
// SAME reference the original testbench used -- the data did not change, only the
// framing -- so a match proves the broadband DATA content is preserved.
function automatic void ref_cycle_words(input int c, input int ddi, output logic [31:0] w [0:3]);
    int coff = (c >= 2) ? (c - 2) : 0;
    int bp  = (ddi + coff) & 9'h1FF;
    int bp1 = (bp + 128)   & 9'h1FF;
    logic [15:0] s0r, s0d, s1r, s1d, s2r, s2d, s3r, s3d;
    s0r = ref_lut[bp];                 s0d = ref_lut[(bp  << 1) & 9'h1FF];
    s1r = ref_lut[(bp  << 2)&9'h1FF];  s1d = ref_lut[(bp  << 3) & 9'h1FF];
    s2r = ref_lut[bp1];                s2d = ref_lut[(bp1 << 1) & 9'h1FF];
    s3r = ref_lut[(bp1 << 2)&9'h1FF];  s3d = ref_lut[(bp1 << 3) & 9'h1FF];
    w[0] = {s0d, s0r};  w[1] = {s1d, s1r};  w[2] = {s2d, s2r};  w[3] = {s3d, s3r};
endfunction

initial begin
    repeat (5) @(negedge clk);
    rstn = 1;
    repeat (5) @(negedge clk);

    // enable transmission + debug mode (reg0 bit0 + bit3), infinite loop,
    // channel_enable = 0xFF at CTRL_REG_2[23:16]
    ctrl[1*32 +: 32] = 32'd0;
    ctrl[2*32 +: 32] = 32'h00FF_0000;
    ctrl[0*32 +: 32] = 32'h0000_0009;

    // run long enough for many full packets (each ~2800 clocks)
    repeat (12*2900) @(negedge clk);
    ctrl[0*32 +: 32] = 32'h0;
    repeat (200) @(negedge clk);

    // ---- find packet starts: w0 == MAGIC and the next word is a valid
    //      broadband TYPE_VER (stream_type=1, version=1). ----
    begin
        int starts [$];
        foreach (wseq[i]) begin
            if (i + 1 < wseq.size() && wseq[i] == UNIFIED_MAGIC) begin
                logic [7:0] st  = wseq[i+1][7:0];
                logic [7:0] ver = wseq[i+1][15:8];
                if (st == STREAM_TYPE_BB && ver == VERSION)
                    starts.push_back(i);
            end
        end
        $display("captured %0d BRAM words, %0d packet starts", wseq.size(), starts.size());
        n_checks++;
        if (starts.size() < 3) begin
            err($sformatf("need >=3 packets, got %0d", starts.size()));
        end else begin
            // use the 2nd and 3rd packet (steady state); require full PKT spacing
            int a = starts[1];
            int b = starts[2];
            n_checks++;
            if (b - a != PKT)
                err($sformatf("packet spacing %0d != %0d (dropped/extra words!)", b-a, PKT));
            if (b + PKT <= wseq.size()) begin
                // ---- declarations (SV: must precede statements in this block) ----
                logic [31:0] hw [0:HDR-1];
                logic [31:0] ce_word, aux0;
                logic [15:0] ndw_field;
                int stuck = 0;
                int first_stuck = -1;
                int cyc, sub;

                // ---- header decode + field assertions on packet `a` ----
                for (int k = 0; k < HDR; k++) hw[k] = wseq[a + k];
                // w5 = AUX0 = channel_enable[7:0] | num_data_words[23:8]
                aux0       = hw[5];
                ce_word    = aux0[7:0];
                ndw_field  = aux0[23:8];
                n_checks++;
                if (ce_word != 32'h0000_00FF)
                    err($sformatf("AUX0 channel_enable=0x%02h, expected 0xFF", ce_word[7:0]));
                n_checks++;
                if (ndw_field != 16'd140)
                    err($sformatf("AUX0 num_data_words=%0d, expected 140", ndw_field));
                // w7 RSVD must be 0
                n_checks++;
                if (hw[7] !== 32'h0)
                    err($sformatf("RSVD (w7) = 0x%08h, expected 0", hw[7]));

                // ---- stuck-word check (the original dropout test), data region ----
                for (int w = HDR; w < PKT; w++) begin
                    n_checks++;
                    if (wseq[a+w] === wseq[b+w]) begin
                        stuck++;
                        if (first_stuck < 0) first_stuck = w;
                        cyc = (w - HDR) / 4;
                        sub = (w - HDR) % 4;
                        if (stuck <= 24)
                            $display("STUCK word[%0d] (cycle %0d chunk %0d): 0x%08h", w, cyc, sub, wseq[a+w]);
                    end
                end
                n_checks++;
                if (stuck > 0)
                    err($sformatf("%0d data words STUCK across packets (first at word %0d)", stuck, first_stuck));
                else
                    $display("no stuck data words: all %0d data words advance between packets", PKT-HDR);

                // ---- BROADBAND CONTENT PRESERVED: brute-force the debug index,
                //      then compare EVERY data word to the SAME sine reference
                //      the original format used. A 0-mismatch match proves the data
                //      payload is byte-identical to before (only framing changed). ----
                begin
                    int best_ddi = -1, best_mism = 99999;
                    for (int ddi = 0; ddi < 512; ddi++) begin
                        int mism = 0;
                        for (int c = 0; c < 35; c++) begin
                            logic [31:0] ew [0:3];
                            ref_cycle_words(c, ddi, ew);
                            for (int j = 0; j < 4; j++)
                                if (wseq[a + HDR + 4*c + j] !== ew[j]) mism++;
                        end
                        if (mism < best_mism) begin best_mism = mism; best_ddi = ddi; end
                    end
                    $display("best debug-index match: ddi=%0d with %0d/%0d data words mismatched",
                             best_ddi, best_mism, PKT-HDR);
                    n_checks++;
                    if (best_mism != 0) begin
                        err($sformatf("%0d data words DIFFER from reference (CONTENT-PRESERVED assertion FAILED)", best_mism));
                        for (int c = 0; c < 35; c++) begin
                            logic [31:0] ew [0:3];
                            int cm = 0;
                            ref_cycle_words(c, best_ddi, ew);
                            for (int j = 0; j < 4; j++)
                                if (wseq[a + HDR + 4*c + j] !== ew[j]) cm++;
                            if (cm > 0)
                                $display("  cycle %0d: %0d/4 words wrong  got[%08h %08h %08h %08h] exp[%08h %08h %08h %08h]",
                                    c, cm,
                                    wseq[a+HDR+4*c+0], wseq[a+HDR+4*c+1], wseq[a+HDR+4*c+2], wseq[a+HDR+4*c+3],
                                    ew[0], ew[1], ew[2], ew[3]);
                        end
                    end else
                        $display("BROADBAND CONTENT PRESERVED: all 35 cycles' data match the reference sine EXACTLY");
                end
            end

            // ---- TEMPORAL check: value-validate EVERY captured packet, verify
            //      the debug index advances by 1/packet, the per-stream SEQ
            //      advances by 1/packet (the loss check), and the timestamp
            //      advances by 1/packet. ----
            begin
                int prev_ddi = -1;
                longint prev_seq = -1;
                longint prev_ts  = -1;
                int n_full = 0;
                $display("--- per-packet value + header check (%0d packets) ---", starts.size());
                for (int pi = 0; pi < starts.size(); pi++) begin
                    int s0 = starts[pi];
                    if (s0 + PKT <= wseq.size()) begin
                        int best_ddi = -1, best_mism = 99999;
                        longint seq, ts;
                        n_full++;
                        for (int ddi = 0; ddi < 512; ddi++) begin
                            int mism = 0;
                            for (int c = 0; c < 35; c++) begin
                                logic [31:0] ew [0:3];
                                ref_cycle_words(c, ddi, ew);
                                for (int j = 0; j < 4; j++)
                                    if (wseq[s0 + HDR + 4*c + j] !== ew[j]) mism++;
                            end
                            if (mism < best_mism) begin best_mism = mism; best_ddi = ddi; end
                        end
                        n_checks++;
                        if (best_mism != 0)
                            err($sformatf("packet %0d: %0d/140 words mismatch reference (ddi=%0d)",
                                          pi, best_mism, best_ddi));
                        // header word 4 = per-stream SEQ; words 2/3 = 64-bit timestamp
                        seq = wseq[s0 + 4];
                        ts  = {wseq[s0 + 3], wseq[s0 + 2]};
                        // ddi must advance by exactly 1 (mod 512) every packet
                        if (prev_ddi >= 0) begin
                            int exp_ddi = (prev_ddi + 1) & 9'h1FF;
                            n_checks++;
                            if (best_ddi != exp_ddi)
                                err($sformatf("packet %0d: ddi=%0d, expected %0d (sequence break!)",
                                              pi, best_ddi, exp_ddi));
                        end
                        // SEQ must advance by exactly 1 every packet (no loss)
                        if (prev_seq >= 0) begin
                            n_checks++;
                            if (seq != prev_seq + 1)
                                err($sformatf("packet %0d: SEQ=%0d, expected %0d (LOSS-CHECK break!)",
                                              pi, seq, prev_seq + 1));
                        end
                        // timestamp must advance by exactly 1 every packet
                        if (prev_ts >= 0) begin
                            n_checks++;
                            if (ts != prev_ts + 1)
                                err($sformatf("packet %0d: ts=%0d, expected %0d (timestamp gap!)",
                                              pi, ts, prev_ts + 1));
                        end
                        $display("  packet %0d @ word %0d: ddi=%0d seq=%0d ts=%0d, %0d/140 mismatched",
                                 pi, s0, best_ddi, seq, ts, best_mism);
                        prev_ddi = best_ddi;
                        prev_seq = seq;
                        prev_ts  = ts;
                    end
                end
                $display("value-checked %0d full packets, all correct & ddi/seq/ts sequential = %0d",
                         n_full, (n_errors == 0));
            end
        end
    end

    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS");
    else               $display("RESULT: FAIL");
    $finish;
end

initial begin
    #5_000_000;
    $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish;
end

endmodule
