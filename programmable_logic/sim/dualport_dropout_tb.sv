// dualport_dropout_tb.sv
//
// Reproduces the Phase-3 "neural channel dropout" report: in debug mode with
// channel_enable = 0xFF, specific SPI cycles' data come out stuck/wrong. Runs
// the FULL datapath (data_generator = core + fifo_bram_interface) and captures
// the BRAM write stream.
//
// Detection without a sine reference: in debug mode dummy_data_index advances
// by 1 every packet, so EVERY data word must differ between two consecutive
// steady-state packets. Any data-region BRAM word that is IDENTICAL across two
// packets is "stuck" -> the bug. Header words (magic, breadcrumbs) may legitimately
// repeat, so only the data region (offset >= 10) is judged.
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

localparam int PKT = 150;          // 10 header + 140 data words at 0xFF
localparam logic [31:0] MAGIC_LO = 32'hDEADBEEF;

// Reference sine LUT, same formula as data_generator_core's initial block.
logic [15:0] ref_lut [0:511];
initial begin
    for (int i = 0; i < 512; i++) begin
        real angle = 2.0 * 3.14159265359 * i / 512.0;
        ref_lut[i] = $rtoi(32767.0/16.0 * $sin(angle) + 32767.0);
    end
end

// Expected 4 packed BRAM words for cycle `c` at debug index `ddi`.
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
    // channel_enable = 0xFF at CTRL_REG_2[15:8]
    ctrl[1*32 +: 32] = 32'd0;
    ctrl[2*32 +: 32] = 32'h0000_FF00;
    ctrl[0*32 +: 32] = 32'h0000_0009;

    // run long enough for several full packets (each ~2800 clocks)
    repeat (4*2900) @(negedge clk);
    ctrl[0*32 +: 32] = 32'h0;
    repeat (200) @(negedge clk);

    // ---- find two consecutive steady-state packets (start = MAGIC_LO) ----
    begin
        int starts [$];
        foreach (wseq[i]) if (wseq[i] == MAGIC_LO) starts.push_back(i);
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
                // compare data region (word offset 10..149) across the two packets
                int stuck = 0;
                int first_stuck = -1;
                int cyc, sub;
                for (int w = 10; w < PKT; w++) begin
                    n_checks++;
                    if (wseq[a+w] === wseq[b+w]) begin
                        stuck++;
                        if (first_stuck < 0) first_stuck = w;
                        // map word offset -> cycle: data starts at offset 10, 4 words/cycle
                        cyc = (w - 10) / 4;
                        sub = (w - 10) % 4;
                        if (stuck <= 24)
                            $display("STUCK word[%0d] (cycle %0d chunk %0d): 0x%08h", w, cyc, sub, wseq[a+w]);
                    end
                end
                n_checks++;
                if (stuck > 0)
                    err($sformatf("%0d data words STUCK across packets (first at word %0d)", stuck, first_stuck));
                else
                    $display("no stuck data words: all %0d data words advance between packets", PKT-10);

                // ---- VALUE reference: brute-force the packet's debug index, then
                //      compare EVERY data word (catches wrong-but-varying corruption) ----
                begin
                    int best_ddi = -1, best_mism = 99999;
                    for (int ddi = 0; ddi < 512; ddi++) begin
                        int mism = 0;
                        for (int c = 0; c < 35; c++) begin
                            logic [31:0] ew [0:3];
                            ref_cycle_words(c, ddi, ew);
                            for (int j = 0; j < 4; j++)
                                if (wseq[a + 10 + 4*c + j] !== ew[j]) mism++;
                        end
                        if (mism < best_mism) begin best_mism = mism; best_ddi = ddi; end
                    end
                    $display("best debug-index match: ddi=%0d with %0d/%0d data words mismatched",
                             best_ddi, best_mism, PKT-10);
                    n_checks++;
                    if (best_mism != 0) begin
                        err($sformatf("%0d data words DIFFER from reference (RTL packing/debug bug)", best_mism));
                        // show which cycles are wrong
                        for (int c = 0; c < 35; c++) begin
                            logic [31:0] ew [0:3];
                            int cm = 0;
                            ref_cycle_words(c, best_ddi, ew);
                            for (int j = 0; j < 4; j++)
                                if (wseq[a + 10 + 4*c + j] !== ew[j]) cm++;
                            if (cm > 0)
                                $display("  cycle %0d: %0d/4 words wrong  got[%08h %08h %08h %08h] exp[%08h %08h %08h %08h]",
                                    c, cm,
                                    wseq[a+10+4*c+0], wseq[a+10+4*c+1], wseq[a+10+4*c+2], wseq[a+10+4*c+3],
                                    ew[0], ew[1], ew[2], ew[3]);
                        end
                    end else
                        $display("RTL data matches the reference EXACTLY for all 35 cycles -> packing is correct");
                end
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
