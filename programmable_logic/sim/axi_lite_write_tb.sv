// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// axi_lite_write_tb.sv
//
// Self-checking xsim testbench for the axi_lite_registers AXI-Lite write
// handshake. Motivated by a hardware wedge: the historical FSM toggles
// awready/wready independently every cycle while the corresponding valid is
// held, and only accepts when BOTH ready/valid pairs coincide -- if the
// interconnect asserts AWVALID and WVALID one cycle apart, the two readys
// oscillate in anti-phase and the write NEVER completes (CPU bus hang).
//
// Checks, for every skew in {AW first by 2,1,0, W first by 1,2}:
//   A. the write completes (BVALID within a timeout)
//   B. the register actually took the data
//   C. back-to-back writes and read-after-write still work
//   D. reads complete under held ARVALID
//
// Run: bash programmable_logic/sim/run_axi_write_tb.sh ("RESULT: PASS")

`timescale 1ns/1ps

module axi_lite_write_tb;

localparam int N_CTRL = 25;
localparam int N_STATUS = 13;

logic aclk = 0, aresetn = 0;
logic pl_clk = 0, pl_rstn = 0;
always #3.8 aclk = ~aclk;     // ~131 MHz
always #5.95 pl_clk = ~pl_clk; // ~84 MHz

logic [31:0] awaddr = 0, wdata = 0, araddr = 0;
logic awvalid = 0, wvalid = 0, arvalid = 0;
logic [3:0] wstrb = 4'hF;
logic bready = 1, rready = 1;
wire awready, wready, bvalid, arready, rvalid;
wire [1:0] bresp, rresp;
wire [31:0] rdata;
wire [32*N_CTRL-1:0] ctrl_regs_pl;
logic [32*N_STATUS-1:0] status_regs_pl = '0;

axi_lite_registers #(.N_CTRL(N_CTRL), .N_STATUS(N_STATUS)) dut (
    .s_axi_aclk(aclk), .s_axi_aresetn(aresetn),
    .pl_clk(pl_clk), .pl_rstn(pl_rstn),
    .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
    .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
    .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .ctrl_regs_pl(ctrl_regs_pl), .status_regs_pl(status_regs_pl)
);

int n_checks = 0, n_errors = 0;

task automatic err(input string msg);
    n_errors++;
    $display("ERROR @%0t: %s", $time, msg);
endtask

// AXI write with a configurable AW-vs-W assertion skew.
//   skew > 0: AWVALID leads WVALID by `skew` cycles
//   skew < 0: WVALID leads AWVALID
// Both valids are held until their handshake completes (AXI-compliant).
task automatic axi_write(input int reg_idx, input logic [31:0] data,
                         input int skew, input string tag);
    int aw_done = 0, w_done = 0, b_done = 0, cycles = 0;
    @(negedge aclk);
    if (skew >= 0) begin awaddr = reg_idx*4; awvalid = 1; end
    else           begin wdata = data;      wvalid = 1;  end
    repeat ((skew >= 0) ? skew : -skew) @(negedge aclk);
    if (skew >= 0) begin wdata = data;      wvalid = 1;  end
    else           begin awaddr = reg_idx*4; awvalid = 1; end

    // drive until both channels accepted and response seen, or timeout
    while ((!aw_done || !w_done || !b_done) && cycles < 200) begin
        @(posedge aclk);
        if (awvalid && awready) aw_done = 1;
        if (wvalid && wready)   w_done = 1;
        if (bvalid && bready)   b_done = 1;
        @(negedge aclk);
        if (aw_done) awvalid = 0;
        if (w_done)  wvalid = 0;
        cycles++;
    end
    awvalid = 0; wvalid = 0;

    n_checks++;
    if (!aw_done || !w_done || !b_done)
        err($sformatf("%s: WRITE DEADLOCK (skew=%0d) aw=%0d w=%0d b=%0d after %0d cycles",
                      tag, skew, aw_done, w_done, b_done, cycles));
endtask

task automatic axi_read(input int reg_idx, output logic [31:0] data,
                        input string tag);
    int ar_done = 0, r_done = 0, cycles = 0;
    @(negedge aclk);
    araddr = reg_idx*4; arvalid = 1;
    while ((!ar_done || !r_done) && cycles < 200) begin
        @(posedge aclk);
        if (arvalid && arready) ar_done = 1;
        if (rvalid && rready) begin r_done = 1; data = rdata; end
        @(negedge aclk);
        if (ar_done) arvalid = 0;
        cycles++;
    end
    arvalid = 0;
    n_checks++;
    if (!ar_done || !r_done)
        err($sformatf("%s: READ DEADLOCK after %0d cycles", tag, cycles));
endtask

initial begin
    logic [31:0] rd;
    repeat (5) @(negedge aclk);
    aresetn = 1; pl_rstn = 1;
    repeat (5) @(negedge aclk);

    // A/B: write with every skew, then read back
    begin
        int skews [5] = '{-2, -1, 0, 1, 2};
        for (int k = 0; k < 5; k++) begin
            logic [31:0] pattern;
            pattern = 32'hA5A50000 + k;
            axi_write(22, pattern, skews[k], $sformatf("skew%0d", skews[k]));
            axi_read(22, rd, $sformatf("rb%0d", skews[k]));
            n_checks++;
            if (rd !== pattern)
                err($sformatf("readback skew=%0d got=%08h exp=%08h", skews[k], rd, pattern));
        end
    end

    // C: back-to-back interleaved read/write traffic (the aux upload pattern)
    for (int i = 0; i < 16; i++) begin
        axi_read(24, rd, "burst-rd24");
        axi_write(23, 32'h1000_0000 + i, (i % 3) - 1, "burst-wr23");  // skew -1,0,1
        axi_write(24, rd ^ 1, (i % 2), "burst-wr24");
    end
    axi_read(23, rd, "burst-final");
    n_checks++;
    if (rd !== 32'h1000_000F) err($sformatf("burst final got=%08h", rd));

    $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
    if (n_errors == 0) $display("RESULT: PASS");
    else               $display("RESULT: FAIL");
    $finish;
end

initial begin
    #200_000;
    $display("ERROR: watchdog timeout");
    $display("RESULT: FAIL");
    $finish;
end

endmodule
