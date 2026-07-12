`timescale 1ns/1ps
//============================================================================
// tb_axi_read_bram_ctrl -- stress the clean read controller with the exact
// scenario that corrupts on hardware: long and back-to-back INCR read bursts
// with random RREADY backpressure, against a 1-cycle BRAM filled with a NON-i*4
// pattern (data[i] = i ^ 0xA5A5A5A5).
//
// The non-i*4 fill is the point: on hardware the corrupt beat's value equals the
// byte-address being read, which the real BRAM's i*4 init masks. Here, an
// address-echo (or any wrong beat) mismatches the i^A5A5A5A5 reference at once.
//
// Uses an inline behavioural 1-cycle memory (a faithful model of bram.sv's port-B
// read: registered, enable-gated) rather than instantiating bram.sv, whose
// module-scope `logic waddr = addr;` is a one-time init under xsim (a sim/synth
// quirk of that file; it synthesises correctly). The DUT under test is the
// controller, which is identical either way.
//
// PASS => a clean controller returns every beat correctly under the failing
//         stimulus => the on-hardware bug is NOT controller read-logic; it is the
//         PS7 M_AXI_GP master (a custom axi_bram_ctrl would not fix it).
//============================================================================
module tb_axi_read_bram_ctrl;
  localparam int ADDR_WIDTH = 16;
  localparam int DATA_WIDTH = 32;
  localparam int ID_WIDTH   = 4;
  localparam int MEMW       = 1024;

  logic aclk = 0, aresetn = 0;
  always #5 aclk = ~aclk;          // 100 MHz

  logic [ID_WIDTH-1:0]   arid;
  logic [ADDR_WIDTH-1:0] araddr;
  logic [7:0]            arlen;
  logic                  arvalid, arready;
  logic [ID_WIDTH-1:0]   rid;
  logic [DATA_WIDTH-1:0] rdata;
  logic [1:0]            rresp;
  logic                  rlast, rvalid, rready;

  logic                  bram_en;
  logic [ADDR_WIDTH-1:0] bram_addr;
  logic [DATA_WIDTH-1:0] bram_rdata, bram_rdata_r;

  int errors = 0, beats_checked = 0, rlast_count = 0, bursts = 0;
  bit [DATA_WIDTH-1:0] exp_q[$];   // expected data, in R order
  bit                  last_q[$];  // expected rlast per beat

  function automatic [DATA_WIDTH-1:0] pat(input int word_idx);
    pat = word_idx ^ 32'hA5A5A5A5;
  endfunction

  // ---- inline 1-cycle BRAM (faithful model of bram.sv port B) ----
  logic [DATA_WIDTH-1:0] mem [0:MEMW-1];
  initial for (int i = 0; i < MEMW; i++) mem[i] = pat(i);   // non-i*4 preload
  always_ff @(posedge aclk) begin
    if (!aresetn)      bram_rdata_r <= '0;
    else if (bram_en)  bram_rdata_r <= mem[bram_addr >> 2];
  end
  assign bram_rdata = bram_rdata_r;

  // ---- DUT ----
  axi_read_bram_ctrl #(.ID_WIDTH(ID_WIDTH), .ADDR_WIDTH(ADDR_WIDTH),
                       .DATA_WIDTH(DATA_WIDTH)) dut (
    .aclk(aclk), .aresetn(aresetn),
    .s_arid(arid), .s_araddr(araddr), .s_arlen(arlen),
    .s_arvalid(arvalid), .s_arready(arready),
    .s_rid(rid), .s_rdata(rdata), .s_rresp(rresp), .s_rlast(rlast),
    .s_rvalid(rvalid), .s_rready(rready),
    .bram_en(bram_en), .bram_addr(bram_addr), .bram_rdata(bram_rdata)
  );

  // ---- random RREADY backpressure ----
  always_ff @(posedge aclk) rready <= (aresetn && ($urandom_range(0,3) != 0));

  // ---- scoreboard: sample at the clock edge (race-free pre-edge read) ----
  always_ff @(posedge aclk) begin
    if (aresetn && rvalid && rready) begin
      bit [DATA_WIDTH-1:0] e;
      bit                  el;
      if (exp_q.size() == 0) begin
        errors++;
        if (errors <= 12) $display("  UNEXPECTED beat @checked=%0d: %08x (addr=%0d)",
                                   beats_checked, rdata, rdata ^ 32'hA5A5A5A5);
      end else begin
        e  = exp_q.pop_front();
        el = last_q.pop_front();
        if (rdata !== e) begin
          errors++;
          if (errors <= 12)
            $display("  MISMATCH @checked=%0d: got %08x exp %08x", beats_checked, rdata, e);
        end
        if (rlast !== el) begin
          errors++;
          if (errors <= 12)
            $display("  RLAST mismatch @checked=%0d exp_addr=%0d: got %0b exp %0b (rid=%0d)",
                     beats_checked, e ^ 32'hA5A5A5A5, rlast, el, rid);
        end
        if (rlast) rlast_count++;
        beats_checked++;
      end
    end
  end

  // ---- issue one INCR burst and enqueue its expected beats ----
  task automatic read_burst(input int word_addr, input int len, input [3:0] id);
    for (int b = 0; b <= len; b++) begin
      exp_q.push_back(pat(word_addr + b));
      last_q.push_back(b == len);
    end
    bursts++;
    @(posedge aclk);
    arid <= id; araddr <= word_addr << 2; arlen <= len[7:0]; arvalid <= 1'b1;
    @(posedge aclk iff arready);
    arvalid <= 1'b0;   // NBA: DUT samples arvalid=1 at this edge, then it clears -> exactly one latch
  endtask

  initial begin
    arvalid <= 0; arid <= 0; araddr <= 0; arlen <= 0;   // all-NBA AR drive; rready by always_ff
    aresetn = 0;
    repeat (6) @(posedge aclk);
    aresetn = 1;
    @(posedge aclk);

    $display("=== TEST 1: one long 150-beat INCR burst (mimics 0xFF packet read) ===");
    read_burst(0, 149, 4'h1);

    $display("=== TEST 2: 10 back-to-back 16-beat bursts (mimics AXI3 GP chunks) ===");
    for (int k = 0; k < 10; k++) read_burst(k*16, 15, k[3:0]);

    $display("=== TEST 3: assorted lengths back-to-back ===");
    read_burst(0, 11, 4'h7);     // 12 beats (the dump_bram corruption threshold)
    read_burst(12, 0, 4'h8);     // single beat
    read_burst(13, 199, 4'h9);   // 200 beats

    // drain (poll each clock; queue methods don't reliably wake `wait` in xsim)
    while (exp_q.size() != 0) @(posedge aclk);
    repeat (5) @(posedge aclk);

    $display("------------------------------------------------------------");
    $display("bursts=%0d beats_checked=%0d rlast_count=%0d errors=%0d",
             bursts, beats_checked, rlast_count, errors);
    if (errors == 0 && rlast_count == bursts)
      $display("RESULT: PASS -- clean controller returns every beat correctly under backpressure");
    else
      $display("RESULT: FAIL -- errors=%0d (rlast_count=%0d vs bursts=%0d)", errors, rlast_count, bursts);
    $display("------------------------------------------------------------");
    $finish;
  end

  initial begin #2_000_000; $display("RESULT: FAIL -- timeout"); $finish; end
endmodule
