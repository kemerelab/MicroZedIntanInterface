`timescale 1ns/1ps
//============================================================================
// axi_read_bram_ctrl -- read-only AXI4 slave -> BRAM (1-cycle read latency)
//
// A from-scratch AXI4 read controller for the capture BRAM, written so RVALID
// is asserted ONLY when read data is genuinely valid: a small FIFO absorbs the
// 1-cycle BRAM latency and any RREADY backpressure, so no beat is dropped or
// returned early (no "address echo / pipeline underrun") even for long
// back-to-back INCR bursts.
//
// Purpose: test in simulation whether a *clean* controller handles the exact
// case that corrupts on hardware -- i.e. whether the on-hardware 0xFF corruption
// could be a controller-logic bug (fixable with a custom axi_bram_ctrl) or is
// instead the PS7 M_AXI_GP *master* (which no slave controller can fix). Write
// channels are omitted: the capture region is read-only.
//============================================================================
module axi_read_bram_ctrl #(
  parameter int ID_WIDTH   = 4,
  parameter int ADDR_WIDTH = 16,   // byte address into the BRAM
  parameter int DATA_WIDTH = 32
)(
  input  logic                  aclk,
  input  logic                  aresetn,
  // AR channel
  input  logic [ID_WIDTH-1:0]   s_arid,
  input  logic [ADDR_WIDTH-1:0] s_araddr,
  input  logic [7:0]            s_arlen,    // beats - 1
  input  logic                  s_arvalid,
  output logic                  s_arready,
  // R channel
  output logic [ID_WIDTH-1:0]   s_rid,
  output logic [DATA_WIDTH-1:0] s_rdata,
  output logic [1:0]            s_rresp,
  output logic                  s_rlast,
  output logic                  s_rvalid,
  input  logic                  s_rready,
  // BRAM read port (1-cycle registered read latency)
  output logic                  bram_en,
  output logic [ADDR_WIDTH-1:0] bram_addr,
  input  logic [DATA_WIDTH-1:0] bram_rdata
);
  localparam int WLSB  = $clog2(DATA_WIDTH/8);   // 2 for 32-bit
  localparam int DEPTH = 8;
  localparam int CW    = $clog2(DEPTH);

  logic                       busy;    // issuing beats of the current burst
  logic [ADDR_WIDTH-WLSB-1:0] waddr;   // current beat word address
  logic [8:0]                 togo;    // beats remaining to ISSUE (arlen+1 .. 0)
  logic [ID_WIDTH-1:0]        id_q;

  // Output FIFO: {last, data}
  logic [DATA_WIDTH:0]        fifo [DEPTH];
  logic [CW:0]                cnt;     // occupancy 0..DEPTH
  logic [CW-1:0]              wp, rp;

  logic                       iss_d;   // a BRAM read was issued last cycle
  logic                       last_d;  // ...and it was the burst's last beat

  // Reserve room for the in-flight (iss_d) beat so we never overflow.
  wire full_block = (cnt + iss_d) >= DEPTH[CW:0];
  wire can_issue  = busy && (togo != 0) && !full_block;

  // Accept a new burst only when fully idle (single outstanding burst).
  assign s_arready = !busy && (cnt == 0);
  assign bram_en   = can_issue;
  assign bram_addr = {waddr, {WLSB{1'b0}}};

  assign s_rvalid = (cnt != 0);
  assign s_rdata  = fifo[rp][DATA_WIDTH-1:0];
  assign s_rlast  = fifo[rp][DATA_WIDTH];
  assign s_rid    = id_q;
  assign s_rresp  = 2'b00;             // OKAY

  wire do_push = iss_d;
  wire do_pop  = s_rvalid && s_rready;

  always_ff @(posedge aclk) begin
    if (!aresetn) begin
      busy <= 1'b0; togo <= '0; waddr <= '0; id_q <= '0;
      cnt <= '0; wp <= '0; rp <= '0; iss_d <= 1'b0; last_d <= 1'b0;
    end else begin
      // Accept a new burst
      if (s_arready && s_arvalid) begin
        busy  <= 1'b1;
        waddr <= s_araddr[ADDR_WIDTH-1:WLSB];
        togo  <= s_arlen + 1'b1;
        id_q  <= s_arid;
      end
      // Issue pipeline (1-cycle BRAM latency)
      iss_d  <= can_issue;
      last_d <= (togo == 9'd1);
      if (can_issue) begin
        waddr <= waddr + 1'b1;
        togo  <= togo - 1'b1;
        if (togo == 9'd1) busy <= 1'b0;   // last beat issued
      end
      // Capture returned data into the FIFO
      if (do_push) begin
        fifo[wp] <= {last_d, bram_rdata};
        wp <= wp + 1'b1;
      end
      if (do_pop) rp <= rp + 1'b1;
      cnt <= cnt + (do_push ? 1 : 0) - (do_pop ? 1 : 0);
    end
  end
endmodule
