// File: bram_axi_lite_wrapper.v  
// Updated Verilog wrapper for 32-bit BRAM Port B
// This file handles all the Vivado-specific interface declarations


module bram_axi_lite_wrapper #(
    parameter integer BRAM_ADDR_WIDTH = 16,  // Byte address width
    parameter integer BRAM_DATA_WIDTH = 32   // 32-bit for Port B
)(
    // Clock and Reset with Vivado attributes
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 S_AXI_CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET S_AXI_ARESETN" *)
    input  wire        s_axi_aclk,
    
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 S_AXI_RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        s_axi_aresetn,
    
    // AXI Lite Slave Interface with correct Vivado attributes
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    (* X_INTERFACE_MODE = "slave" *)
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI4LITE, MAX_BURST_LENGTH 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, SUPPORTS_NARROW_BURST 0, READ_WRITE_MODE READ_WRITE, BUSER_WIDTH 0, RUSER_WIDTH 0, WUSER_WIDTH 0, ARUSER_WIDTH 0, AWUSER_WIDTH 0, ADDR_WIDTH 16, ID_WIDTH 0, FREQ_HZ 100000000, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 1, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1" *)    
    input  wire [15:0] s_axi_awaddr,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
    input  wire [2:0]  s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire        s_axi_awready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire        s_axi_wready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output wire [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output wire        s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire        s_axi_bready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [15:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
    input  wire [2:0]  s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire        s_axi_arready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output wire [31:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output wire        s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire        s_axi_rready,
    
    // BRAM Interface (32-bit Port B) with Vivado attributes
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB CLK" *)
    output wire            bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB RST" *)  
    output wire            bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB EN" *)
    output wire            bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB WE" *)
    output wire [3:0]      bram_we,   // 4 bytes for 32-bit
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB ADDR" *)
    output wire [BRAM_ADDR_WIDTH-1:0] bram_addr,  // Byte address
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB DIN" *)
    output wire [BRAM_DATA_WIDTH-1:0] bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB DOUT" *)
    input  wire [BRAM_DATA_WIDTH-1:0] bram_dout
);

    // Instantiate the SystemVerilog core
    bram_axi_lite_core #(
        .BRAM_ADDR_WIDTH(BRAM_ADDR_WIDTH),
        .BRAM_DATA_WIDTH(BRAM_DATA_WIDTH),  // 32-bit
        .AXI_DATA_WIDTH(32),
        .AXI_ADDR_WIDTH(16)
    ) core_inst (
        // Clock and Reset
        .clk(s_axi_aclk),
        .resetn(s_axi_aresetn),
        
        // AXI Lite Write Address Channel
        .awaddr(s_axi_awaddr),
        .awprot(s_axi_awprot),
        .awvalid(s_axi_awvalid),
        .awready(s_axi_awready),
        
        // AXI Lite Write Data Channel
        .wdata(s_axi_wdata),
        .wstrb(s_axi_wstrb),
        .wvalid(s_axi_wvalid),
        .wready(s_axi_wready),
        
        // AXI Lite Write Response Channel
        .bresp(s_axi_bresp),
        .bvalid(s_axi_bvalid),
        .bready(s_axi_bready),
        
        // AXI Lite Read Address Channel
        .araddr(s_axi_araddr),
        .arprot(s_axi_arprot),
        .arvalid(s_axi_arvalid),
        .arready(s_axi_arready),
        
        // AXI Lite Read Data Channel
        .rdata(s_axi_rdata),
        .rresp(s_axi_rresp),
        .rvalid(s_axi_rvalid),
        .rready(s_axi_rready),
        
        // BRAM Interface
        .bram_clk(bram_clk),
        .bram_rst(bram_rst),
        .bram_en(bram_en),
        .bram_we(bram_we),
        .bram_addr(bram_addr),
        .bram_din(bram_din),
        .bram_dout(bram_dout)
    );

endmodule