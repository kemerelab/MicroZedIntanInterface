
// Verilog wrapper for Vivado compatibility
module simple_dual_port_bram_wrapper #(
    parameter integer ADDR_WIDTH = 16,
    parameter integer DATA_WIDTH = 32,
    parameter integer DEPTH = 16384
)(
    // Port A - Write Only (data generator)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *)
    input  wire                    porta_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *)
    input  wire                    porta_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *)
    input  wire                    porta_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *)
    input  wire [3:0]              porta_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *)
    input  wire [ADDR_WIDTH-1:0]   porta_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *)
    input  wire [DATA_WIDTH-1:0]   porta_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *)
    output wire [DATA_WIDTH-1:0]   porta_dout,
    
    // Port B - Read Only (AXI interface)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB CLK" *)
    input  wire                    portb_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB RST" *)
    input  wire                    portb_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB EN" *)
    input  wire                    portb_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB WE" *)
    input  wire [3:0]              portb_we,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB ADDR" *)
    input  wire [ADDR_WIDTH-1:0]   portb_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB DIN" *)
    input  wire [DATA_WIDTH-1:0]   portb_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB DOUT" *)
    output wire [DATA_WIDTH-1:0]   portb_dout
);

    // Instantiate the SystemVerilog BRAM
    simple_dual_port_bram #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH), 
        .DEPTH(DEPTH)
    ) bram_inst (
        .porta_clk(porta_clk),
        .porta_rst(porta_rst),
        .porta_en(porta_en),
        .porta_we(porta_we),
        .porta_addr(porta_addr),
        .porta_din(porta_din),
        .porta_dout(porta_dout),
        
        .portb_clk(portb_clk),
        .portb_rst(portb_rst),
        .portb_en(portb_en),
        .portb_we(portb_we),
        .portb_addr(portb_addr),
        .portb_din(portb_din),
        .portb_dout(portb_dout)
    );

endmodule