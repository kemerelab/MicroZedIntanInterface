// playback_bram_wrapper.v -- 128 KB dual-port BRAM for synthetic-data playback.
//
// Mirror image of simple_dual_port_bram_wrapper: here Port A is the PL READ side
// (data_generator reads the waveform) and Port B is the PS READ/WRITE side (an
// axi_bram_ctrl @ 0x8C000000 loads the waveform). 32K x 32-bit = 128 KB = 64K
// samples = ~2.13 s @ 30 ksps, 2x16-bit samples per word. (128 KB, not 256 KB,
// so RAMB36 fits alongside capture/LFP/STFT on the -1 part.) Port B carries the
// BRAM_CTRL MEM_SIZE metadata (131072).
module playback_bram_wrapper #(
    parameter integer ADDR_WIDTH = 17,    // 128 KB byte address
    parameter integer DATA_WIDTH = 32,
    parameter integer DEPTH      = 32768  // 32K words
)(
    // Port A - PL READ side (data generator)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *)
    input  wire                    porta_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *)
    input  wire                    porta_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *)
    input  wire                    porta_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *)
    input  wire [3:0]              porta_we,    // ignored (read-only)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *)
    input  wire [ADDR_WIDTH-1:0]   porta_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *)
    input  wire [DATA_WIDTH-1:0]   porta_din,   // ignored (read-only)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *)
    output reg  [DATA_WIDTH-1:0]   porta_dout,

    // Port B - PS READ/WRITE side (axi_bram_ctrl loads the waveform)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL,MEM_SIZE 131072,MEM_WIDTH 32,MEM_ECC NONE,READ_WRITE_MODE READ_WRITE" *)
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
    output reg  [DATA_WIDTH-1:0]   portb_dout
);
    (* ram_style = "block" *)
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    wire [$clog2(DEPTH)-1:0] a_word = porta_addr[ADDR_WIDTH-1:2];
    wire [$clog2(DEPTH)-1:0] b_word = portb_addr[ADDR_WIDTH-1:2];

    // Port A: read-only (PL waveform fetch), 1-cycle latency
    always @(posedge porta_clk) begin
        if (porta_en) porta_dout <= mem[a_word];
    end

    // Port B: read/write (PS load via axi_bram_ctrl), byte-enabled
    always @(posedge portb_clk) begin
        if (portb_en) begin
            if (portb_we[0]) mem[b_word][7:0]   <= portb_din[7:0];
            if (portb_we[1]) mem[b_word][15:8]  <= portb_din[15:8];
            if (portb_we[2]) mem[b_word][23:16] <= portb_din[23:16];
            if (portb_we[3]) mem[b_word][31:24] <= portb_din[31:24];
            portb_dout <= mem[b_word];
        end
    end
endmodule
