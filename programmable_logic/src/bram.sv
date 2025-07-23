// File: simple_dual_port_bram.sv
// Simple dual-port BRAM: Port A write-only (data generator), Port B read-only (AXI)
// Pin-compatible with your existing BRAM interface

module simple_dual_port_bram #(
    parameter int ADDR_WIDTH = 16,    // Byte address width
    parameter int DATA_WIDTH = 32,    // Data width
    parameter int DEPTH = 16384       // Memory depth (64KB / 4 bytes = 16K words)
)(
    // Port A - Write Only (for data generator)
    input  logic                    porta_clk,
    input  logic                    porta_rst,
    input  logic                    porta_en,
    input  logic [3:0]              porta_we,
    input  logic [ADDR_WIDTH-1:0]   porta_addr,   // Byte address
    input  logic [DATA_WIDTH-1:0]   porta_din,
    output logic [DATA_WIDTH-1:0]   porta_dout,   // Not used (write-only)
    
    // Port B - Read Only (for AXI interface)
    input  logic                    portb_clk,
    input  logic                    portb_rst,
    input  logic                    portb_en,
    input  logic [3:0]              portb_we,     // Ignored (read-only)
    input  logic [ADDR_WIDTH-1:0]   portb_addr,   // Byte address
    input  logic [DATA_WIDTH-1:0]   portb_din,    // Ignored (read-only)
    output logic [DATA_WIDTH-1:0]   portb_dout
);

// Memory array - 32-bit words
logic [DATA_WIDTH-1:0] memory [0:DEPTH-1];

// Convert byte addresses to word addresses
logic [$clog2(DEPTH)-1:0] porta_word_addr = porta_addr[ADDR_WIDTH-1:2];
logic [$clog2(DEPTH)-1:0] portb_word_addr = portb_addr[ADDR_WIDTH-1:2];

// Port A - Write Only (for data generator writes)  
always_ff @(posedge porta_clk) begin
    if (!porta_rst) begin  // Active high reset
        if (porta_en && |porta_we) begin  // Write if any write enable bits set
            // Simple write
            memory[porta_word_addr] <= porta_din;
        end
    end
end

// Port B - Read Only (for AXI reads)
always_ff @(posedge portb_clk) begin
    if (portb_rst) begin
        portb_dout <= '0;
    end else if (portb_en) begin
        // Simple read with 1-cycle latency
        portb_dout <= memory[portb_word_addr];
    end
end

// Tie off unused Port A output
assign porta_dout = '0;

// Initialize memory for testing (optional)
initial begin
    for (int i = 0; i < DEPTH; i++) begin
        memory[i] = i * 4;  // Each word contains its byte address
    end
end

endmodule
