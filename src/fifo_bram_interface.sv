// File: fifo_bram_interface.sv
// Dedicated FIFO to BRAM interface with single-cycle BRAM writes
// Much simpler design thanks to single-cycle BRAM capability

module fifo_bram_interface #(
    parameter int BRAM_ADDR_WIDTH = 16,        // Byte address width
    parameter int BRAM_DATA_WIDTH = 32,        // Data width
    parameter int FIFO_DEPTH = 256,           // FIFO depth (entries)
    parameter int BRAM_DEPTH_WORDS = 16384    // BRAM depth in words (64KB / 4 = 16K words)
)(
    input  logic        clk,
    input  logic        rstn,
    
    // FIFO interface (input side)
    input  logic        fifo_write_en,
    input  logic [31:0] fifo_write_data,
    output logic        fifo_full,
    output logic [8:0]  fifo_count,
    
    // Status output for PS monitoring
    output logic [13:0] current_bram_address,
    
    // BRAM interface (output side)
    output logic [15:0] bram_addr,
    output logic [31:0] bram_din,
    output logic        bram_en,
    output logic [3:0]  bram_we,
    output logic        bram_clk,
    output logic        bram_rst
);

// Parameter validation
initial begin
    if (FIFO_DEPTH > 512) begin
        $error("FIFO_DEPTH (%d) too large - maximum 512 entries", FIFO_DEPTH);
    end
    if (BRAM_DEPTH_WORDS > (1 << (BRAM_ADDR_WIDTH - 2))) begin
        $error("BRAM_DEPTH_WORDS (%d) exceeds address space (%d words)", 
               BRAM_DEPTH_WORDS, (1 << (BRAM_ADDR_WIDTH - 2)));
    end
end

// Derived parameters
localparam int FIFO_PTR_WIDTH = $clog2(FIFO_DEPTH);
localparam int FIFO_COUNT_WIDTH = $clog2(FIFO_DEPTH + 1);
localparam int BRAM_WORD_ADDR_WIDTH = $clog2(BRAM_DEPTH_WORDS);

// FIFO storage
logic [31:0] write_fifo [0:FIFO_DEPTH-1];
logic [FIFO_PTR_WIDTH-1:0] fifo_write_ptr;
logic [FIFO_PTR_WIDTH-1:0] fifo_read_ptr;

// FIFO status signals
assign fifo_full = (fifo_count == FIFO_DEPTH);

// BRAM interface registers
logic [15:0] bram_addr_reg;
logic [31:0] bram_din_reg;
logic        bram_en_reg;
logic [3:0]  bram_we_reg;
logic [BRAM_WORD_ADDR_WIDTH-1:0] write_address;

// Connect BRAM interface
assign bram_addr = bram_addr_reg;
assign bram_din  = bram_din_reg;
assign bram_en   = bram_en_reg;
assign bram_we   = bram_we_reg;
assign bram_clk  = clk;
assign bram_rst  = ~rstn;

// Export current write address for PS monitoring
assign current_bram_address = write_address;

// Registered FIFO write signals (1 cycle latency)
logic fifo_write_en_reg;
logic [31:0] fifo_write_data_reg;

// Logical entities used for updating count register
logic fifo_write_this_cycle;
logic fifo_read_this_cycle;

// Combined FIFO and BRAM management - single always block for clean timing
always_ff @(posedge clk) begin
    if (!rstn) begin
        // FIFO write side
        fifo_write_ptr <= '0;
        fifo_write_en_reg <= 1'b0;
        fifo_write_data_reg <= 32'h0;
        
        // FIFO read side and BRAM interface
        fifo_read_ptr <= '0;
        fifo_count <= '0;
        bram_addr_reg <= 16'h0;
        bram_din_reg <= 32'h0;
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        write_address <= '0;
    end else begin
        
        // ====================================================================
        // FIFO WRITE SIDE (Data Generator → FIFO)
        // ====================================================================
        
        // Register the input signals (1 cycle delay for clean timing)
        fifo_write_en_reg <= fifo_write_en;
        fifo_write_data_reg <= fifo_write_data;
        
        // Determine if FIFO write will happen this cycle
        fifo_write_this_cycle = fifo_write_en_reg && !fifo_full;
        
        // Perform FIFO write operation
        if (fifo_write_this_cycle) begin
            write_fifo[fifo_write_ptr] <= fifo_write_data_reg;
            fifo_write_ptr <= fifo_write_ptr + 1;
        end
        
        // ====================================================================
        // FIFO READ SIDE and BRAM WRITE (FIFO → BRAM)
        // ====================================================================
        
        // Default: no BRAM write
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        
        // Determine if FIFO read/BRAM write will happen this cycle
        fifo_read_this_cycle = (fifo_count > 0);
        
        // Perform FIFO read and BRAM write operation (single cycle!)
        if (fifo_read_this_cycle) begin
            // Set up BRAM write
            bram_addr_reg <= {write_address, 2'b00};
            bram_din_reg <= write_fifo[fifo_read_ptr];
            bram_en_reg <= 1'b1;
            bram_we_reg <= 4'hF;
            
            // Consume FIFO entry and advance BRAM address
            fifo_read_ptr <= fifo_read_ptr + 1;
            write_address <= write_address + 1;
            
            // Handle address wrap
            if (write_address >= (BRAM_DEPTH_WORDS - 1)) begin
                write_address <= '0;
            end
        end
        
        // ====================================================================
        // FIFO COUNT MANAGEMENT
        // ====================================================================
        
        // Update FIFO count based on operations (perfectly synchronized!)
        case ({fifo_write_this_cycle, fifo_read_this_cycle})
            2'b00: fifo_count <= fifo_count;        // No operations
            2'b01: fifo_count <= fifo_count - 1;    // Read only
            2'b10: fifo_count <= fifo_count + 1;    // Write only
            2'b11: fifo_count <= fifo_count;        // Both: +1-1 = 0
        endcase
    end
end

endmodule