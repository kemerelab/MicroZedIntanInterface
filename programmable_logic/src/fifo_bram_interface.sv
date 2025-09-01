// File: fifo_bram_interface.sv
// FIFO stores 64-bit words, BRAM writes 32-bit words
// Simple interface with 2-state FSM for BRAM writes

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
    input  logic [63:0] fifo_write_data,      // 64-bit
    output logic        fifo_full,
    output logic [8:0]  fifo_count,           // Count of 64-bit entries
    input logic         fifo_packet_end_flag, // this travels along with every fifo word
    
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
logic [64:0] write_fifo [0:FIFO_DEPTH-1]; // 64-bit data + 1-bit packet_end_flag
logic [FIFO_PTR_WIDTH-1:0] fifo_write_ptr;
logic [FIFO_PTR_WIDTH-1:0] fifo_read_ptr;

// FIFO status signals
assign fifo_full = (fifo_count == FIFO_DEPTH);

// BRAM interface registers
logic [15:0] bram_addr_reg;
logic [BRAM_DATA_WIDTH-1:0] bram_din_reg;
logic        bram_en_reg;
logic [3:0]  bram_we_reg;
logic [BRAM_WORD_ADDR_WIDTH-1:0] write_address;
logic [BRAM_WORD_ADDR_WIDTH-1:0] packet_boundary_address;

// Connect BRAM interface
assign bram_addr = bram_addr_reg;
assign bram_din  = bram_din_reg;
assign bram_en   = bram_en_reg;
assign bram_we   = bram_we_reg;
assign bram_clk  = clk;
assign bram_rst  = ~rstn;

// Export current write address for PS monitoring
assign current_bram_address = packet_boundary_address;

// Registered FIFO write signals (1 cycle latency)
logic fifo_write_en_reg;
logic [63:0] fifo_write_data_reg;

// State machine for 64-bit FIFO to 32-bit BRAM conversion
typedef enum logic {
    WRITE_LOW,  // Write lower 32 bits of 64-bit FIFO data
    WRITE_HIGH  // Write upper 32 bits of 64-bit FIFO data
} bram_write_state_t;

bram_write_state_t bram_state;
logic [63:0] current_64bit_data;
logic current_packet_end_flag;

// Logical entities used for updating count register
logic fifo_write_this_cycle;
logic fifo_read_this_cycle;

// Combined FIFO and BRAM management - single always block for clean timing
always_ff @(posedge clk) begin
    if (!rstn) begin
        // FIFO write side
        fifo_write_ptr <= '0;
        fifo_write_en_reg <= 1'b0;
        fifo_write_data_reg <= 64'h0;
        
        // FIFO read side and BRAM interface
        fifo_read_ptr <= '0;
        fifo_count <= '0;
        bram_addr_reg <= 16'h0;
        bram_din_reg <= 32'h0;
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        write_address <= '0;
        packet_boundary_address <= '0;
        
        // State machine
        bram_state <= WRITE_LOW;
        current_64bit_data <= 64'h0;
        current_packet_end_flag <= 1'b0;
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
            write_fifo[fifo_write_ptr] <= {fifo_packet_end_flag, fifo_write_data_reg};
            fifo_write_ptr <= fifo_write_ptr + 1;
        end
        
        // ====================================================================
        // FIFO READ SIDE and BRAM WRITE (FIFO → BRAM) with State Machine
        // ====================================================================
        
        // Default: no BRAM write
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        
        case (bram_state)
            WRITE_LOW: begin
                // Can only read from FIFO if data is available
                if (fifo_count > 0) begin
                    // Read 64-bit data from FIFO and extract packet end flag
                    logic [64:0] fifo_entry = write_fifo[fifo_read_ptr];
                    current_packet_end_flag <= fifo_entry[64];
                    current_64bit_data <= fifo_entry[63:0];
                    
                    // Write lower 32 bits to BRAM
                    bram_addr_reg <= {write_address, 2'b00};
                    bram_din_reg <= fifo_entry[31:0];
                    bram_en_reg <= 1'b1;
                    bram_we_reg <= 4'hF;
                    
                    // Consume FIFO entry and advance BRAM address
                    fifo_read_ptr <= fifo_read_ptr + 1;
                    write_address <= (write_address >= (BRAM_DEPTH_WORDS - 1)) ? '0 : (write_address + 1);
                    
                    // Move to next state to write upper 32 bits
                    bram_state <= WRITE_HIGH;
                    fifo_read_this_cycle = 1'b1;
                end else begin
                    fifo_read_this_cycle = 1'b0;
                end
            end
            
            WRITE_HIGH: begin
                // Write upper 32 bits to BRAM (no FIFO read required)
                logic [BRAM_WORD_ADDR_WIDTH-1:0] next_write_address;
                
                bram_addr_reg <= {write_address, 2'b00};
                bram_din_reg <= current_64bit_data[63:32]; // Upper 32 bits
                bram_en_reg <= 1'b1;
                bram_we_reg <= 4'hF;
                
                // Advance BRAM address
                next_write_address = (write_address >= (BRAM_DEPTH_WORDS - 1)) ? '0 : (write_address + 1);
                write_address <= next_write_address;
                
                // Check for packet boundary (only on the second write of the 64-bit word)
                if (current_packet_end_flag) begin
                    packet_boundary_address <= next_write_address;  // Point to next packet start
                end
                
                // Return to first state for next 64-bit word
                bram_state <= WRITE_LOW;
                fifo_read_this_cycle = 1'b0; // No FIFO read in this state
            end
        endcase
        
        // ====================================================================
        // FIFO COUNT MANAGEMENT
        // ====================================================================
        
        // Update FIFO count based on operations (perfectly synchronized!)
        case ({fifo_write_this_cycle, fifo_read_this_cycle})
            2'b00: fifo_count <= fifo_count;        // No operations
            2'b01: fifo_count <= fifo_count - 1;    // Read only (happens in WRITE_LOW state)
            2'b10: fifo_count <= fifo_count + 1;    // Write only
            2'b11: fifo_count <= fifo_count;        // Both: +1-1 = 0
        endcase
    end
end

endmodule
