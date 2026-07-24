// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// File: fifo_bram_interface.sv
// FIFO stores 128-bit words + 8-bit channel metadata, BRAM writes 32-bit words.
// The 128-bit word holds up to EIGHT 16-bit segments (two SPI ports x 4 streams
// each); the 8-bit channel mask selects which segments are valid and they are
// packed tightly into the BRAM/packet. With only the low nibble set (port 0 /
// single-port mode) the BRAM write stream is bit-identical to the original
// 64-bit design -- the upper two chunks simply contribute zero segments.
// FSM: PROCESS_CHUNK (chunk_index 0..3), FINALIZE_PACKET.

module fifo_bram_interface #(
    parameter int BRAM_ADDR_WIDTH = 16,        // Byte address width
    parameter int BRAM_DATA_WIDTH = 32,        // Data width
    parameter int FIFO_DEPTH = 256,           // FIFO depth (entries)
    parameter int BRAM_DEPTH_WORDS = 16384    // BRAM depth in words (64KB / 4 = 16K words)
)(
    input  logic        clk,
    input  logic        rstn,

    // FIFO interface (input side)
    input  logic         fifo_write_en,
    input  logic [127:0] fifo_write_data,     // 128-bit data (up to 8 x 16-bit segments)
    input  logic [7:0]   fifo_channel_mask,   // Which 16-bit segments are valid (travels along with every fifo word)
    input  logic         fifo_packet_end_flag,// Signals end of packet (travels along with every fifo word)

    output logic        fifo_full,
    output logic [8:0]  fifo_count,           // Count of 128-bit entries

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

// FIFO entry = 128-bit data + 8-bit channel mask + 1-bit packet-end flag = 137 bits.
// Mapped to distributed RAM (LUTRAM): one synchronous write port + one asynchronous
// read port, and NOT reset element-by-element (see reset block below). Resetting every
// element forces flip-flop inference plus a wide read mux and a giant reset fanout;
// leaving the contents unreset lets Vivado infer memory instead. This mirrors the
// no-reset pattern used by the capture buffer in bram.sv.
// Production follow-up: convert to block RAM (requires a synchronous/registered read).
(* ram_style = "distributed" *)
logic [136:0] write_fifo [0:FIFO_DEPTH-1]; // 128-bit data + 8-bit mask + 1-bit flag
logic [FIFO_PTR_WIDTH-1:0] fifo_write_ptr;
logic [FIFO_PTR_WIDTH-1:0] fifo_read_ptr;

// FIFO status signals
assign fifo_full = (fifo_count == FIFO_DEPTH);

// State machine for processing 128-bit entries
typedef enum logic {
    PROCESS_CHUNK,    // Process 32-bit chunks (index 0..3)
    FINALIZE_PACKET   // Handle leftover 16 bits at packet end
} process_state_t;

process_state_t process_state;
logic [1:0] chunk_index;  // 0..3 = which 32-bit chunk of the 128-bit word

// Data processing registers
logic [31:0] data_buffer_reg;        // Accumulates 32-bit words for BRAM
logic        buffer_valid_reg;       // True when data_buffer_reg contains valid data
logic        packet_end_reg;      // Used to copy packet end flag over to BRAM write as needed

logic [15:0] stash;              // Holds leftover 16-bit segment
logic        stash_valid;        // True when stash contains valid data

// Pipeline registers for packet boundary tracking
logic        current_packet_end;  // Packet end flag for current FIFO entry being processed

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
logic [127:0] fifo_write_data_reg;
logic [7:0] fifo_channel_mask_reg;

// Control signals
logic fifo_write_this_cycle;
logic fifo_read_this_cycle;

// Helper function to extract segments from 32-bit word based on 2-bit mask
function automatic logic [31:0] extract_segments(input logic [31:0] word_32, input logic [1:0] mask_2);
    logic [15:0] seg0 = word_32[15:0];
    logic [15:0] seg1 = word_32[31:16];

    case (mask_2)
        2'b00: return 32'h0;                   // No segments
        2'b01: return {16'h0, seg0};           // Only segment 0
        2'b10: return {16'h0, seg1};           // Only segment 1 - TECHNICALLY THIS SHOULD NEVER OCCUR!
        2'b11: return {seg1, seg0};            // Both segments
    endcase
endfunction

// Helper function to count valid segments in 2-bit mask
function automatic logic [1:0] count_segments(input logic [1:0] mask_2);
    return mask_2[0] + mask_2[1];
endfunction

// Combinatorial logic for next state calculation
logic next_stash_valid;

// Combined FIFO and BRAM management
always_ff @(posedge clk) begin
    if (!rstn) begin
        // FIFO write side
        fifo_write_ptr <= '0;
        fifo_write_en_reg <= 1'b0;
        fifo_write_data_reg <= 128'h0;
        fifo_channel_mask_reg <= 8'h0;

        // FIFO read side
        fifo_read_ptr <= '0;
        fifo_count <= '0;

        // State machine
        process_state <= PROCESS_CHUNK;
        chunk_index <= 2'd0;

        // Data processing
        data_buffer_reg <= 32'h0;
        buffer_valid_reg <= 1'b0;
        stash <= 16'h0;
        stash_valid <= 1'b0;
        current_packet_end <= 1'b0;
        packet_end_reg <= 1'b0;

        // BRAM interface
        bram_addr_reg <= 16'h0;
        bram_din_reg <= 32'h0;
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        write_address <= '0;
        packet_boundary_address <= '0;

        // write_fifo (the FIFO storage array) is intentionally NOT reset here. Clearing
        // every element would force FF inference + a wide reset net; leaving it
        // unreset lets it map to distributed RAM. Safe because an entry is only ever
        // read after it has been written (reads are gated by fifo_count > 0, and
        // read_ptr trails write_ptr by exactly fifo_count written entries).

    end else begin

        // ====================================================================
        // FIFO WRITE SIDE (Data Generator → FIFO)
        // ====================================================================

        // Register the input signals (1 cycle delay for clean timing)
        fifo_write_en_reg <= fifo_write_en;
        fifo_write_data_reg <= fifo_write_data;
        fifo_channel_mask_reg <= fifo_channel_mask;

        // Determine if FIFO write will happen this cycle
        fifo_write_this_cycle = fifo_write_en_reg && !fifo_full;

        // Perform FIFO write operation
        if (fifo_write_this_cycle) begin
            write_fifo[fifo_write_ptr] <= {fifo_packet_end_flag, fifo_channel_mask_reg, fifo_write_data_reg};
            fifo_write_ptr <= fifo_write_ptr + 1;
        end

        // ====================================================================
        // DATA PROCESSING STATE MACHINE
        // ====================================================================

        // Default: no BRAM write
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        fifo_read_this_cycle = 1'b0;
        next_stash_valid = stash_valid;  // Default to current state (in case we skip a chunk)
        buffer_valid_reg <= 1'b0;

        case (process_state)

            PROCESS_CHUNK: begin
                // Process current chunk if FIFO has data
                if (fifo_count > 0) begin
                    // Read FIFO entry directly (pointer doesn't advance until all chunks done).
                    // Declared then blocking-assigned (NOT declared-with-initializer) so the
                    // read re-evaluates every execution: a block-local `logic x = expr;` is a
                    // run-once static initializer under xsim, which would latch stale data.
                    logic [136:0] fifo_entry;
                    logic packet_end;
                    logic [7:0] channel_mask;
                    logic [127:0] data_word;

                    logic [1:0] chunk_mask;
                    logic [31:0] chunk_word;
                    logic [31:0] extracted;
                    logic [1:0] seg_count;

                    fifo_entry = write_fifo[fifo_read_ptr];
                    packet_end = fifo_entry[136];
                    channel_mask = fifo_entry[135:128];
                    data_word = fifo_entry[127:0];

                    // Store packet end flag for this entry
                    current_packet_end <= packet_end;

                    // Select the 32-bit chunk and its 2-bit sub-mask for this chunk_index
                    // (chunk 0 = segments 0/1 = port-0 reg/ddr cipo0; ... chunk 3 = the
                    //  upper port-1 ddr streams).
                    chunk_mask = channel_mask[chunk_index*2 +: 2];
                    chunk_word = data_word[chunk_index*32 +: 32];

                    // Common processing logic based on channel enable mask
                    extracted = extract_segments(chunk_word, chunk_mask);
                    seg_count = count_segments(chunk_mask);

                    // Two possibilities - we have leftover data in stash from last chunk or not
                    if (seg_count == 2'd2) begin
                        // Two segments: fill buffer if we have stash, or store in buffer
                        if (stash_valid) begin
                            // Combine stash with first segment, second becomes new stash
                            data_buffer_reg <= {extracted[15:0], stash};
                            buffer_valid_reg <= 1'b1;
                            stash <= extracted[31:16];
                            next_stash_valid = 1'b1; // Stash entry for next
                        end else begin
                            // Store both segments in buffer
                            data_buffer_reg <= extracted;
                            buffer_valid_reg <= 1'b1;
                            next_stash_valid = 1'b0; // No stash bc we copied directly
                        end
                    end else if (seg_count == 2'd1) begin
                        // One segment
                        if (stash_valid) begin
                            // Combine with stash to make full buffer
                            data_buffer_reg <= {extracted[15:0], stash};
                            buffer_valid_reg <= 1'b1;
                            next_stash_valid = 1'b0; // Combined with old stash
                        end else begin
                            // Put in stash
                            stash <= extracted[15:0];
                            next_stash_valid = 1'b1; // Stash entry for next
                        end
                    end

                    // State transition logic
                    if (chunk_index == 2'd3) begin
                        // Just finished the last (4th) chunk of this entry
                        if (current_packet_end && next_stash_valid) begin
                            // Need to finalize the packet with remaining stash
                            process_state <= FINALIZE_PACKET;
                        end else begin
                            packet_end_reg <= current_packet_end; // Copy the packet end over to the BRAM write

                            // Normal case: consume FIFO entry and move to next
                            fifo_read_ptr <= fifo_read_ptr + 1;
                            fifo_read_this_cycle = 1'b1;
                            process_state <= PROCESS_CHUNK;
                            chunk_index <= 2'd0;  // Reset to first chunk for next entry
                        end
                    end else begin
                        // Move to the next chunk of this entry
                        process_state <= PROCESS_CHUNK;
                        chunk_index <= chunk_index + 2'd1;
                    end
                end
            end

            FINALIZE_PACKET: begin
                // Handle leftover stash at packet end
                data_buffer_reg <= {16'h0000, stash};  // Pad with zeros
                buffer_valid_reg <= 1'b1;
                packet_end_reg <= current_packet_end;
                next_stash_valid = 1'b0; // Cleaned up and ready to go for next

                // Consume FIFO entry and return to normal processing
                fifo_read_ptr <= fifo_read_ptr + 1;
                fifo_read_this_cycle = 1'b1;
                process_state <= PROCESS_CHUNK;
                chunk_index <= 2'd0;
            end

        endcase

        stash_valid <= next_stash_valid;

        // ====================================================================
        // BRAM WRITE LOGIC
        // ====================================================================

        // Write buffer to BRAM when it's valid
        if (buffer_valid_reg) begin
            logic [BRAM_WORD_ADDR_WIDTH-1:0] next_address;

            bram_addr_reg <= {write_address, 2'b00};
            bram_din_reg <= data_buffer_reg;
            bram_en_reg <= 1'b1;
            bram_we_reg <= 4'hF;

            // Advance BRAM address
            next_address = (write_address >= (BRAM_DEPTH_WORDS - 1)) ? '0 : (write_address + 1);
            write_address <= next_address;

            if (packet_end_reg) begin
                packet_boundary_address <= next_address;
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
