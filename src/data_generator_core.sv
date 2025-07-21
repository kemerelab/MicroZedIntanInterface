// File: data_generator_core.sv
// Clean data generator - just focuses on generating data

module data_generator_core (
    input  logic        clk,
    input  logic        rstn,
    
    // Control and status interfaces
    input  logic [32*22-1:0] ctrl_regs_pl,
    output logic [32*6-1:0]  status_regs_pl,  // Only 6 registers - wrapper adds 7th
    
    // FIFO interface - simple and clean
    output logic        fifo_write_en,
    output logic [31:0] fifo_write_data,
    input  logic        fifo_full,
    input  logic [8:0]  fifo_count
);

// Extract control bits
logic enable_transmission = ctrl_regs_pl[0*32 + 0];
logic reset_timestamp     = ctrl_regs_pl[0*32 + 1];
// Loop count: number of 35-cycle frames to run (0 = infinite)
logic [31:0] loop_count = ctrl_regs_pl[1*32 +: 32];

// Unpack extra 16-bit words from ctrl_regs_pl
logic [15:0] ctrl_words [0:39];
genvar i;
generate
    for (i = 0; i < 20; i = i + 1) begin : unpack_ctrl
        assign ctrl_words[2*i]     = ctrl_regs_pl[(i+2)*32 +: 16];
        assign ctrl_words[2*i + 1] = ctrl_regs_pl[(i+2)*32 + 16 +: 16];
    end
endgenerate

// Control counters
logic [6:0] state_counter;
logic [5:0] cycle_counter;

// Constants
localparam logic [31:0] MAGIC_NUMBER_LOW  = 32'hDEADBEEF;
localparam logic [31:0] MAGIC_NUMBER_HIGH = 32'hCAFEBABE;
logic [63:0] timestamp;

// Status tracking
logic [31:0] packets_sent;
logic        transmission_active;
logic [31:0] loop_counter;
logic        synchronizing_dma_reset;

// Dummy data for testing
logic [15:0] dummy_data [3:0];
initial begin
    dummy_data[0] = 16'h1234;
    dummy_data[1] = 16'h5678;
    dummy_data[2] = 16'h9ABC;
    dummy_data[3] = 16'hDEF0;
end

// Helper signals
logic is_last_state = (state_counter == 7'd79);
logic is_first_cycle = (cycle_counter == 6'd0);
logic is_last_cycle = (cycle_counter == 6'd34);
logic loop_limit_reached = (loop_count != 32'd0) && (loop_counter >= loop_count);


// State machine and control logic 
always_ff @(posedge clk) begin
    if (!rstn) begin
        state_counter <= 7'd0;
        cycle_counter <= 6'd0;
        timestamp <= 64'd0;
        transmission_active <= 1'b0;
        loop_counter <= 32'd0;
        synchronizing_dma_reset <= 1'b0;
    end else begin        
        // State machine goes from 0 to 79, then repeats
        if (is_last_state) begin
            state_counter <= 7'd0;
            if (is_last_cycle) begin
                cycle_counter <= 6'd0;

                if (!enable_transmission && reset_timestamp) begin
                    timestamp <= 64'd0;
                end else begin
                    timestamp <= timestamp + 1;
                end

                if (transmission_active) begin
                    loop_counter <= loop_counter + 1;
                end

                if (enable_transmission && !loop_limit_reached) begin
                    transmission_active <= 1'b1;
                end else begin
                    transmission_active <= 1'b0;
                end

            end else begin
                cycle_counter <= cycle_counter + 1;
            end
        end else begin
            state_counter <= state_counter + 1;
        end
    end
end

// Data generation logic - clean and focused
always_ff @(posedge clk) begin
    if (!rstn) begin
        fifo_write_en <= 1'b0;
        fifo_write_data <= 32'h0;
        packets_sent <= 32'd0;
    end else begin
        // Default: no FIFO write
        fifo_write_en <= 1'b0;
        
        if (transmission_active && !fifo_full) begin
            // Header writes (first cycle only)
            if (state_counter inside {7'd0, 7'd1, 7'd2, 7'd3}) begin
                if (is_first_cycle) begin
                    fifo_write_en <= 1'b1;
                    case (state_counter)
                        7'd0: fifo_write_data <= MAGIC_NUMBER_LOW;
                        7'd1: fifo_write_data <= MAGIC_NUMBER_HIGH;
                        7'd2: fifo_write_data <= timestamp[31:0];
                        7'd3: fifo_write_data <= timestamp[63:32];
                        default: fifo_write_data <= 32'h0;
                    endcase
                end
            end 
            
            if ( (state_counter inside {7'd4, 7'd5, 7'd6, 7'd7})) begin
            // Data writes (every cycle)
                fifo_write_en <= 1'b1;
                case (state_counter)
                    7'd4: begin
                        case (cycle_counter)
                            6'd0:  fifo_write_data <= {dummy_data[1], dummy_data[0]};
                            6'd1:  fifo_write_data <= {cycle_counter, 10'h000, cycle_counter, 10'h000};
                            6'd2:  fifo_write_data <= timestamp[31:0];
                            default: fifo_write_data <= {cycle_counter, 6'h000, 6'h000, cycle_counter};
                        endcase
                    end
                    7'd5: begin
                        case (cycle_counter)
                            6'd0:  fifo_write_data <= {dummy_data[3], dummy_data[2]};
                            6'd1:  fifo_write_data <= {cycle_counter, 10'h000, cycle_counter, 10'h000};
                            6'd2:  fifo_write_data <= timestamp[63:32];
                            default: fifo_write_data <= {6'h000, cycle_counter, 6'h000, cycle_counter};
                        endcase
                    end
                    7'd6: fifo_write_data <= {16'h0006, cycle_counter, 10'h000};
                    7'd7: fifo_write_data <= {16'h0007, cycle_counter, 10'h000};
                    default: fifo_write_data <= 32'h0;
                endcase
            end
                
            if (is_last_cycle) begin
                if (is_last_state) begin
                    packets_sent <= packets_sent + 1;
                end
            end

        end
    end
end

// Pack status signals - only data generator's own status
assign status_regs_pl[0*32 +: 32] = {29'd0, loop_limit_reached, synchronizing_dma_reset, transmission_active};
assign status_regs_pl[1*32 +: 32] = {25'd0, state_counter};
assign status_regs_pl[2*32 +: 32] = {26'd0, cycle_counter};
assign status_regs_pl[3*32 +: 32] = packets_sent;
assign status_regs_pl[4*32 +: 32] = timestamp[31:0];
assign status_regs_pl[5*32 +: 32] = timestamp[63:32];
// Status register 6 will be added by wrapper

endmodule
