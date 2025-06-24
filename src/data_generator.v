module data_generator_blk #(
    parameter NUM_CONTROL_REGS = 64,
    parameter NUM_STATUS_REGS = 64
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire [NUM_CONTROL_REGS*8-1:0] pl_control_regs,  // Packed control array from AXI
    output reg  [NUM_STATUS_REGS*8-1:0]  pl_status_regs,   // Packed status array to AXI
    
    output reg  [63:0] m_axis_tdata,
    output reg         m_axis_tvalid,
    input  wire        m_axis_tready,
    output reg         m_axis_tlast
);

// Unpack control registers for easier access
reg [7:0] control_regs [0:NUM_CONTROL_REGS-1];

// Generate blocks for unpacking control registers
genvar j;
generate
    for (j = 0; j < NUM_CONTROL_REGS; j = j + 1) begin : control_unpack_gen
        always @(*) begin
            control_regs[j] = pl_control_regs[j*8+7:j*8];
        end
    end
endgenerate

// Extract control signals from control register 0
wire transmit_enabled = control_regs[0][0];      // Bit 0 of register 0
wire reset_timestamp  = control_regs[0][1];      // Bit 1 of register 0
wire pause_timestamp  = control_regs[0][2];      // Bit 2 of register 0
// Bits 3-7 of register 0: Reserved for future use
// Registers 1-63: Available as control_regs[1] through control_regs[63]

// Interface attributes
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
(* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000, DATA_WIDTH 64, HAS_TLAST 1, HAS_TKEEP 0, HAS_TSTRB 0, HAS_TREADY 1" *)
wire [63:0] axis_tdata;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
wire axis_tvalid;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
wire axis_tready;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
wire axis_tlast;

// Assign interface wires to actual outputs
assign axis_tdata = m_axis_tdata;
assign axis_tvalid = m_axis_tvalid;
assign axis_tready = m_axis_tready;
assign axis_tlast = m_axis_tlast;

// Control counters
reg [6:0] state_counter;      // Counts 0-79 (80 states)
reg [5:0] cycle_counter;      // Counts 0-34 (35 cycles)

// Constants
localparam [63:0] MAGIC_NUMBER = 64'hDEADBEEFCAFEBABE;
reg [63:0] timestamp;

// Status tracking
reg [15:0] packets_sent;      // Count of completed packets
reg        transmission_active; // Currently transmitting
reg        last_packet_sent;   // Indicates last packet was sent

// Dummy data for testing
reg [15:0] dummy_data [3:0];
initial begin
    dummy_data[0] = 16'h1234;
    dummy_data[1] = 16'h5678;
    dummy_data[2] = 16'h9ABC;
    dummy_data[3] = 16'hDEF0;
end

// State machine and data generation
always @(posedge clk) begin
    if (!rstn) begin
        state_counter <= 7'd0;
        cycle_counter <= 6'd0;
        m_axis_tdata <= 64'd0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
        timestamp <= 64'd0;
        packets_sent <= 16'd0;
        transmission_active <= 1'b0;
        last_packet_sent <= 1'b0;
    end else begin
        // Handle control register commands
        if (reset_timestamp && !transmit_enabled) begin
            timestamp <= 64'd0;
        end
        
        // State machine always runs - check if we can transmit (AXI handshake)
        if (!m_axis_tvalid || m_axis_tready) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tlast <= 1'b0;
            last_packet_sent <= 1'b0;
            
            // Only transmit data if transmit_enabled is high
            if (transmit_enabled) begin
                transmission_active <= 1'b1;
                
                // First cycle (cycle_counter == 0) - send magic number, timestamp, and dummy data
                if (cycle_counter == 6'd0) begin
                    case (state_counter)
                        7'd0: begin // State 1 - Full 64-bit magic number
                            m_axis_tdata <= MAGIC_NUMBER;
                            m_axis_tvalid <= 1'b1;
                        end
                        7'd1: begin // State 2 - 64-bit timestamp
                            m_axis_tdata <= timestamp;
                            m_axis_tvalid <= 1'b1;
                        end
                        7'd2: begin // State 3 - 64-bit dummy data word
                            m_axis_tdata <= {dummy_data[0], dummy_data[1], dummy_data[2], dummy_data[3]};
                            m_axis_tvalid <= 1'b1;
                        end
                        default: begin
                            // States 4-80: Do nothing, just advance
                            m_axis_tvalid <= 1'b0;
                        end
                    endcase
                end else begin
                    // Subsequent cycles (1-34) - only send dummy data
                    case (state_counter)
                        7'd2: begin // State 3 - 64-bit dummy data word
                            m_axis_tdata <= {dummy_data[0], dummy_data[1], dummy_data[2], dummy_data[3]};
                            m_axis_tvalid <= 1'b1;
                        end
                        default: begin
                            // All other states: Do nothing
                            m_axis_tvalid <= 1'b0;
                        end
                    endcase
                end
            end else begin
                transmission_active <= 1'b0;
            end
            
            // Always advance state counter (regardless of transmit_enabled)
            if (state_counter == 7'd79) begin
                // End of 80-state sequence
                state_counter <= 7'd0;
                if (cycle_counter == 6'd34) begin
                    // End of 35 cycles - increment timestamp and reset cycle counter
                    if (transmit_enabled) begin
                        m_axis_tlast <= 1'b1;
                        last_packet_sent <= 1'b1;
                        packets_sent <= packets_sent + 1;
                    end
                    if (!pause_timestamp) begin
                        timestamp <= timestamp + 1;
                    end
                    cycle_counter <= 6'd0;
                end else begin
                    // Continue to next cycle
                    cycle_counter <= cycle_counter + 1;
                end
            end else begin
                // Continue in current cycle
                state_counter <= state_counter + 1;
            end
        end
    end
end

// Unpack status registers for easier access
reg [7:0] status_regs [0:NUM_STATUS_REGS-1];

// Build status register array - populate unpacked status registers
always @(*) begin
    // Register 0: Control flags
    status_regs[0] = {6'b000000, last_packet_sent, transmission_active};
    
    // Register 1: State counter (0-79)
    status_regs[1] = {1'b0, state_counter};
    
    // Register 2: Cycle counter (0-34)
    status_regs[2] = {2'b00, cycle_counter};
    
    // Register 3-4: Packet count (16-bit)
    status_regs[3] = packets_sent[7:0];
    status_regs[4] = packets_sent[15:8];
    
    // Register 5: Reserved
    status_regs[5] = 8'h00;

    // Register 6-13: Timestamp (64-bit)
    status_regs[6]  = timestamp[7:0];
    status_regs[7]  = timestamp[15:8];
    status_regs[8]  = timestamp[23:16];
    status_regs[9]  = timestamp[31:24];
    status_regs[10] = timestamp[39:32];
    status_regs[11] = timestamp[47:40];
    status_regs[12] = timestamp[55:48];
    status_regs[13] = timestamp[63:56];
    
end

// Generate blocks for remaining status registers (set to 0)
generate
    for (j = 14; j < NUM_STATUS_REGS; j = j + 1) begin : status_zero_gen
        always @(*) begin
            status_regs[j] = 8'h00;
        end
    end
endgenerate

// Generate blocks for packing status registers
generate
    for (j = 0; j < NUM_STATUS_REGS; j = j + 1) begin : status_pack_gen
        always @(*) begin
            pl_status_regs[j*8+7:j*8] = status_regs[j];
        end
    end
endgenerate

endmodule