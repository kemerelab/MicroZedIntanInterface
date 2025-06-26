module data_generator_blk (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,
    
    // AXI Interface with attributes
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000, DATA_WIDTH 64, HAS_TLAST 1, HAS_TKEEP 0, HAS_TSTRB 0, HAS_TREADY 1" *)
    output reg  [63:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output reg         m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready,
    (* X_INTERFACE_INFOv = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output reg         m_axis_tlast,
    
    input  wire [32*22-1:0] ctrl_regs_pl,     // Full 32-bit control register from AXI
    output wire [32*6-1:0]  status_regs_pl // Status register to AXI
);


// Extract control bits from first control register
wire enable_transmission = ctrl_regs_pl[0*32 + 0];
wire reset_timestamp  = ctrl_regs_pl[0*32 + 1];

// Loop count: number of 35-cycle frames to run (0 = infinite)
wire [31:0] loop_count = 	ctrl_regs_pl[1*32 +: 32];

// Unpack extra 16-bit words from ctrl_regs_pl[2:21]
wire [15:0] ctrl_words [0:39];
genvar i;
generate
    for (i = 0; i < 20; i = i + 1) begin : unpack_ctrl
        assign ctrl_words[2*i]     = ctrl_regs_pl[(i+2)*32 +: 16];
        assign ctrl_words[2*i + 1] = ctrl_regs_pl[(i+2)*32 + 16 +: 16];
    end
endgenerate


// Control counters
reg [6:0] state_counter;      // Counts 0-79 (80 states)
reg [5:0] cycle_counter;      // Counts 0-34 (35 cycles)

// Constants
localparam [63:0] MAGIC_NUMBER = 64'hDEADBEEFCAFEBABE;
reg [63:0] timestamp;

// Status tracking
reg [15:0] packets_sent;      // Count of completed packets
reg        transmission_active; // Currently transmitting

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
    end else begin        

        // When we're not in reset, the state machine should always run.
        // This ensures that the module is accurately keeping a global
        // timestamp of experiment time.

        if (state_counter == 7'd79) begin
            // End of 80-state sequence
            state_counter <= 7'd0;
            if (cycle_counter == 6'd34) begin
                // End of 35 cycles - reset cycle counter, increment timestamp, parse control bits
                cycle_counter <= 6'd0;

                if (transmission_active) begin
                    packets_sent <= packets_sent + 1;
                end

                // Reset timestamp if requested (if transmission is not going to be active next cycle)
                if (!enable_transmission && reset_timestamp) begin
                    timestamp <= 1'b0;
                end else begin
                    timestamp <= timestamp + 1;
                end

                // Activate tranmission if it has been disabled. Disable otherwise!
                if (enable_transmission) begin
                    transmission_active <= 1'b1;
                end else begin
                    transmission_active <= 1'b0;
                end


            end else begin
                // Continue to next cycle
                cycle_counter <= cycle_counter + 1;
            end
        end else begin
            // Continue in current cycle
            state_counter <= state_counter + 1;
        end
        
        // Check if we can transmit (AXI handshake)
        if (!m_axis_tvalid || m_axis_tready) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tlast <= 1'b0;

            // Only transmit data if enable_transmission is high
            if (transmission_active) begin
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
                            if (cycle_counter == 6'd34) begin // This was the last data of the packet
                                m_axis_tlast <= 1'b1;
                            end
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
            end
            
        end
    end
end

// Pack status signals into output bus
assign status_regs_pl[ 0*32 +: 32] = {30'd0, transmission_active};
assign status_regs_pl[ 1*32 +: 32] = {25'd0, state_counter};
assign status_regs_pl[ 2*32 +: 32] = {26'd0, cycle_counter};
assign status_regs_pl[ 3*32 +: 32] = {16'd0, packets_sent};
assign status_regs_pl[ 4*32 +: 32] = timestamp[31:0];
assign status_regs_pl[ 5*32 +: 32] = timestamp[63:32];

endmodule