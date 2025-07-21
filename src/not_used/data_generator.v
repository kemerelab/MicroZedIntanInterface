module data_generator_blk (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,
    
    // DMA reset input (active low)
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 DMA_RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        dma_reset_n,
    
    
    // AXI Interface with attributes
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TDATA" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000, DATA_WIDTH 64, HAS_TLAST 1, HAS_TKEEP 0, HAS_TSTRB 0, HAS_TREADY 1" *)
    output reg  [63:0] m_axis_tdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TVALID" *)
    output reg         m_axis_tvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TREADY" *)
    input  wire        m_axis_tready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 M_AXIS TLAST" *)
    output reg         m_axis_tlast,
    
    // Control and status interfaces
    input  wire [32*22-1:0] ctrl_regs_pl,
    output wire [32*6-1:0]  status_regs_pl
);

// Extract control bits from first control register
wire enable_transmission = ctrl_regs_pl[0*32 + 0];
wire reset_timestamp     = ctrl_regs_pl[0*32 + 1];

// Loop count: number of 35-cycle frames to run (0 = infinite)
wire [31:0] loop_count = ctrl_regs_pl[1*32 +: 32];

// Unpack extra 16-bit words from ctrl_regs_pl[2:21]
wire [15:0] ctrl_words [0:39];
genvar i;
generate
    for (i = 0; i < 20; i = i + 1) begin : unpack_ctrl
        assign ctrl_words[2*i]     = ctrl_regs_pl[(i+2)*32 +: 16];
        assign ctrl_words[2*i + 1] = ctrl_regs_pl[(i+2)*32 + 16 +: 16];
    end
endgenerate

// Create reset signals
wire full_system_reset_n = rstn;
wire axi_reset_n = rstn;
//wire axi_reset_n = rstn & dma_reset_n;

// Control counters
reg [6:0] state_counter;
reg [5:0] cycle_counter;

// Constants
localparam [63:0] MAGIC_NUMBER = 64'hDEADBEEFCAFEBABE;
reg [63:0] timestamp;

// Status tracking
reg [31:0] packets_sent;
reg        transmission_active;
reg [31:0] loop_counter;
reg        synchronizing_dma_reset;

// Dummy data for testing
reg [15:0] dummy_data [3:0];
initial begin
    dummy_data[0] = 16'h1234;
    dummy_data[1] = 16'h5678;
    dummy_data[2] = 16'h9ABC;
    dummy_data[3] = 16'hDEF0;
end

// Helper signals
wire is_last_state = (state_counter == 7'd79);
wire is_last_cycle = (cycle_counter == 6'd34);
wire is_data_state = (state_counter == 7'd2);
wire is_first_cycle = (cycle_counter == 6'd0);
wire loop_limit_reached = (loop_count != 32'd0) && (loop_counter >= loop_count);

// State machine and control logic
always @(posedge clk) begin
    if (!full_system_reset_n) begin
        state_counter <= 7'd0;
        cycle_counter <= 6'd0;
        timestamp <= 64'd0;
        packets_sent <= 32'd0;
        transmission_active <= 1'b0;
        loop_counter <= 32'd0;
        synchronizing_dma_reset <= 1'b0;
    end else begin
        // Detect DMA reset and handle synchronization
//        if (!dma_reset_n) begin
//            transmission_active <= 1'b0;
//            packets_sent <= 32'd0;
//            loop_counter <= 32'd0;
//            synchronizing_dma_reset <= 1'b1;
//        end else if (synchronizing_dma_reset) begin
//            if (state_counter == 7'd0 && cycle_counter == 6'd0) begin
//                synchronizing_dma_reset <= 1'b0;
//            end
//        end
        
        // State machine always runs to maintain timing
        if (is_last_state) begin
            state_counter <= 7'd0;
            if (is_last_cycle) begin
                cycle_counter <= 6'd0;

                if (transmission_active) begin
                    packets_sent <= packets_sent + 1;
                end

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
                
//                if (enable_transmission && !loop_limit_reached && !synchronizing_dma_reset) begin
//                    transmission_active <= 1'b1;
//                end else begin
//                    transmission_active <= 1'b0;
//                end                

            end else begin
                cycle_counter <= cycle_counter + 1;
            end
        end else begin
            state_counter <= state_counter + 1;
        end
        
    end
end

// AXI Stream output logic
always @(posedge clk) begin
    if (!axi_reset_n) begin
        m_axis_tdata <= 64'd0;
        m_axis_tvalid <= 1'b0;
        m_axis_tlast <= 1'b0;
    end else begin
        if (!m_axis_tvalid || m_axis_tready) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tlast <= 1'b0;

            if (transmission_active) begin
                case (state_counter)
                    7'd0: begin
                        if (is_first_cycle) begin
                            m_axis_tdata <= MAGIC_NUMBER;
                            m_axis_tvalid <= 1'b1;
                        end
                    end
                    7'd1: begin
                        if (is_first_cycle) begin
                            m_axis_tdata <= timestamp;
                            m_axis_tvalid <= 1'b1;
                        end
                    end
                    7'd2: begin
                        m_axis_tvalid <= 1'b1;
                        if (is_last_cycle) begin
                            m_axis_tdata <= {dummy_data[3], dummy_data[2], dummy_data[1], dummy_data[0]};
                            m_axis_tlast <= 1'b1;
                        end else begin
                            case (cycle_counter)
                                6'd0:  m_axis_tdata <= {dummy_data[0], dummy_data[1], dummy_data[2], dummy_data[3]};
                                6'd1:  m_axis_tdata <= {cycle_counter, 10'h000, cycle_counter, 10'h000, cycle_counter, 10'h000, cycle_counter, 10'h000};
                                6'd2:  m_axis_tdata <= {timestamp[15:0], timestamp[31:16], timestamp[47:32], timestamp[63:48]};
                                default: m_axis_tdata <= {cycle_counter, cycle_counter, cycle_counter, cycle_counter, 
                                                        cycle_counter, cycle_counter, cycle_counter, cycle_counter};
                            endcase
                        end
                    end
                    default: begin
                        m_axis_tvalid <= 1'b0;
                    end
                endcase
            end
        end
    end
end

// Pack status signals into output bus
assign status_regs_pl[0*32 +: 32] = {29'd0, loop_limit_reached, synchronizing_dma_reset, transmission_active};
assign status_regs_pl[1*32 +: 32] = {25'd0, state_counter};
assign status_regs_pl[2*32 +: 32] = {26'd0, cycle_counter};
assign status_regs_pl[3*32 +: 32] = packets_sent;
assign status_regs_pl[4*32 +: 32] = timestamp[31:0];
assign status_regs_pl[5*32 +: 32] = timestamp[63:32];

endmodule