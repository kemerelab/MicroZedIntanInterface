module data_generator_bram_blk (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,
    
    // Control and status interfaces
    input  wire [32*22-1:0] ctrl_regs_pl,
    output wire [32*7-1:0]  status_regs_pl,
    
    // BRAM Port A interface (32-bit)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *)
    output wire            bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *)
    output wire            bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *)
    output wire [15:0]     bram_addr,      // Byte address (for 64K)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *)
    output wire [31:0]     bram_din,       // 32-bit data
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *)
    input  wire [31:0]     bram_dout,      // 32-bit data from BRAM
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *)
    output wire            bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *)
    output wire [3:0]      bram_we         // 4 bytes for 32-bit
);

// Extract control bits
wire enable_transmission = ctrl_regs_pl[0*32 + 0];
wire reset_timestamp     = ctrl_regs_pl[0*32 + 1];

// Loop count: number of 35-cycle frames to run (0 = infinite)
wire [31:0] loop_count = ctrl_regs_pl[1*32 +: 32];

// Unpack extra 16-bit words from ctrl_regs_pl
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

// Control counters
reg [6:0] state_counter;
reg [5:0] cycle_counter;

// Constants - split 64-bit values into 32-bit pairs
localparam [31:0] MAGIC_NUMBER_LOW  = 32'hDEADBEEF;  // Lower 32 bits
localparam [31:0] MAGIC_NUMBER_HIGH = 32'hCAFEBABE;  // Upper 32 bits
reg [63:0] timestamp;

// Status tracking
reg [31:0] packets_sent;
reg        transmission_active;
reg [31:0] loop_counter;
reg        synchronizing_dma_reset;

// BRAM addressing
reg [13:0] write_address;    // 32-bit word address

// BRAM interface registers
reg [15:0] bram_addr_reg;
reg [31:0] bram_din_reg;     // 32-bit
reg        bram_en_reg;
reg [3:0]  bram_we_reg;      // 4-bit

// Connect BRAM interface
assign bram_addr = bram_addr_reg;
assign bram_din  = bram_din_reg;
assign bram_en   = bram_en_reg;
assign bram_we   = bram_we_reg;
assign bram_clk  = clk;           // Use 84 MHz clock for BRAM
assign bram_rst  = ~rstn;         // Convert active-low to active-high

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
wire is_first_cycle = (cycle_counter == 6'd0);
wire is_last_cycle = (cycle_counter == 6'd34);
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
        // State machine goes from 0 to 7, then repeats
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

            end else begin
                cycle_counter <= cycle_counter + 1;
            end
        end else begin
            state_counter <= state_counter + 1;
        end
    end
end


// BRAM write logic - states 0,1,2,3 for headers, 4,5,6,7 for data
always @(posedge clk) begin
    if (!full_system_reset_n) begin
        bram_addr_reg <= 16'd0;
        bram_din_reg <= 32'd0;
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;
        
        write_address <= 14'd0;
    end else begin
        // Default: no write
        bram_en_reg <= 1'b0;
        bram_we_reg <= 4'h0;

        if (transmission_active) begin
            if (is_first_cycle) begin
                if (state_counter == 7'd1 || 
                           state_counter == 7'd3 || 
                           state_counter == 7'd5 || 
                           state_counter == 7'd7) begin
                           
                    bram_addr_reg <= {write_address + 1, 2'b00};
                    bram_en_reg <= 1'b1;
                    bram_we_reg <= 4'hF;
                    write_address <= write_address + 1;
                
                    case (state_counter)
                        7'd1: begin
                            // Write magic number lower 32 bits first
                            bram_din_reg <= MAGIC_NUMBER_LOW;
                        end
                        7'd3: begin
                            // Write magic number upper 32 bits second
                            bram_din_reg <= MAGIC_NUMBER_HIGH;
                        end
                        7'd5: begin
                            // Write lower 32 bits of timestamp
                            bram_din_reg <= timestamp[31:0];
                        end
                        7'd7: begin
                            // Write upper 32 bits of timestamp
                            bram_din_reg <= timestamp[63:32];
                        end
                        default: begin
                            // we'll catch the data states next
                        end
                    endcase
                end
            end // special first cycle case for header
            
            // Writing data blocks on all cycles
            if (state_counter == 7'd9 || 
                       state_counter == 7'd11 || 
                       state_counter == 7'd13 || 
                       state_counter == 7'd15) begin
                       
                bram_en_reg <= 1'b1;
                bram_we_reg <= 4'hF;
                bram_addr_reg <= {write_address, 2'b00};                    
                write_address <= write_address + 1;
                
                case (state_counter)                
                    7'd9: begin
                        // Data state 4: Write lower 32 bits of data every cycle
                        case (cycle_counter)
                            6'd0:  bram_din_reg <= {dummy_data[1], dummy_data[0]};
                            6'd1:  bram_din_reg <= {cycle_counter, 10'h000, cycle_counter, 10'h000};
                            6'd2:  bram_din_reg <= timestamp[31:0];
                            default: bram_din_reg <= {cycle_counter, cycle_counter, cycle_counter, cycle_counter};
                        endcase
                    end
                    
                    7'd11: begin
                        // Data state 5: Write upper 32 bits of data every cycle
                        case (cycle_counter)
                            6'd0:  bram_din_reg <= {dummy_data[3], dummy_data[2]};
                            6'd1:  bram_din_reg <= {cycle_counter, 10'h000, cycle_counter, 10'h000};
                            6'd2:  bram_din_reg <= timestamp[63:32];
                            default: bram_din_reg <= {cycle_counter, cycle_counter, cycle_counter, cycle_counter};
                        endcase
                    end
                    
                    7'd13: begin
                        // Data state 6: Additional data every cycle
                        bram_din_reg <= {16'h0006, cycle_counter, 10'h000}; // State 6 marker
                    end
                    
                    7'd15: begin
                        // Data state 7: Additional data every cycle
                        bram_din_reg <= {16'h0007, cycle_counter, 10'h000}; // State 7 marker
                        
                        // Handle end of packet
                        if (is_last_cycle) begin                        
                            // Each packet: 4 header words + (35 cycles * 4 data words) = 144 words
                            if (write_address >= 16384 - 144) begin
                                write_address <= 14'd0;
                            end
                        end
                    end
                    
                    default: begin
                        // Other states where nothing happens (yet)
                    end
                endcase
            end
        end // transmission active
    end // not reset
end

// Pack status signals
assign status_regs_pl[0*32 +: 32] = {29'd0, loop_limit_reached, synchronizing_dma_reset, transmission_active};
assign status_regs_pl[1*32 +: 32] = {25'd0, state_counter};
assign status_regs_pl[2*32 +: 32] = {26'd0, cycle_counter};
assign status_regs_pl[3*32 +: 32] = packets_sent;
assign status_regs_pl[4*32 +: 32] = timestamp[31:0];
assign status_regs_pl[5*32 +: 32] = timestamp[63:32];
assign status_regs_pl[6*32 +: 32] = {18'd0, write_address}; 

endmodule