// Implemented as 3 major always blocks.
// 1. Run the master cycled state machine. Maintain the timestamp consistently
//    regardless of whether we acquiring/transmitting data. Process control data 
//    (which comes from the AXI interface), like enable/disable transmission and
//    reset timestamps.
// 2. Run the data acquisiton state machine. This uses the cycles/states controlled
//    by state machine #1.
// 3. Run the data exfiltration state machine. This loads data, prefaced by a
//    a header and a timestamp, into a FIFO for transmission via the dual port BRAM
//    to the PS. (FIFO and BRAM are external to this file.)

module data_generator_core (
    input  logic        clk,
    input  logic        rstn,
    
    // Control and status interfaces
    input  logic [32*22-1:0] ctrl_regs_pl,
    output logic [32*10-1:0]  status_regs_pl,  // Only 10 registers, including mirroring 4 control - wrapper adds 11th
    
    // FIFO interface - simple and clean
    output logic        fifo_write_en,
    output logic [31:0] fifo_write_data,
    input  logic        fifo_full,
    input  logic [8:0]  fifo_count,
    
    // Serial interface signals
    output logic        csn,        // Chip select (active low)
    output logic        sclk,       // Serial clock
    output logic        copi,       // Controller Out, Peripheral In
    input  logic        cipo0,      // Controller In, Peripheral Out 0
    input  logic        cipo1       // Controller In, Peripheral Out 1
);

// Extract control bits
logic enable_transmission = ctrl_regs_pl[0*32 + 0];
logic reset_timestamp     = ctrl_regs_pl[0*32 + 1];
// Loop count: number of 35-cycle frames to run (0 = infinite)
logic [31:0] loop_count = ctrl_regs_pl[1*32 +: 32];

logic [3:0] phase0 = ctrl_regs_pl[2*32 + 3 : 2*32 + 0]; // Cable length control word for CIPO0
logic [3:0] phase1 = ctrl_regs_pl[2*32 + 7 : 2*32 + 4]; // Cable length control word for CIPO1
logic debug_mode = ctrl_regs_pl[2*32 + 8]; // Send dummy data rather than real CIPO serial input

// Reserved control registers for future use
logic [31:0] ctrl_reg_3 = ctrl_regs_pl[3*32 +: 32];  // Reserved for future control


// Extract COPI message words from the last 18 registers (36 x 16-bit words)
logic [15:0] copi_words [0:35];
genvar i;
generate
    for (i = 0; i < 18; i = i + 1) begin : unpack_copi_words
        assign copi_words[2*i]     = ctrl_regs_pl[(i+4)*32 +: 16];      // Low 16 bits
        assign copi_words[2*i + 1] = ctrl_regs_pl[(i+4)*32 + 16 +: 16]; // High 16 bits
    end
endgenerate

// CIPO received data storage (4 separate 16-bit registers per cycle)
logic [31:0] cipo0_data [0:34];  // CIPO0 line, register A (low 16 bits) and B (upper 16 bits)
logic [31:0] cipo1_data [0:34];  // CIPO1 line, register A

// Registers for COPI data from COPI 0 and COPI 1
reg [73:0] cipo0_4x_oversampled;
reg [73:0] cipo1_4x_oversampled;
reg [31:0] cipo0_phase_selected;
reg [31:0] cipo1_phase_selected;

// Instantiate phase selector modules that correct for COPI delay because of long cable length
CIPO_combined_phase_selector cipo0_selector(
    .phase_select(phase0),
    .CIPO4x(cipo0_4x_oversampled),
    .CIPO(cipo0_phase_selected)
);
CIPO_combined_phase_selector cipo1_selector(
    .phase_select(phase1),
    .CIPO4x(cipo1_4x_oversampled),
    .CIPO(cipo1_phase_selected)
);



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

// Dummy data for testing
logic [15:0] dummy_data [3:0];
initial begin
    dummy_data[0] = 16'h1234;
    dummy_data[1] = 16'h5678;
    dummy_data[2] = 16'h9ABC;
    dummy_data[3] = 16'hDEF0;
end

// Helper signals for state machine logic
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

/*
Complete Serial Protocol Timing (80-state machine):

State 0:  CSn=0, SCLK=0, COPI=0 (default)
State 1:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][15] (setup bit 15)
State 2:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][15] (clock bit 15)
State 3:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][15] (hold)
State 4:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][15] (transition)
State 5:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][14] (setup bit 14)
State 6:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][14] (clock bit 14)
State 7:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][14] (hold)
...
State 57: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][1] (setup bit 1)
State 58: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][1] (clock bit 1)
State 59: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][1] (hold)
State 60: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][1] (transition)
State 61: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][0] (setup bit 0)
State 62: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][0] (clock bit 0 - LAST RISING EDGE)
State 63: CSn=0, SCLK=1, COPI=copi_words[cycle_counter][0] (hold - LAST CLOCK HIGH)
State 64: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][0] (LAST FALLING EDGE)
State 65: CSn=0, SCLK=0, COPI=copi_words[cycle_counter][0] (hold low)

*** CSn GOES HIGH HERE ***
State 66: CSn=1, SCLK=0, COPI=0 (inactive)
State 67: CSn=1, SCLK=0, COPI=0 (inactive)
State 68: CSn=1, SCLK=0, COPI=0 (inactive)
State 69: CSn=1, SCLK=0, COPI=0 (inactive)
State 70: CSn=1, SCLK=0, COPI=0 (inactive)
State 71: CSn=1, SCLK=0, COPI=0 (inactive)
State 72: CSn=1, SCLK=0, COPI=0 (inactive)
State 73: CSn=1, SCLK=0, COPI=0 (inactive)
State 74: CSn=1, SCLK=0, COPI=0 (inactive)
State 75: CSn=1, SCLK=0, COPI=0 (inactive)
State 76: CSn=1, SCLK=0, COPI=0 (inactive)
State 77: CSn=1, SCLK=0, COPI=0 (inactive)
State 78: CSn=1, SCLK=0, COPI=0 (inactive)
State 79: CSn=1, SCLK=0, COPI=0 (inactive)

Key Timing:
- CSn active: States 0-65 (66 states total)  
- 16 clocks: Rising edges at states 2,6,10,14,18,22,26,30,34,38,42,46,50,54,58,62
- Last clock high: State 63
- Last falling edge: State 64
- CSn goes HIGH: State 66
- Inactive period: States 66-79 (14 states)
*/

// Serial interface control - CSn, SCLK, and COPI generation 
always_ff @(posedge clk) begin
    if (!rstn) begin
        csn <= 1'b1;           // Default high (inactive)
        sclk <= 1'b0;          // Default low
        copi <= 1'b0;          // Default low
    end else begin
        // Default values (used when not transmitting or not in protocol)
        csn <= 1'b1;           // CSn high when not in protocol
        sclk <= 1'b0;          // SCLK low when not active
        copi <= 1'b0;          // COPI low when not active
        
        if (transmission_active) begin
            if (state_counter <= 7'd65) begin
                // CSn goes low during protocol (states 0-65)
                csn <= 1'b0;
                
                // SCLK is 1/4th the rate of the master clock, and there are 16 clock cycles
                // Clock high when bit 1 is set and state <= 63
                if ((state_counter[1] == 1'b1) && (state_counter <= 7'd63)) begin
                    sclk <= 1'b1;
                end
            end
                
            // COPI data transmission - MSB first, set on states 0,4,8,12,16,20,24,28,32,36,40,44,48,52,56,60
            // Uses copi_words[cycle_counter] as the source for each cycle's transmission  
            // Bit index is just the bitwise NOT of state_counter[5:2] (since 15-x = ~x for 4-bit x)
            if  (state_counter <= 7'd63) begin //removed part of conditional
                logic [3:0] bit_index = ~state_counter[5:2];  // MSB first: ~0=15, ~1=14, ..., ~15=0
                copi <= copi_words[cycle_counter][bit_index];
            end
            
        end
    end
end

// CIPO data sampling - 4 registers total (2 per input line)
always_ff @(posedge clk) begin
    if (!rstn) begin
        // Reset all received data
        for (int j = 0; j < 35; j++) begin
            cipo0_data[j] <= 32'h0;
            cipo1_data[j] <= 32'h0;
        end
        cipo0_4x_oversampled <= 74'h0;
        cipo1_4x_oversampled <= 74'h0;
    end else begin
        if (transmission_active && (state_counter >= 7'd2) && (state_counter <= 75)) begin
            cipo0_4x_oversampled[state_counter - 2] <= cipo0; // Latch data into the phase selector input
            cipo1_4x_oversampled[state_counter - 2] <= cipo1;
        end else if(transmission_active && state_counter == 7'd76) begin
            cipo0_data[cycle_counter] <= cipo0_phase_selected; // Get the phase selector output
            cipo1_data[cycle_counter] <= cipo1_phase_selected;
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
            if(debug_mode) begin
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
            end else begin
                if(state_counter inside {7'd77, 7'd78}) begin
                fifo_write_en <= 1'b1;
                case (state_counter)
                    7'd78: fifo_write_data <= cipo0_data[cycle_counter];
                    7'd79: fifo_write_data <= cipo1_data[cycle_counter];
                    default: fifo_write_data <= 32'h0;
                endcase
                end
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
assign status_regs_pl[0*32 +: 32] = {30'd0, loop_limit_reached, transmission_active};
assign status_regs_pl[1*32 +: 32] = {25'd0, state_counter};
assign status_regs_pl[2*32 +: 32] = {26'd0, cycle_counter};
assign status_regs_pl[3*32 +: 32] = packets_sent;
assign status_regs_pl[4*32 +: 32] = timestamp[31:0];
assign status_regs_pl[5*32 +: 32] = timestamp[63:32];
assign status_regs_pl[6*32 +: 32] = ctrl_regs_pl[0*32 +: 32];
assign status_regs_pl[7*32 +: 32] = ctrl_regs_pl[1*32 +: 32];
assign status_regs_pl[8*32 +: 32] = ctrl_regs_pl[2*32 +: 32];
assign status_regs_pl[9*32 +: 32] = ctrl_regs_pl[3*32 +: 32];

// Status register 11 will be added by wrapper

endmodule