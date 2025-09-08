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
    
    // FIFO interface (64-bit, gets converted to 32-bit for BRAM)
    output logic        fifo_write_en,
    output logic [63:0] fifo_write_data,
    output logic [3:0]  fifo_channel_mask,     // Which 16-bit segments are valid

    input  logic        fifo_full,
    input  logic [8:0]  fifo_count,
    
    output logic        fifo_packet_end_flag, // gets written with each word. 1 if it's the last word in a packet
        
    // Serial interface signals
    output logic        csn,        // Chip select (active low)
    output logic        sclk,       // Serial clock
    output logic        copi,       // Controller Out, Peripheral In
    input  logic        cipo0,      // Controller In, Peripheral Out 0
    input  logic        cipo1       // Controller In, Peripheral Out 1
);

// Extract control bits
logic enable_transmission = ctrl_regs_pl[0*32 + 0];

// Safe control registers - only updated when transmission is not active
logic reset_timestamp_reg;
logic debug_mode_reg;
logic [31:0] loop_count_reg;
logic [3:0] phase0_reg;
logic [3:0] phase1_reg; 
logic [3:0] channel_enable_reg;
// Protected COPI message words (36 x 16-bit words) - only updated when transmission inactive
logic [15:0] copi_words_reg [0:35];

// Reserved control registers for future use
logic [31:0] ctrl_reg_3 = ctrl_regs_pl[3*32 +: 32];  // Reserved for future control

// Safe control register updates - only when transmission is not active
always_ff @(posedge clk) begin
    if (!rstn) begin
        reset_timestamp_reg <= 1'b0;
        debug_mode_reg <= 1'b0;
        loop_count_reg <= 32'd0;
        phase0_reg <= 4'd0;
        phase1_reg <= 4'd0;
        channel_enable_reg <= 4'b1111;  // Default: all channels enabled
        
        // Initialize COPI words to safe defaults
        for (int j = 0; j < 36; j++) begin
            copi_words_reg[j] <= 16'h0;
        end
    end else begin
        // Only update control registers when transmission is not active
        if (!transmission_active) begin
            reset_timestamp_reg <= ctrl_regs_pl[0*32 + 1];
            debug_mode_reg <= ctrl_regs_pl[0*32 + 3];
            loop_count_reg <= ctrl_regs_pl[1*32 +: 32];
            phase0_reg <= ctrl_regs_pl[2*32 + 3 : 2*32 + 0];
            phase1_reg <= ctrl_regs_pl[2*32 + 7 : 2*32 + 4];
            channel_enable_reg <= ctrl_regs_pl[2*32 + 11 : 2*32 + 8];
            
            // Update COPI words from control registers 4-21 (18 registers total)
            for (int j = 0; j < 18; j++) begin
                copi_words_reg[2*j]     <= ctrl_regs_pl[(j+4)*32 +: 16];      // Low 16 bits
                copi_words_reg[2*j + 1] <= ctrl_regs_pl[(j+4)*32 + 16 +: 16]; // High 16 bits
            end
        end
    end
end

// CIPO received data storage (4 separate 16-bit registers per cycle)
logic [31:0] cipo0_data [0:34];  // CIPO0 line, register A (low 16 bits) and B (upper 16 bits)
logic [31:0] cipo1_data [0:34];  // CIPO1 line, register A

// Registers for COPI data from CIPO 0 and CIPO 1
reg [73:0] cipo0_4x_oversampled;
reg [73:0] cipo1_4x_oversampled;
reg [31:0] cipo0_phase_selected;
reg [31:0] cipo1_phase_selected;

// Instantiate phase selector modules that correct for CIPO delay because of long cable length
CIPO_combined_phase_selector cipo0_selector(
    .phase_select(phase0_reg),
    .CIPO4x(cipo0_4x_oversampled),
    .CIPO(cipo0_phase_selected)
);
CIPO_combined_phase_selector cipo1_selector(
    .phase_select(phase1_reg),
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
logic        loop_limit_reached;
logic [31:0] loop_counter;

// Debug mode sine wave table index
logic [8:0] dummy_data_index;
// Debug mode 512-entry sine lookup table (signed 16-bit values)
logic [15:0] sine_lut [0:511];

// Initialize sine lookup table
initial begin
    // Generate 512-point sine wave (signed 16-bit, ±32767 range)
    for (int i = 0; i < 512; i++) begin
        real angle = 2.0 * 3.14159265359 * i / 512.0;
        real sine_real = 32767.0 * $sin(angle);
        sine_lut[i] = $rtoi(sine_real);
    end
end

// Helper signals for state machine logic
logic is_last_state = (state_counter == 7'd79);
logic is_first_cycle = (cycle_counter == 6'd0);
logic is_last_cycle = (cycle_counter == 6'd34);

// State machine and control logic 
always_ff @(posedge clk) begin
    if (!rstn) begin
        state_counter <= 7'd0;
        cycle_counter <= 6'd0;
        timestamp <= 64'd0;
        transmission_active <= 1'b0;
        loop_limit_reached <= 1'b0;
        loop_counter <= 32'd1; // 1 indexed
    end else begin        
        // State machine goes from 0 to 79, then repeats
        if (is_last_state) begin
            state_counter <= 7'd0;
            if (is_last_cycle) begin
                cycle_counter <= 6'd0;

                if (!enable_transmission && reset_timestamp_reg) begin
                    timestamp <= 64'd0;
                end else begin
                    timestamp <= timestamp + 1; // timestamp increments whether transmitting or not
                end

                if (!enable_transmission) begin // either this just happened or is still true
                    transmission_active <= 1'b0;
                    loop_limit_reached <= 1'b0; 
                end

                if (transmission_active) begin
                    if (loop_limit_reached) begin
                        transmission_active <= 1'b0;
                    end
                    loop_counter <= loop_counter + 1;
                    loop_limit_reached <= (loop_count_reg != 32'd0) && (loop_counter >= loop_count_reg);

                end else begin // transmission is not currently active
                    if (enable_transmission && !loop_limit_reached) begin
                        loop_counter <= 32'd1;  // Reset when starting new transmission
                        loop_limit_reached <= (loop_count_reg != 32'd0) && (loop_count_reg <= 32'd1); // Catch the tricky single transmission case
                        transmission_active <= 1'b1;
                    end
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

State 0:  CSn=0, SCLK=0, COPI=0 (default) [first of 35 cycles - fifo enqueue magic header words]
State 1:  CSn=0, SCLK=0, COPI=copi_words[cycle_counter][15] (setup bit 15) 
State 2:  CSn=0, SCLK=1, COPI=copi_words[cycle_counter][15] (clock bit 15) [first of 35 cycles - fifo enqueue timestamp words]
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
State 66: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 67: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 68: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 69: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 70: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 71: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 72: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 73: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 74: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO) 
State 75: CSn=1, SCLK=0, COPI=0 (continue to read in data from CIPO)  
State 76: CSn=1, SCLK=0, COPI=0 (register buffer data from phase selector)
State 77: CSn=1, SCLK=0, COPI=0 (inactive) [fifo enqueue 64b of combined CIPO data]
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
            // Uses copi_words_reg[cycle_counter] as the source for each cycle's transmission  
            // Bit index is just the bitwise NOT of state_counter[5:2] (since 15-x = ~x for 4-bit x)
            if  (state_counter <= 7'd63) begin //removed part of conditional
                logic [3:0] bit_index = ~state_counter[5:2];  // MSB first: ~0=15, ~1=14, ..., ~15=0
                copi <= copi_words_reg[cycle_counter][bit_index];
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
            cipo1_data[cycle_counter] <= cipo1_phase_selected; // It's ready one clock cycle after being latched in
        end
    end
end

// Data-to-BRAM processing
always_ff @(posedge clk) begin
    if (!rstn) begin
        fifo_write_en <= 1'b0;
        fifo_write_data <= 64'h0;
        fifo_channel_mask <= 4'h0;
        packets_sent <= 32'd0;
        fifo_packet_end_flag <= 1'b0;

        dummy_data_index <= 9'd0;
    end else begin
        // Default: no FIFO write
        fifo_write_en <= 1'b0;
        
        if (transmission_active && !fifo_full) begin
            // Header writes (first cycle only) - always fully valid
            if (state_counter inside {7'd0, 7'd1}) begin
                if (is_first_cycle) begin
                    fifo_write_en <= 1'b1;
                    fifo_channel_mask <= 4'b1111;  // Header is always fully valid
                    fifo_packet_end_flag <= 1'b0;  // Header words are never at the end
                    case (state_counter)
                        7'd0: fifo_write_data <= {MAGIC_NUMBER_HIGH, MAGIC_NUMBER_LOW}; // magic number
                        7'd1: fifo_write_data <= timestamp;
                    endcase
                end
            end 
            
            // Data writes - Pack both CIPO lines into single 64-bit write with channel mask
            if (state_counter == 7'd77) begin
                fifo_write_en <= 1'b1;
                fifo_channel_mask <= channel_enable_reg;  // Use current channel enable settings
                fifo_packet_end_flag <= is_last_cycle;    // Only last cycle's data word ends the packet
                
                if (!debug_mode_reg) begin
                    // Pack real CIPO data: CIPO1 in upper 32 bits, CIPO0 in lower 32 bits
                    fifo_write_data <= {cipo1_data[cycle_counter], cipo0_data[cycle_counter]};
                end else begin
                    // Load debug data with sine wave data
                    logic [5:0] channel_offset;  // Only needs 6 bits for values 0-32
                    logic [15:0] cipo0_regular_val, cipo0_ddr_val, cipo1_regular_val, cipo1_ddr_val;
                    logic [8:0] base_phase;         // index into 512-entry LUT
                    
                    // Calculate base sample index (0-32 for cycles 2-34)
                    channel_offset = (cycle_counter >= 6'd2) ? (cycle_counter - 6'd2) : 6'd0;
                    
                    // Base phase for this sample (9 bits total)
                    base_phase = dummy_data_index + channel_offset;
                    
                    // Generate sine values with frequency multiplication using left shifts
                    cipo0_regular_val = sine_lut[base_phase];                       // 1× = 58.6 Hz
                    cipo0_ddr_val     = sine_lut[(base_phase << 1) & 9'h1FF];       // 2× = 117.2 Hz  
                    cipo1_regular_val = sine_lut[(base_phase << 2) & 9'h1FF];       // 4× = 234.4 Hz
                    cipo1_ddr_val     = sine_lut[(base_phase << 3) & 9'h1FF];       // 8× = 468.8 Hz
                    
                    // Pack debug data: CIPO1 in upper 32 bits, CIPO0 in lower 32 bits
                    fifo_write_data <= {{cipo1_ddr_val, cipo1_regular_val}, {cipo0_ddr_val, cipo0_regular_val}};
                end
            end
                    
            if (is_last_cycle) begin
                if (is_last_state) begin
                    packets_sent <= packets_sent + 1;
                    // Increment dummy data index for continuous sine wave across packets
                    dummy_data_index <= dummy_data_index + 9'd1;
                end
            end

        end
    end
end

// Pack status signals
// Status Register 0: Dynamic status and counters (locally generated)
assign status_regs_pl[0*32 +: 32] = {
    15'd0,                // [31:17] - reserved for future flags
    cycle_counter,        // [16:11] - 6 bits
    1'b0,                 // [10] - reserved  
    state_counter,        // [9:3] - 7 bits
    1'b0,                 // [2] - reserved for future flags
    loop_limit_reached,   // [1] - 1 bit
    transmission_active   // [0] - 1 bit
};

// Status Register 1: Reflected control parameters (registered versions)
assign status_regs_pl[1*32 +: 32] = {
    8'd0,                 // [31:24] - reserved
    channel_enable_reg,   // [23:20] - 4 bits
    phase1_reg,           // [19:16] - 4 bits
    phase0_reg,           // [15:12] - 4 bits  
    8'd0,                 // [11:4] - reserved
    debug_mode_reg,       // [3] - 1 bit
    1'b0,                 // [2] - reserved
    reset_timestamp_reg,  // [1] - 1 bit
    enable_transmission   // [0] - 1 bit (current value, not registered)
};

assign status_regs_pl[2*32 +: 32] = packets_sent;
assign status_regs_pl[3*32 +: 32] = timestamp[31:0];
assign status_regs_pl[4*32 +: 32] = timestamp[63:32];
assign status_regs_pl[5*32 +: 32] = loop_count_reg;
assign status_regs_pl[6*32 +: 32] = ctrl_regs_pl[0*32 +: 32]; // reflected
assign status_regs_pl[7*32 +: 32] = ctrl_regs_pl[1*32 +: 32]; // reflected
assign status_regs_pl[8*32 +: 32] = ctrl_regs_pl[2*32 +: 32]; // reflected
assign status_regs_pl[9*32 +: 32] = ctrl_regs_pl[3*32 +: 32]; // reflected

// Status register 11 will be added by wrapper

endmodule
