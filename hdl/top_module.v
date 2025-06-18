`timescale 1ns/1ns

module top_module(
    input wire clk_ext, //50 MHz from the carrier board
    input wire reset,
    input wire SPI_continuous,
    input wire SPI_start,
    input wire [31:0] max_timestep,
    output wire clk,
    output wire clk_stable,
    output wire SCLK,
    output wire CS,
    output wire MOSI,
    output reg [6:0] state_counter,
    output reg [5:0] channel,
    output reg [31:0] timestamp,
    wire [3:0] ctr,
    wire [15:0] instr
    
);

// wire clk;
// wire clk_stable;
//reg [4:0] state_counter = 0;
//reg [5:0] channel;
reg [15:0] instructions [0:34];
assign instr = instructions[channel];
// wire [15:0] instr = instructions[channel];
reg [3:0] instr_counter = 4'hF;
assign MOSI = (CS || state_counter > 7'd63)? 1'bZ : instr[instr_counter];
assign ctr = instr_counter;


clk_wiz_0 pll(
    .clk_in1(clk_ext),
    .reset(1'b0),
    .locked(clk_stable),
    .clk_out1(clk)
);

initial begin
    $readmemh("instr.mem", instructions);
end


always @(posedge clk) begin
    if(reset || ~clk_stable) begin
        state_counter <= 7'd127;
        channel <= 0;
        timestamp <= 0;
        instr_counter <= 4'hF;
    end else begin

        if(state_counter == 7'd79) begin
            state_counter <= 0;
        end else begin
            state_counter <= state_counter + 1;
        end
        if(state_counter <= 7'd63 & state_counter[1:0] == 2'b11) begin
           instr_counter <= instr_counter - 1;
        end

        case (state_counter)
            7'd127: begin
               timestamp <= 0;
               channel <= 0; 
               if(SPI_start) begin
                    state_counter <= 7'd79;
               end else begin
                    state_counter <= state_counter;
               end
            end
            7'd0: begin
                instr_counter <= 4'hF;
            end
            7'd77: begin
                timestamp <= (channel == 0)? (timestamp + 1): timestamp;
            end
            7'd78: begin 
                channel <= (channel == 34)? 0 : (channel + 1);
                if(SPI_continuous) begin
                    state_counter <= state_counter + 1;
                end else begin
                    if (timestamp == max_timestep || max_timestep == 32'b0) begin
                        state_counter <= 7'd127;
                    end else begin
                        state_counter <= state_counter + 1;
                    end
                end
            end
            default: begin
                
            end
        endcase
        
    end
end

assign SCLK = ~CS & state_counter[1];
assign CS = (state_counter > 7'd65);


endmodule