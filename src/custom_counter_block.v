`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/17/2025 11:10:55 AM
// Design Name: 
// Module Name: custom_block
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module custom_counter_block(
    input wire clk,             // 84 MHz input clock from PS
    input wire [31:0] enable,   // control register input
    output reg [31:0] count     // 32-bit output
);

    reg [26:0] clk_divider = 0;  // clock divider for 1 Hz
    reg clk_1hz = 0;

    reg [31:0] prev_enable = 0;
    reg counting = 0;

    // 84 MHz to ~1 Hz divider
    always @(posedge clk) begin
        if (clk_divider >= 84000000 - 1) begin
            clk_divider <= 0;
            clk_1hz <= 1;
        end else begin
            clk_divider <= clk_divider + 1;
            clk_1hz <= 0;
        end
    end

    // Counter and control logic
    always @(posedge clk) begin
        if (enable == 0) begin
            counting <= 0;
        end else if (prev_enable == 0 && enable != 0) begin
            count <= 0;
            counting <= 1;
        end else if (counting && clk_1hz) begin
            count <= count + 1;
        end

        prev_enable <= enable;
    end

endmodule
