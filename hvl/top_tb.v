`timescale 1ns/1ns

module top_tb();


reg clk_ext = 0;
reg reset = 1;
reg SPI_continuous = 0;
reg SPI_start = 1;
reg [31:0] max_timestep = 2;
wire clk;
wire clk_stable;
wire SCLK;
wire CS;
wire [6:0] state_counter;
wire [5:0] channel;
wire [31:0] timestamp;

top_module dut(
    clk_ext, 
    reset,
    SPI_continuous,
    SPI_start,
    max_timestep, 
    clk, 
    clk_stable,
    SCLK, 
    CS, 
    state_counter, 
    channel,
    timestamp);

always #10 clk_ext = ~clk_ext;

initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0,top_tb);
    $display("Testbench started at time %t", $time);
    #50 reset = 1'b0;
    $display("Reset deasserted at time %t", $time);
    @(posedge clk_stable);
    $display("Clock has become stable at time %t", $time);
    #100 SPI_start = 1'b1;
    $display("SPI_start has been asserted at time %t", $time);
    #100 SPI_start = 1'b0;
    $display("SPI_start has been deasserted at time %t", $time);
    #100000;
    $display("Simulation finished at time %t", $time);
    $finish;
end








endmodule