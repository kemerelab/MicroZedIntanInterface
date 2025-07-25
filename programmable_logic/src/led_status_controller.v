// File: led_status_controller.v
// LED status controller that reads status registers and drives LEDs
// LED0: System out of reset (heartbeat)
// LED1: Transmission active

module led_status_controller #(
    parameter integer HEARTBEAT_DIVIDER = 42000000  // For 1Hz blink at 84MHz clock
)(
    // Clock and reset
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *)
    //(* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,
    
    // Status register input (7 registers from data generator)
    input  wire [32*7-1:0] status_regs_pl,
    
    // LED outputs
    (* X_INTERFACE_INFO = "xilinx.com:signal:data:1.0 LED0 DATA" *)
    (* X_INTERFACE_PARAMETER = "LAYERED_METADATA undef" *)
    output reg         led0,    // Heartbeat (system alive)
    
    (* X_INTERFACE_INFO = "xilinx.com:signal:data:1.0 LED1 DATA" *)
    (* X_INTERFACE_PARAMETER = "LAYERED_METADATA undef" *)
    output reg         led1     // Transmission active
);

    // Extract status signals from registers
    wire transmission_active = status_regs_pl[0*32 + 1];  // Bit 1 of status register 0
    wire loop_limit_reached  = status_regs_pl[0*32 + 0];  // Bit 0 of status register 0
    
    // Heartbeat counter for LED0
    reg [$clog2(HEARTBEAT_DIVIDER)-1:0] heartbeat_counter;
    reg heartbeat_toggle;
    
    // Heartbeat generation (1Hz blink when system is alive)
    always @(posedge clk) begin
        if (!rstn) begin
            heartbeat_counter <= 0;
            heartbeat_toggle <= 1'b0;
        end else begin
            if (heartbeat_counter >= HEARTBEAT_DIVIDER - 1) begin
                heartbeat_counter <= 0;
                heartbeat_toggle <= ~heartbeat_toggle;
            end else begin
                heartbeat_counter <= heartbeat_counter + 1;
            end
        end
    end
    
    // LED0: Heartbeat - blinks when system is out of reset
    always @(posedge clk) begin
        if (!rstn) begin
            led0 <= 1'b0;
        end else begin
            led0 <= heartbeat_toggle;
        end
    end
    
    // LED1: Transmission status with different patterns
    always @(posedge clk) begin
        if (!rstn) begin
            led1 <= 1'b0;
        end else begin
            if (transmission_active) begin
                // Solid on when transmitting
                led1 <= 1'b1;
            end else if (loop_limit_reached) begin
                // Fast blink when loop limit reached (completed)
                led1 <= heartbeat_counter < (HEARTBEAT_DIVIDER / 8);  // 1/8 duty cycle, fast blink
            end else begin
                // Off when not transmitting and not completed
                led1 <= 1'b0;
            end
        end
    end

endmodule