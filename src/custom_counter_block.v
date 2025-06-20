module custom_bram_interface (
    input  wire clk,
    input  wire rstn,

    // BRAM Port B interface
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB CLK" *)
    (* X_INTERFACE_PARAMETER = "MASTER_TYPE BRAM_CTRL" *)
    output wire bram_clk,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB RST" *)
    output wire bram_rst,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB EN" *)
    output reg bram_en,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB WE" *)
    output reg [3:0] bram_we,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB ADDR" *)
    output reg [31:0] bram_addr,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB DIN" *)
    output reg [31:0] bram_din,

    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTB DOUT" *)
    input  wire [31:0] bram_dout
);

    // Clock and reset passthrough
    assign bram_clk = clk;
    assign bram_rst = ~rstn;

    // Clock divider for 2.625 MHz tick (84 MHz / 32)
    reg [4:0] clk_div = 0;
    wire counter_tick = (clk_div == 5'd31); // 2.625 MHz
    
    always @(posedge clk or negedge rstn) begin
        if (!rstn)
            clk_div <= 5'd0;
        else
            clk_div <= clk_div + 1;
    end

    // Counter logic (separate block)
    reg [31:0] counter = 0;
    reg [31:0] enable_counter = 0;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            counter <= 32'd0;
        end else if (counter_tick && enable_counter != 32'd0) begin
            counter <= counter + 1;
        end
    end

    // FSM for BRAM interface
    reg [2:0] state;
    localparam STATE_IDLE       = 3'd0;
    localparam STATE_READ       = 3'd1;
    localparam STATE_WAIT       = 3'd2;
    localparam STATE_PREP_WRITE = 3'd3;
    localparam STATE_WRITE      = 3'd4;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state         <= STATE_IDLE;
            bram_en       <= 1'b0;
            bram_we       <= 4'b0000;
            bram_addr     <= 32'h00000000;
            bram_din      <= 32'd0;
            enable_counter <= 32'd0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    bram_en   <= 1'b1;
                    bram_we   <= 4'b0000;
                    bram_addr <= 32'h00000000; // Read enable register
                    state     <= STATE_READ;
                end

                STATE_READ: begin
                    state <= STATE_WAIT;
                end

                STATE_WAIT: begin
                    enable_counter <= bram_dout; // Capture enable value
                    state <= STATE_PREP_WRITE;
                end

                STATE_PREP_WRITE: begin
                    bram_din  <= counter;        // Write current counter value
                    bram_en   <= 1'b1;
                    bram_we   <= 4'b1111;
                    bram_addr <= 32'h00000004;
                    state     <= STATE_WRITE;
                end

                STATE_WRITE: begin
                    bram_en <= 1'b0;
                    bram_we <= 4'b0000;
                    state   <= STATE_IDLE;
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

endmodule
