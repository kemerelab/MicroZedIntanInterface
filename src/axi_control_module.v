module axi_lite_control #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 8,
    parameter NUM_REGS = 64
)(
    // PS Clock Domain - 100MHz
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 100000000" *)
    input wire s_axi_aclk,
    
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire s_axi_aresetn,
    
    // PL Clock Domain - 84MHz  
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 pl_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input wire pl_clk,
    
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 pl_rstn RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input wire pl_rstn,
    
    // AXI Lite Slave Interface (Write Only)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
    input wire [2:0] s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input wire s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output reg s_axi_awready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input wire s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output reg s_axi_wready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output reg [1:0] s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output reg s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input wire s_axi_bready,
    
    // Read channels tied off (not used)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
    input wire [2:0] s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input wire s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire s_axi_arready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output wire [1:0] s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output wire s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input wire s_axi_rready,
    
    // Control outputs to PL (84MHz domain) - packed array
    output reg [NUM_REGS*8-1:0] pl_control_regs
);

    // AXI Lite state machine states
    localparam IDLE = 2'b00;
    localparam WRITE_ADDR = 2'b01;
    localparam WRITE_DATA = 2'b10;
    localparam WRITE_RESP = 2'b11;
    
    // Internal registers in AXI clock domain
    reg [7:0] axi_control_regs [0:NUM_REGS-1];
    reg [1:0] write_state;
    reg [C_S_AXI_ADDR_WIDTH-1:0] write_addr;
    
    // Clock domain crossing signals
    reg [NUM_REGS-1:0] reg_updated;
    reg [NUM_REGS-1:0] reg_updated_sync1, reg_updated_sync2;
    reg [NUM_REGS-1:0] reg_ack_sync1, reg_ack_sync2;
    
    // Tie off read channels (not used in control module)
    assign s_axi_arready = 1'b0;
    assign s_axi_rdata = 32'h00000000;
    assign s_axi_rresp = 2'b11; // DECERR - reads not supported
    assign s_axi_rvalid = 1'b0;
    
    // Generate variables for loops
    genvar j;
    integer i;
    
    // Initialize registers
    initial begin
        for (i = 0; i < NUM_REGS; i = i + 1) begin
            axi_control_regs[i] = 8'h00;
        end
        pl_control_regs = 0;
        reg_updated = 0;
    end
    
    // AXI Write State Machine
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            write_state <= IDLE;
            s_axi_awready <= 1'b0;
            s_axi_wready <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_bresp <= 2'b00;
            reg_updated <= 0;
        end else begin
            case (write_state)
                IDLE: begin
                    s_axi_awready <= 1'b1;
                    s_axi_wready <= 1'b1;
                    s_axi_bvalid <= 1'b0;
                    
                    if (s_axi_awvalid && s_axi_wvalid) begin
                        write_addr <= s_axi_awaddr;
                        write_state <= WRITE_DATA;
                        s_axi_awready <= 1'b0;
                        s_axi_wready <= 1'b0;
                    end
                end
                
                WRITE_DATA: begin
                    // Write to control register array
                    if (write_addr[7:2] < NUM_REGS) begin
                        // Handle byte strobes for 32-bit writes to 8-bit registers
                        case (write_addr[1:0])
                            2'b00: if (s_axi_wstrb[0]) begin
                                axi_control_regs[write_addr[7:2]] <= s_axi_wdata[7:0];
                                reg_updated[write_addr[7:2]] <= 1'b1;
                            end
                            2'b01: if (s_axi_wstrb[1]) begin
                                axi_control_regs[write_addr[7:2]] <= s_axi_wdata[15:8];
                                reg_updated[write_addr[7:2]] <= 1'b1;
                            end
                            2'b10: if (s_axi_wstrb[2]) begin
                                axi_control_regs[write_addr[7:2]] <= s_axi_wdata[23:16];
                                reg_updated[write_addr[7:2]] <= 1'b1;
                            end
                            2'b11: if (s_axi_wstrb[3]) begin
                                axi_control_regs[write_addr[7:2]] <= s_axi_wdata[31:24];
                                reg_updated[write_addr[7:2]] <= 1'b1;
                            end
                        endcase
                        s_axi_bresp <= 2'b00; // OKAY
                    end else begin
                        s_axi_bresp <= 2'b11; // DECERR
                    end
                    
                    write_state <= WRITE_RESP;
                    s_axi_bvalid <= 1'b1;
                end
                
                WRITE_RESP: begin
                    if (s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        write_state <= IDLE;
                    end
                end
            endcase
            
            // Clear update flags when acknowledged
            reg_updated <= reg_updated & ~reg_ack_sync2;
        end
    end
    
    // Two-stage synchronizer in separate always block
    always @(posedge pl_clk) begin
        if (!pl_rstn) begin
            reg_updated_sync1 <= 0;
            reg_updated_sync2 <= 0;
        end else begin
            reg_updated_sync1 <= reg_updated;
            reg_updated_sync2 <= reg_updated_sync1;
        end
    end
    
    // Clock domain crossing: AXI -> PL (use generate blocks)
    generate
        for (j = 0; j < NUM_REGS; j = j + 1) begin : pl_update_gen
            always @(posedge pl_clk) begin
                if (!pl_rstn) begin
                    pl_control_regs[j*8+7:j*8] <= 8'h00;
                    reg_ack_sync1[j] <= 1'b0;
                end else begin
                    // Update PL control registers when change detected
                    if (reg_updated_sync2[j] && !reg_ack_sync1[j]) begin
                        pl_control_regs[j*8+7:j*8] <= axi_control_regs[j];
                        reg_ack_sync1[j] <= 1'b1;
                    end else if (!reg_updated_sync2[j]) begin
                        reg_ack_sync1[j] <= 1'b0;
                    end
                end
            end
        end
    endgenerate
    
    // Clock domain crossing: PL -> AXI (acknowledgment)
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            reg_ack_sync2 <= 0;
        end else begin
            reg_ack_sync2 <= reg_ack_sync1;
        end
    end

endmodule