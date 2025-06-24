module axi_lite_status #(
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
    
    // Write channels tied off (not used)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWPROT" *)
    input wire [2:0] s_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input wire s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire s_axi_awready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input wire [C_S_AXI_DATA_WIDTH-1:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input wire s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire s_axi_wready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output wire [1:0] s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output wire s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input wire s_axi_bready,
    
    // AXI Lite Slave Interface (Read Only)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input wire [C_S_AXI_ADDR_WIDTH-1:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARPROT" *)
    input wire [2:0] s_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input wire s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output reg s_axi_arready,
    
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output reg [C_S_AXI_DATA_WIDTH-1:0] s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output reg [1:0] s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output reg s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input wire s_axi_rready,
    
    // Status inputs from PL (84MHz domain) - packed array
    input wire [NUM_REGS*8-1:0] pl_status_regs
);

    // AXI Lite state machine states
    localparam IDLE = 2'b00;
    localparam READ_ADDR = 2'b01;
    localparam READ_DATA = 2'b10;
    
    // Internal registers in AXI clock domain
    reg [7:0] axi_status_regs [0:NUM_REGS-1];
    reg [1:0] read_state;
    reg [C_S_AXI_ADDR_WIDTH-1:0] read_addr;
    
    // Clock domain crossing signals
    reg [NUM_REGS-1:0] reg_updated;
    reg [NUM_REGS-1:0] reg_updated_sync1, reg_updated_sync2;
    
    // Tie off write channels (not used in status module)
    assign s_axi_awready = 1'b0;
    assign s_axi_wready = 1'b0;
    assign s_axi_bresp = 2'b11; // DECERR - writes not supported
    assign s_axi_bvalid = 1'b0;
    
    // Generate variables for loops
    genvar j;
    integer i;
    
    // Initialize registers
    initial begin
        for (i = 0; i < NUM_REGS; i = i + 1) begin
            axi_status_regs[i] = 8'h00;
        end
        reg_updated = 0;
    end
    
    // AXI Read State Machine
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            read_state <= IDLE;
            s_axi_arready <= 1'b0;
            s_axi_rvalid <= 1'b0;
            s_axi_rresp <= 2'b00;
            s_axi_rdata <= 32'h00000000;
        end else begin
            case (read_state)
                IDLE: begin
                    s_axi_arready <= 1'b1;
                    s_axi_rvalid <= 1'b0;
                    
                    if (s_axi_arvalid) begin
                        read_addr <= s_axi_araddr;
                        read_state <= READ_ADDR;
                        s_axi_arready <= 1'b0;
                    end
                end
                
                READ_ADDR: begin
                    if (read_addr[7:2] < NUM_REGS) begin
                        // Return 8-bit register in appropriate byte position
                        case (read_addr[1:0])
                            2'b00: s_axi_rdata <= {24'h000000, axi_status_regs[read_addr[7:2]]};
                            2'b01: s_axi_rdata <= {16'h0000, axi_status_regs[read_addr[7:2]], 8'h00};
                            2'b10: s_axi_rdata <= {8'h00, axi_status_regs[read_addr[7:2]], 16'h0000};
                            2'b11: s_axi_rdata <= {axi_status_regs[read_addr[7:2]], 24'h000000};
                        endcase
                        s_axi_rresp <= 2'b00; // OKAY
                    end else begin
                        s_axi_rdata <= 32'h00000000;
                        s_axi_rresp <= 2'b11; // DECERR
                    end
                    
                    s_axi_rvalid <= 1'b1;
                    read_state <= READ_DATA;
                end
                
                READ_DATA: begin
                    if (s_axi_rready) begin
                        s_axi_rvalid <= 1'b0;
                        read_state <= IDLE;
                    end
                end
            endcase
        end
    end
    
    // Clock domain crossing: PL -> AXI (status updates)
    // Use generate block for synthesizable loops
    generate
        for (j = 0; j < NUM_REGS; j = j + 1) begin : axi_update_gen
            always @(posedge s_axi_aclk) begin
                if (!s_axi_aresetn) begin
                    axi_status_regs[j] <= 8'h00;
                end else begin
                    // Update AXI status registers when PL changes detected
                    if (reg_updated_sync2[j]) begin
                        axi_status_regs[j] <= pl_status_regs[j*8+7:j*8];
                    end
                end
            end
        end
    endgenerate
    
    // Two-stage synchronizer in separate always block
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            reg_updated_sync1 <= 0;
            reg_updated_sync2 <= 0;
        end else begin
            reg_updated_sync1 <= reg_updated;
            reg_updated_sync2 <= reg_updated_sync1;
        end
    end
    
    // Status change detection in PL domain
    reg [NUM_REGS*8-1:0] pl_status_regs_prev;
    
    generate
        for (j = 0; j < NUM_REGS; j = j + 1) begin : pl_update_gen
            always @(posedge pl_clk) begin
                if (!pl_rstn) begin
                    reg_updated[j] <= 1'b0;
                    pl_status_regs_prev[j*8+7:j*8] <= 8'h00;
                end else begin
                    // Detect changes in PL status registers
                    if (pl_status_regs[j*8+7:j*8] != pl_status_regs_prev[j*8+7:j*8]) begin
                        reg_updated[j] <= 1'b1;
                        pl_status_regs_prev[j*8+7:j*8] <= pl_status_regs[j*8+7:j*8];
                    end else if (reg_updated_sync2[j]) begin
                        // Clear update flag once synchronized to AXI domain
                        reg_updated[j] <= 1'b0;
                    end
                end
            end
        end
    endgenerate

endmodule