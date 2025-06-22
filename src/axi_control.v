module axi_lite_control_reg (
    // AXI Lite Slave Interface (PS domain - 100MHz)
    input  wire        s_axi_aclk,
    input  wire        s_axi_aresetn,
    
    // PL domain clock and reset (84MHz)
    input  wire        pl_clk,
    input  wire        pl_rstn,
    
    // Write Address Channel
    input  wire [31:0] s_axi_awaddr,
    input  wire [2:0]  s_axi_awprot,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    
    // Write Data Channel
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    
    // Write Response Channel
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    
    // Read Address Channel
    input  wire [31:0] s_axi_araddr,
    input  wire [2:0]  s_axi_arprot,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    
    // Read Data Channel
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,
    
    // PL domain interfaces
    output reg  [31:0] control_reg_pl,    // Full 32-bit control register in PL domain
    input  wire [31:0] status_reg_pl      // Status from PL domain
);

// Interface attributes for Vivado IP integration
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR" *)
(* X_INTERFACE_PARAMETER = "DATA_WIDTH 32, PROTOCOL AXI4LITE, FREQ_HZ 100000000, ID_WIDTH 0, ADDR_WIDTH 32, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE read_write, HAS_BURST 0, HAS_LOCK 0, HAS_PROT 1, HAS_CACHE 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, SUPPORTS_NARROW_BURST 0, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, PHASE 0.0, CLK_DOMAIN design_1_processing_system7_0_0_FCLK_CLK0, NUM_READ_THREADS 1, NUM_WRITE_THREADS 1, RUSER_BITS_PER_BYTE 0, WUSER_BITS_PER_BYTE 0" *)
wire [31:0] axi_awaddr;

// Register map
localparam ADDR_CONTROL_REG = 4'h0;  // Offset 0x0: Control register
localparam ADDR_STATUS_REG  = 4'h4;  // Offset 0x4: Status register

// Internal registers (AXI domain)
reg [31:0] control_reg_axi;
reg [31:0] status_reg_axi;

// Clock domain crossing registers
// Control path: AXI domain (100MHz) -> PL domain (84MHz)
reg [31:0] control_sync1, control_sync2;

// Status path: PL domain (84MHz) -> AXI domain (100MHz) 
reg [31:0] status_sync1, status_sync2;

// AXI write state machine
localparam WRITE_IDLE = 2'b00;
localparam WRITE_DATA = 2'b01;
localparam WRITE_RESP = 2'b10;

reg [1:0] write_state;

// AXI read state machine  
localparam READ_IDLE = 2'b00;
localparam READ_DATA = 2'b01;

reg [1:0] read_state;

// Address latches
reg [31:0] write_addr;
reg [31:0] read_addr;

// Clock domain crossing: AXI domain (100MHz) -> PL domain (84MHz)
// Double-flop synchronizer for control register
always @(posedge pl_clk) begin
    if (!pl_rstn) begin
        control_sync1 <= 32'h0;
        control_sync2 <= 32'h0;
        control_reg_pl <= 32'h0;
    end else begin
        control_sync1 <= control_reg_axi;
        control_sync2 <= control_sync1;
        control_reg_pl <= control_sync2;
    end
end

// Clock domain crossing: PL domain (84MHz) -> AXI domain (100MHz)
// Double-flop synchronizer for status register
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        status_sync1 <= 32'h0;
        status_sync2 <= 32'h0;
        status_reg_axi <= 32'h0;
    end else begin
        status_sync1 <= status_reg_pl;
        status_sync2 <= status_sync1;
        status_reg_axi <= status_sync2;
    end
end

// AXI Lite interface logic
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        // Reset all registers and state machines
        write_state <= WRITE_IDLE;
        read_state <= READ_IDLE;
        s_axi_awready <= 1'b0;
        s_axi_wready <= 1'b0;
        s_axi_bvalid <= 1'b0;
        s_axi_bresp <= 2'b00;
        s_axi_arready <= 1'b0;
        s_axi_rvalid <= 1'b0;
        s_axi_rresp <= 2'b00;
        s_axi_rdata <= 32'h0;
        
        // Reset control register (AXI domain)
        control_reg_axi <= 32'h0;
        
        write_addr <= 32'h0;
        read_addr <= 32'h0;
        
    end else begin
        
        // AXI Write State Machine
        case (write_state)
            WRITE_IDLE: begin
                if (s_axi_awvalid && s_axi_wvalid) begin
                    // Both address and data are valid
                    s_axi_awready <= 1'b1;
                    s_axi_wready <= 1'b1;
                    write_addr <= s_axi_awaddr;
                    write_state <= WRITE_DATA;
                end
            end
            
            WRITE_DATA: begin
                s_axi_awready <= 1'b0;
                s_axi_wready <= 1'b0;
                
                // Perform the write based on address
                case (write_addr[3:0])
                    ADDR_CONTROL_REG: begin
                        // Write to control register (only control reg is writable)
                        if (s_axi_wstrb[0]) control_reg_axi[7:0]   <= s_axi_wdata[7:0];
                        if (s_axi_wstrb[1]) control_reg_axi[15:8]  <= s_axi_wdata[15:8];
                        if (s_axi_wstrb[2]) control_reg_axi[23:16] <= s_axi_wdata[23:16];
                        if (s_axi_wstrb[3]) control_reg_axi[31:24] <= s_axi_wdata[31:24];
                    end
                    ADDR_STATUS_REG: begin
                        // Status register is read-only, ignore writes
                        // Could optionally return an error response here
                    end
                endcase
                
                s_axi_bvalid <= 1'b1;
                s_axi_bresp <= 2'b00; // OKAY response
                write_state <= WRITE_RESP;
            end
            
            WRITE_RESP: begin
                if (s_axi_bready) begin
                    s_axi_bvalid <= 1'b0;
                    write_state <= WRITE_IDLE;
                end
            end
        endcase
        
        // AXI Read State Machine
        case (read_state)
            READ_IDLE: begin
                if (s_axi_arvalid) begin
                    s_axi_arready <= 1'b1;
                    read_addr <= s_axi_araddr;
                    read_state <= READ_DATA;
                end
            end
            
            READ_DATA: begin
                s_axi_arready <= 1'b0;
                
                // Read data based on address
                case (read_addr[3:0])
                    ADDR_CONTROL_REG: s_axi_rdata <= control_reg_axi;
                    ADDR_STATUS_REG:  s_axi_rdata <= status_reg_axi;  // Synchronized from PL
                    default:          s_axi_rdata <= 32'hDEADBEEF;   // Invalid address
                endcase
                
                s_axi_rvalid <= 1'b1;
                s_axi_rresp <= 2'b00; // OKAY response
                
                if (s_axi_rready) begin
                    s_axi_rvalid <= 1'b0;
                    read_state <= READ_IDLE;
                end
            end
        endcase
    end
end

endmodule