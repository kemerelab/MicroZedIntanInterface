module axi_lite_registers #(
    parameter integer N_CTRL = 22,     // Updated for your data generator (22 control regs)
    parameter integer N_STATUS = 7     // Updated for your data generator (7 status regs)
)(
    input  wire                     s_axi_aclk,
    input  wire                     s_axi_aresetn,

    input  wire                     pl_clk,
    input  wire                     pl_rstn,

    // AXI Lite Interface
    input  wire [31:0]              s_axi_awaddr,
    input  wire                     s_axi_awvalid,
    output reg                      s_axi_awready,

    input  wire [31:0]              s_axi_wdata,
    input  wire [3:0]               s_axi_wstrb,
    input  wire                     s_axi_wvalid,
    output reg                      s_axi_wready,

    output reg  [1:0]               s_axi_bresp,
    output reg                      s_axi_bvalid,
    input  wire                     s_axi_bready,

    input  wire [31:0]              s_axi_araddr,
    input  wire                     s_axi_arvalid,
    output reg                      s_axi_arready,

    output reg  [31:0]              s_axi_rdata,
    output reg  [1:0]               s_axi_rresp,
    output reg                      s_axi_rvalid,
    input  wire                     s_axi_rready,

    // Control to PL
    output reg  [32*N_CTRL-1:0]     ctrl_regs_pl,

    // Status from PL (data generator status only)
    input  wire [32*N_STATUS-1:0]   status_regs_pl
    
    // REMOVED: All FIFO-related ports
);

// Internal shadow registers (AXI domain)
reg [31:0] ctrl_regs_axi [0:N_CTRL-1];
reg [31:0] status_regs_axi [0:N_STATUS-1];  // Back to N_STATUS only

// Read address register and read pulse generation
reg [31:0] read_addr;
reg [N_STATUS-1:0] status_read_axi;  // Back to N_STATUS only

// Write state machine
integer i;
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        s_axi_awready <= 0;
        s_axi_wready  <= 0;
        s_axi_bvalid  <= 0;
        s_axi_bresp   <= 2'b00;
        // Initialize control registers
        for (i = 0; i < N_CTRL; i = i + 1)
            ctrl_regs_axi[i] <= 32'b0;
    end else begin
        s_axi_awready <= ~s_axi_awready & s_axi_awvalid;
        s_axi_wready  <= ~s_axi_wready  & s_axi_wvalid;

        if (s_axi_awready & s_axi_awvalid & s_axi_wready & s_axi_wvalid) begin
            if (s_axi_awaddr[11:2] < N_CTRL) begin
                i = s_axi_awaddr[11:2];
                // Proper write strobe handling
                if (s_axi_wstrb[0]) ctrl_regs_axi[i][7:0]   <= s_axi_wdata[7:0];
                if (s_axi_wstrb[1]) ctrl_regs_axi[i][15:8]  <= s_axi_wdata[15:8];
                if (s_axi_wstrb[2]) ctrl_regs_axi[i][23:16] <= s_axi_wdata[23:16];
                if (s_axi_wstrb[3]) ctrl_regs_axi[i][31:24] <= s_axi_wdata[31:24];
                s_axi_bresp <= 2'b00; // OKAY response
            end else begin
                s_axi_bresp <= 2'b10; // SLVERR for invalid address
            end
            s_axi_bvalid <= 1;
        end else if (s_axi_bvalid & s_axi_bready) begin
            s_axi_bvalid <= 0;
        end
    end
end

// AXI Read state machine
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        s_axi_rvalid <= 0;
        s_axi_rdata  <= 32'b0;
        s_axi_rresp  <= 2'b00;
        read_addr    <= 32'b0;
        s_axi_arready <= 0;
        status_read_axi <= 0;
    end else begin
        s_axi_arready <= ~s_axi_arready & s_axi_arvalid;
        
        // Clear read pulses by default
        status_read_axi <= 0;

        if (s_axi_arvalid && s_axi_arready) begin
            read_addr <= s_axi_araddr;
            s_axi_rvalid <= 1;

            if (s_axi_araddr[11:2] < N_CTRL) begin
                // Control registers
                s_axi_rdata <= ctrl_regs_axi[s_axi_araddr[11:2]];
                s_axi_rresp <= 2'b00; // OKAY
            end else if ((s_axi_araddr[11:2] - N_CTRL) < N_STATUS) begin
                // Status registers (data generator only)
                s_axi_rdata <= status_regs_axi[s_axi_araddr[11:2] - N_CTRL];
                s_axi_rresp <= 2'b00; // OKAY
                // Generate read pulse when status register read starts
                status_read_axi[s_axi_araddr[11:2] - N_CTRL] <= 1;
            end else begin
                s_axi_rdata <= 32'hdeadbeef;
                s_axi_rresp <= 2'b10; // SLVERR for invalid address
            end
        end else if (s_axi_rvalid && s_axi_rready) begin
            s_axi_rvalid <= 0;
        end
    end
end

// ============================================================================
// CLOCK DOMAIN CROSSING - CONTROL REGISTERS (AXI -> PL)
// ============================================================================

// Two-stage synchronizer for control registers
reg [31:0] ctrl_sync1 [0:N_CTRL-1];
reg [31:0] ctrl_sync2 [0:N_CTRL-1];

always @(posedge pl_clk) begin
    if (!pl_rstn) begin
        for (i = 0; i < N_CTRL; i = i + 1) begin
            ctrl_sync1[i] <= 32'b0;
            ctrl_sync2[i] <= 32'b0;
        end
    end else begin
        for (i = 0; i < N_CTRL; i = i + 1) begin
            ctrl_sync1[i] <= ctrl_regs_axi[i];  // Cross from AXI to PL domain
            ctrl_sync2[i] <= ctrl_sync1[i];     // Second stage synchronizer
        end
    end
end

// Flatten control registers for output to PL
always @(*) begin
    for (i = 0; i < N_CTRL; i = i + 1)
        ctrl_regs_pl[i*32 +: 32] = ctrl_sync2[i];
end

// ============================================================================
// CLOCK DOMAIN CROSSING - STATUS REGISTERS (PL -> AXI)
// ============================================================================

// First stage: Register inputs in PL domain (data generator only)
reg [31:0] status_pl_reg [0:N_STATUS-1];

always @(posedge pl_clk) begin
    if (!pl_rstn) begin
        for (i = 0; i < N_STATUS; i = i + 1) begin
            status_pl_reg[i] <= 32'b0;
        end
    end else begin
        // Register the PL status inputs in their own domain first
        for (i = 0; i < N_STATUS; i = i + 1) begin
            status_pl_reg[i] <= status_regs_pl[i*32 +: 32];
        end
    end
end

// Second stage: Cross to AXI domain with two-stage synchronizers
reg [31:0] status_sync1 [0:N_STATUS-1];
reg [31:0] status_sync2 [0:N_STATUS-1];

always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        for (i = 0; i < N_STATUS; i = i + 1) begin
            status_regs_axi[i] <= 32'b0;
            status_sync1[i] <= 32'b0;
            status_sync2[i] <= 32'b0;
        end
    end else begin
        // Data generator status registers - need clock crossing
        for (i = 0; i < N_STATUS; i = i + 1) begin
            status_sync1[i] <= status_pl_reg[i];    // Cross clock domain
            status_sync2[i] <= status_sync1[i];     // Second stage
            status_regs_axi[i] <= status_sync2[i];  // Final output
        end
    end
end

endmodule