module axi_lite_registers #(
    parameter integer N_CTRL = 4,
    parameter integer N_STATUS = 4
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

    // Status from PL
    input  wire [32*N_STATUS-1:0]   status_regs_pl
);

// Internal shadow registers (AXI domain)
reg [31:0] ctrl_regs_axi [0:N_CTRL-1];
reg [31:0] status_regs_axi [0:N_STATUS-1];

// Write state
integer i;
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        s_axi_awready <= 0;
        s_axi_wready  <= 0;
        s_axi_bvalid  <= 0;
        s_axi_bresp   <= 2'b00;
    end else begin
        s_axi_awready <= ~s_axi_awready & s_axi_awvalid;
        s_axi_wready  <= ~s_axi_wready  & s_axi_wvalid;

        if (s_axi_awready & s_axi_awvalid & s_axi_wready & s_axi_wvalid) begin
            if (s_axi_awaddr[11:2] < N_CTRL) begin
                i = s_axi_awaddr[11:2];
                if (ctrl_regs_axi[i] !== s_axi_wdata) begin
                    ctrl_regs_axi[i] <= s_axi_wdata;
                end
            end
            s_axi_bvalid <= 1;
            s_axi_bresp  <= 2'b00;
        end else if (s_axi_bvalid & s_axi_bready) begin
            s_axi_bvalid <= 0;
        end
    end
end

// Read state
always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        s_axi_arready <= 0;
        s_axi_rvalid  <= 0;
        s_axi_rresp   <= 2'b00;
        s_axi_rdata   <= 32'd0;
    end else begin
        s_axi_arready <= ~s_axi_arready & s_axi_arvalid;

        if (s_axi_arready & s_axi_arvalid) begin
            if (s_axi_araddr[11:2] < N_CTRL) begin
                s_axi_rdata <= ctrl_regs_axi[s_axi_araddr[11:2]];
            end else if ((s_axi_araddr[11:2] - N_CTRL) < N_STATUS) begin
                s_axi_rdata <= status_regs_axi[s_axi_araddr[11:2] - N_CTRL];
            end else begin
                s_axi_rdata <= 32'hdeadbeef;
            end
            s_axi_rvalid <= 1;
            s_axi_rresp  <= 2'b00;
        end else if (s_axi_rvalid & s_axi_rready) begin
            s_axi_rvalid <= 0;
        end
    end
end

// CDC for control regs: AXI -> PL
reg [31:0] ctrl_sync1 [0:N_CTRL-1];
reg [31:0] ctrl_sync2 [0:N_CTRL-1];

always @(posedge pl_clk) begin
    if (!pl_rstn) begin
        for (i = 0; i < N_CTRL; i = i + 1) begin
            ctrl_sync1[i] <= 0;
            ctrl_sync2[i] <= 0;
        end
    end else begin
        for (i = 0; i < N_CTRL; i = i + 1) begin
            ctrl_sync1[i] <= ctrl_regs_axi[i];
            ctrl_sync2[i] <= ctrl_sync1[i];
        end
    end
end

// Flatten for output
always @(*) begin
    for (i = 0; i < N_CTRL; i = i + 1)
        ctrl_regs_pl[i*32 +: 32] = ctrl_sync2[i];
end

// CDC for status regs: PL -> AXI
reg [31:0] status_sync1 [0:N_STATUS-1];
reg [31:0] status_sync2 [0:N_STATUS-1];

always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
        for (i = 0; i < N_STATUS; i = i + 1) begin
            status_sync1[i] <= 0;
            status_sync2[i] <= 0;
        end
    end else begin
        for (i = 0; i < N_STATUS; i = i + 1) begin
            status_sync1[i] <= status_regs_pl[i*32 +: 32];
            status_sync2[i] <= status_sync1[i];
            status_regs_axi[i] <= status_sync2[i];
        end
    end
end

endmodule
