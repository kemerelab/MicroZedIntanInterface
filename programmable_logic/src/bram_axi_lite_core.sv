// File: bram_axi_lite_core.sv
// SystemVerilog core module with single-cycle BRAM read timing
// Optimized for fast BRAM that supports single-cycle reads

module bram_axi_lite_core #(
    parameter int BRAM_ADDR_WIDTH = 16,  // Byte address width
    parameter int BRAM_DATA_WIDTH = 32,  // 32-bit for Port B
    parameter int AXI_DATA_WIDTH = 32,
    parameter int AXI_ADDR_WIDTH = 16
)(
    // Clock and Reset
    input  logic                          clk,
    input  logic                          resetn,
    
    // AXI Lite Write Address Channel
    input  logic [AXI_ADDR_WIDTH-1:0]    awaddr,
    input  logic [2:0]                    awprot,
    input  logic                          awvalid,
    output logic                          awready,
    
    // AXI Lite Write Data Channel
    input  logic [AXI_DATA_WIDTH-1:0]    wdata,
    input  logic [AXI_DATA_WIDTH/8-1:0]  wstrb,
    input  logic                          wvalid,
    output logic                          wready,
    
    // AXI Lite Write Response Channel
    output logic [1:0]                    bresp,
    output logic                          bvalid,
    input  logic                          bready,
    
    // AXI Lite Read Address Channel
    input  logic [AXI_ADDR_WIDTH-1:0]    araddr,
    input  logic [2:0]                    arprot,
    input  logic                          arvalid,
    output logic                          arready,
    
    // AXI Lite Read Data Channel
    output logic [AXI_DATA_WIDTH-1:0]    rdata,
    output logic [1:0]                    rresp,
    output logic                          rvalid,
    input  logic                          rready,
    
    // BRAM Interface (32-bit Port B with byte addressing)
    output logic                          bram_clk,
    output logic                          bram_rst,
    output logic                          bram_en,
    output logic [3:0]                    bram_we,
    output logic [BRAM_ADDR_WIDTH-1:0]    bram_addr,
    output logic [BRAM_DATA_WIDTH-1:0]    bram_din,
    input  logic [BRAM_DATA_WIDTH-1:0]    bram_dout
);

    // AXI response codes
    typedef enum logic [1:0] {
        AXI_OKAY   = 2'b00,
        AXI_EXOKAY = 2'b01,
        AXI_SLVERR = 2'b10,
        AXI_DECERR = 2'b11
    } axi_resp_t;

    // Internal registers
    logic [BRAM_ADDR_WIDTH-1:0]  bram_addr_reg;
    logic                        bram_en_reg;
    logic [AXI_ADDR_WIDTH-1:0]   read_addr_reg;

    // BRAM connections (read-only interface)
    assign bram_clk  = clk;
    assign bram_rst  = ~resetn;
    assign bram_en   = bram_en_reg;
    assign bram_we   = 4'b0000;                    // No writes from this interface
    assign bram_addr = bram_addr_reg;
    assign bram_din  = '0;                         // Tie off unused input

    // AXI Lite Write logic - REJECT ALL WRITES (BRAM is read-only)
    always_ff @(posedge clk) begin : axi_write_logic
        if (!resetn) begin
            awready <= '0;
            wready  <= '0;
            bvalid  <= '0;
            bresp   <= AXI_OKAY;
        end else begin
            // Simple handshake
            awready <= ~awready & awvalid;
            wready  <= ~wready  & wvalid;

            // Complete write transaction (always reject)
            if (awready & awvalid & wready & wvalid) begin
                bresp  <= AXI_SLVERR;    // Reject all writes
                bvalid <= '1;
            end else if (bvalid & bready) begin
                bvalid <= '0;
            end
        end
    end : axi_write_logic

    // Read state machine - optimized for single-cycle BRAM reads
    typedef enum logic [1:0] {
        READ_IDLE,
        READ_RESP
    } read_state_t;
    
    read_state_t read_state;
    
    // AXI Lite Read logic with single-cycle BRAM timing
    always_ff @(posedge clk) begin : axi_read_logic
        if (!resetn) begin
            read_state    <= READ_IDLE;
            arready       <= '0;
            rvalid        <= '0;
            rdata         <= '0;
            rresp         <= AXI_OKAY;
            bram_addr_reg <= '0;
            bram_en_reg   <= '0;
            read_addr_reg <= '0;
        end else begin
            
            case (read_state)
                READ_IDLE: begin
                    // Wait for read request
                    arready <= '1;
                    bram_en_reg <= '0;
                    
                    if (arvalid & arready) begin
                        read_addr_reg <= araddr;
                        arready <= '0;
                        
                        // Start BRAM read immediately - single cycle operation!
                        bram_addr_reg <= araddr;
                        bram_en_reg <= '1;
                        read_state <= READ_RESP;
                    end
                end
                
                READ_RESP: begin
                    // BRAM data is available in single cycle - use direct output
                    bram_en_reg <= '0;
                    rvalid <= '1;
                    rdata <= bram_dout;  // Use direct BRAM output for maximum speed
                    
                    // Check address bounds
                    if (read_addr_reg >= (1 << BRAM_ADDR_WIDTH)) begin
                        rresp <= AXI_SLVERR;
                        rdata <= 32'hDEADBEEF;
                    end else begin
                        rresp <= AXI_OKAY;
                    end
                    
                    // Complete transaction
                    if (rvalid & rready) begin
                        rvalid <= '0;
                        read_state <= READ_IDLE;
                    end
                end
                
                default: begin
                    read_state <= READ_IDLE;
                end
            endcase
        end
    end : axi_read_logic

    // Debug assertions for simulation
    `ifdef SIMULATION
        // Check BRAM address bounds
        property bram_addr_bounds;
            @(posedge clk) disable iff (!resetn)
            bram_en_reg |-> (bram_addr_reg < (1 << BRAM_ADDR_WIDTH));
        endproperty
        
        assert property (bram_addr_bounds) else 
            $error("BRAM address out of bounds: 0x%x", bram_addr_reg);
            
        // Check AXI protocol compliance
        property axi_read_valid_stable;
            @(posedge clk) disable iff (!resetn)
            rvalid && !rready |-> ##1 rvalid && $stable(rdata) && $stable(rresp);
        endproperty
        
        assert property (axi_read_valid_stable) else
            $error("AXI read data must remain stable when valid is high");
            
        // Debug BRAM reads
        always @(posedge clk) begin
            if (bram_en_reg) begin
                $display("BRAM Read: addr=0x%04x", bram_addr_reg);
            end
            if (rvalid && rready) begin
                $display("AXI Response: addr=0x%04x, data=0x%08x", read_addr_reg, rdata);
            end
        end
    `endif

endmodule