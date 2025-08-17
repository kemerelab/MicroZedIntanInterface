// File: data_generator_bram_blk.v
// Clean Verilog wrapper that combines data generator and FIFO-BRAM interface
// Now with proper parameter support and serial interface


module data_generator #(
    // BRAM configuration parameters
    parameter integer BRAM_ADDR_WIDTH = 16,        // Byte address width  
    parameter integer BRAM_DATA_WIDTH = 32,        // Data width
    parameter integer BRAM_DEPTH_WORDS = 16384,   // BRAM depth in words (64KB / 4 = 16K words)
    parameter integer FIFO_DEPTH = 256            // FIFO depth (entries)
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *)
    //(* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,
    
    // Control and status interfaces
    input  wire [32*22-1:0] ctrl_regs_pl,
    output wire [32*11-1:0]  status_regs_pl,
    
    // BRAM Port A interface (32-bit)
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA CLK" *)
    output wire            bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA RST" *)
    output wire            bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA ADDR" *)
    output wire [BRAM_ADDR_WIDTH-1:0] bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DIN" *)
    output wire [BRAM_DATA_WIDTH-1:0] bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA DOUT" *)
    input  wire [BRAM_DATA_WIDTH-1:0] bram_dout,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA EN" *)
    output wire            bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 BRAM_PORTA WE" *)
    output wire [3:0]      bram_we,
    
    
    // Serial interface signals
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi csn" *)
    output wire            csn,         // Chip select (active low)
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi sclk" *)
    output wire            sclk,        // Serial clock (84MHz/4 = 24MHz)
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi copi" *)
    output wire            copi,         // Controller Out, Peripheral In
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo0" *)
    input  wire        cipo0,      // Controller In, Peripheral Out 0

    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo1" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF intan_spi, ASSOCIATED_RESET rst_n, ASSOCIATED_CLKEN clk" *)
    input  wire        cipo1       // Controller In, Peripheral Out 1

);

    // Parameter validation
    initial begin
        if (BRAM_DEPTH_WORDS > (1 << (BRAM_ADDR_WIDTH - 2))) begin
            $error("BRAM_DEPTH_WORDS (%d) exceeds address space (%d words)", 
                   BRAM_DEPTH_WORDS, (1 << (BRAM_ADDR_WIDTH - 2)));
        end
        if (FIFO_DEPTH < 144) begin  // Hardcode the packet size check
            $warning("FIFO_DEPTH (%d) is smaller than typical packet size (144 words) - may cause flow control issues", 
                     FIFO_DEPTH);
        end
    end

    // Internal signals for connecting modules
    
    // FIFO interface signals
    wire        fifo_write_en;
    wire [31:0] fifo_write_data;
    wire        fifo_full;
    wire [8:0]  fifo_count;
    wire [13:0] current_bram_address;
    
    // Data generator status (only 6 registers - wrapper adds 7th)
    wire [32*10-1:0] data_gen_status;

    // Instantiate the data generator core
    data_generator_core data_gen_inst (
        .clk(clk),
        .rstn(rstn),
        .ctrl_regs_pl(ctrl_regs_pl),
        .status_regs_pl(data_gen_status),  // Only 10 registers
        
        // FIFO interface
        .fifo_write_en(fifo_write_en),
        .fifo_write_data(fifo_write_data),
        .fifo_full(fifo_full),
        .fifo_count(fifo_count),
        
        // Serial interface
        .csn(csn),
        .sclk(sclk),
        .copi(copi),
        .cipo0(cipo0),
        .cipo1(cipo1)
    );

    // Instantiate the FIFO-BRAM interface
    fifo_bram_interface #(
        .BRAM_ADDR_WIDTH(BRAM_ADDR_WIDTH),
        .BRAM_DATA_WIDTH(BRAM_DATA_WIDTH),
        .BRAM_DEPTH_WORDS(BRAM_DEPTH_WORDS),
        .FIFO_DEPTH(FIFO_DEPTH)
    ) fifo_bram_inst (
        .clk(clk),
        .rstn(rstn),
        
        // FIFO interface
        .fifo_write_en(fifo_write_en),
        .fifo_write_data(fifo_write_data),
        .fifo_full(fifo_full),
        .fifo_count(fifo_count),
        .current_bram_address(current_bram_address),
        
        // BRAM interface
        .bram_addr(bram_addr),
        .bram_din(bram_din),
        .bram_en(bram_en),
        .bram_we(bram_we),
        .bram_clk(bram_clk),
        .bram_rst(bram_rst)
    );
    
    // Combine status registers in wrapper
    // Clean separation: data generator owns 0-5, wrapper adds FIFO/BRAM status as 6
    assign status_regs_pl[0*32 +: 32] = data_gen_status[0*32 +: 32];  // Generator status 0 (includes clock_counter)
    assign status_regs_pl[1*32 +: 32] = data_gen_status[1*32 +: 32];  // Generator status 1  
    assign status_regs_pl[2*32 +: 32] = data_gen_status[2*32 +: 32];  // Generator status 2
    assign status_regs_pl[3*32 +: 32] = data_gen_status[3*32 +: 32];  // Generator status 3
    assign status_regs_pl[4*32 +: 32] = data_gen_status[4*32 +: 32];  // Generator status 4
    assign status_regs_pl[5*32 +: 32] = data_gen_status[5*32 +: 32];  // Generator status 5
    assign status_regs_pl[6*32 +: 32] = data_gen_status[6*32 +: 32];  // Generator status 5
    assign status_regs_pl[7*32 +: 32] = data_gen_status[7*32 +: 32];  // Generator status 5
    assign status_regs_pl[8*32 +: 32] = data_gen_status[8*32 +: 32];  // Generator status 5
    assign status_regs_pl[9*32 +: 32] = data_gen_status[9*32 +: 32];  // Generator status 5

    assign status_regs_pl[10*32 +: 32] = {9'd0, fifo_count, current_bram_address}; // FIFO + BRAM status

endmodule