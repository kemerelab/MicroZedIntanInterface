// File: data_generator_bram_blk.v
// Clean Verilog wrapper that combines data generator and FIFO-BRAM interface

module data_generator #(
    // BRAM configuration parameters
    parameter integer BRAM_ADDR_WIDTH = 16,        // Byte address width
    parameter integer BRAM_DATA_WIDTH = 32,        // Data width
    parameter integer BRAM_DEPTH_WORDS = 16384,   // BRAM depth in words (64KB / 4 = 16K words)
    // Wavelet (Tier-3) results BRAM: 64 KB = 16384 words -> 16-bit byte address
    // (the wrapper default). Holds the full wire packet for K up to 127 (K=128 =
    // 8200 words also fits); the v2 2-MAC + work-spread engine tops out near
    // K=128, so 64 KB is ample. (The k256 WIP tried 128 KB but the wrapper's
    // read-only MEM_SIZE attribute broke validate_bd_design -- reverted.)
    parameter integer WAV_BRAM_ADDR_WIDTH = 16,    // 64 KB wavelet result BRAM
    parameter integer FIFO_DEPTH = 256,           // FIFO depth (64-bit entries)
    parameter integer BUFFER_DEPTH = 16           // Segment buffer depth for selective copying
)(
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 CLK CLK" *)
    //(* X_INTERFACE_PARAMETER = "FREQ_HZ 84000000" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 RST RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rstn,
    
    // Control and status interfaces
    // Widths must match axi_lite_registers (N_CTRL=32, N_STATUS=15). Control
    // regs 0..21 are the legacy map; 22..24 configure the aux command
    // sequencer / override layer; 25..27 configure the LFP/DSP engine; 28..31
    // configure the Tier-3 wavelet scalogram engine. Status 11 = aux status,
    // 12 = read result, 13 = LFP, 14 = wavelet.
    input  wire [32*32-1:0] ctrl_regs_pl,
    output wire [32*15-1:0]  status_regs_pl,
    
    // Digital input (eventually should add analog input here!)
    input  wire [7:0]  digital_in,

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

    // LFP output BRAM Port A (PL writes the decimated stream; PS reads it via a
    // second axi_bram_ctrl mapped at 0x84000000).
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM CLK" *)
    output wire            lfp_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM RST" *)
    output wire            lfp_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM ADDR" *)
    output wire [BRAM_ADDR_WIDTH-1:0] lfp_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM DIN" *)
    output wire [BRAM_DATA_WIDTH-1:0] lfp_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM DOUT" *)
    input  wire [BRAM_DATA_WIDTH-1:0] lfp_bram_dout,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM EN" *)
    output wire            lfp_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 LFP_BRAM WE" *)
    output wire [3:0]      lfp_bram_we,

    // Wavelet (Tier-3) results BRAM Port A (PL writes the scalogram columns;
    // PS reads via a third axi_bram_ctrl mapped at 0x90000000).
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM CLK" *)
    output wire            wav_bram_clk,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM RST" *)
    output wire            wav_bram_rst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM ADDR" *)
    output wire [WAV_BRAM_ADDR_WIDTH-1:0] wav_bram_addr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM DIN" *)
    output wire [BRAM_DATA_WIDTH-1:0] wav_bram_din,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM DOUT" *)
    input  wire [BRAM_DATA_WIDTH-1:0] wav_bram_dout,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM EN" *)
    output wire            wav_bram_en,
    (* X_INTERFACE_INFO = "xilinx.com:interface:bram:1.0 WAV_BRAM WE" *)
    output wire [3:0]      wav_bram_we,

    // Serial interface signals
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi csn" *)
    output wire            csn,         // Chip select (active low)
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi sclk" *)
    output wire            sclk,        // Serial clock (84MHz/4 = 24MHz)
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi copi" *)
    output wire            copi,         // Controller Out, Peripheral In
    
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo0" *)
    input  wire        cipo0,      // Port A: Controller In, Peripheral Out 0

    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi cipo1" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF intan_spi, ASSOCIATED_RESET rst_n, ASSOCIATED_CLKEN clk" *)
    input  wire        cipo1,      // Port A: Controller In, Peripheral Out 1

    // Second SPI port (cable B). The master signals are the SAME logical
    // csn/sclk/copi broadcast to both ports (common command set); only the
    // CIPO return lines differ. Exposed as a second intan_spi bus so the block
    // design can drop a second (identical) LVDS buffer onto port-B's pins.
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi_b csn" *)
    output wire            csn_b,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi_b sclk" *)
    output wire            sclk_b,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi_b copi" *)
    output wire            copi_b,
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi_b cipo0" *)
    input  wire        cipo2,      // Port B: Controller In, Peripheral Out 0
    (* X_INTERFACE_INFO = "kemerelab.org:intan:intan_spi:1.0 intan_spi_b cipo1" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF intan_spi_b" *)
    input  wire        cipo3       // Port B: Controller In, Peripheral Out 1

);

    // Parameter validation
    initial begin
        if (BRAM_DEPTH_WORDS > (1 << (BRAM_ADDR_WIDTH - 2))) begin
            $error("BRAM_DEPTH_WORDS (%d) exceeds address space (%d words)", 
                   BRAM_DEPTH_WORDS, (1 << (BRAM_ADDR_WIDTH - 2)));
        end
        // Note: Packet size is variable depending on channel enable settings
        // Minimum: 2 header + 17 data words (if only 1 channel active) = 19 x 64-bit words 
        // Maximum: 2 header + 35 data words (if all channels active) = 37 x 64-bit words
        if (FIFO_DEPTH < 37) begin  
            $warning("FIFO_DEPTH (%d) is smaller than maximum packet size (37 x 64-bit words) - may cause flow control issues", 
                     FIFO_DEPTH);
        end
        if ((BUFFER_DEPTH & (BUFFER_DEPTH - 1)) != 0) begin
            $error("BUFFER_DEPTH (%d) must be power of 2", BUFFER_DEPTH);
        end
        if (BUFFER_DEPTH < 16) begin
            $warning("BUFFER_DEPTH (%d) may be too small for worst-case buffering", BUFFER_DEPTH);
        end
    end

    // Internal signals for connecting modules
    
    // FIFO interface signals
    wire        fifo_write_en;
    wire [127:0] fifo_write_data;       // 128-bit: up to 8 x 16-bit segments (2 ports)
    wire [7:0]  fifo_channel_mask;      // Channel metadata for selective copying
    wire        fifo_packet_end_flag;   // Channel metadata for tagging packets
    wire        fifo_full;
    wire [8:0]  fifo_count;
    wire [13:0] current_bram_address;

    // DSP tap from the core to the LFP engine
    wire         dsp_sample_valid;
    wire [127:0] dsp_sample_data;
    wire [5:0]   dsp_sample_slot;
    wire         dsp_packet_tick;
    wire [63:0]  dsp_master_timestamp;   // live master sample count (LFP frame stamp)
    wire [7:0]   dsp_channel_enable;     // broadband mask -> LFP lane_mask (single source)
    wire [15:0]  lfp_wr_addr;
    wire         lfp_overrun;

    // Decimated LFP output stream tap -> the Tier-3 wavelet engine.
    wire         lfp_out_valid;
    wire [7:0]   lfp_out_channel;     // $clog2(8*32) = 8
    wire signed [15:0] lfp_out_data;
    wire         lfp_out_frame_start;
    // Wavelet engine status
    wire [31:0]  wav_frame_seq;
    wire         wav_busy, wav_overrun;

    // Data generator status (only 10 registers - wrapper adds 11th..13th)
    wire [32*10-1:0] data_gen_status;
    wire [31:0] aux_status;
    wire [31:0] aux_read_result;

    // Instantiate the data generator core
    data_generator_core data_gen_inst (
        .clk(clk),
        .rstn(rstn),
        .ctrl_regs_pl(ctrl_regs_pl[32*25-1:0]),   // core uses regs 0..24 only
        .status_regs_pl(data_gen_status),  // Only 10 registers
        .aux_status(aux_status),
        .aux_read_result(aux_read_result),
        
        // FIFO interface
        .fifo_write_en(fifo_write_en),
        .fifo_write_data(fifo_write_data),          // 128-bit data
        .fifo_channel_mask(fifo_channel_mask),      // 8-bit channel metadata
        .fifo_full(fifo_full),
        .fifo_count(fifo_count),                    // Count of entries
        .fifo_packet_end_flag(fifo_packet_end_flag),

        // Serial interface
        .csn(csn),
        .sclk(sclk),
        .copi(copi),
        .cipo0(cipo0),
        .cipo1(cipo1),
        // Port B (cable B) CIPO inputs, from the second LVDS buffer.
        .cipo2(cipo2),
        .cipo3(cipo3),

        // Digital input
        .digital_in(digital_in),

        // DSP tap to the LFP engine
        .dsp_sample_valid(dsp_sample_valid),
        .dsp_sample_data(dsp_sample_data),
        .dsp_sample_slot(dsp_sample_slot),
        .dsp_packet_tick(dsp_packet_tick),
        .dsp_master_timestamp(dsp_master_timestamp),
        .dsp_channel_enable(dsp_channel_enable)
    );

    // Instantiate the on-PL LFP/DSP engine (control regs 25..27; writes its own
    // output BRAM read by the PS via a 2nd axi_bram_ctrl).
    lfp_dsp_block #(
        .LFP_BRAM_AW(BRAM_ADDR_WIDTH)
    ) lfp_dsp_inst (
        .clk(clk),
        .rstn(rstn),
        .dsp_sample_valid(dsp_sample_valid),
        .dsp_sample_data(dsp_sample_data),
        .dsp_sample_slot(dsp_sample_slot),
        .dsp_packet_tick(dsp_packet_tick),
        // Master timestamp tap (frame stamp) + broadband mask (LFP lane_mask source).
        .dsp_master_timestamp(dsp_master_timestamp),
        .dsp_channel_enable(dsp_channel_enable),
        .lfp_cfg(ctrl_regs_pl[25*32 +: 32]),
        .lfp_coef(ctrl_regs_pl[26*32 +: 32]),
        .lfp_strobe(ctrl_regs_pl[27*32 +: 32]),
        .bram_clk(lfp_bram_clk),
        .bram_rst(lfp_bram_rst),
        .bram_addr(lfp_bram_addr),
        .bram_din(lfp_bram_din),
        .bram_dout(lfp_bram_dout),
        .bram_en(lfp_bram_en),
        .bram_we(lfp_bram_we),
        .lfp_wr_addr(lfp_wr_addr),
        .lfp_overrun(lfp_overrun),
        // decimated output stream tap (signed) -> wavelet engine
        .lfp_out_valid(lfp_out_valid),
        .lfp_out_channel(lfp_out_channel),
        .lfp_out_data(lfp_out_data),
        .lfp_out_frame_start(lfp_out_frame_start)
    );

    // Instantiate the Tier-3 on-PL wavelet scalogram engine (control regs
    // 28..31; writes its own results BRAM read by the PS via a 3rd
    // axi_bram_ctrl mapped at 0x90000000).
    //
    // v2 STEP 1 -- 2 MAC lanes: the voice MAC now computes the RE and IM parts
    // of a tap in ONE cycle (lane A=re, lane B=im, shared ring read), halving
    // the worst-case (all-octaves-coincide) pass from ~1758*K to ~990*K clocks.
    // Worst-case overrun-TB measurements (8 oct/4 voc/24 tap, fcount=0 frame):
    //   K=16 -> 15847 clk (CLEAN, < 28000 budget),  K=32 -> 31687 (1.13x over).
    // So the STEP-1 real-time-clean ceiling is K=16. The result BRAM stays
    // 128 KB (RES_AW=17) -- room for a larger K once STEP-2 work-spread lands.
    wavelet_dsp_block #(
        .N_CH(256), .K(16), .N_OCTAVES(8), .V(4), .N_TAPS(24), .HB_TAPS(7),
        .RES_AW(WAV_BRAM_ADDR_WIDTH)
    ) wav_dsp_inst (
        .clk(clk),
        .rstn(rstn),
        .lfp_out_valid(lfp_out_valid),
        .lfp_out_channel(lfp_out_channel),
        .lfp_out_data(lfp_out_data),
        .lfp_frame_start(lfp_out_frame_start),
        .wav_cfg(ctrl_regs_pl[28*32 +: 32]),
        .wav_gain(ctrl_regs_pl[29*32 +: 32]),
        .wav_data(ctrl_regs_pl[30*32 +: 32]),
        .wav_strobe(ctrl_regs_pl[31*32 +: 32]),
        .bram_clk(wav_bram_clk),
        .bram_rst(wav_bram_rst),
        .bram_addr(wav_bram_addr),
        .bram_din(wav_bram_din),
        .bram_dout(wav_bram_dout),
        .bram_en(wav_bram_en),
        .bram_we(wav_bram_we),
        .wav_frame_seq(wav_frame_seq),
        .wav_busy(wav_busy),
        .wav_overrun(wav_overrun)
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
        
        // FIFO interface - 64-bit with additional metadata
        .fifo_write_en(fifo_write_en),
        .fifo_write_data(fifo_write_data),          // 128-bit data
        .fifo_channel_mask(fifo_channel_mask),      // 8-bit channel metadata
        .fifo_full(fifo_full),
        .fifo_count(fifo_count),                    // Count of 64-bit entries
        .fifo_packet_end_flag(fifo_packet_end_flag),
        .current_bram_address(current_bram_address),
        
        // BRAM interface (stays 32-bit)
        .bram_addr(bram_addr),
        .bram_din(bram_din),
        .bram_en(bram_en),
        .bram_we(bram_we),
        .bram_clk(bram_clk),
        .bram_rst(bram_rst)
    );
    
    // Combine status registers in wrapper
    // Clean separation: data generator owns 0-9, wrapper adds FIFO/BRAM status as 10
    assign status_regs_pl[0*32 +: 32] = data_gen_status[0*32 +: 32];  // Generator status 0 
    assign status_regs_pl[1*32 +: 32] = data_gen_status[1*32 +: 32];  // Generator status 1  
    assign status_regs_pl[2*32 +: 32] = data_gen_status[2*32 +: 32];  // Generator status 2
    assign status_regs_pl[3*32 +: 32] = data_gen_status[3*32 +: 32];  // Generator status 3
    assign status_regs_pl[4*32 +: 32] = data_gen_status[4*32 +: 32];  // Generator status 4
    assign status_regs_pl[5*32 +: 32] = data_gen_status[5*32 +: 32];  // Generator status 5
    assign status_regs_pl[6*32 +: 32] = data_gen_status[6*32 +: 32];  // Generator status 6
    assign status_regs_pl[7*32 +: 32] = data_gen_status[7*32 +: 32];  // Generator status 7
    assign status_regs_pl[8*32 +: 32] = data_gen_status[8*32 +: 32];  // Generator status 8
    assign status_regs_pl[9*32 +: 32] = data_gen_status[9*32 +: 32];  // Generator status 9

    assign status_regs_pl[10*32 +: 32] = {9'd0, fifo_count, current_bram_address}; // FIFO + BRAM status

    // Aux command sequencer status (see data_generator_core for bit layout)
    assign status_regs_pl[11*32 +: 32] = aux_status;
    assign status_regs_pl[12*32 +: 32] = aux_read_result;

    // LFP engine status: [15:0] output-BRAM write byte-address (PS read pointer),
    // [16] sticky compute-overrun flag.
    assign status_regs_pl[13*32 +: 32] = {15'd0, lfp_overrun, lfp_wr_addr};

    // Wavelet (Tier-3) engine status: [29:0] completed scalogram-column count
    // (the PS polls this to detect a fresh column), [30] busy, [31] sticky
    // compute-overrun flag.
    assign status_regs_pl[14*32 +: 32] = {wav_overrun, wav_busy, wav_frame_seq[29:0]};

    // Port-B master outputs are the same broadcast commands as port A.
    assign csn_b  = csn;
    assign sclk_b = sclk;
    assign copi_b = copi;

endmodule
