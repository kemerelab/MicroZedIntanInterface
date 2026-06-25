// =====================================================================
// accel_extract_block.sv
//
// Movement front-end: tracks the aux-command rotation, pulls the 3 head-stage
// accelerometer axes out of the broadband sample stream, decimates them to a
// lower rate, and BUILDS A DMA-ABLE BLOCK in a PS-readable BRAM -- so the PS
// just CDMAs whole blocks and runs the movement compute on clean [x,y,z] rows
// (no per-sample work, no Xil_In32 loop). This is the PL half of the
// PL-extract / PS-compute split (see the plan + docs/command-bank-design.md).
//
// HOW THE ACCEL IS LABELLED (precise channel matching):
//   The aux sequencer's slot 1 (cycle 33) sweeps CONVERT(32)->(33)->(34), one
//   axis per packet (10 kHz/axis). The result of that command lands at packet
//   DATA WORD 0 (cycle_counter 0), i.e. dsp_sample_slot == ACCEL_SLOT. The core
//   latches the originating command into echo_slot2_prev and feeds it here as
//   dsp_accel_cmd (with dsp_accel_cmd_valid = aux_seq_en_pkt & echo_valid). So
//   axis = (CONVERT_ch - 32) is taken from the SAME echo the host/plugin use,
//   already pipeline-deskewed -- the labelling matches by construction. The
//   override-layer invariant (only slot 0 is ever command-replaced; injection
//   borrows slot 2) guarantees the slot-1 accel sweep is never perturbed.
//
// DECIMATION (v1): a per-axis 1-pole EMA anti-alias (wide accumulator, no
//   dead-zone) sampled every DECIM_M packets -> one [x,y,z] triplet. Movement
//   is < ~15 Hz so this is ample; a CIC/boxcar can replace the EMA later. The
//   PS does the real filtering (gravity removal, speed proxy) on the block.
//
// BLOCK FORMAT (PL-built, mirrors lfp_dsp_block's header discipline):
//   per block = [6-word header | N_TRIPLETS x (wordA, wordB)]
//     w0 = ACCEL_MAGIC_LOW (0x1F1FACE1)
//     w1 = ACCEL_MAGIC_HIGH (0xCAFEBABE)
//     w2/w3 = 64-bit master timestamp of the block's first triplet
//     w4 = N_TRIPLETS[7:0] | (decim_M[15:0]<<8) | (headstage[1:0]<<24) | (overrun<<31)
//     w5 = block sequence number (++ per emitted block)
//   each triplet: wordA = {y[15:0], x[15:0]}, wordB = {16'h0, z[15:0]}  (SIGNED,
//   two's-complement centered -- offset-binary already removed here).
//   accel_wr_addr publishes ONLY at block completion, so the PS always DMAs a
//   whole [header|payload] block. The output BRAM must be in axi_cdma_0/Data.
// =====================================================================

module accel_extract_block #(
    parameter int N_LANES    = 8,     // dsp_sample_data = 8 x 16-bit lanes
    parameter int DATA_W     = 16,
    parameter int ACCEL_SLOT = 0,     // cycle_counter where the accel result lands (data word 0)
    parameter int N_TRIPLETS = 100,   // triplets per DMA block (keep even-friendly; any >=1 ok)
    parameter int EMA_FRAC   = 8,     // fractional bits in the per-axis EMA accumulator
    parameter int BRAM_AW    = 14     // accel output BRAM byte-address width (16 KB)
) (
    input  logic         clk,
    input  logic         rstn,

    // ---- tap from data_generator_core (same stream lfp_dsp_block uses) ----
    input  logic         dsp_sample_valid,
    input  logic [N_LANES*DATA_W-1:0] dsp_sample_data,
    input  logic [5:0]   dsp_sample_slot,        // cycle_counter, 0..34
    input  logic         dsp_packet_tick,        // 1-clk pulse, one per packet
    input  logic [63:0]  dsp_master_timestamp,   // live master sample count
    input  logic [15:0]  dsp_accel_cmd,          // echo_slot2_prev: rotating accel CONVERT echo
    input  logic         dsp_accel_cmd_valid,    // aux_seq_en_pkt & echo_valid

    // ---- control register slice (CDC'd by axi_lite_registers) ----
    //   [0] accel_en, [2:1] headstage (0..3 -> regular lane 0/2/4/6),
    //   [6:3] ema_shift (EMA leak K), [22:8] decim_M (packets per triplet, min 1)
    input  logic [31:0]  accel_cfg,

    // ---- accel output BRAM port (PL writes; PS reads via axi_bram_ctrl/CDMA) ----
    output logic                bram_clk,
    output logic                bram_rst,
    output logic [BRAM_AW-1:0]  bram_addr,
    output logic [31:0]         bram_din,
    input  logic [31:0]         bram_dout,        // unused (write-only side)
    output logic                bram_en,
    output logic [3:0]          bram_we,

    // ---- status ----
    output logic [BRAM_AW-1:0]  accel_wr_addr,    // byte addr just past the last COMPLETE block
    output logic                accel_overrun     // sticky: a triplet dropped while writing a block
);

    localparam int WORD_AW = BRAM_AW - 2;                       // 32-bit word address width
    localparam int BLOCK_WORDS = 6 + 2*N_TRIPLETS;              // header + 2 words/triplet
    localparam logic [15:0] OFFSET16 = 16'h8000;
    localparam logic [31:0] ACCEL_MAGIC_LOW  = 32'h1F1FACE1;
    localparam logic [31:0] ACCEL_MAGIC_HIGH = 32'hCAFEBABE;

    // -----------------------------------------------------------------
    // Control unpack.
    // -----------------------------------------------------------------
    wire        accel_en  = accel_cfg[0];
    wire [1:0]  headstage = accel_cfg[2:1];
    wire [3:0]  ema_shift = accel_cfg[6:3];
    wire [14:0] decim_m15 = accel_cfg[22:8];
    wire [14:0] decim_M   = (decim_m15 == 0) ? 15'd1 : decim_m15;

    // -----------------------------------------------------------------
    // Accel-sample ingest: gate on the accel data word, decode the axis from the
    // echo, center offset-binary -> signed, drive the per-axis EMA.
    // The regular (non-DDR) stream for head-stage h is lane 2*h (lanes 0/2/4/6).
    // -----------------------------------------------------------------
    logic signed [15:0] accel_raw;
    always_comb begin
        unique case (headstage)
            2'd0: accel_raw = $signed(dsp_sample_data[0*DATA_W +: DATA_W] ^ OFFSET16);
            2'd1: accel_raw = $signed(dsp_sample_data[2*DATA_W +: DATA_W] ^ OFFSET16);
            2'd2: accel_raw = $signed(dsp_sample_data[4*DATA_W +: DATA_W] ^ OFFSET16);
            default: accel_raw = $signed(dsp_sample_data[6*DATA_W +: DATA_W] ^ OFFSET16);
        endcase
    end

    wire        cmd_is_convert = (dsp_accel_cmd[15:14] == 2'b00);
    wire [5:0]  cmd_ch         = dsp_accel_cmd[13:8];
    wire        cmd_ch_accel   = (cmd_ch >= 6'd32) && (cmd_ch <= 6'd34);
    wire [1:0]  cmd_axis       = cmd_ch[1:0];                   // 32->0, 33->1, 34->2
    wire        accel_hit = accel_en && dsp_sample_valid &&
                            (dsp_sample_slot == ACCEL_SLOT[5:0]) &&
                            dsp_accel_cmd_valid && cmd_is_convert && cmd_ch_accel;

    // Per-axis EMA accumulator (signed, EMA_FRAC fractional bits -> no dead-zone).
    localparam int EMA_W = 16 + EMA_FRAC;
    logic signed [EMA_W-1:0] ema_acc [0:2];
    logic [2:0]              have_axis;                          // seen-at-least-once per axis
    logic                    have_all;

    always_ff @(posedge clk) begin
        if (!rstn || !accel_en) begin
            ema_acc[0] <= '0; ema_acc[1] <= '0; ema_acc[2] <= '0;
            have_axis  <= 3'b000;
            have_all   <= 1'b0;
        end else if (accel_hit) begin
            logic signed [EMA_W-1:0] target, delta;
            target = EMA_W'(accel_raw) <<< EMA_FRAC;
            delta  = (target - ema_acc[cmd_axis]) >>> ema_shift;
            ema_acc[cmd_axis] <= ema_acc[cmd_axis] + delta;
            have_axis[cmd_axis] <= 1'b1;
            if ((have_axis | (3'b001 << cmd_axis)) == 3'b111)
                have_all <= 1'b1;
        end
    end

    // Integer part of each axis EMA (truncate the fractional bits).
    wire signed [15:0] ema_x = ema_acc[0][EMA_W-1:EMA_FRAC];
    wire signed [15:0] ema_y = ema_acc[1][EMA_W-1:EMA_FRAC];
    wire signed [15:0] ema_z = ema_acc[2][EMA_W-1:EMA_FRAC];

    // -----------------------------------------------------------------
    // Decimation: emit one triplet every DECIM_M packets, once all axes seen.
    // -----------------------------------------------------------------
    logic [14:0] decim_cnt;
    logic        trip_req;                    // 1-clk: a new triplet is ready
    logic signed [15:0] trip_x, trip_y, trip_z;
    logic [63:0] trip_ts;

    always_ff @(posedge clk) begin
        if (!rstn || !accel_en) begin
            decim_cnt <= 15'd0;
            trip_req  <= 1'b0;
        end else begin
            trip_req <= 1'b0;
            if (dsp_packet_tick) begin
                if (!have_all) begin
                    decim_cnt <= 15'd0;
                end else if (decim_cnt + 15'd1 >= decim_M) begin
                    decim_cnt <= 15'd0;
                    trip_req  <= 1'b1;
                    trip_x    <= ema_x;
                    trip_y    <= ema_y;
                    trip_z    <= ema_z;
                    trip_ts   <= dsp_master_timestamp;
                end else begin
                    decim_cnt <= decim_cnt + 15'd1;
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // Block writer: header micro-sequence at each block base, then 2 words per
    // triplet; publish accel_wr_addr only when a block completes.
    // -----------------------------------------------------------------
    typedef enum logic [1:0] { W_IDLE, W_HDR, W_TA, W_TB } wstate_t;
    wstate_t            wstate;
    logic [2:0]         hdr_idx;
    logic [WORD_AW-1:0] wr_word;        // running BRAM write word pointer
    logic [WORD_AW-1:0] block_base;     // base of the in-progress block
    logic [7:0]         trip_idx;       // triplets written into this block
    logic [63:0]        block_ts;       // header timestamp (first triplet of block)
    logic [31:0]        block_seq;
    logic               ov_sticky;
    // latched triplet being written
    logic signed [15:0] cur_x, cur_y, cur_z;

    logic               we_r;
    logic [31:0]        din_r;
    logic [WORD_AW-1:0] waddr_r;

    // [7:0]=N_TRIPLETS, [22:8]=decim_M, [24:23]=headstage, [31]=overrun
    wire [31:0] hdr_cfg = {ov_sticky, 6'd0, headstage, decim_M, N_TRIPLETS[7:0]};

    always_ff @(posedge clk) begin
        if (!rstn || !accel_en) begin
            wstate     <= W_IDLE;
            hdr_idx    <= 3'd0;
            wr_word    <= '0;
            block_base <= '0;
            trip_idx   <= 8'd0;
            block_seq  <= 32'd0;
            ov_sticky  <= 1'b0;
            we_r       <= 1'b0;
            accel_wr_addr <= '0;
        end else begin
            we_r <= 1'b0;

            // Overrun: a new triplet arrived while still writing the previous one.
            if (trip_req && (wstate != W_IDLE))
                ov_sticky <= 1'b1;

            case (wstate)
                W_IDLE: begin
                    if (trip_req) begin
                        cur_x <= trip_x; cur_y <= trip_y; cur_z <= trip_z;
                        if (trip_idx == 8'd0) begin
                            // start a new block: latch ts/seq, write the header first
                            block_ts <= trip_ts;
                            hdr_idx  <= 3'd0;
                            wstate   <= W_HDR;
                        end else begin
                            wstate <= W_TA;
                        end
                    end
                end

                W_HDR: begin
                    we_r    <= 1'b1;
                    waddr_r <= block_base + WORD_AW'(hdr_idx);
                    case (hdr_idx)
                        3'd0: din_r <= ACCEL_MAGIC_LOW;
                        3'd1: din_r <= ACCEL_MAGIC_HIGH;
                        3'd2: din_r <= block_ts[31:0];
                        3'd3: din_r <= block_ts[63:32];
                        3'd4: din_r <= hdr_cfg;
                        default: din_r <= block_seq;            // 3'd5
                    endcase
                    if (hdr_idx == 3'd5) begin
                        wr_word <= block_base + WORD_AW'(6);
                        wstate  <= W_TA;
                    end else begin
                        hdr_idx <= hdr_idx + 3'd1;
                    end
                end

                W_TA: begin                                     // wordA = {y, x}
                    we_r    <= 1'b1;
                    waddr_r <= wr_word;
                    din_r   <= {cur_y, cur_x};
                    wr_word <= wr_word + WORD_AW'(1);
                    wstate  <= W_TB;
                end

                W_TB: begin                                     // wordB = {0, z}
                    we_r    <= 1'b1;
                    waddr_r <= wr_word;
                    din_r   <= {16'h0, cur_z};
                    wr_word <= wr_word + WORD_AW'(1);
                    if (trip_idx + 8'd1 >= N_TRIPLETS[7:0]) begin
                        // block complete: publish the read pointer, advance the ring
                        accel_wr_addr <= {(block_base + WORD_AW'(BLOCK_WORDS)), 2'b00};
                        block_base    <= block_base + WORD_AW'(BLOCK_WORDS);
                        trip_idx      <= 8'd0;
                        block_seq     <= block_seq + 32'd1;
                    end else begin
                        trip_idx <= trip_idx + 8'd1;
                    end
                    wstate <= W_IDLE;
                end

                default: wstate <= W_IDLE;
            endcase
        end
    end

    assign accel_overrun = ov_sticky;
    assign bram_clk = clk;
    assign bram_rst = ~rstn;
    assign bram_en  = 1'b1;
    assign bram_we  = we_r ? 4'hF : 4'h0;
    assign bram_addr = {waddr_r, 2'b00};
    assign bram_din  = din_r;

endmodule
