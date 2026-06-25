`timescale 1ns/1ps
// Self-checking TB for accel_extract_block.
//
// Core correctness proven here (the "data matches the channel precisely" ask):
//   * the rotating accel (CONVERT 32/33/34, one axis/packet) is de-interleaved to
//     the right x/y/z slot using the echo, and offset-binary is centered to signed;
//   * the per-axis EMA decimation produces clean [x,y,z] triplets in DMA-able
//     blocks with the documented header/format;
//   * the extractor self-gates: a non-CONVERT echo (fast-settle/inject on a slot)
//     and aux_seq-off (cmd_valid=0) produce NO extraction (axes hold).
//
// Drive constant per-axis values so the EMA converges; triplets then equal the
// inputs within +/-2 LSB (EMA output truncation), unambiguous given the spread.
module accel_extract_block_tb;

    localparam int N_LANES    = 8;
    localparam int DATA_W     = 16;
    localparam int ACCEL_SLOT = 0;
    localparam int N_TRIPLETS = 4;     // small blocks so many complete quickly
    localparam int EMA_FRAC   = 8;
    localparam int BRAM_AW    = 14;
    localparam int WORD_AW    = BRAM_AW - 2;
    localparam int BLOCK_WORDS = 6 + 2*N_TRIPLETS;   // = 14

    localparam logic [31:0] MAGIC_LOW  = 32'h1F1FACE1;
    localparam logic [31:0] MAGIC_HIGH = 32'hCAFEBABE;

    // cfg: [0]en [2:1]headstage [6:3]ema_shift [22:8]decim_M
    localparam int EMA_SHIFT = 1;      // fast EMA so the TB settles quickly
    localparam int DECIM_M   = 2;

    logic clk = 0, rstn = 0;
    always #5 clk = ~clk;

    logic         dsp_sample_valid;
    logic [N_LANES*DATA_W-1:0] dsp_sample_data;
    logic [5:0]   dsp_sample_slot;
    logic         dsp_packet_tick;
    logic [63:0]  dsp_master_timestamp;
    logic [15:0]  dsp_accel_cmd;
    logic         dsp_accel_cmd_valid;
    logic [31:0]  accel_cfg;

    logic                bram_clk, bram_rst, bram_en;
    logic [BRAM_AW-1:0]  bram_addr;
    logic [31:0]         bram_din;
    logic [3:0]          bram_we;
    logic [BRAM_AW-1:0]  accel_wr_addr;
    logic                accel_overrun;

    accel_extract_block #(
        .N_LANES(N_LANES), .DATA_W(DATA_W), .ACCEL_SLOT(ACCEL_SLOT),
        .N_TRIPLETS(N_TRIPLETS), .EMA_FRAC(EMA_FRAC), .BRAM_AW(BRAM_AW)
    ) dut (
        .clk(clk), .rstn(rstn),
        .dsp_sample_valid(dsp_sample_valid), .dsp_sample_data(dsp_sample_data),
        .dsp_sample_slot(dsp_sample_slot), .dsp_packet_tick(dsp_packet_tick),
        .dsp_master_timestamp(dsp_master_timestamp),
        .dsp_accel_cmd(dsp_accel_cmd), .dsp_accel_cmd_valid(dsp_accel_cmd_valid),
        .accel_cfg(accel_cfg),
        .bram_clk(bram_clk), .bram_rst(bram_rst), .bram_addr(bram_addr),
        .bram_din(bram_din), .bram_dout(32'h0), .bram_en(bram_en), .bram_we(bram_we),
        .accel_wr_addr(accel_wr_addr), .accel_overrun(accel_overrun)
    );

    // ---- BRAM capture model ----
    logic [31:0] mem [0:(1<<WORD_AW)-1];
    always_ff @(posedge clk) begin
        if (bram_we == 4'hF) mem[bram_addr >> 2] <= bram_din;
    end

    int errors = 0;
    task chk(input bit cond, input string msg);
        if (!cond) begin errors++; $display("  FAIL: %s", msg); end
    endtask

    // rotating axis state for the driver
    int axis = 0;
    // per-axis offset-binary input (centered value + 0x8000)
    logic [15:0] axval [0:2];

    // Drive one acquisition packet: a dummy non-accel slot, then the accel word
    // at slot 0 with the current axis' echo, then packet_tick.
    task drive_packet(input bit cmd_valid, input bit force_convert);
        // dummy non-accel slot (must be ignored by the extractor)
        @(posedge clk);
        dsp_sample_slot  <= 6'd5;
        dsp_sample_data  <= '0;
        dsp_sample_valid <= 1'b1;
        @(posedge clk);
        dsp_sample_valid <= 1'b0;
        // accel word at slot 0
        dsp_accel_cmd_valid <= cmd_valid;
        dsp_accel_cmd <= force_convert ? (16'(axis + 32) << 8)   // CONVERT(32+axis)
                                       : 16'h8302;               // WRITE(3): not a convert
        @(posedge clk);
        dsp_sample_slot  <= 6'd0;
        dsp_sample_data  <= {112'h0, axval[axis]};               // lane 0 = accel
        dsp_sample_valid <= 1'b1;
        @(posedge clk);
        dsp_sample_valid <= 1'b0;
        // end-of-packet tick: a SINGLE-clock pulse (as the real core emits)
        repeat (2) @(posedge clk);
        dsp_master_timestamp <= dsp_master_timestamp + 1;
        dsp_packet_tick <= 1'b1;
        @(posedge clk);
        dsp_packet_tick <= 1'b0;
        axis = (axis + 1) % 3;
    endtask

    function automatic int sx16(input logic [15:0] v);
        return $signed(v);
    endfunction

    // Parse + check the b-th complete block in mem (no wrap in this run).
    task check_block(input int b, input int ex, input int ey, input int ez,
                     input bit check_vals);
        int base; int t; int wa; int wb;
        int gx, gy, gz;
        base = b * BLOCK_WORDS;
        chk(mem[base+0] === MAGIC_LOW,  $sformatf("block %0d magic_low", b));
        chk(mem[base+1] === MAGIC_HIGH, $sformatf("block %0d magic_high", b));
        chk(mem[base+5] === 32'(b),     $sformatf("block %0d seq", b));
        chk(mem[base+4][7:0] === 8'(N_TRIPLETS),  $sformatf("block %0d cfg.N", b));
        chk(mem[base+4][22:8] === 15'(DECIM_M),   $sformatf("block %0d cfg.decimM", b));
        chk(mem[base+4][24:23] === 2'd0,          $sformatf("block %0d cfg.headstage", b));
        if (check_vals) begin
            for (t = 0; t < N_TRIPLETS; t++) begin
                wa = mem[base + 6 + 2*t];
                wb = mem[base + 6 + 2*t + 1];
                gx = sx16(wa[15:0]); gy = sx16(wa[31:16]); gz = sx16(wb[15:0]);
                chk(wb[31:16] === 16'h0, $sformatf("block %0d trip %0d padding", b, t));
                chk((gx >= ex-2) && (gx <= ex+2), $sformatf("block %0d trip %0d x=%0d exp~%0d", b, t, gx, ex));
                chk((gy >= ey-2) && (gy <= ey+2), $sformatf("block %0d trip %0d y=%0d exp~%0d", b, t, gy, ey));
                chk((gz >= ez-2) && (gz <= ez+2), $sformatf("block %0d trip %0d z=%0d exp~%0d", b, t, gz, ez));
            end
        end
    endtask

    int i;
    int blocks_done;
    initial begin
        dsp_sample_valid = 0; dsp_sample_data = 0; dsp_sample_slot = 0;
        dsp_packet_tick = 0; dsp_master_timestamp = 64'd100;
        dsp_accel_cmd = 0; dsp_accel_cmd_valid = 0;
        // constant per-axis inputs (offset-binary): x=+1000, y=+2000, z=-1500
        axval[0] = 16'h8000 + 16'd1000;
        axval[1] = 16'h8000 + 16'd2000;
        axval[2] = 16'h8000 - 16'd1500;
        accel_cfg = 32'd0;

        repeat (6) @(posedge clk);
        rstn = 1;
        accel_cfg = 32'd1 | (32'd0 << 1) | (EMA_SHIFT << 3) | (DECIM_M << 8); // en, hs0, shift, M

        // --- Phase A: settle on constants (axis-labeling correctness) ---
        for (i = 0; i < 200; i++) drive_packet(1, 1);

        // blocks completed so far (accel_wr_addr is byte addr of next block base)
        blocks_done = (accel_wr_addr >> 2) / BLOCK_WORDS;
        chk(blocks_done >= 12, $sformatf("expected several blocks, got %0d", blocks_done));
        // check settled blocks (skip the first 10 while the EMA converges)
        for (i = 10; i < blocks_done; i++)
            check_block(i, 1000, 2000, -1500, 1);

        // --- Phase B: invariant -- non-CONVERT echo must NOT update the axes ---
        // hold for a full block of packets with a WRITE(3) echo; values must stay.
        for (i = 0; i < 24; i++) drive_packet(1, 0);   // force_convert=0
        // --- Phase C: invariant -- cmd_valid=0 (aux seq off) must NOT update ---
        for (i = 0; i < 24; i++) drive_packet(0, 1);   // cmd_valid=0
        // any blocks completed across B+C must still read the settled constants
        begin
            int bd2 = (accel_wr_addr >> 2) / BLOCK_WORDS;
            for (i = blocks_done; i < bd2; i++)
                check_block(i, 1000, 2000, -1500, 1);
            blocks_done = bd2;
        end

        // --- Phase D: value tracking -- change x, confirm later triplets follow ---
        axval[0] = 16'h8000 + 16'd500;
        for (i = 0; i < 200; i++) drive_packet(1, 1);
        begin
            int bd3 = (accel_wr_addr >> 2) / BLOCK_WORDS;
            // the last block should track the new x ~= 500 (y,z unchanged)
            check_block(bd3-1, 500, 2000, -1500, 1);
        end

        chk(accel_overrun === 1'b0, "no overrun expected at this rate");

        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    // safety timeout
    initial begin
        #20_000_000;
        $display("RESULT: FAIL (timeout)");
        $finish;
    end

endmodule
