// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// lfp_packet_tb.sv
//
// Guards the LFP packet the PL assembles in its output BRAM -- the part the
// filter testbenches do not reach. They check that the arithmetic is right;
// this checks that the answers end up in the right place, in a frame the host
// can actually decode.
//
// What it asserts, all of which is wire contract:
//   * the 8-word common header is present and correct (magic, stream_type,
//     SEQ incrementing once per frame, num_samples, and the AUX0 field packing
//     that net.py and the plugin decode by bit position)
//   * every enabled channel's sample lands at the word and half implied by its
//     rank among enabled lanes -- the engine emits several lanes at once and so
//     writes OUT of wire order, and only the address arithmetic puts it right
//   * no word of the frame is left unwritten
//   * lfp_wr_addr only advances once the whole frame, header included, is down,
//     so the PS can never DMA a partial packet
//
// It runs with a SHORT stage-2 filter on purpose. num_taps is host-settable
// from 1, and the shorter it is the sooner the first sample appears after the
// decimation tick: with a long filter the header has hundreds of spare cycles,
// with a short one it has almost none. A builder that writes the header and the
// samples through the same port without excluding them will corrupt a frame
// here and pass with the default filter.
//
// Run: bash programmable_logic/sim/run_lfp_packet_tb.sh   ("RESULT: PASS")

`timescale 1ns/1ps

import unified_pkt_pkg::*;

module lfp_packet_tb;

    localparam int N_LANES = 8, N_SLOTS = 32;
    localparam int DATA_W = 16;
    localparam int LFP_BRAM_AW = 14;
    localparam int WORD_AW = LFP_BRAM_AW - 2;
    localparam int DECIM_TOTAL = 10;
    localparam int SHORT_TAPS = 1;          // tightest legal case (see header)
    localparam logic [7:0] LANE_MASK = 8'h03;   // 2 lanes -> 64 channels
    localparam int N_EN_CH = 2 * N_SLOTS;
    localparam int N_FRAMES = 2;

    logic clk = 0; always #5 clk = ~clk;
    logic rstn = 0;

    int n_checks = 0;
    int n_err_bus = 0;      // written only by the bus monitor
    int n_err_chk = 0;      // written only by the checker

    // ---- DUT ----
    logic         dsp_sample_valid = 0, dsp_packet_tick = 0;
    logic [127:0] dsp_sample_data = '0;
    logic [5:0]   dsp_sample_slot = '0;
    logic [63:0]  dsp_master_timestamp = 0;
    logic [31:0]  lfp_cfg, lfp_coef = '0, lfp_strobe = '0;
    logic                   bram_clk, bram_rst, bram_en;
    logic [LFP_BRAM_AW-1:0] bram_addr;
    logic [31:0]            bram_din;
    logic [3:0]             bram_we;
    logic [LFP_BRAM_AW-1:0] lfp_wr_addr;
    logic                   lfp_overrun;

    // enable | lane_mask(unused) | decim_R(reported) | num_taps
    assign lfp_cfg = {8'(SHORT_TAPS), 8'd0, 8'd0, 7'd0, 1'b1};

    lfp_dsp_block #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .LFP_BRAM_AW(LFP_BRAM_AW)
    ) dut (
        .clk(clk), .rstn(rstn),
        .dsp_sample_valid(dsp_sample_valid), .dsp_sample_data(dsp_sample_data),
        .dsp_sample_slot(dsp_sample_slot), .dsp_packet_tick(dsp_packet_tick),
        .dsp_master_timestamp(dsp_master_timestamp), .dsp_channel_enable(LANE_MASK),
        .lfp_cfg(lfp_cfg), .lfp_coef(lfp_coef), .lfp_strobe(lfp_strobe),
        .bram_clk(bram_clk), .bram_rst(bram_rst), .bram_addr(bram_addr),
        .bram_din(bram_din), .bram_dout(32'd0), .bram_en(bram_en), .bram_we(bram_we),
        .lfp_wr_addr(lfp_wr_addr), .lfp_overrun(lfp_overrun)
    );

    // ---- model of the output BRAM, plus per-half write tracking ----
    logic [31:0] mem  [0:(1<<WORD_AW)-1];
    bit          wrote_lo [0:(1<<WORD_AW)-1];
    bit          wrote_hi [0:(1<<WORD_AW)-1];
    logic [WORD_AW-1:0] published;      // lfp_wr_addr as a word address
    int    publish_count = 0;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            publish_count <= 0;
        end else begin
            if (bram_en && bram_we[1:0] == 2'b11) begin
                mem[bram_addr[LFP_BRAM_AW-1:2]][15:0] <= bram_din[15:0];
                wrote_lo[bram_addr[LFP_BRAM_AW-1:2]] <= 1'b1;
            end
            if (bram_en && bram_we[3:2] == 2'b11) begin
                mem[bram_addr[LFP_BRAM_AW-1:2]][31:16] <= bram_din[31:16];
                wrote_hi[bram_addr[LFP_BRAM_AW-1:2]] <= 1'b1;
            end
            // A byte-enable pattern other than a clean half or a full word would
            // mean the placement arithmetic has gone wrong.
            if (bram_en && bram_we != 4'h0 && bram_we != 4'hF
                        && bram_we != 4'b0011 && bram_we != 4'b1100) begin
                n_err_bus++;
                $display("ERROR: unexpected byte-enable %b at word %0d",
                         bram_we, bram_addr[LFP_BRAM_AW-1:2]);
            end
            published <= lfp_wr_addr[LFP_BRAM_AW-1:2];
            if (lfp_wr_addr[LFP_BRAM_AW-1:2] != published) publish_count <= publish_count + 1;
        end
    end

    // ---- stimulus ----
    function automatic logic [15:0] samp(input int pkt, input int lane, input int slot);
        // offset-binary, distinct per (pkt, lane, slot)
        samp = 16'h8000 + 16'((pkt*7 + lane*131 + slot*17) & 16'h0FFF);
    endfunction

    task automatic drive_packet(input int pkt);
        for (int slot = 0; slot < N_SLOTS; slot++) begin
            @(negedge clk);
            dsp_sample_slot <= 6'(slot + 2);          // FIRST_AMP_SLOT = 2
            for (int l = 0; l < N_LANES; l++)
                dsp_sample_data[l*DATA_W +: DATA_W] <= samp(pkt, l, slot);
            dsp_sample_valid <= 1'b1;
            @(negedge clk); dsp_sample_valid <= 1'b0;
            @(negedge clk);
        end
        @(negedge clk);
        dsp_master_timestamp <= dsp_master_timestamp + 1;
        dsp_packet_tick <= 1'b1;
        @(negedge clk); dsp_packet_tick <= 1'b0;
        repeat (3000) @(negedge clk);        // let both stages settle
    endtask

    // ---- check one assembled frame ----
    task automatic check_frame(input int fr, input logic [WORD_AW-1:0] base);
        logic [31:0] w;
        int n_samp, rank;
        n_samp = N_EN_CH;

        n_checks++;
        if (mem[base] !== UNIFIED_MAGIC) begin
            n_err_chk++;
            $display("ERROR: frame %0d magic got %08h exp %08h", fr, mem[base], UNIFIED_MAGIC);
        end
        n_checks++;
        w = mem[base+1];
        if (w[7:0] !== STREAM_TYPE_LFP) begin
            n_err_chk++;
            $display("ERROR: frame %0d stream_type got %0d exp %0d",
                     fr, w[7:0], STREAM_TYPE_LFP);
        end
        n_checks++;
        if (mem[base+4] !== 32'(fr)) begin
            n_err_chk++;
            $display("ERROR: frame %0d SEQ got %0d exp %0d", fr, mem[base+4], fr);
        end
        // AUX0 field packing is decoded by bit position on the host side.
        n_checks++;
        w = mem[base+5];
        if (w[7:0] !== LANE_MASK || w[15:8] !== 8'(DECIM_TOTAL)
                                 || w[23:16] !== 8'(SHORT_TAPS)) begin
            n_err_chk++;
            $display("ERROR: frame %0d AUX0 %08h (lane_mask/decim_R/num_taps)", fr, w);
        end
        n_checks++;
        if (mem[base+6] !== 32'(n_samp)) begin
            n_err_chk++;
            $display("ERROR: frame %0d num_samples got %0d exp %0d",
                     fr, mem[base+6], n_samp);
        end
        // w7 is reserved and must read 0. It is the LAST header word written and
        // so the first casualty if a sample write steals the port, which makes it
        // the most sensitive check in this frame.
        n_checks++;
        if (mem[base+7] !== 32'd0) begin
            n_err_chk++;
            $display("ERROR: frame %0d RSVD got %08h exp 0 (header word overwritten?)",
                     fr, mem[base+7]);
        end
        // Every header word must have been written as a FULL word.
        for (int k = 0; k < UNIFIED_HDR_WORDS; k++) begin
            n_checks++;
            if (!(wrote_lo[base+k] && wrote_hi[base+k])) begin
                n_err_chk++;
                $display("ERROR: frame %0d header word %0d not fully written", fr, k);
            end
        end

        // Every enabled channel must occupy its own half-word, and every half of
        // the payload must have been written exactly once.
        for (int lane = 0; lane < N_LANES; lane++) begin
            if (!LANE_MASK[lane]) continue;
            for (int slot = 0; slot < N_SLOTS; slot++) begin
                rank = lane * N_SLOTS + slot;   // lanes 0,1 enabled -> rank == channel
                n_checks++;
                if (!(rank[0] ? wrote_hi[base + UNIFIED_HDR_WORDS + (rank>>1)]
                              : wrote_lo[base + UNIFIED_HDR_WORDS + (rank>>1)])) begin
                    n_err_chk++;
                    if (n_err_chk <= 10)
                        $display("ERROR: frame %0d ch %0d (rank %0d) half never written",
                                 fr, lane*N_SLOTS+slot, rank);
                end
            end
        end
    endtask

    logic [WORD_AW-1:0] frame_base [0:3];
    int    n_pub = 0;
    logic [WORD_AW-1:0] last_pub = '0;

    // Record where each published frame ended, to derive its base.
    always_ff @(posedge clk) begin
        if (rstn && lfp_wr_addr[LFP_BRAM_AW-1:2] != last_pub) begin
            if (n_pub < 4) frame_base[n_pub] = last_pub;
            last_pub <= lfp_wr_addr[LFP_BRAM_AW-1:2];
            n_pub    <= n_pub + 1;
        end
    end

    initial begin
        for (int i = 0; i < (1<<WORD_AW); i++) begin
            wrote_lo[i] = 1'b0; wrote_hi[i] = 1'b0; mem[i] = 32'hDEADBEEF;
        end
        repeat (8) @(negedge clk); rstn = 1; repeat (8) @(negedge clk);

        for (int p = 0; p < N_FRAMES*DECIM_TOTAL + 2; p++) drive_packet(p);
        repeat (500) @(negedge clk);

        if (n_pub < N_FRAMES) begin
            n_err_chk++;
            $display("ERROR: only %0d frames published, expected >= %0d", n_pub, N_FRAMES);
        end else begin
            for (int f = 0; f < N_FRAMES; f++) check_frame(f, frame_base[f]);
        end
        if (lfp_overrun) begin
            n_err_chk++;
            $display("ERROR: compute_overrun asserted");
        end

        begin
            int n_errors; n_errors = n_err_bus + n_err_chk;
            $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
            if (n_errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        end
        $finish;
    end

    initial begin #200_000_000; $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish; end

endmodule
