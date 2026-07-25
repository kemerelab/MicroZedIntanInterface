// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// lfp_halfband_dec2_tb.sv
//
// Guards LFP stage 1 (30 kHz -> 15 kHz) against a bit-exact reference.
//
// Every decimated sample is compared to the value gen_lfp_hb_vectors.py computes
// with the same integer arithmetic the hardware uses -- no tolerance, because a
// tolerance would swallow the bugs that actually happen here: a product
// truncated to its self-determined width, the wrong output shift, an operand
// that lost its signedness, or a channel reading another channel's history.
//
// It should fail if: the tap ordering reverses, the delay-line ring addressing
// slips, the decimation fires on the wrong packet, the round/saturate changes,
// or the output ordering (lane-major, then slot) changes -- all of which are
// contract, not implementation detail, because stage 2 and the packet builder
// depend on them.
//
// Run: bash programmable_logic/sim/run_lfp_hb_tb.sh   ("RESULT: PASS")

`timescale 1ns/1ps

module lfp_halfband_dec2_tb;

    localparam int N_LANES = 8, N_SLOTS = 32;
    localparam int DATA_W = 16, OUT_W = 18;
    localparam int N_PACKETS = 30, DECIM = 2;
    localparam int N_CH = N_LANES * N_SLOTS;
    localparam int N_FRAMES = N_PACKETS / DECIM;

    logic clk = 0; always #5 clk = ~clk;          // 100 MHz sim clock
    logic rstn = 0;

    // Split by driving process: xsim rejects a variable written from both an
    // always_ff and an initial block. Summed for the verdict.
    int n_checks = 0;
    int n_err_stream = 0;   // written only by the output checker below
    int n_err_final  = 0;   // written only by the stimulus/verdict block

    // ---- stimulus / reference, from the Python generator ----
    logic [127:0] samples [0:N_PACKETS*N_SLOTS-1];
    logic [17:0]  expected [0:N_FRAMES*N_CH-1];

    // ---- DUT ----
    logic                    sample_valid = 0, packet_tick = 0;
    logic [N_LANES*DATA_W-1:0] sample_data = '0;
    logic [4:0]              sample_slot = '0;
    logic                    out_valid, out_frame_start, busy, compute_overrun;
    logic [7:0]              out_channel;
    logic signed [OUT_W-1:0] out_data;

    lfp_halfband_dec2 #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .DATA_W(DATA_W), .OUT_W(OUT_W)
    ) dut (
        .clk(clk), .rstn(rstn),
        .sample_valid(sample_valid), .sample_data(sample_data),
        .sample_slot(sample_slot), .packet_tick(packet_tick),
        .lfp_en(1'b1), .lane_mask(8'hFF),
        .coef_wr_en(1'b0), .coef_wr_addr('0), .coef_wr_data('0),
        .out_valid(out_valid), .out_channel(out_channel), .out_data(out_data),
        .out_frame_start(out_frame_start), .busy(busy),
        .compute_overrun(compute_overrun)
    );

    // ---- collect outputs in emission order and check against the reference ----
    int out_idx = 0;
    always_ff @(posedge clk) begin
        if (rstn && out_valid) begin
            n_checks++;
            if (out_idx >= N_FRAMES*N_CH) begin
                n_err_stream++;
                $display("ERROR: extra output #%0d (expected only %0d)",
                         out_idx, N_FRAMES*N_CH);
            end else begin
                if (out_data !== $signed(expected[out_idx])) begin
                    n_err_stream++;
                    if (n_err_stream <= 10)
                        $display("ERROR: out[%0d] (frame %0d ch %0d) got %0d exp %0d",
                                 out_idx, out_idx/N_CH, out_idx%N_CH,
                                 out_data, $signed(expected[out_idx]));
                end
                // Emission order is contract: lane-major, then slot.
                if (out_channel !== (out_idx % N_CH)) begin
                    n_err_stream++;
                    if (n_err_stream <= 10)
                        $display("ERROR: out[%0d] channel got %0d exp %0d",
                                 out_idx, out_channel, out_idx % N_CH);
                end
            end
            out_idx++;
        end
    end

    // ---- drive one acquisition packet: 32 slot words, then the packet tick ----
    task automatic drive_packet(input int pkt);
        for (int slot = 0; slot < N_SLOTS; slot++) begin
            @(negedge clk);
            sample_slot  <= slot[4:0];
            sample_data  <= samples[pkt*N_SLOTS + slot];
            sample_valid <= 1'b1;
            @(negedge clk);
            sample_valid <= 1'b0;
            // Idle between slots, as the real acquisition frame does.
            repeat (2) @(negedge clk);
        end
        @(negedge clk); packet_tick <= 1'b1;
        @(negedge clk); packet_tick <= 1'b0;
        // Let the compute pass run to completion before the next packet.
        repeat (4000) @(negedge clk);
    endtask

    initial begin
        $readmemh("lfp_hb_samples.hex", samples);
        $readmemh("lfp_hb_exp.hex", expected);

        repeat (8) @(negedge clk); rstn = 1; repeat (8) @(negedge clk);

        for (int p = 0; p < N_PACKETS; p++) drive_packet(p);
        repeat (200) @(negedge clk);

        if (out_idx != N_FRAMES*N_CH) begin
            n_err_final++;
            $display("ERROR: emitted %0d outputs, expected %0d", out_idx, N_FRAMES*N_CH);
        end
        if (compute_overrun) begin
            n_err_final++;
            $display("ERROR: compute_overrun asserted -- the pass did not fit its budget");
        end

        begin int n_errors; n_errors = n_err_stream + n_err_final;
        $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
        if (n_errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL"); end
        $finish;
    end

    initial begin #500_000_000; $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish; end

endmodule
