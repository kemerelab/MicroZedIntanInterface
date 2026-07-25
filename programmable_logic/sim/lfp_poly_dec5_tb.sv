// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// lfp_poly_dec5_tb.sv
//
// Guards LFP stage 2 (15 kHz -> 3 kHz, up to 120 taps) against a bit-exact
// reference, with no tolerance: the reference uses the same integers the
// hardware does, so a truncated product, a wrong output shift or a lost sign
// shows up as a mismatch rather than as a small error someone waves through.
//
// This engine computes N_PAR lanes at once, so it emits OUT OF wire order by
// design. The test therefore scores each output by its out_channel into a
// per-frame array and checks the array -- which verifies the channel tagging
// (a mis-tagged sample lands in the wrong slot and fails) as well as the
// arithmetic, and would NOT be checked by comparing an emission-ordered list.
//
// It should fail if: tap ordering reverses, the ring addressing slips, the
// decimation fires on the wrong frame, a lane group reads another group's
// memory, the round/saturate changes, or a channel is emitted twice or not at
// all (the per-frame coverage check catches the last one).
//
// Run: bash programmable_logic/sim/run_lfp_poly_tb.sh   ("RESULT: PASS")

`timescale 1ns/1ps

module lfp_poly_dec5_tb;

    localparam int N_LANES = 8, N_SLOTS = 32;
    localparam int IN_W = 18, OUT_W = 16;
    localparam int N_TAPS = 120, DECIM = 5;
    localparam int N_FRAMES_IN = 150;
    localparam int N_CH = N_LANES * N_SLOTS;
    localparam int N_OUT_FRAMES = N_FRAMES_IN / DECIM;

    logic clk = 0; always #5 clk = ~clk;
    logic rstn = 0;

    int n_checks = 0;
    int n_err_stream = 0;    // written only by the output collector
    int n_err_final  = 0;    // written only by the stimulus/verdict block

    logic [17:0] samples  [0:N_FRAMES_IN*N_CH-1];
    logic [15:0] expected [0:N_OUT_FRAMES*N_CH-1];

    // ---- DUT ----
    logic                    sample_valid = 0, frame_tick = 0;
    logic [7:0]              sample_channel = '0;
    logic signed [IN_W-1:0]  sample_data = '0;
    logic                    out_valid, frame_start, busy, compute_overrun;
    logic [7:0]              out_channel;
    logic signed [OUT_W-1:0] out_data;

    lfp_poly_dec5 #(
        .N_LANES(N_LANES), .N_SLOTS(N_SLOTS), .IN_W(IN_W), .OUT_W(OUT_W),
        .MAX_TAPS(N_TAPS), .N_PAR(4), .DECIM(DECIM)
    ) dut (
        .clk(clk), .rstn(rstn),
        .sample_valid(sample_valid), .sample_channel(sample_channel),
        .sample_data(sample_data), .frame_tick(frame_tick),
        .lfp_en(1'b1), .lane_mask(8'hFF), .num_taps(N_TAPS[7:0]),
        .coef_wr_en(1'b0), .coef_wr_addr('0), .coef_wr_data('0),
        .out_valid(out_valid), .out_channel(out_channel), .out_data(out_data),
        .frame_start(frame_start), .busy(busy), .compute_overrun(compute_overrun)
    );

    // ---- score outputs BY CHANNEL, per frame ----
    // got/seen are written ONLY here. The DUT's own frame_start pulse clears the
    // scoreboard, so the checker task is read-only: writing these from both an
    // always_ff and a task would be two procedural drivers on one variable, which
    // races (and is what xsim rejects outright for scalars).
    logic signed [OUT_W-1:0] got  [0:N_CH-1];
    bit                      seen [0:N_CH-1];
    int out_frame = 0;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            for (int c = 0; c < N_CH; c++) seen[c] <= 1'b0;
        end else begin
            if (frame_start)
                for (int c = 0; c < N_CH; c++) seen[c] <= 1'b0;
            if (out_valid) begin
                if (seen[out_channel]) begin
                    n_err_stream++;
                    if (n_err_stream <= 10)
                        $display("ERROR: frame %0d channel %0d emitted twice",
                                 out_frame, out_channel);
                end
                got[out_channel]  <= out_data;
                seen[out_channel] <= 1'b1;
            end
        end
    end

    task automatic check_frame(input int fr);
        for (int c = 0; c < N_CH; c++) begin
            n_checks++;
            if (!seen[c]) begin
                n_err_final++;
                if (n_err_final <= 10)
                    $display("ERROR: frame %0d channel %0d never emitted", fr, c);
            end else if (got[c] !== $signed(expected[fr*N_CH + c])) begin
                n_err_final++;
                if (n_err_final <= 10)
                    $display("ERROR: frame %0d ch %0d got %0d exp %0d",
                             fr, c, got[c], $signed(expected[fr*N_CH + c]));
            end
        end
    endtask

    // ---- drive one stage-1 frame: N_CH samples, then the frame tick ----
    task automatic drive_frame(input int fr);
        for (int c = 0; c < N_CH; c++) begin
            @(negedge clk);
            sample_channel <= c[7:0];
            sample_data    <= $signed(samples[fr*N_CH + c]);
            sample_valid   <= 1'b1;
            @(negedge clk);
            sample_valid   <= 1'b0;
        end
        @(negedge clk); frame_tick <= 1'b1;
        @(negedge clk); frame_tick <= 1'b0;
    endtask

    initial begin
        $readmemh("lfp_poly_samples.hex", samples);
        $readmemh("lfp_poly_exp.hex", expected);
        repeat (8) @(negedge clk); rstn = 1; repeat (8) @(negedge clk);

        for (int f = 0; f < N_FRAMES_IN; f++) begin
            drive_frame(f);
            if (f % DECIM == DECIM-1) begin
                // A compute pass runs for ~8200 clocks; wait it out before scoring.
                repeat (10000) @(negedge clk);
                check_frame(out_frame);
                out_frame++;
            end else begin
                repeat (20) @(negedge clk);
            end
        end

        if (out_frame != N_OUT_FRAMES) begin
            n_err_final++;
            $display("ERROR: got %0d output frames, expected %0d", out_frame, N_OUT_FRAMES);
        end
        if (compute_overrun) begin
            n_err_final++;
            $display("ERROR: compute_overrun -- the pass did not fit its budget");
        end

        begin
            int n_errors; n_errors = n_err_stream + n_err_final;
            $display("Checks: %0d, Errors: %0d", n_checks, n_errors);
            if (n_errors == 0) $display("RESULT: PASS"); else $display("RESULT: FAIL");
        end
        $finish;
    end

    initial begin #2_000_000_000; $display("ERROR: watchdog"); $display("RESULT: FAIL"); $finish; end

endmodule
