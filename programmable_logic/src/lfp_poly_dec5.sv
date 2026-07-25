// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// =====================================================================
// lfp_poly_dec5.sv
//
// Stage 2 of the LFP decimation chain: a decimate-by-5 FIR taking the 15 kHz
// output of lfp_halfband_dec2 down to 3 kHz. Up to MAX_TAPS taps, runtime
// selectable, with N_PAR MACs running in parallel.
//
// On "polyphase"
// --------------
// A decimating FIR only has to produce the outputs it keeps, so each output
// costs num_taps MACs no matter how the sum is organised. An explicit
// M-branch polyphase decomposition computes the same products in a different
// order for the same cost; its advantage is over the naive filter-then-throw-
// away-4-of-5, which this never does. So this is the polyphase-efficient form
// written directly: on the decimation tick, walk the taps once per channel.
// The shipped filters are 120 taps = 5 phases x 24 so the decomposition is
// exact on paper, but the engine itself accepts any length up to MAX_TAPS.
//
// Why the MACs are spread across LANES
// ------------------------------------
// The delay line is one memory per lane, and the read address depends only on
// (slot, tap) -- NOT on lane. So N_PAR MACs working on N_PAR different lanes
// issue ONE address, read N_PAR separate memories, and all want the SAME
// coefficient in the same cycle. That costs no memory replication and one
// coefficient read broadcast N_PAR ways.
//
// Parallelising across taps or slots instead would put N_PAR different
// addresses into the SAME memory, forcing it to be replicated N_PAR times --
// roughly 64 block RAMs here instead of 16.
//
// The cost is that N_PAR lanes finish together, so outputs do NOT emerge in
// wire order (lane-major). The packet builder places each sample by its channel
// index rather than packing them in arrival order, which makes the emission
// order a non-issue -- see lfp_dsp_block.
//
// Throughput
// ----------
// One output per channel every DECIM stage-1 frames = 5 * (84 MHz / 15 kHz) =
// 28000 clocks. Cost is (N_LANES/N_PAR) groups * N_SLOTS * (num_taps + overhead)
// = 2 * 32 * ~127 = ~8100 clocks at 120 taps, about 3.5x inside the budget. That
// headroom is deliberate: a minimum-phase filter is not symmetric, so unlike a
// linear-phase design it cannot be folded to half the multiplies, and the engine
// must fit the worst case.
//
// Fixed point
// -----------
// Samples arrive from stage 1 as IN_W bits carrying (IN_W - OUT_W) fractional
// bits below the wire-format sample LSB. Coefficients are Q1.COEF_FRAC. The
// accumulator therefore holds COEF_FRAC + (IN_W - OUT_W) fractional bits, which
// is what gets rounded away on the way out to the OUT_W wire format.
// =====================================================================

module lfp_poly_dec5 #(
    parameter  int N_LANES    = 8,
    parameter  int N_SLOTS    = 32,
    parameter  int IN_W       = 18,   // stage-1 output width
    parameter  int COEF_W     = 18,
    parameter  int COEF_FRAC  = 17,
    parameter  int ACC_W      = 48,
    parameter  int OUT_W      = 16,   // wire sample width
    parameter  int MAX_TAPS   = 120,
    parameter  int RING_DEPTH = 128,  // power of 2, >= MAX_TAPS
    parameter  int N_PAR      = 4,    // parallel MACs, one per lane (see header)
    parameter  int DECIM      = 5,
    // ---- derived (do not override) ----
    localparam int SLOT_W  = (N_SLOTS  <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int RING_AW = $clog2(RING_DEPTH),
    localparam int TAP_W   = $clog2(MAX_TAPS + 1),
    localparam int N_CH    = N_LANES * N_SLOTS,
    localparam int CH_W    = (N_CH    <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W  = (N_LANES <= 1) ? 1 : $clog2(N_LANES),
    localparam int N_GROUP = N_LANES / N_PAR,
    localparam int GRP_W   = (N_GROUP <= 1) ? 1 : $clog2(N_GROUP),
    localparam int PAR_W   = (N_PAR   <= 1) ? 1 : $clog2(N_PAR),
    localparam int MEM_AW  = $clog2(N_SLOTS * RING_DEPTH)
) (
    input  logic                       clk,
    input  logic                       rstn,

    // ---- samples from stage 1: one channel at a time, in its emission order ----
    input  logic                       sample_valid,
    input  logic [CH_W-1:0]            sample_channel,   // lane*N_SLOTS + slot
    input  logic signed [IN_W-1:0]     sample_data,
    input  logic                       frame_tick,       // a stage-1 frame is complete

    // ---- configuration ----
    input  logic                       lfp_en,
    input  logic [N_LANES-1:0]         lane_mask,
    input  logic [TAP_W-1:0]           num_taps,         // 1..MAX_TAPS

    // ---- coefficient write port ----
    input  logic                       coef_wr_en,
    input  logic [TAP_W-1:0]           coef_wr_addr,
    input  logic [COEF_W-1:0]          coef_wr_data,

    // ---- decimated output ----
    output logic                       out_valid,
    output logic [CH_W-1:0]            out_channel,
    output logic signed [OUT_W-1:0]    out_data,
    output logic                       frame_start,      // lead-in pulse: a pass began
    output logic                       busy,
    output logic                       compute_overrun
);

    import lfp_coef_pkg::*;

    initial begin
        if (N_LANES % N_PAR != 0)
            $error("N_LANES (%0d) must be a multiple of N_PAR (%0d)", N_LANES, N_PAR);
        if (RING_DEPTH < MAX_TAPS)
            $error("RING_DEPTH (%0d) must be >= MAX_TAPS (%0d)", RING_DEPTH, MAX_TAPS);
    end

    localparam logic [RING_AW-1:0] RMASK  = RING_DEPTH - 1;
    localparam int                 PROD_W = IN_W + COEF_W;
    // Fractional bits to shed on the way out: the coefficients' own, plus the
    // extra resolution stage 1 handed us.
    localparam int                 OUT_SHIFT = COEF_FRAC + (IN_W - OUT_W);
    localparam signed [OUT_W:0]    OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0]    OUT_MIN = -(1 <<< (OUT_W-1));

    // =================================================================
    // Coefficient RAM -- ONE instance. The parallel MACs all work on the same
    // tap in the same cycle, so a single read is broadcast to all of them.
    // (Tap-parallel MACs would each need a different coefficient, forcing this
    // RAM to be replicated too.)
    // =================================================================
    logic signed [COEF_W-1:0] coef_ram [0:MAX_TAPS-1];
    logic        [TAP_W-1:0]  coef_rd_addr;
    logic signed [COEF_W-1:0] coef_rdata;

    initial
        for (int i = 0; i < MAX_TAPS; i++)
            coef_ram[i] = (i < LFP_POLY_TAPS) ? LFP_POLY_DEFAULT_COEF[i] : '0;

    always_ff @(posedge clk) begin
        if (coef_wr_en) coef_ram[coef_wr_addr] <= coef_wr_data;
        coef_rdata <= coef_ram[coef_rd_addr];
    end

    // =================================================================
    // Delay line: one memory per lane, [slot][ring]. Stage 1 delivers one
    // channel at a time, so exactly one lane is written per clock; during
    // compute the N_PAR lanes of a group share a read address.
    // =================================================================
    logic [RING_AW-1:0]     wr_pos;
    logic [MEM_AW-1:0]      dl_rd_addr;
    logic signed [IN_W-1:0] dl_rdata [0:N_LANES-1];

    wire [LANE_W-1:0] in_lane = sample_channel[CH_W-1 -: LANE_W];
    wire [SLOT_W-1:0] in_slot = sample_channel[SLOT_W-1:0];
    wire [MEM_AW-1:0] dl_wr_addr = in_slot * RING_DEPTH + wr_pos;

    genvar gl;
    generate
        for (gl = 0; gl < N_LANES; gl++) begin : g_lane_mem
            logic signed [IN_W-1:0] mem [0:N_SLOTS*RING_DEPTH-1];
            initial for (int ii = 0; ii < N_SLOTS*RING_DEPTH; ii++) mem[ii] = '0;
            always_ff @(posedge clk) begin
                if (sample_valid && in_lane == LANE_W'(gl))
                    mem[dl_wr_addr] <= sample_data;
                dl_rdata[gl] <= mem[dl_rd_addr];
            end
        end
    endgenerate

    // =================================================================
    // Ingest pointer + /DECIM counter, clocked by stage-1 frames.
    // =================================================================
    logic [$clog2(DECIM+1)-1:0] decim_cnt;
    logic [RING_AW-1:0]         head_snap;
    logic                       start_pass;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            wr_pos <= '0; decim_cnt <= '0; head_snap <= '0; start_pass <= 1'b0;
        end else begin
            start_pass <= 1'b0;
            if (frame_tick) begin
                if (decim_cnt + 1'b1 >= DECIM[$clog2(DECIM+1)-1:0]) begin
                    decim_cnt <= '0;
                    if (lfp_en) begin
                        head_snap  <= wr_pos;   // the frame just written is the newest
                        start_pass <= 1'b1;
                    end
                end else begin
                    decim_cnt <= decim_cnt + 1'b1;
                end
                wr_pos <= (wr_pos + 1'b1) & RMASK;
            end
        end
    end

    // =================================================================
    // Compute FSM: for each lane GROUP, for each slot, walk the taps; then let
    // the MAC pipeline drain and emit the group's N_PAR results one per clock.
    // =================================================================
    typedef enum logic [2:0] {C_IDLE, C_RUN, C_WAIT, C_EMIT, C_DRAIN} cstate_t;
    cstate_t            cstate;
    logic [GRP_W-1:0]   cur_group;
    logic [SLOT_W-1:0]  cur_slot;
    logic [TAP_W-1:0]   cur_tap;
    logic [1:0]         wait_cnt;
    logic [PAR_W:0]     emit_idx;

    // ag_* = "address generation": the s0 stage's markers, registered here so
    // they arrive at s1/s2 in step with the RAM data they describe. The address
    // is issued one cycle before its data appears, so a marker that travelled
    // with the address rather than alongside it would describe the wrong tap.
    logic               ag_valid, ag_first, ag_last;
    logic [GRP_W-1:0]   ag_group;

    wire last_group = (cur_group == GRP_W'(N_GROUP-1));
    wire last_slot  = (cur_slot  == SLOT_W'(N_SLOTS-1));
    wire last_tap   = (cur_tap + 1'b1 >= num_taps);

    // Any lane in this group enabled? A wholly disabled group is skipped.
    logic group_active;
    always_comb begin
        group_active = 1'b0;
        for (int k = 0; k < N_PAR; k++)
            if (lane_mask[cur_group*N_PAR + k]) group_active = 1'b1;
    end

    // One address for the whole group: tap t is the sample t frames old.
    always_comb begin
        dl_rd_addr   = cur_slot * RING_DEPTH + ((head_snap - cur_tap[RING_AW-1:0]) & RMASK);
        coef_rd_addr = cur_tap;
    end

    always_ff @(posedge clk) begin
        if (!rstn) begin
            cstate <= C_IDLE;
            cur_group <= '0; cur_slot <= '0; cur_tap <= '0;
            wait_cnt <= '0; emit_idx <= '0;
            ag_valid <= 1'b0; ag_first <= 1'b0; ag_last <= 1'b0; ag_group <= '0;
            busy <= 1'b0; compute_overrun <= 1'b0;
        end else begin
            ag_valid <= 1'b0;

            if (start_pass && cstate != C_IDLE) compute_overrun <= 1'b1;

            case (cstate)
                C_IDLE: begin
                    busy <= 1'b0;
                    if (start_pass) begin
                        cur_group <= '0; cur_slot <= '0; cur_tap <= '0;
                        cstate <= C_RUN; busy <= 1'b1;
                    end
                end

                C_RUN: begin
                    busy <= 1'b1;
                    if (!group_active) begin
                        cur_slot <= '0; cur_tap <= '0;
                        if (last_group) cstate <= C_DRAIN;
                        else            cur_group <= cur_group + 1'b1;
                    end else begin
                        ag_valid <= 1'b1;
                        ag_first <= (cur_tap == '0);
                        ag_last  <= last_tap;
                        ag_group <= cur_group;
                        if (last_tap) begin
                            cur_tap  <= '0;
                            wait_cnt <= 2'd2;        // let the 3-stage MAC drain
                            cstate   <= C_WAIT;
                        end else begin
                            cur_tap <= cur_tap + 1'b1;
                        end
                    end
                end

                C_WAIT: begin
                    if (wait_cnt == 0) begin
                        emit_idx <= '0;
                        cstate   <= C_EMIT;
                    end else wait_cnt <= wait_cnt - 1'b1;
                end

                C_EMIT: begin
                    if (emit_idx == N_PAR) begin
                        // Group's results are out; move to the next slot/group.
                        if (last_slot) begin
                            cur_slot <= '0;
                            if (last_group) cstate <= C_DRAIN;
                            else begin cur_group <= cur_group + 1'b1; cstate <= C_RUN; end
                        end else begin
                            cur_slot <= cur_slot + 1'b1;
                            cstate   <= C_RUN;
                        end
                    end else begin
                        emit_idx <= emit_idx + 1'b1;
                    end
                end

                C_DRAIN: cstate <= C_IDLE;
                default: cstate <= C_IDLE;
            endcase
        end
    end

    // =================================================================
    // MAC pipeline: N_PAR independent accumulators, one coefficient broadcast.
    // Everything explicitly signed -- a single unsigned operand would make the
    // whole expression unsigned and wreck negative partial sums.
    // =================================================================
    logic signed [PROD_W-1:0] prod1 [0:N_PAR-1];
    logic                     v1, first1, last1;
    logic [GRP_W-1:0]         grp1;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            for (int k = 0; k < N_PAR; k++) prod1[k] <= '0;
            v1 <= 1'b0; first1 <= 1'b0; last1 <= 1'b0; grp1 <= '0;
        end else begin
            for (int k = 0; k < N_PAR; k++)
                prod1[k] <= PROD_W'($signed(dl_rdata[ag_group*N_PAR + k])) *
                            PROD_W'($signed(coef_rdata));
            v1     <= ag_valid;
            first1 <= ag_first;
            last1  <= ag_last;
            grp1   <= ag_group;
        end
    end

    logic signed [ACC_W-1:0] acc  [0:N_PAR-1];
    logic signed [ACC_W-1:0] hold [0:N_PAR-1];   // finished results awaiting emission
    logic [GRP_W-1:0]        hold_group;
    logic [SLOT_W-1:0]       hold_slot;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            for (int k = 0; k < N_PAR; k++) begin acc[k] <= '0; hold[k] <= '0; end
            hold_group <= '0; hold_slot <= '0;
        end else begin
            if (v1) begin
                for (int k = 0; k < N_PAR; k++)
                    acc[k] <= first1 ? ACC_W'(prod1[k]) : (acc[k] + ACC_W'(prod1[k]));
            end
            if (v1 && last1) begin
                for (int k = 0; k < N_PAR; k++)
                    hold[k] <= first1 ? ACC_W'(prod1[k]) : (acc[k] + ACC_W'(prod1[k]));
                hold_group <= grp1;
                hold_slot  <= cur_slot;
            end
        end
    end

    // =================================================================
    // Emission: one result per clock, skipping lanes the mask disables. The
    // order is group-then-lane, NOT wire order -- out_channel carries the true
    // identity and the packet builder places each sample by it.
    // =================================================================
    // Scale the accumulator back to wire format: it holds OUT_SHIFT fractional
    // bits (the coefficients' own, plus the extra resolution stage 1 passed
    // down). RND adds half an output LSB before the arithmetic shift so the
    // result rounds to nearest rather than truncating toward negative infinity,
    // which would leave a half-LSB DC offset on every channel. The clamp that
    // follows saturates instead of wrapping, so an overrange sample reads as a
    // rail rather than flipping sign.
    localparam signed [ACC_W-1:0] RND = ACC_W'(1) <<< (OUT_SHIFT-1);
    logic signed [ACC_W-1:0] emit_rounded;
    logic [LANE_W-1:0]       emit_lane;
    logic                    frame_first;

    always_comb begin
        emit_lane    = LANE_W'(hold_group*N_PAR + emit_idx[PAR_W-1:0]);
        emit_rounded = (hold[emit_idx[PAR_W-1:0]] + RND) >>> OUT_SHIFT;
    end

    always_ff @(posedge clk) begin
        if (!rstn) begin
            out_valid <= 1'b0; out_data <= '0; out_channel <= '0;
            frame_start <= 1'b0; frame_first <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            if (cstate == C_EMIT && emit_idx < N_PAR && lane_mask[emit_lane]) begin
                out_valid   <= 1'b1;
                out_channel <= CH_W'(emit_lane * N_SLOTS + hold_slot);
                if (emit_rounded > OUT_MAX)      out_data <= OUT_MAX[OUT_W-1:0];
                else if (emit_rounded < OUT_MIN) out_data <= OUT_MIN[OUT_W-1:0];
                else                             out_data <= emit_rounded[OUT_W-1:0];
            end

            // frame_start leads the payload so the builder can lay the header down.
            frame_start <= start_pass;
            if (start_pass) frame_first <= 1'b1;
            else if (out_valid) frame_first <= 1'b0;
        end
    end

endmodule
