// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2025-2026 Caleb Kemere, Reet Sinha, Allen Mikhailov, Rice University

// =====================================================================
// lfp_halfband_dec2.sv
//
// Stage 1 of the LFP decimation chain: an 11-tap FIR decimating 30 kHz -> 15 kHz.
// One time-shared MAC (a single DSP48) serves every (lane x slot) channel.
//
// Why a first stage at all
// -----------------------
// A filter's transition width is a fraction of ITS sample rate, so the sharp
// 1.2/1.8 kHz filter is far cheaper to build after the rate has been halved:
// ~120 taps at 15 kHz instead of ~245 at 30 kHz, and half the delay-line BRAM.
// This stage exists only to make that halving safe -- it has to suppress
// whatever would fold onto the final passband.
//
// The default coefficients are a halfband (see lfp_coef_pkg), which is why 4 of
// the 11 taps are zero and the centre is exactly 0.5. The engine does NOT
// exploit that: it MACs all 11 taps unconditionally. Skipping the zeros would
// save 4 of 11 cycles the budget does not need, and would silently compute the
// wrong answer for any non-halfband coefficients the host uploads -- and the
// host is allowed to upload anything.
//
// Throughput
// ----------
// One output per channel every 2 packets, so the pass has 2 * (84 MHz / 30 kHz)
// = 5600 clocks. Cost is n_channels * N_TAPS = 256 * 11 = 2816, about half the
// budget. If a pass is still running when the next one is due, compute_overrun
// latches and the late frame is dropped whole -- never emitted half-summed.
//
// Fixed point
// -----------
// Samples in are DATA_W signed; coefficients are Q1.COEF_FRAC. The accumulator
// carries COEF_FRAC fractional bits, and the output keeps (OUT_W - DATA_W) of
// them so stage 2 starts from more resolution than it was handed -- rounding
// straight back to DATA_W here would inject quantisation noise ahead of the
// narrowband filter that has to live with it.
//
// MAC pipeline (3-cycle latency)
//   s0 (addr gen): drive the delay-line and coefficient read addresses, register markers
//   s1 (read)    : RAM data valid -> register the product
//   s2 (acc)     : accumulate; on the last tap of a channel, round/saturate and emit
// =====================================================================

module lfp_halfband_dec2 #(
    parameter  int N_LANES    = 8,     // CIPO streams packed per sample word
    parameter  int N_SLOTS    = 32,    // amplifier channels per lane
    parameter  int DATA_W     = 16,    // input sample width (signed)
    parameter  int COEF_W     = 18,    // coefficient width (signed, Q1.COEF_FRAC)
    parameter  int COEF_FRAC  = 17,    // fractional bits in the coefficients
    parameter  int ACC_W      = 48,    // MAC accumulator width
    parameter  int OUT_W      = 18,    // output width -- the stage-2 intermediate
    parameter  int N_TAPS     = 11,    // filter length
    parameter  int RING_DEPTH = 16,    // delay-line depth, power of 2, >= N_TAPS
    // ---- derived (do not override) ----
    localparam int SLOT_W  = (N_SLOTS  <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int RING_AW = $clog2(RING_DEPTH),
    localparam int TAP_W   = $clog2(N_TAPS + 1),
    localparam int N_CH    = N_LANES * N_SLOTS,
    localparam int CH_W    = (N_CH    <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W  = (N_LANES <= 1) ? 1 : $clog2(N_LANES),
    localparam int MEM_AW  = $clog2(N_SLOTS * RING_DEPTH)
) (
    input  logic                        clk,     // 84 MHz PL data clock
    input  logic                        rstn,

    // ---- sample tap from data_generator_core (one word per amplifier slot) ----
    input  logic                        sample_valid,
    input  logic [N_LANES*DATA_W-1:0]   sample_data,   // N_LANES x DATA_W signed, lane 0 low
    input  logic [SLOT_W-1:0]           sample_slot,
    input  logic                        packet_tick,   // the packet just written is complete

    // ---- configuration ----
    input  logic                        lfp_en,
    input  logic [N_LANES-1:0]          lane_mask,

    // ---- coefficient write port (synchronised to clk upstream) ----
    input  logic                        coef_wr_en,
    input  logic [TAP_W-1:0]            coef_wr_addr,
    input  logic [COEF_W-1:0]           coef_wr_data,

    // ---- decimated output, one sample per enabled channel per pass ----
    output logic                        out_valid,
    output logic [CH_W-1:0]             out_channel,
    output logic signed [OUT_W-1:0]     out_data,
    output logic                        out_frame_start,
    output logic                        busy,
    output logic                        compute_overrun
);

    import lfp_coef_pkg::*;

    localparam logic [RING_AW-1:0] RMASK  = RING_DEPTH - 1;
    localparam int                 PROD_W = DATA_W + COEF_W;
    // Drop COEF_FRAC fractional bits to return to input scale, then keep
    // (OUT_W - DATA_W) of them as extra resolution for stage 2.
    localparam int                 OUT_SHIFT = COEF_FRAC - (OUT_W - DATA_W);
    localparam signed [OUT_W:0]    OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0]    OUT_MIN = -(1 <<< (OUT_W-1));

    // =================================================================
    // Coefficient RAM. Small enough to be distributed RAM; initialised to the
    // designed halfband so the board filters correctly with no host upload.
    // =================================================================
    logic signed [COEF_W-1:0] coef_ram [0:N_TAPS-1];
    logic        [TAP_W-1:0]  coef_rd_addr;
    logic signed [COEF_W-1:0] coef_rdata;

    initial
        for (int i = 0; i < N_TAPS; i++)
            coef_ram[i] = (i < LFP_HB_TAPS) ? LFP_HB_DEFAULT_COEF[i] : '0;

    always_ff @(posedge clk) begin
        if (coef_wr_en) coef_ram[coef_wr_addr] <= coef_wr_data;
        coef_rdata <= coef_ram[coef_rd_addr];
    end

    // =================================================================
    // Delay line: one memory per lane, [slot][ring]. All N_LANES samples of a
    // slot arrive in the SAME word, so the lanes are written in parallel at a
    // shared address; during compute the lanes share a read address too and the
    // active one is muxed out, which keeps every memory single-ported.
    // =================================================================
    logic [RING_AW-1:0]       wr_pos;
    logic [MEM_AW-1:0]        dl_wr_addr, dl_rd_addr;
    logic signed [DATA_W-1:0] dl_rdata [0:N_LANES-1];

    assign dl_wr_addr = sample_slot * RING_DEPTH + wr_pos;

    genvar gl;
    generate
        for (gl = 0; gl < N_LANES; gl++) begin : g_lane_mem
            logic signed [DATA_W-1:0] mem [0:N_SLOTS*RING_DEPTH-1];
            initial for (int ii = 0; ii < N_SLOTS*RING_DEPTH; ii++) mem[ii] = '0;
            always_ff @(posedge clk) begin
                if (sample_valid) mem[dl_wr_addr] <= sample_data[gl*DATA_W +: DATA_W];
                dl_rdata[gl] <= mem[dl_rd_addr];
            end
        end
    endgenerate

    // =================================================================
    // Ingest pointer + /2 decimation.
    // =================================================================
    logic              decim_phase;   // /2: fire on every second packet
    logic [RING_AW-1:0] head_snap;    // ring head frozen for the running pass
    logic              start_pass;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            wr_pos <= '0; decim_phase <= 1'b0; head_snap <= '0; start_pass <= 1'b0;
        end else begin
            start_pass <= 1'b0;
            if (packet_tick) begin
                // The packet just written lives at wr_pos, so that is the newest
                // sample for any pass starting now.
                if (decim_phase) begin
                    decim_phase <= 1'b0;
                    if (lfp_en) begin
                        head_snap  <= wr_pos;
                        start_pass <= 1'b1;
                    end
                end else begin
                    decim_phase <= 1'b1;
                end
                wr_pos <= (wr_pos + 1'b1) & RMASK;
            end
        end
    end

    // =================================================================
    // Compute FSM: walk the enabled channels, one tap per clock.
    // =================================================================
    typedef enum logic [1:0] {C_IDLE, C_RUN, C_DRAIN} cstate_t;
    cstate_t           cstate;
    logic [LANE_W-1:0] cur_lane;
    logic [SLOT_W-1:0] cur_slot;
    logic [TAP_W-1:0]  cur_tap;
    logic [1:0]        drain_cnt;

    logic              ag_valid, ag_first, ag_last;
    logic [CH_W-1:0]   ag_chan;
    logic [LANE_W-1:0] ag_lane;

    wire last_lane = (cur_lane == LANE_W'(N_LANES-1));
    wire last_slot = (cur_slot == SLOT_W'(N_SLOTS-1));
    wire last_tap  = (cur_tap  == TAP_W'(N_TAPS-1));

    // Tap t is the sample t packets old: walk backwards from the frozen head.
    always_comb begin
        dl_rd_addr   = cur_slot * RING_DEPTH + ((head_snap - cur_tap[RING_AW-1:0]) & RMASK);
        coef_rd_addr = cur_tap;
    end

    always_ff @(posedge clk) begin
        if (!rstn) begin
            cstate <= C_IDLE;
            cur_lane <= '0; cur_slot <= '0; cur_tap <= '0; drain_cnt <= '0;
            ag_valid <= 1'b0; ag_first <= 1'b0; ag_last <= 1'b0;
            ag_chan <= '0; ag_lane <= '0;
            busy <= 1'b0; compute_overrun <= 1'b0;
        end else begin
            ag_valid <= 1'b0;

            // A new pass due while the previous one is alive: the design cannot
            // keep up with its own configuration. Latch it and drop the frame.
            if (start_pass && cstate != C_IDLE) compute_overrun <= 1'b1;

            case (cstate)
                C_IDLE: begin
                    busy <= 1'b0;
                    if (start_pass) begin
                        cur_lane <= '0; cur_slot <= '0; cur_tap <= '0;
                        cstate <= C_RUN; busy <= 1'b1;
                    end
                end

                C_RUN: begin
                    busy <= 1'b1;
                    if (lane_mask[cur_lane]) begin
                        ag_valid <= 1'b1;
                        ag_first <= (cur_tap == '0);
                        ag_last  <= last_tap;
                        ag_lane  <= cur_lane;
                        ag_chan  <= CH_W'(cur_lane * N_SLOTS + cur_slot);

                        if (last_tap) begin
                            cur_tap <= '0;
                            if (last_slot) begin
                                cur_slot <= '0;
                                if (last_lane) begin cstate <= C_DRAIN; drain_cnt <= 2'd3; end
                                else           cur_lane <= cur_lane + 1'b1;
                            end else begin
                                cur_slot <= cur_slot + 1'b1;
                            end
                        end else begin
                            cur_tap <= cur_tap + 1'b1;
                        end
                    end else begin
                        // Disabled lane: skip it whole, spending no MAC cycles.
                        cur_slot <= '0; cur_tap <= '0;
                        if (last_lane) begin cstate <= C_DRAIN; drain_cnt <= 2'd3; end
                        else           cur_lane <= cur_lane + 1'b1;
                    end
                end

                C_DRAIN: begin
                    busy <= 1'b1;                       // let the MAC pipeline empty
                    if (drain_cnt == 0) cstate <= C_IDLE;
                    else                drain_cnt <= drain_cnt - 1'b1;
                end

                default: cstate <= C_IDLE;
            endcase
        end
    end

    // =================================================================
    // MAC pipeline. Everything is explicitly signed: one unsigned operand would
    // make the whole expression unsigned and turn negative partial sums into
    // large positives.
    // =================================================================
    logic signed [PROD_W-1:0] prod1;
    logic                     v1, first1, last1;
    logic [CH_W-1:0]          chan1;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            prod1 <= '0; v1 <= 1'b0; first1 <= 1'b0; last1 <= 1'b0; chan1 <= '0;
        end else begin
            // Widen both operands to the product width BEFORE multiplying: a bare
            // a*b takes a self-determined width of max(|a|,|b|) and would truncate.
            prod1  <= PROD_W'($signed(dl_rdata[ag_lane])) * PROD_W'($signed(coef_rdata));
            v1     <= ag_valid;
            first1 <= ag_first;
            last1  <= ag_last;
            chan1  <= ag_chan;
        end
    end

    localparam signed [ACC_W-1:0] RND = (OUT_SHIFT > 0) ? (ACC_W'(1) <<< (OUT_SHIFT-1)) : '0;
    logic signed [ACC_W-1:0] acc, acc_sum, rounded;
    logic                    frame_first;
    wire                     mac_out = v1 & last1;

    always_comb begin
        acc_sum = first1 ? ACC_W'(prod1) : (acc + ACC_W'(prod1));
        rounded = (acc_sum + RND) >>> OUT_SHIFT;
    end

    always_ff @(posedge clk) begin
        if (!rstn) begin
            acc <= '0; out_valid <= 1'b0; out_data <= '0; out_channel <= '0;
            out_frame_start <= 1'b0; frame_first <= 1'b0;
        end else begin
            if (v1) acc <= acc_sum;

            out_valid   <= mac_out;
            out_channel <= chan1;
            if (rounded > OUT_MAX)      out_data <= OUT_MAX[OUT_W-1:0];
            else if (rounded < OUT_MIN) out_data <= OUT_MIN[OUT_W-1:0];
            else                        out_data <= rounded[OUT_W-1:0];

            if (start_pass)                 frame_first <= 1'b1;
            else if (mac_out & frame_first) frame_first <= 1'b0;
            out_frame_start <= mac_out & frame_first;
        end
    end

endmodule
