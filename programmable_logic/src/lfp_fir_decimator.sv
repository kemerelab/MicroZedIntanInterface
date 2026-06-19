// =====================================================================
// lfp_fir_decimator.sv
//
// On-PL LFP extraction: a time-shared decimating FIR. ONE pipelined MAC
// serves every (lane x slot) channel, so the whole thing costs ~1 DSP48 +
// a per-lane delay-line BRAM + a small shared coefficient RAM.
//
// See docs/lfp-dsp-engine-design.md for the architecture rationale.
//
// Sample model
// ------------
// The integration layer feeds one sample word per AMPLIFIER channel
// (sample_slot 0..N_SLOTS-1, N_SLOTS = 32), each word carrying N_LANES 16-bit
// CIPO samples already converted offset-binary -> two's-complement signed.
// (Upstream it drops the 3 aux slots and removes the 2-cycle SPI readback
// offset: data_generator_core cycle_counter 2..33 -> slot 0..31.) Every
// (lane, slot) pair is one independent channel sampled once per packet
// (30 kHz). Because all channels advance together (exactly one new sample each
// per packet) a SINGLE ring write-pointer is shared by all of them -- the
// delay line is just [lane][slot][ring], addressed
// slot*RING_DEPTH + ((head - tap) & (RING_DEPTH-1)).
//
// Decimation
// ----------
// One output per channel every DECIM_R packets. On the decimation tick the
// compute FSM walks the lane_mask-enabled channels and, for each, MACs the
// last `num_taps` samples against the shared coefficients.
//
// Throughput note (v1 = single MAC, 1 tap/clock)
// ----------------------------------------------
// The compute pass must finish within DECIM_R packets. Budget (84 MHz,
// 30 kHz packets) = DECIM_R * ~2800 clocks. Cost = n_enabled_channels *
// num_taps MACs. At R=15 that is ~42000 clocks for up to ~256ch*160tap, or
// ~128ch*312tap. If the host exceeds the budget, `compute_overrun` latches
// and the late frame is dropped (never corrupted). Parallel MAC lanes are a
// later optimization.
//
// MAC pipeline (3-cycle latency)
//   s0 (addr gen): drive dl_rd_addr/coef_rd_addr (comb) + register markers
//   s1 (read)    : dl_rdata[ag_lane], coef_rdata valid -> register product
//   s2 (acc)     : accumulate; emit output on the last tap of a channel
// =====================================================================

module lfp_fir_decimator #(
    parameter  int N_LANES    = 8,     // CIPO streams packed per sample word
    parameter  int N_SLOTS    = 32,    // amplifier channels per lane (aux slots dropped upstream)
    parameter  int DATA_W     = 16,    // sample width (signed; offset->signed done upstream)
    parameter  int COEF_W     = 18,    // coefficient width (signed, Q1.COEF_FRAC)
    parameter  int COEF_FRAC  = 17,    // fractional bits in the coefficients
    parameter  int ACC_W      = 48,    // MAC accumulator width
    parameter  int RING_DEPTH = 256,   // delay-line depth per channel (power of 2)
    parameter  int OUT_W      = 16,    // decimated output sample width
    // ---- derived widths (do not override) ----
    localparam int SLOT_W  = (N_SLOTS  <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int RING_AW = $clog2(RING_DEPTH),
    localparam int TAPN_W  = $clog2(RING_DEPTH + 1),
    localparam int N_CH    = N_LANES * N_SLOTS,
    localparam int CH_W    = (N_CH    <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W  = (N_LANES <= 1) ? 1 : $clog2(N_LANES),
    localparam int MEM_AW  = $clog2(N_SLOTS * RING_DEPTH)
) (
    input  logic                        clk,    // 84 MHz PL data clock
    input  logic                        rstn,   // 84 MHz domain reset, active low

    // ---- Sample tap from data_generator_core (one data word per slot) ----
    input  logic                        sample_valid,   // pulse: a data word is on sample_data
    input  logic [N_LANES*DATA_W-1:0]   sample_data,    // N_LANES x DATA_W signed, lane 0 in low bits
    input  logic [SLOT_W-1:0]           sample_slot,    // amplifier channel index, 0..N_SLOTS-1
    input  logic                        packet_tick,    // pulse: the just-written packet is complete

    // ---- Configuration (host latches these while streaming is stopped) ----
    input  logic                        lfp_en,         // master enable for the compute pass
    input  logic [N_LANES-1:0]          lane_mask,      // which lanes to filter (LFP stream mask)
    input  logic [7:0]                  decim_R,        // packets per output (>=1)
    input  logic [TAPN_W-1:0]           num_taps,       // active FIR length, 1..RING_DEPTH

    // ---- Coefficient indirect write port (already synchronized to clk) ----
    input  logic                        coef_wr_en,     // 1-cycle write strobe
    input  logic [RING_AW-1:0]          coef_wr_addr,   // tap index 0..RING_DEPTH-1
    input  logic [COEF_W-1:0]           coef_wr_data,   // signed coefficient

    // ---- Decimated output stream (to the LFP output ring) ----
    output logic                        out_valid,      // pulse per decimated sample
    output logic [CH_W-1:0]             out_channel,    // lane*N_SLOTS + slot
    output logic [OUT_W-1:0]            out_data,       // decimated sample (signed)
    output logic                        out_frame_start,// pulse at the first output of a frame
    output logic                        busy,           // compute pass in progress
    output logic                        compute_overrun // sticky: a frame was dropped (budget exceeded)
);

    localparam logic [RING_AW-1:0] RMASK   = RING_DEPTH - 1;
    localparam int                 PROD_W  = DATA_W + COEF_W;
    localparam signed [OUT_W:0]    OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0]    OUT_MIN = -(1 <<< (OUT_W-1));

    // =====================================================================
    // Coefficient RAM (shared, RING_DEPTH deep). Written from the config
    // side, read by the MAC. Coefficients are only written while stopped,
    // but keep it a true dual-port so a stray overlap can't X the read.
    // =====================================================================
    logic signed [COEF_W-1:0] coef_ram [0:RING_DEPTH-1];
    logic signed [COEF_W-1:0] coef_rdata;
    logic        [RING_AW-1:0] coef_rd_addr;

    initial for (int ii = 0; ii < RING_DEPTH; ii++) coef_ram[ii] = '0;  // BRAM config-init

    always_ff @(posedge clk) begin
        if (coef_wr_en) coef_ram[coef_wr_addr] <= coef_wr_data;
        coef_rdata <= coef_ram[coef_rd_addr];
    end

    // =====================================================================
    // Delay-line BRAMs: one per lane, [slot][ring]. Written in parallel on
    // sample_valid (lanes share the write address); read one at a time
    // during compute (lanes share the read address, selected lane muxed out).
    // =====================================================================
    logic [RING_AW-1:0] wr_pos;        // ring position of the in-progress packet
    logic [MEM_AW-1:0]  dl_wr_addr, dl_rd_addr;
    logic               dl_we;
    logic signed [DATA_W-1:0] dl_rdata [0:N_LANES-1];

    assign dl_we      = sample_valid;
    assign dl_wr_addr = sample_slot * RING_DEPTH + wr_pos;

    genvar gl;
    generate
        for (gl = 0; gl < N_LANES; gl++) begin : g_lane_mem
            logic signed [DATA_W-1:0] mem [0:N_SLOTS*RING_DEPTH-1];
            initial for (int ii = 0; ii < N_SLOTS*RING_DEPTH; ii++) mem[ii] = '0;  // BRAM config-init
            always_ff @(posedge clk) begin
                if (dl_we)
                    mem[dl_wr_addr] <= sample_data[gl*DATA_W +: DATA_W];
                dl_rdata[gl] <= mem[dl_rd_addr];
            end
        end
    endgenerate

    // =====================================================================
    // Ingest pointer + decimation counter.
    // =====================================================================
    logic [7:0]         decim_cnt;
    logic [RING_AW-1:0] head_snap;     // ring head used by the current compute pass
    logic               start_pass;    // 1-cycle pulse to kick the compute FSM

    always_ff @(posedge clk) begin
        if (!rstn) begin
            wr_pos     <= '0;
            decim_cnt  <= '0;
            head_snap  <= '0;
            start_pass <= 1'b0;
        end else begin
            start_pass <= 1'b0;
            if (packet_tick) begin
                // The packet just written lives at wr_pos -> snapshot it as the
                // most-recent sample for any compute pass triggered now.
                if (decim_cnt + 8'd1 >= decim_R) begin
                    decim_cnt <= '0;
                    if (lfp_en) begin
                        head_snap  <= wr_pos;
                        start_pass <= 1'b1;
                    end
                end else begin
                    decim_cnt <= decim_cnt + 8'd1;
                end
                wr_pos <= (wr_pos + 1'b1) & RMASK;
            end
        end
    end

    // =====================================================================
    // Compute FSM (address generation). Walks lane_mask-enabled channels,
    // emitting one (delay,coef) read per tap. Markers ride the pipeline so
    // the accumulate stage knows the first/last tap of each channel.
    // =====================================================================
    typedef enum logic [1:0] {C_IDLE, C_RUN, C_DRAIN} cstate_t;
    cstate_t            cstate;
    logic [LANE_W-1:0]  cur_lane;
    logic [SLOT_W-1:0]  cur_slot;
    logic [TAPN_W-1:0]  cur_tap;
    logic [1:0]         drain_cnt;

    // s0 markers (registered here, valid next cycle alongside the RAM reads).
    logic               ag_valid, ag_first, ag_last;
    logic [CH_W-1:0]    ag_chan;
    logic [LANE_W-1:0]  ag_lane;

    wire last_lane = (cur_lane == LANE_W'(N_LANES-1));
    wire last_slot = (cur_slot == SLOT_W'(N_SLOTS-1));
    wire last_tap  = (cur_tap  == num_taps - 1'b1);

    always_comb begin
        dl_rd_addr   = cur_slot * RING_DEPTH + ((head_snap - cur_tap) & RMASK);
        coef_rd_addr = cur_tap[RING_AW-1:0];
    end

    always_ff @(posedge clk) begin
        if (!rstn) begin
            cstate   <= C_IDLE;
            cur_lane <= '0; cur_slot <= '0; cur_tap <= '0;
            drain_cnt<= '0;
            ag_valid <= 1'b0; ag_first <= 1'b0; ag_last <= 1'b0;
            ag_chan  <= '0; ag_lane <= '0;
            busy     <= 1'b0;
            compute_overrun <= 1'b0;
        end else begin
            ag_valid <= 1'b0;

            // Overrun: a fresh pass requested while the previous one is alive.
            if (start_pass && cstate != C_IDLE)
                compute_overrun <= 1'b1;

            case (cstate)
                C_IDLE: begin
                    busy <= 1'b0;
                    if (start_pass) begin
                        cur_lane <= '0; cur_slot <= '0; cur_tap <= '0;
                        cstate   <= C_RUN;
                        busy     <= 1'b1;
                    end
                end

                C_RUN: begin
                    busy <= 1'b1;
                    if (lane_mask[cur_lane]) begin
                        // Emit a tap for (cur_lane, cur_slot, cur_tap).
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
                        // Skip a disabled lane (no emit).
                        cur_slot <= '0; cur_tap <= '0;
                        if (last_lane) begin cstate <= C_DRAIN; drain_cnt <= 2'd3; end
                        else           cur_lane <= cur_lane + 1'b1;
                    end
                end

                C_DRAIN: begin
                    // Let the MAC pipeline empty before declaring idle.
                    busy <= 1'b1;
                    if (drain_cnt == 0) cstate <= C_IDLE;
                    else                drain_cnt <= drain_cnt - 1'b1;
                end

                default: cstate <= C_IDLE;
            endcase
        end
    end

    // =====================================================================
    // MAC pipeline.
    // =====================================================================
    // s1: product (consumes the s0 markers + the now-valid RAM reads).
    logic signed [PROD_W-1:0] prod1;
    logic                     v1, first1, last1;
    logic [CH_W-1:0]          chan1;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            prod1 <= '0; v1 <= 1'b0; first1 <= 1'b0; last1 <= 1'b0; chan1 <= '0;
        end else begin
            prod1  <= dl_rdata[ag_lane] * coef_rdata;   // both signed
            v1     <= ag_valid;
            first1 <= ag_first;
            last1  <= ag_last;
            chan1  <= ag_chan;
        end
    end

    // s2: accumulate, round, saturate, emit. All arithmetic is kept explicitly
    // signed -- a single unsigned operand would make the whole expression
    // unsigned and turn negative partial sums into huge positives.
    localparam signed [ACC_W-1:0] RND = ACC_W'(1) <<< (COEF_FRAC-1);  // round-to-nearest
    logic signed [ACC_W-1:0] acc, acc_sum, rounded;
    logic                    frame_first;     // 1 until the first output of a pass
    wire                     mac_out = v1 & last1;

    always_comb begin
        acc_sum = first1 ? $signed(prod1) : (acc + prod1);
        rounded = (acc_sum + RND) >>> COEF_FRAC;
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

            // frame_first: set when a pass starts, cleared at its first output.
            if (start_pass)               frame_first <= 1'b1;
            else if (mac_out & frame_first) frame_first <= 1'b0;
            out_frame_start <= mac_out & frame_first;
        end
    end

endmodule
