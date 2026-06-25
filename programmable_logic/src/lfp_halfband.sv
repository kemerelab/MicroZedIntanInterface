// =====================================================================
// lfp_halfband.sv
//
// A clean, parameterized time-shared *divide-by-2* decimating FIR. ONE pipelined
// MAC serves every (lane x slot) channel, exactly like lfp_fir_decimator but the
// decimation factor is fixed at 2 and the input arrives at an arbitrary (sub-30k)
// rate via a `sample_valid`/`packet_tick` handshake. Coefficients are
// host-loadable (Q1.COEF_FRAC), so this block can be a true linear-phase
// halfband OR a CIC droop-compensation low-pass.
//
// Phase A uses it as the 2nd stage of CIC(/5)+halfband(/2) (6 kHz -> 3 kHz).
// Phase B reuses it unchanged as the octave-cascade /2 building block.
//
// Per-channel state is the same shared-ring delay line as the FIR engine: all
// channels advance together (one new sample per `packet_tick`), so a single ring
// write-pointer is shared; the line is [lane][slot][ring]. One output per channel
// every 2 ticks. The compute pass on the decimation tick walks the lane_mask-
// enabled channels MACing the last `num_taps` samples against the coefficients.
//
// MAC pipeline = the proven 3-cycle registered path (s0 addr-gen / s1 product /
// s2 accumulate-round-saturate). Single MAC lane (a /2 stage at <=6 kHz has a
// huge clock budget: 6000 outputs/s * 256 ch * ~32 taps ~ 49 MMAC/s << 84 MHz).
// =====================================================================

module lfp_halfband #(
    parameter  int N_LANES    = 8,
    parameter  int N_SLOTS    = 32,
    parameter  int DATA_W     = 16,
    parameter  int COEF_W     = 18,
    parameter  int COEF_FRAC  = 17,
    parameter  int ACC_W      = 48,
    parameter  int RING_DEPTH = 64,    // /2 stage needs far fewer taps than the full FIR
    parameter  int OUT_W      = 16,
    // ---- derived (do not override) ----
    localparam int SLOT_W  = (N_SLOTS  <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int RING_AW = $clog2(RING_DEPTH),
    localparam int TAPN_W  = $clog2(RING_DEPTH + 1),
    localparam int N_CH    = N_LANES * N_SLOTS,
    localparam int CH_W    = (N_CH    <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W  = (N_LANES <= 1) ? 1 : $clog2(N_LANES),
    localparam int MEM_AW  = $clog2(N_SLOTS * RING_DEPTH)
) (
    input  logic                        clk,
    input  logic                        rstn,

    // ---- per-channel input sample stream ----
    input  logic                        sample_valid,   // pulse: a data word per slot
    input  logic [N_LANES*DATA_W-1:0]   sample_data,    // N_LANES x DATA_W signed
    input  logic [SLOT_W-1:0]           sample_slot,
    input  logic                        packet_tick,    // pulse: all slots of one input frame written

    // ---- configuration ----
    input  logic                        en,
    input  logic [N_LANES-1:0]          lane_mask,
    input  logic [TAPN_W-1:0]           num_taps,       // halfband length, <= RING_DEPTH

    // ---- coefficient indirect write port (synchronized to clk) ----
    input  logic                        coef_wr_en,
    input  logic [RING_AW-1:0]          coef_wr_addr,
    input  logic [COEF_W-1:0]           coef_wr_data,

    // ---- decimated (/2) output stream ----
    output logic                        out_valid,
    output logic [CH_W-1:0]             out_channel,
    output logic [OUT_W-1:0]            out_data,
    output logic                        out_frame_start,
    output logic                        frame_tick,     // pulse on the decimation tick (start of a frame)
    output logic                        busy,
    output logic                        compute_overrun
);

    localparam logic [RING_AW-1:0] RMASK   = RING_DEPTH - 1;
    localparam int                 PROD_W  = DATA_W + COEF_W;
    localparam signed [OUT_W:0]    OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0]    OUT_MIN = -(1 <<< (OUT_W-1));

    // ---- coefficient RAM (shared) ----
    logic signed [COEF_W-1:0] coef_ram [0:RING_DEPTH-1];
    logic signed [COEF_W-1:0] coef_rdata;
    logic        [RING_AW-1:0] coef_rd_addr;
    initial for (int ii = 0; ii < RING_DEPTH; ii++) coef_ram[ii] = '0;
    always_ff @(posedge clk) begin
        if (coef_wr_en) coef_ram[coef_wr_addr] <= coef_wr_data;
        coef_rdata <= coef_ram[coef_rd_addr];
    end

    // ---- delay-line BRAMs (one per lane, [slot][ring]) ----
    logic [RING_AW-1:0] wr_pos;
    logic [MEM_AW-1:0]  dl_wr_addr, dl_rd_addr;
    logic               dl_we;
    logic signed [DATA_W-1:0] dl_rdata [0:N_LANES-1];
    assign dl_we      = sample_valid;
    assign dl_wr_addr = sample_slot * RING_DEPTH + wr_pos;
    genvar gl;
    generate
        for (gl = 0; gl < N_LANES; gl++) begin : g_lane_mem
            logic signed [DATA_W-1:0] mem [0:N_SLOTS*RING_DEPTH-1];
            initial for (int ii = 0; ii < N_SLOTS*RING_DEPTH; ii++) mem[ii] = '0;
            always_ff @(posedge clk) begin
                if (dl_we) mem[dl_wr_addr] <= sample_data[gl*DATA_W +: DATA_W];
                dl_rdata[gl] <= mem[dl_rd_addr];
            end
        end
    endgenerate

    // ---- ingest pointer + /2 counter ----
    logic               decim_phase;   // 0/1; output every 2nd tick
    logic [RING_AW-1:0] head_snap;
    logic               start_pass;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            wr_pos <= '0; decim_phase <= 1'b0; head_snap <= '0; start_pass <= 1'b0;
        end else begin
            start_pass <= 1'b0;
            if (packet_tick) begin
                if (decim_phase) begin
                    decim_phase <= 1'b0;
                    if (en) begin head_snap <= wr_pos; start_pass <= 1'b1; end
                    // (start_pass is exposed as frame_tick below -- the decimation
                    //  tick that begins a new output frame; the LFP block latches
                    //  the master timestamp here.)
                end else begin
                    decim_phase <= 1'b1;
                end
                wr_pos <= (wr_pos + 1'b1) & RMASK;
            end
        end
    end

    // ---- compute FSM (address generation) ----
    typedef enum logic [1:0] {C_IDLE, C_RUN, C_DRAIN} cstate_t;
    cstate_t            cstate;
    logic [LANE_W-1:0]  cur_lane;
    logic [SLOT_W-1:0]  cur_slot;
    logic [TAPN_W-1:0]  cur_tap;
    logic [1:0]         drain_cnt;
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
            cstate <= C_IDLE; cur_lane <= '0; cur_slot <= '0; cur_tap <= '0;
            drain_cnt <= '0; ag_valid <= 1'b0; ag_first <= 1'b0; ag_last <= 1'b0;
            ag_chan <= '0; ag_lane <= '0; busy <= 1'b0; compute_overrun <= 1'b0;
        end else begin
            ag_valid <= 1'b0;
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
                            end else cur_slot <= cur_slot + 1'b1;
                        end else cur_tap <= cur_tap + 1'b1;
                    end else begin
                        cur_slot <= '0; cur_tap <= '0;
                        if (last_lane) begin cstate <= C_DRAIN; drain_cnt <= 2'd3; end
                        else           cur_lane <= cur_lane + 1'b1;
                    end
                end
                C_DRAIN: begin
                    busy <= 1'b1;
                    if (drain_cnt == 0) cstate <= C_IDLE;
                    else                drain_cnt <= drain_cnt - 1'b1;
                end
                default: cstate <= C_IDLE;
            endcase
        end
    end

    // ---- MAC pipeline ----
    logic signed [PROD_W-1:0] prod1;
    logic                     v1, first1, last1;
    logic [CH_W-1:0]          chan1;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            prod1 <= '0; v1 <= 1'b0; first1 <= 1'b0; last1 <= 1'b0; chan1 <= '0;
        end else begin
            prod1  <= PROD_W'($signed(dl_rdata[ag_lane])) * PROD_W'($signed(coef_rdata));
            v1     <= ag_valid; first1 <= ag_first; last1 <= ag_last; chan1 <= ag_chan;
        end
    end
    localparam signed [ACC_W-1:0] RND = ACC_W'(1) <<< (COEF_FRAC-1);
    logic signed [ACC_W-1:0] acc, acc_sum, rounded;
    logic                    frame_first;
    wire                     mac_out = v1 & last1;
    always_comb begin
        acc_sum = first1 ? $signed(prod1) : (acc + $signed(prod1));
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
            if (start_pass)                 frame_first <= 1'b1;
            else if (mac_out & frame_first) frame_first <= 1'b0;
            out_frame_start <= mac_out & frame_first;
        end
    end

    // Decimation tick -> frame boundary (lead-in pulse, ~num_taps clk ahead of the
    // first out_valid). The LFP block uses this to write the packet header.
    assign frame_tick = start_pass;
endmodule
