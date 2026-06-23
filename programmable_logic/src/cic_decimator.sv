// =====================================================================
// cic_decimator.sv
//
// Time-shared CIC decimator (order N_ORDER, rate R, differential delay M=1),
// per channel, for the LFP front end. Integrators run at the input rate (one
// pass per packet_tick); combs run at the output rate (every R-th tick). All
// per-channel state lives in BRAM (no multipliers, no coefficient RAM, no long
// FIR delay line) -- this is the ~5x BRAM saving vs the single-stage FIR delay
// lines. One CIC instance feeds the lfp_halfband (/2) comp stage.
//
// Fixed-point: the integrator/comb accumulators are MODULAR two's-complement at
// ACC_W bits. The classic CIC guarantee: as long as
//   ACC_W >= input_W + ceil(N_ORDER*log2(R*M)),
// the modular wraparound is exact and the comb output is correct. The output is
// the final comb value arithmetic-right-shifted by GAIN_SHIFT (= the same
// ceil(...) term -> ~unity DC gain) then saturated to OUT_W.
//
// Schedule (all channels advance together: one new sample per packet_tick):
//   * INTEGRATE pass (every tick): walk channels x stages, cascade-add into the
//     per-channel integrator RAM. Pipelined read->add->write.
//   * On the R-th tick the COMB pass runs after INTEGRATE: walk channels,
//     cascade-diff the last integrator value against the per-channel comb RAM,
//     emit the gain-normalized output.
// Budget @ 84 MHz: INTEGRATE = N_CH*N_ORDER ops/tick (256*4=1024 << 2800 clk);
// COMB = N_CH*N_ORDER ops every R ticks (<< R*2800). compute_overrun latches if
// a pass cannot finish before the next tick (never corrupts: the late frame is
// dropped).
// =====================================================================

module cic_decimator #(
    parameter  int N_LANES   = 8,
    parameter  int N_SLOTS   = 32,
    parameter  int DATA_W    = 16,
    parameter  int R         = 5,
    parameter  int N_ORDER   = 4,
    parameter  int ACC_W     = 32,
    parameter  int OUT_W     = 16,
    parameter  int GAIN_SHIFT = 10,   // = ceil(N_ORDER*log2(R*M)); host/build picks
    // ---- derived ----
    localparam int SLOT_W = (N_SLOTS  <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int N_CH   = N_LANES * N_SLOTS,
    localparam int CH_W   = (N_CH    <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W = (N_LANES <= 1) ? 1 : $clog2(N_LANES),
    localparam int ORD_W  = (N_ORDER <= 1) ? 1 : $clog2(N_ORDER),
    localparam int STAGE_AW = $clog2(N_CH * N_ORDER)
) (
    input  logic                      clk,
    input  logic                      rstn,

    // ---- per-channel input sample stream (offset->signed done upstream) ----
    input  logic                      sample_valid,
    input  logic [N_LANES*DATA_W-1:0] sample_data,
    input  logic [SLOT_W-1:0]         sample_slot,
    input  logic                      packet_tick,

    // ---- configuration ----
    input  logic                      en,
    input  logic [N_LANES-1:0]        lane_mask,

    // ---- decimated (/R) output stream ----
    output logic                      out_valid,
    output logic [CH_W-1:0]           out_channel,
    output logic [OUT_W-1:0]          out_data,
    output logic                      out_frame_start,
    output logic                      busy,
    output logic                      compute_overrun
);

    localparam signed [OUT_W:0] OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0] OUT_MIN = -(1 <<< (OUT_W-1));

    // -----------------------------------------------------------------
    // Input capture: latch the just-arrived sample word per slot so the
    // INTEGRATE pass (which runs after packet_tick) can read it. We need the
    // whole packet's slots, so store them in a small per-(slot,lane) buffer.
    // -----------------------------------------------------------------
    logic signed [DATA_W-1:0] in_buf [0:N_SLOTS-1][0:N_LANES-1];
    genvar gs, gl;
    generate
        for (gs = 0; gs < N_SLOTS; gs++) begin : g_inbuf_s
            for (gl = 0; gl < N_LANES; gl++) begin : g_inbuf_l
                always_ff @(posedge clk)
                    if (sample_valid && sample_slot == SLOT_W'(gs))
                        in_buf[gs][gl] <= $signed(sample_data[gl*DATA_W +: DATA_W]);
            end
        end
    endgenerate

    // -----------------------------------------------------------------
    // Per-channel integrator + comb state RAMs. Address = ch*N_ORDER + stage.
    // -----------------------------------------------------------------
    logic signed [ACC_W-1:0] integ_ram [0:N_CH*N_ORDER-1];
    logic signed [ACC_W-1:0] comb_ram  [0:N_CH*N_ORDER-1];
    initial begin
        for (int i = 0; i < N_CH*N_ORDER; i++) begin integ_ram[i]='0; comb_ram[i]='0; end
    end

    // -----------------------------------------------------------------
    // Tick scheduling.
    // -----------------------------------------------------------------
    logic [$clog2(R+1)-1:0] decim_cnt;
    logic                   run_int;    // pulse: start an integrate pass
    logic                   run_comb;   // pulse: start a comb pass (after integrate)
    always_ff @(posedge clk) begin
        if (!rstn) begin
            decim_cnt <= '0; run_int <= 1'b0; run_comb <= 1'b0;
        end else begin
            run_int <= 1'b0; run_comb <= 1'b0;
            if (packet_tick && en) begin
                run_int <= 1'b1;
                if (decim_cnt + 1'b1 >= R[$clog2(R+1)-1:0]) begin
                    decim_cnt <= '0;
                    run_comb  <= 1'b1;     // comb pass is queued behind the integrate pass
                end else begin
                    decim_cnt <= decim_cnt + 1'b1;
                end
            end
        end
    end

    // -----------------------------------------------------------------
    // INTEGRATE pass FSM. For each enabled channel, cascade the N_ORDER
    // integrators: integ[0]+=x; integ[1]+=integ[0]; ... Sequentially walks
    // (lane,slot,stage). Pipelined: s0 addr, s1 read, s2 add+write.
    // The cascade input for stage k is the just-written stage k-1 value, so we
    // process stages strictly in order and forward the running accumulator.
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {I_IDLE, I_RUN, I_DONE} istate_t;
    istate_t            istate;
    logic [LANE_W-1:0]  i_lane;
    logic [SLOT_W-1:0]  i_slot;
    logic [ORD_W:0]     i_stage;        // 0..N_ORDER-1
    logic signed [ACC_W-1:0] i_run_acc; // running cascade accumulator (= prev stage out)
    logic               i_pending_comb; // remember to launch comb after integrate

    wire i_last_stage = (i_stage == ORD_W'(N_ORDER-1));
    wire i_last_slot  = (i_slot  == SLOT_W'(N_SLOTS-1));
    wire i_last_lane  = (i_lane  == LANE_W'(N_LANES-1));
    wire [CH_W-1:0] i_chan = CH_W'(i_lane * N_SLOTS + i_slot);

    // -----------------------------------------------------------------
    // COMB pass FSM. For each enabled channel, cascade the N_ORDER combs:
    // d[0]=integ_last - cprev[0]; cprev[0]=integ_last; d[1]=d[0]-cprev[1]; ...
    // Emits the gain-normalized, saturated output on the last stage.
    // -----------------------------------------------------------------
    typedef enum logic [1:0] {K_IDLE, K_RUN, K_DONE} kstate_t;
    kstate_t            kstate;
    logic [LANE_W-1:0]  k_lane;
    logic [SLOT_W-1:0]  k_slot;
    logic [ORD_W:0]     k_stage;
    logic signed [ACC_W-1:0] k_run;     // running comb cascade value
    logic               k_frame_first;
    wire k_last_stage = (k_stage == ORD_W'(N_ORDER-1));
    wire k_last_slot  = (k_slot  == SLOT_W'(N_SLOTS-1));
    wire k_last_lane  = (k_lane  == LANE_W'(N_LANES-1));
    wire [CH_W-1:0] k_chan = CH_W'(k_lane * N_SLOTS + k_slot);

    // RAM access arbitration: INTEGRATE owns integ_ram, COMB reads integ_ram
    // (last stage) + read/writes comb_ram. The two passes never overlap (comb is
    // launched only after integrate finishes), so a single combinational addr mux
    // per RAM is safe.
    logic [STAGE_AW-1:0] integ_addr;
    logic signed [ACC_W-1:0] integ_rd, integ_wr;
    logic                integ_we;
    logic [STAGE_AW-1:0] comb_addr;
    logic signed [ACC_W-1:0] comb_rd, comb_wr;
    logic                comb_we;

    always_ff @(posedge clk) begin
        integ_rd <= integ_ram[integ_addr];
        if (integ_we) integ_ram[integ_addr] <= integ_wr;
        comb_rd  <= comb_ram[comb_addr];
        if (comb_we) comb_ram[comb_addr] <= comb_wr;
    end

    // ---- INTEGRATE datapath (read stage k state @ s1, add running acc, write) ----
    // We march stage-by-stage; the running accumulator i_run_acc holds the output
    // of the previous stage (or the input x for stage 0). Because read latency is
    // 1, we register the address at s0, read at s1, and the add/write also at s1+1.
    // To keep it simple and correct we use a 2-cycle-per-stage schedule.
    logic               i_phase;        // 0 = issue read, 1 = add+write
    logic [STAGE_AW-1:0] i_addr_r;
    logic signed [ACC_W-1:0] i_x;       // stage-0 input for the current channel (held)
    logic [ORD_W:0]     i_stage_r;      // stage being written at phase1
    // combinational cascade input at phase1: stage 0 = channel input, else the
    // previous stage's output (held in i_run_acc from the prior phase1).
    logic signed [ACC_W-1:0] i_casc_in;
    always_comb i_casc_in = (i_stage_r == 0) ? i_x : i_run_acc;

    // ---- COMB datapath ----
    logic               k_phase;        // 0 = issue reads, 1 = compute+write
    logic [STAGE_AW-1:0] k_addr_r;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            istate <= I_IDLE; i_lane <= '0; i_slot <= '0; i_stage <= '0;
            i_run_acc <= '0; i_phase <= 1'b0; i_pending_comb <= 1'b0; i_x <= '0;
            i_stage_r <= '0; i_addr_r <= '0; k_addr_r <= '0;
            kstate <= K_IDLE; k_lane <= '0; k_slot <= '0; k_stage <= '0;
            k_run <= '0; k_phase <= 1'b0; k_frame_first <= 1'b0;
            integ_we <= 1'b0; comb_we <= 1'b0;
            out_valid <= 1'b0; out_channel <= '0; out_data <= '0; out_frame_start <= 1'b0;
            busy <= 1'b0; compute_overrun <= 1'b0;
        end else begin
            integ_we  <= 1'b0;
            comb_we   <= 1'b0;
            out_valid <= 1'b0;
            out_frame_start <= 1'b0;

            // overrun: a new tick arrives while a pass is still running
            if (run_int && (istate != I_IDLE || kstate != K_IDLE))
                compute_overrun <= 1'b1;

            // ============ INTEGRATE ============
            case (istate)
                I_IDLE: begin
                    if (run_int) begin
                        istate <= I_RUN;
                        i_lane <= '0; i_slot <= '0; i_stage <= '0;
                        i_phase <= 1'b0;
                        i_run_acc <= '0;
                        i_pending_comb <= run_comb;
                        i_x <= in_buf[0][0];          // (slot0,lane0) loaded below
                    end
                end
                I_RUN: begin
                    busy <= 1'b1;
                    if (lane_mask[i_lane]) begin
                        if (i_phase == 1'b0) begin
                            // issue read of stage state; latch the channel input
                            // and the stage index for phase1.
                            integ_addr <= STAGE_AW'(i_chan * N_ORDER + i_stage);
                            i_addr_r   <= STAGE_AW'(i_chan * N_ORDER + i_stage);
                            i_stage_r  <= i_stage;
                            i_x        <= in_buf[i_slot][i_lane];  // x for this channel
                            i_phase    <= 1'b1;
                        end else begin
                            // integ_rd valid now: new = state + cascade_in
                            integ_wr  <= integ_rd + i_casc_in;
                            integ_addr<= i_addr_r;
                            integ_we  <= 1'b1;
                            i_run_acc <= integ_rd + i_casc_in;   // feed next stage
                            i_phase   <= 1'b0;
                            // advance (lane,slot,stage)
                            if (i_last_stage) begin
                                i_stage <= '0;
                                if (i_last_slot) begin
                                    i_slot <= '0;
                                    if (i_last_lane) istate <= I_DONE;
                                    else             i_lane <= i_lane + 1'b1;
                                end else i_slot <= i_slot + 1'b1;
                            end else i_stage <= i_stage + 1'b1;
                        end
                    end else begin
                        // skip disabled lane
                        i_slot <= '0; i_stage <= '0; i_phase <= 1'b0;
                        if (i_last_lane) istate <= I_DONE;
                        else             i_lane <= i_lane + 1'b1;
                    end
                end
                I_DONE: begin
                    istate <= I_IDLE;
                    if (i_pending_comb) begin
                        kstate  <= K_RUN;
                        k_lane  <= '0; k_slot <= '0; k_stage <= '0;
                        k_phase <= 1'b0; k_run <= '0; k_frame_first <= 1'b1;
                    end else if (kstate == K_IDLE) begin
                        busy <= 1'b0;
                    end
                end
                default: istate <= I_IDLE;
            endcase

            // ============ COMB ============
            case (kstate)
                K_IDLE: ;
                K_RUN: begin
                    busy <= 1'b1;
                    if (lane_mask[k_lane]) begin
                        if (k_phase == 1'b0) begin
                            // read comb prev state AND (stage 0) the last integrator
                            comb_addr  <= STAGE_AW'(k_chan * N_ORDER + k_stage);
                            k_addr_r   <= STAGE_AW'(k_chan * N_ORDER + k_stage);
                            if (k_stage == 0)
                                integ_addr <= STAGE_AW'(k_chan * N_ORDER + (N_ORDER-1));
                            k_phase <= 1'b1;
                        end else begin
                            // for stage 0, k_run source = last integrator value
                            // (integ_rd valid this cycle); else k_run already holds
                            // previous stage diff.
                            logic signed [ACC_W-1:0] stage_in, diff;
                            stage_in = (k_stage == 0) ? integ_rd : k_run;
                            diff      = stage_in - comb_rd;     // x[n]-x[n-1]
                            comb_wr   <= stage_in;              // store current as prev
                            comb_addr <= k_addr_r;
                            comb_we   <= 1'b1;
                            k_run     <= diff;
                            k_phase   <= 1'b0;
                            if (k_last_stage) begin
                                // emit gain-normalized + saturated output
                                logic signed [ACC_W-1:0] y;
                                y = diff >>> GAIN_SHIFT;
                                out_valid   <= 1'b1;
                                out_channel <= k_chan;
                                if (y > OUT_MAX)      out_data <= OUT_MAX[OUT_W-1:0];
                                else if (y < OUT_MIN) out_data <= OUT_MIN[OUT_W-1:0];
                                else                  out_data <= y[OUT_W-1:0];
                                out_frame_start <= k_frame_first;
                                k_frame_first   <= 1'b0;
                                k_stage <= '0;
                                if (k_last_slot) begin
                                    k_slot <= '0;
                                    if (k_last_lane) kstate <= K_DONE;
                                    else             k_lane <= k_lane + 1'b1;
                                end else k_slot <= k_slot + 1'b1;
                            end else k_stage <= k_stage + 1'b1;
                        end
                    end else begin
                        k_slot <= '0; k_stage <= '0; k_phase <= 1'b0;
                        if (k_last_lane) kstate <= K_DONE;
                        else             k_lane <= k_lane + 1'b1;
                    end
                end
                K_DONE: begin
                    kstate <= K_IDLE;
                    busy   <= 1'b0;
                end
                default: kstate <= K_IDLE;
            endcase
        end
    end
endmodule
