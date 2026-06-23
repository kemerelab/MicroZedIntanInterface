// =====================================================================
// cic_decimator.sv
//
// Time-shared CIC decimator (order N_ORDER, rate R, differential delay M=1),
// per channel, for the LFP front end. Integrators run at the input rate (one
// pass per packet_tick); combs run at the output rate (every R-th tick). All
// per-channel state lives in BRAM (no multipliers, no coefficient RAM, no long
// FIR delay line) -- this is the ~5x delay-line-BRAM saving vs the single-stage
// FIR. One CIC instance feeds the lfp_halfband (/2) comp stage.
//
// State storage: the N_ORDER integrator (and comb) accumulators for a channel
// are packed into ONE wide word at address = channel. So each channel is a clean
// single read -> combinational cascade -> single write-back; no per-stage RAM
// addressing dance, no read-latency hazards. N_ORDER is small (4) so the
// combinational adder cascade is cheap at 84 MHz.
//
// Fixed-point: the integrator/comb accumulators are MODULAR two's-complement at
// ACC_W bits. The classic CIC guarantee: as long as
//   ACC_W >= input_W + ceil(N_ORDER*log2(R*M)),
// the modular wraparound is exact and the comb output is correct. For R=5, M=1,
// N=4, input 16b: need >= 16 + 10 = 26 -> ACC_W=32 is comfortable. The output is
// the final comb value arithmetic-right-shifted by GAIN_SHIFT (= the same
// ceil(...) term -> ~unity DC gain) then saturated to OUT_W.
//
// Schedule (all channels advance together: one new sample per packet_tick):
//   * INTEGRATE pass (every tick): per channel read its packed integrator word,
//     cascade-add the input through N_ORDER stages, write the new word back.
//   * On the R-th tick the COMB pass runs after INTEGRATE: per channel read the
//     packed integrator word (top stage = the decimated value) + the packed comb
//     word, cascade-diff, write comb back, emit the gain-normalized output.
// Budget @ 84 MHz: INTEGRATE = 2*N_CH cyc/tick (2*256=512 << 2800); COMB the same
// every R ticks. compute_overrun latches if a pass can't finish before the next
// tick (never corrupts: the late frame is dropped).
// =====================================================================

module cic_decimator #(
    parameter  int N_LANES   = 8,
    parameter  int N_SLOTS   = 32,
    parameter  int DATA_W    = 16,
    parameter  int R         = 5,
    parameter  int N_ORDER   = 4,
    parameter  int ACC_W     = 32,
    parameter  int OUT_W     = 16,
    parameter  int GAIN_SHIFT = 10,   // = ceil(N_ORDER*log2(R*M))
    // ---- derived ----
    localparam int SLOT_W = (N_SLOTS  <= 1) ? 1 : $clog2(N_SLOTS),
    localparam int N_CH   = N_LANES * N_SLOTS,
    localparam int CH_W   = (N_CH    <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W = (N_LANES <= 1) ? 1 : $clog2(N_LANES),
    localparam int WIDE_W = N_ORDER * ACC_W
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
    // INTEGRATE pass (which runs after packet_tick) can read it.
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
    // Per-channel packed state RAMs (one wide word per channel).
    // -----------------------------------------------------------------
    logic [WIDE_W-1:0] integ_ram [0:N_CH-1];
    logic [WIDE_W-1:0] comb_ram  [0:N_CH-1];
    initial for (int i = 0; i < N_CH; i++) begin integ_ram[i]='0; comb_ram[i]='0; end

    // ---- FSM state (declared up here so the combinational read-address mux can
    // reference the current channel before the FSM bodies below) ----
    typedef enum logic [1:0] {I_IDLE, I_RUN, I_DONE} istate_t;
    typedef enum logic [1:0] {K_IDLE, K_RUN, K_DONE} kstate_t;
    istate_t           istate;
    kstate_t           kstate;
    logic [LANE_W-1:0] i_lane, k_lane;
    logic [SLOT_W-1:0] i_slot, k_slot;
    logic              i_phase, k_phase;
    logic [CH_W-1:0]   i_ch_r, k_ch_r;
    logic signed [DATA_W-1:0] i_x_r;
    logic              i_pending_comb, k_frame_first;
    wire [CH_W-1:0] i_chan = CH_W'(i_lane * N_SLOTS + i_slot);
    wire [CH_W-1:0] k_chan = CH_W'(k_lane * N_SLOTS + k_slot);
    wire i_last_slot = (i_slot == SLOT_W'(N_SLOTS-1));
    wire i_last_lane = (i_lane == LANE_W'(N_LANES-1));
    wire k_last_slot = (k_slot == SLOT_W'(N_SLOTS-1));
    wire k_last_lane = (k_lane == LANE_W'(N_LANES-1));

    // Read address is COMBINATIONAL (driven from the FSM state at phase0) so the
    // registered read `*_rd` captures the addressed word at the phase0->phase1
    // edge and is valid AT phase1 -- a registered read address would land the
    // data one cycle late. The write address/data are registered (applied the
    // cycle after the read), to a separate held write address so a read and a
    // write to different locations coexist on the same inferred-BRAM port.
    logic [CH_W-1:0]   integ_raddr, comb_raddr;     // combinational read address
    logic [CH_W-1:0]   integ_waddr, comb_waddr;     // registered write address
    logic [WIDE_W-1:0] integ_rd, integ_wr, comb_rd, comb_wr;
    logic              integ_we, comb_we;
    always_ff @(posedge clk) begin
        integ_rd <= integ_ram[integ_raddr];
        if (integ_we) integ_ram[integ_waddr] <= integ_wr;
        comb_rd  <= comb_ram[comb_raddr];
        if (comb_we) comb_ram[comb_waddr] <= comb_wr;
    end

    // combinational read-address mux from whichever pass is active at phase0
    always_comb begin
        integ_raddr = i_chan;
        comb_raddr  = k_chan;
        if (kstate == K_RUN) integ_raddr = k_chan;   // comb reads integrators too
    end

    // -----------------------------------------------------------------
    // Tick scheduling.
    // -----------------------------------------------------------------
    logic [$clog2(R+1)-1:0] decim_cnt;
    logic                   run_int, run_comb;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            decim_cnt <= '0; run_int <= 1'b0; run_comb <= 1'b0;
        end else begin
            run_int <= 1'b0; run_comb <= 1'b0;
            if (packet_tick && en) begin
                run_int <= 1'b1;
                if (decim_cnt + 1'b1 >= R[$clog2(R+1)-1:0]) begin
                    decim_cnt <= '0;
                    run_comb  <= 1'b1;
                end else decim_cnt <= decim_cnt + 1'b1;
            end
        end
    end

    // -----------------------------------------------------------------
    // INTEGRATE FSM: per channel, 2 cycles (phase0 = drive read addr; phase1 =
    // integ_rd valid -> combinational cascade -> write back). Walk (lane,slot).
    // (state regs declared above the read-address mux.)
    // combinational integrator cascade over the packed read word
    // -----------------------------------------------------------------
    function automatic logic [WIDE_W-1:0] integ_cascade
            (input logic [WIDE_W-1:0] state, input logic signed [ACC_W-1:0] x);
        logic [WIDE_W-1:0] nstate;
        logic signed [ACC_W-1:0] acc, s_i;
        acc = x;
        for (int i = 0; i < N_ORDER; i++) begin
            s_i = $signed(state[i*ACC_W +: ACC_W]);
            s_i = s_i + acc;                  // modular ACC_W add
            nstate[i*ACC_W +: ACC_W] = s_i;
            acc = s_i;                        // cascade: next stage input = this output
        end
        return nstate;
    endfunction

    // -----------------------------------------------------------------
    // COMB FSM: per channel, 2 cycles (phase0 = drive read addrs; phase1 =
    // integ_rd + comb_rd valid -> combinational comb cascade -> write comb back,
    // emit output). Walk (lane,slot). (state regs declared above.)
    // combinational comb cascade. Input = the top integrator stage (decimated
    // value). cprev (packed) are the per-stage previous inputs; returns the new
    // packed cprev and the final difference.
    function automatic void comb_cascade
            (input  logic [WIDE_W-1:0] integ_state,
             input  logic [WIDE_W-1:0] cprev,
             output logic [WIDE_W-1:0] ncprev,
             output logic signed [ACC_W-1:0] result);
        logic signed [ACC_W-1:0] stage, prev, diff;
        stage = $signed(integ_state[(N_ORDER-1)*ACC_W +: ACC_W]);  // last integrator
        for (int i = 0; i < N_ORDER; i++) begin
            prev = $signed(cprev[i*ACC_W +: ACC_W]);
            diff = stage - prev;               // modular ACC_W
            ncprev[i*ACC_W +: ACC_W] = stage;  // store current as next prev
            stage = diff;                      // cascade
        end
        result = stage;
    endfunction

    always_ff @(posedge clk) begin
        if (!rstn) begin
            istate <= I_IDLE; i_lane <= '0; i_slot <= '0; i_phase <= 1'b0;
            i_ch_r <= '0; i_x_r <= '0; i_pending_comb <= 1'b0;
            kstate <= K_IDLE; k_lane <= '0; k_slot <= '0; k_phase <= 1'b0;
            k_ch_r <= '0; k_frame_first <= 1'b0;
            integ_waddr <= '0; comb_waddr <= '0; integ_wr <= '0; comb_wr <= '0;
            integ_we <= 1'b0; comb_we <= 1'b0;
            out_valid <= 1'b0; out_channel <= '0; out_data <= '0; out_frame_start <= 1'b0;
            busy <= 1'b0; compute_overrun <= 1'b0;
        end else begin
            integ_we  <= 1'b0;
            comb_we   <= 1'b0;
            out_valid <= 1'b0;
            out_frame_start <= 1'b0;

            if (run_int && (istate != I_IDLE || kstate != K_IDLE))
                compute_overrun <= 1'b1;

            // ============ INTEGRATE ============
            case (istate)
                I_IDLE: begin
                    if (run_int) begin
                        istate <= I_RUN; i_lane <= '0; i_slot <= '0; i_phase <= 1'b0;
                        i_pending_comb <= run_comb;
                    end
                end
                I_RUN: begin
                    busy <= 1'b1;
                    if (lane_mask[i_lane]) begin
                        if (i_phase == 1'b0) begin
                            // read addr is combinational (= i_chan); just latch
                            // the channel + input for the phase1 write.
                            i_ch_r     <= i_chan;
                            i_x_r      <= in_buf[i_slot][i_lane];
                            i_phase    <= 1'b1;
                        end else begin
                            // integ_rd valid: cascade, write back
                            integ_waddr <= i_ch_r;
                            integ_wr    <= integ_cascade(integ_rd, i_x_r);
                            integ_we    <= 1'b1;
                            i_phase     <= 1'b0;
                            if (i_last_slot) begin
                                i_slot <= '0;
                                if (i_last_lane) istate <= I_DONE;
                                else             i_lane <= i_lane + 1'b1;
                            end else i_slot <= i_slot + 1'b1;
                        end
                    end else begin
                        i_slot <= '0; i_phase <= 1'b0;
                        if (i_last_lane) istate <= I_DONE;
                        else             i_lane <= i_lane + 1'b1;
                    end
                end
                I_DONE: begin
                    istate <= I_IDLE;
                    if (i_pending_comb) begin
                        kstate <= K_RUN; k_lane <= '0; k_slot <= '0; k_phase <= 1'b0;
                        k_frame_first <= 1'b1;
                    end else if (kstate == K_IDLE) busy <= 1'b0;
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
                            // read addrs combinational (integ_raddr/comb_raddr =
                            // k_chan); latch the channel for the phase1 write.
                            k_ch_r     <= k_chan;
                            k_phase    <= 1'b1;
                        end else begin
                            logic [WIDE_W-1:0] ncprev;
                            logic signed [ACC_W-1:0] res, y;
                            comb_cascade(integ_rd, comb_rd, ncprev, res);
                            comb_waddr <= k_ch_r;
                            comb_wr    <= ncprev;
                            comb_we    <= 1'b1;
                            y = res >>> GAIN_SHIFT;
                            out_valid   <= 1'b1;
                            out_channel <= k_ch_r;
                            if (y > OUT_MAX)      out_data <= OUT_MAX[OUT_W-1:0];
                            else if (y < OUT_MIN) out_data <= OUT_MIN[OUT_W-1:0];
                            else                  out_data <= y[OUT_W-1:0];
                            out_frame_start <= k_frame_first;
                            k_frame_first   <= 1'b0;
                            k_phase <= 1'b0;
                            if (k_last_slot) begin
                                k_slot <= '0;
                                if (k_last_lane) kstate <= K_DONE;
                                else             k_lane <= k_lane + 1'b1;
                            end else k_slot <= k_slot + 1'b1;
                        end
                    end else begin
                        k_slot <= '0; k_phase <= 1'b0;
                        if (k_last_lane) kstate <= K_DONE;
                        else             k_lane <= k_lane + 1'b1;
                    end
                end
                K_DONE: begin kstate <= K_IDLE; busy <= 1'b0; end
                default: kstate <= K_IDLE;
            endcase
        end
    end
endmodule
