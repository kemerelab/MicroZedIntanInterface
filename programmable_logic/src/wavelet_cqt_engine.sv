// =====================================================================
// wavelet_cqt_engine.sv  --  Tier-3 multirate wavelet (constant-Q / Morse)
// scalogram engine. ONE time-shared MAC computes both the octave-cascade
// halfband ÷2 decimations and the V complex bandpass voices per octave,
// producing a real-time scalogram of K selected channels over N_OCTAVES
// octaves x V complex voices.
//
// Generalizes lfp_fir_decimator.sv (time-shared MAC, shared coef RAM,
// per-channel delay-line BRAM, compute-overrun guard) to:
//   * COMPLEX voice coefficients (Q1.17 re/im). The LFP input is real, so a
//     complex tap is just acc_re += cre*x and acc_im += cim*x -- 2 real MACs
//     per tap sharing the same sample.
//   * V voices per octave sharing N_TAPS-long complex shapes (constant-Q):
//     the SAME V shapes are reused at every octave (the octave sample rate
//     halves, so each band's center freq in Hz halves -- a dyadic cascade).
//   * a multirate à-trous octave cascade: octave o runs at fs/2^o, fed by a
//     halfband ÷2 of octave o-1's stream. Per-(lane,octave) sample rings.
//
// SINGLE-MAC discipline: the engine owns one ring read port, one voice-coef
// read port, one halfband-coef read port, and one MAC pipeline. The FSM
// sequences (a) HALFBAND passes -- HB_TAPS real MACs producing one ÷2 sample
// stored back into the next octave's ring, and (b) VOICE passes -- 2*n_taps
// real MACs (re then im phase) producing one complex bin written to the
// results BRAM. A standalone wavelet_halfband.sv exists as the reusable ÷2
// primitive (+ its own unit TB); the engine integrates the same arithmetic
// for clean single-bus scheduling.
//
// Scheduling (K=32 first build = single MAC, naive per-base-frame pass):
//   On every base-rate LFP frame: stage each lane's new octave-0 sample,
//   then run one pass that (1) commits octave-0 samples, (2) cascades the
//   halfband for every octave that advances this frame (octave o advances
//   iff fcount mod 2^o == 0), (3) runs the V voices for every advanced
//   octave and writes its complex column to the results BRAM. Worst case
//   (octave 0, every frame): K*(V*2*N_TAPS) MAC cycles + a little overhead
//   -- tiny vs the ~28000-clock base-frame budget at 3 kHz. `overrun`
//   latches if a fresh frame arrives mid-pass (late frame dropped, never
//   corrupted) -- exactly lfp_fir_decimator's guarantee.
//
// Results BRAM layout -- THE COMPLETE WIRE PACKET (32-bit words; byte addr =
// word<<2), so the PS just DMAs the whole frame into a pbuf and sends it (no
// PS-side header math or compaction loop), exactly as lfp_dsp_block does:
//
//   [ 8-word header | compacted (lane,scale) re/im payload ]
//
//   Header (matches remote/net.py receive_wavelet + the firmware wire format):
//     w0 = WAV_MAGIC_LOW  (0x5CA70900)
//     w1 = WAV_MAGIC_HIGH (0xCAFEBABE)
//     w2 = seq           (== frame_seq, the completed-column count)
//     w3 = 0
//     w4 = n_octaves | (n_voices<<8) | (K<<16) | (overrun<<24)
//     w5 = seq           (duplicate, torn-frame cross-check)
//     w6 = nscales | (n_taps<<16)        nscales = n_octaves*n_voices (runtime)
//     w7 = gain          (4 bits/octave left-shift, the host gain word)
//   Payload (COMPACTED -- lane stride = nscales, NOT the padded build max):
//     word_addr(lane,scale) = HDR_WORDS + (lane*nscales + scale)*2 ; +0=re,+1=im
//     scale = octave*n_voices + voice. Values signed OUT_W, sign-extended to 32.
//     Octaves that did not advance this frame keep their previous column (the
//     per-(lane,scale) address is stable while the config is held, so the snap-
//     shot semantics survive compaction).
//
//   nscales is RUNTIME (n_octaves*n_voices), <= the build max N_OCTAVES*V. The
//   header + compacted layout are built in the PL so the PS does a single
//   DMA+send (the hard PL->PS DMA rule -- no CPU repack of the staging buffer).
//
// Voice coef RAM: V*N_TAPS complex pairs interleaved -- word
//   2*(v*N_TAPS+j)+0 = re, +1 = im.
// =====================================================================
module wavelet_cqt_engine #(
    parameter int N_CH      = 256,
    parameter int K         = 4,
    parameter int N_OCTAVES = 4,
    parameter int V         = 4,
    parameter int N_TAPS    = 24,
    parameter int HB_TAPS   = 7,
    parameter int DATA_W    = 16,
    parameter int COEF_W    = 18,
    parameter int COEF_FRAC = 17,
    parameter int ACC_W     = 48,
    parameter int OUT_W     = 18,
    parameter int RING_DEPTH= 64,   // >= max(N_TAPS, HB_TAPS+1), power of 2
    parameter int RES_AW    = 14,
    // derived
    localparam int CH_W   = (N_CH <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W = (K    <= 1) ? 1 : $clog2(K),
    localparam int OCT_W  = (N_OCTAVES <= 1) ? 1 : $clog2(N_OCTAVES),
    localparam int VOICE_W= (V    <= 1) ? 1 : $clog2(V),
    localparam int TAP_W  = (N_TAPS<= 1) ? 1 : $clog2(N_TAPS),
    localparam int HBTAP_W= (HB_TAPS<=1) ? 1 : $clog2(HB_TAPS),
    localparam int RING_AW= $clog2(RING_DEPTH),
    localparam int COEFN  = V * N_TAPS,
    localparam int COEF_AW= $clog2(2*COEFN)
) (
    input  logic clk,
    input  logic rstn,

    // ---- Tier-1 LFP output stream tap (signed, from lfp_dsp_block) ----
    input  logic                 lfp_out_valid,
    input  logic [CH_W-1:0]      lfp_out_channel,
    input  logic signed [DATA_W-1:0] lfp_out_data,
    input  logic                 lfp_frame_start,

    // ---- configuration (host, latched while disabled) ----
    input  logic                 wav_en,
    input  logic [OCT_W:0]       n_octaves_cfg,
    input  logic [VOICE_W:0]     n_voices_cfg,
    input  logic [TAP_W:0]       n_taps_cfg,
    input  logic [4*N_OCTAVES-1:0] gain_cfg,      // 4 bits/octave (left-shift)

    // ---- channel selector write ----
    input  logic                 sel_wr_en,
    input  logic [LANE_W-1:0]    sel_wr_lane,
    input  logic [CH_W-1:0]      sel_wr_ch,

    // ---- voice coef write (interleaved re,im) ----
    input  logic                 coef_wr_en,
    input  logic [COEF_AW-1:0]   coef_wr_addr,
    input  logic signed [COEF_W-1:0] coef_wr_data,

    // ---- halfband coef write ----
    input  logic                 hb_wr_en,
    input  logic [HBTAP_W-1:0]   hb_wr_addr,
    input  logic signed [COEF_W-1:0] hb_wr_data,

    // ---- results BRAM port A ----
    output logic                 res_bram_clk,
    output logic                 res_bram_rst,
    output logic [RES_AW-1:0]    res_bram_addr,
    output logic [31:0]          res_bram_din,
    input  logic [31:0]          res_bram_dout,
    output logic                 res_bram_en,
    output logic [3:0]           res_bram_we,

    // ---- status ----
    output logic [31:0]          frame_seq,
    output logic                 busy,
    output logic                 overrun
);
    localparam int PROD_W  = DATA_W + COEF_W;
    localparam int RES_WAW = RES_AW - 2;

    // Wire-packet header (matches remote/net.py receive_wavelet + the firmware).
    localparam int          HDR_WORDS      = 8;
    localparam logic [31:0] WAV_MAGIC_LOW  = 32'h5CA70900;
    localparam logic [31:0] WAV_MAGIC_HIGH = 32'hCAFEBABE;
    localparam signed [OUT_W:0]  OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0]  OUT_MIN = -(1 <<< (OUT_W-1));
    localparam signed [DATA_W:0] DAT_MAX =  (1 <<< (DATA_W-1)) - 1;
    localparam signed [DATA_W:0] DAT_MIN = -(1 <<< (DATA_W-1));
    localparam signed [ACC_W-1:0] RND_FRAC = ACC_W'(1) <<< (COEF_FRAC-1);

    // =================================================================
    // Channel selector + reverse map.
    // =================================================================
    logic [CH_W-1:0]   sel_ch  [0:K-1];
    logic [LANE_W-1:0] ch_lane [0:N_CH-1];
    logic              ch_sel  [0:N_CH-1];
    integer ci;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            for (ci=0; ci<N_CH; ci++) ch_sel[ci] <= 1'b0;
        end else if (sel_wr_en) begin
            sel_ch[sel_wr_lane] <= sel_wr_ch;
            ch_lane[sel_wr_ch]  <= sel_wr_lane;
            ch_sel[sel_wr_ch]   <= 1'b1;
        end
    end

    // =================================================================
    // Coef RAMs (voice complex interleaved + halfband).
    // =================================================================
    logic signed [COEF_W-1:0] coef_ram [0:2*COEFN-1];
    logic signed [COEF_W-1:0] coef_rd;
    logic [COEF_AW-1:0]       coef_rd_addr;
    initial for (int ii=0; ii<2*COEFN; ii++) coef_ram[ii]='0;
    always_ff @(posedge clk) begin
        if (coef_wr_en) coef_ram[coef_wr_addr] <= coef_wr_data;
        coef_rd <= coef_ram[coef_rd_addr];
    end

    logic signed [COEF_W-1:0] hb_ram [0:HB_TAPS-1];
    logic signed [COEF_W-1:0] hb_rd;
    logic [HBTAP_W-1:0]       hb_rd_addr;
    initial for (int ii=0; ii<HB_TAPS; ii++) hb_ram[ii]='0;
    always_ff @(posedge clk) begin
        if (hb_wr_en) hb_ram[hb_wr_addr] <= hb_wr_data;
        hb_rd <= hb_ram[hb_rd_addr];
    end

    // =================================================================
    // Per-(lane,octave) decimated sample ring buffer (one unified BRAM).
    //   addr = ((lane*N_OCTAVES + octave)*RING_DEPTH) + ring_pos
    // head[lane*N_OCTAVES+octave] = ring pos of the NEWEST sample.
    // =================================================================
    localparam int RING_N     = K * N_OCTAVES * RING_DEPTH;
    localparam int RING_MEMAW = $clog2(RING_N);
    logic signed [DATA_W-1:0] ring [0:RING_N-1];
    initial for (int ii=0; ii<RING_N; ii++) ring[ii]='0;
    logic [RING_MEMAW-1:0]    ring_wr_addr, ring_rd_addr;
    logic                     ring_we;
    logic signed [DATA_W-1:0] ring_wr_data, ring_rd;
    always_ff @(posedge clk) begin
        if (ring_we) ring[ring_wr_addr] <= ring_wr_data;
        ring_rd <= ring[ring_rd_addr];
    end
    logic [RING_AW-1:0] head [0:K*N_OCTAVES-1];

    function automatic [RING_MEMAW-1:0] ridx
        (input int ln, input int oc, input [RING_AW-1:0] pos);
        ridx = (((ln*N_OCTAVES) + oc) * RING_DEPTH) + pos;
    endfunction

    // =================================================================
    // Frame staging + dyadic schedule.
    // =================================================================
    logic signed [DATA_W-1:0] stage [0:K-1];
    logic signed [DATA_W-1:0] snap  [0:K-1];
    logic [K-1:0]             stage_vld;
    logic [31:0]              fcount;
    logic                     start_pass;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            stage_vld <= '0; start_pass <= 1'b0;
        end else begin
            start_pass <= 1'b0;
            if (lfp_out_valid & lfp_frame_start) begin
                // the just-completed frame lives in stage[]; snapshot it so the
                // next frame's writes (this cycle's lane0 + the rest) can't clobber
                // the compute window. NB-assign reads OLD stage[] this cycle.
                if (wav_en && (|stage_vld)) begin
                    start_pass <= 1'b1;
                    for (int l=0;l<K;l++) snap[l] <= stage[l];
                end
                stage_vld <= '0;
            end
            if (lfp_out_valid && ch_sel[lfp_out_channel]) begin
                stage[ch_lane[lfp_out_channel]]     <= lfp_out_data;
                stage_vld[ch_lane[lfp_out_channel]] <= 1'b1;
            end
        end
    end

    logic [N_OCTAVES-1:0] oct_adv;
    always_comb begin
        for (int o=0; o<N_OCTAVES; o++)
            oct_adv[o] = ((fcount & ((32'd1<<o)-1)) == 0);
    end
    logic [N_OCTAVES-1:0] oct_adv_snap;

    // active config (guarded)
    wire [OCT_W:0]   n_oct = (n_octaves_cfg==0) ? (OCT_W+1)'(1) :
                             (n_octaves_cfg>N_OCTAVES) ? (OCT_W+1)'(N_OCTAVES) : n_octaves_cfg;
    wire [VOICE_W:0] n_voc = (n_voices_cfg==0) ? (VOICE_W+1)'(1) :
                             (n_voices_cfg>V) ? (VOICE_W+1)'(V) : n_voices_cfg;
    wire [TAP_W:0]   n_tap = (n_taps_cfg==0) ? (TAP_W+1)'(1) :
                             (n_taps_cfg>N_TAPS) ? (TAP_W+1)'(N_TAPS) : n_taps_cfg;

    // =================================================================
    // FSM state.
    // =================================================================
    typedef enum logic [3:0] {
        S_IDLE, S_COMMIT0, S_HB_NEXT, S_HB_RUN, S_HB_STORE,
        S_V_NEXT, S_V_RUN, S_V_EMIT, S_DRAIN
    } st_t;
    st_t st;

    logic [LANE_W-1:0]  cur_lane;
    logic [OCT_W:0]     cur_oct;     // voice octave being computed
    logic [VOICE_W:0]   cur_voice;
    logic [TAP_W:0]     cur_tap;
    logic               cur_im;      // 0 = re coeff phase, 1 = im coeff phase
    logic [LANE_W-1:0]  commit_lane;

    // -----------------------------------------------------------------
    // Per-pass config snapshot for the wire-packet header + compacted payload
    // addressing. n_oct/n_voc/n_tap are latched-while-disabled host config, so
    // they are stable across a pass; we snapshot them at start_pass anyway so
    // the header writer and the payload address generator use ONE consistent
    // view (and nscales is a multiply we want to do once). nscales = the active
    // scale count = n_oct*n_voc, the COMPACTED lane stride on the wire.
    // -----------------------------------------------------------------
    localparam int NSC_W = $clog2(N_OCTAVES*V + 1);
    logic [NSC_W-1:0] nscales_snap;  // active scales this pass (lane stride)
    logic [VOICE_W:0] nvoc_snap;     // voices/octave (scale = octave*nvoc + voice)
    logic [3:0]       noct_snap;     // raw n_octaves for the header w4 field
    logic [3:0]       nvoc4_snap;    // raw n_voices  for the header w4 field
    logic [7:0]       ntap_snap;     // raw n_taps    for the header w6 field
    logic [31:0]      hdr_seq;       // seq value stamped in this frame's header
    logic             ov_snap;       // overrun snapshot for this frame's header
    logic [31:0]      gain_snap;     // gain word for the header w7 field

    // header-write micro-sequence (runs at pass start, before any v_emit)
    logic [2:0]       hdr_idx;       // 0..7 while writing the 8-word header
    logic             hdr_busy;
    logic             hdr_kick;      // FSM->BRAM-writer: start the header now
    // w4 = n_octaves | (n_voices<<8) | (K<<16) | (overrun<<24)
    wire  [31:0]      hdr_w4 = {32'(noct_snap)} | ({32'(nvoc4_snap)} << 8)
                             | ({32'(K)} << 16) | ({31'd0, ov_snap} << 24);
    // w6 = nscales | (n_taps<<16)
    wire  [31:0]      hdr_w6 = {16'd0, {(16-NSC_W){1'b0}}, nscales_snap}
                             | ({32'(ntap_snap)} << 16);

    // halfband pass bookkeeping
    logic [LANE_W-1:0]  hb_lane;
    logic [OCT_W:0]     hb_oct;      // octave being PRODUCED (source = hb_oct-1)
    logic [HBTAP_W:0]   hb_tap;
    logic               hb_run_last; // last tap issued

    logic [2:0]         drain;

    // =================================================================
    // MAC address generation (single ring + single coef read bus).
    // The "mode" selects which read window the FSM drives.
    //   HB mode  : ring = octave (hb_oct-1) of hb_lane, newest..oldest;
    //              coef = hb_ram[hb_tap]
    //   VOICE mode: ring = octave cur_oct of cur_lane, newest..oldest;
    //              coef = coef_ram[2*(voice*N_TAPS+tap)+cur_im]
    // For VOICE the ring slot is read once (on the re phase) and reused for
    // the im phase, so the ring address only changes on the re phase.
    // =================================================================
    wire in_hb    = (st == S_HB_RUN);
    wire in_voice = (st == S_V_RUN);

    wire [RING_AW-1:0] hb_head  = head[(hb_lane*N_OCTAVES) + (hb_oct-1)];
    wire [RING_AW-1:0] v_head   = head[(cur_lane*N_OCTAVES) + cur_oct];

    always_comb begin
        // defaults
        ring_rd_addr = '0;
        hb_rd_addr   = '0;
        coef_rd_addr = '0;
        if (in_hb) begin
            ring_rd_addr = ridx(hb_lane, hb_oct-1,
                                (hb_head - RING_AW'(hb_tap)) & (RING_DEPTH-1));
            hb_rd_addr   = hb_tap[HBTAP_W-1:0];
        end else begin // voice
            ring_rd_addr = ridx(cur_lane, cur_oct,
                                (v_head - RING_AW'(cur_tap)) & (RING_DEPTH-1));
            coef_rd_addr = COEF_AW'(2*(cur_voice*N_TAPS + cur_tap) + cur_im);
        end
    end

    // =================================================================
    // MAC markers (s0 -> registered, valid alongside the 1-cyc RAM reads).
    // =================================================================
    logic        ag_v, ag_first, ag_last, ag_im, ag_is_hb;
    logic [3:0]  ag_gain;
    // (output routing for VOICE: which lane/scale to write)
    logic [LANE_W-1:0] ag_lane;
    logic [OCT_W:0]    ag_oct;
    logic [VOICE_W:0]  ag_voice;

    // s1: product + markers (the sample read is ring_rd; coef = coef_rd / hb_rd)
    logic signed [PROD_W-1:0] prod1;
    logic        v1, first1, last1, im1, is_hb1;
    logic [3:0]  gain1;
    logic [LANE_W-1:0] lane1;
    logic [OCT_W:0]    oct1;
    logic [VOICE_W:0]  voice1;

    // For VOICE we need the same ring sample on both re and im phases. The
    // ring read has 1-cycle latency aligned to ag_v; on the im phase the FSM
    // holds cur_tap (re address) so ring_rd is the SAME slot. So no special
    // hold is needed: ring_rd is correct for both phases because the address
    // is identical across the re/im pair.
    //
    // coef select MUST use the s1-stage marker ag_is_hb -- it is registered
    // ONCE (from the FSM), so it is valid alongside ring_rd/hb_rd/coef_rd in
    // this same cycle. (is_hb1 is the s2 copy, one cycle too late for the
    // product, used only by the accumulate stage below.)
    wire signed [COEF_W-1:0] coef_sel = ag_is_hb ? hb_rd : coef_rd;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            prod1<='0; v1<=0; first1<=0; last1<=0; im1<=0; is_hb1<=0;
            gain1<='0; lane1<='0; oct1<='0; voice1<='0;
        end else begin
            prod1  <= ring_rd * coef_sel;
            v1     <= ag_v; first1<=ag_first; last1<=ag_last; im1<=ag_im; is_hb1<=ag_is_hb;
            gain1  <= ag_gain; lane1<=ag_lane; oct1<=ag_oct; voice1<=ag_voice;
        end
    end

    // =================================================================
    // s2: accumulate. Voice has re + im accumulators; halfband uses acc_re.
    // first1 clears the relevant accumulator. On the last tap of a VOICE
    // (im phase) both re/im are complete -> emit a complex pair. On a HB
    // last tap, the result feeds back into the ring.
    // =================================================================
    logic signed [ACC_W-1:0] acc_re, acc_im, acc_hb;
    logic signed [ACC_W-1:0] vre_sum, vim_sum, hb_sum;

    // first1 marks the first beat of an accumulator: re-phase of tap0 clears
    // acc_re, im-phase of tap0 clears acc_im. (ag_first is asserted on BOTH.)
    always_comb begin
        vre_sum = (first1 && !im1) ? $signed(prod1) : (acc_re + prod1);
        vim_sum = (first1 &&  im1) ? $signed(prod1) : (acc_im + prod1);
        hb_sum  = (first1)         ? $signed(prod1) : (acc_hb + prod1);
    end

    // round/shift helpers
    function automatic signed [OUT_W-1:0] vshift_sat
        (input signed [ACC_W-1:0] a, input [3:0] g);
        int eff; logic signed [ACC_W-1:0] r;
        eff = COEF_FRAC - g;
        if (eff <= 0) r = a <<< (-eff);
        else          r = (a + (ACC_W'(1) <<< (eff-1))) >>> eff;
        if (r > OUT_MAX)      vshift_sat = OUT_MAX[OUT_W-1:0];
        else if (r < OUT_MIN) vshift_sat = OUT_MIN[OUT_W-1:0];
        else                  vshift_sat = r[OUT_W-1:0];
    endfunction

    function automatic signed [DATA_W-1:0] hbshift_sat (input signed [ACC_W-1:0] a);
        logic signed [ACC_W-1:0] r;
        r = (a + RND_FRAC) >>> COEF_FRAC;
        if (r > DAT_MAX)      hbshift_sat = DAT_MAX[DATA_W-1:0];
        else if (r < DAT_MIN) hbshift_sat = DAT_MIN[DATA_W-1:0];
        else                  hbshift_sat = r[DATA_W-1:0];
    endfunction

    // emit signals from the accumulate stage back to the FSM
    logic                     v_emit;          // a complex voice pair is ready
    logic signed [OUT_W-1:0]  v_emit_re, v_emit_im;
    logic [LANE_W-1:0]        v_emit_lane;
    logic [OCT_W:0]           v_emit_oct;
    logic [VOICE_W:0]         v_emit_voice;
    logic                     hb_emit;         // a halfband ÷2 sample is ready
    logic signed [DATA_W-1:0] hb_emit_data;

    // ---- emit pipeline (timing): on the last tap, register the RAW
    // accumulators + routing; apply the round/variable-shift/saturate in the
    // NEXT cycle. This breaks the DSP-add -> barrel-shift-round-saturate carry
    // chain that otherwise failed setup at 84 MHz on the v_emit_im path. The
    // FSM (S_V_EMIT / S_HB_STORE) already waits for the *_emit pulse, so the
    // extra cycle is free in the per-frame budget. ----
    logic                     v_raw;           // raw voice result captured, shift/sat pending
    logic signed [ACC_W-1:0]  v_raw_re, v_raw_im;
    logic [3:0]               v_raw_gain;
    logic [LANE_W-1:0]        v_raw_lane;
    logic [OCT_W:0]           v_raw_oct;
    logic [VOICE_W:0]         v_raw_voice;
    logic                     hb_raw;
    logic signed [ACC_W-1:0]  hb_raw_sum;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            acc_re<='0; acc_im<='0; acc_hb<='0;
            v_emit<=0; hb_emit<=0; v_emit_re<='0; v_emit_im<='0;
            v_emit_lane<='0; v_emit_oct<='0; v_emit_voice<='0; hb_emit_data<='0;
            v_raw<=0; v_raw_re<='0; v_raw_im<='0; v_raw_gain<='0;
            v_raw_lane<='0; v_raw_oct<='0; v_raw_voice<='0; hb_raw<=0; hb_raw_sum<='0;
        end else begin
            v_emit  <= 1'b0;
            hb_emit <= 1'b0;
            v_raw   <= 1'b0;
            hb_raw  <= 1'b0;

            // stage 2 of the emit pipeline: shift/saturate the captured raw
            if (v_raw) begin
                v_emit       <= 1'b1;
                v_emit_re    <= vshift_sat(v_raw_re, v_raw_gain);
                v_emit_im    <= vshift_sat(v_raw_im, v_raw_gain);
                v_emit_lane  <= v_raw_lane;
                v_emit_oct   <= v_raw_oct;
                v_emit_voice <= v_raw_voice;
            end
            if (hb_raw) begin
                hb_emit      <= 1'b1;
                hb_emit_data <= hbshift_sat(hb_raw_sum);
            end

            if (v1) begin
                if (is_hb1) begin
                    acc_hb <= hb_sum;
                    if (last1) begin   // capture the raw sum; saturate next cycle
                        hb_raw     <= 1'b1;
                        hb_raw_sum <= hb_sum;
                    end
                end else begin
                    // voice: re phase -> acc_re, im phase -> acc_im
                    if (!im1) acc_re <= vre_sum;
                    else      acc_im <= vim_sum;
                    if (last1) begin   // last tap, im phase -> complex done
                        // acc_re already holds the full re sum (the last re beat
                        // preceded this im beat); vim_sum is the just-finished im.
                        // Capture RAW here; shift/saturate is the next cycle.
                        v_raw        <= 1'b1;
                        v_raw_re     <= acc_re;
                        v_raw_im     <= vim_sum;
                        v_raw_gain   <= gain1;
                        v_raw_lane   <= lane1;
                        v_raw_oct    <= oct1;
                        v_raw_voice  <= voice1;
                    end
                end
            end
        end
    end

    // =================================================================
    // FSM.
    // =================================================================
    always_ff @(posedge clk) begin
        if (!rstn) begin
            st<=S_IDLE; busy<=0; overrun<=0; frame_seq<='0; fcount<='0;
            cur_lane<='0; cur_oct<='0; cur_voice<='0; cur_tap<='0; cur_im<=0;
            commit_lane<='0; hb_lane<='0; hb_oct<='0; hb_tap<='0; hb_run_last<=0;
            ring_we<=0; drain<='0; oct_adv_snap<='0;
            ag_v<=0; ag_first<=0; ag_last<=0; ag_im<=0; ag_is_hb<=0; ag_gain<='0;
            ag_lane<='0; ag_oct<='0; ag_voice<='0;
            nscales_snap<='0; nvoc_snap<='0; noct_snap<='0; nvoc4_snap<='0;
            ntap_snap<='0; hdr_seq<='0; ov_snap<=0; gain_snap<='0; hdr_kick<=0;
            for (int i=0;i<K*N_OCTAVES;i++) head[i]<='0;
        end else begin
            ring_we  <= 1'b0;
            ag_v     <= 1'b0;
            ag_first <= 1'b0; ag_last<=1'b0; ag_im<=1'b0; ag_is_hb<=1'b0;
            hdr_kick <= 1'b0;

            if (start_pass && st != S_IDLE) overrun <= 1'b1;

            case (st)
            // -----------------------------------------------------------
            S_IDLE: begin
                busy <= 1'b0;
                if (start_pass) begin
                    // snap[] was captured in the staging block at frame-start.
                    oct_adv_snap <= oct_adv;
                    commit_lane  <= '0;
                    busy <= 1'b1;
                    // Snapshot the wire-packet header fields + the compacted
                    // lane stride (nscales = n_oct*n_voc) for THIS pass, and
                    // kick the header writer. n_oct/n_voc/n_tap are held while
                    // disabled, so they are stable for the whole pass. seq is
                    // the value frame_seq becomes after this pass completes
                    // (S_DRAIN does frame_seq++), so the status reg the PS polls
                    // and this header carry the same number for the same column.
                    nscales_snap <= NSC_W'(n_oct * n_voc);
                    nvoc_snap    <= n_voc;
                    noct_snap    <= 4'(n_oct);   // zero-extend (n_oct may be <4 bits)
                    nvoc4_snap   <= 4'(n_voc);
                    ntap_snap    <= 8'(n_tap);
                    gain_snap    <= 32'(gain_cfg);   // zero-extend the host gain word
                    ov_snap      <= overrun;
                    hdr_seq      <= frame_seq + 1'b1;
                    hdr_kick     <= 1'b1;
                    st <= S_COMMIT0;
                end
            end

            // commit each lane's new octave-0 sample
            S_COMMIT0: begin
                head[(commit_lane*N_OCTAVES)+0] <=
                    (head[(commit_lane*N_OCTAVES)+0] + 1'b1) & (RING_DEPTH-1);
                ring_wr_addr <= ridx(commit_lane, 0,
                                  (head[(commit_lane*N_OCTAVES)+0] + 1'b1) & (RING_DEPTH-1));
                ring_wr_data <= snap[commit_lane];
                ring_we      <= 1'b1;
                if (commit_lane + 1 >= K) begin
                    hb_lane <= '0; hb_oct <= (OCT_W+1)'(1);
                    st <= S_HB_NEXT;
                end else commit_lane <= commit_lane + 1'b1;
            end

            // -----------------------------------------------------------
            // Halfband cascade: for each (octave 1..n_oct-1 that advances,
            // each lane) produce one ÷2 sample. Octave hb_oct's NEW sample
            // is HB over octave hb_oct-1's newest HB_TAPS samples.
            S_HB_NEXT: begin
                if (hb_oct >= n_oct) begin
                    cur_lane <= '0; cur_oct <= '0;
                    st <= S_V_NEXT;
                end else if (!oct_adv_snap[hb_oct[OCT_W-1:0]]) begin
                    hb_oct <= hb_oct + 1'b1; hb_lane <= '0;
                end else begin
                    hb_tap <= '0; hb_run_last <= 1'b0;
                    st <= S_HB_RUN;
                end
            end

            // issue HB_TAPS taps into the MAC
            S_HB_RUN: begin
                ag_v     <= 1'b1;
                ag_is_hb <= 1'b1;
                ag_first <= (hb_tap == 0);
                ag_last  <= (hb_tap == HB_TAPS-1);
                if (hb_tap == HB_TAPS-1) st <= S_HB_STORE;
                else                     hb_tap <= hb_tap + 1'b1;
            end

            // wait for hb_emit, store the ÷2 sample into octave hb_oct ring
            S_HB_STORE: begin
                if (hb_emit) begin
                    head[(hb_lane*N_OCTAVES)+hb_oct] <=
                        (head[(hb_lane*N_OCTAVES)+hb_oct] + 1'b1) & (RING_DEPTH-1);
                    ring_wr_addr <= ridx(hb_lane, hb_oct,
                                     (head[(hb_lane*N_OCTAVES)+hb_oct] + 1'b1) & (RING_DEPTH-1));
                    ring_wr_data <= hb_emit_data;
                    ring_we      <= 1'b1;
                    if (hb_lane + 1 >= K) begin
                        hb_lane <= '0; hb_oct <= hb_oct + 1'b1;
                    end else hb_lane <= hb_lane + 1'b1;
                    st <= S_HB_NEXT;
                end
            end

            // -----------------------------------------------------------
            // Voice MACs: for each (lane, advancing octave, voice) compute
            // the complex bin and write it to the results BRAM.
            S_V_NEXT: begin
                if (cur_oct >= n_oct) begin
                    cur_oct <= '0;
                    if (cur_lane + 1 >= K) begin st <= S_DRAIN; drain <= 3'd5; end
                    else cur_lane <= cur_lane + 1'b1;
                end else if (!oct_adv_snap[cur_oct[OCT_W-1:0]]) begin
                    cur_oct <= cur_oct + 1'b1;
                end else begin
                    cur_voice <= '0; cur_tap <= '0; cur_im <= 1'b0;
                    st <= S_V_RUN;
                end
            end

            // walk taps: re phase then im phase for each tap
            S_V_RUN: begin
                ag_v     <= 1'b1;
                ag_is_hb <= 1'b0;
                ag_first <= (cur_tap==0);  // tap0 re-beat clears acc_re, tap0 im-beat clears acc_im
                ag_im    <= cur_im;
                ag_last  <= (cur_tap==n_tap-1) && (cur_im==1'b1);
                ag_gain  <= gain_cfg[4*cur_oct +: 4];
                ag_lane  <= cur_lane;
                ag_oct   <= cur_oct;
                ag_voice <= cur_voice;
                if (cur_im == 1'b0) begin
                    cur_im <= 1'b1;            // im phase next, SAME tap (ring reused)
                end else begin
                    cur_im <= 1'b0;
                    if (cur_tap + 1 >= n_tap) st <= S_V_EMIT;
                    else cur_tap <= cur_tap + 1'b1;
                end
            end

            // wait for the complex pair to emit, then advance voice/octave
            S_V_EMIT: begin
                if (v_emit) begin
                    if (cur_voice + 1 >= n_voc) begin
                        cur_oct <= cur_oct + 1'b1; st <= S_V_NEXT;
                    end else begin
                        cur_voice <= cur_voice + 1'b1;
                        cur_tap <= '0; cur_im <= 1'b0;
                        st <= S_V_RUN;
                    end
                end
            end

            S_DRAIN: begin
                if (drain == 0) begin
                    frame_seq <= frame_seq + 1'b1;
                    fcount    <= fcount + 1'b1;
                    st <= S_IDLE; busy <= 1'b0;
                end else drain <= drain - 1'b1;
            end

            default: st <= S_IDLE;
            endcase
        end
    end

    // =================================================================
    // Results BRAM write: BUILD THE FULL WIRE PACKET (header + compacted
    // payload) in the results BRAM so the PS just DMAs+sends it (mirrors
    // lfp_dsp_block). One write port, shared by:
    //   (a) the 8-word HEADER micro-sequence (kicked at pass start by
    //       hdr_kick, one word/clk into addresses 0..7), and
    //   (b) the per-(lane,scale) PAYLOAD writes (on v_emit, re then im).
    // These never race: the header completes 8 clocks after start_pass, while
    // the first v_emit cannot fire until after the whole HB cascade + a voice
    // MAC run (many tens of clocks later) -- exactly the LFP precedent where
    // the 6-word header always lands before the first out_valid.
    //
    // COMPACTED payload addressing (lane stride = nscales, the active scale
    // count, NOT the padded build max N_OCTAVES*V):
    //   word_addr(lane,scale) = HDR_WORDS + (lane*nscales + scale)*2
    //   scale = octave*n_voices + voice ; +0 = re, +1 = im.
    // The address for a given (lane,octave,voice) is stable while the config is
    // held, so octaves that did not advance keep their previous column.
    // =================================================================
    logic              we_r, em_phase;
    logic [RES_WAW-1:0] addr_r;
    logic [31:0]       din_r;
    logic [31:0]       im_hold;
    logic [RES_WAW-1:0] base_word;

    // compacted base word for the in-flight complex bin (re slot)
    wire [RES_WAW-1:0] scale_word = RES_WAW'(HDR_WORDS +
        (((v_emit_lane*nscales_snap) + (v_emit_oct*nvoc_snap + v_emit_voice)) << 1));

    always_ff @(posedge clk) begin
        if (!rstn) begin
            we_r<=0; em_phase<=0; addr_r<='0; din_r<='0; im_hold<='0; base_word<='0;
            hdr_idx<=3'd0; hdr_busy<=1'b0;
        end else begin
            we_r <= 1'b0;

            if (hdr_kick) begin
                // start the header micro-sequence (takes priority; v_emit can't
                // arrive this early in the pass).
                hdr_busy <= 1'b1;
                hdr_idx  <= 3'd0;
            end

            if (hdr_busy) begin
                // emit one header word per clock into addresses 0..HDR_WORDS-1
                we_r   <= 1'b1;
                addr_r <= RES_WAW'(hdr_idx);
                case (hdr_idx)
                    3'd0: din_r <= WAV_MAGIC_LOW;
                    3'd1: din_r <= WAV_MAGIC_HIGH;
                    3'd2: din_r <= hdr_seq;
                    3'd3: din_r <= 32'd0;
                    3'd4: din_r <= hdr_w4;
                    3'd5: din_r <= hdr_seq;
                    3'd6: din_r <= hdr_w6;
                    default: din_r <= gain_snap;   // 3'd7
                endcase
                if (hdr_idx == 3'(HDR_WORDS-1)) hdr_busy <= 1'b0;
                else                            hdr_idx  <= hdr_idx + 1'b1;
            end else if (!em_phase) begin
                if (v_emit) begin
                    base_word <= scale_word;
                    addr_r    <= scale_word;
                    din_r     <= {{(32-OUT_W){v_emit_re[OUT_W-1]}}, v_emit_re};
                    im_hold   <= {{(32-OUT_W){v_emit_im[OUT_W-1]}}, v_emit_im};
                    we_r      <= 1'b1;
                    em_phase  <= 1'b1;
                end
            end else begin
                addr_r   <= base_word | 1'b1;
                din_r    <= im_hold;
                we_r     <= 1'b1;
                em_phase <= 1'b0;
            end
        end
    end

    assign res_bram_clk  = clk;
    assign res_bram_rst  = ~rstn;
    assign res_bram_en   = 1'b1;
    assign res_bram_we   = we_r ? 4'hF : 4'h0;
    assign res_bram_addr = {addr_r, 2'b00};
    assign res_bram_din  = din_r;
endmodule
