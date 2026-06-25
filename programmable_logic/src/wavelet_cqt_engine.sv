// =====================================================================
// wavelet_cqt_engine.sv  --  Tier-3 multirate wavelet (constant-Q / Morse)
// scalogram engine. A time-shared MAC computes both the octave-cascade
// halfband ÷2 decimations and the V complex bandpass voices per octave,
// producing a real-time scalogram of K selected channels over N_OCTAVES
// octaves x V complex voices.
//
// v2 STEP 1 -- TWO MAC LANES. The voice MAC uses two real multipliers: lane A
// multiplies the ring sample by the RE coefficient and lane B by the IM
// coefficient of the SAME tap in the SAME cycle (both lanes share one ring read;
// the voice coef RAM is replicated two ways with independent read addresses, the
// lfp_fir_decimator N_MAC precedent). A voice tap therefore costs ONE MAC cycle
// instead of the old re/im 2-phase pair -- this halves the worst-case
// (all-octaves-coincide) pass from ~1758*K to ~990*K clocks. The scalogram
// VALUES are bit-identical to the single-MAC engine (only the issue order
// changed; re and im now accumulate in lockstep instead of alternating). The
// halfband ÷2 still uses lane A only (lane B idle during HB). Two MAC lanes
// alone reach K=16 real-time-clean.
//
// v2 STEP 2 -- LAZY WORK-SPREAD (the big win). The old engine ran the WHOLE
// voice MAC for every advancing octave inside the single coincidence-frame pass
// (peak = all octaves at fcount=0). STEP 2 keeps octave 0 (and the cheap HB
// cascade) EAGER each frame but DEFERS each slower octave's voice column into a
// persistent, deadline-monotonic drain that spreads it across the octave's
// 2^o-frame window. The peak per-frame compute collapses from ~990*K (peak)
// toward ~2x octave-0's cost (~198*K average), so the real-time-clean ceiling
// rises to K=96 (busy duty 80%) / K=112 (94%, tight); K=128 overruns (99%).
// Each deferred voice column reads the ring at a head-pointer snapshot pinned at
// the octave's enqueue deadline (head_snap[o]) -- the octave's ring is written
// only at its own deadlines and RING_DEPTH (64) >> N_TAPS (24), so the
// newest-N_TAPS window stays intact for the whole deferred drain (the
// snapshot-across-frames rule, satisfied by a pointer snapshot, no BRAM copy).
// The arithmetic is the identical 2-MAC datapath, so the VALUES stay bit-exact;
// only WHEN each column is emitted changes. See the FSM block below for detail.
// Reaching K=256 needs a 4-MAC (2-voices-per-cycle) datapath -- not built here.
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
// MAC-bus discipline: the engine owns one ring read port (shared by both MAC
// lanes), TWO voice-coef read ports (re on lane A, im on lane B), one halfband-
// coef read port, and a 2-lane MAC pipeline. The FSM sequences (a) HALFBAND
// passes -- HB_TAPS real MACs (lane A) producing one ÷2 sample stored back into
// the next octave's ring, and (b) VOICE passes -- n_taps cycles, each producing
// the re AND im product of one tap (lanes A+B), accumulating a complex bin
// written to the results BRAM. A standalone wavelet_halfband.sv exists as the
// reusable ÷2 primitive (+ its own unit TB); the engine integrates the same
// arithmetic for clean single-bus scheduling.
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
    //
    // v2 STEP 1 -- 2 MAC lanes. The voice coef RAM is REPLICATED two ways
    // (identical contents, independent read addresses) so MAC lane A can read
    // the RE coefficient and MAC lane B the IM coefficient of the SAME tap in
    // the SAME cycle. This is the lfp_fir_decimator N_MAC precedent (replicate
    // is cheaper than a true dual-port BRAM and keeps each read single-cycle).
    // Both copies see every write, so they stay identical. Lane A's copy
    // (coef_rd_a) also serves the halfband path is NOT needed -- HB has its own
    // hb_ram -- so during HB only lane A's ring/coef are exercised.
    // =================================================================
    logic signed [COEF_W-1:0] coef_ram_a [0:2*COEFN-1];  // lane A (RE coef)
    logic signed [COEF_W-1:0] coef_ram_b [0:2*COEFN-1];  // lane B (IM coef)
    logic signed [COEF_W-1:0] coef_rd_a, coef_rd_b;
    logic [COEF_AW-1:0]       coef_rd_addr_a, coef_rd_addr_b;
    initial for (int ii=0; ii<2*COEFN; ii++) begin coef_ram_a[ii]='0; coef_ram_b[ii]='0; end
    always_ff @(posedge clk) begin
        if (coef_wr_en) begin
            coef_ram_a[coef_wr_addr] <= coef_wr_data;
            coef_ram_b[coef_wr_addr] <= coef_wr_data;
        end
        coef_rd_a <= coef_ram_a[coef_rd_addr_a];
        coef_rd_b <= coef_ram_b[coef_rd_addr_b];
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
    //
    // v2 STEP 2 -- LAZY WORK-SPREAD. The old engine ran the WHOLE voice MAC for
    // every advancing octave inside the single coincidence-frame pass (peak =
    // all 8 octaves at fcount=0). STEP 2 splits the per-frame work into:
    //   (1) an EAGER phase per frame: commit octave-0's new sample, run the HB
    //       cascade for every advancing octave (cheap), and ENQUEUE each
    //       advancing octave's voice column (set pend[o], snapshot its ring head
    //       head_snap[o]); and
    //   (2) a persistent VOICE DRAIN that, in deadline-monotonic order (octave 0
    //       first -- it is due every frame -- then 1,2,...), computes ONE pending
    //       octave's full K*V voice column and clears its pend bit, then re-picks.
    // The drain CONTINUES ACROSS FRAME BOUNDARIES: a new frame's eager phase
    // preempts it (at a voice-column boundary), then the drain resumes. So slow
    // octaves' columns are spread over their 2^o-frame windows and the PEAK
    // per-frame busy collapses toward the AVERAGE (~2x octave-0's cost) instead
    // of the all-octaves-coincide sum.
    //
    // BIT-EXACT: each voice column reads the ring at head_snap[o] -- the head
    // pinned when octave o was enqueued. octave o's ring is written ONLY at its
    // own deadlines (every 2^o frames) and RING_DEPTH (64) >> N_TAPS (24), so the
    // newest-N_TAPS window stays intact for the whole deferred drain (the
    // snapshot-across-frames rule -- a head-pointer snapshot suffices, no BRAM
    // copy). The arithmetic is the identical 2-MAC datapath; only WHEN each
    // column is emitted changes, not its value. When real-time-clean every
    // octave drains before its next deadline, so head_snap is always its own
    // deadline's head -> identical to the eager engine.
    // =================================================================
    typedef enum logic [3:0] {
        S_IDLE, S_COMMIT0, S_HB_NEXT, S_HB_RUN, S_HB_STORE,
        S_V_PICK, S_V_RUN, S_V_EMIT
    } st_t;
    st_t st;

    logic [LANE_W-1:0]  cur_lane;
    logic [OCT_W:0]     cur_oct;     // voice octave being computed
    logic [VOICE_W:0]   cur_voice;
    logic [TAP_W:0]     cur_tap;
    logic [LANE_W-1:0]  commit_lane;

    // ---- work-spread bookkeeping ----
    // pend[o] : octave o has a voice column enqueued (computed from head_snap[o])
    //           that has not yet been fully emitted.
    // head_snap[o] : ring head pinned for octave o's pending column. All lanes
    //           share one octave head (they advance together), so one snap/octave.
    // frame_req : a new frame's eager phase is owed (start_pass latched). Serviced
    //           at the next voice-column boundary (or immediately if idle).
    // oct0_was_pending_at_frame : tracks whether octave 0 missed its deadline.
    logic [N_OCTAVES-1:0] pend;
    logic [RING_AW-1:0]   head_snap [0:N_OCTAVES-1];
    logic                 frame_req;
    logic [N_OCTAVES-1:0] frame_adv;   // oct_adv latched for the owed eager phase
    logic [31:0]          seq_inc;     // pending frame_seq increments (one per frame)

    // lowest set bit of pend = the most-urgent pending octave (deadline-monotonic)
    logic [OCT_W:0] pick_oct;
    logic           pick_vld;
    always_comb begin
        pick_vld = 1'b0;
        pick_oct = '0;
        for (int o=N_OCTAVES-1; o>=0; o--) begin
            if (pend[o]) begin pick_oct = (OCT_W+1)'(o); pick_vld = 1'b1; end
        end
    end

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
    // MAC address generation (single shared ring read + TWO coef read buses).
    // The "mode" selects which read window the FSM drives.
    //   HB mode  : ring = octave (hb_oct-1) of hb_lane, newest..oldest;
    //              coef = hb_ram[hb_tap]   (lane A only; lane B idle)
    //   VOICE mode: ring = octave cur_oct of cur_lane, newest..oldest (ONE
    //               read, shared by both MAC lanes -- they consume the same
    //               sample); coef A = coef_ram_a[2*(voice*N_TAPS+tap)+0] (RE),
    //               coef B = coef_ram_b[2*(voice*N_TAPS+tap)+1] (IM). Both
    //               coef reads + the shared ring read happen in ONE cycle, so a
    //               tap now costs ONE MAC cycle instead of the old re/im 2-phase
    //               pair. This is the v2 STEP-1 2x.
    // =================================================================
    wire in_hb    = (st == S_HB_RUN);
    wire in_voice = (st == S_V_RUN);

    // TIMING: at K=96 the head[] array is K*N_OCTAVES (768) entries, so the
    // combinational read mux head[(hb_lane*N_OCTAVES)+hb_oct] in the ring
    // address path is a 768:1 mux feeding the BRAM address -- the worst-case
    // critical path (hb_oct -> head mux -> +1 -> ridx -> ring ADDRBWRADDR,
    // ~10 logic levels, missed setup). Fix: latch the source- and dest-octave
    // heads into small REGISTERS (hb_src_head/hb_dst_head) at S_HB_NEXT entry
    // (hb_lane/hb_oct are stable through the HB_TAPS-cycle run), so the per-cycle
    // ring read addr and the store write addr come from registers, not the big
    // mux. The voice path reads head_snap (only N_OCTAVES=8 entries -> tiny mux).
    logic [RING_AW-1:0] hb_src_head;   // head of source octave (hb_oct-1), latched
    logic [RING_AW-1:0] hb_dst_head;   // head of dest octave (hb_oct), latched
    // octave 0's ring head is UNIFORM across lanes (all lanes commit one sample
    // per frame together), so track it in ONE register instead of reading the
    // 768-entry head[] array per lane in S_COMMIT0 (same critical-path fix).
    logic [RING_AW-1:0] oct0_head;     // octave-0 newest ring position (all lanes)
    logic [RING_AW-1:0] commit_pos;    // this frame's octave-0 write position
    // VOICE reads the PINNED head (head_snap), not the live head -- the deferred
    // column must see octave cur_oct's ring as of its enqueue deadline.
    wire [RING_AW-1:0] v_head   = head_snap[cur_oct];

    always_comb begin
        // defaults
        ring_rd_addr   = '0;
        hb_rd_addr     = '0;
        coef_rd_addr_a = '0;
        coef_rd_addr_b = '0;
        if (in_hb) begin
            ring_rd_addr   = ridx(hb_lane, hb_oct-1,
                                  (hb_src_head - RING_AW'(hb_tap)) & (RING_DEPTH-1));
            hb_rd_addr     = hb_tap[HBTAP_W-1:0];
        end else begin // voice -- both lanes read the SAME ring slot
            ring_rd_addr   = ridx(cur_lane, cur_oct,
                                  (v_head - RING_AW'(cur_tap)) & (RING_DEPTH-1));
            coef_rd_addr_a = COEF_AW'(2*(cur_voice*N_TAPS + cur_tap) + 0); // RE
            coef_rd_addr_b = COEF_AW'(2*(cur_voice*N_TAPS + cur_tap) + 1); // IM
        end
    end

    // =================================================================
    // MAC markers (s0 -> registered, valid alongside the 1-cyc RAM reads).
    // v2 STEP 1: no more re/im phase -- each VOICE tap produces re AND im
    // products in one cycle (lane A = re, lane B = im). ag_is_hb selects the
    // halfband single-MAC path (lane A only).
    // =================================================================
    logic        ag_v, ag_first, ag_last, ag_is_hb;
    logic [3:0]  ag_gain;
    // (output routing for VOICE: which lane/scale to write)
    logic [LANE_W-1:0] ag_lane;
    logic [OCT_W:0]    ag_oct;
    logic [VOICE_W:0]  ag_voice;

    // s1: products + markers. Lane A product (prod1_a) uses coef_rd_a (RE) for
    // VOICE, or hb_rd for the halfband; lane B product (prod1_b) uses coef_rd_b
    // (IM) and is only meaningful for VOICE. Both lanes share ring_rd.
    logic signed [PROD_W-1:0] prod1_a, prod1_b;
    logic        v1, first1, last1, is_hb1;
    logic [3:0]  gain1;
    logic [LANE_W-1:0] lane1;
    logic [OCT_W:0]    oct1;
    logic [VOICE_W:0]  voice1;

    // coef select for lane A MUST use the s1-stage marker ag_is_hb -- it is
    // registered ONCE (from the FSM), so it is valid alongside the RAM reads in
    // this same cycle. (is_hb1 is the s2 copy, one cycle too late for the
    // product, used only by the accumulate stage below.)
    wire signed [COEF_W-1:0] coef_sel_a = ag_is_hb ? hb_rd : coef_rd_a;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            prod1_a<='0; prod1_b<='0; v1<=0; first1<=0; last1<=0; is_hb1<=0;
            gain1<='0; lane1<='0; oct1<='0; voice1<='0;
        end else begin
            prod1_a <= ring_rd * coef_sel_a;   // RE (or HB) product
            prod1_b <= ring_rd * coef_rd_b;    // IM product (VOICE only)
            v1      <= ag_v; first1<=ag_first; last1<=ag_last; is_hb1<=ag_is_hb;
            gain1   <= ag_gain; lane1<=ag_lane; oct1<=ag_oct; voice1<=ag_voice;
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

    // first1 (tap0) clears both voice accumulators -- with 2 MAC lanes the re
    // and im products of a tap arrive TOGETHER, so both acc_re and acc_im
    // advance every voice cycle (no more re/im phase). The halfband uses
    // acc_hb fed by lane A (prod1_a).
    always_comb begin
        vre_sum = (first1) ? $signed(prod1_a) : (acc_re + prod1_a);
        vim_sum = (first1) ? $signed(prod1_b) : (acc_im + prod1_b);
        hb_sum  = (first1) ? $signed(prod1_a) : (acc_hb + prod1_a);
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
                    // voice: both re and im advance every cycle (2 MAC lanes).
                    acc_re <= vre_sum;
                    acc_im <= vim_sum;
                    if (last1) begin   // last tap -> complex pair done THIS cycle
                        // vre_sum / vim_sum are the just-finished full sums.
                        // Capture RAW here; shift/saturate is the next cycle.
                        v_raw        <= 1'b1;
                        v_raw_re     <= vre_sum;
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
    // helper: do the per-frame EAGER phase setup (header + config snapshot + the
    // oct_adv latch). Called when a frame_req is serviced.
    // (inlined below; kept as a comment marker for readability)

    always_ff @(posedge clk) begin
        if (!rstn) begin
            st<=S_IDLE; busy<=0; overrun<=0; frame_seq<='0; fcount<='0;
            cur_lane<='0; cur_oct<='0; cur_voice<='0; cur_tap<='0;
            commit_lane<='0; hb_lane<='0; hb_oct<='0; hb_tap<='0; hb_run_last<=0;
            ring_we<=0; drain<='0; oct_adv_snap<='0;
            ag_v<=0; ag_first<=0; ag_last<=0; ag_is_hb<=0; ag_gain<='0;
            ag_lane<='0; ag_oct<='0; ag_voice<='0;
            nscales_snap<='0; nvoc_snap<='0; noct_snap<='0; nvoc4_snap<='0;
            ntap_snap<='0; hdr_seq<='0; ov_snap<=0; gain_snap<='0; hdr_kick<=0;
            pend<='0; frame_req<=0; frame_adv<='0; seq_inc<='0;
            hb_src_head<='0; hb_dst_head<='0; oct0_head<='0; commit_pos<='0;
            for (int i=0;i<K*N_OCTAVES;i++) head[i]<='0;
            for (int i=0;i<N_OCTAVES;i++) head_snap[i]<='0;
        end else begin
            ring_we  <= 1'b0;
            ag_v     <= 1'b0;
            ag_first <= 1'b0; ag_last<=1'b0; ag_is_hb<=1'b0;
            hdr_kick <= 1'b0;

            // ---- frame arrival: latch a frame request. The eager phase is owed
            // and will be serviced at the next voice-column boundary (S_V_PICK)
            // or immediately if idle. OVERRUN = octave 0 still pending from the
            // previous frame when a new frame arrives (oct0 missed its deadline),
            // OR an eager phase is still owed (frame_req already set = the engine
            // never got to service the previous frame). Latch (sticky).
            if (start_pass) begin
                if (frame_req || pend[0]) overrun <= 1'b1;
                frame_req <= 1'b1;
                frame_adv <= oct_adv;        // capture this frame's advancing set
                fcount    <= fcount + 1'b1;  // one fcount tick per frame
            end

            case (st)
            // -----------------------------------------------------------
            // IDLE: nothing pending. Service an owed frame immediately.
            S_IDLE: begin
                busy <= 1'b0;
                if (frame_req || start_pass) begin
                    busy <= 1'b1;
                    // begin the eager phase (header + config snapshot below)
                    st <= S_COMMIT0;
                    commit_lane  <= '0;
                    commit_pos   <= (oct0_head + 1'b1) & (RING_DEPTH-1);  // this frame's oct0 pos
                    oct0_head    <= (oct0_head + 1'b1) & (RING_DEPTH-1);
                    oct_adv_snap <= start_pass ? oct_adv : frame_adv;
                    nscales_snap <= NSC_W'(n_oct * n_voc);
                    nvoc_snap    <= n_voc;
                    noct_snap    <= 4'(n_oct);
                    nvoc4_snap   <= 4'(n_voc);
                    ntap_snap    <= 8'(n_tap);
                    gain_snap    <= 32'(gain_cfg);
                    ov_snap      <= overrun;
                    hdr_seq      <= frame_seq + 1'b1;
                    hdr_kick     <= 1'b1;
                    frame_seq    <= frame_seq + 1'b1;   // one column per frame
                    frame_req    <= 1'b0;               // consuming the request
                end
            end

            // commit each lane's new octave-0 sample, then enqueue octave 0's
            // voice column (pend[0]=1, head_snap[0] = oct0's new head). All lanes
            // write the SAME ring position (commit_pos, computed once at entry
            // from the single oct0_head register) -- so the write address carries
            // only the lane shift in ridx, NOT the 768:1 head[] read mux. The
            // per-lane head[] entry is still updated so the HB octave-1 read
            // (hb_src_head latch) sees octave 0's current head.
            S_COMMIT0: begin
                head[(commit_lane*N_OCTAVES)+0] <= commit_pos;
                ring_wr_addr <= ridx(commit_lane, 0, commit_pos);
                ring_wr_data <= snap[commit_lane];
                ring_we      <= 1'b1;
                if (commit_lane + 1 >= K) begin
                    head_snap[0] <= commit_pos;   // octave 0's new newest position
                    pend[0]      <= 1'b1;
                    hb_lane <= '0; hb_oct <= (OCT_W+1)'(1);
                    st <= S_HB_NEXT;
                end else commit_lane <= commit_lane + 1'b1;
            end

            // -----------------------------------------------------------
            // Halfband cascade: for each (advancing octave 1..n_oct-1, each lane)
            // produce one ÷2 sample. After an octave's HB finishes for all lanes,
            // pin its head + enqueue its voice column (pend[o]=1).
            S_HB_NEXT: begin
                if (hb_oct >= n_oct) begin
                    st <= S_V_PICK;       // eager phase done -> (re)enter the drain
                end else if (!oct_adv_snap[hb_oct[OCT_W-1:0]]) begin
                    hb_oct <= hb_oct + 1'b1; hb_lane <= '0;
                end else begin
                    // latch the source/dest octave heads for (hb_lane, hb_oct)
                    // into registers so the HB ring read/write addresses don't
                    // carry the 768:1 head[] mux on their critical path.
                    hb_src_head <= head[(hb_lane*N_OCTAVES) + (hb_oct-1)];
                    hb_dst_head <= head[(hb_lane*N_OCTAVES) + hb_oct];
                    hb_tap <= '0; hb_run_last <= 1'b0;
                    st <= S_HB_RUN;
                end
            end

            // issue HB_TAPS taps into the MAC (lane A)
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
                    // hb_dst_head was latched at S_HB_NEXT entry for this
                    // (hb_lane, hb_oct); the new sample goes one slot past it.
                    head[(hb_lane*N_OCTAVES)+hb_oct] <=
                        (hb_dst_head + 1'b1) & (RING_DEPTH-1);
                    ring_wr_addr <= ridx(hb_lane, hb_oct,
                                     (hb_dst_head + 1'b1) & (RING_DEPTH-1));
                    ring_wr_data <= hb_emit_data;
                    ring_we      <= 1'b1;
                    if (hb_lane + 1 >= K) begin
                        // octave hb_oct advanced -> pin its head (= the new dst
                        // head) + enqueue its voice column.
                        head_snap[hb_oct] <= (hb_dst_head + 1'b1) & (RING_DEPTH-1);
                        pend[hb_oct]      <= 1'b1;
                        hb_lane <= '0; hb_oct <= hb_oct + 1'b1;
                    end else hb_lane <= hb_lane + 1'b1;
                    st <= S_HB_NEXT;
                end
            end

            // -----------------------------------------------------------
            // VOICE DRAIN (persistent, deadline-monotonic). Pick the lowest
            // pending octave; if a frame's eager phase is owed, service it FIRST
            // (preempt at this column boundary). If nothing pending and no frame
            // owed -> go idle.
            S_V_PICK: begin
                if (frame_req) begin
                    // a new frame arrived; do its eager phase before more voices
                    busy <= 1'b1;
                    st <= S_COMMIT0;
                    commit_lane  <= '0;
                    commit_pos   <= (oct0_head + 1'b1) & (RING_DEPTH-1);
                    oct0_head    <= (oct0_head + 1'b1) & (RING_DEPTH-1);
                    oct_adv_snap <= frame_adv;
                    nscales_snap <= NSC_W'(n_oct * n_voc);
                    nvoc_snap    <= n_voc;
                    noct_snap    <= 4'(n_oct);
                    nvoc4_snap   <= 4'(n_voc);
                    ntap_snap    <= 8'(n_tap);
                    gain_snap    <= 32'(gain_cfg);
                    ov_snap      <= overrun;
                    hdr_seq      <= frame_seq + 1'b1;
                    hdr_kick     <= 1'b1;
                    frame_seq    <= frame_seq + 1'b1;
                    frame_req    <= 1'b0;
                end else if (pick_vld) begin
                    busy      <= 1'b1;
                    cur_oct   <= pick_oct;
                    cur_lane  <= '0;
                    cur_voice <= '0;
                    cur_tap   <= '0;
                    st <= S_V_RUN;
                end else begin
                    busy <= 1'b0;
                    st   <= S_IDLE;
                end
            end

            // walk taps: ONE cycle per tap (re on lane A, im on lane B together)
            S_V_RUN: begin
                ag_v     <= 1'b1;
                ag_is_hb <= 1'b0;
                ag_first <= (cur_tap==0);  // tap0 clears both acc_re and acc_im
                ag_last  <= (cur_tap==n_tap-1);
                ag_gain  <= gain_cfg[4*cur_oct +: 4];
                ag_lane  <= cur_lane;
                ag_oct   <= cur_oct;
                ag_voice <= cur_voice;
                if (cur_tap + 1 >= n_tap) st <= S_V_EMIT;
                else cur_tap <= cur_tap + 1'b1;
            end

            // wait for the complex pair to emit, then advance voice/lane within
            // the current octave's column. When the column (all lanes, all
            // voices) is done, clear pend[cur_oct] and re-pick.
            S_V_EMIT: begin
                if (v_emit) begin
                    if (cur_voice + 1 >= n_voc) begin
                        cur_voice <= '0; cur_tap <= '0;
                        if (cur_lane + 1 >= K) begin
                            // octave column complete
                            pend[cur_oct] <= 1'b0;
                            st <= S_V_PICK;
                        end else begin
                            cur_lane <= cur_lane + 1'b1;
                            st <= S_V_RUN;
                        end
                    end else begin
                        cur_voice <= cur_voice + 1'b1;
                        cur_tap <= '0;
                        st <= S_V_RUN;
                    end
                end
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
