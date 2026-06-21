// =====================================================================
// stft_engine.sv  --  Tier-2 on-PL sliding-window STFT spectral estimator.
//
// Taps the Tier-1 LFP output stream (256 ch @ 2 kHz, signed), keeps a per-lane
// sliding sample buffer for K selected channels, and every H frames runs an
// N-point FFT per lane: Hann-window (fixed) -> fixed->float -> a shared float32
// FFT -> capture complex float32 bins into a results BRAM the PS streams.
//
// One FFT is time-shared across the K lanes (load = K*N*Fs/H; N=64/H=1/32ch @
// 2 kHz ~ 4 Msamp/s, a few % of the FFT clock). See docs/tier2-stft-design.md.
//
// The FFT is a swappable sub-block `stft_fft` presenting the Xilinx xfft AXIS
// (config/data in, data out) -- the real IP for the build, a behavioral float
// model for sim. This module owns the addressing/windowing/sequencing.
// =====================================================================

module stft_engine #(
    parameter int N_CH      = 256,    // LFP channels (lanes*slots = 8*32)
    parameter int K         = 32,     // selected channels processed
    parameter int MAX_N     = 256,    // max FFT length (power of 2)
    parameter int DATA_W    = 16,     // LFP sample width (signed)
    parameter int WIN_W     = 16,     // Hann coefficient width (signed Q15)
    parameter int RES_AW    = 14,     // results BRAM byte-addr width
    // derived widths (do not override)
    localparam int CH_W   = (N_CH <= 1) ? 1 : $clog2(N_CH),
    localparam int LANE_W = (K    <= 1) ? 1 : $clog2(K),
    localparam int NMAX_W = $clog2(MAX_N)
) (
    input  logic                 clk,
    input  logic                 rstn,

    // ---- Tier-1 LFP output stream tap (signed, pre-offset) ----
    input  logic                 lfp_out_valid,    // a decimated sample is present
    input  logic [CH_W-1:0]      lfp_out_channel,  // 0..N_CH-1
    input  logic signed [DATA_W-1:0] lfp_out_data,
    input  logic                 lfp_frame_start,  // pulse with the FIRST channel of each LFP frame

    // ---- configuration (host, latched while disabled) ----
    input  logic                 stft_en,
    input  logic [3:0]           nfft_log2,        // log2(N): 6 -> N=64
    input  logic [15:0]          hop,              // H (frames between passes), >=1

    // ---- channel-selector table write: sel_ch[lane]=channel, host-loaded ----
    input  logic                 sel_wr_en,
    input  logic [LANE_W-1:0]    sel_wr_lane,
    input  logic [CH_W-1:0]      sel_wr_ch,

    // ---- Hann window coefficient RAM write (host-loaded, MAX_N deep) ----
    input  logic                 win_wr_en,
    input  logic [NMAX_W-1:0]    win_wr_addr,
    input  logic signed [WIN_W-1:0] win_wr_data,

    // ---- results BRAM port A (PL writes complex float32 bins; PS reads) ----
    output logic                 res_bram_clk,
    output logic                 res_bram_rst,
    output logic [RES_AW-1:0]    res_bram_addr,
    output logic [31:0]          res_bram_din,
    input  logic [31:0]          res_bram_dout,
    output logic                 res_bram_en,
    output logic [3:0]           res_bram_we,

    // ---- status ----
    output logic [31:0]          frame_seq,        // completed STFT frames
    output logic                 busy,
    output logic                 overflow          // a pass started before the last finished
);

    localparam int BIN_W   = $clog2(MAX_N/2 + 1);
    localparam int RES_WAW = RES_AW - 2;            // 32-bit word addr (re/im interleaved)

    // active N from nfft_log2
    wire [NMAX_W:0] Nfft = (16'd1 << nfft_log2);
    wire [BIN_W-1:0] NBINS = Nfft[NMAX_W:1] + 1'b1; // N/2 + 1 (Hermitian half)

    // =================================================================
    // Channel selector: sel_ch[lane] -> source channel. A reverse map
    // ch_lane[channel] -> lane (+valid) lets the ingest path do an O(1)
    // lookup as samples stream by.
    // =================================================================
    logic [CH_W-1:0]  sel_ch   [0:K-1];
    logic [LANE_W-1:0] ch_lane [0:N_CH-1];
    logic             ch_sel   [0:N_CH-1];

    integer ci;
    always_ff @(posedge clk) begin
        if (!rstn) begin
            for (ci = 0; ci < N_CH; ci++) ch_sel[ci] <= 1'b0;
        end else if (sel_wr_en) begin
            sel_ch[sel_wr_lane]  <= sel_wr_ch;
            ch_lane[sel_wr_ch]   <= sel_wr_lane;
            ch_sel[sel_wr_ch]    <= 1'b1;
        end
    end

    // =================================================================
    // Hann window coefficient RAM (host-loaded, signed Q15).
    // =================================================================
    logic signed [WIN_W-1:0] win_ram [0:MAX_N-1];
    logic signed [WIN_W-1:0] win_rd;
    logic [NMAX_W-1:0]       win_rd_addr;
    initial for (int ii = 0; ii < MAX_N; ii++) win_ram[ii] = '0;
    always_ff @(posedge clk) begin
        if (win_wr_en) win_ram[win_wr_addr] <= win_wr_data;
        win_rd <= win_ram[win_rd_addr];
    end

    // =================================================================
    // Per-lane sliding sample buffer (K x MAX_N ring; shared head pointer
    // since every selected channel advances one sample per LFP frame).
    // =================================================================
    localparam int BUF_AW = LANE_W + NMAX_W;
    logic signed [DATA_W-1:0] sbuf [0:K*MAX_N-1];
    logic [NMAX_W-1:0]        wr_head;       // ring write position
    logic [BUF_AW-1:0]        sbuf_wr_addr, sbuf_rd_addr;
    logic                     sbuf_we;
    logic signed [DATA_W-1:0] sbuf_wr_data, sbuf_rd;

    always_ff @(posedge clk) begin
        if (sbuf_we) sbuf[sbuf_wr_addr] <= sbuf_wr_data;
        sbuf_rd <= sbuf[sbuf_rd_addr];
    end

    // ---- ingest: capture selected channels' samples into the ring ----
    // The frame-start pulse advances the head, and the frame's writes (this
    // cycle's channel + the rest) all land in the NEW slot -- so the head is
    // advanced combinationally for the write address on the start cycle.
    wire [NMAX_W-1:0] cur_head = (lfp_out_valid & lfp_frame_start)
                                 ? (wr_head + 1'b1) : wr_head;   // wraps mod MAX_N
    always_comb begin
        sbuf_we      = lfp_out_valid & ch_sel[lfp_out_channel];
        sbuf_wr_addr = ch_lane[lfp_out_channel] * MAX_N + cur_head;
        sbuf_wr_data = lfp_out_data;
    end

    // =================================================================
    // Hop / pass trigger: advance the ring head once per LFP frame; every
    // `hop` frames, snapshot the head and kick a compute pass.
    // =================================================================
    logic [15:0]        hop_cnt;
    logic [NMAX_W-1:0]  head_snap;
    logic               start_pass;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            wr_head <= '0; hop_cnt <= '0; start_pass <= 1'b0;
        end else begin
            start_pass <= 1'b0;
            // a frame boundary = the start pulse of the NEXT frame; the previous
            // frame (at wr_head, pre-advance) is now complete and snapshot-able.
            if (lfp_out_valid & lfp_frame_start) begin
                if (hop_cnt + 16'd1 >= (hop == 0 ? 16'd1 : hop)) begin
                    hop_cnt <= '0;
                    if (stft_en) begin head_snap <= wr_head; start_pass <= 1'b1; end
                end else begin
                    hop_cnt <= hop_cnt + 16'd1;
                end
                wr_head <= wr_head + 1'b1;       // advance to the new frame's slot (mod MAX_N)
            end
        end
    end

    // =================================================================
    // FFT sub-block interface (Xilinx xfft AXIS). The feed FSM streams N
    // windowed int samples/lane (real; imag=0 added in the wrap); the
    // capture FSM stores the returned complex float32 bins.
    // =================================================================
    logic [23:0] fft_cfg_tdata;  logic fft_cfg_tvalid; logic fft_cfg_tready;
    logic signed [DATA_W+WIN_W-1:0] fft_in_tdata; logic fft_in_tvalid, fft_in_tready, fft_in_tlast;
    logic [63:0] fft_out_tdata; logic fft_out_tvalid, fft_out_tready, fft_out_tlast;

    stft_fft #(.MAX_N(MAX_N), .SAMP_W(DATA_W+WIN_W)) u_fft (
        .clk(clk), .rstn(rstn),
        .cfg_tdata(fft_cfg_tdata), .cfg_tvalid(fft_cfg_tvalid), .cfg_tready(fft_cfg_tready),
        .in_tdata(fft_in_tdata), .in_tvalid(fft_in_tvalid), .in_tready(fft_in_tready), .in_tlast(fft_in_tlast),
        .out_tdata(fft_out_tdata), .out_tvalid(fft_out_tvalid), .out_tready(fft_out_tready), .out_tlast(fft_out_tlast)
    );

    // =================================================================
    // Feed FSM: per pass, for lane 0..K-1, send config then N windowed
    // samples (oldest->newest so the FFT sees a normal time series).
    // =================================================================
    typedef enum logic [1:0] {F_IDLE, F_CFG, F_FEED, F_NEXT} fstate_t;
    fstate_t           fstate;
    logic [LANE_W-1:0] f_lane;
    logic [NMAX_W:0]   f_n;            // sample index 0..N-1
    logic              feed_pending;   // a sample read is in flight (1-cyc BRAM latency)
    logic              issue_last;     // stage-1 last flag, parallels feed_pending

    assign fft_cfg_tdata  = {20'd0, nfft_log2};     // runtime-N config; forward FFT
    assign fft_cfg_tvalid = (fstate == F_CFG);
    // window-aligned read: sample j (oldest first) = ring[(head_snap - (N-1) + j)]
    // window-aligned read in the deep ring: sample j (oldest first) =
    // ring[(head_snap - (N-1) + j) mod MAX_N]. Masking mod MAX_N (not N) keeps
    // the snapshot window stable while later frames advance the head into fresh
    // slots, so streaming ingest never overwrites the window mid-pass.
    always_comb begin
        win_rd_addr  = f_n[NMAX_W-1:0];
        sbuf_rd_addr = f_lane * MAX_N + ((head_snap - (Nfft - 1'b1) + f_n) & (MAX_N - 1));
    end
    // windowed product registered alongside the 1-cycle buffer/window read
    logic signed [DATA_W+WIN_W-1:0] win_prod;
    logic                            prod_valid, prod_last;
    always_ff @(posedge clk) begin
        win_prod   <= sbuf_rd * win_rd;
        prod_valid <= feed_pending;
        prod_last  <= issue_last;     // aligned to win_prod (same 2-stage pipe as prod_valid)
    end
    assign fft_in_tdata  = win_prod;
    assign fft_in_tvalid = prod_valid;
    assign fft_in_tlast  = prod_last;

    always_ff @(posedge clk) begin
        if (!rstn) begin
            fstate <= F_IDLE; f_lane <= '0; f_n <= '0; feed_pending <= 1'b0;
            busy <= 1'b0; overflow <= 1'b0;
        end else begin
            feed_pending <= 1'b0;
            issue_last   <= 1'b0;
            case (fstate)
                F_IDLE: begin
                    busy <= 1'b0;
                    if (start_pass) begin f_lane <= '0; fstate <= F_CFG; busy <= 1'b1; end
                end
                F_CFG: begin
                    busy <= 1'b1;
                    if (fft_cfg_tready) begin f_n <= '0; fstate <= F_FEED; end
                end
                F_FEED: begin
                    busy <= 1'b1;
                    // issue one windowed-sample read per beat the FFT can accept.
                    // (the behavioral model holds tready high for a whole frame; harden
                    //  with a 2-deep skid buffer if the real xfft backpressures mid-frame)
                    if (fft_in_tready) begin
                        feed_pending <= 1'b1;
                        issue_last   <= (f_n + 1 >= Nfft);
                        if (f_n + 1 >= Nfft) fstate <= F_NEXT;
                        else                 f_n <= f_n + 1'b1;
                    end
                end
                F_NEXT: begin
                    busy <= 1'b1;
                    if (f_lane + 1 >= K) fstate <= F_IDLE;
                    else begin f_lane <= f_lane + 1'b1; fstate <= F_CFG; end
                end
                default: fstate <= F_IDLE;
            endcase
            if (start_pass && fstate != F_IDLE) overflow <= 1'b1;
        end
    end

    // =================================================================
    // Capture FSM: the FFT streams N complex bins/frame in order; keep the
    // Hermitian half (bins 0..N/2) and write {im,re} as two 32-bit words to
    // the results BRAM. Frames arrive lane-ordered, so a lane counter maps
    // them to per-lane regions.
    // =================================================================
    logic [LANE_W-1:0]  c_lane;
    logic [BIN_W:0]     c_bin;
    logic               cap_phase;      // 0 = accept bin / write re, 1 = write im
    logic [31:0]        cap_im_hold;
    logic               we_r;
    logic [31:0]        din_r;
    logic [RES_WAW-1:0] addr_r;

    // accept a new bin only in phase 0; phase 1 stalls the FFT to write the im word
    assign fft_out_tready = (cap_phase == 1'b0);
    // compact per-N layout: lane stride = NBINS complex words (not MAX_N) so K lanes
    // fit the results BRAM; the PS computes the same stride from N.
    wire [RES_WAW-1:0] bin_word = ((c_lane * NBINS) + c_bin) << 1;          // re-word addr

    always_ff @(posedge clk) begin
        if (!rstn) begin
            c_lane <= '0; c_bin <= '0; cap_phase <= 1'b0; we_r <= 1'b0; frame_seq <= '0;
        end else begin
            we_r <= 1'b0;
            if (cap_phase == 1'b0) begin
                if (fft_out_tvalid) begin
                    if (c_bin < NBINS) begin                 // keep the Hermitian half
                        addr_r      <= bin_word;             // real word
                        din_r       <= fft_out_tdata[31:0];
                        cap_im_hold <= fft_out_tdata[63:32];
                        we_r        <= 1'b1;
                        cap_phase   <= 1'b1;                 // write im next cycle
                    end else begin                           // drop the upper half
                        if (fft_out_tlast) begin
                            c_bin <= '0;
                            if (c_lane + 1 >= K) begin c_lane <= '0; frame_seq <= frame_seq + 1; end
                            else                       c_lane <= c_lane + 1'b1;
                        end else c_bin <= c_bin + 1'b1;
                    end
                end
            end else begin                                   // phase 1: write the im word
                addr_r    <= bin_word | 1'b1;                 // im word = re-word + 1
                din_r     <= cap_im_hold;
                we_r      <= 1'b1;
                cap_phase <= 1'b0;
                c_bin     <= c_bin + 1'b1;                    // advance to the next bin
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
