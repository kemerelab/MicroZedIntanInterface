// =====================================================================
// wavelet_halfband.sv
//
// A time-shared decimate-by-2 anti-alias FIR for the Tier-3 wavelet
// octave cascade (a trous / dyadic). ONE pipelined MAC serves every
// (lane x octave) decimation stage, so the whole halfband cascade costs
// ~1 DSP48 + a per-stage delay-line and a small shared coef RAM.
//
// Each *stage* turns a stream at rate R into a stream at R/2. The wavelet
// engine drives one stage per (lane, octave-boundary): octave o's input
// stream is produced by HB applied to octave o-1's stream. This module is
// a generic ÷2 FIR primitive; the engine owns the cascade scheduling and
// supplies, per request, the lane/stage delay-line address window.
//
// To keep it self-contained and bit-exact with the Python reference, this
// module is purely combinational-arithmetic over a host-supplied delay
// window: the caller presents N_TAPS samples (newest..oldest), the module
// MACs them against the shared halfband coefficients and emits one output.
// The caller is responsible for only requesting an output every 2 input
// samples (the ÷2) and for maintaining the per-(lane,stage) ring buffers.
//
// Arithmetic (matches gen_wavelet_vectors.py halfband_decimate):
//   acc = sum_j hb[j] * x[newest-j];  out = sat16( round>>FRAC( acc ) )
//   round-to-nearest = + (1<<(FRAC-1)) before the arithmetic shift.
//
// MAC pipeline (3-cycle latency), same shape as lfp_fir_decimator.
// =====================================================================
module wavelet_halfband #(
    parameter int DATA_W    = 16,   // sample width (signed)
    parameter int COEF_W    = 18,   // coefficient width (signed Q1.COEF_FRAC)
    parameter int COEF_FRAC = 17,
    parameter int ACC_W     = 48,
    parameter int HB_TAPS   = 7,    // halfband length (odd, symmetric)
    parameter int OUT_W     = 16,
    // derived
    localparam int TAP_W = (HB_TAPS <= 1) ? 1 : $clog2(HB_TAPS)
) (
    input  logic clk,
    input  logic rstn,

    // ---- coefficient write port (already CDC'd) ----
    input  logic                 coef_wr_en,
    input  logic [TAP_W-1:0]     coef_wr_addr,
    input  logic [COEF_W-1:0]    coef_wr_data,

    // ---- request interface: caller drives the delay window one tap/clk ----
    input  logic                 tap_valid,   // a (sample,coef) pair is presented
    input  logic                 tap_first,   // first tap of this output (clears acc)
    input  logic                 tap_last,    // last tap of this output (emits)
    input  logic signed [DATA_W-1:0] tap_sample,
    input  logic [TAP_W-1:0]     tap_index,   // coef index for this tap

    // ---- output ----
    output logic                 out_valid,
    output logic signed [OUT_W-1:0] out_data
);
    localparam int PROD_W = DATA_W + COEF_W;
    localparam signed [OUT_W:0] OUT_MAX =  (1 <<< (OUT_W-1)) - 1;
    localparam signed [OUT_W:0] OUT_MIN = -(1 <<< (OUT_W-1));
    localparam signed [ACC_W-1:0] RND = ACC_W'(1) <<< (COEF_FRAC-1);

    // ---- coefficient RAM ----
    logic signed [COEF_W-1:0] coef_ram [0:HB_TAPS-1];
    logic signed [COEF_W-1:0] coef_rdata;
    initial for (int ii = 0; ii < HB_TAPS; ii++) coef_ram[ii] = '0;
    always_ff @(posedge clk) begin
        if (coef_wr_en) coef_ram[coef_wr_addr] <= coef_wr_data;
        coef_rdata <= coef_ram[tap_index];
    end

    // ---- s0 -> s1: read coef (1 cyc), register sample + markers alongside ----
    logic signed [DATA_W-1:0] samp_d;
    logic v_d, first_d, last_d;
    always_ff @(posedge clk) begin
        if (!rstn) begin v_d<=0; first_d<=0; last_d<=0; samp_d<='0; end
        else begin
            v_d <= tap_valid; first_d <= tap_first; last_d <= tap_last;
            samp_d <= tap_sample;
        end
    end

    // ---- s1 -> s2: product ----
    logic signed [PROD_W-1:0] prod;
    logic v1, first1, last1;
    always_ff @(posedge clk) begin
        if (!rstn) begin prod<='0; v1<=0; first1<=0; last1<=0; end
        else begin
            prod   <= samp_d * coef_rdata;   // both signed
            v1     <= v_d; first1 <= first_d; last1 <= last_d;
        end
    end

    // ---- s2: accumulate, round, saturate, emit ----
    logic signed [ACC_W-1:0] acc, acc_sum, rounded;
    always_comb begin
        acc_sum = first1 ? $signed(prod) : (acc + prod);
        rounded = (acc_sum + RND) >>> COEF_FRAC;
    end
    always_ff @(posedge clk) begin
        if (!rstn) begin acc<='0; out_valid<=0; out_data<='0; end
        else begin
            if (v1) acc <= acc_sum;
            out_valid <= v1 & last1;
            // latch the rounded/saturated result ONLY on the emit beat, so
            // out_data holds a stable value until the next output (later beats
            // recompute `rounded` from a stale acc_sum and must not leak out).
            if (v1 & last1) begin
                if (rounded > OUT_MAX)      out_data <= OUT_MAX[OUT_W-1:0];
                else if (rounded < OUT_MIN) out_data <= OUT_MIN[OUT_W-1:0];
                else                        out_data <= rounded[OUT_W-1:0];
            end
        end
    end
endmodule
