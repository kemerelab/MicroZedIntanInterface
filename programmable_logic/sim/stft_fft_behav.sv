// Behavioral stand-in for the Xilinx xfft (float32), matching the engine's
// `stft_fft` interface. Collects N int samples (real, imag=0), computes the DFT
// in real arithmetic, and streams N complex bins as float32 ({imag32, real32}).
// For SIM ONLY -- the real build instantiates the xfft IP behind the same ports.
// Verifies the engine's windowing/addressing/sequencing/capture against a
// Python float reference (with tolerance).
module stft_fft #(
    parameter int MAX_N  = 256,
    parameter int SAMP_W = 32
) (
    input  logic        clk,
    input  logic        rstn,
    input  logic [23:0] cfg_tdata,
    input  logic        cfg_tvalid,
    output logic        cfg_tready,
    input  logic signed [SAMP_W-1:0] in_tdata,
    input  logic        in_tvalid,
    output logic        in_tready,
    input  logic        in_tlast,
    output logic [63:0] out_tdata,
    output logic        out_tvalid,
    input  logic        out_tready,
    output logic        out_tlast
);
    localparam real PI = 3.14159265358979;

    integer  Nfft;
    real     xin [0:MAX_N-1];
    integer  n_in;
    integer  k_out;
    integer  state;   // 0=collect, 1=emit

    assign cfg_tready = 1'b1;
    assign in_tready  = (state == 0);

    // latch N from config (log2 in low bits)
    always_ff @(posedge clk) begin
        if (!rstn) begin
            Nfft <= 64; n_in <= 0; k_out <= 0; state <= 0;
            out_tvalid <= 1'b0; out_tlast <= 1'b0;
        end else begin
            if (cfg_tvalid) Nfft <= (1 << (cfg_tdata[3:0]));

            if (state == 0) begin
                out_tvalid <= 1'b0; out_tlast <= 1'b0;
                if (in_tvalid) begin
                    xin[n_in] <= $itor(in_tdata);
                    if (in_tlast) begin n_in <= 0; k_out <= 0; state <= 1; end
                    else          n_in <= n_in + 1;
                end
            end else begin
                // emit one bin per accepted beat
                if (out_tready || !out_tvalid) begin
                    real re, im, ang;
                    re = 0.0; im = 0.0;
                    for (int n = 0; n < Nfft; n++) begin
                        ang = -2.0 * PI * k_out * n / Nfft;
                        re  = re + xin[n] * $cos(ang);
                        im  = im + xin[n] * $sin(ang);
                    end
                    out_tdata  <= {$shortrealtobits(im), $shortrealtobits(re)};
                    out_tvalid <= 1'b1;
                    out_tlast  <= (k_out == Nfft - 1);
                    if (k_out == Nfft - 1) begin k_out <= 0; state <= 0; end
                    else                   k_out <= k_out + 1;
                end
            end
        end
    end
endmodule
