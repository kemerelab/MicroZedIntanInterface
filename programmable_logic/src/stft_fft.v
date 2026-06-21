// =====================================================================
// stft_fft.v -- BUILD-TIME FFT sub-block for stft_engine.
//
// Presents the same `stft_fft` interface the engine instantiates, but backed by
// real Xilinx IP:
//   * fix2float_stft  : signed int32 windowed product -> float32 (real part)
//   * xfft_stft       : single-precision float, runtime-N, pipelined streaming
//
// The imaginary input is the float32 constant 0.0 (0x00000000). The FFT output
// is already complex float32 {imag[63:32], real[31:0]} -- passed straight out.
//
// SIM uses sim/stft_fft_behav.sv (a behavioral DFT) under the SAME module name;
// this file is the synthesis view (src/ is the build source set, sim/ is not).
// Generate the two IPs with scripts/create_stft_ip.tcl before building.
//
// NOTE (verify against generated IP): the 16-bit FFT config word is built as
//   {3'b0, NFFT[4:0]@[12:8], 7'b0, FWD@[0]} = (nfft_log2<<8)|1  (forward FFT).
// This byte-aligned FWD/NFFT layout is the standard FFT config; confirm in
// PG109 for the exact generated configuration if a frame mis-sizes.
// =====================================================================
`default_nettype none
module stft_fft #(
    parameter integer MAX_N  = 256,
    parameter integer SAMP_W = 32        // DATA_W + WIN_W (signed int windowed product)
) (
    input  wire        clk,
    input  wire        rstn,

    // config: low 4..5 bits = log2(N); engine drives {pad, nfft_log2}
    input  wire [23:0] cfg_tdata,
    input  wire        cfg_tvalid,
    output wire        cfg_tready,

    // windowed real samples in (signed int), AXIS
    input  wire signed [SAMP_W-1:0] in_tdata,
    input  wire        in_tvalid,
    output wire        in_tready,
    input  wire        in_tlast,

    // complex float32 bins out {imag32, real32}, AXIS
    output wire [63:0] out_tdata,
    output wire        out_tvalid,
    input  wire        out_tready,
    output wire        out_tlast
);
    wire aresetn = rstn;

    // ---- fixed (int32) -> float32, real part ----
    wire [31:0] re_f;
    wire        re_f_tvalid, re_f_tready, re_f_tlast;

    fix2float_stft u_f2f (
        .aclk                (clk),
        .aresetn             (aresetn),
        .s_axis_a_tvalid     (in_tvalid),
        .s_axis_a_tready     (in_tready),
        .s_axis_a_tdata      (in_tdata[31:0]),
        .s_axis_a_tlast      (in_tlast),
        .m_axis_result_tvalid(re_f_tvalid),
        .m_axis_result_tready(re_f_tready),
        .m_axis_result_tdata (re_f),
        .m_axis_result_tlast (re_f_tlast)
    );

    // ---- FFT config word (forward, runtime length) ----
    wire [15:0] fft_cfg = {3'b000, cfg_tdata[4:0], 7'b000_0000, 1'b1};

    // ---- xfft: complex float32 in {imag=0.0, real}, complex float32 out ----
    xfft_stft u_fft (
        .aclk                      (clk),
        .s_axis_config_tdata       (fft_cfg),
        .s_axis_config_tvalid      (cfg_tvalid),
        .s_axis_config_tready      (cfg_tready),
        .s_axis_data_tdata         ({32'h0000_0000, re_f}),  // {imag 0.0, real}
        .s_axis_data_tvalid        (re_f_tvalid),
        .s_axis_data_tready        (re_f_tready),
        .s_axis_data_tlast         (re_f_tlast),
        .m_axis_data_tdata         (out_tdata),
        .m_axis_data_tvalid        (out_tvalid),
        .m_axis_data_tready        (out_tready),
        .m_axis_data_tlast         (out_tlast),
        .event_frame_started       (),
        .event_tlast_unexpected    (),
        .event_tlast_missing       (),
        .event_status_channel_halt (),
        .event_data_in_channel_halt(),
        .event_data_out_channel_halt()
    );
endmodule
`default_nettype wire
