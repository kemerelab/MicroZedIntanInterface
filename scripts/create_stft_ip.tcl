# =====================================================================
# create_stft_ip.tcl -- generate the two Xilinx IP cores the STFT engine needs.
# Source this from create_vivado_project.tcl AFTER the project is created (so the
# IPs land in the project's IP repository and are built with it).
#
#   xfft_stft       : Fast Fourier Transform, single-precision FLOAT, runtime-N,
#                     pipelined streaming. Interface locked in stft_fft.v.
#   fix2float_stft  : Floating-Point, Fixed-to-float, signed 32-bit int -> float32,
#                     blocking flow control, tlast passthrough.
#
# Default max transform length = 64 (the N=64 spec default; runtime-selectable
# down to 8). To support N up to 256, set CONFIG.transform_length {256} below AND
# bump RES_AW to 16 (64 KB) on stft_dsp_block + the STFT results BRAM.
# =====================================================================

create_ip -name xfft -vendor xilinx.com -library ip -module_name xfft_stft
set_property -dict [list \
    CONFIG.data_format {floating_point} \
    CONFIG.transform_length {64} \
    CONFIG.run_time_configurable_transform_length {true} \
    CONFIG.implementation_options {pipelined_streaming_io} \
    CONFIG.target_clock_frequency {84} \
] [get_ips xfft_stft]

create_ip -name floating_point -vendor xilinx.com -library ip -module_name fix2float_stft
set_property -dict [list \
    CONFIG.Operation_Type {Fixed_to_float} \
    CONFIG.A_Precision_Type {Custom} \
    CONFIG.C_A_Exponent_Width {32} \
    CONFIG.C_A_Fraction_Width {0} \
    CONFIG.Result_Precision_Type {Single} \
    CONFIG.C_Result_Exponent_Width {8} \
    CONFIG.C_Result_Fraction_Width {24} \
    CONFIG.Flow_Control {Blocking} \
    CONFIG.Has_ARESETn {true} \
    CONFIG.Has_A_TLAST {true} \
] [get_ips fix2float_stft]

generate_target {all} [get_ips xfft_stft]
generate_target {all} [get_ips fix2float_stft]
puts "STFT IP generated: xfft_stft (float32, N<=64 runtime), fix2float_stft (int32->float32)"
