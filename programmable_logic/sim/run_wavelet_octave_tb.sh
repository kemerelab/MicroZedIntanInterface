#!/usr/bin/env bash
# Compile + run the STEP 2 unified per-octave wavelet packetizer TB under xsim.
# Generates the stimulus/expected vectors with the Python reference first.
# Usage: source /opt/Xilinx/2025.1/Vivado/settings64.sh && bash run_wavelet_octave_tb.sh
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HERE/../src"
WORK="$(mktemp -d)"

python3 "$HERE/gen_wavelet_vectors.py" || exit 1
cp "$HERE"/wav_coef.hex "$HERE"/wav_hb.hex "$HERE"/wav_samp.hex \
   "$HERE"/wav_oct0_exp.hex "$HERE"/wav_oct1_exp.hex "$HERE"/wav_oct2_exp.hex "$WORK/" || exit 1

cd "$WORK" || exit 99
xvlog -sv "$SRC/wavelet_halfband.sv" "$SRC/wavelet_cqt_engine.sv" \
         "$HERE/wavelet_octave_tb.sv" || exit 1
xelab -debug off -timescale 1ns/1ps work.wavelet_octave_tb -s tb_snap || exit 1
xsim tb_snap -R | tee sim.log

grep -q "RESULT: PASS" sim.log && { echo "TB_PASS"; exit 0; } || { echo "TB_FAIL"; exit 1; }
