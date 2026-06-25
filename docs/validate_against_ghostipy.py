#!/usr/bin/env python3
# =====================================================================
# validate_against_ghostipy.py -- prove the hardware wavelet bank IS
# ghostipy's generalized Morse wavelet.
#
# wavelet_coeffs.py reproduces the exact Q1.17 taps the PL uses. This script
# checks those taps against ghostipy.spectral.MorseWavelet (gamma, beta) -- the
# canonical reference you analyze data with -- so out-of-band ghostipy analysis
# lines up with the hardware. It needs numpy + ghostipy:
#
#     pip install ghostipy            # pulls numpy + scipy
#     python3 docs/validate_against_ghostipy.py
#
# It checks, for every voice:
#   (1) our analytic frequency formula == ghostipy.MorseWavelet.freq_domain()
#       (should be ~0 -- confirms identical normalization + definition), and
#   (2) the finite Q1.17 FIR's frequency response matches ghostipy's wavelet
#       in shape: same peak frequency, small passband magnitude error.
#
# Reference (cite when using these wavelets):
#   Chu, J. P. et al. & Kemere, C. "ghostipy: an efficient signal processing
#   and spectral analysis toolbox for large data." eNeuro (2021).
#   https://github.com/kemerelab/ghostipy
# =====================================================================
import math
import os
import sys

try:
    import numpy as np
    import ghostipy as gsp
except ImportError as e:
    print("SKIP: needs numpy + ghostipy (pip install ghostipy):", e)
    sys.exit(2)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wavelet_coeffs as wc

# max |Hn - Psin| over the +/-0.6-octave passband. The deviation is finite-FIR
# truncation (the hardware genuinely uses N_TAPS taps), not a family mismatch:
# ~2e-4 at 24 taps, ~6e-5 at 40, ~1.6e-3 at 16. 3e-3 covers short FIRs; for a
# bit-exact filter use the wavelet_coeffs.py taps directly, not this ideal.
PASSBAND_TOL = 3e-3
PEAK_TOL     = 2e-3       # |measured peak freq - target fc| (cycles/sample)


def main():
    V, N_TAPS = wc.V, wc.N_TAPS
    fc_top, gamma, beta = wc.FC_TOP, wc.GAMMA, wc.BETA
    frac = wc.COEF_FRAC
    w = gsp.MorseWavelet(gamma=gamma, beta=beta)
    wp = (beta / gamma) ** (1.0 / gamma)
    log_a = math.log(2.0) + (beta / gamma) * (1.0 + math.log(gamma) - math.log(beta))
    omega = np.linspace(1e-6, math.pi, 4000)        # angular freq, rad/sample (0..Nyquist)
    n = np.arange(N_TAPS)

    voices = wc.design_voice_bank()
    print("ghostipy", gsp.__version__, "MorseWavelet(gamma=%g, beta=%g)" % (gamma, beta))
    print("voice   fc(target)  peak(ghost)  peak(FIR)   passband_maxerr   corr   formula_maxdiff")
    ok = True
    for v, voice in enumerate(voices):
        fc = fc_top * (2.0 ** (-v / float(V)))
        s = wp / (2.0 * math.pi * fc)
        psif = np.asarray(w.freq_domain(omega, np.array([s]))).reshape(-1)   # ghostipy Psi(w)
        # our analytic formula (what the FIR is sampled from) vs ghostipy:
        psif_mine = np.exp(log_a + beta * np.log(s * omega) - (s * omega) ** gamma)
        formula_maxdiff = float(np.max(np.abs(psif - psif_mine)))
        # finite Q1.17 FIR frequency response:
        taps = np.array([(re + 1j * im) / (1 << frac) for (re, im) in
                         [t['q'] for t in voice]])
        H = (taps[None, :] * np.exp(-1j * omega[:, None] * n[None, :])).sum(axis=1)
        gmag, hmag = np.abs(psif), np.abs(H)
        gmag_n, hmag_n = gmag / gmag.max(), hmag / hmag.max()
        f = omega / (2 * math.pi)
        pk_g, pk_h = f[np.argmax(gmag)], f[np.argmax(hmag)]
        band = (f > fc / 1.6) & (f < fc * 1.6)
        passband_err = float(np.max(np.abs(gmag_n[band] - hmag_n[band])))
        corr = float(np.corrcoef(gmag_n, hmag_n)[0, 1])
        good = (formula_maxdiff < 1e-9 and abs(pk_g - pk_h) < PEAK_TOL
                and passband_err < PASSBAND_TOL)
        ok = ok and good
        print("v%d      %.4f      %.4f       %.4f       %.4e        %.5f   %.2e  %s" %
              (v, fc, pk_g, pk_h, passband_err, corr, formula_maxdiff,
               "" if good else "  <-- FAIL"))
    print("\nRESULT:", "PASS -- hardware bank == ghostipy Morse" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
