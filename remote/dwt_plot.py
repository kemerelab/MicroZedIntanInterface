#!/usr/bin/env python3
"""
dwt_plot.py -- standalone scrolling DWT (wavelet) scalogram plotter.

Binds UDP 5004, decodes the wavelet scalogram monitor stream emitted by the
MicroZed Tier-3 engine (see remote/net.py::receive_wavelet + design_wavelet_bank
for the authoritative decode), accumulates one COLUMN per packet, and renders a
scrolling time-frequency heat-map.

This is the headless-testable twin of the Open Ephys "DwtCanvas" visualizer:
it validates the exact packet decode + the scale->frequency mapping the OE
viewer reuses, and gives you something working without the GUI.

Packet format (8-word little-endian header, then payload):
    w0 = 0x5CA70900  (WAV_MAGIC_LOW, "SCALOG")
    w1 = 0xCAFEBABE  (WAV_MAGIC_HIGH)
    w2 = seq (== w5)            w3 = 0 (timestamp high)
    w4 = n_octaves | (n_voices<<8) | (K<<16) | (overrun<<24)
    w5 = seq                    w6 = nscales | (n_taps<<16)
    w7 = gain word
  payload: for lane in 0..K-1: for s in 0..nscales-1: (re int32, im int32)
           ONE packet == one scalogram column (all K lanes x nscales bins at
           one time step).  magnitude = sqrt(re^2 + im^2).
           nscales = n_octaves * n_voices ; scale s = octave*n_voices + voice.

Scale index -> center frequency: design_wavelet_bank(fs=3000) returns the
per-octave/voice center frequencies; octave 0 is the highest band (fs=3000),
each octave halves it, and within an octave higher voice -> lower frequency.
So bin 0 is the highest frequency and the last bin is the lowest.

Rendering:
  Y axis = scale / center frequency (log, labeled from the center-freq table)
  X axis = time (scrolling, newest on the right)
  color  = magnitude, normalized PER-SCALE (per row) so the 1/f roll-off
           doesn't wash out the high-frequency rows.  A global log-magnitude
           mode is also available (--norm global).

Backends, in order of preference (so it runs anywhere):
  1. matplotlib  -- full axes + colorbar (live window if $DISPLAY, else PNG)
  2. PIL/Pillow  -- writes a PNG with a simple text-free heat-map
  3. pure-python zlib PNG writer -- last-resort, always works

Usage:
    python3 dwt_plot.py                       # bind 5004, live/auto, save PNG on exit
    python3 dwt_plot.py --lane 3 --secs 30
    python3 dwt_plot.py --selftest            # craft synthetic packets + render, no socket
    python3 dwt_plot.py --png out.png --frames 600
"""

import argparse
import math
import socket
import struct
import sys
import time

# ---------------------------------------------------------------------------
# Constants -- MUST match remote/net.py (WAV_* block) and the firmware.
# ---------------------------------------------------------------------------
WAV_UDP_PORT   = 5004
WAV_MAGIC_LOW  = 0x5CA70900
WAV_MAGIC_HIGH = 0xCAFEBABE
WAV_HDR_WORDS  = 8
WAV_HDR_BYTES  = WAV_HDR_WORDS * 4

# default build params (overridden at runtime by each packet's header)
WAV_K         = 32
WAV_N_OCTAVES = 8
WAV_V         = 4
WAV_N_TAPS    = 24
WAV_HB_TAPS   = 7
WAV_COEF_FRAC = 17
WAV_FS        = 3000.0


# ---------------------------------------------------------------------------
# Center-frequency table -- a self-contained copy of net.py's
# design_wavelet_bank() center-frequency math (the part the viewer needs).
# Returns a FLAT list of length n_octaves*n_voices, indexed by scale s where
# s = octave*n_voices + voice, matching the packet's bin order. Bin 0 = highest.
# ---------------------------------------------------------------------------
def scale_center_freqs(n_octaves=WAV_N_OCTAVES, n_voices=WAV_V,
                       fs=WAV_FS, fc_top=0.34):
    centers = []
    for o in range(n_octaves):
        fr = fs / (2.0 ** o)                       # this octave's effective rate
        for v in range(n_voices):
            centers.append(fc_top * (2.0 ** (-v / float(n_voices))) * fr)
    return centers


# ---------------------------------------------------------------------------
# Packet decode -- mirrors net.py::receive_wavelet exactly.
# Returns dict {seq, n_oct, n_voc, K, overrun, nscales, n_taps, gain, mag}
# where mag is a list of K lists of nscales magnitudes, or None if not a
# well-formed wavelet datagram.
# ---------------------------------------------------------------------------
def decode_wavelet_packet(data):
    if len(data) < WAV_HDR_BYTES:
        return None
    (mlo, mhi, ts_lo, ts_hi, w4, seq, w6, gain) = struct.unpack('<IIIIIIII',
                                                                 data[:WAV_HDR_BYTES])
    if mlo != WAV_MAGIC_LOW or mhi != WAV_MAGIC_HIGH:
        return None
    n_oct   = w4 & 0xFF
    n_voc   = (w4 >> 8) & 0xFF
    K       = (w4 >> 16) & 0xFF
    overrun = (w4 >> 24) & 1
    nscales = w6 & 0xFFFF
    n_taps  = (w6 >> 16) & 0xFFFF
    if K == 0 or nscales == 0:
        return None
    pay = data[WAV_HDR_BYTES:]
    nints = len(pay) // 4
    expect = K * nscales * 2
    if nints < expect:
        return None
    vals = struct.unpack('<%di' % expect, pay[:expect * 4])  # signed re,im int32
    mag = []
    for lane in range(K):
        base = lane * nscales * 2
        row = [math.hypot(vals[base + 2 * s], vals[base + 2 * s + 1])
               for s in range(nscales)]
        mag.append(row)
    return dict(seq=seq, ts=(ts_hi << 32) | ts_lo, n_oct=n_oct, n_voc=n_voc,
                K=K, overrun=overrun, nscales=nscales, n_taps=n_taps,
                gain=gain, mag=mag)


# ---------------------------------------------------------------------------
# Viridis-ish colormap (6 anchors) -- identical anchors to the OE DwtCanvas so
# the two viewers look the same. Input t in [0,1] -> (r,g,b) 0..255.
# ---------------------------------------------------------------------------
_VIRIDIS = [
    (0.00,  68,   1,  84),
    (0.20,  72,  37, 118),
    (0.40,  62,  73, 137),
    (0.60,  31, 158, 137),
    (0.80, 109, 205,  89),
    (1.00, 253, 231,  37),
]


def viridis(t):
    if t <= 0.0:
        return _VIRIDIS[0][1:]
    if t >= 1.0:
        return _VIRIDIS[-1][1:]
    for i in range(1, len(_VIRIDIS)):
        if t <= _VIRIDIS[i][0]:
            t0, r0, g0, b0 = _VIRIDIS[i - 1]
            t1, r1, g1, b1 = _VIRIDIS[i]
            a = (t - t0) / (t1 - t0)
            return (int(round(r0 * (1 - a) + r1 * a)),
                    int(round(g0 * (1 - a) + g1 * a)),
                    int(round(b0 * (1 - a) + b1 * a)))
    return (0, 0, 0)


# ---------------------------------------------------------------------------
# Scalogram accumulator: a fixed-width ring of columns for ONE lane, with
# per-row (per-scale) normalization tracking so the 1/f roll-off doesn't bury
# the high frequencies.
# ---------------------------------------------------------------------------
class Scalogram:
    def __init__(self, width=1024, lane=0, norm='per-scale', floor_db=-40.0):
        self.width = width
        self.lane = lane
        self.norm = norm
        self.floor_db = floor_db
        self.nscales = None
        self.n_oct = None
        self.n_voc = None
        self.cols = []          # list of per-scale magnitude lists (oldest..newest)
        self.row_max = None     # EMA of per-row max magnitude (per-scale norm)
        self.glob_max = 1.0     # EMA of global max (global norm)
        self.columns_seen = 0
        self.last_seq = None
        self.seq_gaps = 0

    def add(self, pkt):
        if pkt is None:
            return
        if self.lane >= pkt['K']:
            return
        row = pkt['mag'][self.lane]
        ns = pkt['nscales']
        if self.nscales is None:
            self.nscales = ns
            self.n_oct = pkt['n_oct']
            self.n_voc = pkt['n_voc']
            self.row_max = [1.0] * ns
        if ns != self.nscales:           # config changed mid-stream -- reset
            self.nscales = ns
            self.n_oct = pkt['n_oct']
            self.n_voc = pkt['n_voc']
            self.row_max = [1.0] * ns
            self.cols = []
        # seq gap accounting
        if self.last_seq is not None:
            expected = (self.last_seq + 1) & 0x3FFFFFFF
            if pkt['seq'] != expected:
                self.seq_gaps += 1
        self.last_seq = pkt['seq']

        # track normalization
        alpha = 0.02
        gmax = 0.0
        for s in range(ns):
            m = row[s]
            if m > gmax:
                gmax = m
            self.row_max[s] = max(m, (1 - alpha) * self.row_max[s] + alpha * m)
        self.glob_max = max(gmax, (1 - alpha) * self.glob_max + alpha * gmax)

        self.cols.append(row)
        if len(self.cols) > self.width:
            self.cols.pop(0)
        self.columns_seen += 1

    def _t(self, mag, s):
        """Map a magnitude at scale s to [0,1] color coordinate."""
        if self.norm == 'per-scale':
            ref = self.row_max[s] if self.row_max[s] > 1e-9 else 1.0
            ratio = mag / ref
        else:  # global
            ref = self.glob_max if self.glob_max > 1e-9 else 1.0
            ratio = mag / ref
        # log compression: floor_db..0 dB -> 0..1
        if ratio <= 0.0:
            return 0.0
        db = 20.0 * math.log10(ratio)
        t = (db - self.floor_db) / (0.0 - self.floor_db)
        return max(0.0, min(1.0, t))

    def to_rgb_grid(self):
        """Return (height=nscales, width) grid of (r,g,b). Row 0 = highest
        frequency (bin 0) at the TOP."""
        ns = self.nscales or 1
        ncol = len(self.cols)
        grid = [[(0, 0, 0)] * self.width for _ in range(ns)]
        x0 = self.width - ncol           # newest at right
        for cx, col in enumerate(self.cols):
            x = x0 + cx
            for s in range(ns):
                grid[s][x] = viridis(self._t(col[s], s))
        return grid


# ---------------------------------------------------------------------------
# Rendering backends
# ---------------------------------------------------------------------------
def render_matplotlib(scal, centers, png_path, live=False, title=""):
    import matplotlib
    if not live:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    ns = scal.nscales
    ncol = len(scal.cols)
    if ns is None or ncol == 0:
        print("[dwt_plot] no columns to render")
        return False

    # Build a normalized magnitude image: rows = scale (bin 0 at top), cols=time.
    img = np.zeros((ns, scal.width), dtype=float)
    x0 = scal.width - ncol
    for cx, col in enumerate(scal.cols):
        for s in range(ns):
            img[s, x0 + cx] = scal._t(col[s], s)

    fig, ax = plt.subplots(figsize=(11, 5))
    im = ax.imshow(img, aspect='auto', origin='upper', cmap='viridis',
                   vmin=0.0, vmax=1.0, interpolation='nearest')
    ax.set_xlabel("time (columns, newest at right)")
    ax.set_ylabel("center frequency (Hz)")
    # frequency ticks from the center table (bin 0 = top = highest freq)
    nt = min(ns, 12)
    yt = [int(round(i * (ns - 1) / (nt - 1))) for i in range(nt)] if nt > 1 else [0]
    ax.set_yticks(yt)
    ax.set_yticklabels(["%.1f" % centers[i] for i in yt])
    cb = fig.colorbar(im, ax=ax)
    cb.set_label("normalized magnitude (%s, %.0f dB floor)" %
                 (scal.norm, scal.floor_db))
    ax.set_title(title or "DWT scalogram (lane %d, %d scales)" % (scal.lane, ns))
    fig.tight_layout()
    if live:
        plt.show()
    if png_path:
        fig.savefig(png_path, dpi=110)
        print("[dwt_plot] wrote %s (matplotlib)" % png_path)
    plt.close(fig)
    return True


def render_pil(scal, centers, png_path):
    from PIL import Image, ImageDraw
    ns = scal.nscales
    if ns is None or len(scal.cols) == 0:
        print("[dwt_plot] no columns to render")
        return False
    grid = scal.to_rgb_grid()
    # heat-map area scaled up; add margins for axes labels
    cell_h = max(1, 360 // ns)
    heat_h = ns * cell_h
    heat_w = scal.width
    left, top, right, bottom = 70, 20, 20, 30
    W, H = left + heat_w + right, top + heat_h + bottom
    im = Image.new("RGB", (W, H), (20, 20, 24))
    px = im.load()
    for s in range(ns):
        for x in range(heat_w):
            r, g, b = grid[s][x]
            for dy in range(cell_h):
                px[left + x, top + s * cell_h + dy] = (r, g, b)
    d = ImageDraw.Draw(im)
    nt = min(ns, 12)
    for i in range(nt):
        s = int(round(i * (ns - 1) / (nt - 1))) if nt > 1 else 0
        yy = top + s * cell_h
        d.text((4, yy), "%6.1f Hz" % centers[s], fill=(220, 220, 220))
    d.text((left, H - 16), "time -> (newest at right)  lane=%d scales=%d norm=%s"
           % (scal.lane, ns, scal.norm), fill=(200, 200, 200))
    im.save(png_path)
    print("[dwt_plot] wrote %s (PIL)" % png_path)
    return True


def render_zlib_png(scal, centers, png_path):
    """Pure-stdlib PNG writer (no PIL/matplotlib). RGB, no text axes."""
    import zlib
    ns = scal.nscales
    if ns is None or len(scal.cols) == 0:
        print("[dwt_plot] no columns to render")
        return False
    grid = scal.to_rgb_grid()
    cell_h = max(1, 300 // ns)
    H = ns * cell_h
    W = scal.width
    raw = bytearray()
    for s in range(ns):
        for _dy in range(cell_h):
            raw.append(0)                # PNG filter type 0 per scanline
            for x in range(W):
                r, g, b = grid[s][x]
                raw += bytes((r, g, b))

    def chunk(typ, body):
        c = struct.pack(">I", len(body)) + typ + body
        c += struct.pack(">I", zlib.crc32(typ + body) & 0xFFFFFFFF)
        return c

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)   # 8-bit RGB
    idat = zlib.compress(bytes(raw), 6)
    with open(png_path, "wb") as f:
        f.write(sig)
        f.write(chunk(b"IHDR", ihdr))
        f.write(chunk(b"IDAT", idat))
        f.write(chunk(b"IEND", b""))
    print("[dwt_plot] wrote %s (zlib PNG, %dx%d)" % (png_path, W, H))
    return True


def render(scal, centers, png_path, live=False, title=""):
    """Try matplotlib, then PIL, then a stdlib PNG writer."""
    try:
        return render_matplotlib(scal, centers, png_path, live=live, title=title)
    except ImportError:
        pass
    except Exception as e:
        print("[dwt_plot] matplotlib failed (%s); falling back" % e)
    try:
        return render_pil(scal, centers, png_path)
    except ImportError:
        pass
    except Exception as e:
        print("[dwt_plot] PIL failed (%s); falling back" % e)
    return render_zlib_png(scal, centers, png_path)


# ---------------------------------------------------------------------------
# Synthetic packet builder (for --selftest and dwt_test_sender.py). Emits a
# well-formed 5004 packet for ONE column. `freqs_active` is a set of scale
# indices to light up for this column (a diagonal sweep, ridge, etc).
# ---------------------------------------------------------------------------
def build_packet(seq, K=4, n_oct=WAV_N_OCTAVES, n_voc=WAV_V, n_taps=WAV_N_TAPS,
                 gain=0, active=None, amp=1 << 20):
    nscales = n_oct * n_voc
    w0 = WAV_MAGIC_LOW
    w1 = WAV_MAGIC_HIGH
    w2 = seq & 0xFFFFFFFF
    w3 = 0
    w4 = (n_oct & 0xFF) | ((n_voc & 0xFF) << 8) | ((K & 0xFF) << 16)
    w5 = seq & 0xFFFFFFFF
    w6 = (nscales & 0xFFFF) | ((n_taps & 0xFFFF) << 16)
    w7 = gain & 0xFFFFFFFF
    hdr = struct.pack('<IIIIIIII', w0, w1, w2, w3, w4, w5, w6, w7)
    active = active or {}
    body = bytearray()
    for lane in range(K):
        for s in range(nscales):
            a = active.get((lane, s), 0.0)
            # complex value with a per-scale 1/f-ish baseline + the active ridge
            base = amp / (1.0 + s) * 0.05
            mag = base + a * amp
            phase = 0.3 * s + 0.1 * seq
            re = int(mag * math.cos(phase))
            im = int(mag * math.sin(phase))
            body += struct.pack('<ii', re, im)
    return hdr + bytes(body)


def synth_columns(n_frames, K=4, n_oct=WAV_N_OCTAVES, n_voc=WAV_V):
    """Generate a sequence of synthetic packets with a known pattern:
       - lane 0: a diagonal frequency sweep (ridge moves from low scale to high)
       - all lanes: a steady ridge at one mid scale
    so a correct decode/render shows a clear moving diagonal + a horizontal band."""
    nscales = n_oct * n_voc
    for i in range(n_frames):
        active = {}
        sweep_s = int((i / float(max(1, n_frames - 1))) * (nscales - 1))
        for lane in range(K):
            active[(lane, sweep_s)] = 1.0                     # moving diagonal
            active[(lane, nscales // 2)] = 0.6                # steady band
        yield build_packet(i, K=K, n_oct=n_oct, n_voc=n_voc, active=active)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Scrolling DWT scalogram plotter (UDP 5004)")
    ap.add_argument("--port", type=int, default=WAV_UDP_PORT)
    ap.add_argument("--lane", type=int, default=0, help="which of K lanes to plot")
    ap.add_argument("--width", type=int, default=1024, help="columns of history")
    ap.add_argument("--secs", type=float, default=0.0,
                    help="capture seconds then render (0 = until --frames or Ctrl-C)")
    ap.add_argument("--frames", type=int, default=0,
                    help="stop after N columns (0 = unlimited)")
    ap.add_argument("--norm", choices=["per-scale", "global"], default="per-scale")
    ap.add_argument("--floor-db", type=float, default=-40.0)
    ap.add_argument("--png", default="dwt_scalogram.png", help="output PNG path")
    ap.add_argument("--live", action="store_true",
                    help="show a live matplotlib window (needs DISPLAY)")
    ap.add_argument("--selftest", action="store_true",
                    help="decode/render synthetic packets, no socket -- proves the pipeline")
    ap.add_argument("--fs", type=float, default=WAV_FS)
    ap.add_argument("--fc-top", type=float, default=0.34)
    args = ap.parse_args()

    scal = Scalogram(width=args.width, lane=args.lane, norm=args.norm,
                     floor_db=args.floor_db)

    # ----- self-test: no socket, craft + decode + render synthetic packets ---
    if args.selftest:
        nframes = args.frames or 400
        print("[dwt_plot] SELFTEST: %d synthetic columns, lane %d" % (nframes, args.lane))
        ok = 0
        for raw in synth_columns(nframes, K=max(4, args.lane + 1)):
            pkt = decode_wavelet_packet(raw)
            assert pkt is not None, "synthetic packet failed to decode!"
            scal.add(pkt)
            ok += 1
        centers = scale_center_freqs(scal.n_oct, scal.n_voc, fs=args.fs,
                                     fc_top=args.fc_top)
        print("[dwt_plot] decoded %d/%d columns  K=%d nscales=%d seq_gaps=%d"
              % (scal.columns_seen, ok, pkt['K'], scal.nscales, scal.seq_gaps))
        print("[dwt_plot] center freqs (Hz): bin0=%.1f  bin%d=%.1f"
              % (centers[0], scal.nscales - 1, centers[-1]))
        render(scal, centers, args.png, live=args.live,
               title="DWT scalogram SELFTEST (diagonal sweep + steady band)")
        return 0

    # ----- live: bind UDP 5004 and consume the wavelet stream ----------------
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    except OSError:
        pass
    s.bind(('', args.port))
    s.settimeout(1.0)
    print("[dwt_plot] bound UDP %d -- waiting for wavelet packets "
          "(Ctrl-C to render + exit)" % args.port)

    t0 = time.time()
    bad = 0
    try:
        while True:
            try:
                data, addr = s.recvfrom(65535)
            except socket.timeout:
                if args.secs and (time.time() - t0) >= args.secs:
                    break
                continue
            pkt = decode_wavelet_packet(data)
            if pkt is None:
                bad += 1
                continue
            scal.add(pkt)
            if scal.columns_seen <= 5 or scal.columns_seen % 100 == 0:
                print("[dwt_plot] col=%d seq=%d K=%d nscales=%d ov=%d gaps=%d"
                      % (scal.columns_seen, pkt['seq'], pkt['K'], pkt['nscales'],
                         pkt['overrun'], scal.seq_gaps))
            if args.frames and scal.columns_seen >= args.frames:
                break
            if args.secs and (time.time() - t0) >= args.secs:
                break
    except KeyboardInterrupt:
        print("\n[dwt_plot] interrupted")
    finally:
        s.close()

    if scal.columns_seen == 0:
        print("[dwt_plot] no wavelet packets received (engine enabled? port %d?). "
              "%d non-wavelet/short datagrams." % (args.port, bad))
        return 1
    centers = scale_center_freqs(scal.n_oct, scal.n_voc, fs=args.fs,
                                 fc_top=args.fc_top)
    print("[dwt_plot] %d columns, %d non-wavelet datagrams, %d seq gaps"
          % (scal.columns_seen, bad, scal.seq_gaps))
    render(scal, centers, args.png, live=args.live)
    return 0


if __name__ == "__main__":
    sys.exit(main())
