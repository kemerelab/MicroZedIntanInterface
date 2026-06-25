#!/usr/bin/env python3
"""
dwt_test_sender.py -- emit a few well-formed wavelet (DWT scalogram) UDP packets
to 5004 (or any host/port), so dwt_plot.py and the OE DwtCanvas can be exercised
without the board.

Pattern: lane 0 carries a diagonal frequency sweep (a ridge that walks from the
lowest scale to the highest over the run); every lane also carries a steady
mid-scale band. A correct decode/render shows a moving diagonal + a horizontal
line -- an unambiguous visual check that the scale axis is oriented correctly.

Usage:
    python3 dwt_test_sender.py                       # 400 packets to 127.0.0.1:5004
    python3 dwt_test_sender.py --host 192.168.18.5 --port 5004 --rate 50 --frames 1000
"""
import argparse
import socket
import time

import dwt_plot   # reuse the exact builder used by the plotter's selftest


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=dwt_plot.WAV_UDP_PORT)
    ap.add_argument("--frames", type=int, default=400)
    ap.add_argument("--rate", type=float, default=200.0, help="packets/second")
    ap.add_argument("--K", type=int, default=4)
    ap.add_argument("--n-oct", type=int, default=dwt_plot.WAV_N_OCTAVES)
    ap.add_argument("--n-voc", type=int, default=dwt_plot.WAV_V)
    args = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dt = 1.0 / args.rate if args.rate > 0 else 0.0
    nscales = args.n_oct * args.n_voc
    print("[sender] -> %s:%d  frames=%d K=%d nscales=%d rate=%.0f/s"
          % (args.host, args.port, args.frames, args.K, nscales, args.rate))
    for i in range(args.frames):
        active = {}
        sweep_s = int((i / float(max(1, args.frames - 1))) * (nscales - 1))
        for lane in range(args.K):
            active[(lane, sweep_s)] = 1.0
            active[(lane, nscales // 2)] = 0.6
        pkt = dwt_plot.build_packet(i, K=args.K, n_oct=args.n_oct,
                                    n_voc=args.n_voc, active=active)
        s.sendto(pkt, (args.host, args.port))
        if dt:
            time.sleep(dt)
    s.close()
    print("[sender] done (%d packets)" % args.frames)


if __name__ == "__main__":
    main()
