#!/usr/bin/env python3
"""Local UDP loopback throughput harness for net.py's UnifiedSink recv path.
Runs net.py's REAL recv loop + DataValidator against a synthetic broadband
blaster on 127.0.0.1 -- validates net.py can drain 30k s/s with NO board and NO
flashing. Run: python3 netperf_loopback.py [target_kpps] [seconds] [words]"""
import importlib.util, struct, socket, time, os, contextlib, multiprocessing, sys

PORT = 15055

def blaster(port, words, target_pps, seconds):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 << 20)
    pkt = bytearray(words * 4)
    struct.pack_into('<I', pkt, 0, 0xCAFEBABE)     # UNIFIED_MAGIC
    struct.pack_into('<I', pkt, 4, 0x00000101)     # broadband, v1
    dest = ('127.0.0.1', port); seq = 0
    end = time.perf_counter() + seconds
    # pace in 5ms slices to hit target_pps without a per-packet sleep
    per_slice = max(1, target_pps // 200); slice_dt = per_slice / target_pps
    while time.perf_counter() < end:
        t = time.perf_counter()
        for _ in range(per_slice):
            seq = (seq + 1) & 0xFFFFFFFF
            struct.pack_into('<I', pkt, 16, seq)   # SEQ at word 4
            s.sendto(pkt, dest)
        rest = slice_dt - (time.perf_counter() - t)
        if rest > 0: time.sleep(rest)
    s.close()

def main():
    target_kpps = float(sys.argv[1]) if len(sys.argv) > 1 else 35.0
    seconds     = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    words       = int(sys.argv[3]) if len(sys.argv) > 3 else 154
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location('netmod', os.path.join(here, 'net.py'))
    m = importlib.util.module_from_spec(spec)
    with open(os.devnull, 'w') as dn, contextlib.redirect_stdout(dn):
        spec.loader.exec_module(m)
    m.validator.expected_packet_size_words = words
    m.validator.expected_packet_size_bytes = words * 4
    sink = m.UnifiedSink(port=PORT)
    with open(os.devnull, 'w') as dn, contextlib.redirect_stdout(dn):
        assert sink.start()
    p = multiprocessing.Process(target=blaster, args=(PORT, words, int(target_kpps*1000), seconds), daemon=True)
    p.start(); time.sleep(1.0)                       # warm-up
    c0, g0, t0 = m.validator.packet_count, m.validator.seq_gaps, time.perf_counter()
    time.sleep(seconds - 2.0)
    c1, g1, t1 = m.validator.packet_count, m.validator.seq_gaps, time.perf_counter()
    p.join(); sink.stop()
    rate = (c1 - c0) / (t1 - t0) / 1000
    print(f"offered {target_kpps:.0f}k/s | net.py drained {rate:.1f}k/s | "
          f"seq_gaps in window: {g1 - g0} | ring_drops: {sink._ring_drops}")

if __name__ == '__main__':
    main()
