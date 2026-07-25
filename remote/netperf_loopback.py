#!/usr/bin/env python3
"""Local UDP loopback throughput harness for net.py's UnifiedSink recv path.

Runs net.py's REAL recv loop + DataValidator against a synthetic broadband
blaster on 127.0.0.1 -- measures net.py's drain ceiling with NO board and NO
flashing. Reports BOTH the sender's actual rate and net.py's drained rate, so a
sender-bound run (harness artifact) can't be mistaken for a receiver-bound one
(net.py's real limit).

  python3 netperf_loopback.py            # open-loop: blast max, read net.py's ceiling
  python3 netperf_loopback.py 30 6 154   # paced 30k for 6s, 154-word packets
  python3 netperf_loopback.py max 6 154  # explicit open-loop

Interpretation:
  * sent < 30k                  -> SENDER-bound (this box can't even generate 30k
                                   in one Python process); ignore -- not a net.py result.
  * sent >> drained, gaps > 0   -> RECEIVER-bound: net.py's ceiling == drained rate.
  * sent ~= drained, gaps ~= 0  -> net.py kept up with everything offered.
Localhost runs the sender AND net.py AND the loopback stack on the same box, so a
passing loopback is CONSERVATIVE vs the real path (board sends, Mac only runs net.py).
"""
import importlib.util, struct, socket, time, os, contextlib, multiprocessing, sys

PORT = 15055

def blaster(port, words, target_pps, seconds, counter, lfp_every=0, lfp_words=142):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8 << 20)
    pkt = bytearray(words * 4)
    struct.pack_into('<I', pkt, 0, 0xCAFEBABE)     # UNIFIED_MAGIC
    struct.pack_into('<I', pkt, 4, 0x00000101)     # broadband, v1
    # A second, LFP-tagged frame every `lfp_every` broadband packets reproduces
    # the lfp_on mix on ONE socket -- the case that actually fails on the board.
    lpkt = bytearray(lfp_words * 4)
    struct.pack_into('<I', lpkt, 0, 0xCAFEBABE)
    struct.pack_into('<I', lpkt, 4, 0x00000102)    # stream_type = 2 (LFP)
    sendto = s.sendto; pack_into = struct.pack_into
    dest = ('127.0.0.1', port); seq = 0; sent = 0; lseq = 0
    end = time.perf_counter() + seconds
    open_loop = target_pps <= 0
    if open_loop:
        # blast as fast as one process can; publish count every 512 packets
        while time.perf_counter() < end:
            for _ in range(512):
                seq = (seq + 1) & 0xFFFFFFFF
                pack_into('<I', pkt, 16, seq)      # SEQ at word 4
                sendto(pkt, dest)
            sent += 512
            counter.value = sent
    else:
        # paced: send a slice, then SPIN-WAIT to the deadline (time.sleep is too
        # coarse on macOS and was itself capping the offered rate at ~10k).
        per_slice = max(1, target_pps // 500); slice_dt = per_slice / target_pps
        while time.perf_counter() < end:
            deadline = time.perf_counter() + slice_dt
            for _ in range(per_slice):
                seq = (seq + 1) & 0xFFFFFFFF
                pack_into('<I', pkt, 16, seq)
                sendto(pkt, dest)
                if lfp_every and seq % lfp_every == 0:
                    lseq = (lseq + 1) & 0xFFFFFFFF
                    pack_into('<I', lpkt, 16, lseq)
                    sendto(lpkt, dest)
            sent += per_slice
            counter.value = sent
            while time.perf_counter() < deadline:
                pass
    counter.value = sent
    s.close()

def lfp_words_note(words):
    return "LFP frames are 142 words / 568 B"

def main():
    arg1 = sys.argv[1] if len(sys.argv) > 1 else 'max'
    target_kpps = 0.0 if arg1 in ('max', '0') else float(arg1)
    seconds     = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    words       = int(sys.argv[3]) if len(sys.argv) > 3 else 154
    # 4th arg: broadband packets per LFP frame. 10 == the real 30k/3k mix,
    # 0 == broadband only.
    lfp_every   = int(sys.argv[4]) if len(sys.argv) > 4 else 0
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
    counter = multiprocessing.Value('q', 0, lock=False)
    p = multiprocessing.Process(target=blaster,
                                args=(PORT, words, int(target_kpps * 1000), seconds,
                                      counter, lfp_every),
                                daemon=True)
    p.start(); time.sleep(1.0)                       # warm-up
    s0, c0, g0, t0 = counter.value, m.validator.packet_count, m.validator.seq_gaps, time.perf_counter()
    l0 = sink.lfp_pkts
    time.sleep(max(1.0, seconds - 2.0))
    s1, c1, g1, t1 = counter.value, m.validator.packet_count, m.validator.seq_gaps, time.perf_counter()
    l1 = sink.lfp_pkts
    p.join(); sink.stop()
    dt = t1 - t0
    sent_rate = (s1 - s0) / dt / 1000
    drain_rate = (c1 - c0) / dt / 1000
    lfp_rate = (l1 - l0) / dt / 1000
    mode = 'open-loop' if target_kpps == 0 else f'paced {target_kpps:.0f}k'
    mode += ' +LFP' if lfp_every else ' broadband-only'
    if lfp_every:
        print(f"  [mix] 1 LFP frame per {lfp_every} broadband; "
              f"LFP drained {lfp_rate:.1f}k/s ({lfp_words_note(words)})")
    print(f"[{mode}] sender sent {sent_rate:.1f}k/s | net.py drained {drain_rate:.1f}k/s | "
          f"seq_gaps in window: {g1 - g0} | ring_drops: {sink._ring_drops}")
    if sent_rate < 29:
        print("  -> SENDER-bound: this box can't generate 30k in one process; "
              "not a net.py result (net.py saw everything sent, drops at kernel if any).")
    elif drain_rate >= 29.5 and (g1 - g0) < sent_rate * 10:
        print(f"  -> net.py KEEPS UP: drained {drain_rate:.1f}k with sender pushing "
              f"{sent_rate:.1f}k. net.py is NOT your bottleneck on this host.")
    else:
        print(f"  -> RECEIVER-bound: net.py's ceiling on this host is ~{drain_rate:.1f}k "
              f"(sender offered {sent_rate:.1f}k, {g1 - g0} gaps). This is the regression surface.")

if __name__ == '__main__':
    main()
