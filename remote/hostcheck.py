#!/usr/bin/env python3
"""Is the host slow at COMPUTE, at SYSCALLS, or only at NETWORKING?

netperf_loopback measures the recv loop, which needs all three -- so a low
number there cannot say which one degraded. These three microbenchmarks use no
board and no network interface, and each isolates one layer.

Run it healthy to get baselines, then again when degraded. Whichever ratio
moves is the layer that broke.

  python3 hostcheck.py            # benchmarks only
  python3 hostcheck.py --state    # benchmarks + the kernel state worth keeping

Capture a run while the host is HEALTHY first. This fault clears on reboot, so
without a baseline there is nothing to compare a degraded run against.
"""
import time, socket, os, sys, subprocess

def bench(label, fn, target_s=1.5):
    n, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < target_s:
        fn(); n += 1
    return label, n / (time.perf_counter() - t0)

# 1. Pure CPU: no syscalls, no allocation churn. Sensitive ONLY to clock
#    speed / core type -- i.e. thermal or power throttling, or E-core demotion.
def cpu_work(_r=range(20000)):
    s = 0
    for i in _r: s += i * i
    return s

# 2. Syscall round trip, no networking at all. Separates "kernel entry got
#    expensive" from "the network stack got expensive".
def syscall_work():
    for _ in range(2000): os.getpid()

# 3. Loopback UDP: the actual thing netperf exercises.
_s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_s.bind(('127.0.0.1', 0)); _s.settimeout(1.0)
_dest = _s.getsockname(); _buf = bytearray(616); _view = memoryview(_buf)
def net_work():
    for _ in range(200):
        _s.sendto(_buf, _dest); _s.recv_into(_view)

print("host layer check -- run healthy for a baseline, again when degraded\n")
for label, rate in [bench("1. CPU      (no syscalls) ", cpu_work),
                    bench("2. syscalls (no network)  ", syscall_work),
                    bench("3. loopback UDP           ", net_work)]:
    print(f"  {label}: {rate:10.1f} iters/s")
if "--state" in sys.argv:
    # Kernel/network state that persists until reboot, which is the profile of
    # this fault. mbuf counters are the ones to watch: a long high-rate run
    # churns tens of millions of kernel buffers, and a leak or fragmentation
    # there degrades every socket -- loopback included -- until reboot.
    print("\n--- kernel state ---")
    for label, cmd in [
        ("mbuf pool (KEY: 'denied'/'delayed' should be 0)", ["netstat", "-m"]),
        ("power / thermal",                                  ["pmset", "-g", "therm"]),
        ("low power mode",                                   ["pmset", "-g"]),
        ("top CPU consumers",                                ["ps", "-Ao", "pcpu,comm", "-r"]),
        ("uptime / load",                                    ["uptime"]),
    ]:
        print(f"\n== {label} ==")
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=15).stdout
            print("\n".join(out.splitlines()[:14]) or "  (no output)")
        except Exception as e:
            print(f"  ({cmd[0]} unavailable: {e})")

print("""
Reading it:
  ALL THREE down ~3x      -> CPU is throttled (thermal / Low Power Mode / E-core
                             demotion). Not a networking bug at all.
  1 fine, 2 and 3 down    -> kernel entry got expensive (a system extension or
                             security agent hooking syscalls).
  1 and 2 fine, 3 down    -> genuinely the network stack.""")
