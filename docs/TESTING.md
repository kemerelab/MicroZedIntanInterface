# Testing & benchmarking

How to run the verification tooling for the broadband datapath. All RTL sims use
Vivado's `xsim` (`source /opt/Xilinx/2025.1/Vivado/settings64.sh` first); the
benchmarks run on the board and report over the serial console / `net.py`.

## RTL simulations (`programmable_logic/sim/`)

Each testbench has a `run_*.sh` that compiles the RTL it needs and runs `xsim`, then
prints `RESULT: PASS` / `RESULT: FAIL`.

```bash
cd programmable_logic/sim
source /opt/Xilinx/2025.1/Vivado/settings64.sh
bash run_dualport_dropout_tb.sh     # THE broadband integrity check (run this first)
```

| Testbench | `run_*.sh` | What it proves |
|---|---|---|
| `dualport_dropout_tb.sv` | `run_dualport_dropout_tb.sh` | **Broadband data integrity**: every data word out of the wrapper is byte‑exact vs the RTL sine reference across both cable ports, and SEQ/timestamp advance +1/packet with no gaps. This is the canonical "no dropout / no loss" proof. |
| `chirp_tb.sv` | `run_chirp_tb.sh` | The analytic **chirp NCO** (debug‑mode synthetic sweep) is bit‑exact vs the Python reference (`gen_chirp_vectors.py`). Captures the 35 data words/packet off the FIFO write interface (7‑word header skipped). |
| `fifo_bram_dualport_tb.sv` | `run_fifo_bram_dualport_tb.sh` | FIFO→BRAM dual‑port write/read path. |
| `aux_command_engine_tb.sv` | `run_aux_engine_tb.sh` | The merged **aux command engine** (banked looping store + real‑time fast‑settle / digout / DSP‑reset override + one‑shot injection), checked against expected values using the named roles from `acq_frame_pkg`. |
| `data_generator_aux_wire_tb.sv` | `run_aux_wire_tb.sh` | Aux **wire‑level integration**: decodes the serialized COPI out of the real core and verifies the always‑on aux commands (channel CONVERTs, the programmed aux loop, fast‑settle replace / D5‑force, injection) reach the chip. |
| `axi_lite_write_tb.sv`, `tb_axi_read_bram_ctrl.sv` | `run_axi_write_tb.sh`, `run_axi_read_sim.sh` | AXI‑Lite register write + BRAM‑controller read paths. |

Regenerate the chirp reference vectors (only if you change the NCO): `python3 gen_chirp_vectors.py`.

## BRAM‑read benchmark — *why DMA is required* (`benchmark_bram_reads.c`)

A bring‑up diagnostic that times the three ways of reading the capture BRAM, so you
can see for yourself why the shipped design uses **AXI‑CDMA** rather than the CPU.

Run it from the **serial debug console** (core‑1 UART, 115200 8N1):

```
benchmark
```

It reads a fixed block of packets from the capture BRAM via (a) word‑by‑word
`Xil_In32` single‑beat reads and (b) an `memcpy`/burst path, and prints the elapsed
time + throughput for each to the console. The takeaway: single‑beat reads over the
PS `M_AXI_GP` master cannot sustain the 0xFF (256‑ch, 150‑word) packet rate at the
131.25 MHz AXI clock, and CPU bursts of the BRAM corrupt the 0xFF stream — which is
why the real path moves the packet BRAM→DDR by CDMA (a PL master over `S_AXI_HP0`),
landing straight into the pbuf. (See `docs/bram_burst_read_bug.md` and the DMA rule
in `CLAUDE.md`.)

## check‑dma guardrail (`.claude/skills/check-dma/`)

A skill/script that scans for the anti‑pattern the benchmark above motivates: any
new PL→PS bulk‑data path that loops the CPU over BRAM/staging instead of using CDMA.
Run it before declaring a data‑path change done; genuinely‑justified single‑beat
peeks (e.g. a 2‑word magic/resync read) must be annotated `// DMA-EXEMPT: <reason>`.

## Host‑side (`remote/net.py`)

`python3 net.py` connects (TCP `0x6900`), starts streaming, and validates the
unified UDP stream (data on `0x6800`): per‑stream SEQ continuity (the loss check),
magic/size checks, and cable/phase detection. A clean run shows **0 SEQ gaps**.

> **UDP throughput benchmark** (board‑side blaster + `net.py` meter for MB/s vs
> packet size) is a separate tool being re‑added on top of this branch — it needs a
> dedicated bench port in the new `0x68xx` scheme and is best validated with a live
> host‑side throughput measurement.
