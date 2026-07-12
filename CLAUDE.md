# CLAUDE.md

Guidance for working in this repository.

## Hard rules (non-negotiable)

- **NO DATA LOSS — top priority, set in stone.** This is a neural-data acquisition system;
  broadband is archival and must never drop a packet, and the principle applies to every
  stream. **We step back the spec to guarantee perfect data — we never design around losing
  it.** Concretely: size every packet to fit ONE standard datagram (no IP fragmentation, no
  jumbo MTU, no MTU framer/chunker) — *specify* the data so it inherently fits (e.g. one
  octave per scalogram packet); if a config wouldn't fit or would saturate the link/TX path,
  **reduce the spec** (fewer channels/octaves/rate), do not accept loss. Carry a per-stream
  sequence number so loss is *provably zero and detectable*, not assumed. Drain promiscuously
  host-side (recv → ring, process later) so nothing is dropped while waiting. Never write
  "best-effort", "drop-tolerant", or "lossy monitor" into a design — if you're tempted to,
  step the spec back instead.

## What this project is

An FPGA-based data acquisition system for **Intan RHD2000-style neural recording
chips**, built on a **MicroZed (Xilinx Zynq-7020)** board with a custom carrier PCB.

Data path end to end:

```
Intan chips ──SPI(DDR)──► PL (FPGA fabric) ──FIFO/BRAM──► PS (dual ARM A9) ──UDP──► host
                              ▲                                  ▲
                              └────── AXI-Lite registers ────────┘
host (net.py) ──TCP control──────────────────────────────────────┘
```

- The **PL** talks to up to **two cables** of RHD2000 chips (ports A/B, a dual-port
  datapath) over a modified SPI protocol. Each cable is SCLK, CS, COPI, and two CIPO
  lines using **double-data-rate** (two 16-bit words interleaved on alternate clock
  phases) — so up to four chips / **256 channels** total. Long cables introduce CIPO
  phase delay, so the PL latches CIPO with a programmable (per-cable) delay relative
  to the clock.
- The acquisition loop is **80 states @ 84 MHz** (4× oversampling of the 24 MHz SPI
  clock), repeated 35× per packet (35 COPI commands → 35×2 readback words per CIPO line).
- Each packet = 10 header words (magic `0xCAFEBABE_DEADBEEF` + 64-bit timestamp +
  digital-in + metadata + 8 external ADC values) plus up to 140 data words. Users select
  which of 8 streams (regular/DDR × CIPO0/CIPO1 × two cables) to stream via the 8-bit
  channel-enable mask.
- The **PS** runs bare-metal on both ARM cores: **core 0** runs the fast loop (TCP
  commands, BRAM reads, UDP streaming, ~9 MB/s per cable, up to 256 ch @ 30 ksps ≈
  ~18 MB/s at the full `0xFF` config); **core 1** runs the serial debug console.
- The **host** (`remote/net.py`) sends TCP control commands and receives/validates the
  UDP data stream.

`overview.txt` has the authoritative narrative description — read it before making
architectural changes.

## Repo layout

| Path | What |
|------|------|
| `programmable_logic/src/` | Verilog/SystemVerilog RTL (the PL) |
| `programmable_logic/constraints/` | XDC pin/timing constraints |
| `programmable_logic/block_design/design_1_bd.tcl` | Vivado block design |
| `programmable_logic/ip/` | Custom Intan SPI IP |
| `firmware/src-core0/` | PS core-0 firmware (main loop, network, PL control) |
| `firmware/src-core1/` | PS core-1 firmware (debug console) |
| `firmware/src-shared/` | Code shared between cores |
| `firmware/include/main.h` | Register map, packet layout, struct defs |
| `remote/net.py` | Host-side TCP control + UDP capture + cable/phase detection |
| `scripts/` | Vivado/Vitis build + bootgen scripts |
| `pcb/` | KiCad carrier board design |
| `docs/`, `blobs/` | Documentation and prebuilt firmware blob |

## Key source files

**PL:**
- `data_generator_core.sv` — the core: 3 always blocks (master state machine + timestamp,
  acquisition FSM, exfiltration FSM). Control/status register *meanings* are decoded here.
- `CIPO_phase_selector.v` — picks the correctly delayed CIPO sample.
- `lvds_spi_interface.v`, `fifo_bram_interface.sv`, `bram.sv`, `bram_wrapper.v`
- `axi_lite_registers.v` — generic AXI-Lite register file (CDC between AXI and PL clocks);
  22 control regs (PS→PL), 11 status regs (PL→PS). Register *layout* lives here.
- `data_generator_wrapper.v` — wires the core, FIFO/BRAM, and AXI regs together.

**PS (core 0):**
- `main.c` — init + `network_maintenance_loop`.
- `network.c` — lwIP TCP server (port 0x6900), UDP stream (port 0x6800), status struct.
- `pl_control.c` — AXI-Lite register read/write helpers, COPI command sequences.
- `pl_dma.c` — AXI CDMA driver: the bulk capture-BRAM read goes BRAM→DDR via a PL
  master (CDMA over S_AXI_HP0), **not** the CPU's M_AXI_GP (whose long burst reads
  corrupt the 0xFF stream). `BRAM_READ_METHOD` in `main.c` selects DMA (default) vs
  the single-beat `Xil_In32` reference; only the magic/resync *peeks* still use GP.

**Host:**
- `net.py` — single-file client. Command IDs, packet validation (`DataValidator`),
  and `CableDetection` mirror the firmware/PL definitions.

## The register / packet contract (keep these in sync!)

Three places encode the **same** register map and packet format. Changing one means
changing the others:

1. `programmable_logic/src/axi_lite_registers.v` + `data_generator_core.sv` (PL)
2. `firmware/include/main.h` (`CTRL_*` / `STATUS_*` offsets, `status_response_t`) (PS)
3. `remote/net.py` (`CMD_*` IDs, `calculate_packet_size`, struct unpacking) (host)

- Control regs: base `0x40000000` (AXI-Lite). CTRL_REG_0..2 = enable/phase/channel;
  MOSI/COPI words start at reg offset 4.
- Status regs: read back starting at offset `(PL_N_CTRL_REGS*4)` = `(25*4)` (grew from
  `22*4` when the aux control regs landed at 22–24); see `STATUS_REG_*_OFFSET`.
- BRAM: base `0x80000000`, 16384 × 32-bit words (64 KB). LFP output BRAM: base
  `0x84000000` (same size), in the `axi_cdma_0/Data` space.
- Packet: 10 header words + 18..140 data words depending on `channel_enable` (8-bit mask).
- **LFP packet (PL-built):** the PL builds the complete LFP wire packet in the LFP output
  BRAM — a **6-word header** (`w0=0x1F1FBEEF`, `w1=0xCAFEBABE`, `w2/w3=`64-bit master
  timestamp of the last contributing broadband sample, `w4=lane_mask|(decim_R<<8)|
  (num_taps<<16)|(overrun<<24)`, `w5=`PL frame seq) then the decimated samples — and the PS
  just CDMAs the whole frame into a pbuf and sends it. The **LFP lane mask MIRRORS the
  broadband `channel_enable` mask** (single source of truth: `data_generator_core.sv` drives
  it from `channel_enable_reg`); `lfp_cfg[15:8]` and `CMD_LFP_SET_CHANNELS` are deprecated.
  Keep this header in sync across `lfp_dsp_block.sv`, `network.c`, and `net.py::receive_lfp`.
- **Rule — `get_status` reports everything configurable.** Any setting the host can
  change (a CTRL register or a command that alters behavior) must also be surfaced in
  `status_response_t`, so the host can always read back the full device configuration.
  When you add a control, add its read-back to `collect_status_data` (`network.c`) and to
  `net.py`'s `get_status` decode/print. A `_Static_assert` on `sizeof(status_response_t)`
  (in `main.c`) guards the wire size — bump it and net.py's length/offsets together.

## Build & run

**PL (Vivado, 2025.1):**
```bash
source ~/Xilinx/2025.1/Vivado/settings64.sh
vivado -mode batch -source scripts/create_vivado_project.tcl   # creates vivado_project/
vivado -mode batch -source scripts/build_bitstream.tcl         # -> vivado_project/klab.xsa
```
Part is `xc7z020clg400-1` (set in `scripts/create_vivado_project.tcl`).

**PS (Vitis, 2025.1):**
```bash
source ~/Xilinx/2025.1/Vitis/settings64.sh
vitis -s scripts/create_vitis_project.py    # creates vitis_workspace/ + builds both cores
vitis -s scripts/build_vitis_project.py     # incremental rebuild of firmware only
```
Firmware builds at **-O3** (cannot meet timing at -O0). Core 0 loads at DDR `0x100000`,
core 1 at `0x20000000`.

- After **any PL change**, regenerate the platform with the **`create_`** script (it
  rebuilds the platform from the new `.xsa`). `build_vitis_project.py` only recompiles the
  apps — its `update_hw(...)` is commented out, so it would build against *stale* hardware.
  `create_` needs a clean workspace, so remove/move `vitis_workspace/` first.
- The firmware ELFs and FSBL don't depend on the PL clocks, so a PL-only change needs only
  a bitstream re-stage + `bootgen` (below), not a firmware rebuild.
- A clean PL build is ~13–16 min (longer if it congests).
- **Incremental PL rebuilds can use a STALE netlist.** `scripts/build_bitstream.tcl` runs
  `reset_run synth_1` (the *top* run only). The `data_generator` is an **out-of-context (OOC)
  module run** (`design_1_data_generator_0_synth_1`) that it does **not** reset — so after
  editing `data_generator` or its submodules, an incremental `build_bitstream.tcl`
  re-implements the *old* OOC netlist and your RTL change silently doesn't take. Either
  `reset_run design_1_data_generator_0_synth_1` first, or rebuild from scratch via
  `create_vivado_project.tcl` → `build_bitstream.tcl`.

**Bootable SD card:**
```bash
bootgen -image scripts/boot.bif -o BOOT.bin -w   # copy BOOT.bin to FAT32 'Boot' partition
```
`boot.bif` packs FSBL + bitstream + both core ELFs, and references the bitstream by an
**explicit path** (`vitis_workspace/klab-firmware/_ide/bitstream/klab_project.bit`), *not*
the copy inside the `.xsa`. After a PL rebuild, stage the fresh bitstream there first:
```bash
mkdir -p vitis_workspace/klab-firmware/_ide/bitstream
cp vivado_project/klab_project.runs/impl_1/design_1_wrapper.bit \
   vitis_workspace/klab-firmware/_ide/bitstream/klab_project.bit
```
The repo keeps the current bootable image at `blobs/BOOT.bin`.

**Host testing:**
```bash
cd remote && python3 net.py     # connects to ZYNQ_IP (default 192.168.18.10)
```
`net.py` auto-detects the host IP and reconfigures the board's UDP destination over TCP.
Edit `ZYNQ_IP`/ports at the top of the file if the board address differs.

## Conventions & gotchas

- **PL-first: ask "can the fabric solve this?" before changing the protocol or PS software.**
  The PS is a single, bare-metal, **fully-polled, run-to-completion** core (no interrupts —
  `platform.c` sets up only caches+UART, the CDMA runs `XAxiCdma_IntrDisable`). So PS-software
  fixes (e.g. batching UDP packets) are band-aids that still load that one core; the PL can
  usually restructure the data so the bottleneck disappears, deterministically. Example: the
  per-packet timing jitter and the LFP/wavelet contention come from doing **3 separate polled
  CDMA transfers** (broadband/LFP/DWT BRAMs) + 3 sends per cycle — the PL fix is to assemble
  **all PL→PS data into one shared BRAM stream** so the PS does a *single* DMA + demux-by-magic
  to the right UDP port (LFP adds ~10% data, DWT a little at the LFP rate — negligible vs the
  3-way poll contention). Reach for protocol/software changes only after ruling out a PL one.
- `vivado_project/` and `vitis_workspace/` are **generated and gitignored** — never commit
  them. Regenerate from the `scripts/` tcl/py files.
- The PL crosses two clock domains (131.25 MHz AXI fabric ↔ 84 MHz PL data path) via
  two-stage synchronizers in `axi_lite_registers.v` plus the dual-port capture BRAM.
  Status/control are flat 32×N bit busses. The two clocks are declared **asynchronous**
  (`constraints/zzz_clock_groups.xdc`) — they communicate only through those CDC structures,
  so don't add single-cycle paths between them or time them as synchronous.
- The PL data path (`data_generator`, 84 MHz) must be reset from the **84 MHz**
  `proc_sys_reset_0_84M`, **not** the AXI/175-domain reset — a cross-domain reset fails
  timing on ~20k endpoints. (Root-caused in `docs/routing_report.md`.)
- **PL→PS bulk data ALWAYS moves by DMA — never loop the CPU over BRAM or staging.** This is
  a hard rule, not a preference. The CPU's `M_AXI_GP` long burst reads corrupt the 0xFF stream,
  and single-beat `Xil_In32` reads (or word-by-word reads of the non-cacheable DMA staging
  buffer) are **slow** and starve the core-0 loop — the cost scales with payload, so it blows
  the 33 µs/packet budget as channels/scales grow (an LFP `Xil_In32` loop pushed recv→transmit
  to ~63 µs; a wavelet 2048-word uncached staging repack pushed it to **2.6 ms**, 80× over).
  Move bulk data by **AXI CDMA landed straight into the pbuf payload**; the cleanest form is to
  have the **PL build the whole wire packet (header + payload) in its result BRAM** and the PS
  just DMA+send it — exactly as the broadband path does (the PL writes the 10-word header in
  `data_generator_core.sv`). **The LFP path now does this**: `lfp_dsp_block.sv` builds the
  complete LFP wire packet (6-word header + decimated samples) in the LFP output BRAM, and
  `network.c::lfp_stream_service` CDMAs each frame into a non-cacheable staging buffer
  (`pl_dma_read_addr`) and sends a `PBUF_REF` — no PS header math, no `Xil_In32` sample loop.
  Result/analysis BRAMs must be in the `axi_cdma_0/Data` address space in the BD (the wavelet
  @0x90000000 is; the LFP @0x84000000 **now is too** — added to `axi_cdma_0/Data` in
  `design_1_bd.tcl`; it had been excluded). **`scripts/check_dma.sh` enforces this** (and the
  `/check-dma` skill) — run it before declaring any PL↔PS data-path change done; annotate
  genuinely-justified single-beat peeks (e.g. a 2-word magic/resync read) with
  `// DMA-EXEMPT: <reason>`.
  (History: an earlier "CDMA hung on the STFT result BRAM" turned out to be a missing
  `axi_cdma_0/Data` address segment, not a CDMA limitation — see the wavelet DMA fix.)
- **A compute pass that spans multiple acquisition packets must snapshot its inputs.** The
  next 30 kHz packet's data arrives *during* a long pass, so a single-buffered input gets
  overwritten mid-pass (this bit the CIC LFP path; the FIR decimator avoids it with a ring +
  `head_snap`). Snapshot or double-buffer the pass inputs.
- `write_fifo` in `fifo_bram_interface.sv` must **not** be reset element-by-element — that
  forces ~18k flip-flops + a 256:1 read mux and a routing-congestion hotspot. Leaving the
  array unreset makes it infer LUTRAM (safe: entries are only read after being written).
- **xsim treats `logic name = expr;` as a one-time initializer, not a continuous assign.**
  In Vivado *synthesis* a declaration-with-initializer is a continuous assignment, but under
  *xsim* (per the LRM) it runs once at time 0, so the signal sticks at its initial value
  (usually X) and silently idles the datapath in simulation while synthesizing correctly. Use
  `wire name = expr;` for continuous combinational assigns — identical netlist, correct sim
  semantics. (This once made an identity testbench "pass" vacuously with both cores idle.)
- **A bare `a*b` product takes a self-determined width and can truncate.** A multiply nested
  in a wider expression (e.g. inside a ternary) is evaluated at `max(width(a),width(b))` — the
  product is truncated *before* it reaches the wider target. Widen both operands to the full
  product width first (`PROD_W`). (Bit the dual-MAC FIR generalization.)
- **AXI-Lite writes must accept AW and W jointly, or they can deadlock.** A write FSM that
  toggles `awready`/`wready` independently against their own valids can hang: if the
  interconnect presents `AWVALID` and `WVALID` one cycle apart (legal, traffic-dependent), the
  two readys oscillate in anti-phase forever — the write never completes, `BVALID` never
  asserts, and the ARM core stalls mid-store on the GP port. Assert `AWREADY`/`WREADY`
  **together**, only when both valids are present and no response is pending (AXI-compliant),
  and guard `ARREADY` with `~rvalid`. This was latent in `axi_lite_registers.v` until a
  microsecond-fast back-to-back command burst (the aux-bank upload from the ephys-socket
  plugin) hit the skew and hard-wedged core 0. TB: `sim/axi_lite_write_tb.sv`.
- The AXI fabric clock is **131.25 MHz** (was 175 MHz, over-spec for the `-1` part's AXI-GP
  port — it caused the only remaining setup violations). ~525 MB/s BRAM bandwidth still far
  exceeds the ~9 MB/s stream.
- Many control regs are **only latched while transmission is inactive** (see the "safe
  control register" logic in `data_generator_core.sv`) — changing phase/channel/COPI
  words mid-stream has no effect until you stop and restart.
- `net.py` runs on macOS and Linux. Some socket options are platform-specific (e.g.
  `TCP_KEEPIDLE` is Linux-only, `TCP_KEEPALIVE` is the macOS equivalent) — guard new ones
  with `hasattr(socket, ...)`. See `configure_tcp_keepalive()`.
- **A TCP command *burst* (e.g. the LFP coefficient upload in `lfp_config`/`lfp_sweep`, ~43
  back-to-back commands) can hang the board on the *first* interaction after (re)connecting,
  intermittently — but sending a single `get_status` (any one command→response) first
  "primes" it and the burst then goes through; a clean/long power-cycle also clears it.
  Likely cause: **DDR data remanence** — a Zynq power cycle reloads the bitstream and resets
  the PS, but does **not** zero DRAM (only `.bss`/`.sbss` are zeroed by the C startup), so
  stale DDR-resident state (most likely the lwIP pbuf pool / GEM Ethernet BD rings) leaves
  the send/ack path flaky until a single round-trip drains it; a longer power-off lets DRAM
  decay so "it goes away." The coef-write path itself does **not** busy-wait and the command
  parser resyncs on `accept`, so it's not a firmware spin. Workaround: do one `get_status`
  right after connecting before any burst (`lfp_sweep` already preflights with one). Proper
  fix (TODO): explicitly zero / re-init the GEM descriptor rings + lwIP pools at boot rather
  than relying on implicit zero, and confirm `_start` zeroes the `NOLOAD` `.sbss`.
- Git: remote `origin` uses the `github.com-microzed` SSH host alias. **Push feature
  branches to `origin` by default** — after committing work, `git push -u origin <branch>`
  (the maintainer debugs on a separate machine and needs branches available remotely). Do
  **not** commit or push directly to `main`, and never `merge` to `main`, without being
  asked — open a PR (or push the branch and let the maintainer merge) instead.

## Working autonomously / overnight

- When the user hands off for an **overnight / autonomous** session, plan and execute on
  the order of **~8 hours of work**, not 15 minutes. Be ambitious about scope.
- **Attempt the risky thing rather than staging it for review.** A full PL build is ~15 min;
  you can run a half-dozen of them, on different approaches, and `git`-roll-back the ones
  that don't pan out — that is a good use of hours you're not otherwise using. "Stage the
  BD change for review" is the wrong default overnight: just make the change, build it, and
  report what happened (closed timing / failed where). Commit working states as checkpoints;
  branch/stash to explore alternatives.
- Verify what you can in sim first (cheap), then **push all the way through the build** —
  don't stop at "proven in sim, plumbing remains." The plumbing + build closure is exactly
  the part worth burning overnight cycles on.
- Leave a clear morning summary: what built, what timing closed at, what failed and why,
  and any genuine decisions left for the user.
