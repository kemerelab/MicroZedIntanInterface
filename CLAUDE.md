# CLAUDE.md

Guidance for working in this repository.

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

- The **PL** talks to up to two RHD2000 chips over a modified SPI protocol: SCLK, CS,
  COPI, and two CIPO lines using **double-data-rate** (two 16-bit words interleaved on
  alternate clock phases). Long cables introduce CIPO phase delay, so the PL latches
  CIPO with a programmable delay relative to the clock.
- The acquisition loop is **80 states @ 84 MHz** (4× oversampling of the 24 MHz SPI
  clock), repeated 35× per packet (35 COPI commands → 35×2 readback words per CIPO line).
- Each packet = 10 header words (magic `0xCAFEBABE_DEADBEEF` + 64-bit timestamp +
  digital-in + metadata + 8 external ADC values) plus up to 70 data words. Users select
  which of 4 channels (regular/DDR × CIPO0/CIPO1) to stream.
- The **PS** runs bare-metal on both ARM cores: **core 0** runs the fast loop (TCP
  commands, BRAM reads, UDP streaming, ~9 MB/s, up to 128 ch @ 30 ksps); **core 1**
  runs the serial debug console.
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
- `network.c` — lwIP TCP server (port 6000), UDP stream (port 5000), status struct.
- `pl_control.c` — AXI-Lite register read/write helpers, COPI command sequences.

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
- Status regs: read back starting at offset `(22*4)`; see `STATUS_REG_*_OFFSET`.
- BRAM: base `0x80000000`, 16384 × 32-bit words (64 KB).
- Packet: 10 header words + 18..70 data words depending on `channel_enable`.

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
- `write_fifo` in `fifo_bram_interface.sv` must **not** be reset element-by-element — that
  forces ~18k flip-flops + a 256:1 read mux and a routing-congestion hotspot. Leaving the
  array unreset makes it infer LUTRAM (safe: entries are only read after being written).
- The AXI fabric clock is **131.25 MHz** (was 175 MHz, over-spec for the `-1` part's AXI-GP
  port — it caused the only remaining setup violations). ~525 MB/s BRAM bandwidth still far
  exceeds the ~9 MB/s stream.
- Many control regs are **only latched while transmission is inactive** (see the "safe
  control register" logic in `data_generator_core.sv`) — changing phase/channel/COPI
  words mid-stream has no effect until you stop and restart.
- `net.py` runs on macOS and Linux. Some socket options are platform-specific (e.g.
  `TCP_KEEPIDLE` is Linux-only, `TCP_KEEPALIVE` is the macOS equivalent) — guard new ones
  with `hasattr(socket, ...)`. See `configure_tcp_keepalive()`.
- Git: remote `origin` uses the `github.com-microzed` SSH host alias. Commit/push only
  when asked.
