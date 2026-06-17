# Hardware-in-the-loop (HITL) setup

> **Superseded (2026-06-16):** laptop-driven HITL via JTAG/QSPI was attempted and
> **abandoned** — see `docs/hitl_laptop_findings.md` for what worked, what didn't, and
> why. Use the SD-card + remote-build workflow; keep the laptop for `net.py`
> verification. The brainstormed plan below is kept for reference only.

Goal: let a Claude Code instance build, flash, and test firmware/PL changes
against the real MicroZed board autonomously — so chunk-size tuning and the DMA
work can iterate in minutes instead of round-tripping through a human.

This is a brainstormed process; the agent on the HITL laptop should verify each
step against the actual hardware and refine the helper scripts (especially the
JTAG target names, which vary).

## 0. What you need

- **Laptop** with **Vivado *and* Vitis 2025.1** installed (Vivado alone can program
  the bitstream but not download the ARM ELFs or build firmware — you need Vitis's
  `xsct` and the `arm-none-eabi` toolchain). Linux preferred (the build scripts and
  `net.py` are tested there); macOS works for `net.py`.
- **MicroZed + carrier board.**
- **One USB cable** to the MicroZed's FTDI port → exposes **both** JTAG and a UART
  serial console (two functions on one cable).
- **Ethernet** between the board and the laptop (direct cable with static IPs, or
  via a switch). Needed for `net.py` TCP control (6000) + UDP capture (5000).
- The repo cloned on the laptop, on branch **`claude/dual-port`**.
- `pip install pyserial` (for reading the UART from scripts).

## 1. Physical + network bring-up (one time)

1. Plug in USB (JTAG+UART) and Ethernet. Power the board.
2. Find the serial port:
   - Linux: `ls /dev/ttyUSB*` (the MicroZed usually shows two; the **higher**
     index is typically the console — confirm by reading it during boot). 115200 8N1.
   - macOS: `ls /dev/tty.usbserial*`. Windows: Device Manager COM ports.
3. Network: `net.py` auto-detects the host IP and reconfigures the board's UDP
   destination over TCP, but the board's **own** IP must be reachable. Default in
   `remote/net.py` is `ZYNQ_IP = 192.168.18.10`. Put the laptop on the same /24
   (e.g. `192.168.18.1`) or edit `ZYNQ_IP`. Verify with `ping 192.168.18.10` once
   the board is running.

## 2. Flashing — two options

### Option A (recommended for the loop): JTAG download via `xsct`

No SD swaps. After the board powers up (booting whatever is on the SD just gets
the PS/DDR initialized), `xsct` halts the cores, (re)programs the PL, and downloads
fresh ELFs. **Firmware-only** changes (chunk tuning) don't even need a PL reprogram
— just re-download core 0.

Starting script (`scripts/hitl_download.tcl`, **verify target names with `targets`**):

```tcl
connect
# --- PL (only needed when the bitstream changed) ---
targets -set -nocase -filter {name =~ "*xc7z020*" || name =~ "*PL*"}
fpga -file vivado_project/klab_project.runs/impl_1/design_1_wrapper.bit
# --- PS init (DDR/clocks). If the board already SD-booted, DDR is up and you may
#     skip this; otherwise source the exported ps7_init and call it. ---
# source vitis_workspace/klab-platform/.../ps7_init.tcl ; ps7_init ; ps7_post_config
# --- core 0 ---
targets -set -nocase -filter {name =~ "*Cortex-A9 #0*" || name =~ "*APU*0*"}
rst -processor
dow vitis_workspace/klab-firmware/build/klab-firmware.elf
con
# --- core 1 (debug console) ---
targets -set -nocase -filter {name =~ "*Cortex-A9 #1*"}
rst -processor
dow vitis_workspace/klab-firmware-core1/build/klab-firmware-core1.elf
con
```
Run with `xsct scripts/hitl_download.tcl`. First time, run `xsct`, `connect`,
`targets` interactively to learn the exact names and whether the dual-core start
handshake (core 0 writes core 1's start addr at `ARM1_BASEADDR`) needs care under
JTAG vs. just downloading+running both.

### Option B (fallback): SD card (one manual step per iteration)

`bootgen -image scripts/boot.bif -o BOOT.bin -w`, copy `BOOT.bin` to the FAT32
`Boot` partition, re-insert, power-cycle. Reliable but needs a human for the swap —
only use if JTAG download proves troublesome.

## 3. Reading the serial console (for `dump_bram`, boot/magic messages)

A small pyserial helper the agent can call (`scripts/hitl_serial.py`, suggested):

```python
import sys, time, serial
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB1"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 5
s = serial.Serial(port, 115200, timeout=0.2)
t0 = time.time()
while time.time() - t0 < secs:
    line = s.readline()
    if line:
        sys.stdout.write(line.decode(errors="replace"))
        sys.stdout.flush()
```
`python3 scripts/hitl_serial.py /dev/ttyUSB1 5` dumps 5 s of console. For a
`dump_bram` capture, trigger the command over TCP (net.py / a raw command) and read
the serial concurrently. (Serial is core 1's console; `send_message` is
non-blocking and drops under flood, so keep dumps small — the existing
`dump_bram <n>` is capped at 150 words.)

## 4. Driving `net.py` non-interactively

`net.py`'s main loop reads stdin lines, so the agent can pipe a command sequence
and capture stdout:

```bash
printf 'set_debug 1\nset_channels 0xFF\nverify_sine\nquit\n' | python3 remote/net.py
```
`verify_sine` self-times (captures ~300 packets then prints the report), so this
works headless. Parse the `VALUE CHECK` / `PACKET LOSS` / `real worst word` lines.

## 5. The test loop (agent)

1. Edit firmware/PL. **Firmware-only** (e.g. chunk size): `vitis -s
   scripts/build_vitis_project.py` (incremental, ~1–2 min). **PL change**:
   `create_vivado_project.tcl` + `build_bitstream.tcl` (~15 min), then
   `create_vitis_project.py`.
2. `xsct scripts/hitl_download.tcl` (PL reprogram only if the bitstream changed).
3. `python3 scripts/hitl_serial.py <port> 3` → confirm clean boot ("BRAM streaming
   …", no magic-fail loop).
4. `printf '…\nverify_sine\nquit\n' | python3 remote/net.py` → read the report.
5. For a `dump_bram` diagnostic: start serial capture, send the command, read.
6. Decide, iterate.

## 6. Caveats / safety

- The two cores share non-cacheable memory at `0x3F000000`; download both ELFs (or
  at least don't leave core 1 in a bad state) so the shared print/status structs are
  sane.
- After **any PL change** you must regenerate the Vitis platform (`create_` script,
  clean `vitis_workspace` first) — `build_` only recompiles apps against the stale
  platform.
- The board has **no chip attached** by default — use **debug mode** (synthetic
  sine) for `verify_sine`. `0xFF` is the failing case; `0x0F`/`0x3F` are the
  controls.
- Don't commit/push or change sample-value processing without the human's OK
  (see the data-fidelity rule in `CLAUDE.md` / `bram_burst_read_bug.md`).
- Toolchains live in `/opt/Xilinx/2025.1` on the original dev box; on the HITL
  laptop, `source <vivado>/settings64.sh` and `<vitis>/settings64.sh`.
