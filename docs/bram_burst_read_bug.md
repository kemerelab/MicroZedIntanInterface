# BRAM burst-read corruption — problem & task brief

> **RESOLVED (fw 1.1.0.0).** The fix is **AXI CDMA**, not the chunked read this brief
> proposes: a PL master (CDMA) reads the capture BRAM and writes DDR over S_AXI_HP0,
> taking the corrupting PS M_AXI_GP master off the data path entirely. The earlier
> chunked/`ldmia` CPU reads were dead ends (still bursts over the broken GP master).
> See `docs/custom_axi_bram_ctrl_decision.md` and `firmware/src-core0/pl_dma.c`. The
> root-cause analysis below remains accurate and is kept as the record.

Status: **RESOLVED via DMA** (historical: "candidate fix in test", chunked read).
Audience: a Claude Code instance picking this up, possibly with hardware attached.

## TL;DR

When streaming the **full dual-port `0xFF`** configuration (8 streams, 150-word /
600-byte packets), a small, localized run of samples comes out as out-of-range
garbage. Smaller configs (`0x0F` 80-word, `0x3F` 115-word packets) are clean.

**Root cause (proven):** the PS core-0 reads the capture BRAM with `memcpy`, which
issues **long AXI bursts over the M_AXI_GP port**. Long GP-port bursts
intermittently corrupt one short run of beats. It is **not** the BRAM, **not**
read-during-write, **not** the clock, **not** cache.

**Quick fix (in test):** read the packet in small `memcpy` chunks
(`BRAM_READ_CHUNK_WORDS = 8` in `firmware/src-core0/main.c`), each a short burst
below the length that triggers the corruption.

**Real fix (next):** move BRAM→DDR onto a **PL DMA** (e.g. `axi_datamover`) so the
CPU GP port is off the data path entirely.

## The system (1 paragraph)

MicroZed (Zynq-7020, `xc7z020clg400-1`). PL acquires Intan RHD2000 data and writes
packets into a 64 KB capture BRAM (`0x80000000`). PS core 0 polls the BRAM, copies
each packet to a DDR buffer, and UDP-streams it (port 5000); core 1 is the serial
debug console. Host control + capture is `remote/net.py` (TCP 6000). Read
`overview.txt` and `CLAUDE.md` for the full picture and the 3-layer
register/packet contract.

## How we proved the root cause

1. **`verify_sine`** (a `net.py` command): puts the board in debug mode (synthetic
   sine, no chip needed) and compares every received 16-bit sample to an
   RTL-exact reference. At `0xFF` it reports ~3.2% of samples wrong, **out of LUT
   range**, clustered at packet **words ~116–128** (decode to cycles 26–29 →
   de-skewed channels 24–27 of each of the 8 stream blocks — the glitch bands you
   see in Open Ephys). 0% packet loss. `0x0F`/`0x3F` pass.
2. **`dump_bram <addr> <count>`** (serial command, `pl_dump_bram_data`): reads the
   same BRAM region **two ways** — `memcpy` (AXI burst) and `Xil_In32`
   (single beats) — and flags diffs. **On a STOPPED board** (no PL writing), the
   `memcpy` burst returns garbage at a short run of words while the single reads
   read the same addresses correctly. That rules out read-during-write and the
   BRAM content; it is the burst transaction itself.
3. **`dump_bram 0 10` is always clean; `dump_bram 0 20` corrupts at words 12–15.**
   So reads stay clean below ~11 words and corrupt at ≥ ~12. Only `0xFF`'s
   150-word read is long enough to reach the corrupt offset.
4. **`report_timing`** on the routed design: the BRAM read path is healthy
   (+2.357 ns data, +1.74 ns address; not false-pathed). The worst path in the
   design is the SPI COPI output at +0.133 ns, unrelated. So timing / output
   registers on the BRAM are **not** the fix.

The corrupted values (`0x0404/0408/0624/0628`, DDR halves zero) are an AXI-path
artifact, not memory content — they appeared identically on a zero-initialized
vendor `blk_mem_gen`, so don't read meaning into the values; the **positions** are
what matter.

## Dead ends — do NOT re-chase these (each was built + tested on hardware)

- **Vendor BRAM** (`blk_mem_gen` swap): same corruption → not the BRAM primitive.
- **Cache**: making the BRAM non-cacheable only *shifted* the offset by one cache
  line → cache is not the source (it reshapes the burst).
- **Clock / phase**: 175 MHz did **not** fix it and *spread* the corruption onto
  the header timestamp (causing `net.py` errors that the firmware's magic-only
  validation doesn't see). Reverted to 131.25 MHz.
- **Write-pointer CDC deglitch** and **read-pointer guard band**: each only moved
  the corruption. Kept as harmless hygiene.
- **Read-during-write / BRAM timing margin / output register**: ruled out by the
  STOPPED-board reproduction and the +2.3 ns timing slack.

## Current state of the branch (`claude/dual-port`)

- Firmware read is a **chunked `memcpy`** (`BRAM_READ_CHUNK_WORDS = 8`).
- AXI clock at **131.25 MHz** (clean timing, WNS ~+0.13).
- Custom `simple_dual_port_bram` (vendor swap reverted; it's not at fault).
- **Stop/restart fix**: `handle_enable_streaming` scans back from the write pointer
  to the real `0xDEADBEEF/0xCAFEBABE` magic to re-sync `ps_read` (the datapath's
  write_address/FIFO are only cleared by the hardware reset, so a stop mid-packet
  used to leave `ps_read` a few words off the boundary → endless
  `Magic validation failed (0x..11CE8)` loop).
- Diagnostic kept: `dump_bram` shows `burst | single | flag` and a diff count.
- Pre-built image: `blobs/BOOT.bin`.

## Remaining task list

1. **Verify the chunked fix** on hardware: `verify_sine 0xFF` → expect
   `VALUE CHECK: PASS` and `PACKET LOSS: PASS`. Check Open Ephys glitch bands gone.
   Check start/stop/restart is clean.
2. **Tune `BRAM_READ_CHUNK_WORDS`**: if clean + no loss, raise it (10, then higher)
   re-checking `verify_sine` each step to find the largest safe/ fastest burst; if
   any diffs survive, drop to 4. Firmware-only, minutes per iteration.
3. **DMA (real fix)**: add an `axi_datamover`/`axi_dma` PL master to copy
   BRAM→DDR with controlled bursts, freeing core 0. Verify it doesn't reintroduce
   the GP-port pattern (it reads through the same `axi_bram_ctrl` but as a
   different, well-formed master). Bitstream + firmware change.
4. Re-confirm `0x0F`/`0x3F` still pass (no regression) and the plugin
   (`ephys-socket`, branch `claude/dual-port-plugin`) streams cleanly.

## Key tools

- `remote/net.py` → `verify_sine [ce=FF] [n]` (the ground-truth value checker),
  `set_channels`, `set_debug`, `start`/`stop`, `dump_bram <word> <count>`.
- Serial debug console (core 1) shows firmware messages and `dump_bram` output.
- Build: `scripts/create_vivado_project.tcl` + `scripts/build_bitstream.tcl` (PL),
  `scripts/create_vitis_project.py` (platform + both apps; clean `vitis_workspace`
  first), `scripts/build_vitis_project.py` (firmware-only incremental),
  `bootgen -image scripts/boot.bif -o BOOT.bin -w`.

## Hard rule

**Data fidelity:** never alter acquired sample values (no detrend/baseline/filter/
offset) without explicit approval. Fix display via scale factors / viewer ranges.
