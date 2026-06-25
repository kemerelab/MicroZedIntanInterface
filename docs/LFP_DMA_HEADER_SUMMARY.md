# LFP PL→PS DMA-correct path (PL-built header + mask mirror)

Branch: `claude/lfp-3khz`. Makes the LFP (Tier-1) PL→PS path fully DMA-correct: the **PL
builds the complete LFP wire packet** (header + samples) in the LFP output BRAM, and the PS
just **CDMAs it straight into a pbuf and sends it** — no `Xil_In32` sample loop, no PS-side
header math. The **LFP lane mask mirrors the broadband channel-enable mask**.

## PL-built LFP packet (in `lfp_dsp_block.sv`)

LFP output BRAM (`0x84000000`) layout, per frame = one complete wire packet:
`[6 header words | decimated sample words]`.

Header (6 words, little-endian on the wire):

| word | field | value |
|-----:|-------|-------|
| 0 | magic low  | `0x1F1FBEEF` |
| 1 | magic high | `0xCAFEBABE` |
| 2 | timestamp low  | `ts[31:0]` |
| 3 | timestamp high | `ts[63:32]` |
| 4 | cfg | `lane_mask | (decim_R<<8) | (num_taps<<16) | (overrun<<24)` |
| 5 | seq | PL-maintained LFP frame counter, ++ per emitted frame |

A small header-write micro-sequence in `lfp_dsp_block.sv`, triggered by the engine's
`frame_tick` (= the decimation tick / `start_pass`, exposed from `lfp_halfband.sv` and
`lfp_fir_decimator.sv`), writes the 6 header words at the frame base, advancing the BRAM write
pointer; the decimated samples then pack (signed → offset-binary, 2×16-bit/word) immediately
after. `frame_tick` leads the frame's first `out_valid` by ~`num_taps` clk (≫ 6), so header
and sample writes never race. `lfp_wr_addr` (PS read pointer, STATUS_REG_13) only advances past
a full `[header|samples]` frame, so the PS never reads a torn frame.

## Timestamp semantics

The decimating filters are just FIR: when one emits an output there is exactly one real,
already-acquired broadband sample that is the **newest input in that output's support window**,
and the stamp is that sample's master count.

**`ts` = the master count of the newest broadband sample in this output's decimation window**
— i.e. the most recent broadband packet clocked into the filter bank for this output, the
*same* counter the broadband header stamps (word 1). For frame `m` at total decimation `R` it
is broadband packet `R·m + (R−1)`. Causal, monotonic, `R` apart; an absolute master timestamp,
not a frame index.

Accounting through the CIC(/5)→halfband(/2)=/10 cascade:
- CIC emits on every 5th broadband `packet_tick` → CIC output `k` ↔ newest broadband packet `5k+4`.
- halfband emits on every 2nd CIC output → halfband output `m` ↔ CIC output `2m+1` ↔ newest
  broadband packet `5·(2m+1)+4 = 10m+9`.

Implementation: `data_generator_core.sv` exposes the live master `timestamp` as
`dsp_master_timestamp` (it has already incremented to "just-finished packet + 1" on the edge
`dsp_packet_tick` rises, so the finished packet's count is `dsp_master_timestamp − 1`).
`lfp_dsp_block` updates `ts_ingest` to that on **every** `dsp_packet_tick`, and snapshots it
into the header on `frame_tick` (the halfband decimation tick). The cascade latency (CIC comb
pass + glue replay, a few hundred clk) is far less than one ~2800-clk broadband packet, so no
new `packet_tick` arrives in between and the snapshot is exactly `10m+9`. (A frame-behind slip
under load would set `compute_overrun` → header word 4 bit 24.)

**This marks the newest *input* sample, not the instant the LFP value represents** — a
linear-phase anti-alias filter's output is centered earlier by the chain's group delay (a
fixed, known number of broadband samples). Subtract the group delay to align an LFP sample
with the broadband stream's *content*; use the stamp directly to know which input data has
been folded in. No RTL change for correctness — the stamp is the causal window-edge count by
design.

## LFP lane mask = broadband mask (single source of truth)

`data_generator_core.sv` exposes `channel_enable_reg` as `dsp_channel_enable`; `lfp_dsp_block`
drives the engine `lane_mask` from it (not from `lfp_cfg[15:8]`). The LFP filters exactly the
broadband-enabled lanes — pick streams with `set_channels` (broadband mask). `lfp_cfg[15:8]`
and `CMD_LFP_SET_CHANNELS` are deprecated (firmware accepts-and-ignores; net.py drops the mask
arg). `get_status` reports `lfp_lane_mask = pl_get_current_channel_enable()`.

## BD change (`design_1_bd.tcl`)

Added `0x84000000` (`axi_bram_ctrl_1/S_AXI/Mem0`) into `axi_cdma_0/Data`, mirroring the
existing `0x80000000` capture-BRAM CDMA assignment (no `exclude_bd_addr_seg` for it).
`smartconnect_1` already routes the CDMA (SI) to `axi_bram_ctrl_1` (M02). The CDMA Data space
now holds `0x80000000`, `0x84000000`, and DDR (HP0).

## Firmware (`network.c`, `pl_dma.c`, `pl_control.c`)

- `pl_dma_read_addr(dst, src_addr, n)` — generalized CDMA read from an arbitrary BRAM source
  byte address (the LFP BRAM); `pl_dma_read_bram` now wraps it.
- A dedicated non-cacheable LFP staging buffer (`pl_dma_lfp_staging`, 1 MB-aligned) so the LFP
  CDMA + zero-copy pbuf never clobber the broadband staging buffer.
- `lfp_stream_service` rewritten: on a complete frame (PL write pointer vs read pointer), CDMA
  the whole `[header|samples]` frame from the LFP BRAM into the LFP staging buffer (split at
  the ring wrap), then `udp_sendto` a `PBUF_REF`. **The PS-side header construction
  (`lfp_pktbuf[0..5]`) and the `Xil_In32` sample loop are deleted.**
- `pl_lfp_set_config` drops the `lane_mask` param (writes 0 to `lfp_cfg[15:8]`, which the PL
  ignores). `CMD_LFP_SET_CHANNELS` accepts-and-ignores.
- `status_response_t` size unchanged (168 B) — `_Static_assert` unchanged.

## `check_dma` result

`bash scripts/check_dma.sh firmware` → **PASS** (6 exempt). The LFP `Xil_In32` loop is **gone**
(now CDMA). The remaining single-beat sites are annotated `// DMA-EXEMPT`: the magic/resync
2-word peeks (`main.c`), the `BRAM_READ_SINGLE` reference reader (`main.c`), and the `dump_bram`
debug single-beat reference reader (`pl_control.c`).

## Sim

All LFP TBs pass under xsim (Vivado 2025.1):
- `lfp_dsp_block_tb` — PASS (864 BRAM words: 16 frames × (6 header + 48 sample words);
  verifies the PL-built header, timestamp, seq, and the broadband-mask-driven lane mask).
- `lfp_halfband_tb`, `lfp_fir_decimator_tb`, `cic_decimator_tb`, `cic_chain_tb` — PASS
  (new `frame_tick` output, engines unchanged otherwise).
- `data_generator` aux/chirp/dualport-dropout TBs — PASS (new `dsp_master_timestamp` /
  `dsp_channel_enable` taps wired; dualport runner fixed to compile the LFP sources).

## Build (Vivado 2025.1, `xc7z020clg400-1`, fresh create → bitstream)

BD changed → full `create_vivado_project.tcl` → `build_bitstream.tcl`. `write_bitstream
completed successfully`; BD validated + synthesized with 0 errors (no critical warning mentions
any new LFP signal).

**Timing — all constraints met** (`design_1_wrapper_timing_summary_routed.rpt`):

| clock | freq | period | setup WNS | hold WHS | failing |
|-------|-----:|-------:|----------:|---------:|--------:|
| clk_out1 (84 MHz data path) | 84.000 MHz | 11.905 ns | **+0.714 ns** | +0.022 ns | 0 |
| clk_out2 (AXI/GP) | 131.250 MHz | 7.619 ns | +0.814 ns | +0.050 ns | 0 |

Overall WNS +0.714 / TNS 0.000 / WHS +0.022, 0 failing endpoints (96289). The 84 MHz path stayed
positive (new header FSM / 64-bit timestamp latches / seq counter are registered and off the
critical path) — better than the ~0.45 ns the task flagged.

**Utilization** (`design_1_wrapper_utilization_placed.rpt`):

| resource | used | avail | % |
|----------|-----:|------:|--:|
| Slice LUTs | 17029 | 53200 | 32.0 |
| Slice Registers | 33664 | 106400 | 31.6 |
| Block RAM Tile | 46.5 | 140 | 33.2 |
| DSPs | 1 | 220 | 0.45 |

BRAM unchanged by the header-in-PL change (the LFP output BRAM was already allocated); the BD
CDMA address add is routing-only.

## Firmware build (Vitis 2025.1, -O3, both cores)

Fresh `create_vitis_project.py` from the new `.xsa`. Platform + both cores all report
**`Build Finished successfully`** (0 errors, 0 warnings on the changed files). The
`_Static_assert(sizeof(status_response_t) == 168)` **passes** (main.c compiled clean at -O3;
struct size unchanged — `lfp_lane_mask` is still a `uint8_t`, only its source changed to the
broadband mask). ELFs: `klab-firmware.elf` (core 0, ~1.13 MB), `klab-firmware-core1.elf`
(core 1, ~0.30 MB).

## check_dma

`bash scripts/check_dma.sh firmware` → **PASS, exit 0** (0 violations, 6 exempt). The LFP
`Xil_In32` sample loop is gone (now CDMA); the 6 exempt sites are the magic/resync 2-word
peeks + the `BRAM_READ_SINGLE` reference reader (`main.c`) and the `dump_bram` debug
single-beat reader (`pl_control.c`), each annotated `// DMA-EXEMPT: <reason>`.

## Note on BOOT.bin

No new `blobs/BOOT.bin` is committed: this change is **HW-UNVALIDATED** (built + sim-verified
here, not run on a board). The existing committed `blobs/BOOT.bin` is left untouched. A test
image can be bootgen'd from the fresh bitstream + ELFs if needed; mark any such image
HW-UNVALIDATED.

## HW test steps (not run here — HW-UNVALIDATED)

1. Flash the fresh `BOOT.bin`, connect, send one `get_status` to prime (DDR-remanence guard).
2. `set_channels 0x05` (or any mask) → `lfp_config cic` → `lfp_on` → `start`.
3. `lfp_recv 200`: confirm magic, `lane_mask` == the broadband mask, `seq` monotonic, `ts`
   advancing by the decimation stride, and sample count == `popcount(mask)*32`.
4. `get_status`: `lfp_lane_mask` == broadband `channel_enable`; `lfp_overrun` == no.
5. Cross-check an LFP sample's `ts` against the broadband stream's timestamp for alignment.
