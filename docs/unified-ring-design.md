# Unified single-stream ring buffer (broadband + LFP in one BRAM)

Status: **design + WIP**. Branch `claude/unified-ring`, off `claude/lfp-3khz`
(which already has the OCM staging change, firmware v1.5). Goal: the PL writes
broadband **and** LFP packets into the **one** capture-BRAM ring, so the PS does a
single demux-by-magic service loop + a single DMA stream — retiring the separate
LFP BRAM (`0x84`) and the separate `lfp_stream_service`.

This is the "share the BRAM across the PL packets" architecture. It is **higher
risk** than the OCM change (real RTL surgery + a PL rebuild) and, per the perf
data, **lower marginal value once OCM staging is in** (OCM already removed the
DDR contention; merging the BRAMs only saves ~one CDMA-setup per LFP frame,
~0.6 % CPU). It is worth doing for the architectural simplicity (one ring, one
service, one DMA) — but it is a *follow-up* to validate against the OCM build,
not a prerequisite.

## Why it's tractable here

Two facts make this much smaller than feared:

1. **The magics are already distinct.** Broadband packets start with
   `0xCAFEBABE_DEADBEEF` (low word `0xDEADBEEF`); LFP packets start with
   `0xCAFEBABE_1F1FBEEF` (low word `0x1F1FBEEF`, see `lfp_dsp_block.sv`
   `LFP_MAGIC_LOW`). So both can coexist in one ring and the PS tells them apart
   by the low magic word — **no wire change**, `net.py`/ephys-socket untouched.

2. **The capture BRAM is already a magic-delimited ring** with a PL write-pointer
   (`fifo_bram_interface.sv` `packet_boundary_address`, exposed as status reg 10)
   that the PS walks (`main.c` resync + `ps_read_address`). Adding LFP packets to
   that ring is "another producer of packets," not a new mechanism.

## The one real problem: two producers, one ring writer

`fifo_bram_interface.sv` drains **one** FIFO (the broadband packetizer's 128-bit /
8-segment stream), packs the masked 16-bit segments into 32-bit BRAM words, writes
them into the ring, and advances `write_address` / `packet_boundary_address`.
Broadband enqueues ~40 entries spread over ~2975 clocks/packet; a packet is a
**contiguous run** of FIFO entries delimited by `fifo_packet_end_flag`.

The LFP engine (`lfp_dsp_block.sv`) today writes its own BRAM directly (6-word
header micro-sequence + sample packing, see the `hdr_busy`/`pack_phase` FSM). It
produces a full frame (6 + popcount(mask)*16 words, ≤134) once per `R`=10
broadband packets.

We cannot interleave LFP words *between* broadband words within a packet (the
packer's `packet_end_flag`/stash logic delimits one contiguous producer stream).
So merge at the **drain side** with a packet-level arbiter:

```
  broadband FIFO ─┐
                  ├─► [arbiter: pick source at each packet boundary] ─► ring write
  LFP FIFO ───────┘        (stash empty at boundaries -> safe to switch)
                                                          │
                              write_address / packet_boundary_address (shared)
                                                          ▼
                                            capture BRAM ring @ 0x80
```

### Arbiter rules
- A `current_source` register (BB | LFP), changed **only at packet boundaries**
  (when the just-finished packet's `packet_end` is consumed and the stash is
  empty — guaranteed at a boundary).
- The LFP engine streams a **whole frame** into the LFP FIFO, then bumps a
  `lfp_frames_ready` counter. The arbiter serves LFP next (once) only when
  `lfp_frames_ready > 0`, so the drain never underruns mid-frame. After draining
  the frame (its `packet_end`), decrement `lfp_frames_ready` and return to BB.
- Broadband has priority and is never blocked. During an LFP drain (~hundreds of
  clocks) the BB FIFO keeps filling (~0.014 entries/clk → a few entries); BB FIFO
  depth 256 absorbs it; the packer catches up after.

### LFP drain path
Feed LFP 32-bit words as a simple 1-word/clock drain into the shared BRAM-write
backend (`buffer_valid_reg` / `data_buffer_reg` / `packet_end_reg`) — do **not**
route them through the broadband 4-chunk segment packer. The two drain FSMs
(BB packer, LFP passthrough) are mutually exclusive via `current_source`, so only
one drives the backend per cycle.

### LFP engine change
`lfp_dsp_block.sv` stops driving a BRAM port; instead it emits its frame as
`(lfp_word[31:0], lfp_valid, lfp_last)` into the LFP FIFO. The existing header
micro-sequence (w0=`0x1F1FBEEF`, w1=`0xCAFEBABE`, ts, cfg, seq) and the sample
packer stay — they just target the FIFO instead of the BRAM, and `lfp_last`
asserts on the final sample word of the frame. Retire `bram_*`/`lfp_wr_addr`.

## Block design change
Retire `axi_bram_ctrl_1` + `simple_dual_port_bram_lfp` + `smartconnect_1/M02`,
and remove the `0x84000000` segments from both `processing_system7_0/Data` and
`axi_cdma_0/Data` (`design_1_bd.tcl`). Frees a BRAM controller + a SmartConnect
port + ~the LFP BRAM tiles. The capture BRAM (`0x80`) is unchanged.

## PS change (`main.c` / `network.c`)
`process_packet_from_bram()` becomes a demuxer:
- peek the 64-bit magic at `ps_read_address`;
- `high != 0xCAFEBABE` → resync (as today);
- low `0xDEADBEEF` → broadband: length `current_packet_size`, port 5000;
- low `0x1F1FBEEF` → LFP: read header word `w4` (lane_mask) → length
  `6 + popcount(mask)*16`, port 5001;
- CDMA `length` words BRAM→OCM staging (split at ring wrap), `PBUF_REF`,
  `udp_sendto` to the per-type port, advance `ps_read_address += length`.
Delete `lfp_stream_service()` and the separate LFP read pointer; the LFP packets
now flow through the one ring walk. `packets_available()` already works off the
shared write-pointer.

OCM staging is inherited from v1.5 (both buffers in low OCM). With one ring, one
broadband-sized OCM buffer suffices, but keeping the two is harmless.

## Verification
1. **Sim** (`programmable_logic/sim/`): a TB feeding the modified
   `fifo_bram_interface` both a broadband-style packet stream and LFP frames →
   assert the BRAM ring contains both packet types, correctly delimited, in order,
   with the right magics, and `packet_boundary_address` advances past each. Reuse
   the `run_fifo_bram_dualport_tb.sh` harness pattern.
2. **PL build**: `create_vivado_project.tcl` → `build_bitstream.tcl` (full, to
   avoid the OOC stale-netlist trap noted in CLAUDE.md); confirm timing closes
   @ 84 MHz and the LFP BRAM tiles are freed.
3. **HW**: `perf_reset` → stream broadband + LFP → `get_status`; expect the
   over-budget count to match or beat the OCM-only build, with LFP packets
   arriving on 5001 exactly as before (wire-compatible).

## Risk register
- Arbiter correctness (switching only at boundaries; stash empty) — the main sim
  target.
- BB FIFO not overflowing during an LFP drain — bounded above; assert in sim.
- The OOC stale-netlist trap (CLAUDE.md) — build from scratch.
- If timing doesn't close or sim finds a corner, the OCM-only v1.5 build on
  `claude/lfp-3khz` remains the validated fallback.
