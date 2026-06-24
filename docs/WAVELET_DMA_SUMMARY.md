# Tier-3 wavelet result transfer: single-beat `Xil_In32` -> AXI CDMA

Branch: `claude/tier3-wavelet`. This change switches the wavelet scalogram monitor's
results-BRAM read from a single-beat CPU `Xil_In32` loop to **AXI CDMA**, the same
DMA path the broadband capture already uses.

> **HW status: UNVALIDATED.** No board was available. The change builds clean (PL bitstream
> + firmware) and the BD address map is verified in the build log, but whether the CDMA
> actually reaches 0x90000000 and `XAxiCdma_IsBusy` clears on real hardware is **pending the
> user + a board**. See "HW test to run" below.

## Root cause (why the old code was single-beat / why the STFT branch's CDMA "hung")

The wavelet results BRAM is a dual-port BRAM: the PL writes one port
(`data_generator/WAV_BRAM`), the PS reads the other through an AXI BRAM controller
(`axi_bram_ctrl_2`, the 0x90000000 results BRAM).

In `programmable_logic/block_design/design_1_bd.tcl`:

- `smartconnect_1` already routed the CDMA master to that BRAM controller:
  `axi_cdma_0/M_AXI -> smartconnect_1/S01 -> M03 -> axi_bram_ctrl_2/S_AXI`. **The physical
  path existed.**
- BUT the CDMA's own address space (`axi_cdma_0/Data`) had only two segments assigned:
  `0x80000000` (capture BRAM, `axi_bram_ctrl_0`) and the 1 GB DDR (`S_AXI_HP0`). There was
  **no** segment for `0x90000000`.
- A CDMA read of 0x90000000 therefore decoded to nothing, the AXI read transaction never
  completed, and `XAxiCdma_IsBusy()` spun forever. That is exactly the STFT-branch symptom
  ("CDMA read from a results BRAM hangs") — it was a missing address assignment, not a CDMA
  or BRAM limitation. (The broadband CDMA worked only because 0x80000000 *was* in
  `axi_cdma_0/Data`.) Corroborated by `docs/dma_research_notes.md`: "the BRAM controller's
  S_AXI must be reachable by the CDMA master ... 'CDMA can't reach BRAM' is the top forum
  failure," and the CDMA M_AXI address map must include the BRAM segment.

## The BD change

`programmable_logic/block_design/design_1_bd.tcl`, in "Create address segments":

```tcl
# Tier-3 wavelet results BRAM (0x90000000) into the CDMA master address space too.
assign_bd_address -offset 0x90000000 -range 0x00010000 \
  -target_address_space [get_bd_addr_spaces axi_cdma_0/Data] \
  [get_bd_addr_segs axi_bram_ctrl_2/S_AXI/Mem0] -force
```

Plus an explicit exclusion to keep the CDMA address space clean and silence a
`BD 41-1356` unassigned-segment critical warning (the LFP BRAM is reachable through the same
crossbar but uses PS GP single-beat reads, not the CDMA):

```tcl
exclude_bd_addr_seg -target_address_space [get_bd_addr_spaces axi_cdma_0/Data] \
  [get_bd_addr_segs axi_bram_ctrl_1/S_AXI/Mem0]
```

**SmartConnect connectivity:** `smartconnect_1` has no sparse-connectivity restriction set
(default full crossbar), so connectivity follows the assigned address map — adding the
0x90000000 segment to `axi_cdma_0/Data` is exactly what enables the S01->M03 route. No extra
connectivity property was needed.

The `create_vivado_project.tcl` build log confirms the resulting CDMA master address space:

```
Slave segment '/axi_bram_ctrl_0/S_AXI/Mem0' ... '/axi_cdma_0/Data' at <0x8000_0000 [ 64K ]>.
Slave segment '/axi_bram_ctrl_2/S_AXI/Mem0' ... '/axi_cdma_0/Data' at <0x9000_0000 [ 64K ]>.  <-- the fix
Slave segment '/processing_system7_0/S_AXI_HP0/HP0_DDR_LOWOCM' ... '/axi_cdma_0/Data' at <0x0000_0000 [ 1G ]>.
```

The PS read view (`processing_system7_0/Data` @ 0x90000000) is unchanged.

## The firmware change

`firmware/src-core0/pl_dma.{c,h}`:
- Generalized to `pl_dma_read_addr(uint32_t *dst, uintptr_t src_addr, uint32_t n_words)` — a
  CDMA read from an arbitrary source AXI address into the non-cacheable DDR staging buffer.
- `pl_dma_read_bram(dst, bram_word_addr, n_words)` is now a thin wrapper over it (capture-BRAM
  path unchanged in behavior).

`firmware/src-core0/network.c` `wav_stream_service()`:
- On each scalogram-column advance (STATUS_REG_14), one CDMA transfer copies the **full**
  results surface (`WAV_K * WAV_N_SCALES * 2` = 2048 words = 8 KB) from `WAV_BRAM_BASE_ADDR`
  into a non-cacheable DDR staging sub-region, then the active `(lane, scale<nscales)`
  sub-region is repacked into the UDP packet from staging. The wire format is unchanged.
- The single-beat `Xil_In32` loop is removed.
- The `frame_seq` torn-frame re-check is kept (re-read the column counter after the read; if it
  advanced, skip and retry on the next advance — `wav_last_seq` is left unchanged on a torn or
  DMA-error column, so nothing is dropped silently).
- Added a `wav_dma_errors` diagnostic counter (not on the wire — like the broadband
  `dma_errors`, it is not a host-configurable setting, so the `status_response_t` contract and
  its `_Static_assert` are untouched).

**Staging buffer:** the wavelet CDMA writes into a 512 KB-in sub-region (`offset 0x80000`) of
the existing 1 MB non-cacheable `pl_dma_staging` array (see `pl_dma.h`). It is disjoint from
the broadband packet region at the front of the buffer (max 150 words), and the core-0 loop is
single-threaded (broadband send completes before its function returns), so the two paths never
alias even with an in-flight broadband pbuf. No new linker section or TLB entry is needed — the
sub-region is already inside the section `pl_dma_init()` marks `NORM_NONCACHE`.

## Build / timing / utilization

- Built from scratch (BD changed): clean `create_vivado_project.tcl` (regenerates all OOC
  runs — avoids the OOC-stale-netlist gotcha in CLAUDE.md / PHASE_B_SUMMARY §4) then
  `build_bitstream.tcl`. Vivado 2025.1, part `xc7z020clg400-1`. Bitstream
  (`design_1_wrapper.bit`) + `klab_project.xsa` both written successfully; post-route DRC 0
  errors.
- **Setup timing @ 84 MHz: MET. WNS = +0.401 ns, 0 failing setup endpoints / 85207.** This is
  the metric the task asked for (WNS >= 0 @ 84 MHz) — it is met, and is consistent with the
  expectation that this BD change adds no logic to the 84 MHz wavelet datapath (it only adds an
  address-decode segment in the 175 MHz AXI fabric for a crossbar path that already existed).
  (Baseline PHASE_B §4 was WNS +0.286 ns; the +0.401 here is just run-to-run P&R variance.)
- **Hold: one -0.004 ns violation (WHS = -0.004 ns, 1 failing endpoint).** It is NOT in the
  wavelet/CDMA logic. The path is `spi_lvds_0_cipo0_p` (the Intan SPI CIPO0 LVDS **input
  port**) -> `data_generator/.../cipo0_4x_oversampled_reg[39]/D` — an I/O input-capture path
  governed by the `set_input_delay -min 0.5` on the CIPO pins (`intan_io.xdc`), 75% route
  delay. `docs/routing_report.md` already flags the CIPO LVDS inputs as the route-dominated
  paths, and a prior build landed at WHS +0.013 ns — this run's -4 ps is the same path moving
  ~17 ps with placement variance, unrelated to the CDMA change. These CIPO capture paths are
  what the runtime 4x-oversample + programmable phase selector exists to absorb; a -4 ps static
  hold on this I/O path does not gate the broadband/wavelet datapath. (If a hold-clean bitstream
  is wanted, re-run impl — it is P&R-variance — or tighten the CIPO `set_input_delay -min`.)
- **Utilization (post-route, xc7z020):** LUTs 17065/53200 (32.1%), FF 25503/106400 (24.0%),
  BRAM 90.5/140 (64.6%), DSP 4/220 (1.8%), IOB 32/125 (25.6%), BUFG 4/32. The BD change added
  no DSP/BRAM/logic (the results BRAM `axi_bram_ctrl_2` already existed); utilization matches
  the pre-change design within noise.
- **Firmware: rebuilt at -O3 via `create_vitis_project.py` (both cores) — SUCCESS, 0 errors.**
  `klab-firmware.elf` (core 0, 1.14 MB) + `klab-firmware-core1.elf` (304 KB) + `fsbl.elf`.
  Verified the changed symbols linked into the core-0 ELF: `pl_dma_read_addr` (new),
  `pl_dma_read_bram` (now a wrapper), `wav_dma_errors` (new), `wav_stream_service` (rewritten).
  The `_Static_assert` on `sizeof(status_response_t)` (main.c) compiled clean, so the
  PS<->host wire contract is unchanged.

## HW test to run (pending user + board)

1. Boot the new bitstream + firmware (bootgen). Watch the core-0 UART at startup:
   `pl_dma_init()` runs a CDMA self-test (BRAM[0x80000000]->staging); it should print
   `CDMA: ready ...` then `CDMA: self-test OK`. (That self-test reads the *capture* BRAM,
   which already worked — it confirms the CDMA control/data path is alive.)
2. From the host (`remote/net.py`): enable the wavelet engine, set params, push coefficients,
   and start streaming. Confirm UDP packets arrive on port 5004 (the scalogram monitor) and
   that the surface advances (the column index in the packet header increments).
3. **The actual thing this change validates:** the wavelet CDMA read of 0x90000000 must
   complete — i.e. `wav_stream_service` must NOT hang and `wav_dma_errors` must stay 0.
   If `XAxiCdma_IsBusy` ever spun, `pl_dma_read_addr` would hit its guard timeout, return
   -3, and `wav_dma_errors` would increment (packets would stop). So: stream for a while and
   confirm packets keep flowing and `wav_dma_errors == 0`. (Surface `wav_dma_errors` via the
   debug console / a temporary status print if needed — it is intentionally not on the wire.)
4. Sanity-check the packet contents against the single-beat reference if desired: temporarily
   flip back to an `Xil_In32` read and diff one surface — the bytes must be identical.
