# Merge of `main` (Phase A) into `claude/tier3-wavelet`

Date: 2026-06-24
Merge: `origin/main` @ `0176066` (Phase A: 3 kHz LFP via CIC⁴÷5 + comp-FIR
halfband, analytic chirp NCO, `lfp_sweep`, firmware v1.3) into
`claude/tier3-wavelet` @ `e165dff` (Tier-3 Morse-wavelet scalogram engine +
its AXI-CDMA result path).

Both feature sets survive. The register/CMD allocations were deliberately
non-colliding, so most of the heavy files auto-merged; only four files had
textual conflicts and all were purely additive.

## Files that conflicted vs. auto-merged

`git merge` reported content conflicts in **4** files (not the 7 the brief
anticipated):

| File | Result |
|------|--------|
| `firmware/include/main.h` | CONFLICT — resolved (status struct union) |
| `firmware/src-core0/main.c` | CONFLICT — resolved (`_Static_assert` size) |
| `firmware/src-core0/network.c` | CONFLICT — resolved (`collect_status_data` union) |
| `remote/net.py` | CONFLICT — resolved (len, unpack offsets, dict, prints, help) |
| `firmware/src-core0/pl_control.c` | auto-merged cleanly (helper fns non-overlapping) |
| `programmable_logic/src/lfp_dsp_block.sv` | auto-merged cleanly (CIC datapath + tap) |
| `.gitignore` | auto-merged cleanly (union) |

The command-dispatch `case`s in `network.c` and the CMD defs / designers in
`net.py` were on disjoint line ranges, so git folded them automatically — they
were verified by grep afterwards, not hand-merged.

## Resolutions

### `status_response_t` union (the critical one)

Both new field groups were kept. They are non-overlapping appends to a shared
160-byte base:

- **chirp** (from main, +8 bytes): `chirp_mode`, `chirp_stride`,
  `chirp_fspan` (u16), `chirp_rate` (u16), `chirp_reserved[2]`.
- **wavelet** (from tier3, +20 bytes): `wav_enable`, `wav_n_octaves`,
  `wav_n_voices`, `wav_n_taps`, `wav_K`, `wav_overrun`, `wav_busy`,
  `wav_reserved`, `wav_gain` (u32), `wav_frame_seq` (u32),
  `wav_packets_sent` (u32).

Chosen field order in the struct: **base → LFP → chirp → wavelet**. The struct
is `__attribute__((packed))`, so no padding:

```
sizeof(status_response_t) = 160 (base, incl. LFP) + 8 (chirp) + 20 (wav) = 188
```

(main asserted 168 = 160 + 8; tier3 asserted 180 = 160 + 20. The union is 188,
larger than both — computed with a packed-struct gcc test, not guessed.)

Updated to match the 188-byte wire layout:
- `firmware/src-core0/main.c`: `_Static_assert(sizeof(status_response_t) == 188, …)`.
- `remote/net.py` `get_status`: length check `!= 188`; chirp unpacked from
  `data[160:168]`, wavelet 8×u8 from `data[168:176]` and 3×u32 from
  `data[176:188]`. (The auto-merge had left BOTH chirp and wavelet reading
  `data[160:168]` — the wavelet offsets were corrected to follow chirp.)
- `net.py` status dict + `print_status` carry both groups.

### Register map / CMD ids — no renumbering

Already non-colliding; `PL_N_CTRL_REGS = 32` (tier3's superset value) covers all:

- chirp @ `CTRL_REG_3`
- LFP @ regs **25–27** (main): cfg / coef / strobe
- wavelet @ regs **28–31** (tier3): cfg / gain / data / strobe

`axi_lite_registers.v` already had `N_CTRL = 32` and `N_STATUS = 15` (status
regs 0–14, with **STATUS_REG_13 = LFP** (main) and **STATUS_REG_14 = wavelet**
(tier3)). `STATUS_REG_BASE = PL_N_CTRL_REGS*4`. No change needed — tier3's PL
allocation was already a superset of main's.

CMD ids (all kept, no collision): chirp `0x77`; LFP `0x80–0x83`;
wavelet `0x88–0x8C`; UDP_BENCH `0x90`.

### `network.c`

`collect_status_data` now populates both the chirp fields (from
`chirp_cfg_*` tracking globals) and the wavelet fields (from `wav_cfg_*` +
`pl_wav_read_status()`), in struct order. All command `case`s
(chirp `pl_set_chirp`, LFP `pl_lfp_*`, wavelet `pl_wav_*`) plus
`wav_stream_service` (CDMA result path) are present.

### `pl_control.c`

Auto-merged union of helpers verified present: chirp (`pl_set_chirp`,
`pl_get_chirp_cfg`), LFP/CIC (`pl_lfp_set_config`, `pl_lfp_coef_*`,
`pl_lfp_read_status`), wavelet (`pl_wav_set_enable/_set_params`,
`pl_wav_sel/_coef/_hb_*`, `pl_wav_read_status`).

### `net.py`

Union of CMD defs and of the designers: from main `configure_chirp`,
`configure_lfp` (with `datapath=cic|fir`), `design_cic_comp_fir`,
`measure_lfp_response` (`lfp_sweep`); from tier3 `design_wavelet_bank`,
`configure_wavelet`, `receive_wavelet`. The interactive menu help shows both
LFP and wavelet lines. The `lfp_config` help string was reconciled to main's
`[mask] [cic|fir] [taps]` form (the surviving handler uses that signature; the
stale tier3 `[R] [taps] [cutoff]` text in the detailed-help block was updated
too). `python3 -m py_compile remote/net.py` passes.

### `lfp_dsp_block.sv` — the actual integration

Kept main's **CIC datapath** (`USE_CIC` default **1**, instantiating
`cic_decimator` → `cic_to_halfband` glue → `lfp_halfband`) AND tier3's
**wavelet tap** (`lfp_out_valid/_channel/_data/_frame_start`). Both the
BRAM packer and the tap are driven from the same shared `out_*` wires, which
under `USE_CIC=1` come from `lfp_halfband` (the CIC chain's final stage).

Net effect: **the Tier-3 wavelet engine is now fed by the 3 kHz CIC-decimated
LFP stream.** Verified the tap exists and is driven on the CIC path
(`lfp_dsp_block.sv` lines ~271–274), and that `data_generator_wrapper.v` wires
`lfp_dsp_inst.lfp_out_*` → `wav_dsp_inst.lfp_out_*` (lines ~231–249).

### Halfband decision — keep BOTH (do not share)

`lfp_halfband.sv` (main) and `wavelet_halfband.sv` (tier3) are genuinely
different modules and are both kept:
- `lfp_halfband.sv`: time-shared streaming ÷2 FIR with a `sample_valid` /
  `packet_tick` handshake and an *internal* shared-ring delay line; serves the
  CIC droop-compensation stage (6 kHz → 3 kHz).
- `wavelet_halfband.sv`: a generic combinational ÷2 FIR primitive over a
  *caller-supplied* delay window; the wavelet engine owns the octave-cascade
  scheduling.

Different interfaces, different state ownership, different role → NOT shared.

### `.gitignore`

Union (Vivado/Vitis artifacts, `lfp_*.hex`, `wav_*.hex`, `__pycache__/`, etc.).

### `blobs/BOOT.bin` — intentionally NOT updated

The merge tried to bring in main's `BOOT.bin` (4,455,828 B). Per the standing
"never commit an HW-unvalidated BOOT.bin" rule, it was restored to tier3's
tracked version (4,455,508 B) and is **not** part of the merge commit. A fresh
bootable image must be regenerated and HW-validated separately.

## Verification

Sim (xsim, Vivado 2025.1) — all PASS:

| TB | Result |
|----|--------|
| `run_wavelet_tb.sh` (bit-accuracy) | PASS — checked=128 errors=0 overrun=0 frame_seq=256 (**bit-exact, unchanged by merge**) |
| `run_cic_tb.sh` | PASS — 3072 outputs, 24 frames |
| `run_cic_chain_tb.sh` | PASS — chain ÷10, 2560 outs, no overrun |
| `run_halfband_tb.sh` | PASS — 3840 outputs, 40 frames |
| `run_lfp_block_tb.sh` | PASS — 768 BRAM words checked |

Struct size: a packed-struct gcc compile confirms
`sizeof(status_response_t) == 188`, matching the `_Static_assert` and net.py.

### PL build (Vivado 2025.1, fresh `create_vivado_project.tcl` → `build_bitstream.tcl`)

Full rebuild from a clean project (BD now includes the Tier-3 wavelet results
BRAM `axi_bram_ctrl_2 @0x90000000` + `axi_cdma_0` + the CIC LFP datapath).

- Synthesis: **0 errors, 0 critical warnings** (all merge RTL —
  `cic_decimator.sv`, `cic_to_halfband.sv`, `lfp_halfband.sv`,
  `wavelet_*`, the integrated `lfp_dsp_block.sv` — synthesized clean).
- Routed timing (`design_1_wrapper_timing_summary_routed.rpt`):
  **"All user specified timing constraints are met."**
  - Design: **WNS = +0.140 ns**, TNS = 0.000 (0/111341 failing);
    **WHS = +0.050 ns**, THS = 0.000 (0/111341 failing); WPWS = +2.560.
  - **84 MHz data path** (`clk_out2_…_84M`): **WNS = +0.140 ns ≥ 0** ✓,
    WHS = +0.051 ns. (131.25 MHz AXI domain `clk_out1`: WNS +0.322, WHS +0.050.)
  - No CIPO0 LVDS hold blip present in the routed report this build (the
    transient negative WHS seen mid-routing converged out; hold is clean).
- Utilization (xc7z020 `-1`): LUT **21126/53200 = 39.7%**,
  FF **38439/106400 = 36.1%**, BRAM **71/140 = 50.7%** (68×RAMB36 + 6×RAMB18:
  capture + LFP-out + wavelet-results BRAMs), DSP **4/220 = 1.8%**.
- Outputs: `vivado_project/klab_project.runs/impl_1/design_1_wrapper.bit`
  and `vivado_project/klab_project.xsa` written.

### Vitis firmware build (2025.1, fresh `create_vitis_project.py`, -O3, both cores)

`_Static_assert(sizeof(status_response_t) == 188, …)` — confirms the
struct/size reconciliation is self-consistent (it would be a compile error
otherwise).

Result: **3× "Build Finished successfully" (platform + core0 + core1), 0
errors.** Both ELFs produced:
- `vitis_workspace/klab-firmware/build/klab-firmware.elf` (core 0, 1,140,264 B)
- `vitis_workspace/klab-firmware-core1/build/klab-firmware-core1.elf` (core 1, 304,020 B)

Because core0's `main.c` (which holds the `_Static_assert`) compiled into its
ELF with no errors, the **188-byte struct size is confirmed** end-to-end
(firmware 188 ↔ `_Static_assert` 188 ↔ net.py length/offsets 188).

### One benign pre-existing warning (not introduced by the merge)

`led_status_controller` consumes only the lower 13 status regs (416 bits) while
`data_generator` now exports 15 (480 bits) → BD width-mismatch
CRITICAL_WARNING "only lower order bits connected". This predates the merge
(tier3 already set `N_STATUS = 15`); the LED peripheral simply doesn't display
the LFP/wavelet status regs. Functionally harmless.
