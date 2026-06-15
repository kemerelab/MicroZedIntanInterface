# Roadmap — toward a feature-complete Intan acquisition system

Working TODO list for the MicroZed/Intan DAQ. Grouped into epics; check items off as
they land. Chip-feature details should be confirmed against the datasheets in `docs/`
(`Intan_RHD2000_series_datasheet.pdf`, `Intan_RHD2164_datasheet.pdf`) before implementing.

Remember the three-layer contract: every register/packet change touches
`programmable_logic/` (PL), `firmware/include/main.h` + `pl_control.c` (PS), and
`remote/net.py` (host). See CLAUDE.md.

---

## A. Intan chip feature-completeness (expose RHD2000 capabilities)

- [x] **Fast settle — software trigger** — amp fast settle = write reg 0 bit 5: `0x80FE` on,
      `0x80DE` off (datasheet pulse ~2.5/fH). Optional DSP-reset variant = OR the CONVERT LSB
      ("bit H") to reset the digital high-pass filter. *(top 5)*
      **Done 2026-06-11** (`claude/aux-seq-v2`: `override_layer.sv` + `CMD_SET_FAST_SETTLE`;
      pulse duration is host policy). Sim-verified; needs on-hardware validation.
- [x] **Fast settle — GPIO trigger** — software-selectable `digital_in_0[*]` pin, edge-triggered
      injection of `0x80FE`/`0x80DE` into one aux command slot (Intan-style: preserves throughput
      except on transition packets). Refs: `data_generator_core.sv:273,312`; Intan
      [`rhythm/main.v`](https://github.com/open-ephys/rhythm/blob/master/main.v)`:750-767,1028-1030`.
      **Done 2026-06-11**: pin software-selectable (`fast_settle gpio <pin>`), level sampled
      once per packet, edge → one injection packet each way; DSP bit-H independent (sw or pin).
      **RESOLVED (was OPEN):** the injection *replaces Slot 1's command* on transition packets
      (the design doc's Slot-1-only invariant) — Slot 1 is the real-time-control slot whose
      default program is just a Reg-3 carrier, so Slots 2/3 (accel, housekeeping) are never
      perturbed; the only loss is one digout refresh on the transition packet.
- [x] **Digital output (`auxout`) — GPIO mirror** — drive the chip's `auxout` pin from a
      software-selectable controller GPIO in real time (Slot 1 Reg-3 override; confirmed Intan
      `main.v:1252` routes `TTL_in[ch]` → digout bit). Pairs with the Reg-3 shadow.
      **Done 2026-06-11**: Reg-3 shadow (host-owned D7..D1 + live D0), any-slot WRITE(3)
      coherence, `CMD_SET_DIGOUT` / `digout gpio <pin>`. ~1-packet latency.
- [ ] **Configurable amplifier bandwidth** — upper/lower cutoff registers (RH1/RH2/RL),
      host command + firmware sequence instead of the hardcoded init.
- [ ] **DSP high-pass / offset removal** config (register 0 DSP cutoff bits).
- [ ] **ADC self-calibration** command exposed to host (currently buried in the init blob).
- [ ] **Aux ADC inputs** — read aux1/aux2/aux3 (temperature sensor, supply voltage) and
      surface them in the packet + status.
      *Mechanism done 2026-06-11*: slot-2 default = accel sweep @10 kHz, slot-3 default =
      supply/temp/chip-ID loop; results land in packet words 34/0/1 labeled by command echo.
      Remaining: host-side scaling/labeling into engineering units (°C, V) + recording format.
- [ ] **Impedance testing** — test-current injection + capacitor-DAC select (regs 5/6/7),
      host workflow to sweep electrode impedance.
- [ ] **ADC format config** — two's-complement vs offset-binary, absolute-value mode.
- [ ] **Configurable digital inputs** capture into the packet (`:273` TODO).
- [x] **Generalize COPI sequences** — replace the hardcoded `convert`/`init`/`cable_test`
      blobs in `pl_control.c` with host-configurable, **looping aux command banks** + a
      fast-settle override layer. **Design: [`docs/command-bank-design.md`](docs/command-bank-design.md).**
      This is the foundation the fast-settle items (above) and impedance/temp/supply build on.
      **Done 2026-06-11** (`claude/aux-seq-v2`): `aux_command_sequencer.sv` — 3 slots, 2 banks
      each, length bound to bank, atomic packet-boundary swap, live bank upload + confirm
      handshake, one-shot injection for runtime register R/W (`CMD_READ_/WRITE_REGISTER`),
      command-echo identity in the packet header. Default OFF — datapath proven bit-identical
      to main when disabled. See `docs/NIGHT_LOG-2026-06-11.md`.
      *(Init/calibration/cable-test keep the legacy full-table path by design — they run while
      stopped; the banks cover the streaming-time use cases.)*

## B. Board-rev sensor / IO interfaces (2 ADC + 2 DAC + 9-axis IMU)

- [ ] **ADC SPI master** (new PL module) — develop test-first in sim against the
      converter datasheet timing. + testbench.
- [ ] **DAC SPI master** (new PL module) + testbench.
- [ ] **AXI registers** — DAC setpoints (PS→PL), ADC config; wire into `axi_lite_registers.v`.
- [ ] **Integrate 2 ADC samples into the packet** — fills the external-ADC metadata slots
      that are currently `0x0` (ties into Epic C).
- [ ] **Firmware control API + host commands** for DAC output / ADC config.
- [ ] **Constraints/pinout** for the new board revision (`programmable_logic/constraints/`).
- [ ] **9-axis IMU (BNO055) — absolute orientation.** I²C sensor; on-chip fusion outputs
      quaternion / Euler / linear-accel / gravity / temp / calibration. Read via the **Zynq PS
      hard-I²C (EMIO)** — mostly firmware, little/no new PL RTL — and stamp ~100 Hz into packet
      metadata (Epic C). **Physical layer (proven by the OE 3D headstage):** RHD2164 DDR uses a
      *single* MISO pair, so the **second MISO pair's two cable conductors are repurposed as
      dedicated SDA/SCL** (12-pin Omnetics: pin3=SDA, pin4=SCL). Our system uses the same DDR
      topology → same cable trick applies. Refs:
      [low-profile-headstage-64ch-3d](https://github.com/open-ephys/low-profile-headstage-64ch-3d)
      (RHD2164+BNO055 schematic); host decode
      [acquisition-board](https://github.com/open-ephys-plugins/acquisition-board) `AcqBoardONI.cpp`.
- [ ] **(Free win) Analog accelerometer via aux** — an ADXL335 on auxin1/2/3 is already captured
      by `CONVERT(32/33/34)`; needs only host-side decode/labeling (see Epic A aux item).

## C. Packet / metadata payload completion

- [ ] **Fill header words 3–4 + the 8 external ADC values** (presently hardwired `0x0`).
      Ref: `data_generator_core.sv:352` breadcrumb.
- [ ] **64-bit non-neural metadata state** in the exfil FSM (fast-settle flag, digital in,
      discrete-ADC values, IMU quaternion/accel, status).
- [ ] **Keep the 3-layer contract in sync** (PL ↔ `main.h` ↔ `net.py`).

## D. PL→PS throughput / DMA

- [ ] **Characterize the current BRAM-polling path** — measure the real ceiling vs the
      ~9 MB/s figure; find where it saturates (PS read loop? BRAM bandwidth? UDP?).
- [ ] **Re-attempt DMA, simulation-first** — AXI-DMA / AXI-DataMover from FIFO/stream to
      DDR. Prove the `tlast`/backpressure/burst-boundary handshake in a testbench before
      any bitstream (this is what bit the previous attempt).
- [ ] **Double-buffer + interrupt** scheme to replace polling.
- [ ] **Benchmark** against the current path; document the win.

## E. Firmware robustness / networking

- [ ] **Hotplug + DHCP** — finish lwIP hotplug init and add a DHCP/discovery option.
      Refs: `main.c:421-422`.
- [x] **Fix the 10 ms print timeout** that stalls RX (`shared_print.c:163`).
      **Done 2026-06-13 (`claude/dual-port`):** `send_message()` is non-blocking (drops +
      counts instead of the 10 ms spin); routine status moved off the print ring onto a
      shared-memory snapshot that **core 1** formats/prints (core 0 only does cheap binary
      stores), so status reporting never stalls the data pump. Fixed the "`get_status` adds
      2-7 packet errors while streaming" symptom that surfaced at the dual-port packet size.
- [ ] **Error-state recovery** — track timestamp-gap duration on recovery (`main.c:143`).
- [ ] **De-hardcode IPs** — config mechanism instead of baked-in 192.168.18.x.

## F. OpenEphys integration (`ephys-socket` plugin)

Target: [`ckemere/ephys-socket`](https://github.com/ckemere/ephys-socket) (clone at
`~/Code/ephys-socket`) — a forked OpenEphys "Ephys Socket" `DataThread`, already MicroZed-specific
(`IntanInterface` = UDP recv + TCP control; `IntanSocket` = parse → GUI buffer). It is the **third
consumer of the register/packet contract** (after firmware and `net.py`) — `IntanInterface`
duplicates `net.py`'s `CMD_*` set.

Current gaps (plugin is WIP):
- [x] **Packet alignment** — reads data flat (`dataWords[ch/2]`, `IntanSocket.cpp:481`) with **no
      +2 pipeline shift** and ignores metadata words [6–9]. Needs the PL-side alignment +
      command-echo (Epics A/C) to get correctly-labeled neural + aux channels.
      **Done**: `fix/channel-reorg-aux-deskew` added the +2 de-skew + stream de-interleave;
      `claude/aux-test-tooling` adds the command-echo decode (header words 4/5).
- [x] **Aux split + scaling** — allocates 3 aux/bank + `aux_data_scale` but doesn't actually
      separate/align them; wire to the echo metadata.
      **Done 2026-06-12** (`claude/aux-test-tooling`): per-packet flag-driven parse — legacy
      cycles 34/0/1 vs sequencer-mode echo-identified accel de-interleave (1 axis/packet,
      sample-and-hold). Absolute aux calibration (engineering units) still pending.
- [x] **Command parity (testing subset)** — add the new `CMD_*` (aux bank write, fast settle,
      digout, register R/W) to `IntanInterface`, in lockstep with `net.py` + firmware.
      **Done 2026-06-12** for bank write/select/seq-en, fast settle, READ_REGISTER + 98-byte
      status, with GUI controls (STATUS / SETTLE / AUX SEQ buttons, TTL-settle dropdown — all
      live during acquisition). Remaining: SET_DIGOUT and WRITE_REGISTER are defined as command
      IDs but have no GUI affordance yet.
- [ ] **Contract single-source** (see Epic H) — make the plugin consume the generated
      packet/register/`CMD` definition so all 3 consumers stay in sync. *This is the real coupling.*

Integration approach (must stay installable via the OpenEphys Plugin Installer, which requires a
standalone repo + releases):
- [ ] **Decide submodule vs subtree** for co-locating in this repo:
      - *Submodule* (recommended): plugin stays its own repo (source of truth, installable); this
        repo references it at e.g. `openephys-plugin/`. Standard, low-risk; bump pointer on change.
      - *Subtree*: plugin code lives in this repo (primary dev); `git subtree push` to the
        standalone repo to cut installable releases. Smoother co-dev, more arcane.
      - Either way the standalone repo + its CMake/CI/releases persist for installability.
- [ ] **Build + docs** for the plugin within this repo.

## G. Verification infrastructure (scoped to where iteration cost is highest)

- [ ] **RTL elaboration/synth smoke check** — cheap; catches width/latch/multi-driver/loop
      errors in minutes. Run on every RTL change.
- [ ] **Testbenches for NEW modules** — ADC/DAC SPI masters, the DMA datapath, packet
      packing. (Not the proven legacy datapath.)
- [ ] **Python mock board** — UDP packet generator + TCP responder so `net.py`'s
      `DataValidator`/`CableDetection` run with no hardware.
- [ ] **Host-side unit tests** — pure logic (`calculate_packet_size`, struct unpacking).
- [ ] **Native-gcc firmware unit tests** for pure logic (packet-size math, struct packing).

## H. Maintainability

- [ ] **Single-source register/packet contract** — one definition that generates the PL
      params, `main.h` offsets, and `net.py` constants. Kills hand-sync duplication.
- [ ] **Fix CLAUDE.md toolchain path** — tools are at `/opt/Xilinx/2025.1/`, not `~/Xilinx/`.
- [ ] **Build/CI smoke** — scripted elaborate + firmware compile (`scripts/clean_build_all.sh`
      already does a full Vivado→XSA→Vitis build end-to-end).

## I. Second cable interface (dual headstage port)

The carrier PCB has a **second Omnetics connector already routed to the FPGA** with its own
SPI signals (CS=IO_L15, SCLK=IO_L17, COPI=IO_L19, CIPO1=IO_L22, CIPO2=IO_L24) — so this is a
PL→firmware→host→plugin job, **no board respin**. A second cable doubles channel capacity
(up to 256 ch) but also doubles the PS→UDP stream (~9 → ~18 MB/s), which is the main risk and
the reason to validate the memory bus first. **Common command set initially** (one acquisition
FSM, master outputs fanned out to both ports' pins), but **independent per-port cable length /
phase** from the start. Keep default-OFF / bit-identical to the single-port path when the
second port is disabled, the same discipline used for the aux sequencer.

- [x] **Phase 1 — datapath + bandwidth characterization (debug mode).** Widen the FIFO word
      64→128-bit `{cipo3,cipo2,cipo1,cipo0}`, extend `channel_enable` 4→8 bits + per-port phase
      regs, extend the packet format and the three-layer contract (`main.h`/`pl_control.c`/
      `net.py`). Have **debug mode synthesize port-2 data** so packets are full doubled size,
      then measure whether PS core-0's BRAM-poll + lwIP UDP loop sustains ~18 MB/s. No real
      CIPO-capture risk yet — pure datapath/throughput. Feeds, and is gated by, **Epic D**.
      **RTL/firmware/host DONE 2026-06-13 (`claude/dual-port`):** 128-bit packer (4-chunk),
      core/wrapper/contract widened, default-OFF & proven bit-identical (FIFO-packer TB 423
      checks + integration TB 26,006 checks incl. 25,910-cycle co-sim vs main). Full build
      routes & meets timing (WNS +0.646, 0 failing endpoints, no congestion; LUTRAM 368→933).
      `blobs/BOOT.bin` rebuilt. **Remaining: run the on-hardware bandwidth measurement**
      (debug mode + `set_channels 0xFF` → ~150-word packets; confirm PS→UDP sustains ~18 MB/s).
- [~] **Phase 2 — real second-port capture + independent cable detection.** Add the two
      `IBUFDS` + phase selectors and `phase2`/`phase3`; extend `auto_cable_detect` / `net.py`
      INTAN-pattern sweep to run **per port** (different cable lengths → different optimal phase).
      **PL/firmware/host DONE 2026-06-15 (`claude/dual-port`):** 2nd LVDS buffer instance +
      `intan_spi_b` interface + external `spi_lvds_1` port; port-B pins on **bank 35** (CS=F19/F20,
      SCLK=J20/H20, COPI=H15/G15, cipo2=L14/L15, cipo3=K16/J16 — verified placed in the routed
      checkpoint). Build routes + meets timing (WNS +0.133, 0 failing). Per-port phase via
      `set_phase <p0> <p1> [p2 p3]` / `CMD_SET_PHASE_B`; phases shown in the core-1 status snapshot.
      `blobs/BOOT.bin` rebuilt. **Remaining: (1) on-hardware bring-up** — plug a 2nd analog
      headstage into port B, sweep `set_phase ... p2 p3` for the INTAN pattern, confirm real
      neural on port B; **(2) automate per-port `auto_cable_detect`** (host-only, best done with
      the board). **Verify COPI's bank-35 IO_L19 (H15/G15) against the board before power-on.**
- [ ] **Phase 3 — plugin multi-input (acq-board model).** One `DataStream`, channels grouped by
      port with prefixes (`A_CH1…` / `B_CH1…`, per-port AUX), per-port chip detection in the
      editor. Mirrors `acquisition-board` `DeviceThread`/`Headstage` (single stream, prefixed
      headstage groups, `setFirstChannel` offsets).
- [ ] **Future — independent per-port commands.** Split command generation (or a second light
      FSM) so one port can run SPI + analog aux while the other runs the I²C-repurpose **digital
      IMU** (lab has one digital-IMU headstage + two analog). Pins already exist; design the
      Phase-1 register layout so this stays additive.

---

## Reference designs (OpenEphys / Intan open source)

References mined while scoping the above. Note OE's 3rd-gen path uses **liboni/ONI framing**
(a software protocol), *not* ONIX coax hardware — the 3D headstage still connects over the
standard Intan SPI Omnetics cable.

- [open-ephys/rhythm](https://github.com/open-ephys/rhythm) — Intan/OE **Rhythm FPGA** Verilog:
  fast-settle injection, looping **aux command banks** (template for "generalize COPI sequences"),
  DDR/MISO phase handling.
- [open-ephys/low-profile-headstage-64ch-3d](https://github.com/open-ephys/low-profile-headstage-64ch-3d)
  — OE 64ch **3D headstage** (RHD2164 + **BNO055**); KiCad schematic proves the dedicated SDA/SCL
  cable wiring (2nd MISO pair → I²C).
- [open-ephys-plugins/acquisition-board](https://github.com/open-ephys-plugins/acquisition-board)
  — OE GUI plugin; `AcqBoardONI.cpp` decodes the BNO frame;
  `rhythm-api/rhd2000registers.cpp::createCommandListTempSensor` builds the aux/accelerometer/
  temp/supply command bank.
- [open-ephys/commutator-controller](https://github.com/open-ephys/commutator-controller) — RP2040
  commutator firmware (JSON-`{turn}` follower; does *not* read the IMU).
- Intan **RHD2000 Rhythm** USB/FPGA reference (`RHD2000InterfaceXEM6010`, from intantech.com downloads).
