# Laptop-based HITL (JTAG / QSPI): findings & decision

**Date:** 2026-06-16
**Outcome:** **Abandoned.** Laptop-driven hardware-in-the-loop (flash + reboot the
MicroZed without SD swaps) is not worth it. **Go back to the SD-card + remote-build
workflow.** Keep the laptop only for `net.py`/`verify_sine` verification over the
network.

This documents an attempt to set up autonomous HITL on the laptop (per
`docs/hardware_in_the_loop.md`) so firmware iterations could be flashed and tested
without round-tripping through a human. It records what worked, what didn't, and why,
so we don't re-derive it.

## BLUF — why it was dropped

Two blockers made the laptop loop no better than the SD workflow:

1. **Reboot can't be automated.** On this Zynq-7020, JTAG cannot trigger a
   power-on reset: `rst -por` → "por not supported for target"; `rst -srst` and an
   SLCR soft-reset (`mwr 0xF8000244 0x1`, SLCR unlocked) reset PS logic but do **not**
   re-run the BootROM, so a newly-flashed QSPI image never boots without a **physical
   power toggle**. That's the same manual step as swapping an SD card.
2. **The laptop can't build a working image.** Rebuilding the *committed* source on
   this laptop (both the existing `vitis_workspace` and a clean
   `create_vitis_project.py` regen) produced BOOT.bins that boot to **garbage status /
   dead networking / silent console**, while the dev-box-built `blobs/BOOT.bin` from
   the same source boots clean. The laptop's app ELFs are deterministic (identical
   across rebuilds), so this is a build-environment / project-state gap, not
   randomness. The image diff vs `blobs/BOOT.bin` begins in the FSBL region. Root
   cause not fully isolated; builds happen remotely anyway, so the laptop was never
   going to be the build machine.

Net: the laptop loop collapses to *remote build → copy BOOT.bin → `program_flash`
(~2 min) → **power toggle** → test* — a lateral move vs *remote build → write SD →
insert → **power on** → test*. Both need a manual board action per iteration. Only a
fully hands-off loop would justify the switch, and that needs either a
software-triggerable reboot (unachievable here) or a USB-controlled power relay
(hardware not present).

## What actually worked (reusable)

- **JTAG via Digilent JTAG-HS2.** The cable's FTDI (`0403:6014`) gets grabbed by the
  kernel `ftdi_sio` driver, blocking `hw_server`. Fixed with a one-time udev rule
  (`/etc/udev/rules.d/99-xilinx-jtag-hs2.rules`) that sets `MODE=0666` and unbinds
  `ftdi_sio` on the `bind` action. JTAG chain then enumerates fine (`arm_dap` +
  `xc7z020`; processor view `APU` > `ARM Cortex-A9 MPCore #0/#1`). The board UART is a
  separate FT230X (`0403:6015`) → `/dev/ttyUSB*`, 115200 8N1. **This udev rule is left
  in place** — harmless (only affects the HS2 cable) and useful for JTAG *debugging*.
  - Discipline that matters: don't thrash `hw_server` (`pkill -9` + immediate restart
    leaves the FT232H half-open and the chain scans empty). Let `xsct` own it; if you
    must kill, `pkill -TERM -x hw_server` then wait. On a cold connect, poll until the
    target tree populates.
- **QSPI boot is deterministic and clean.** Booting the dev-box `blobs/BOOT.bin` from
  QSPI gives a healthy boot every time (sane status, ARP resolves, TCP:6000 up).
- **`program_flash` reflashes QSPI over JTAG with jumpers parked on QSPI** — no jumper
  change per reflash. Do `rst -srst` first to clean the PS for the writer-FSBL.
  Command: `program_flash -f BOOT.bin -fsbl <fsbl.elf> -flash_type qspi-x4-single
  -verify -url tcp:127.0.0.1:3121` (generic `qspi-x4-single` auto-detects the Micron
  flash; use `-url`, not `-cable`). NOTE: the **first** program of an EMPTY flash needs
  **JTAG boot mode** (empty-flash + QSPI-boot-mode = BootROM lockdown →
  "Problem in Initializing Hardware"); once a valid image is present, reflash works in
  QSPI boot mode.
- **`net.py` / `verify_sine` over the network** — the ground-truth check. This is the
  valuable laptop asset and is independent of how firmware is loaded.

## What didn't work

- **Pure-JTAG PS init** (`ps7_init.tcl` or FSBL-over-JTAG, then `dow` the app ELFs):
  non-deterministic — boots fully one run, silent the next; `unable to alloc pbuf in
  init_dma` (EMAC RX dead), corrupted netif IP print. Repeated DDR re-inits in one
  session corrupt DDR. Not viable.
- **JTAG-triggered reboot** (see blocker #1).
- **Local rebuilds** (see blocker #2).

## The actual deliverable still stands

The point of all this was to test the BRAM burst-read fix (see
`docs/bram_burst_read_bug.md`). That is **verified working** on hardware via the
committed `blobs/BOOT.bin` (the 8-word chunked-read fix, `BRAM_READ_CHUNK_WORDS = 8`):

- `verify_sine 0xFF`, `0x3F`, `0x0F`: **real corruption (|d|>1) = 0**, **PACKET LOSS:
  PASS (0 dropped)**. The original out-of-range garbage (~3.2% on `0xFF`) is gone.
- Caveat: `verify_sine`'s strict VALUE CHECK reports FAIL on ~0.5–0.8% of samples that
  differ by exactly **±1 LSB** — present on the `0x0F`/`0x3F` controls too. `net.py`
  itself attributes these to sine-ROM rounding (`$rtoi/$sin`), *not* corruption. Read
  the `real errors (|d|>1)` line, not the headline. (TODO if desired: add a ±1 LSB
  tolerance to the VALUE CHECK so it prints PASS — a validation-threshold change only,
  does not touch sample values.)
- Minor open item: at `0xFF` the capture reaches ~268–270/300 packets (controls hit
  300) with a rare `Magic number mismatch` ~packet 269 — ~10% lower throughput on the
  600-byte packets, no mid-stream loss. Worth characterizing but not corruption.

## Recommended workflow going forward

1. Build remotely (the dev box produces working images; the laptop does not).
2. Load `BOOT.bin` via **SD card** (your existing, proven path), normal boot-mode
   jumpers.
3. Use **this laptop's `net.py`** (`ZYNQ_IP = 192.168.18.10`) for `verify_sine` and
   stream validation — that's the ground-truth check and the one piece that genuinely
   belongs here. Host NIC must be on `192.168.18.0/24` (the USB dongle
   `enx00e04cd1aa13` → `192.168.18.100`).

If fully hands-off HITL is ever wanted, the missing piece is **programmatic power
control** (a USB-switched relay on the board's supply), which would make the
QSPI-`program_flash` loop autonomous. Until then, SD is simpler.
