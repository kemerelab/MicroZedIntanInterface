# Prompt to paste into the hardware-in-the-loop Claude Code instance

Copy everything below the line into the new Claude Code session (run from the repo
root on the laptop that has the MicroZed attached + Vivado/Vitis 2025.1).

---

You are working in the MicroZedIntanInterface repo on branch `claude/dual-port`,
on a laptop that has the **MicroZed hardware physically attached** (USB JTAG+UART,
and Ethernet to this laptop) plus **Vivado and Vitis 2025.1**. So you can build,
flash, and test against real hardware — this is hardware-in-the-loop.

**Read first:**
- `docs/bram_burst_read_bug.md` — the bug, the proven root cause, the dead ends
  (don't re-chase them), and the remaining task list.
- `docs/hardware_in_the_loop.md` — how to flash (JTAG via `xsct`), read the serial
  console (pyserial helper), and drive `net.py` non-interactively.
- `CLAUDE.md` and `overview.txt` — system architecture, build commands, the
  3-layer register/packet contract, and conventions.

**The situation in one breath:** at the full dual-port `0xFF` config, ~3% of
streamed samples are corrupt because the PS reads the capture BRAM with `memcpy`,
which issues long AXI bursts over the M_AXI_GP port, and long GP-port bursts
intermittently corrupt a short run of beats. It's reproducible on a *stopped* board
(`dump_bram` shows `memcpy`-burst garbage where single `Xil_In32` reads are clean),
so it's the burst transaction — not the BRAM, RDW, clock, or cache (all ruled out
and rebuilt/tested). A candidate fix is in place: read the packet in **8-word
`memcpy` chunks** (`BRAM_READ_CHUNK_WORDS` in `firmware/src-core0/main.c`), staying
under the ~11-word length that triggers the corruption. `blobs/BOOT.bin` is the
pre-built image with this fix + a stop/restart re-sync fix.

**Your tasks, in order:**
1. **Bring up HITL.** Confirm you can flash (prefer JTAG `xsct` download — no SD
   swaps), read the serial console, and run `remote/net.py` against the board.
   Verify network reachability (`ping`, `ZYNQ_IP` in `net.py`). Build the helper
   scripts from `docs/hardware_in_the_loop.md` and adjust JTAG target names to this
   board.
2. **Verify the chunked fix.** Flash the current build, then debug-mode
   `verify_sine 0xFF` → expect `VALUE CHECK: PASS` and `PACKET LOSS: PASS`. Also
   run `verify_sine 0x0F` and `0x3F` (controls — must stay clean) and exercise
   start/stop/restart a few times (the magic-scan re-sync should keep it clean).
3. **Tune `BRAM_READ_CHUNK_WORDS`** (firmware-only, ~1–2 min/iter): if clean + no
   loss, raise it (10, then higher) re-checking `verify_sine` each step to find the
   largest safe / fastest chunk; if any diffs survive, drop to 4. Record the
   sweep — chunk size vs. pass/fail vs. packet-loss.
4. **DMA (the real fix).** Scaffold a PL DMA (`axi_datamover`/`axi_dma`) to copy
   BRAM→DDR with controlled bursts, taking the CPU GP port off the data path.
   Verify it doesn't reintroduce the corruption and that core 0 is freed up. This
   is a bitstream + firmware change.
5. Re-confirm the OpenEphys plugin (`ephys-socket`, branch
   `claude/dual-port-plugin`) streams `0xFF` cleanly end to end.

**Method:** make ONE change at a time and verify on hardware before the next — this
bug has burned many builds on plausible-but-wrong theories, so let the hardware
adjudicate. Use `verify_sine` (ground truth, no chip needed) and the `dump_bram`
`burst | single | flag` diagnostic. When a result contradicts a hypothesis, trust
the result.

**Hard constraints:**
- **Data fidelity:** never alter acquired sample values (no detrend/baseline/
  filter/offset). Fix display via scale factors / viewer ranges only.
- Don't `git push` or change anything outside this task without asking the human.
- After any PL change, regenerate the Vitis platform with the `create_` script
  (clean `vitis_workspace/` first); `build_` only recompiles apps against the
  stale platform.
- The `-1` part's M_AXI_GP is rated ~150 MHz — keep the AXI clock at 131.25 MHz
  (175 MHz was tried; it doesn't fix this and over-clocks the hardened PS7).

Start by reading the two docs above, then bring up the HITL loop and report what
you can/can't reach (flash, serial, network) before changing anything.
