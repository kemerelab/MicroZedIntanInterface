# Boot-time RX wedge / `[Errno 64] Host is down` — solutions to test

## Important: I corrected course partway through

I started to just re-apply the reverted `7c8a18c` fix — then I read the memory
note you left last time, whose description says the root cause is still **OPEN**:
the pbuf/un-serviced-window theory was **ruled out by a serial log**, and
`7c8a18c` was reverted because its `resetrx` call **toggled RXEN on healthy RX
and regressed normal boots**. So I threw away the naive re-apply and rebuilt an
**honest, diverse** set instead. Critically, **none of these branches re-ship the
`resetrx` RXEN-toggle** that burned you last time.

Because the cause is genuinely open, the highest-value thing isn't another blind
fix — it's the **diagnostic build** that shows the actual GEM RX hardware state
when it wedges. Flash that first.

## What the symptom tells us (and what it doesn't)

`[Errno 64] Host is down` (EHOSTDOWN) is produced by *your Mac's* stack when ARP
resolution gets no reply — i.e. the board isn't answering ARP. Your "hang, hang,
then Host is down" sequence is just the Mac's ARP cache aging: while it still has
a (stale) entry, `connect()` blocks sending SYNs into the void (~75 s); once the
entry expires and re-ARP gets no reply, it fails fast with EHOSTDOWN. Ping also
failing confirms it's **Ethernet-level on the board** (RX not answering), not a
host ARP-table problem — which is why clearing the host ARP cache never helped.
What we *don't* yet know is *why* the board's RX dies. That's what the diagnostic
resolves.

## The branches (all off `claude/unified-ports`; all include the host fix)

| Branch | Kind | Reflash? | What |
|--------|------|----------|------|
| `claude/fix-net-connect-retry` | **Definite improvement** | No | `net.py` retries instead of hanging/crashing. Apply always. |
| `claude/rxwedge-diag` | **Diagnostic (flash FIRST)** | Yes | Logs real GEM RX state (`RXSR`/`RXEN`/`RXcnt`…) so we see the wedge signature. |
| `claude/rxwedge-service` | Hypothesis | Yes | Drain RX continuously through boot (NO resetrx). Cleaner test than 7c8a18c. |
| `claude/rxwedge-arp-ready` | Hypothesis / ergonomics | Yes | Gratuitous ARP + accurate `READY:` line. Board announces itself. |

Each firmware branch has its built image at `blobs/BOOT.bin` (Vitis 2025.1,
firmware-only, no PL change). Copy it to the FAT32 Boot partition.

## Suggested test procedure

1. **Pull + run `net.py` from `claude/fix-net-connect-retry` right now** (no
   reflash). Launch it *early*, during boot. It should print `Waiting for board
   ...` and connect the instant the board is up — no traceback. This is also a
   diagnostic: if it waits **forever**, the board really is wedged (→ step 2); if
   it always connects after a few seconds, the board was only *booting*, not
   wedging, and you may be done.

2. **Flash `claude/rxwedge-diag`, capture the serial console, and reproduce the
   wedge** (connect early until it dies). Read the `[GEMDIAG …]` lines:
   - `RXSR` shows `BUFFNA=1` or `OVR=1` with `RXcnt` frozen → **RX-used-bit hang
     errata confirmed** (ring starved / overrun). → try `rxwedge-service`; if that
     doesn't hold, we need a *correct* RX-hang reset (sketch below), not resetrx.
   - `RXEN=0` → something **disabled** the receiver. → look at `eth_link_detect` /
     PHY-speed reconfig racing with early traffic.
   - Everything `0`/healthy but still no reply → the wedge is **not** in the MAC
     RX visible state → suspect the BD-ring/DDR-remanence angle (sketch below) or
     the PHY.
   Send me those lines and I'll know exactly which fix class is real.

3. Based on step 2, flash `rxwedge-service` and/or `rxwedge-arp-ready` and re-run
   the torture test (power-cycle, spam early connects, yank/replug cable ×10).

## What I deliberately did NOT rebuild

- **`resetrx`-on-idle** (the 7c8a18c regression). Gone from every branch.

## Further levers I can build once the diagnostic points somewhere (not built blind)

- **Correct RX-hang watchdog:** reset the RX path *only* on the real hang
  signature (`RXSR.BUFFNA/OVR` set **and** `RXcnt` stalled **while link up**),
  never on a merely-idle RX. This is what `resetrx` *should* have been.
- **BD-ring / pbuf-pool zero + re-init at boot:** your CLAUDE.md already flags DDR
  remanence — a Zynq reset doesn't zero DRAM, so the GEM BD rings / lwIP pools can
  carry stale state across a power cycle. If the diag shows a healthy-looking MAC
  that still won't receive, explicitly re-initing those rings at boot is the prime
  suspect fix.
- **Hold RX bring-up until the listener is up:** reorder so nothing is received
  until we're ready to service, if the diag shows early traffic is the trigger.

## One thing I need from you

The serial log from step 2 (or the earlier one that "ruled out the pbuf theory").
That single capture collapses four hypotheses into one fix.
