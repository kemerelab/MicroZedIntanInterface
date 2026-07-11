# Device discovery beacon

Once fully initialized, the board **broadcasts** a small identity datagram ~1 Hz.
A client uses it to discover the board's IP, know the board is up, and stay fully
passive during the board's fragile boot window (no packets to the board until it
has announced itself — the clean way to avoid the early-connect RX wedge).

## Transport
- **Subnet-directed broadcast** to `<subnet>.255` (e.g. `192.168.18.255`), UDP
  **port 0x6880 / 26752** (`BEACON_PORT`).
- Sent every ~1 s from `network_maintenance_loop` while the link is up, once the
  board has finished init (so hearing it == "ready to connect").
- Broadcast on purpose: needs no IGMP, and a host with no listener on 0x6880 will
  **not** send an ICMP port-unreachable (hosts don't ICMP-error broadcasts), so an
  idle beacon never storms anyone.

## Wire format — `device_beacon_t` (28 bytes, little-endian, naturally aligned)

| Off | Type       | Field       | Notes |
|-----|------------|-------------|-------|
| 0   | u32        | `magic`     | `0x4B4C4231` (`BEACON_MAGIC`) |
| 4   | u32        | `version`   | `1` (`BEACON_VERSION`) |
| 8   | u32/4B     | `ip`        | board IPv4, **network byte order** (== datagram source) |
| 12  | u16        | `tcp_port`  | control port (0x6900) |
| 14  | u16        | `udp_port`  | unified data port (0x6800) |
| 16  | u32        | `fw_version`| `maj<<24 | min<<16 | patch<<8 | build` |
| 20  | u8[6]      | `mac`       | board MAC = unique device id |
| 26  | u16        | `reserved`  | 0 |

Defined in `firmware/include/main.h` (`device_beacon_t`, guarded by a
`_Static_assert` on the 28-byte size), built in `firmware/src-core0/network.c`
(`beacon_send`). **Keep these three in sync:** firmware, `remote/net.py`
(`_parse_beacon`, format `'<II4sHHI6sH'`), and the ephys-socket plugin.

## Client pattern
1. Bind UDP 0x6880/26752 (`INADDR_ANY`, `SO_REUSEADDR`), listen.
2. On a datagram: check `magic`, take the **source address** as the board IP
   (authoritative), read ports / fw / mac from the payload.
3. If several boards answer, list them by MAC and let the user pick.
4. Only then open the TCP control connection (bounded, paced retry — see
   `connect_with_retry` in net.py / the ephys-socket TODO). If no beacon arrives
   within a timeout, fall back to a configured IP (for older firmware).

net.py implements all of this (`discover_board`, and the `__main__` gate that
auto-fills `ZYNQ_IP`). ephys-socket should mirror it (phase 2).
