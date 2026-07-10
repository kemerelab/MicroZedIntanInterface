import socket
import errno
import sys
import threading
import struct
import time
import random
import queue
import math
import ipaddress
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

ZYNQ_IP = "192.168.18.10"  # IP of the Zynq board
TCP_PORT = 6000  # Must match your board's TCP_PORT
UDP_PORT = 5000  # Must match your board's UDP_PORT (ALL streams; demux by stream_type)

# ---------------------------------------------------------------------------
# Device discovery beacon (contract: firmware main.h device_beacon_t, built in
# network.c). The board broadcasts a 28-byte beacon to <subnet>.255:BEACON_PORT
# ~1 Hz once it's up. We listen for it to (a) auto-discover the board's IP (no
# hardcoding), (b) confirm it's ready before we send it anything, and (c) stay
# fully passive during its fragile boot window. Falls back to the configured
# ZYNQ_IP if no beacon is heard (older firmware without the beacon). The beacon
# is a subnet BROADCAST, so an unbound host port does NOT provoke an ICMP
# port-unreachable (hosts don't ICMP-error broadcasts) -- safe to close after.
# ---------------------------------------------------------------------------
BEACON_PORT = 5050
BEACON_MAGIC = 0x4B4C4231
_BEACON_FMT = '<II4sHHI6sH'   # magic,version,ip,tcp,udp,fw,mac,reserved = 28 bytes
BEACON_DISCOVERY_TIMEOUT = 0   # must exceed board boot (>20 s) PLUS any
                                  # interface bring-up (a USB-C Ethernet adapter
                                  # re-enumerating on a power-cycle adds seconds).
                                  # Set to 0 to skip discovery (old fw / blocked port).


def _parse_beacon(data):
    if len(data) < 28:
        return None
    magic, version, ip_bytes, tcp_port, udp_port, fw, mac, _ = \
        struct.unpack(_BEACON_FMT, data[:28])
    if magic != BEACON_MAGIC:
        return None
    return {
        'payload_ip': socket.inet_ntoa(ip_bytes),
        'tcp_port': tcp_port,
        'udp_port': udp_port,
        'fw': f"{(fw >> 24) & 0xff}.{(fw >> 16) & 0xff}.{(fw >> 8) & 0xff}.{fw & 0xff}",
        'mac': ':'.join(f'{b:02x}' for b in mac),
        'version': version,
    }


def _open_beacon_socket(quiet=False):
    """Create + bind a fresh UDP listener on BEACON_PORT (INADDR_ANY). Returns the
    socket or None."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    for opt in ('SO_REUSEPORT', 'SO_BROADCAST'):
        try:
            sock.setsockopt(socket.SOL_SOCKET, getattr(socket, opt), 1)
        except (AttributeError, OSError):
            pass
    try:
        sock.bind(('', BEACON_PORT))
    except OSError as e:
        if not quiet:
            print(f"[DISCOVERY] Could not bind UDP {BEACON_PORT}: {e}")
        sock.close()
        return None
    return sock


def discover_board(timeout=BEACON_DISCOVERY_TIMEOUT, quiet=False, rebind_interval=5.0):
    """Listen for the device discovery beacon on UDP BEACON_PORT and return the
    first valid board (dict incl. 'ip' from the datagram source), or None if none
    arrives within `timeout`. Fully passive -- sends nothing to the board.

    Re-creates the listening socket every `rebind_interval` s. This matters when
    the board-facing interface appears or flaps DURING discovery -- e.g. a USB-C
    Ethernet adapter re-enumerating on a board power-cycle takes the interface
    down then up. A socket bound while that interface was down won't reliably
    receive its broadcasts once it returns; a fresh bind after it's up does."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        # Diagnostic: which local IP would reach the board right now? If it's not
        # on the board's subnet, the board-facing interface isn't up yet (this is
        # a pure route lookup -- no packet is sent to the board).
        if not quiet:
            print(f"[DISCOVERY] listening on UDP {BEACON_PORT} "
                  f"(local route toward {ZYNQ_IP}: {get_local_ip()})")
        sock = _open_beacon_socket(quiet)
        if sock is None:
            time.sleep(1.0)
            continue
        try:
            sub_deadline = min(deadline, time.time() + rebind_interval)
            while time.time() < sub_deadline:
                remaining = sub_deadline - time.time()
                sock.settimeout(max(0.2, min(remaining, 3.0)))
                try:
                    data, addr = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                dev = _parse_beacon(data)
                if dev is None:
                    continue             # not our beacon; keep listening
                dev['src_ip'] = addr[0]
                dev['ip'] = addr[0]      # datagram source is authoritative
                return dev
        finally:
            sock.close()
        left = deadline - time.time()
        if not quiet and left > 0:
            print(f"[DISCOVERY]   ...still listening ({left:.0f}s left, re-binding)")
    return None

# Updated data generator constants
MAGIC_NUMBER_LOW = 0xDEADBEEF
MAGIC_NUMBER_HIGH = 0xCAFEBABE

# ---------------------------------------------------------------------------
# Unified packet format (docs/unified-packet-format.md). Every PL stream emits
# the SAME 8 x 32-bit little-endian common header on UDP 5000; the host demuxes
# by stream_type and verifies the per-stream SEQ continuity (the loss check).
#   w0 MAGIC=0xCAFEBABE | w1 TYPE_VER=type[7:0]|version[15:8]|flags[31:16]
#   w2/w3 64-bit timestamp | w4 SEQ | w5 AUX0 | w6 AUX1 | w7 RSVD
# ---------------------------------------------------------------------------
UNIFIED_MAGIC = 0xCAFEBABE
UNIFIED_VERSION = 1
UNIFIED_HEADER_WORDS = 8
STREAM_TYPE_BROADBAND = 1
STREAM_TYPE_LFP = 2
STREAM_TYPE_WAVELET = 3   # reserved for the follow-on branch

# Binary command protocol constants
CMD_MAGIC = 0xDEADBEEF
CMD_PACKET_SIZE = 20
ACK_PACKET_SIZE = 3

# Command IDs
CMD_START = 0x01
CMD_STOP = 0x02
CMD_RESET_TIMESTAMP = 0x03
CMD_SET_LOOP_COUNT = 0x10
CMD_SET_PHASE = 0x11
CMD_SET_DEBUG_MODE = 0x12
CMD_SET_CHANNEL_ENABLE = 0x13
CMD_SET_PHASE_B = 0x14   # port B (second cable) CIPO phase
CMD_LOAD_CONVERT = 0x20
CMD_LOAD_INIT = 0x21
CMD_LOAD_CABLE_TEST = 0x22
CMD_FULL_CABLE_TEST = 0x30
CMD_GET_STATUS = 0x40
CMD_DUMP_BRAM = 0x41
CMD_SET_UDP_DEST = 0x50
CMD_PING = 0x60
# Aux command sequencer / override layer (mirror firmware/src-core0/network.c)
CMD_AUX_WRITE_WORD = 0x70   # param1 = slot | bank<<8 | is_len<<16; param2 = addr<<16 | data
CMD_AUX_BANK_SELECT = 0x71  # param1 = slot; param2 = bank
CMD_AUX_SEQ_EN = 0x72       # param1 = 0/1
CMD_READ_REGISTER = 0x73    # param1 = reg -> 4-byte {cipo1, cipo0} response
CMD_WRITE_REGISTER = 0x74   # param1 = reg; param2 = value -> 4-byte echo response
CMD_SET_FAST_SETTLE = 0x75  # param1 = amp: sw|gpio_en<<1|pin<<4; param2 = dsp: same layout
CMD_SET_DIGOUT = 0x76       # param1 = sw|gpio_en<<1|pin<<4; param2 = reg3_static byte
CMD_SET_CHIRP = 0x77        # param1 = mode | stride<<8; param2 = fspan | rate<<16 (CTRL_REG_3)

# Analytic chirp NCO scaling (must match data_generator_core.sv CTRL_REG_3)
CHIRP_PHW          = 32
CHIRP_FSPAN_SHIFT  = 16     # f_max = fspan << 16 (phase-accumulator units)
CHIRP_RATE_SHIFT   = 9      # freq_acc step/packet = rate << 9
PACKET_RATE_HZ     = 30000  # one phase update per broadband packet
# LFP/DSP engine (Tier-1) -- configure while disabled, then enable
CMD_LFP_ENABLE = 0x80       # param1 = 0/1
CMD_LFP_SET_PARAMS = 0x81   # param1 = decim_R, param2 = num_taps
CMD_LFP_SET_CHANNELS = 0x82 # DEPRECATED: LFP lane mask now mirrors broadband channel_enable (firmware accepts-and-ignores)
CMD_LFP_WRITE_COEF = 0x83   # param1 = [0] clear-ptr-first; param2 = 18-bit signed coef
CMD_UDP_BENCH = 0x90        # param1 = payload bytes, param2 = n_packets (throughput test)
CMD_PERF_RESET = 0x91       # clear recv->transmit sticky maxes + histogram + counts

UDP_BENCH_PORT = 5002       # UDP throughput-benchmark blaster
# Unified port: the LFP band now arrives on UDP_PORT (5000) mixed with broadband,
# demuxed by stream_type=2. The persistent UnifiedSink (created in __main__)
# drains 5000 promiscuously and fans the LFP frames out to subscribers, so the
# host never replies ICMP port-unreachable to the board while LFP streams.
UNIFIED_SINK = None
LFP_COEF_FRAC = 17          # Q1.17 coefficient fixed-point

AUX_BANK_ENTRIES = 64       # entries per bank (PL aux_command_sequencer ADDR_W=6)

# RHD2000 SPI command encodings (datasheet-confirmed; see docs/)
def rhd_convert(ch, h=0):
    """CONVERT(ch); h=1 sets bit H (DSP high-pass reset when DSP enabled)"""
    return ((ch & 0x3F) << 8) | (h & 1)

def rhd_write(reg, val):
    """WRITE(reg, val)"""
    return 0x8000 | ((reg & 0x3F) << 8) | (val & 0xFF)

def rhd_read(reg):
    """READ(reg)"""
    return 0xC000 | ((reg & 0x3F) << 8)

def rhd_decode(cmd):
    """Human-readable form of an echoed RHD command word"""
    top = (cmd >> 14) & 0x3
    arg = (cmd >> 8) & 0x3F
    if top == 0:
        names = {32: 'aux1/accelX', 33: 'aux2/accelY', 34: 'aux3/accelZ',
                 48: 'supply', 49: 'temp'}
        tag = f" ({names[arg]})" if arg in names else ""
        h = " +H" if (cmd & 1) else ""
        return f"CONVERT({arg}){tag}{h}"
    if top == 2:
        return f"WRITE({arg}, 0x{cmd & 0xFF:02X})"
    if top == 3:
        return f"READ({arg})"
    if cmd == 0x5500:
        return "CALIBRATE"
    return f"0x{cmd:04X}"

# Default aux slot programs (command-bank-design.md slot roles)
AUX_SLOT0_DEFAULT = [rhd_write(3, 0x02)]                # RT slot: Reg-3 carrier (rewritten by shadow)
AUX_SLOT1_DEFAULT = [rhd_convert(32), rhd_convert(33), rhd_convert(34)]  # accel @ 10 kHz
AUX_SLOT2_DEFAULT = [rhd_convert(48), rhd_convert(49),  # supply, temp
                     rhd_read(63), rhd_read(62),        # chip ID, #amps
                     rhd_read(40), rhd_read(41), rhd_read(42), rhd_read(43), rhd_read(44)]  # 'INTAN'

# ACK status codes
ACK_SUCCESS = 0x06
ACK_ERROR = 0x15

# Cable test constants - ALWAYS use all channels for cable test
CABLE_TEST_CHANNEL_ENABLE = 0x0F  # All channels always
CABLE_TEST_PACKET_SIZE_WORDS = 84  # 14-word unified header + 70 data words
CABLE_TEST_PACKET_SIZE_BYTES = CABLE_TEST_PACKET_SIZE_WORDS * 4

# Cable test globals
cable_test_mode = False
cable_test_packets_captured = 0
manual_cable_test_mode = False

# ============================================================================
# AUTOMATED CABLE DETECTION CLASSES
# ============================================================================

@dataclass
class PhaseResult:
    phase: int
    cipo0_score: float
    cipo1_score: float
    intan_pattern_cipo0: List[int]
    intan_pattern_cipo1: List[int]
    miso_register_cipo0: int
    miso_register_cipo1: int
    cipo0_valid: bool
    cipo1_valid: bool
    cipo0_has_ddr: bool = False
    cipo1_has_ddr: bool = False

@dataclass
class DetectionResult:
    success: bool
    chips_detected: bool
    best_phase0: int
    best_phase1: int
    optimal_channel_mask: int
    best_cipo0_score: float
    best_cipo1_score: float
    cipo0_present: bool
    cipo1_present: bool
    cipo0_has_ddr: bool
    cipo1_has_ddr: bool
    all_results: List[PhaseResult]
    
    def get_channel_summary(self) -> str:
        if not self.chips_detected:
            return "No chips detected"
        
        channels = []
        if self.cipo0_present:
            if self.cipo0_has_ddr:
                channels.append("CIPO0 (Regular + DDR)")
            else:
                channels.append("CIPO0 (Regular only)")
        
        if self.cipo1_present:
            if self.cipo1_has_ddr:
                channels.append("CIPO1 (Regular + DDR)")
            else:
                channels.append("CIPO1 (Regular only)")
        
        return f"Active channels: {', '.join(channels)}" if channels else "Chips detected but channels unclear"
    
    def get_recommendation(self) -> str:
        if not self.success:
            return "Detection failed. Check connections and try manual configuration."
        
        if not self.chips_detected:
            return ("No Intan chips detected. Verify:\n"
                   "  - SPI cable connections\n"
                   "  - Chip power supply\n"
                   "  - Cable integrity")
        
        confidence = "High" if self.best_score > 50 else "Medium"
        return (f"Recommended configuration:\n"
               f"  Phase0: {self.best_phase0}\n"
               f"  Phase1: {self.best_phase1}\n"
               f"  Channel mask: 0x{self.optimal_channel_mask:X}\n"
               f"  {self.get_channel_summary()}\n"
               f"  Detection confidence: {confidence}")

"""
Simplified automated cable detection for Intan interface
Reduced from ~500 lines to ~250 lines while maintaining functionality
"""

import queue
import time
import struct
from typing import List, Tuple, Optional
from dataclasses import dataclass

# Cable test uses all channels (for detection only)
CABLE_TEST_CHANNEL_ENABLE = 0x0F
CABLE_TEST_PACKET_SIZE_WORDS = 84
CABLE_TEST_PACKET_SIZE_BYTES = CABLE_TEST_PACKET_SIZE_WORDS * 4

# Expected patterns and chip IDs
INTAN_PATTERN = [0x0049, 0x004E, 0x0054, 0x0041, 0x004E]  # 'I', 'N', 'T', 'A', 'N'
CHIP_ID_DDR = 4        # RHD2164 with DDR
CHIP_ID_NO_DDR = 1     # RHD2132 without DDR
MISO_REG_DDR = 0x35    # MISO register regular word when DDR available
MISO_DDR_DDR = 0x3A    # MISO register DDR word when DDR available
MISO_NO_DDR = 0x00     # MISO register when no DDR

"""
Simplified automated cable detection for Intan interface
Reduced from ~500 lines to ~250 lines while maintaining functionality
"""

import queue
import time
import struct
from typing import List, Tuple, Optional
from dataclasses import dataclass

# Cable test uses all channels (for detection only)
CABLE_TEST_CHANNEL_ENABLE = 0x0F
CABLE_TEST_PACKET_SIZE_WORDS = 84
CABLE_TEST_PACKET_SIZE_BYTES = CABLE_TEST_PACKET_SIZE_WORDS * 4

# Expected patterns and chip IDs
INTAN_PATTERN = [0x0049, 0x004E, 0x0054, 0x0041, 0x004E]  # 'I', 'N', 'T', 'A', 'N'
CHIP_ID_DDR = 4        # RHD2164 with DDR
CHIP_ID_NO_DDR = 1     # RHD2132 without DDR
MISO_REG_DDR = 0x35    # MISO register regular word when DDR available
MISO_DDR_DDR = 0x3A    # MISO register DDR word when DDR available
MISO_NO_DDR = 0x00     # MISO register when no DDR

@dataclass
class PhaseResult:
    phase: int
    cipo0_score: float
    cipo1_score: float
    cipo0_has_ddr: bool
    cipo1_has_ddr: bool

@dataclass
class DetectionResult:
    success: bool
    best_phase: int
    optimal_channel_mask: int
    cipo0_detected: bool
    cipo1_detected: bool
    cipo0_has_ddr: bool
    cipo1_has_ddr: bool
    all_phases: List[PhaseResult]
    
    def summary(self) -> str:
        if not self.success:
            return "No chips detected. Check SPI connections and power supply."
        
        channels = []
        if self.cipo0_detected:
            channels.append(f"CIPO0 ({'DDR' if self.cipo0_has_ddr else 'Regular only'})")
        if self.cipo1_detected:
            channels.append(f"CIPO1 ({'DDR' if self.cipo1_has_ddr else 'Regular only'})")
        
        return (f" Chips detected!\n"
                f"  Phase: {self.best_phase}\n"
                f"  Channels: {', '.join(channels)}\n"
                f"  Channel mask: 0x{self.optimal_channel_mask:X}")


class CableDetection:
    def __init__(self, send_tcp_command_func):
        """
        Initialize with a command function that takes (cmd_id, param1, param2)
        and returns (success: bool, data: Optional[bytes])
        """
        self.send_cmd = send_tcp_command_func
        self.packet_queue = queue.Queue()
        self.capturing = False
    
    def capture_packet(self, words: List[int]):
        """Callback for UDP validator to provide packets during detection"""
        if self.capturing:
            try:
                self.packet_queue.put_nowait(list(words))
            except queue.Full:
                pass
    
    def detect(self, verbose=False) -> DetectionResult:
        """Run automated detection and return results"""
        
        result = DetectionResult(
            success=False, best_phase=0, optimal_channel_mask=0,
            cipo0_detected=False, cipo1_detected=False,
            cipo0_has_ddr=False, cipo1_has_ddr=False, all_phases=[]
        )
        
        try:
            if verbose:
                print("[Detection] Starting automated cable detection...")
            
            # Initialize and configure
            if not self._initialize_chips(verbose):
                return result
            
            # Test all phases
            best_score = -1000
            for phase in range(16):
                if verbose:
                    print(f"[Detection] Testing phase {phase}...")
                
                phase_result = self._test_phase(phase, verbose)
                result.all_phases.append(phase_result)
                
                # Only consider phases where at least one channel is detected (score > 60)
                cipo0_valid = phase_result.cipo0_score > 60
                cipo1_valid = phase_result.cipo1_score > 60
                
                if cipo0_valid or cipo1_valid:
                    # For valid detections, use sum of scores
                    total_score = phase_result.cipo0_score + phase_result.cipo1_score
                    if total_score > best_score:
                        best_score = total_score
                        result.best_phase = phase
                        result.cipo0_detected = cipo0_valid
                        result.cipo1_detected = cipo1_valid
                        result.cipo0_has_ddr = phase_result.cipo0_has_ddr
                        result.cipo1_has_ddr = phase_result.cipo1_has_ddr
            
            # Calculate success and channel mask
            result.success = result.cipo0_detected or result.cipo1_detected
            
            if result.success:
                result.optimal_channel_mask = 0
                if result.cipo0_detected:
                    result.optimal_channel_mask |= 0x01  # CIPO0 regular
                    if result.cipo0_has_ddr:
                        result.optimal_channel_mask |= 0x02  # CIPO0 DDR
                if result.cipo1_detected:
                    result.optimal_channel_mask |= 0x04  # CIPO1 regular
                    if result.cipo1_has_ddr:
                        result.optimal_channel_mask |= 0x08  # CIPO1 DDR
            
            if verbose:
                print(f"[Detection] Complete: {result.summary()}")
        
        except Exception as e:
            if verbose:
                print(f"[Detection] Error: {e}")
        
        return result
    
    def apply_config(self, result: DetectionResult) -> bool:
        """Apply detected configuration to device"""
        if not result.success:
            return False
        
        CMD_SET_PHASE = 0x11
        CMD_SET_CHANNEL_ENABLE = 0x13
        
        return (self.send_cmd(CMD_SET_PHASE, result.best_phase, result.best_phase)[0] and
                self.send_cmd(CMD_SET_CHANNEL_ENABLE, result.optimal_channel_mask)[0])
    
    def _initialize_chips(self, verbose) -> bool:
        """Initialize chips for testing"""
        CMD_STOP = 0x02
        CMD_START = 0x01
        CMD_SET_LOOP_COUNT = 0x10
        CMD_LOAD_INIT = 0x21
        CMD_LOAD_CABLE_TEST = 0x22
        CMD_SET_CHANNEL_ENABLE = 0x13
        
        if verbose:
            print("[Detection] Initializing chips...")
        
        # Stop, set loop count, enable all channels
        if not (self.send_cmd(CMD_STOP)[0] and
                self.send_cmd(CMD_SET_LOOP_COUNT, 1)[0] and
                self.send_cmd(CMD_SET_CHANNEL_ENABLE, CABLE_TEST_CHANNEL_ENABLE)[0]):
            return False
        
        # Load and run initialization sequence
        if not self.send_cmd(CMD_LOAD_INIT)[0]:
            return False
        
        if not self.send_cmd(CMD_START)[0]:
            return False
        time.sleep(0.1)
        self.send_cmd(CMD_STOP)
        
        # Load cable test sequence
        return self.send_cmd(CMD_LOAD_CABLE_TEST)[0]
    
    def _test_phase(self, phase: int, verbose: bool) -> PhaseResult:
        """Test a single phase configuration"""
        CMD_SET_PHASE = 0x11
        CMD_START = 0x01
        CMD_STOP = 0x02
        
        result = PhaseResult(
            phase=phase, cipo0_score=0, cipo1_score=0,
            cipo0_has_ddr=False, cipo1_has_ddr=False
        )
        
        try:
            # Set phase
            if not self.send_cmd(CMD_SET_PHASE, phase, phase)[0]:
                return result
            time.sleep(0.01)
            
            # Capture packet
            self.capturing = True
            self.send_cmd(CMD_START)
            
            try:
                packet = self.packet_queue.get(timeout=2.0)
            except queue.Empty:
                return result
            finally:
                self.send_cmd(CMD_STOP)
                self.capturing = False
            
            # Score packet
            result.cipo0_score, result.cipo0_has_ddr = self._score_channel(packet, 0, verbose)
            result.cipo1_score, result.cipo1_has_ddr = self._score_channel(packet, 1, verbose)
            
            if verbose and (result.cipo0_score > 0 or result.cipo1_score > 0):
                print(f"  Phase {phase}: CIPO0={result.cipo0_score:.0f}, CIPO1={result.cipo1_score:.0f}")
        
        except Exception as e:
            if verbose:
                print(f"  Error testing phase {phase}: {e}")
        
        return result
    
    def _score_channel(self, packet: List[int], channel: int, verbose: bool) -> Tuple[float, bool]:
        """
        Score a channel (0=CIPO0, 1=CIPO1) from packet data
        
        Packet structure: [Header(4)] + [Data(70)]
        Data words alternate: CIPO0, CIPO1, CIPO0, CIPO1, ...
        Each word: [Regular(15:0), DDR(31:16)]
        
        Cable test reads (with 2-cycle pipeline delay):
          Cycles 0-4: INTAN pattern -> appears at data indices 2-6
          Cycle 5: Chip ID -> appears at data index 7
          Cycle 6: MISO register -> appears at data index 8
        """
        if len(packet) < CABLE_TEST_PACKET_SIZE_WORDS:
            return 0.0, False
        
        score = 0.0
        has_ddr = False
        
        # Extract this channel's data (every other word, starting at channel offset).
        # The unified broadband header grew from 10 to 14 words; this empirical
        # offset tracks that growth (+4) so it points at the same data position it
        # always did (cable detection runs only with a physical chip attached).
        data_words = packet[8:]  # skip relative to the unified header
        channel_words = [data_words[i] for i in range(channel, 70, 2)]  # Get every other word
        
        if len(channel_words) < 9:
            return 0.0, False
        
        # Extract regular and DDR streams
        regular = [w & 0xFFFF for w in channel_words]
        ddr = [(w >> 16) & 0xFFFF for w in channel_words]
        
        # Score INTAN pattern (indices 2-6 due to 2-cycle pipeline delay)
        intan_found = []
        for i, expected in enumerate(INTAN_PATTERN):
            idx = i + 2  # Pipeline delay
            if idx < len(regular):
                intan_found.append(regular[idx])
                if regular[idx] == expected:
                    score += 10
        
        # Check chip ID (index 7)
        if len(regular) > 7 and len(ddr) > 7:
            chip_id_reg = regular[7]
            chip_id_ddr = ddr[7]
            
            if chip_id_reg == CHIP_ID_DDR and chip_id_ddr == CHIP_ID_DDR:
                has_ddr = True
                score += 10
            elif chip_id_reg == CHIP_ID_NO_DDR:
                score += 10
        
        # Check MISO register (index 8)
        if len(regular) > 8 and len(ddr) > 8:
            miso_reg = regular[8]
            miso_ddr = ddr[8]
            
            if has_ddr and miso_reg == MISO_REG_DDR and miso_ddr == MISO_DDR_DDR:
                score += 10
            elif not has_ddr and miso_reg == MISO_NO_DDR:
                score += 10
        
        if verbose and score > 60:
            pattern_str = ''.join(chr(x) if 32 <= x <= 126 else '?' for x in intan_found)
            ddr_str = "DDR" if has_ddr else "No DDR"
            print(f"    CIPO{channel}: '{pattern_str}' ({ddr_str})")
        
        return score, has_ddr

def calculate_data_words(channel_enable):
    """Number of 32-bit data words for the 8-bit channel_enable mask.
    [3:0] = port 0 streams, [7:4] = port 1 (dual cable)."""
    num_channels = bin(channel_enable & 0xFF).count('1')
    if num_channels == 0:
        return 70
    total_16bit_words = 35 * num_channels
    return (total_16bit_words + 1) // 2

# Broadband framing (unified format): 8-word common header + 6-word broadband
# sub-block = 14 header words ahead of the data (docs/unified-packet-format.md).
BB_HEADER_WORDS = 14

def calculate_packet_size(channel_enable):
    """Total broadband packet size in words (header + data). Up to 14 + 140 = 154."""
    return BB_HEADER_WORDS + calculate_data_words(channel_enable)

# ---------------------------------------------------------------------------
# Debug-mode sine reference model.
#
# In debug mode the PL fills every CIPO line with a synthetic sine, so we can
# verify the data path end-to-end with NO chip attached. This mirrors
# data_generator_core.sv exactly (proven bit-identical by
# programmable_logic/sim/dualport_dropout_tb.sv):
#
#   sine_lut[i] = rtoi(32767.0/16 * sin(2*pi*i/512) + 32767.0)     i in 0..511
#   per acquisition cycle c (0..34):  coff = max(c-2, 0)
#       base_phase   bp  = (dummy_data_index + coff) & 0x1FF
#       base_phase_p1 b1 = (bp + 128) & 0x1FF
#   8 segments, in channel_enable bit order:
#       bit0 A_CIPO0_REG=lut[bp]      bit1 A_CIPO0_DDR=lut[bp<<1]
#       bit2 A_CIPO1_REG=lut[bp<<2]   bit3 A_CIPO1_DDR=lut[bp<<3]
#       bit4 B_CIPO0_REG=lut[b1]      bit5 B_CIPO0_DDR=lut[b1<<1]
#       bit6 B_CIPO1_REG=lut[b1<<2]   bit7 B_CIPO1_DDR=lut[b1<<3]
# dummy_data_index increments by 1 per packet, in lockstep with the timestamp,
# so after locking it on one packet we predict it for any other from the
# timestamp delta -- which lets us tell genuine corruption (wrong values) apart
# from packet loss (correct values, missing timestamps).
# ---------------------------------------------------------------------------

# Human-readable name for each enable bit (== segment-within-cycle order).
SEG_NAMES = ["A_CIPO0_REG", "A_CIPO0_DDR", "A_CIPO1_REG", "A_CIPO1_DDR",
             "B_CIPO0_REG", "B_CIPO0_DDR", "B_CIPO1_REG", "B_CIPO1_DDR"]

def build_sine_lut():
    """512-entry unsigned-16-bit sine LUT, identical to the RTL initial block.
    SV $rtoi truncates toward zero; the argument is always positive here, so
    Python int() matches it exactly."""
    return [int(32767.0 / 16.0 * math.sin(2.0 * math.pi * i / 512.0) + 32767.0) & 0xFFFF
            for i in range(512)]

def _debug_seg_values(bp, lut):
    """The 8 segment values (bit0..bit7) for a given base phase bp."""
    b1 = (bp + 128) & 0x1FF
    return [lut[bp],                  lut[(bp << 1) & 0x1FF],
            lut[(bp << 2) & 0x1FF],   lut[(bp << 3) & 0x1FF],
            lut[b1],                  lut[(b1 << 1) & 0x1FF],
            lut[(b1 << 2) & 0x1FF],   lut[(b1 << 3) & 0x1FF]]

def expected_debug_segments(channel_enable, ddi, lut):
    """Flat list of expected 16-bit segments for one debug packet at index ddi,
    plus matching (cycle, bit) metadata. Order == the PL's tight packing."""
    enabled_bits = [b for b in range(8) if channel_enable & (1 << b)]
    segs, meta = [], []
    for cycle in range(35):
        coff = (cycle - 2) if cycle >= 2 else 0
        vals = _debug_seg_values((ddi + coff) & 0x1FF, lut)
        for b in enabled_bits:
            segs.append(vals[b])
            meta.append((cycle, b))
    return segs, meta

def unpack_data_segments(data_words, n_seg):
    """Unpack n_seg tightly-packed 16-bit segments (low half first) from the
    32-bit data words of a packet."""
    segs = []
    for w in data_words:
        segs.append(w & 0xFFFF)
        segs.append((w >> 16) & 0xFFFF)
    return segs[:n_seg]

def channel_enable_to_string(channel_enable):
    """Convert channel enable bits to human readable string (both ports)"""
    names = ["A_CIPO0_REG", "A_CIPO0_DDR", "A_CIPO1_REG", "A_CIPO1_DDR",
             "B_CIPO0_REG", "B_CIPO0_DDR", "B_CIPO1_REG", "B_CIPO1_DDR"]
    channels = [names[b] for b in range(8) if channel_enable & (1 << b)]
    return ", ".join(channels) if channels else "NONE"

def get_local_ip():
    """Get the local IP address that can reach the Zynq"""
    try:
        # Create a socket to determine which interface would be used
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((ZYNQ_IP, TCP_PORT))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "127.0.0.1"

class DataValidator:
    def __init__(self):
        self.last_timestamp = None
        self.packet_count = 0
        self.error_count = 0
        self.start_time = None
        self.timestamp_errors = 0
        self.magic_errors = 0
        self.size_errors = 0
        # Per-stream SEQ continuity = the broadband loss check (unified format,
        # header word 4). MUST stay 0 for the archival broadband stream.
        self.last_seq = None
        self.seq_gaps = 0          # broadband gap count (the no-loss assertion)
        self.seq_lost_packets = 0  # total packets implied missing by the gaps
        self.last_stats_time = None
        self.last_packet_count = 0
        self.last_packet_raw = None
        self.last_packet_words = None
        self.current_channel_enable = 0x0F
        self.expected_packet_size_bytes = calculate_packet_size(0x0F) * 4
        self.expected_packet_size_words = calculate_packet_size(0x0F)
        self._manual_queue = queue.Queue()
        self._manual_lock = threading.Lock()

        # Cable detection integration
        self.cable_detector = None

        # Debug-sine verification capture
        self.sine_capture_active = False
        self.sine_capture_target = 0
        self.sine_capture_ce = 0xFF
        self.sine_capture = []   # list of (timestamp, tuple-of-data-words)

    def set_cable_detector(self, detector):
        """Set cable detector for packet capture integration"""
        self.cable_detector = detector

    def set_channel_enable(self, channel_enable):
        """Update channel enable setting and recalculate packet sizes"""
        self.current_channel_enable = channel_enable
        self.expected_packet_size_words = calculate_packet_size(channel_enable)
        self.expected_packet_size_bytes = self.expected_packet_size_words * 4
        print(f"[INFO] Channel enable updated to 0x{channel_enable:X}")
        print(f"[INFO] Enabled channels: {channel_enable_to_string(channel_enable)}")
        print(f"[INFO] Expected packet size: {self.expected_packet_size_words} words ({self.expected_packet_size_bytes} bytes)")

    def start_cable_test_capture(self):
        global cable_test_mode, cable_test_packets_captured
        cable_test_mode = True
        cable_test_packets_captured = 0
        print("Starting cable test packet capture...")

    def start_manual_cable_test(self):
        global manual_cable_test_mode
        manual_cable_test_mode = True
        with self._manual_lock:
            while not self._manual_queue.empty():
                try:
                    self._manual_queue.get_nowait()
                except queue.Empty:
                    break
        print("Manual cable test mode started")

    def wait_for_manual_packet(self, timeout=5.0):
        try:
            return self._manual_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_manual_test_packets(self):
        global manual_cable_test_mode
        packets = []
        with self._manual_lock:
            while not self._manual_queue.empty():
                try:
                    packets.append(self._manual_queue.get_nowait())
                except queue.Empty:
                    break
        manual_cable_test_mode = False
        return packets
        
    def start_sine_capture(self, channel_enable, n_packets):
        """Begin stashing data words for debug-sine verification."""
        self.sine_capture_ce = channel_enable
        self.sine_capture_target = n_packets
        self.sine_capture = []
        self.sine_capture_active = True

    def analyze_sine_capture(self, write_ptr=None):
        """Verify the captured debug packets against the RTL sine reference.
        Reports value correctness (per CIPO line / REG-DDR) and packet loss
        (from timestamp gaps), and tells the two apart. write_ptr (optional) is
        the BRAM word write pointer read at stop, used to print absolute BRAM
        addresses for the most-recent corrupted samples."""
        cap = self.sine_capture
        ce = self.sine_capture_ce
        if len(cap) < 2:
            print("[SINE] Not enough packets captured to verify "
                  f"({len(cap)}). Is the board streaming in debug mode?")
            return

        lut = build_sine_lut()
        ndw = calculate_data_words(ce)
        n_seg = 35 * bin(ce & 0xFF).count('1')
        enabled_bits = [b for b in range(8) if ce & (1 << b)]

        ts0 = cap[0][0]
        # Lock the debug index jointly over the first packets (robust to a
        # corrupted lead packet): pick ddi0 minimizing total mismatch when each
        # packet's index is predicted from its timestamp delta.
        lock_n = min(len(cap), 20)
        exp_cache = {}
        def exp_segs(ddi):
            if ddi not in exp_cache:
                exp_cache[ddi] = expected_debug_segments(ce, ddi, lut)[0]
            return exp_cache[ddi]
        best_ddi0, best_total = 0, None
        for ddi0 in range(512):
            tot = 0
            for ts, dw in cap[:lock_n]:
                ddi = (ddi0 + (ts - ts0)) & 0x1FF
                got = unpack_data_segments(dw, n_seg)
                exp = exp_segs(ddi)
                tot += sum(1 for a, b in zip(got, exp) if a != b)
                if best_total is not None and tot >= best_total:
                    break
            if best_total is None or tot < best_total:
                best_total, best_ddi0 = tot, ddi0

        # Single-segment reference value (for alias checks below).
        def seg_val(ddi, cyc, bit):
            return _debug_seg_values((ddi + (cyc - 2 if cyc >= 2 else 0)) & 0x1FF, lut)[bit]
        lut_lo, lut_hi = min(lut), max(lut)

        # Verify every captured packet at its predicted index.
        _, meta = expected_debug_segments(ce, 0, lut)   # (cycle,bit) layout is index-independent
        seg_bad = {b: 0 for b in enabled_bits}
        cycle_bad = [0] * 35
        total_seg = 0
        total_bad = 0
        exact_pkts = 0
        mism_list = []        # (pkt_idx, word_in_pkt, ddi, cyc, bit, exp, got)
        for pkt_idx, (ts, dw) in enumerate(cap):
            ddi = (best_ddi0 + (ts - ts0)) & 0x1FF
            got = unpack_data_segments(dw, n_seg)
            exp = exp_segs(ddi)
            bad = 0
            for i in range(n_seg):
                total_seg += 1
                if got[i] != exp[i]:
                    bad += 1
                    cyc, bit = meta[i]
                    seg_bad[bit] += 1
                    cycle_bad[cyc] += 1
                    word_in_pkt = 10 + (i // 2)   # 10 header words, then 2 segs/word
                    if len(mism_list) < 20000:
                        mism_list.append((pkt_idx, word_in_pkt, ddi, cyc, bit, exp[i], got[i]))
            total_bad += bad
            if bad == 0:
                exact_pkts += 1

        # Packet loss from timestamp span vs packets received.
        ts_span = cap[-1][0] - ts0 + 1
        received = len(cap)
        dropped = max(0, ts_span - received)
        loss_pct = 100.0 * dropped / ts_span if ts_span > 0 else 0.0

        print("\n" + "=" * 64)
        print("  DEBUG SINE VERIFICATION")
        print("=" * 64)
        print(f"  channel_enable : 0x{ce:02X}  ({channel_enable_to_string(ce)})")
        print(f"  packets verified: {received}   data words/packet: {ndw}")
        print(f"  debug index lock: ddi0={best_ddi0}  (advances 1/packet w/ timestamp)")
        print("-" * 64)
        # Value correctness
        if total_bad == 0:
            print(f"  VALUE CHECK : PASS - all {total_seg} samples bit-exact vs RTL reference")
        else:
            # Classify each mismatch by magnitude and by "alias" (does the wrong
            # value equal a neighbouring cycle/packet's correct value -> stale data).
            d1 = d_small = d_big = oor = 0          # |delta|==1, 2..16, >16, out-of-LUT-range
            a_prevpkt = a_nextpkt = a_prevcyc = a_nextcyc = a_otherbit = 0
            for (pkt_idx, wip, ddi, cyc, bit, e, g) in mism_list:
                ad = abs(g - e)
                if not (lut_lo <= g <= lut_hi):
                    oor += 1
                if ad == 1:
                    d1 += 1
                elif ad <= 16:
                    d_small += 1
                else:
                    d_big += 1
                if ad > 1:   # only chase aliases for non-rounding errors
                    if g == seg_val((ddi - 1) & 0x1FF, cyc, bit):       a_prevpkt += 1
                    elif g == seg_val((ddi + 1) & 0x1FF, cyc, bit):     a_nextpkt += 1
                    elif cyc > 0  and g == seg_val(ddi, cyc - 1, bit):  a_prevcyc += 1
                    elif cyc < 34 and g == seg_val(ddi, cyc + 1, bit):  a_nextcyc += 1
                    elif any(g == seg_val(ddi, cyc, ob) for ob in enabled_bits if ob != bit):
                        a_otherbit += 1
            sampled = len(mism_list)
            real = d_small + d_big   # |delta| > 1 == not LSB rounding

            print(f"  VALUE CHECK : FAIL - {total_bad}/{total_seg} samples wrong "
                  f"({100.0*total_bad/total_seg:.3f}%), {exact_pkts}/{received} packets perfect")
            print(f"  magnitude   : |d|=1: {d1}   |d|=2..16: {d_small}   "
                  f"|d|>16: {d_big}   outside-LUT: {oor}   (of {sampled} sampled)")
            print(f"  real errors (|d|>1): {real}  -> "
                  f"{'NONE - all diffs are 1 LSB' if real == 0 else 'see breakdown'}")
            # Breakdown of the REAL (non-rounding) errors only.
            if real > 0:
                rc = [0] * 35
                rb = {b: 0 for b in enabled_bits}
                word_hist = {}
                for (pkt_idx, wip, ddi, cyc, bit, e, g) in mism_list:
                    if abs(g - e) > 1:
                        rc[cyc] += 1
                        rb[bit] += 1
                        word_hist[wip] = word_hist.get(wip, 0) + 1
                print("  real by channel: " +
                      "  ".join(f"{SEG_NAMES[b]}={rb[b]}" for b in enabled_bits if rb[b]))
                worst = sorted(((rc[c], c) for c in range(35) if rc[c]), reverse=True)[:8]
                print("  real worst cyc : " +
                      ", ".join(f"cyc{c}({n})" for n, c in worst))
                # Packet-WORD-offset histogram: corruption that is fixed at a
                # buffer/transaction offset (not a cycle) shows up here as a tight
                # cluster of word indices, independent of channel_enable.
                wtop = sorted(word_hist.items(), key=lambda kv: -kv[1])[:10]
                print("  real worst word: " +
                      ", ".join(f"w{w}({n})" for w, n in sorted(wtop)))
                if max(a_prevpkt, a_nextpkt, a_prevcyc, a_nextcyc, a_otherbit) > 0:
                    print(f"  alias of real  : prev-pkt={a_prevpkt} next-pkt={a_nextpkt} "
                          f"prev-cyc={a_prevcyc} next-cyc={a_nextcyc} other-chan={a_otherbit}")
                # ---- BRAM navigation: where does the MOST RECENT corruption sit
                #      relative to the end of the capture (= the PL write pointer
                #      when streaming stopped)? words_back = how many BRAM words to
                #      step backward from the final write pointer to reach it. ----
                last_idx = received - 1
                recent = [m for m in mism_list if abs(m[6] - m[5]) > 1]
                recent_pkt = max(m[0] for m in recent)
                rmism = sorted([m for m in recent if m[0] == recent_pkt], key=lambda m: m[1])
                pkts_from_end = last_idx - recent_pkt
                print(f"  --- most-recent corruption: {pkts_from_end} packet(s) before the "
                      f"end of capture ---")
                print(f"      (final captured packet = index {last_idx}; PL write pointer at "
                      f"stop is just past it)")
                psize = self.expected_packet_size_words
                for (pkt_idx, wip, ddi, cyc, bit, e, g) in rmism[:8]:
                    words_back = pkts_from_end * psize + (psize - wip)
                    if write_ptr is not None:
                        abs_word = (write_ptr - words_back) % 16384   # BRAM = 16384 words
                        addr = 0x80000000 + abs_word * 4
                        win = (abs_word - 24) % 16384   # widen: sub-packet capture/stop gap
                        loc = (f"~BRAM word {abs_word} = 0x{addr:08X}  "
                               f"(dump_bram {win} 48  -- search this window)")
                    else:
                        loc = f"~{words_back} words before write ptr (wrptr-{words_back})"
                    print(f"      pkt-{pkts_from_end:>2} word {wip:>3} {SEG_NAMES[bit]:<12} "
                          f"0x{e:04X}->0x{g:04X}   {loc}")
                print("  examples (pkt#-from-end, word, chan, expected -> got, delta):")
                shown = 0
                for (pkt_idx, wip, ddi, cyc, bit, e, g) in mism_list:
                    if abs(g - e) > 1:
                        print(f"      pkt-{last_idx - pkt_idx:>3} word{wip:>3} {SEG_NAMES[bit]:<12} "
                              f"0x{e:04X} -> 0x{g:04X}  ({g - e:+d})")
                        shown += 1
                        if shown >= 12:
                            break
        print("-" * 64)
        # Packet loss
        verb = "PASS" if dropped == 0 else "LOSS"
        print(f"  PACKET LOSS : {verb} - received {received} of {ts_span} "
              f"expected over the capture ({dropped} dropped, {loss_pct:.2f}%)")
        print("-" * 64)
        # Interpretation
        real_errs = locals().get('real', 0)
        if total_bad == 0 and dropped == 0:
            print("  => Clean: the board generates and transmits debug data correctly.")
        elif total_bad == 0 and dropped > 0:
            print("  => Values are CORRECT but packets are being DROPPED. The data path")
            print("     is fine; this is transport loss (host socket buffer / rate).")
            print("     Try a bigger SO_RCVBUF, or compare board udp_packets_sent vs here.")
        elif total_bad > 0 and real_errs == 0:
            print("  => All differences are exactly 1 LSB. This is the synthesis-vs-")
            print("     simulation rounding of the sine ROM ($rtoi/$sin), NOT corruption.")
            print("     The transmitted sinewave is correct to within one count.")
            if dropped > 0:
                print(f"     (But {dropped} packets were also dropped - see PACKET LOSS.)")
        else:
            print("  => Genuine corruption: values differ by more than 1 LSB. See the")
            print("     real-error breakdown/aliases above. If they alias to a neighbour")
            print("     cycle/packet it is stale data (timing/FIFO); if random/out-of-LUT")
            print("     it is a bit error. Capture the BRAM (dump_bram) to localize.")
        print("=" * 64 + "\n")

    def validate_packet(self, data):
        global cable_test_mode, cable_test_packets_captured, manual_cable_test_mode

        self.packet_count += 1
        self.last_packet_raw = data

        if cable_test_mode:
            if cable_test_packets_captured < 17:
                if len(data) == CABLE_TEST_PACKET_SIZE_BYTES:
                    words = struct.unpack(f'<{CABLE_TEST_PACKET_SIZE_WORDS}I', data)
                    self.last_packet_words = words
                    if cable_test_packets_captured == 0:
                        print(f"Packet {cable_test_packets_captured + 1} (Init): Word 12: 0x{words[12]:08X}, Word 13: 0x{words[13]:08X}")
                    else:
                        phase1 = cable_test_packets_captured - 1
                        print(f"Packet {cable_test_packets_captured + 1} (Phase1={phase1}): Word 12: 0x{words[12]:08X}, Word 13: 0x{words[13]:08X}")
                cable_test_packets_captured += 1
                if cable_test_packets_captured >= 17:
                    cable_test_mode = False
                    print("Cable test capture complete.")
                return None

        if manual_cable_test_mode:
            if len(data) == CABLE_TEST_PACKET_SIZE_BYTES:
                words = struct.unpack(f'<{CABLE_TEST_PACKET_SIZE_WORDS}I', data)
                try:
                    self._manual_queue.put_nowait(words)
                except queue.Full:
                    pass
                print("Captured manual test packet")
            return None

        if self.start_time is None:
            self.start_time = time.time()
            self.last_stats_time = self.start_time

        if len(data) != self.expected_packet_size_bytes:
            self.size_errors += 1
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Wrong size {len(data)}, expected {self.expected_packet_size_bytes}")
            return None

        try:
            words = struct.unpack(f'<{self.expected_packet_size_words}I', data)
            self.last_packet_words = words

            # Feed packets to cable detector if active (only for cable test packets)
            if self.cable_detector and len(data) == CABLE_TEST_PACKET_SIZE_BYTES:
                self.cable_detector.capture_packet(words)

            # Unified header: w0 = MAGIC (0xCAFEBABE), w1 = TYPE_VER (low byte =
            # stream_type, must be BROADBAND here; next byte = version).
            stream_type = words[1] & 0xFF
            version = (words[1] >> 8) & 0xFF
            if words[0] != UNIFIED_MAGIC or stream_type != STREAM_TYPE_BROADBAND:
                self.magic_errors += 1
                self.error_count += 1
                print(f"[ERROR] Packet {self.packet_count}: header mismatch "
                      f"(magic=0x{words[0]:08X} type={stream_type} ver={version})")
                return None

            timestamp = (words[3] << 32) | words[2]
            seq = words[4]   # per-stream broadband sequence (the loss check)

            # SEQ continuity check: each broadband packet's SEQ must be exactly
            # +1 (mod 2^32) from the previous. A gap = lost broadband packet(s).
            if self.last_seq is not None:
                expected_seq = (self.last_seq + 1) & 0xFFFFFFFF
                if seq != expected_seq:
                    self.seq_gaps += 1
                    self.error_count += 1
                    missing = (seq - expected_seq) & 0xFFFFFFFF
                    self.seq_lost_packets += missing
                    print(f"[LOSS] Broadband SEQ gap: expected {expected_seq}, got "
                          f"{seq} (+{missing} missing). gap_count={self.seq_gaps}")
            self.last_seq = seq

            # Debug-sine capture: stash data words for offline verification.
            if self.sine_capture_active and len(self.sine_capture) < self.sine_capture_target:
                self.sine_capture.append((timestamp, words[BB_HEADER_WORDS:]))
                if len(self.sine_capture) >= self.sine_capture_target:
                    self.sine_capture_active = False

            now = time.time()
            if self.packet_count % 30000 == 0 or (now - self.last_stats_time) >= 5.0:
                elapsed = now - self.start_time
                total_rate = self.packet_count / elapsed if elapsed > 0 else 0
                inst_rate = (self.packet_count - self.last_packet_count) / (now - self.last_stats_time) if (now - self.last_stats_time) > 0 else 0
                
                if len(words) >= BB_HEADER_WORDS + 4:
                    h = BB_HEADER_WORDS
                    data_sample = (f"Data: [0x{words[h]:08X}, 0x{words[h+1]:08X}, "
                                   f"0x{words[h+2]:08X}, 0x{words[h+3]:08X}]")
                else:
                    data_sample = f"Data: [packet too short]"

                print(f"[INFO] Packet {self.packet_count}: ts={timestamp} seq={seq}, "
                      f"Rate: {total_rate:.1f} pkt/s (avg), {inst_rate:.1f} pkt/s (inst), "
                      f"Errors: {self.error_count} (seq_gaps={self.seq_gaps})")
                print(f"       {data_sample}")
                
                self.last_stats_time = now
                self.last_packet_count = self.packet_count

            return timestamp

        except struct.error as e:
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Failed to unpack: {e}")
            return None

    def print_last_packet_hex(self, words_per_line=8):
        if self.last_packet_words is None:
            print("[INFO] No packets received yet")
            return
            
        print(f"\n=== LAST PACKET - HEX DUMP ===")
        print(f"Packet size: {len(self.last_packet_words)} words")
        words = self.last_packet_words
        
        for i in range(0, len(words), words_per_line):
            chunk = words[i:i+words_per_line]
            hex_words = ' '.join(f'{w:08X}' for w in chunk)
            print(f"{i:2d}: {hex_words}")

    def print_aux_info(self):
        """Decode the aux command-echo metadata of the last received packet.
        Unified broadband header:
          word6 (AUX1) = {echo_slot0[31:16], flags[15:8], digital_in[7:0]}
          word8 (sub-block) = {echo_slot2_prev[31:16], echo_slot1_prev[15:0]}
        Result locations (SPI 2-command pipeline): slot-0's result is THIS
        packet's data word 34; slot-1/2 echoes label data words 0 and 1."""
        if self.last_packet_words is None or len(self.last_packet_words) < 9:
            print("[AUX] No packet captured yet")
            return
        w_aux1, w_echo = self.last_packet_words[6], self.last_packet_words[8]
        digital_in = w_aux1 & 0xFF
        flags = (w_aux1 >> 8) & 0xFF
        if not (flags & 0x01):
            print(f"[AUX] Sequencer inactive in last packet (digital_in=0x{digital_in:02X})")
            return
        echo0 = (w_aux1 >> 16) & 0xFFFF
        echo1 = w_echo & 0xFFFF
        echo2 = (w_echo >> 16) & 0xFFFF
        print(f"[AUX] digital_in=0x{digital_in:02X}  "
              f"fast_settle={bool(flags & 0x02)} digout={bool(flags & 0x04)} "
              f"dsp={bool(flags & 0x08)} echo_valid={bool(flags & 0x10)} "
              f"inject_result_in_word1={bool(flags & 0x20)}")
        print(f"[AUX] slot0 (result @ data word 34): {rhd_decode(echo0)}")
        print(f"[AUX] slot1 (result @ data word 0):  {rhd_decode(echo1)}")
        print(f"[AUX] slot2 (result @ data word 1):  {rhd_decode(echo2)}")

    def print_statistics(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print(f"\n=== STATISTICS ===")
        print(f"Total packets: {self.packet_count}")
        print(f"Total errors: {self.error_count}")
        print(f"Elapsed time: {elapsed:.1f}s")
        print(f"Average rate: {rate:.1f} packets/second")
        if rate > 0:
            print(f"Data rate: {(rate * self.expected_packet_size_bytes * 8 / 1000000):.1f} Mbps")

validator = DataValidator()

def verify_debug_sine(sock, channel_enable=0xFF, n_packets=300):
    """Put the board in debug mode, capture N packets via the running UDP
    listener, and verify the received synthetic sinewaves against the RTL
    reference (data_generator_core.sv). Distinguishes genuine corruption from
    packet loss. Leaves the board stopped."""
    global validator
    print(f"[SINE] Debug-sine check: channel_enable=0x{channel_enable:02X}, "
          f"target {n_packets} packets")

    send_binary_command(sock, CMD_STOP)
    time.sleep(0.05)
    if not send_binary_command(sock, CMD_SET_DEBUG_MODE, 1)[0]:
        print("[SINE] Failed to enable debug mode"); return
    if not send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, channel_enable)[0]:
        print("[SINE] Failed to set channel_enable"); return
    validator.set_channel_enable(channel_enable)

    validator.start_sine_capture(channel_enable, n_packets)
    if not send_binary_command(sock, CMD_START)[0]:
        print("[SINE] Failed to start streaming"); return

    # The udp_listener daemon thread fills validator.sine_capture.
    t_end = time.time() + 10.0
    while validator.sine_capture_active and time.time() < t_end:
        time.sleep(0.05)
    send_binary_command(sock, CMD_STOP)

    if validator.sine_capture_active:
        validator.sine_capture_active = False
        print(f"[SINE] Timed out with {len(validator.sine_capture)}/{n_packets} "
              f"packets. Is UDP reaching this host (run set_udp / check the network)?")

    # Read the BRAM write pointer after stop so the analysis can give absolute
    # BRAM addresses for corrupted samples (for dump_bram correlation). NOTE: the
    # PL usually writes a few packets between the last UDP packet this host got
    # and the stop, so treat the absolute address as the centre of a small window.
    wrptr = None
    st = get_status(sock)
    if st is not None:
        wrptr = st.get('bram_write_addr')
        print(f"[SINE] BRAM write pointer at stop: {wrptr} (word) = "
              f"0x{0x80000000 + (wrptr or 0)*4:08X}")
    validator.analyze_sine_capture(write_ptr=wrptr)


# ---------------------------------------------------------------------------
# Unified UDP sink: ONE socket on port 5000 carrying ALL streams (broadband +
# LFP), demuxed by stream_type. This is the no-loss design from CLAUDE.md /
# docs/unified-packet-format.md:
#   * a tight recv->ring thread does the MINIMUM work (recvfrom -> queue), so
#     broadband is NEVER blocked while a slow consumer processes a packet;
#   * a demux thread pops from the ring, peeks header word 1 (stream_type), and
#     routes broadband -> DataValidator (which checks per-stream SEQ continuity
#     and prints the broadband gap count, which MUST be 0) and LFP -> the LFP
#     subscriber queues (receive_lfp / lfp_sweep read those).
# Big SO_RCVBUF so nothing is dropped while waiting.
# ---------------------------------------------------------------------------
class UnifiedSink:
    def __init__(self, port=UDP_PORT, rcvbuf=16 * 1024 * 1024, ring_max=200000):
        self.port = port
        self.rcvbuf = rcvbuf
        self._sock = None
        self._recv_thread = None
        self._demux_thread = None
        self._running = False
        self._ring = queue.Queue(maxsize=ring_max)
        self._ring_drops = 0           # datagrams dropped because the ring was full
        # LFP fan-out (pub/sub), like the old LfpSink
        self._lfp_subs = []
        self._lfp_last_seq = None
        self._lfp_gaps = 0
        self.lfp_pkts = 0
        self.lfp_bytes = 0
        self.bb_pkts = 0
        self.other_pkts = 0
        self.last_addr = None
        self._lock = threading.Lock()
        # last-timestamp continuity (broadband) handled inside DataValidator now

    def start(self):
        if self._running:
            return True
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.rcvbuf)
            got = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
            eff = got // 2 if sys.platform.startswith("linux") else got
            msg = f"[UDP] SO_RCVBUF = {eff // 1024} KB"
            if eff < self.rcvbuf:
                if sys.platform == "darwin":
                    msg += " (clamped; raise it: sudo sysctl -w kern.ipc.maxsockbuf=33554432)"
                elif sys.platform.startswith("linux"):
                    msg += " (clamped; raise it: sudo sysctl -w net.core.rmem_max=33554432)"
                else:
                    msg += " (clamped by the OS socket-buffer cap)"
            print(msg)
        except OSError as e:
            print(f"[UDP] Could not set SO_RCVBUF: {e}")
        try:
            sock.bind(("", self.port))
        except OSError as e:
            print(f"[UDP] could NOT bind UDP {self.port}: {e}")
            return False
        sock.settimeout(1.0)
        self._sock = sock
        self._running = True
        self._recv_thread = threading.Thread(target=self._recv_loop, name="udp-recv", daemon=True)
        self._demux_thread = threading.Thread(target=self._demux_loop, name="udp-demux", daemon=True)
        self._recv_thread.start()
        self._demux_thread.start()
        print(f"[UDP] Unified listener on port {self.port} (demux by stream_type; "
              f"broadband + LFP on one port)")
        return True

    def _recv_loop(self):
        # The hot path: recv -> ring, nothing else (so broadband is never blocked
        # while the demux/validator does its work downstream).
        ring = self._ring
        while self._running:
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            self.last_addr = addr
            try:
                ring.put_nowait(data)
            except queue.Full:
                self._ring_drops += 1   # host-side ring overflow (NOT a board drop)

    def _demux_loop(self):
        while self._running:
            try:
                data = self._ring.get(timeout=0.5)
            except queue.Empty:
                continue
            if len(data) < UNIFIED_HEADER_WORDS * 4:
                self.other_pkts += 1
                continue
            magic, type_ver = struct.unpack('<II', data[:8])
            if magic != UNIFIED_MAGIC:
                self.other_pkts += 1
                continue
            stream_type = type_ver & 0xFF
            if stream_type == STREAM_TYPE_BROADBAND:
                self._handle_broadband(data)
            elif stream_type == STREAM_TYPE_LFP:
                self._handle_lfp(data)
            else:
                self.other_pkts += 1

    def _handle_broadband(self, data):
        # A datagram is exactly one broadband packet (the board sends one packet
        # per datagram). Re-chunk defensively in case several were coalesced.
        self.bb_pkts += 1
        psize = validator.expected_packet_size_bytes
        if psize <= 0:
            return
        total = len(data)
        if total == psize:
            validator.validate_packet(data)
            return
        for off in range(0, total - psize + 1, psize):
            validator.validate_packet(data[off:off + psize])

    def _handle_lfp(self, data):
        self.lfp_pkts += 1
        self.lfp_bytes += len(data)
        # SEQ continuity for the LFP stream (header word 4)
        seq = struct.unpack('<I', data[16:20])[0]
        if self._lfp_last_seq is not None and seq != (self._lfp_last_seq + 1) & 0xFFFFFFFF:
            self._lfp_gaps += 1
        self._lfp_last_seq = seq
        with self._lock:
            subs = list(self._lfp_subs)
        for q in subs:
            try:
                q.put_nowait(data)
            except queue.Full:
                pass

    def subscribe_lfp(self, maxsize=20000):
        q = queue.Queue(maxsize=maxsize)
        with self._lock:
            self._lfp_subs.append(q)
        return q

    def unsubscribe_lfp(self, q):
        with self._lock:
            if q in self._lfp_subs:
                self._lfp_subs.remove(q)

    def stop(self):
        self._running = False


def udp_listener():
    """Compatibility wrapper: start the unified sink (the real work is in its
    recv/demux threads). Kept so __main__ can spawn it like before."""
    global UNIFIED_SINK
    if UNIFIED_SINK is None:
        UNIFIED_SINK = UnifiedSink()
    UNIFIED_SINK.start()
    # Block this (daemon) thread until shutdown; print stats on exit.
    try:
        while UNIFIED_SINK._running:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        validator.print_statistics()

def send_binary_command(sock, cmd_id, param1=0, param2=0, timeout=0.5):
    """Send a binary command and wait for ACK or data response"""
    try:
        ack_id = random.randint(1, 65535)
        command_data = struct.pack('<IIIII', CMD_MAGIC, cmd_id, ack_id, param1, param2)
        sock.sendall(command_data)
        
        sock.settimeout(timeout)
        
        # Read initial response (at least 3 bytes for ACK)
        response = sock.recv(5)
        
        if len(response) >= 3:
            recv_ack_id = (response[0] << 8) | response[1]
            status = response[2]
            
            if recv_ack_id != ack_id:
                print(f"[TCP] ACK ID mismatch: sent {ack_id}, got {recv_ack_id}")
                return (False, None)
            
            if status != ACK_SUCCESS:
                print(f"[TCP] Command failed (status: 0x{status:02X})")
                return (False, None)
            
            # Check if there's data (5-byte header)
            if len(response) == 5:
                data_len = (response[3] << 8) | response[4]
                if data_len > 0:
                    # Read the data
                    data = sock.recv(data_len)
                    return (True, data)
            
            return (True, None)
        
        return (False, None)

    except socket.timeout:
        # Let timeout propagate to reconnection handler
        raise
    except Exception as e:
        print(f"[TCP] Error: {e}")
        raise
    finally:
        sock.settimeout(None)

# ============================================================================
# LFP/DSP engine (Tier-1) host control + receive
# ============================================================================
# ============================================================================
# Analytic chirp NCO (memory-free swept sine in the PL; reuses the sine LUT)
# ============================================================================
def chirp_fmax_to_fspan(f_max_hz):
    """Map a desired sweep top frequency (Hz) to the 12-bit f_span field.
    f_max = (fspan << 16) / 2^32 * 30000  ->  fspan = f_max/30000 * 2^16."""
    f_max_acc = f_max_hz / PACKET_RATE_HZ * (1 << CHIRP_PHW)
    fspan = round(f_max_acc) >> CHIRP_FSPAN_SHIFT
    return max(0, min(0xFFF, fspan))

def chirp_sweep_to_rate(f_max_hz, period_s):
    """Pick the 12-bit sweep_rate so one ramp (0->f_max) takes ~period_s/2
    (a full triangle period is `period_s`). freq_acc step/packet = rate<<9."""
    fmax_acc = (chirp_fmax_to_fspan(f_max_hz) << CHIRP_FSPAN_SHIFT)
    half_packets = max(1, PACKET_RATE_HZ * period_s / 2.0)
    rate = round((fmax_acc / half_packets)) >> CHIRP_RATE_SHIFT
    return max(1, min(0xFFF, rate))

def configure_chirp(sock, f_max_hz=1400.0, period_s=2.0, stride=4, enable=True):
    """Enable the analytic chirp debug signal: a swept sine 0->f_max->0 with a
    full triangle period of `period_s`, per-channel phase `stride`. Requires
    debug mode (set it too). Disable with enable=False (or CMD_SET_DEBUG_MODE 0).
    Default sweep ~1 Hz -> ~1.4 kHz covers the new 3 kHz LFP passband + transition."""
    fspan = chirp_fmax_to_fspan(f_max_hz)
    rate  = chirp_sweep_to_rate(f_max_hz, period_s)
    p1 = (1 if enable else 0) | ((stride & 0x3F) << 8)
    p2 = (fspan & 0xFFF) | ((rate & 0xFFF) << 16)
    if enable:
        send_binary_command(sock, CMD_SET_DEBUG_MODE, 1)
    send_binary_command(sock, CMD_SET_CHIRP, p1, p2)
    f_lo = 0.0
    f_hi = (fspan << CHIRP_FSPAN_SHIFT) / (1 << CHIRP_PHW) * PACKET_RATE_HZ
    print(f"[CHIRP] {'ENABLED' if enable else 'disabled'}: sweep {f_lo:.0f}->{f_hi:.0f} Hz, "
          f"period~{period_s:.1f}s, stride={stride} (fspan={fspan} rate={rate})")
    return fspan, rate

def _kaiser_window(num_taps, beta):
    """Kaiser window via the I0 Bessel series (no numpy dependency)."""
    import math
    def i0(x):
        s, t, k = 1.0, 1.0, 0
        while True:
            k += 1
            t *= (x * x) / (4 * k * k)
            s += t
            if t < 1e-12 * s:
                return s
    a = (num_taps - 1) / 2.0
    return [i0(beta * math.sqrt(1 - ((n - a) / a) ** 2)) / i0(beta)
            for n in range(num_taps)]

def design_lfp_lowpass(num_taps, cutoff_hz=1250.0, fs=30000.0, window="kaiser",
                       beta=6.5):
    """Windowed-sinc low-pass FIR, unity DC gain, quantized to Q1.17 (18-bit
    signed). The decimation anti-alias; place cutoff (-6 dB) inside the
    transition band fs/(2*R_pass) .. fs/(2*R). For R=10 (3 kHz, Nyquist 1.5 kHz)
    the default ~131-tap Kaiser(beta=6.5) gives <1 dB ripple to 1 kHz, ~21 dB at
    1.5 kHz, and >46 dB rejection of any band that folds onto the 0-1 kHz
    passband. 'hamming' reproduces the legacy 2 kHz designer."""
    import math
    fc = cutoff_hz / fs                       # normalized cutoff (cycles/sample)
    M = num_taps - 1
    if window == "kaiser":
        win = _kaiser_window(num_taps, beta)
    else:  # legacy Hamming
        win = [0.54 - 0.46 * math.cos(2 * math.pi * n / M) for n in range(num_taps)]
    h = []
    for n in range(num_taps):
        x = n - M / 2.0
        s = 2 * fc if abs(x) < 1e-9 else math.sin(2 * math.pi * fc * x) / (math.pi * x)
        h.append(s * win[n])
    g = sum(h) or 1.0
    scale, lim = (1 << LFP_COEF_FRAC), (1 << 17)
    return [max(-lim, min(lim - 1, int(round(c / g * scale)))) for c in h]

def design_cic_comp_fir(num_taps=43, fc=1300.0, beta=6.0,
                        R_cic=5, n_order=4, gain_shift=10, fs_in=30000.0):
    """Droop-compensated comp-FIR / halfband (/2) for the CIC^4(/5)+FIR(/2) = /10
    LFP datapath (USE_CIC=1, the default build). Designed at the CIC output rate
    fs_in/R_cic (6 kHz) via frequency sampling: the target passband response is
    1/CIC-droop so the COMBINED /10 chain is flat to ~1 kHz (<=0.02 dB), with
    ~ -54 dB worst alias-into-passband and unity DC gain. Quantized Q1.17.
    MUST match programmable_logic/sim/gen_cic_chain_vectors.py exactly."""
    import math
    fs1 = fs_in / R_cic                       # comp-FIR input rate (6 kHz)
    def cic_mag(f):
        w = math.pi * f / fs_in
        if abs(w) < 1e-12: return 1.0
        d = R_cic * math.sin(w)
        return (math.sin(R_cic * w) / d) ** n_order if abs(d) > 1e-15 else 1.0
    cic_dc = (R_cic ** n_order) / (1 << gain_shift)
    win = _kaiser_window(num_taps, beta)
    M = num_taps - 1
    a = M / 2.0
    def desired(f):
        if f > fc: return 0.0
        dr = cic_mag(f)
        return (1.0 / dr) if dr > 1e-6 else 1.0
    L = 2048
    fs_grid = [fs1 / 2 * k / L for k in range(L + 1)]
    Hd = [desired(f) for f in fs_grid]
    df = fs_grid[1] - fs_grid[0]
    h = [0.0] * num_taps
    for n in range(num_taps):
        acc = 0.0
        for k in range(L + 1):
            wgt = 0.5 if (k == 0 or k == L) else 1.0
            acc += wgt * Hd[k] * math.cos(2 * math.pi * fs_grid[k] * (n - a) / fs1)
        h[n] = acc * df * 2.0 / (fs1 / 2) * win[n]
    dc = sum(h) or 1.0
    h = [c * (1.0 / cic_dc) / dc for c in h]   # combined DC gain -> unity
    scale, lim = (1 << LFP_COEF_FRAC), (1 << 17)
    return [max(-lim, min(lim - 1, int(round(c * scale)))) for c in h]

def lfp_upload_coeffs(sock, coeffs):
    """Stream taps through the indirect window (first write clears the pointer).
    This is a 100+ command burst that the board acks one-by-one, so use a generous
    per-command timeout and stop early (with a clear message) if an ack stalls --
    that distinguishes a merely-slow board from a wedged one."""
    for i, c in enumerate(coeffs):
        ok, _ = send_binary_command(sock, CMD_LFP_WRITE_COEF, 1 if i == 0 else 0,
                                    c & 0x3FFFF, timeout=2.0)
        if not ok:
            print(f"[LFP] coefficient upload stalled at tap {i}/{len(coeffs)} -- the board "
                  f"stopped acking. Reconnect and 'ping': if it answers it's a timing issue, "
                  f"if it's dead the coef path wedged the firmware.")
            return False
    return True

def configure_lfp(sock, datapath="cic", num_taps=None, cutoff_hz=1250.0):
    """Disable, set params, design + upload the LP kernel. Call lfp_enable(sock,
    True) afterwards to start streaming on the unified UDP port (5000,
    stream_type=2).

    The LFP lane mask is NO LONGER set here -- it MIRRORS the broadband
    channel-enable mask in the PL (single source of truth). Choose which streams
    to filter with `set_channels` (the broadband mask); the LFP filters exactly
    the broadband-enabled lanes.

    datapath="cic" (default, matches the USE_CIC=1 build): uploads the 43-tap
      droop-compensated comp-FIR halfband; the engine's CIC^4(/5)+halfband(/2)=/10
      decimation is hardwired -> 3 kHz LFP. num_taps defaults to 43.
    datapath="fir" (USE_CIC=0 fallback build): uploads a 131-tap single-stage
      Kaiser 3 kHz anti-alias; decim_R=10. num_taps defaults to 131.

    Either way R=10 -> 3 kHz; the LFP packet's self-describing R field is set
    accordingly so the host/plugin auto-tracks the rate."""
    send_binary_command(sock, CMD_LFP_ENABLE, 0)
    if datapath == "cic":
        nt = num_taps if num_taps else 43
        send_binary_command(sock, CMD_LFP_SET_PARAMS, 10, nt & 0xFF)
        coeffs = design_cic_comp_fir(nt)
        lfp_upload_coeffs(sock, coeffs)
        print(f"[LFP] configured CIC^4(/5)+halfband(/2)=/10: lane mask = broadband "
              f"channel_enable; comp_taps={nt} -> 3000 sps out (flat to ~1 kHz)")
    else:
        nt = num_taps if num_taps else 131
        send_binary_command(sock, CMD_LFP_SET_PARAMS, 10, nt & 0xFF)
        coeffs = design_lfp_lowpass(nt, cutoff_hz)
        lfp_upload_coeffs(sock, coeffs)
        print(f"[LFP] configured single-stage FIR /10: lane mask = broadband "
              f"channel_enable; taps={nt} cutoff={cutoff_hz:.0f}Hz -> 3000 sps out")
    return coeffs

def fs_out_str(decim_R):
    return f"{30000.0 / max(decim_R,1):.0f} sps out"

def lfp_enable(sock, on=True):
    send_binary_command(sock, CMD_LFP_ENABLE, 1 if on else 0)
    print(f"[LFP] {'ENABLED' if on else 'disabled'}")


# ---------------------------------------------------------------------------
# LFP datagram reader. Unified port: the LFP band arrives on UDP_PORT (5000)
# mixed with broadband, demuxed by the UnifiedSink (stream_type=2). The reader
# subscribes to that sink's LFP fan-out so a single socket drains 5000 for the
# whole session (an unconsumed UDP port makes the host kernel reply ICMP
# "port unreachable" per datagram -> an RX-interrupt storm that preempts the
# board's polled loop). If the sink isn't running, fall back to a private bind
# on 5000 that demuxes LFP itself.
# ---------------------------------------------------------------------------
class _LfpReader:
    """Yields LFP datagrams from the persistent UnifiedSink if running, else from
    a private socket bound to UDP_PORT that filters stream_type=2 itself."""
    def __init__(self, port=UDP_PORT, timeout=3.0):
        self.port = port
        self.timeout = timeout
        self._q = None
        self._sock = None

    def open(self):
        if UNIFIED_SINK is not None and UNIFIED_SINK._running:
            self._q = UNIFIED_SINK.subscribe_lfp()
        else:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('', self.port))
            s.settimeout(self.timeout)
            self._sock = s
        return self

    def recv(self):
        """Next LFP datagram (stream_type=2), or raise socket.timeout."""
        if self._q is not None:
            try:
                return self._q.get(timeout=self.timeout)
            except queue.Empty:
                raise socket.timeout()
        # private-bind fallback: demux LFP ourselves
        while True:
            data, _ = self._sock.recvfrom(4096)
            if len(data) >= 8:
                magic, type_ver = struct.unpack('<II', data[:8])
                if magic == UNIFIED_MAGIC and (type_ver & 0xFF) == STREAM_TYPE_LFP:
                    return data

    def close(self):
        if self._q is not None:
            UNIFIED_SINK.unsubscribe_lfp(self._q)
            self._q = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None


def receive_lfp(n_packets=200, bind_port=UDP_PORT):
    """Bind the LFP UDP port and parse frames -- a reference receiver for the
    plugin. Payload samples are offset-binary 16-bit (subtract 0x8000 for signed).

    The unified 8-word common header is built BY THE PL (not the PS): the PL
    writes the complete wire packet (header + samples) into its output BRAM and
    the PS just CDMAs it into a pbuf and sends it on UDP_PORT (5000), demuxed by
    stream_type=2. Header layout (docs/unified-packet-format.md):
      w0 = MAGIC (0xCAFEBABE), w1 = TYPE_VER (stream_type=2 | version<<8)
      w2/w3 = 64-bit master timestamp = the master count of the NEWEST broadband
              sample in this output's decimation window -- i.e. the most recent
              broadband packet that was clocked into the (FIR) filter bank for
              this output. For frame m at total decimation R it is broadband
              packet R*m + (R-1) (R=10 -> 10m+9). Causal, monotonic, R apart.
              NB: this marks the newest *input* sample, not the instant the LFP
              value represents; subtract the filter group delay to align with
              broadband *content*.
      w4 = SEQ = PL-maintained LFP frame sequence number (++ per emitted frame)
      w5 = AUX0 = lane_mask | (decim_R<<8) | (num_taps<<16) | (overrun<<24);
           lane_mask = the broadband channel_enable mask (single source of truth).
      w6 = AUX1 = num_samples (popcount(lane_mask) * 32)
      w7 = RSVD = 0."""
    HDR = UNIFIED_HEADER_WORDS * 4   # 32-byte common header
    r = _LfpReader(port=bind_port, timeout=3.0).open()
    got = bad = 0
    last_seq = None
    try:
        while got < n_packets:
            data = r.recv()
            if len(data) < HDR:
                bad += 1; continue
            magic, type_ver, ts_lo, ts_hi, seq, cfg, num_samples, rsvd = \
                struct.unpack('<IIIIIIII', data[:HDR])
            if magic != UNIFIED_MAGIC or (type_ver & 0xFF) != STREAM_TYPE_LFP:
                bad += 1; continue
            lane_mask, decim_R, num_taps, overrun = (cfg & 0xFF, (cfg >> 8) & 0xFF,
                                                     (cfg >> 16) & 0xFF, (cfg >> 24) & 1)
            pay = data[HDR:]
            usamp = struct.unpack(f'<{len(pay)//2}H', pay)
            samp = [u - 0x8000 for u in usamp]      # offset binary -> signed
            ts = ts_lo | (ts_hi << 32)
            if last_seq is not None and seq != (last_seq + 1) & 0xFFFFFFFF:
                print(f"[LFP] seq gap {last_seq}->{seq}")
            last_seq = seq
            got += 1
            if got <= 5 or got % 50 == 0:
                print(f"[LFP] seq={seq} ts={ts} mask=0x{lane_mask:02X} R={decim_R} "
                      f"taps={num_taps} ov={overrun} n={len(samp)} first={samp[:4]}")
    except socket.timeout:
        print("[LFP] receive timeout (is the engine enabled + streaming?)")
    finally:
        r.close()
    print(f"[LFP] received {got} frames, {bad} non-LFP/short datagrams")
    return got

def _goertzel_power(x, f, fs):
    """Power of x at frequency f (Hz), sample rate fs (Goertzel; pure Python)."""
    import math
    cw = 2.0 * math.cos(2.0 * math.pi * (f / fs))
    s1 = s2 = 0.0
    for v in x:
        s0 = v + cw * s1 - s2
        s2 = s1; s1 = s0
    return s1*s1 + s2*s2 - cw*s1*s2

def measure_lfp_response(sock, f_max=1490.0, period=3.0, n_periods=2,
                         channel=0, lane_mask=0x0F, fmin=25, fstep=25):
    """Drive the analytic chirp across [0, f_max] and MEASURE the LFP anti-alias
    magnitude response |H(f)| from the host. Method: capture one LFP channel for a
    few chirp triangle periods; in each short window find the dominant frequency
    (Goertzel peak over a grid) and its amplitude; bin amplitude vs frequency.
    Self-calibrating -- does NOT assume a flat chirp spectrum. f_max defaults just
    under the 1500 Hz LFP Nyquist so nothing folds. Prints a dB table + ASCII plot.
    The chirp/debug/coef configs latch only while stopped, so this stops, arms, and
    restarts the stream itself."""
    import math, time
    fs = 3000.0   # LFP rate (30 kHz / decim_R=10)
    # --- preflight: confirm the board is alive and on v1.3+ firmware. The chirp /
    #     3 kHz-LFP commands lfp_sweep needs ONLY exist in v1.3; running against a
    #     wedged or older-image board is the usual cause of an apparent "hang".
    #     Fail fast and clearly instead. ---
    st = get_status(sock)
    if not st:
        print("[SWEEP] board did NOT answer get_status -- unresponsive/wedged. "
              "Power-cycle and flash main's v1.3 BOOT.bin (4,455,828 B). Aborting.")
        return
    fw = st.get('firmware_version', 0)
    fmaj, fmin = (fw >> 24) & 0xFF, (fw >> 16) & 0xFF
    print(f"[SWEEP] board firmware {fmaj}.{fmin}")
    if (fmaj, fmin) < (1, 3):
        print(f"[SWEEP] firmware {fmaj}.{fmin} is too old for the chirp/3kHz-LFP path "
              f"(lfp_sweep needs >=1.3). You're on the wrong board image -- flash main's "
              f"v1.3 BOOT.bin. Aborting.")
        return
    send_binary_command(sock, CMD_STOP)
    # The LFP lane mask mirrors the broadband channel_enable, so select the lanes
    # via set_channels; the LFP engine then filters exactly those lanes.
    send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, lane_mask & 0xFF)
    configure_lfp(sock, datapath="cic")                     # upload CIC comp-FIR
    configure_chirp(sock, f_max, period, stride=0, enable=True)  # debug+chirp on
    lfp_enable(sock, True)
    send_binary_command(sock, CMD_START)
    # ---- capture one channel's time series off the unified UDP port (5000) ----
    settle = int(0.3 * fs)
    n_want = int(n_periods * period * fs) + settle
    HDR = UNIFIED_HEADER_WORDS * 4   # 32-byte common header
    r = _LfpReader(port=UDP_PORT, timeout=3.0).open()
    series = []
    t_start = time.time()
    max_wall = n_periods * period * 4.0 + 8.0   # hard wall-clock cap; never spin forever
    try:
        while len(series) < n_want and (time.time() - t_start) < max_wall:
            data = r.recv()
            if len(data) < HDR: continue
            magic, type_ver = struct.unpack('<II', data[:8])
            if magic != UNIFIED_MAGIC or (type_ver & 0xFF) != STREAM_TYPE_LFP: continue
            usamp = struct.unpack(f'<{(len(data)-HDR)//2}H', data[HDR:])
            if channel < len(usamp):
                series.append(usamp[channel] - 0x8000)      # offset-binary -> signed
    except socket.timeout:
        print("[SWEEP] capture timed out -- is the chirp+LFP streaming?")
    finally:
        r.close()
    series = series[settle:]
    if len(series) < int(period * fs):
        print(f"[SWEEP] only {len(series)} LFP samples in ~{max_wall:.0f}s -- the board "
              f"answered control (fw {fmaj}.{fmin}) but isn't streaming LFP. The LFP/CIC "
              f"datapath likely wedged on this image. Aborting."); return
    # ---- short-time dominant-freq + amplitude -> response ----
    grid = list(range(int(fmin), int(min(f_max, fs/2.0)) + 1, int(fstep)))
    win, hop = 256, 64    # hop small enough that the swept tone hits every grid bin
    acc = {f: [0.0, 0] for f in grid}
    i, n = 0, len(series)
    while i + win <= n:
        w = series[i:i+win]
        m = sum(w) / win
        w = [v - m for v in w]
        bf, bp = grid[0], -1.0
        for f in grid:
            p = _goertzel_power(w, f, fs)
            if p > bp: bp, bf = p, f
        acc[bf][0] += math.sqrt(max(bp, 0.0)) * 2.0 / win   # dominant-tone amplitude
        acc[bf][1] += 1
        i += hop
    resp = [(f, (acc[f][0]/acc[f][1] if acc[f][1] else 0.0), acc[f][1]) for f in grid]
    pb = sorted(a for (f, a, c) in resp if 100 <= f <= 800 and c)   # passband ref
    ref = pb[len(pb)//2] if pb else (max((a for _, a, _ in resp), default=1.0) or 1.0)
    print(f"\n[SWEEP] LFP anti-alias |H(f)|  (chirp 0->{f_max:.0f} Hz, "
          f"{n_periods}x{period:.1f}s, ch{channel}, {len(series)} samp @ {fs:.0f} Hz)")
    print("  f(Hz)  |H| dB  hits  |" + "-"*42)
    crossings, pf, pdb = {}, None, None
    for f, a, c in resp:
        if c == 0:
            print(f"  {f:5d}     --     0  |")     # no window peaked here
            continue
        db = 20*math.log10(a/ref) if (a > 0 and ref > 0) else -99.0
        bar = "#" * int(max(0, min(42, (db + 60.0) / 60.0 * 42)))
        print(f"  {f:5d}  {db:6.1f}  {c:4d}  |{bar}")
        for thr in (-3.0, -6.0, -20.0):
            if thr not in crossings and pdb is not None and pdb >= thr > db:
                crossings[thr] = pf + (thr - pdb) * (f - pf) / (db - pdb)
        pf, pdb = f, db
    for thr in (-3.0, -6.0, -20.0):
        if thr in crossings:
            print(f"  measured {thr:+.0f} dB at ~{crossings[thr]:.0f} Hz")
    print("  predicted (shipped CIC+comp): flat to ~1.0 kHz, -3 dB ~1.25 kHz, "
          "-6 dB ~1.30 kHz, -32 dB @ 1.50 kHz Nyquist")
    print("  (chirp left running; 'chirp_off' to stop, or raise f_max for more transition)")
    return resp

# ============================================================================
# UDP throughput benchmark -- measure the real sustained MB/s vs packet size,
# replacing the ~18 MB/s small-packet broadband assumption with a measured
# large-packet ceiling. (Payloads > ~1472 B fragment unless the path is jumbo-
# enabled; the sweep shows the effect.)
# ============================================================================
def udp_bench(sock, payload_bytes=1400, n_packets=50000, bind_port=UDP_BENCH_PORT):
    import time
    us = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        us.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 * 1024 * 1024)
    except OSError:
        pass
    us.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    us.bind(('', bind_port))
    us.settimeout(2.0)
    ack_id = random.randint(1, 65535)
    sock.sendall(struct.pack('<IIIII', CMD_MAGIC, CMD_UDP_BENCH, ack_id,
                             payload_bytes, n_packets))
    got = nbytes = 0
    t0 = t1 = None
    try:
        while got < n_packets:
            data, _ = us.recvfrom(payload_bytes + 200)
            now = time.time()
            if t0 is None:
                t0 = now
            t1 = now
            got += 1; nbytes += len(data)
    except socket.timeout:
        pass
    us.close()
    try:                                  # the firmware ACKs after the blast
        sock.settimeout(1.0); sock.recv(8)
    except Exception:
        pass
    finally:
        sock.settimeout(None)
    if t0 and got > 1 and t1 > t0:
        dt = t1 - t0
        mbps, pps = nbytes / dt / 1e6, got / dt
        loss = 100.0 * (1 - got / n_packets)
        print(f"  {payload_bytes:>6} B x {n_packets}:  {mbps:6.1f} MB/s   "
              f"{pps/1000:6.1f}k pps   loss {loss:5.1f}%   ({got} rx)")
        return mbps, pps, loss
    print(f"  {payload_bytes:>6} B: no/insufficient data received "
          f"(check ZYNQ_IP / firewall on port {bind_port})")
    return 0.0, 0.0, 100.0

def udp_bench_sweep(sock, n_packets=50000):
    print("UDP throughput sweep (small=broadband-like, 1472=MTU, larger=jumbo/fragmented):")
    for sz in (256, 600, 1024, 1472, 2944, 5888, 8800):
        udp_bench(sock, sz, n_packets)

def aux_upload_bank(sock, slot, bank, cmds, loop_idx=0):
    """Upload a command program (with its length record) into a standby bank.
    Works during acquisition; swap it live with aux_bank_select()."""
    if not (1 <= len(cmds) <= AUX_BANK_ENTRIES) or not (0 <= loop_idx < len(cmds)):
        print(f"[AUX] Bad program: {len(cmds)} cmds, loop {loop_idx}")
        return False
    p1 = (slot & 3) | ((bank & 1) << 8)
    for i, c in enumerate(cmds):
        ok, _ = send_binary_command(sock, CMD_AUX_WRITE_WORD, p1, (i << 16) | (c & 0xFFFF))
        if not ok:
            print(f"[AUX] Upload failed at word {i}")
            return False
    length_data = (loop_idx & 0x3F) | (((len(cmds) - 1) & 0x3F) << 8)
    ok, _ = send_binary_command(sock, CMD_AUX_WRITE_WORD, p1 | (1 << 16), length_data)
    if ok:
        print(f"[AUX] Slot {slot} bank {bank}: {len(cmds)} commands (loop at {loop_idx})")
    return ok

def aux_bank_select(sock, slot, bank):
    """Atomically swap a slot to a bank at the next packet boundary.
    The firmware confirms the swap (bank_active poll) before ACKing."""
    ok, _ = send_binary_command(sock, CMD_AUX_BANK_SELECT, slot & 3, bank & 1)
    print(f"[AUX] Slot {slot} -> bank {bank}: {'OK' if ok else 'FAILED'}")
    return ok

def aux_seq_enable(sock, enable):
    ok, _ = send_binary_command(sock, CMD_AUX_SEQ_EN, 1 if enable else 0)
    print(f"[AUX] Sequencer {'enabled' if enable else 'disabled'}: {'OK' if ok else 'FAILED'}")
    return ok

def read_register(sock, reg):
    """Read an RHD register at runtime (injected via the sequencer; requires
    streaming + sequencer enabled). Returns (cipo0_value, cipo1_value)."""
    ok, data = send_binary_command(sock, CMD_READ_REGISTER, reg & 0x3F)
    if not ok or data is None or len(data) != 4:
        print(f"[AUX] READ_REGISTER {reg} failed")
        return None
    result = struct.unpack('<I', data)[0]
    c0, c1 = result & 0xFFFF, (result >> 16) & 0xFFFF
    print(f"[AUX] Register {reg}: CIPO0=0x{c0:04X} CIPO1=0x{c1:04X}")
    return (c0, c1)

def write_register(sock, reg, value):
    """Write an RHD register at runtime. The chip echoes the data byte in the
    low byte of the result (upper byte all-ones) -- verified here.
    Note: Reg 0 D5 and all of Reg 3 are owned by the override layer; use
    set_fast_settle()/set_digout() for those bits."""
    ok, data = send_binary_command(sock, CMD_WRITE_REGISTER, reg & 0x3F, value & 0xFF)
    if not ok or data is None or len(data) != 4:
        print(f"[AUX] WRITE_REGISTER {reg} failed")
        return False
    result = struct.unpack('<I', data)[0]
    echo_ok = (result & 0xFF) == (value & 0xFF)
    print(f"[AUX] Register {reg} <= 0x{value & 0xFF:02X}: echo "
          f"{'confirmed' if echo_ok else f'MISMATCH (0x{result:08X})'}")
    return echo_ok

def set_fast_settle(sock, amp_sw=False, amp_gpio_en=False, amp_pin=0,
                    dsp_sw=False, dsp_gpio_en=False, dsp_pin=0):
    """Configure amplifier fast settle (Reg-0 D5 via Slot-1 injection) and the
    DSP-reset bit-H. Software levels and/or a digital_in pin trigger."""
    p1 = (1 if amp_sw else 0) | (2 if amp_gpio_en else 0) | ((amp_pin & 7) << 4)
    p2 = (1 if dsp_sw else 0) | (2 if dsp_gpio_en else 0) | ((dsp_pin & 7) << 4)
    ok, _ = send_binary_command(sock, CMD_SET_FAST_SETTLE, p1, p2)
    print(f"[AUX] Fast settle config: {'OK' if ok else 'FAILED'}")
    return ok

def set_digout(sock, sw=False, gpio_en=False, pin=0, reg3_static=0x00):
    """Configure the auxout digital-output mirror and the host-owned Reg-3
    static bits (D7..D1: MUX load, tempS2/S1/tempen, digout HiZ). HiZ (D1)
    must be 0 in reg3_static for auxout to drive."""
    p1 = (1 if sw else 0) | (2 if gpio_en else 0) | ((pin & 7) << 4)
    ok, _ = send_binary_command(sock, CMD_SET_DIGOUT, p1, reg3_static & 0xFF)
    print(f"[AUX] Digout config: {'OK' if ok else 'FAILED'}")
    return ok

def aux_demo_setup(sock):
    """Load the default slot programs and enable the sequencer:
    slot 0 = Reg-3 carrier (digout mirror), slot 1 = accel sweep @10 kHz,
    slot 2 = supply/temp/link housekeeping."""
    if not (aux_upload_bank(sock, 0, 0, AUX_SLOT0_DEFAULT) and
            aux_upload_bank(sock, 1, 0, AUX_SLOT1_DEFAULT) and
            aux_upload_bank(sock, 2, 0, AUX_SLOT2_DEFAULT)):
        return False
    return aux_seq_enable(sock, True)

def get_status(sock):
    """Get full status from device"""
    success, data = send_binary_command(sock, CMD_GET_STATUS)

    if not success or data is None:
        print("[TCP] Failed to get status")
        return None

    if len(data) != 288:
        print(f"[TCP] Invalid status response length: {len(data)} (expected 220)")
        return None
    
    # Parse status_response_t structure (86 bytes)
    # Version and identification (8 bytes)
    version, device_type, firmware_version = struct.unpack('<HHI', data[0:8])
    
    # PL Hardware Status (22 bytes)
    timestamp, packets_sent, bram_write_addr, fifo_count, state_counter, cycle_counter, flags_pl, _ = \
        struct.unpack('<QIIHBBBB', data[8:30])
    
    # PS Software Status (28 bytes)
    # Format: 6 uint32_t + 1 uint8_t + 3 reserved bytes
    packets_received, error_count, udp_packets_sent, udp_send_errors, ps_read_addr, packet_size, flags_ps = \
        struct.unpack('<IIIIIIB3x', data[30:58])
    
    # Current Configuration (16 bytes)
    # Format: 1 uint32_t + 4 uint8_t + 8 reserved bytes
    loop_count, phase0, phase1, channel_enable, debug_mode = \
        struct.unpack('<IBBBB8x', data[58:74])
    
    # UDP Stream Information (12 bytes)
    udp_dest_ip, udp_dest_port, udp_packet_format, udp_bytes_sent = \
        struct.unpack('<IHHi', data[74:86])

    # Aux command sequencer status (12 bytes)
    aux_read_result, aux_bank_active, aux_flags, aux_i0, aux_i1, aux_i2 = \
        struct.unpack('<IBBBBB3x', data[86:98])

    # DMA / performance instrumentation (24 bytes: raw ticks + tick frequency)
    dma_errors, dma_ticks_last, dma_ticks_max, loop_ticks_last, loop_ticks_max, timer_hz = \
        struct.unpack('<IIIIII', data[98:122])

    # Aux control register (CTRL_REG_22): live fast-settle / DSP / digout config (4 bytes)
    (aux_ctrl,) = struct.unpack('<I', data[122:126])

    # RHD chip register mirror: commanded state of regs 0..21 (22 bytes)
    rhd_reg = struct.unpack('<22B', data[126:148])
    # LFP/DSP engine config + status (12 bytes)
    (lfp_enable, lfp_lane_mask, lfp_decim_R, lfp_num_taps) = struct.unpack('<4B', data[148:152])
    (lfp_packets_sent,) = struct.unpack('<I', data[152:156])
    (lfp_overrun,) = struct.unpack('<B', data[156:157])
    # data[157:160] = lfp_reserved[3]
    # Analytic chirp NCO config (8 bytes): mode, stride, fspan(u16), rate(u16), 2 rsvd
    (chirp_mode, chirp_stride, chirp_fspan, chirp_rate) = \
        struct.unpack('<BBHH2x', data[160:168])

    # recv->transmit spike instrumentation (52 bytes): the recv->transmit window
    # split into udp_sendto / worst-case breakdown + a 6-bucket histogram. All
    # times are raw ticks (converted to us against timer_hz in print_status).
    (send_ticks_last, send_ticks_max, over_budget_count, worst_pkt_index,
     worst_cdma_ticks, worst_send_ticks, worst_other_ticks) = \
        struct.unpack('<7I', data[168:196])
    loop_hist = struct.unpack('<6I', data[196:220])

    # TX drop diagnostics (v1.6, 68 bytes): split udp_send_errors into broadband
    # vs LFP, pbuf-alloc-fail (MEMP_PBUF pool empty) vs udp_sendto error (+ err
    # code), first/last drop packet index, MEMP_NUM_PBUF, and an 8-deep ring of
    # recent drop indices. err_t: ERR_MEM=-1, ERR_BUF=-2, ERR_RTE=-4, ...
    (bb_pbuf_alloc_fail, bb_send_err, bb_last_send_err,
     lfp_pbuf_alloc_fail, lfp_send_err, lfp_last_send_err,
     first_drop_pkt, last_drop_pkt, memp_num_pbuf) = \
        struct.unpack('<IIiIIiIII', data[220:256])
    drop_ring = struct.unpack('<8I', data[256:288])

    status = {
        'version': version,
        'device_type': device_type,
        'firmware_version': firmware_version,
        'timestamp': timestamp,
        'packets_sent': packets_sent,
        'bram_write_addr': bram_write_addr,
        'fifo_count': fifo_count,
        'state_counter': state_counter,
        'cycle_counter': cycle_counter,
        'transmission_active': bool(flags_pl & 0x01),
        'loop_limit_reached': bool(flags_pl & 0x02),
        'packets_received': packets_received,
        'error_count': error_count,
        'udp_packets_sent': udp_packets_sent,
        'udp_send_errors': udp_send_errors,
        'ps_read_addr': ps_read_addr,
        'packet_size': packet_size,
        'stream_enabled': bool(flags_ps & 0x01),
        'loop_count': loop_count,
        'phase0': phase0,
        'phase1': phase1,
        'channel_enable': channel_enable,
        'debug_mode': debug_mode,
        'udp_dest_ip': ipaddress.IPv4Address(udp_dest_ip),
        'udp_dest_port': udp_dest_port,
        'udp_packet_format': udp_packet_format,
        'udp_bytes_sent': udp_bytes_sent,
        'aux_seq_enabled': bool(aux_flags & 0x01),
        'aux_fast_settle': bool(aux_flags & 0x02),
        'aux_digout': bool(aux_flags & 0x04),
        'aux_dsp_reset': bool(aux_flags & 0x08),
        'aux_bank_active': aux_bank_active,
        'aux_indices': (aux_i0, aux_i1, aux_i2),
        'aux_read_result': aux_read_result,
        'dma_errors': dma_errors,
        'dma_ticks_last': dma_ticks_last,
        'dma_ticks_max': dma_ticks_max,
        'loop_ticks_last': loop_ticks_last,
        'loop_ticks_max': loop_ticks_max,
        'timer_hz': timer_hz,
        # recv->transmit spike instrumentation (cleared by perf_reset)
        'send_ticks_last': send_ticks_last,
        'send_ticks_max': send_ticks_max,
        'over_budget_count': over_budget_count,
        'worst_pkt_index': worst_pkt_index,
        'worst_cdma_ticks': worst_cdma_ticks,
        'worst_send_ticks': worst_send_ticks,
        'worst_other_ticks': worst_other_ticks,
        'loop_hist': list(loop_hist),
        # TX drop diagnostics (v1.6)
        'bb_pbuf_alloc_fail': bb_pbuf_alloc_fail,
        'bb_send_err': bb_send_err,
        'bb_last_send_err': bb_last_send_err,
        'lfp_pbuf_alloc_fail': lfp_pbuf_alloc_fail,
        'lfp_send_err': lfp_send_err,
        'lfp_last_send_err': lfp_last_send_err,
        'first_drop_pkt': first_drop_pkt,
        'last_drop_pkt': last_drop_pkt,
        'memp_num_pbuf': memp_num_pbuf,
        'drop_ring': list(drop_ring),
        # Aux config decoded from CTRL_REG_22 (fast-settle / DSP / digout)
        'aux_ctrl': aux_ctrl,
        'fs_sw': bool(aux_ctrl & (1 << 4)),
        'fs_gpio_en': bool(aux_ctrl & (1 << 5)),
        'fs_pin': (aux_ctrl >> 6) & 0x7,
        'dsp_sw': bool(aux_ctrl & (1 << 9)),
        'dsp_gpio_en': bool(aux_ctrl & (1 << 10)),
        'dsp_pin': (aux_ctrl >> 11) & 0x7,
        'digout_sw': bool(aux_ctrl & (1 << 14)),
        'digout_gpio_en': bool(aux_ctrl & (1 << 15)),
        'digout_pin': (aux_ctrl >> 16) & 0x7,
        # RHD chip register mirror (commanded state of regs 0..21)
        'rhd_reg': list(rhd_reg),
        'lfp_enable': lfp_enable,
        'lfp_lane_mask': lfp_lane_mask,
        'lfp_decim_R': lfp_decim_R,
        'lfp_num_taps': lfp_num_taps,
        'lfp_packets_sent': lfp_packets_sent,
        'lfp_overrun': lfp_overrun,
        'chirp_mode': chirp_mode,
        'chirp_stride': chirp_stride,
        'chirp_fspan': chirp_fspan,
        'chirp_rate': chirp_rate,
    }

    return status

def print_status(status):
    """Pretty print status information"""
    if not status:
        return
    
    fw_ver = status['firmware_version']
    fw_str = f"{(fw_ver>>24)&0xFF}.{(fw_ver>>16)&0xFF}.{(fw_ver>>8)&0xFF}.{fw_ver&0xFF}"
    
    print("\n=== DEVICE STATUS ===")
    print(f"Device Type: 0x{status['device_type']:04X}")
    print(f"Firmware: v{fw_str}")
    print(f"Protocol Version: {status['version']}")
    
    print("\n--- PL Hardware ---")
    print(f"Timestamp: {status['timestamp']}")
    print(f"Packets Sent: {status['packets_sent']}")
    print(f"BRAM Write Addr: {status['bram_write_addr']}")
    print(f"FIFO Count: {status['fifo_count']}")
    print(f"State/Cycle: {status['state_counter']}/{status['cycle_counter']}")
    print(f"Transmission Active: {status['transmission_active']}")
    print(f"Loop Limit Reached: {status['loop_limit_reached']}")
    
    print("\n--- PS Software ---")
    print(f"Packets Received: {status['packets_received']}")
    print(f"Error Count: {status['error_count']}")
    print(f"UDP Packets Sent: {status['udp_packets_sent']}")
    print(f"UDP Send Errors: {status['udp_send_errors']}")
    print(f"PS Read Addr: {status['ps_read_addr']}")
    print(f"Packet Size: {status['packet_size']} words")
    print(f"Stream Enabled: {status['stream_enabled']}")
    
    print("\n--- Configuration ---")
    print(f"Loop Count: {status['loop_count']}")
    print(f"Phase0: {status['phase0']}, Phase1: {status['phase1']}")
    print(f"Channel Enable: 0x{status['channel_enable']:X} ({channel_enable_to_string(status['channel_enable'])})")
    print(f"Debug Mode: {status['debug_mode']}")
    
    print("\n--- UDP Stream ---")
    print(f"Destination: {status['udp_dest_ip']}:{status['udp_dest_port']}")
    print(f"Packet Format: 0x{status['udp_packet_format']:04X}")
    print(f"Bytes Sent: {status['udp_bytes_sent']}")

    print("\n--- Aux Sequencer ---")
    print(f"Enabled: {status['aux_seq_enabled']}, "
          f"Fast Settle: {status['aux_fast_settle']}, "
          f"Digout: {status['aux_digout']}, "
          f"DSP Reset: {status['aux_dsp_reset']}")
    ba = status['aux_bank_active']
    print(f"Active Banks: slot0={ba & 1}, slot1={(ba >> 1) & 1}, slot2={(ba >> 2) & 1}")
    print(f"Slot Indices: {status['aux_indices']}")
    print(f"Last Inject Result: 0x{status['aux_read_result']:08X}")
    def _src(sw, gp, pin):
        return f"GPIO pin {pin}" if gp else ("software" if sw else "off")
    print(f"Config (CTRL_REG_22 0x{status['aux_ctrl']:08X}): "
          f"fast-settle={_src(status['fs_sw'], status['fs_gpio_en'], status['fs_pin'])}, "
          f"dsp-reset={_src(status['dsp_sw'], status['dsp_gpio_en'], status['dsp_pin'])}, "
          f"digout={_src(status['digout_sw'], status['digout_gpio_en'], status['digout_pin'])}")

    print("\n--- Performance (budget 33.3 us/packet @ 30 kHz) ---")
    hz = status['timer_hz'] or 1   # raw ticks -> us converted here, not in firmware
    to_us = lambda t: t * 1e6 / hz
    print(f"CDMA transfer:  last {to_us(status['dma_ticks_last']):.2f} us, max {to_us(status['dma_ticks_max']):.2f} us")
    print(f"UDP send:       last {to_us(status['send_ticks_last']):.2f} us, max {to_us(status['send_ticks_max']):.2f} us")
    print(f"Recv->transmit: last {to_us(status['loop_ticks_last']):.2f} us, max {to_us(status['loop_ticks_max']):.2f} us")
    lm = to_us(status['loop_ticks_max'])
    print(f"Headroom (max): {33.3 - lm:.2f} us  ({100.0*lm/33.3:.0f}% of budget used)")
    # Worst-packet breakdown: WHAT dominated the worst recv->transmit. If send
    # >> cdma here (and the histogram has a tail), the spike is the GEM TX reaping.
    wc, ws, wo = (to_us(status['worst_cdma_ticks']),
                  to_us(status['worst_send_ticks']),
                  to_us(status['worst_other_ticks']))
    print(f"Worst pkt #{status['worst_pkt_index']}: cdma={wc:.2f} send={ws:.2f} other={wo:.2f} us  (sum={wc+ws+wo:.2f})")
    # recv->transmit distribution (us bucket edges) + over-budget frequency
    h = status['loop_hist']
    edges = ["<16", "16-25", "25-33", "33-50", "50-100", ">=100"]
    total = sum(h) or 1
    print("Recv->transmit histogram (us):")
    for lbl, c in zip(edges, h):
        print(f"   {lbl:>7}: {c:>10}  ({100.0*c/total:5.1f}%)")
    print(f"Over budget (>=33 us): {status['over_budget_count']}  ({100.0*status['over_budget_count']/total:.3f}% of {total} pkts)")
    print(f"DMA errors: {status['dma_errors']}   (timer {hz/1e6:.1f} MHz)   [perf_reset to clear maxes/histogram]")

    # --- TX drops (v1.6): WHY udp_send_errors happened. pbuf-alloc-fail => the
    # shared MEMP_PBUF zero-copy pool (bb + lfp) was momentarily empty; sendto-err
    # (ERR_MEM=-1) => no TX BD/mem. first/last/ring show whether drops cluster at
    # stream start (cold/warmup) or recur in steady state. Cleared by perf_reset.
    _errname = {0: "OK", -1: "ERR_MEM", -2: "ERR_BUF", -3: "ERR_TIMEOUT",
                -4: "ERR_RTE", -6: "ERR_VAL", -7: "ERR_WOULDBLOCK"}
    _en = lambda c: _errname.get(c, str(c))
    print("\n--- TX drops (zero-copy pbuf pool / send) ---")
    print(f"MEMP_NUM_PBUF (shared bb+lfp zero-copy pool): {status['memp_num_pbuf']}")
    print(f"Broadband: pbuf_alloc-fail={status['bb_pbuf_alloc_fail']}  "
          f"sendto-err={status['bb_send_err']} (last {_en(status['bb_last_send_err'])})")
    print(f"LFP:       pbuf_alloc-fail={status['lfp_pbuf_alloc_fail']}  "
          f"sendto-err={status['lfp_send_err']} (last {_en(status['lfp_last_send_err'])})")
    print(f"Broadband drop span: first pkt={status['first_drop_pkt']}, last pkt={status['last_drop_pkt']}")
    print(f"  recent drop pkt ring (last 8): {list(status['drop_ring'])}")

    rr = status['rhd_reg']
    print("\n--- RHD Chip Registers (mirror, commanded state) ---")
    print("  reg  0-7 : " + " ".join(f"{b:02X}" for b in rr[0:8]))
    print("  reg  8-15: " + " ".join(f"{b:02X}" for b in rr[8:16]))
    print("  reg 16-21: " + " ".join(f"{b:02X}" for b in rr[16:22]))
    amps = sum(bin(rr[r]).count("1") for r in range(14, 22))
    print(f"  DSP HPF: {'on' if rr[4] & 0x10 else 'off'} (cutoff code {rr[4] & 0x0F})"
          f" | BW DACs: RH1={rr[8]} RH2={rr[10]} RL={rr[12]}"
          f" | amps powered: {amps}/64")

    R = status['lfp_decim_R'] or 1
    print(f"\n--- LFP/DSP engine (Tier-1, UDP {UDP_PORT} stream_type=2) ---")
    print(f"  {'ENABLED' if status['lfp_enable'] else 'disabled'}  "
          f"lane_mask=0x{status['lfp_lane_mask']:02X} (= broadband channel_enable)  "
          f"decim_R={status['lfp_decim_R']} "
          f"(-> {30000.0/R:.0f} sps)  num_taps={status['lfp_num_taps']}")
    print(f"  packets sent: {status['lfp_packets_sent']}   "
          f"overrun: {'YES' if status['lfp_overrun'] else 'no'}")

    # Analytic chirp NCO config
    f_hi = (status['chirp_fspan'] << CHIRP_FSPAN_SHIFT) / (1 << CHIRP_PHW) * PACKET_RATE_HZ
    print(f"\n--- Analytic chirp NCO (CTRL_REG_3) ---")
    print(f"  {'ENABLED' if status['chirp_mode'] else 'disabled'}  "
          f"sweep 0->{f_hi:.0f} Hz  stride={status['chirp_stride']}  "
          f"(fspan={status['chirp_fspan']} rate={status['chirp_rate']})")
    print("=" * 50)

def set_udp_dest(sock, ip_str, port):
    """Configure UDP destination"""
    try:
        ip_int = int(ipaddress.IPv4Address(ip_str))
        success, _ = send_binary_command(sock, CMD_SET_UDP_DEST, ip_int, port)
        if success:
            print(f"[TCP] UDP destination set to {ip_str}:{port}")
            return True
        else:
            print(f"[TCP] Failed to set UDP destination")
            return False
    except Exception as e:
        print(f"[TCP] Error setting UDP destination: {e}")
        return False

def ping(sock, timeout=0.1):
    """Send lightweight ping to check link status without affecting UDP streaming"""
    success, _ = send_binary_command(sock, CMD_PING, timeout=timeout)
    if success:
        print(f"[TCP] Ping successful - link is up")
        return True
    else:
        print(f"[TCP] Ping failed - no response")
        return False

def manual_cable_test(sock):
    """Manual cable test using existing UDP infrastructure"""
    print("Manual cable test starting...")

    if not send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, CABLE_TEST_CHANNEL_ENABLE)[0]:
        print("Failed to set channel enable")
        return
    time.sleep(0.1)
    
    validator.start_manual_cable_test()
    collected_packets = []
    
    try:
        if not send_binary_command(sock, CMD_SET_LOOP_COUNT, 1)[0]:
            print("Failed to set loop count")
            return
        time.sleep(0.1)
        
        print("Running initialization...")
        if not send_binary_command(sock, CMD_LOAD_INIT)[0]:
            return
        time.sleep(0.1)
        
        if not send_binary_command(sock, CMD_START)[0]:
            return
        time.sleep(0.1)
        
        if not send_binary_command(sock, CMD_STOP)[0]:
            return
        
        init_words = validator.wait_for_manual_packet(timeout=5.0)
        if init_words is None:
            print("Timeout waiting for init packet")
            return
        collected_packets.append(init_words)
        print("Collected init packet")
        
        if not send_binary_command(sock, CMD_LOAD_CABLE_TEST)[0]:
            return
        time.sleep(0.1)
        
        for phase in range(16):
            print(f"Testing phase {phase}...")
            
            if not send_binary_command(sock, CMD_SET_PHASE, phase, phase)[0]:
                continue
            time.sleep(0.1)
            
            if not send_binary_command(sock, CMD_START)[0]:
                continue
            time.sleep(0.1)
            
            if not send_binary_command(sock, CMD_STOP)[0]:
                continue
            
            words = validator.wait_for_manual_packet(timeout=5.0)
            if words is None:
                print(f"Timeout waiting for phase {phase} packet")
                return
            collected_packets.append(words)
            print(f"Collected phase {phase} packet")
        
        print(f"\nCollected {len(collected_packets)} packets total")
        for i, words in enumerate(collected_packets):
            if i == 0:
                print(f"Packet {i + 1} (Init): Word 12: 0x{words[12]:08X}, Word 13: 0x{words[13]:08X}")
            else:
                phase = i - 1
                print(f"Packet {i + 1} (Phase1={phase}): Word 12: 0x{words[12]:08X}, Word 13: 0x{words[13]:08X}")
        
    except Exception as e:
        print(f"Error during manual cable test: {e}")
    finally:
        global manual_cable_test_mode
        manual_cable_test_mode = False

# ============================================================================
# AUTOMATED CABLE DETECTION FUNCTION
# ============================================================================

def run_detection(sock, verbose=True):

    # Create cable detector using our send_binary_command function
    def command_wrapper(cmd_id, param1=0, param2=0):
        return send_binary_command(sock, cmd_id, param1, param2)

    detector = CableDetection(command_wrapper)
    
    # Hook into UDP validator
    validator.set_cable_detector(detector)
    
    try:
        result = detector.detect(verbose=verbose)
        
        print("\n" + "="*60)
        print("DETECTION RESULTS")
        print("="*60)
        print(result.summary())
        
        if result.success:
            print("\nPhase Analysis:")
            print("Phase  CIPO0  CIPO1  DDR0  DDR1")
            print("-----  -----  -----  ----  ----")
            for pr in result.all_phases:
                marker = "*" if pr.phase == result.best_phase else " "
                print(f"{pr.phase:3d}{marker}  {pr.cipo0_score:5.0f}  {pr.cipo1_score:5.0f}  "
                      f"{'Yes' if pr.cipo0_has_ddr else 'No ':3s}  "
                      f"{'Yes' if pr.cipo1_has_ddr else 'No ':3s}")
            
            if detector.apply_config(result):
                print("\nConfiguration applied successfully!")
            else:
                print("\nFailed to apply configuration")
        
        return result
    
    finally:
        validator.set_cable_detector(None)


def configure_tcp_keepalive(sock):
    """Enable TCP keepalive to detect dead connections faster.

    Some keepalive tuning options are platform-specific: Linux uses
    TCP_KEEPIDLE while macOS uses TCP_KEEPALIVE for the idle time, and
    TCP_KEEPINTVL / TCP_KEEPCNT are not available on macOS. Guard each
    one so this works across platforms.
    """
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    # Idle time before the first keepalive probe is sent.
    if hasattr(socket, "TCP_KEEPIDLE"):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 1)
    elif hasattr(socket, "TCP_KEEPALIVE"):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPALIVE, 1)
    # Send probes every 1 second.
    if hasattr(socket, "TCP_KEEPINTVL"):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 1)
    # Close connection after 3 failed probes.
    if hasattr(socket, "TCP_KEEPCNT"):
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)


# ---------------------------------------------------------------------------
# Robust TCP connect.
#
# The board needs >20 s after power-on to bring up its lwIP listener, and while
# it is still booting -- or if its GEM RX has wedged (early-SYN Zynq-7000 RX-hang
# errata) -- it does not answer at all. A plain blocking connect() then either
# hangs for the OS connect timeout (~75 s on macOS) or, once the host's ARP entry
# for the board ages out and re-resolution fails, fails outright with
# "[Errno 64] Host is down" (EHOSTDOWN) / EHOSTUNREACH. That's the crash you were
# hitting. None of those mean "give up" -- they mean "the board isn't ready yet."
#
# So instead of one blocking connect we poll with a short per-attempt timeout and
# retry until the board answers: net.py waits patiently and connects the instant
# the listener is up, and never spews a traceback. errno values are compared
# symbolically because the numbers differ between macOS and Linux.
# ---------------------------------------------------------------------------
CONNECT_TIMEOUT   = 3.0    # seconds per connection attempt (bounds the "hang")
CONNECT_RETRY_GAP = 1.0    # seconds between attempts
CONNECT_MAX_WAIT  = None   # overall deadline (s); None = wait forever, Ctrl-C to stop

# errno values that just mean "board not ready yet, keep waiting".
_RETRYABLE_ERRNOS = {
    errno.ECONNREFUSED,   # host reachable but listener not up yet
    errno.EHOSTDOWN,      # no ARP reply -> board booting or RX wedged ([Errno 64])
    errno.EHOSTUNREACH,   # ARP incomplete / no route to the board yet
    errno.ENETUNREACH,
    errno.ENETDOWN,
    errno.ETIMEDOUT,      # SYN went unanswered
    errno.ECONNRESET,
    errno.ECONNABORTED,
}


def connect_with_retry(zynq_ip=None, tcp_port=None,
                       per_attempt_timeout=CONNECT_TIMEOUT,
                       retry_gap=CONNECT_RETRY_GAP,
                       max_wait=CONNECT_MAX_WAIT):
    """Return a connected, blocking TCP socket, or None if the deadline/Ctrl-C
    hits first. Retries on the 'board not ready' errno family instead of hanging
    on a single 75 s connect() or crashing on EHOSTDOWN."""
    zynq_ip = zynq_ip or ZYNQ_IP
    tcp_port = tcp_port or TCP_PORT
    start = time.time()
    attempt = 0
    last_reason = None
    try:
        while True:
            attempt += 1
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(per_attempt_timeout)
            try:
                sock.connect((zynq_ip, tcp_port))
                sock.settimeout(None)   # blocking again for the command session
                if attempt > 1:
                    print(f"[TCP] Board answered after {attempt} attempts "
                          f"({time.time() - start:.0f}s).")
                return sock
            except (socket.timeout, TimeoutError):
                reason = "no SYN-ACK (board still booting / listener down)"
                retry = True
            except OSError as e:
                reason = f"{e.__class__.__name__}: {e}"
                retry = getattr(e, "errno", None) in _RETRYABLE_ERRNOS
            try:
                sock.close()
            except OSError:
                pass
            if not retry:
                print(f"[TCP] Connect failed (non-retryable): {reason}")
                return None
            if max_wait is not None and (time.time() - start) >= max_wait:
                print(f"[TCP] Gave up waiting for {zynq_ip}:{tcp_port} after "
                      f"{max_wait:.0f}s ({reason}).")
                return None
            # Only reprint when the reason changes, so a long wait doesn't spam.
            if reason != last_reason:
                print(f"[TCP] Waiting for board at {zynq_ip}:{tcp_port} ... "
                      f"({reason}). Retrying every {retry_gap:.0f}s; Ctrl-C to stop.")
                last_reason = reason
            time.sleep(retry_gap)
    except KeyboardInterrupt:
        print("\n[TCP] Connect wait cancelled by user.")
        return None


def tcp_control():
    sock = connect_with_retry()
    if sock is None:
        return
    try:
        print(f"[TCP] Connected to {ZYNQ_IP}:{TCP_PORT}")

        # Enable TCP keepalive to detect dead connections faster
        configure_tcp_keepalive(sock)
        print(f"[TCP] TCP keepalive enabled (detect disconnection in ~3-5 seconds)")

        # Auto-configure UDP destination
        local_ip = get_local_ip()
        print(f"[TCP] Detected local IP: {local_ip}")
        print(f"[TCP] Configuring device to send UDP to this machine...")
        
        if set_udp_dest(sock, local_ip, UDP_PORT):
            print(f"[TCP] Device configured to send UDP packets here")
        else:
            print(f"[TCP] Failed to configure UDP destination")
            print(f"[TCP] Device may still be sending to default: 192.168.18.100:{UDP_PORT}")
        
        # Get and display initial status
        print("\n[TCP] Getting initial device status...")
        status = get_status(sock)
        if status:
            print_status(status)
            validator.set_channel_enable(status['channel_enable'])
        
        print(f"\n[TCP] Available commands:")
        print(f"  Basic: start, stop, reset_timestamp, loop <count>")
        print(f"  COPI: convert, init, cable_test, full_cable_test, manual_cable_test")
        print(f"  Config: set_phase <p0> <p1> [p2 p3], set_debug <0|1>, set_channels <0x00-0xFF>")
        print(f"  Network: set_udp <ip> <port>, get_status, perf_reset, ping")
        print(f"  Debug: dump_bram [start] [count], stats, hex")
        print(f"  Bandwidth: bench [bytes] [n], bench_sweep  (raw UDP throughput, port 5002)")
        print(f"  LFP (Tier-1): lfp_config [cic|fir] [taps], lfp_on, lfp_off, lfp_recv [n], sink  (port 5000, stream_type=2)")
        print(f"         verify_sine [ce=FF] [n=300] - check debug sinewaves vs RTL ref")
        print(f"  Chirp: chirp [f_max=1400] [period=2.0] [stride=4], chirp_off  (analytic swept sine)")
        print(f"  LFP sweep: lfp_sweep [f_max=1490] [period=2.0] [n_periods=2]  (measure anti-alias |H(f)|)")
        print(f"  auto_cable_detect - Automated cable detection!")
        print(f"  Aux: aux_demo, aux_en <0|1>, aux_bank <slot> <bank>, aux")
        print(f"       read_reg <r>, write_reg <r> <v>")
        print(f"       fast_settle <0|1> [dsp] | gpio <pin> | off")
        print(f"       digout <0|1> | gpio <pin> | hiz")
        print(f"  Utility: help, quit")
        
        while True:
            try:
                cmd = input("\n[TCP] Command: ").strip().lower()

                if cmd == "quit":
                    break
                elif cmd == "auto_cable_detect":
                    run_detection(sock, verbose=True)
                elif cmd == "start":
                    send_binary_command(sock, CMD_START)
                elif cmd == "stop":
                    send_binary_command(sock, CMD_STOP)
                elif cmd == "reset_timestamp":
                    send_binary_command(sock, CMD_RESET_TIMESTAMP)
                    validator.last_timestamp = None
                elif cmd == "convert":
                    send_binary_command(sock, CMD_LOAD_CONVERT)
                elif cmd == "init":
                    send_binary_command(sock, CMD_LOAD_INIT)
                elif cmd == "cable_test":
                    send_binary_command(sock, CMD_LOAD_CABLE_TEST)
                elif cmd == "full_cable_test":
                    if send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, CABLE_TEST_CHANNEL_ENABLE)[0]:
                        validator.start_cable_test_capture()
                        send_binary_command(sock, CMD_FULL_CABLE_TEST)
                elif cmd == "manual_cable_test":
                    manual_cable_test(sock)
                elif cmd == "get_status":
                    status = get_status(sock)
                    if status:
                        print_status(status)
                elif cmd == "perf_reset":
                    ok, _ = send_binary_command(sock, CMD_PERF_RESET)
                    print("[PERF] window reset" if ok else "[PERF] reset failed")
                elif cmd == "ping":
                    ping(sock)
                elif cmd.startswith("loop "):
                    try:
                        loop_count = int(cmd.split()[1])
                        send_binary_command(sock, CMD_SET_LOOP_COUNT, loop_count)
                    except (ValueError, IndexError):
                        print("Usage: loop <count>")
                elif cmd.startswith("set_phase "):
                    try:
                        parts = cmd.split()
                        if len(parts) == 3:   # port A only
                            send_binary_command(sock, CMD_SET_PHASE, int(parts[1]), int(parts[2]))
                        elif len(parts) == 5: # both ports: p0 p1 p2 p3
                            send_binary_command(sock, CMD_SET_PHASE, int(parts[1]), int(parts[2]))
                            send_binary_command(sock, CMD_SET_PHASE_B, int(parts[3]), int(parts[4]))
                        else:
                            print("Usage: set_phase <p0> <p1> [p2 p3]   (p2/p3 = port B)")
                    except ValueError:
                        print("Invalid phase values")
                elif cmd.startswith("set_debug "):
                    try:
                        debug_mode = int(cmd.split()[1])
                        send_binary_command(sock, CMD_SET_DEBUG_MODE, debug_mode)
                    except (ValueError, IndexError):
                        print("Usage: set_debug <0|1>")
                elif cmd.startswith("set_channels "):
                    try:
                        val = cmd.split()[1]
                        channel_enable = int(val, 16) if val.startswith('0x') else int(val)
                        if 0 <= channel_enable <= 0xFF:
                            if send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, channel_enable)[0]:
                                validator.set_channel_enable(channel_enable)
                        else:
                            print("Channel enable must be 0x00-0xFF ([3:0]=port A, [7:4]=port B)")
                    except (ValueError, IndexError):
                        print("Usage: set_channels <0x00-0xFF>")
                elif cmd.startswith("verify_sine"):
                    try:
                        parts = cmd.split()
                        ce = 0xFF
                        n = 300
                        if len(parts) >= 2:
                            ce = int(parts[1], 16)   # always hex, e.g. ff or 0xff
                        if len(parts) >= 3:
                            n = int(parts[2])
                        verify_debug_sine(sock, ce, n)
                    except ValueError:
                        print("Usage: verify_sine [channel_enable hex, default FF] [n_packets, default 300]")
                elif cmd.startswith("set_udp "):
                    try:
                        parts = cmd.split()
                        if len(parts) == 3:
                            set_udp_dest(sock, parts[1], int(parts[2]))
                        else:
                            print("Usage: set_udp <ip> <port>")
                    except (ValueError, IndexError):
                        print("Invalid IP or port")
                elif cmd.startswith("dump_bram"):
                    try:
                        parts = cmd.split()
                        start_addr = int(parts[1]) if len(parts) > 1 else 0
                        word_count = int(parts[2]) if len(parts) > 2 else 10
                        send_binary_command(sock, CMD_DUMP_BRAM, start_addr, word_count)
                    except (ValueError, IndexError):
                        send_binary_command(sock, CMD_DUMP_BRAM, 0, 10)
                elif cmd == "stats":
                    validator.print_statistics()
                elif cmd == "hex":
                    validator.print_last_packet_hex()
                elif cmd == "bench" or cmd.startswith("bench "):
                    # measure raw UDP throughput vs packet size (port 5002 blaster)
                    parts = cmd.split()
                    sz = int(parts[1]) if len(parts) > 1 else 1472
                    n  = int(parts[2]) if len(parts) > 2 else 50000
                    udp_bench(sock, sz, n)
                elif cmd == "bench_sweep":
                    udp_bench_sweep(sock)
                elif cmd == "lfp_config" or cmd.startswith("lfp_config "):
                    # lfp_config [datapath=cic|fir] [taps]  (configure while off)
                    # default = CIC^4(/5)+halfband(/2)=/10 -> 3 kHz, 43 comp taps.
                    # The LFP lane mask mirrors the broadband channel_enable mask
                    # (single source of truth) -- pick lanes with set_channels.
                    parts = cmd.split()
                    dp   = parts[1] if len(parts) > 1 else "cic"
                    taps = int(parts[2]) if len(parts) > 2 else None
                    configure_lfp(sock, datapath=dp, num_taps=taps)
                elif cmd == "chirp" or cmd.startswith("chirp "):
                    # chirp [f_max_hz] [period_s] [stride]  (analytic swept-sine debug)
                    parts = cmd.split()
                    fmx = float(parts[1]) if len(parts) > 1 else 1400.0
                    per = float(parts[2]) if len(parts) > 2 else 2.0
                    std = int(parts[3])   if len(parts) > 3 else 4
                    configure_chirp(sock, fmx, per, std, enable=True)
                elif cmd == "chirp_off":
                    configure_chirp(sock, enable=False)
                    send_binary_command(sock, CMD_SET_DEBUG_MODE, 0)
                elif cmd == "lfp_sweep" or cmd.startswith("lfp_sweep "):
                    # lfp_sweep [f_max=1490] [period=3.0] [n_periods=2]
                    # chirp across the band + measure the LFP anti-alias |H(f)|
                    parts = cmd.split()
                    fmx = float(parts[1]) if len(parts) > 1 else 1490.0
                    per = float(parts[2]) if len(parts) > 2 else 3.0
                    npd = int(parts[3])   if len(parts) > 3 else 2
                    measure_lfp_response(sock, f_max=fmx, period=per, n_periods=npd)
                elif cmd == "lfp_on":
                    st = get_status(sock)        # quietly read back the (broadband) lane mask
                    if st and st.get('lfp_lane_mask', 0) == 0:
                        print("[LFP] lane_mask is 0 (no broadband streams enabled) -- run "
                              "set_channels first, or no data will stream.")
                    lfp_enable(sock, True)
                elif cmd == "lfp_off":
                    lfp_enable(sock, False)
                elif cmd == "lfp_recv" or cmd.startswith("lfp_recv "):
                    # read decoded LFP frames from the persistent sink (blocks until n or timeout)
                    parts = cmd.split()
                    n = int(parts[1]) if len(parts) > 1 else 200
                    receive_lfp(n)
                elif cmd == "lfp_sink" or cmd == "sink":
                    if UNIFIED_SINK is not None and UNIFIED_SINK._running:
                        print(f"[UDP-SINK] draining UDP {UNIFIED_SINK.port}: "
                              f"broadband={UNIFIED_SINK.bb_pkts} LFP={UNIFIED_SINK.lfp_pkts} "
                              f"other={UNIFIED_SINK.other_pkts} pkts, "
                              f"LFP={UNIFIED_SINK.lfp_bytes/1024.0:.0f} KB, "
                              f"bb_seq_gaps={validator.seq_gaps} lfp_seq_gaps={UNIFIED_SINK._lfp_gaps}, "
                              f"host-ring-drops={UNIFIED_SINK._ring_drops}, last from {UNIFIED_SINK.last_addr}")
                    else:
                        print("[UDP-SINK] not running")
                elif cmd == "aux":
                    validator.print_aux_info()
                elif cmd == "aux_demo":
                    aux_demo_setup(sock)
                elif cmd.startswith("aux_en "):
                    try:
                        aux_seq_enable(sock, int(cmd.split()[1]))
                    except (ValueError, IndexError):
                        print("Usage: aux_en <0|1>")
                elif cmd.startswith("aux_bank "):
                    try:
                        parts = cmd.split()
                        aux_bank_select(sock, int(parts[1]), int(parts[2]))
                    except (ValueError, IndexError):
                        print("Usage: aux_bank <slot> <bank>")
                elif cmd.startswith("read_reg "):
                    try:
                        read_register(sock, int(cmd.split()[1]))
                    except (ValueError, IndexError):
                        print("Usage: read_reg <reg 0-63>")
                elif cmd.startswith("write_reg "):
                    try:
                        parts = cmd.split()
                        val = int(parts[2], 16) if parts[2].startswith('0x') else int(parts[2])
                        write_register(sock, int(parts[1]), val)
                    except (ValueError, IndexError):
                        print("Usage: write_reg <reg> <value>")
                elif cmd.startswith("fast_settle "):
                    try:
                        parts = cmd.split()
                        if parts[1] in ("0", "1"):           # fast_settle <0|1> [dsp]
                            set_fast_settle(sock, amp_sw=parts[1] == "1",
                                            dsp_sw=(len(parts) > 2 and parts[2] == "1"))
                        elif parts[1] == "gpio":             # fast_settle gpio <pin>
                            set_fast_settle(sock, amp_gpio_en=True, amp_pin=int(parts[2]))
                        elif parts[1] == "off":
                            set_fast_settle(sock)
                        else:
                            print("Usage: fast_settle <0|1> [dsp] | gpio <pin> | off")
                    except (ValueError, IndexError):
                        print("Usage: fast_settle <0|1> [dsp] | gpio <pin> | off")
                elif cmd.startswith("digout "):
                    try:
                        parts = cmd.split()
                        if parts[1] in ("0", "1"):           # digout <0|1>  (HiZ off)
                            set_digout(sock, sw=parts[1] == "1", reg3_static=0x00)
                        elif parts[1] == "gpio":             # digout gpio <pin>
                            set_digout(sock, gpio_en=True, pin=int(parts[2]), reg3_static=0x00)
                        elif parts[1] == "hiz":              # digout hiz (release the pin)
                            set_digout(sock, reg3_static=0x02)
                        else:
                            print("Usage: digout <0|1> | gpio <pin> | hiz")
                    except (ValueError, IndexError):
                        print("Usage: digout <0|1> | gpio <pin> | hiz")
                elif cmd == "help":
                    print("Commands:")
                    print("  start, stop, reset_timestamp")
                    print("  loop <count>, set_phase <p0> <p1>")
                    print("  set_debug <0|1>, set_channels <0x00-0xFF>")
                    print("  convert, init, cable_test")
                    print("  full_cable_test, manual_cable_test")
                    print("  auto_cable_detect - NEW: Automated detection!")
                    print("  set_udp <ip> <port>, get_status, perf_reset, ping")
                    print("  dump_bram [start] [count]")
                    print("  stats, hex, quit")
                    print("Bandwidth (raw UDP throughput, port 5002):")
                    print("  bench [bytes] [n]   - one size (default 1472 B x 50000)")
                    print("  bench_sweep         - sweep 256..8800 B")
                    print("LFP / Tier-1 (unified UDP 5000, stream_type=2):")
                    print("  lfp_config [cic|fir] [taps] - set datapath/taps + upload LP kernel")
                    print("  lfp_on / lfp_off    - enable / disable the engine")
                    print("  lfp_recv [n]        - capture + print decoded LFP frames")
                    print("  (LFP lane mask = the broadband channel_enable mask -- pick lanes")
                    print("   with set_channels; run lfp_config + set_channels BEFORE lfp_on)")
                    print("Aux sequencer (bank-programmable aux commands):")
                    print("  aux_demo            - load default slot programs + enable")
                    print("  aux_en <0|1>        - enable/disable the sequencer")
                    print("  aux_bank <slot> <bank> - atomic bank swap (live)")
                    print("  aux                 - decode last packet's command echo")
                    print("  read_reg <r> / write_reg <r> <v> - runtime RHD register access")
                    print("  fast_settle <0|1> [dsp] | gpio <pin> | off")
                    print("  digout <0|1> | gpio <pin> | hiz")
                else:
                    print(f"Unknown command: '{cmd}'. Type 'help' for list.")

            except (socket.timeout, ConnectionError, BrokenPipeError, OSError) as e:
                print(f"\n[TCP] Connection lost: {e}")
                print(f"[TCP] Attempting to reconnect...")
                try:
                    sock.close()
                except OSError:
                    pass
                # Bounded reconnect: wait up to 60 s for the board to come back
                # (it may be rebooting), then hand control back rather than hang.
                sock = connect_with_retry(max_wait=60.0)
                if sock is None:
                    print(f"[TCP] Reconnection failed.")
                    print(f"[TCP] Please check network cable and device, then restart.")
                    break
                # Re-enable TCP keepalive on reconnection
                configure_tcp_keepalive(sock)
                print(f"[TCP] Reconnected successfully!")
                
    except ConnectionRefusedError:
        print(f"[TCP] Could not connect to {ZYNQ_IP}:{TCP_PORT}")
    except KeyboardInterrupt:
        print("\n[TCP] Closing connection")
    finally:
        sock.close()

if __name__ == "__main__":
    print("=== Zynq BRAM Data Generator Validator ===")

    # Discover the board by listening for its broadcast beacon -- passive, we send
    # nothing until we've heard it (so we can't disturb the board during boot).
    # Auto-fills ZYNQ_IP from the beacon's source address; falls back to the
    # configured address if no beacon arrives (older firmware without the beacon).
    print(f"[DISCOVERY] Listening for a board beacon on UDP {BEACON_PORT} "
          f"(up to {BEACON_DISCOVERY_TIMEOUT:.0f}s)...")
    _dev = discover_board()
    if _dev:
        ZYNQ_IP = _dev['ip']
        TCP_PORT = _dev['tcp_port'] or TCP_PORT
        UDP_PORT = _dev['udp_port'] or UDP_PORT
        print(f"[DISCOVERY] Found board at {_dev['ip']}  fw {_dev['fw']}  "
              f"MAC {_dev['mac']}  (TCP {_dev['tcp_port']}, UDP {_dev['udp_port']})")
    else:
        print(f"[DISCOVERY] No beacon heard -- using configured {ZYNQ_IP} "
              f"(older firmware without the beacon, or wrong subnet?)")

    print(f"Device: {ZYNQ_IP}:{TCP_PORT}")
    print(f"UDP Port: {UDP_PORT}")
    print("Press Ctrl+C to stop.\n")

    # ONE socket on UDP_PORT (5000) for ALL streams. The UnifiedSink drains it
    # promiscuously (recv->ring), demuxes by stream_type (broadband -> validator,
    # LFP -> subscribers), and verifies per-stream SEQ continuity. Draining 5000
    # for the whole session also keeps the host from replying ICMP port-
    # unreachable to the board (an unconsumed UDP port => ~1 ICMP/packet => an
    # RX-interrupt storm that preempts the board's polled loop). Started here (not
    # in tcp_control) so it drains regardless of the TCP control state.
    UNIFIED_SINK = UnifiedSink(port=UDP_PORT)
    UNIFIED_SINK.start()

    tcp_control()

    # Shutdown summary: the broadband no-loss assertion (gap count MUST be 0).
    if UNIFIED_SINK is not None:
        UNIFIED_SINK.stop()
        print(f"\n[UDP] demux summary: broadband={UNIFIED_SINK.bb_pkts} pkts, "
              f"LFP={UNIFIED_SINK.lfp_pkts} pkts, other={UNIFIED_SINK.other_pkts}, "
              f"host-ring-drops={UNIFIED_SINK._ring_drops}")
        print(f"[UDP] broadband SEQ gaps = {validator.seq_gaps} "
              f"({validator.seq_lost_packets} packets implied missing)  "
              f"{'OK (no loss)' if validator.seq_gaps == 0 else 'LOSS DETECTED'}")
        print(f"[UDP] LFP SEQ gaps = {UNIFIED_SINK._lfp_gaps}")
    validator.print_statistics()
    time.sleep(0.5)
