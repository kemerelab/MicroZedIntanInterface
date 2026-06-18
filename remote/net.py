import socket
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
UDP_PORT = 5000  # Must match your board's UDP_PORT

# Updated data generator constants
MAGIC_NUMBER_LOW = 0xDEADBEEF
MAGIC_NUMBER_HIGH = 0xCAFEBABE

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
CABLE_TEST_PACKET_SIZE_WORDS = 74  # 4 header + 70 data words
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
CABLE_TEST_PACKET_SIZE_WORDS = 74
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
CABLE_TEST_PACKET_SIZE_WORDS = 80
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
        
        # Extract this channel's data (every other word, starting at channel offset)
        data_words = packet[4:]  # Skip header
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

def calculate_packet_size(channel_enable):
    """Total packet size in words (header + data). Up to 10 + 140 = 150."""
    return 10 + calculate_data_words(channel_enable)

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
                        print(f"Packet {cable_test_packets_captured + 1} (Init): Word 8: 0x{words[8]:08X}, Word 9: 0x{words[9]:08X}")
                    else:
                        phase1 = cable_test_packets_captured - 1
                        print(f"Packet {cable_test_packets_captured + 1} (Phase1={phase1}): Word 8: 0x{words[8]:08X}, Word 9: 0x{words[9]:08X}")
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

            magic_combined = (words[1] << 32) | words[0]
            expected_magic = (MAGIC_NUMBER_HIGH << 32) | MAGIC_NUMBER_LOW

            if magic_combined != expected_magic:
                self.magic_errors += 1
                self.error_count += 1
                print(f"[ERROR] Packet {self.packet_count}: Magic number mismatch")
                return None            
            
            timestamp = (words[3] << 32) | words[2]

            # Debug-sine capture: stash data words for offline verification.
            if self.sine_capture_active and len(self.sine_capture) < self.sine_capture_target:
                self.sine_capture.append((timestamp, words[10:]))
                if len(self.sine_capture) >= self.sine_capture_target:
                    self.sine_capture_active = False

            now = time.time()
            if self.packet_count % 30000 == 0 or (now - self.last_stats_time) >= 5.0:
                elapsed = now - self.start_time
                total_rate = self.packet_count / elapsed if elapsed > 0 else 0
                inst_rate = (self.packet_count - self.last_packet_count) / (now - self.last_stats_time) if (now - self.last_stats_time) > 0 else 0
                
                if len(words) >= 14:
                    data_sample = f"Data: [0x{words[10]:08X}, 0x{words[11]:08X}, 0x{words[12]:08X}, 0x{words[13]:08X}]"
                else:
                    data_sample = f"Data: [packet too short]"
                
                print(f"[INFO] Packet {self.packet_count}: Timestamp {timestamp}, "
                      f"Rate: {total_rate:.1f} pkt/s (avg), {inst_rate:.1f} pkt/s (inst), "
                      f"Errors: {self.error_count}")
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
        Header 64-bit word 2 (32-bit words 4/5):
          word4 = {echo_slot0[15:0], flags[7:0], digital_in[7:0]}
          word5 = {echo_slot2_prev[15:0], echo_slot1_prev[15:0]}
        Result locations (SPI 2-command pipeline): slot-0's result is THIS
        packet's data word 34; slot-1/2 echoes label data words 0 and 1."""
        if self.last_packet_words is None or len(self.last_packet_words) < 6:
            print("[AUX] No packet captured yet")
            return
        w4, w5 = self.last_packet_words[4], self.last_packet_words[5]
        digital_in = w4 & 0xFF
        flags = (w4 >> 8) & 0xFF
        if not (flags & 0x01):
            print(f"[AUX] Sequencer inactive in last packet (digital_in=0x{digital_in:02X})")
            return
        echo0 = (w4 >> 16) & 0xFFFF
        echo1 = w5 & 0xFFFF
        echo2 = (w5 >> 16) & 0xFFFF
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


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Big receive buffer: at ~30k pkt/s and up to 600 B/pkt the default buffer
    # overflows on any brief stall and drops packets (worse at dual-port). Ask
    # for 16 MB; the OS may clamp to net.core.rmem_max (raise that to use it).
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
        got = sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF)
        print(f"[UDP] SO_RCVBUF = {got // 1024} KB "
              f"(raise net.core.rmem_max if smaller than requested)")
    except OSError as e:
        print(f"[UDP] Could not set SO_RCVBUF: {e}")
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening on port {UDP_PORT}...")

    last_timestamp = None
    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                total_len = len(data)

                for offset in range(0, total_len - validator.expected_packet_size_bytes + 1, validator.expected_packet_size_bytes):
                    chunk = data[offset:offset + validator.expected_packet_size_bytes]
                    timestamp = validator.validate_packet(chunk)
                    if timestamp is not None:
                        if last_timestamp is not None and timestamp != last_timestamp + 1:
                            validator.timestamp_errors += 1
                            validator.error_count += 1
                        last_timestamp = timestamp
            except socket.timeout:
                continue
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        print("\n[UDP] Stopping UDP listener")
    finally:
        sock.close()
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

    if len(data) != 126:
        print(f"[TCP] Invalid status response length: {len(data)} (expected 126)")
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
    print(f"Recv->transmit: last {to_us(status['loop_ticks_last']):.2f} us, max {to_us(status['loop_ticks_max']):.2f} us")
    lm = to_us(status['loop_ticks_max'])
    print(f"Headroom (max): {33.3 - lm:.2f} us  ({100.0*lm/33.3:.0f}% of budget used)")
    print(f"DMA errors: {status['dma_errors']}   (timer {hz/1e6:.1f} MHz)")
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
                print(f"Packet {i + 1} (Init): Word 8: 0x{words[8]:08X}, Word 9: 0x{words[9]:08X}")
            else:
                phase = i - 1
                print(f"Packet {i + 1} (Phase1={phase}): Word 8: 0x{words[8]:08X}, Word 9: 0x{words[9]:08X}")
        
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


def tcp_control():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ZYNQ_IP, TCP_PORT))
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
        print(f"  Network: set_udp <ip> <port>, get_status, ping")
        print(f"  Debug: dump_bram [start] [count], stats, hex")
        print(f"         verify_sine [ce=FF] [n=300] - check debug sinewaves vs RTL ref")
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
                    print("  set_udp <ip> <port>, get_status, ping")
                    print("  dump_bram [start] [count]")
                    print("  stats, hex, quit")
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
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.connect((ZYNQ_IP, TCP_PORT))

                    # Re-enable TCP keepalive on reconnection
                    configure_tcp_keepalive(sock)

                    print(f"[TCP] Reconnected successfully!")
                except Exception as reconnect_error:
                    print(f"[TCP] Reconnection failed: {reconnect_error}")
                    print(f"[TCP] Please check network cable and device, then restart.")
                    break
                
    except ConnectionRefusedError:
        print(f"[TCP] Could not connect to {ZYNQ_IP}:{TCP_PORT}")
    except KeyboardInterrupt:
        print("\n[TCP] Closing connection")
    finally:
        sock.close()

if __name__ == "__main__":
    print("=== Zynq BRAM Data Generator Validator ===")
    print(f"Device: {ZYNQ_IP}:{TCP_PORT}")
    print(f"UDP Port: {UDP_PORT}")
    print("Press Ctrl+C to stop.\n")
    
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()
    
    tcp_control()
    
    time.sleep(0.5)
