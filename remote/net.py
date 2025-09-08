import socket
import threading
import struct
import time
import random
import queue

ZYNQ_IP = "192.168.18.10"  # IP of the Zynq board
TCP_PORT = 6000  # Must match your board's TCP_PORT
UDP_PORT = 5000  # Must match your board's UDP_PORT

# Updated data generator constants for your current implementation
MAGIC_NUMBER_LOW = 0xDEADBEEF   # Lower 32 bits
MAGIC_NUMBER_HIGH = 0xCAFEBABE  # Upper 32 bits
# EXPECTED_PACKET_SIZE = 296      # 74 words * 4 bytes = 296 bytes
# WORDS_PER_PACKET = 74           # 4 header + (35 cycles * 2 data words)

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
CMD_LOAD_CONVERT = 0x20
CMD_LOAD_INIT = 0x21
CMD_LOAD_CABLE_TEST = 0x22
CMD_LOAD_TEST_PATTERN = 0x23
CMD_FULL_CABLE_TEST = 0x30
CMD_GET_STATUS = 0x40
CMD_DUMP_BRAM = 0x41
CMD_HELP = 0x42

# ACK status codes
ACK_SUCCESS = 0x06
ACK_ERROR = 0x15

# Cable test globals
cable_test_mode = False
cable_test_packets_captured = 0

# Manual cable test globals
manual_cable_test_mode = False
manual_cable_test_packets = []      # kept for compatibility; not used after fix
manual_cable_test_waiting = False   # kept for compatibility; not used after fix

def calculate_data_words(channel_enable):
    """Calculate number of 32-bit data words based on channel enable setting"""
    num_channels = 0
    
    # Count enabled channels
    if channel_enable & 0x01: num_channels += 1  # CIPO0 regular
    if channel_enable & 0x02: num_channels += 1  # CIPO0 DDR
    if channel_enable & 0x04: num_channels += 1  # CIPO1 regular  
    if channel_enable & 0x08: num_channels += 1  # CIPO1 DDR
    
    if num_channels == 0:
        print("WARNING: No channels enabled, defaulting to all channels")
        return 70  # Default to maximum (4 channels × 35 cycles ÷ 2)
    
    # Calculate 32-bit words needed for the data
    # Each cycle produces num_channels × 16-bit words
    # Total 16-bit words = 35 × num_channels
    # Convert to 32-bit words with proper rounding up
    total_16bit_words = 35 * num_channels
    data_32bit_words = (total_16bit_words + 1) // 2  # Round up division
    
    return data_32bit_words

def calculate_packet_size(channel_enable):
    """Calculate total packet size in words (header + data)"""
    PACKET_HEADER_WORDS = 4  # Magic number + timestamp
    return PACKET_HEADER_WORDS + calculate_data_words(channel_enable)

def channel_enable_to_string(channel_enable):
    """Convert channel enable bits to human readable string"""
    channels = []
    if channel_enable & 0x01: channels.append("CIPO0_REG")
    if channel_enable & 0x02: channels.append("CIPO0_DDR")
    if channel_enable & 0x04: channels.append("CIPO1_REG")
    if channel_enable & 0x08: channels.append("CIPO1_DDR")
    
    if not channels:
        return "NONE"
    return ", ".join(channels)

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
        self.last_packet_raw = None  # Store raw bytes
        self.last_packet_words = None  # Store unpacked words

        # Channel enable tracking
        self.current_channel_enable = 0x0F  # Default: all channels enabled
        self.expected_packet_size_bytes = calculate_packet_size(self.current_channel_enable) * 4
        self.expected_packet_size_words = calculate_packet_size(self.current_channel_enable)

        # ---- added: thread-safe synchronization for manual mode ----
        self._manual_queue = queue.Queue()
        self._manual_lock = threading.Lock()

    def set_channel_enable(self, channel_enable):
        """Update channel enable setting and recalculate packet sizes"""
        self.current_channel_enable = channel_enable
        self.expected_packet_size_words = calculate_packet_size(channel_enable)
        self.expected_packet_size_bytes = self.expected_packet_size_words * 4
        
        print(f"[INFO] Channel enable updated to 0x{channel_enable:X}")
        print(f"[INFO] Enabled channels: {channel_enable_to_string(channel_enable)}")
        print(f"[INFO] Expected packet size: {self.expected_packet_size_words} words ({self.expected_packet_size_bytes} bytes)")

    def start_cable_test_capture(self):
        """Start capturing cable test packets"""
        global cable_test_mode, cable_test_packets_captured
        cable_test_mode = True
        cable_test_packets_captured = 0
        print("Starting cable test packet capture...")

    def start_manual_cable_test(self):
        """Start manual cable test mode"""
        global manual_cable_test_mode
        manual_cable_test_mode = True
        # clear any stale packets from previous runs
        with self._manual_lock:
            while not self._manual_queue.empty():
                try:
                    self._manual_queue.get_nowait()
                except queue.Empty:
                    break
        print("Manual cable test mode started")

    def wait_for_manual_packet(self, timeout=5.0):
        """Block until one manual-test packet arrives; return unpacked words or None on timeout."""
        try:
            words = self._manual_queue.get(timeout=timeout)
            return words
        except queue.Empty:
            return None

    def get_manual_test_packets(self):
        """Drain collected manual test packets and exit manual mode"""
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
        
    def validate_packet(self, data):
        global cable_test_mode, cable_test_packets_captured
        global manual_cable_test_mode, manual_cable_test_packets, manual_cable_test_waiting
        
        self.packet_count += 1
        self.last_packet_raw = data  # Store raw packet data

        # Handle cable test mode
        if cable_test_mode:
            if cable_test_packets_captured < 17:
                # Process cable test packet (cable tests use all channels enabled)
                expected_size = calculate_packet_size(0x0F) * 4  # All channels for cable test
                if len(data) == expected_size:
                    words = struct.unpack(f'<{calculate_packet_size(0x0F)}I', data)
                    self.last_packet_words = words  # Store the words for hex command
                    if cable_test_packets_captured == 0:
                        print(f"Packet {cable_test_packets_captured + 1} (Init): Word 8: 0x{words[8]:08X}, Word 9: 0x{words[9]:08X}")
                    else:
                        phase1 = cable_test_packets_captured - 1
                        print(f"Packet {cable_test_packets_captured + 1} (Phase1={phase1}): Word 8: 0x{words[8]:08X}, Word 9: 0x{words[9]:08X}")
                cable_test_packets_captured += 1
                
                if cable_test_packets_captured >= 17:
                    cable_test_mode = False
                    print("Cable test capture complete.")
                
                return None  # Don't process as normal packet
            else:
                cable_test_mode = False

        # Handle manual cable test mode  
        if manual_cable_test_mode:
            expected_size = calculate_packet_size(0x0F) * 4  # All channels for cable test
            if len(data) == expected_size:
                words = struct.unpack(f'<{calculate_packet_size(0x0F)}I', data)
                # enqueue to wake the waiting TCP/command thread
                try:
                    self._manual_queue.put_nowait(words)
                except queue.Full:
                    pass
                print("Captured manual test packet")
            return None  # Don't process as normal packet

        if self.start_time is None:
            self.start_time = time.time()
            self.last_stats_time = self.start_time

        if len(data) != self.expected_packet_size_bytes:
            self.size_errors += 1
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Wrong size {len(data)}, expected {self.expected_packet_size_bytes}")
            print(f"[ERROR] Current channel enable: 0x{self.current_channel_enable:X} ({channel_enable_to_string(self.current_channel_enable)})")
            hex_dump = ' '.join(f'{b:02X}' for b in data[:32])
            print(f"[DEBUG] First 32 bytes: {hex_dump}")
            return None

        try:
            # Unpack as 32-bit little-endian words
            words = struct.unpack(f'<{self.expected_packet_size_words}I', data)
            self.last_packet_words = words  # Store unpacked words

            # Check magic number (words 0 and 1)
            magic_combined = (words[1] << 32) | words[0]
            expected_magic = (MAGIC_NUMBER_HIGH << 32) | MAGIC_NUMBER_LOW

            if magic_combined != expected_magic:
                self.magic_errors += 1
                self.error_count += 1
                print(f"[ERROR] Packet {self.packet_count}: Magic number mismatch")
                print(f"[ERROR] Expected: 0x{expected_magic:016X}, Got: 0x{magic_combined:016X}")
                return None            
            
            # Extract timestamp (words 2 and 3)
            timestamp = (words[3] << 32) | words[2]

            # Show periodic stats
            now = time.time()
            if self.packet_count % 30000 == 0 or (now - self.last_stats_time) >= 5.0:
                elapsed = now - self.start_time
                total_rate = self.packet_count / elapsed if elapsed > 0 else 0
                inst_rate = (self.packet_count - self.last_packet_count) / (now - self.last_stats_time) if (now - self.last_stats_time) > 0 else 0
                
                # Show some data words for verification (first few data words after header)
                if len(words) >= 8:
                    data_sample = f"Data: [0x{words[4]:08X}, 0x{words[5]:08X}, 0x{words[6]:08X}, 0x{words[7]:08X}]"
                else:
                    data_sample = f"Data: [packet too short for data display]"
                
                print(f"[INFO] Packet {self.packet_count}: Timestamp {timestamp}, "
                      f"Rate: {total_rate:.1f} pkt/s (avg), {inst_rate:.1f} pkt/s (inst), "
                      f"Errors: {self.error_count}")
                print(f"       {data_sample}")
                print(f"       Channel enable: 0x{self.current_channel_enable:X} ({channel_enable_to_string(self.current_channel_enable)})")
                
                self.last_stats_time = now
                self.last_packet_count = self.packet_count

            return timestamp

        except struct.error as e:
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Failed to unpack data: {e}")
            print(f"[ERROR] Expected {self.expected_packet_size_words} words, got {len(data)} bytes")
            hex_dump = ' '.join(f'{b:02X}' for b in data[:64])
            print(f"[DEBUG] First 64 bytes: {hex_dump}")
            return None

    def print_last_packet_hex(self, words_per_line=8):
        """Print the most recent packet in hex format"""
        if self.last_packet_words is None:
            print("[INFO] No packets received yet")
            return
            
        print(f"\n=== LAST PACKET - HEX DUMP ===")
        print(f"Channel enable: 0x{self.current_channel_enable:X} ({channel_enable_to_string(self.current_channel_enable)})")
        print(f"Packet size: {len(self.last_packet_words)} words")
        words = self.last_packet_words
        
        for i in range(0, len(words), words_per_line):
            chunk = words[i:i+words_per_line]
            hex_words = ' '.join(f'{w:08X}' for w in chunk)
            print(f"{i:2d}: {hex_words}")

    def print_statistics(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print(f"\n=== STATISTICS ===")
        print(f"Channel enable: 0x{self.current_channel_enable:X} ({channel_enable_to_string(self.current_channel_enable)})")
        print(f"Expected packet size: {self.expected_packet_size_words} words ({self.expected_packet_size_bytes} bytes)")
        print(f"Total packets received: {self.packet_count}")
        print(f"Total errors: {self.error_count}")
        print(f"  - Timestamp errors: {self.timestamp_errors}")
        print(f"  - Magic number errors: {self.magic_errors}")
        print(f"  - Size errors: {self.size_errors}")
        print(f"Elapsed time: {elapsed:.1f}s")
        print(f"Average rate: {rate:.1f} packets/second")
        if rate > 0:
            print(f"Data rate: {(rate * self.expected_packet_size_bytes * 8 / 1000000):.1f} Mbps")
        print(f"Last timestamp: {self.last_timestamp}")
        
        if self.error_count == 0 and self.packet_count > 0:
            print("All packets validated successfully!")
        elif self.packet_count == 0:
            print("No packets received")
        else:
            error_rate = (self.error_count / self.packet_count) * 100 if self.packet_count > 0 else 0
            print(f"Error rate: {error_rate:.2f}%")

validator = DataValidator()

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening on port {UDP_PORT}...")
    print(f"[UDP] Expected packet size: {validator.expected_packet_size_bytes} bytes ({validator.expected_packet_size_words} words)")
    print(f"[UDP] Magic number: 0x{MAGIC_NUMBER_HIGH:08X}{MAGIC_NUMBER_LOW:08X}")
    print(f"[UDP] Channel enable: 0x{validator.current_channel_enable:X} ({channel_enable_to_string(validator.current_channel_enable)})")

    last_timestamp = None

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                total_len = len(data)

                if total_len % validator.expected_packet_size_bytes != 0:
                    print(f"[WARN] Received {total_len} bytes, not a multiple of {validator.expected_packet_size_bytes}")

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

def send_binary_command(sock, cmd_id, param1=0, param2=0, timeout=2.0):
    """Send a binary command and wait for ACK response"""
    try:
        # Generate unique ACK ID
        ack_id = random.randint(1, 65535)
        
        # Pack command: magic, cmd_id, ack_id, param1, param2 (all 32-bit little-endian)
        command_data = struct.pack('<IIIII', CMD_MAGIC, cmd_id, ack_id, param1, param2)
        
        # Send command
        sock.sendall(command_data)
        print(f"[TCP] Sent binary command 0x{cmd_id:02X} (ACK ID: {ack_id})")
        
        # Set timeout and wait for ACK
        sock.settimeout(timeout)
        ack_response = sock.recv(ACK_PACKET_SIZE)
        
        if len(ack_response) == ACK_PACKET_SIZE:
            recv_ack_id = (ack_response[0] << 8) | ack_response[1]
            status = ack_response[2]
            
            if recv_ack_id == ack_id:
                if status == ACK_SUCCESS:
                    print(f"[TCP] ACK received (ID: {ack_id})")
                    return True
                else:
                    print(f"[TCP] Command failed (ID: {ack_id}, status: 0x{status:02X})")
                    return False
            else:
                print(f"[TCP] ACK ID mismatch: sent {ack_id}, got {recv_ack_id}")
                return False
        else:
            print(f"[TCP] Invalid ACK response length: {len(ack_response)}")
            return False
            
    except socket.timeout:
        print(f"[TCP] Timeout waiting for ACK")
        return False
    except Exception as e:
        print(f"[TCP] Error: {e}")
        return False
    finally:
        sock.settimeout(None)  # Reset to blocking mode

def manual_cable_test(sock):
    """Manual cable test using existing UDP infrastructure"""
    
    print("Manual cable test starting...")

    # Enable all channels for cable testing
    print("Setting all channels enabled for cable test...")
    if not send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, 0x0F):  # All channels
        print("Failed to set channel enable")
        return
    time.sleep(0.1)
    
    # Start manual cable test mode
    validator.start_manual_cable_test()
    
    # collect packets locally to print in order
    collected_packets = []
    try:
        # Step 1: Set loop count to 1
        if not send_binary_command(sock, CMD_SET_LOOP_COUNT, 1):
            print("Failed to set loop count")
            return
        time.sleep(0.1)
        
        # Step 2: Run initialization sequence
        print("Running initialization...")
        if not send_binary_command(sock, CMD_LOAD_INIT):
            print("Failed to set init sequence")
            return
        time.sleep(0.1)
        
        if not send_binary_command(sock, CMD_START):
            print("Failed to start init")
            return
        time.sleep(0.1)
        
        if not send_binary_command(sock, CMD_STOP):
            print("Failed to stop init")
            return
        
        # Wait for init packet
        init_words = validator.wait_for_manual_packet(timeout=5.0)
        if init_words is None:
            print("Timeout waiting for init packet")
            return
        collected_packets.append(init_words)
        print("Collected init packet")
        
        # Step 3: Set cable test sequence
        if not send_binary_command(sock, CMD_LOAD_CABLE_TEST):
            print("Failed to set cable test sequence")
            return
        time.sleep(0.1)
        
        # Step 4: Test each phase
        for phase in range(16):
            print(f"Testing phase {phase}...")
            
            # Set phase
            if not send_binary_command(sock, CMD_SET_PHASE, phase, phase):
                print(f"Failed to set phase {phase}")
                continue
            time.sleep(0.1)
            
            # Run acquisition
            if not send_binary_command(sock, CMD_START):
                print(f"Failed to start phase {phase}")
                continue
            time.sleep(0.1)
            
            if not send_binary_command(sock, CMD_STOP):
                print(f"Failed to stop phase {phase}")
                continue
            
            # Wait for packet
            words = validator.wait_for_manual_packet(timeout=5.0)
            if words is None:
                print(f"Timeout waiting for phase {phase} packet")
                return
            collected_packets.append(words)
            print(f"Collected phase {phase} packet")
        
        # Print results in same format as existing cable test
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
        # Ensure manual test mode is disabled
        global manual_cable_test_mode
        manual_cable_test_mode = False

def tcp_control():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ZYNQ_IP, TCP_PORT))
        print(f"[TCP] Connected to {ZYNQ_IP}:{TCP_PORT}")
        print(f"[TCP] Using binary command protocol")
        print(f"[TCP] Available commands:")
        print(f"  Basic Control:")
        print(f"    start - Begin data streaming")
        print(f"    stop  - Stop data streaming") 
        print(f"    reset_timestamp - Reset timestamp to 0")
        print(f"    loop <count> - Set loop count (0=infinite)")
        print(f"  COPI Command Sequences:")
        print(f"    convert - Set normal data acquisition sequence")
        print(f"    init    - Set chip initialization sequence")
        print(f"    cable_test - Set cable length test sequence")
        print(f"    test_pattern - Set COPI test pattern")
        print(f"    full_cable_test - Run automated cable test")
        print(f"    manual_cable_test - Manual cable test with step-by-step control")
        print(f"  Configuration:")
        print(f"    set_phase <p0> <p1> - Set phase delay for CIPO cables")
        print(f"    set_debug <0|1> - Send dummy data vs real CIPO data")
        print(f"    set_channels <hex> - Set channel enable (0x1=CIPO0_REG, 0x2=CIPO0_DDR, 0x4=CIPO1_REG, 0x8=CIPO1_DDR)")
        print(f"  Status/Debug:")
        print(f"    status - Show PL status")
        print(f"    dump_bram [start] [count] - Show BRAM contents")
        print(f"    stats - Show current statistics")
        print(f"    hex - Show last packet in hex format")
        print(f"  Utility:")
        print(f"    help - Show all commands")
        print(f"    quit - Exit program")
        
        while True:
            cmd = input("\n[TCP] Enter command: ").strip().lower()
            
            if cmd == "quit":
                break
            elif cmd == "start":
                send_binary_command(sock, CMD_START)
            elif cmd == "stop":
                send_binary_command(sock, CMD_STOP)
            elif cmd == "reset_timestamp":
                send_binary_command(sock, CMD_RESET_TIMESTAMP)
                validator.last_timestamp = None
                print("[TCP] Local timestamp tracking reset")
            elif cmd == "convert":
                send_binary_command(sock, CMD_LOAD_CONVERT)
            elif cmd == "init":
                send_binary_command(sock, CMD_LOAD_INIT)
            elif cmd == "cable_test":
                send_binary_command(sock, CMD_LOAD_CABLE_TEST)
            elif cmd == "test_pattern":
                send_binary_command(sock, CMD_LOAD_TEST_PATTERN)
            elif cmd == "full_cable_test":
                # Enable all channels for cable test
                if send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, 0x0F):
                    validator.start_cable_test_capture()
                    send_binary_command(sock, CMD_FULL_CABLE_TEST)
                else:
                    print("Failed to enable all channels for cable test")
            elif cmd == "manual_cable_test":
                manual_cable_test(sock)
            elif cmd == "status":
                send_binary_command(sock, CMD_GET_STATUS)
            elif cmd == "help":
                send_binary_command(sock, CMD_HELP)
            elif cmd.startswith("loop "):
                try:
                    parts = cmd.split()
                    if len(parts) == 2:
                        loop_count = int(parts[1])
                        send_binary_command(sock, CMD_SET_LOOP_COUNT, loop_count)
                    else:
                        print("Usage: loop <count>")
                except ValueError:
                    print("Invalid loop count")
            elif cmd.startswith("set_phase "):
                try:
                    parts = cmd.split()
                    if len(parts) == 3:
                        phase0 = int(parts[1])
                        phase1 = int(parts[2])
                        send_binary_command(sock, CMD_SET_PHASE, phase0, phase1)
                    else:
                        print("Usage: set_phase <phase0> <phase1>")
                except ValueError:
                    print("Invalid phase values")
            elif cmd.startswith("set_debug "):
                try:
                    parts = cmd.split()
                    if len(parts) == 2:
                        debug_mode = int(parts[1])
                        send_binary_command(sock, CMD_SET_DEBUG_MODE, debug_mode)
                    else:
                        print("Usage: set_debug <0|1>")
                except ValueError:
                    print("Invalid debug value")
            elif cmd.startswith("set_channels "):
                try:
                    parts = cmd.split()
                    if len(parts) == 2:
                        # Support both hex (0xF) and decimal (15) input
                        if parts[1].lower().startswith('0x'):
                            channel_enable = int(parts[1], 16)
                        else:
                            channel_enable = int(parts[1])
                        
                        if 0 <= channel_enable <= 15:
                            if send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, channel_enable):
                                validator.set_channel_enable(channel_enable)
                            else:
                                print("Failed to set channel enable on device")
                        else:
                            print("Channel enable must be 0-15 (0x0-0xF)")
                    else:
                        print("Usage: set_channels <0x0-0xF>")
                        print("  0x1 = CIPO0 regular, 0x2 = CIPO0 DDR")
                        print("  0x4 = CIPO1 regular, 0x8 = CIPO1 DDR")
                        print(f"  Current: 0x{validator.current_channel_enable:X} ({channel_enable_to_string(validator.current_channel_enable)})")
                except ValueError:
                    print("Invalid channel enable value")
            elif cmd.startswith("dump_bram"):
                try:
                    parts = cmd.split()
                    start_addr = int(parts[1]) if len(parts) > 1 else 0
                    word_count = int(parts[2]) if len(parts) > 2 else 10
                    send_binary_command(sock, CMD_DUMP_BRAM, start_addr, word_count)
                except (ValueError, IndexError):
                    send_binary_command(sock, CMD_DUMP_BRAM, 0, 10)  # Default values
            elif cmd == "stats":
                validator.print_statistics()             
            elif cmd == "hex" or cmd == "packet_hex":
                validator.print_last_packet_hex()
            else:
                print("[TCP] Invalid command. Available commands:")
                print("  Basic: start, stop, reset_timestamp, status, help, loop <count>")
                print("  COPI: convert, init, cable_test, test_pattern, full_cable_test, manual_cable_test")
                print("  Config: set_phase <p0> <p1>, set_debug <0|1>, set_channels <0x0-0xF>")
                print("  Status/Debug: dump_bram [start] [count], stats, hex")
                print("  Utility: quit")
                
    except ConnectionRefusedError:
        print(f"[TCP] Could not connect to {ZYNQ_IP}:{TCP_PORT}")
        print(f"[TCP] Make sure the Zynq board is running and reachable")
    except KeyboardInterrupt:
        print("\n[TCP] Closing TCP connection")
    finally:
        sock.close()

if __name__ == "__main__":
    print("=== Zynq BRAM Data Generator Validator ===")
    print("This program validates data from your BRAM-based Zynq data generator.")
    print(f"Default: {validator.expected_packet_size_words}-word packets ({validator.expected_packet_size_bytes} bytes)")
    print(f"Magic number: 0x{MAGIC_NUMBER_HIGH:08X}{MAGIC_NUMBER_LOW:08X}")
    print("Using binary TCP command protocol")
    print("Press Ctrl+C to stop.\n")
    
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()
    
    tcp_control()
    
    # Give UDP thread a moment to print final stats
    time.sleep(0.5)
    
