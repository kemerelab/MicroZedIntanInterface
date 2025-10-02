import socket
import threading
import struct
import time
import random
import queue
import ipaddress

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
CMD_LOAD_CONVERT = 0x20
CMD_LOAD_INIT = 0x21
CMD_LOAD_CABLE_TEST = 0x22
CMD_FULL_CABLE_TEST = 0x30
CMD_GET_STATUS = 0x40
CMD_DUMP_BRAM = 0x41
CMD_SET_UDP_DEST = 0x50

# ACK status codes
ACK_SUCCESS = 0x06
ACK_ERROR = 0x15

# Cable test globals
cable_test_mode = False
cable_test_packets_captured = 0
manual_cable_test_mode = False

def calculate_data_words(channel_enable):
    """Calculate number of 32-bit data words based on channel enable setting"""
    num_channels = bin(channel_enable & 0x0F).count('1')
    if num_channels == 0:
        return 70
    total_16bit_words = 35 * num_channels
    return (total_16bit_words + 1) // 2

def calculate_packet_size(channel_enable):
    """Calculate total packet size in words (header + data)"""
    return 4 + calculate_data_words(channel_enable)

def channel_enable_to_string(channel_enable):
    """Convert channel enable bits to human readable string"""
    channels = []
    if channel_enable & 0x01: channels.append("CIPO0_REG")
    if channel_enable & 0x02: channels.append("CIPO0_DDR")
    if channel_enable & 0x04: channels.append("CIPO1_REG")
    if channel_enable & 0x08: channels.append("CIPO1_DDR")
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
        
    def validate_packet(self, data):
        global cable_test_mode, cable_test_packets_captured, manual_cable_test_mode
        
        self.packet_count += 1
        self.last_packet_raw = data

        if cable_test_mode:
            if cable_test_packets_captured < 17:
                expected_size = calculate_packet_size(0x0F) * 4
                if len(data) == expected_size:
                    words = struct.unpack(f'<{calculate_packet_size(0x0F)}I', data)
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
            expected_size = calculate_packet_size(0x0F) * 4
            if len(data) == expected_size:
                words = struct.unpack(f'<{calculate_packet_size(0x0F)}I', data)
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

            magic_combined = (words[1] << 32) | words[0]
            expected_magic = (MAGIC_NUMBER_HIGH << 32) | MAGIC_NUMBER_LOW

            if magic_combined != expected_magic:
                self.magic_errors += 1
                self.error_count += 1
                print(f"[ERROR] Packet {self.packet_count}: Magic number mismatch")
                return None            
            
            timestamp = (words[3] << 32) | words[2]

            now = time.time()
            if self.packet_count % 30000 == 0 or (now - self.last_stats_time) >= 5.0:
                elapsed = now - self.start_time
                total_rate = self.packet_count / elapsed if elapsed > 0 else 0
                inst_rate = (self.packet_count - self.last_packet_count) / (now - self.last_stats_time) if (now - self.last_stats_time) > 0 else 0
                
                if len(words) >= 8:
                    data_sample = f"Data: [0x{words[4]:08X}, 0x{words[5]:08X}, 0x{words[6]:08X}, 0x{words[7]:08X}]"
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

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
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

def send_binary_command(sock, cmd_id, param1=0, param2=0, timeout=2.0):
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
        print(f"[TCP] Timeout waiting for response")
        return (False, None)
    except Exception as e:
        print(f"[TCP] Error: {e}")
        return (False, None)
    finally:
        sock.settimeout(None)

def get_status(sock):
    """Get full status from device"""
    success, data = send_binary_command(sock, CMD_GET_STATUS)
    
    if not success or data is None:
        print("[TCP] Failed to get status")
        return None
    
    if len(data) != 86:
        print(f"[TCP] Invalid status response length: {len(data)}")
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
        'udp_bytes_sent': udp_bytes_sent
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

def manual_cable_test(sock):
    """Manual cable test using existing UDP infrastructure"""
    print("Manual cable test starting...")

    if not send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, 0x0F)[0]:
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

def tcp_control():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ZYNQ_IP, TCP_PORT))
        print(f"[TCP] Connected to {ZYNQ_IP}:{TCP_PORT}")
        
        # Auto-configure UDP destination
        local_ip = get_local_ip()
        print(f"[TCP] Detected local IP: {local_ip}")
        print(f"[TCP] Configuring device to send UDP to this machine...")
        
        if set_udp_dest(sock, local_ip, UDP_PORT):
            print(f"[TCP] ✓ Device configured to send UDP packets here")
        else:
            print(f"[TCP] ✗ Failed to configure UDP destination")
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
        print(f"  Config: set_phase <p0> <p1>, set_debug <0|1>, set_channels <0x0-0xF>")
        print(f"  Network: set_udp <ip> <port>, get_status")
        print(f"  Debug: dump_bram [start] [count], stats, hex")
        print(f"  Utility: help, quit")
        
        while True:
            cmd = input("\n[TCP] Command: ").strip().lower()
            
            if cmd == "quit":
                break
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
                if send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, 0x0F)[0]:
                    validator.start_cable_test_capture()
                    send_binary_command(sock, CMD_FULL_CABLE_TEST)
            elif cmd == "manual_cable_test":
                manual_cable_test(sock)
            elif cmd == "get_status":
                status = get_status(sock)
                if status:
                    print_status(status)
            elif cmd.startswith("loop "):
                try:
                    loop_count = int(cmd.split()[1])
                    send_binary_command(sock, CMD_SET_LOOP_COUNT, loop_count)
                except (ValueError, IndexError):
                    print("Usage: loop <count>")
            elif cmd.startswith("set_phase "):
                try:
                    parts = cmd.split()
                    if len(parts) == 3:
                        send_binary_command(sock, CMD_SET_PHASE, int(parts[1]), int(parts[2]))
                    else:
                        print("Usage: set_phase <phase0> <phase1>")
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
                    if 0 <= channel_enable <= 15:
                        if send_binary_command(sock, CMD_SET_CHANNEL_ENABLE, channel_enable)[0]:
                            validator.set_channel_enable(channel_enable)
                    else:
                        print("Channel enable must be 0-15")
                except (ValueError, IndexError):
                    print("Usage: set_channels <0x0-0xF>")
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
            elif cmd == "help":
                print("Commands:")
                print("  start, stop, reset_timestamp")
                print("  loop <count>, set_phase <p0> <p1>")
                print("  set_debug <0|1>, set_channels <0x0-0xF>")
                print("  convert, init, cable_test")
                print("  full_cable_test, manual_cable_test")
                print("  set_udp <ip> <port>, get_status")
                print("  dump_bram [start] [count]")
                print("  stats, hex, quit")
            else:
                print(f"Unknown command: '{cmd}'. Type 'help' for list.")
                
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
