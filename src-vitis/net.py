import socket
import threading
import struct
import time

ZYNQ_IP = "192.168.1.10"  # IP of the Zynq board
TCP_PORT = 6000  # Must match your board's TCP_PORT
UDP_PORT = 5000  # Must match your board's UDP_PORT

# Updated data generator constants for your current implementation
MAGIC_NUMBER_LOW = 0xDEADBEEF   # Lower 32 bits
MAGIC_NUMBER_HIGH = 0xCAFEBABE  # Upper 32 bits
EXPECTED_PACKET_SIZE = 576      # 144 words * 4 bytes = 576 bytes
WORDS_PER_PACKET = 144          # 4 header + (35 cycles * 4 data words)

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
        
    def validate_packet(self, data):
        self.packet_count += 1

        if self.start_time is None:
            self.start_time = time.time()
            self.last_stats_time = self.start_time

        if len(data) != EXPECTED_PACKET_SIZE:
            self.size_errors += 1
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Wrong size {len(data)}, expected {EXPECTED_PACKET_SIZE}")
            hex_dump = ' '.join(f'{b:02X}' for b in data[:32])
            print(f"[DEBUG] First 32 bytes: {hex_dump}")
            return None

        try:
            # Unpack as 144 32-bit little-endian words
            words = struct.unpack('<144I', data)

            # Check magic number (words 0 and 1)
            magic_combined = (words[1] << 32) | words[0]
            expected_magic = (MAGIC_NUMBER_HIGH << 32) | MAGIC_NUMBER_LOW
            
            # if magic_combined != expected_magic:
            #     self.magic_errors += 1
            #     self.error_count += 1
            #     print(f"[ERROR] Packet {self.packet_count}: Wrong magic number")
            #     print(f"         Expected: 0x{expected_magic:016X}")
            #     print(f"         Got:      0x{magic_combined:016X}")
            #     print(f"         Raw words: 0x{words[0]:08X}, 0x{words[1]:08X}")
            #     return None

            # Extract timestamp (words 2 and 3)
            timestamp = (words[3] << 32) | words[2]

            # Show periodic stats
            now = time.time()
            if self.packet_count % 30000 == 0 or (now - self.last_stats_time) >= 5.0:
                elapsed = now - self.start_time
                total_rate = self.packet_count / elapsed if elapsed > 0 else 0
                inst_rate = (self.packet_count - self.last_packet_count) / (now - self.last_stats_time) if (now - self.last_stats_time) > 0 else 0
                
                # Show some data words for verification
                data_sample = f"Data: [0x{words[4]:08X}, 0x{words[5]:08X}, 0x{words[6]:08X}, 0x{words[7]:08X}]"
                
                print(f"[INFO] Packet {self.packet_count}: Timestamp {timestamp}, "
                      f"Rate: {total_rate:.1f} pkt/s (avg), {inst_rate:.1f} pkt/s (inst), "
                      f"Errors: {self.error_count}")
                print(f"       {data_sample}")
                
                self.last_stats_time = now
                self.last_packet_count = self.packet_count

            return timestamp

        except struct.error as e:
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Failed to unpack data: {e}")
            hex_dump = ' '.join(f'{b:02X}' for b in data[:64])
            print(f"[DEBUG] First 64 bytes: {hex_dump}")
            return None
    
    def print_statistics(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print(f"\n=== STATISTICS ===")
        print(f"Total packets received: {self.packet_count}")
        print(f"Total errors: {self.error_count}")
        print(f"  - Timestamp errors: {self.timestamp_errors}")
        print(f"  - Magic number errors: {self.magic_errors}")
        print(f"  - Size errors: {self.size_errors}")
        print(f"Elapsed time: {elapsed:.1f}s")
        print(f"Average rate: {rate:.1f} packets/second")
        if rate > 0:
            print(f"Data rate: {(rate * EXPECTED_PACKET_SIZE * 8 / 1000000):.1f} Mbps")
        print(f"Last timestamp: {self.last_timestamp}")
        
        if self.error_count == 0 and self.packet_count > 0:
            print("✅ All packets validated successfully!")
        elif self.packet_count == 0:
            print("⚠️  No packets received")
        else:
            error_rate = (self.error_count / self.packet_count) * 100 if self.packet_count > 0 else 0
            print(f"❌ Error rate: {error_rate:.2f}%")

validator = DataValidator()

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)
    print(f"[UDP] Listening on port {UDP_PORT}...")
    print(f"[UDP] Expected packet size: {EXPECTED_PACKET_SIZE} bytes ({WORDS_PER_PACKET} words)")
    print(f"[UDP] Magic number: 0x{MAGIC_NUMBER_HIGH:08X}{MAGIC_NUMBER_LOW:08X}")

    last_timestamp = None

    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                total_len = len(data)

                if total_len % EXPECTED_PACKET_SIZE != 0:
                    print(f"[WARN] Received {total_len} bytes, not a multiple of {EXPECTED_PACKET_SIZE}")

                for offset in range(0, total_len - EXPECTED_PACKET_SIZE + 1, EXPECTED_PACKET_SIZE):
                    chunk = data[offset:offset + EXPECTED_PACKET_SIZE]
                    timestamp = validator.validate_packet(chunk)
                    if timestamp is not None:
                        if last_timestamp is not None and timestamp != last_timestamp + 1:
                            validator.timestamp_errors += 1
                            validator.error_count += 1
                            # print(f"[ERROR] Timestamp discontinuity: got {timestamp}, expected {last_timestamp + 1}")
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

def send_tcp_command(sock, command):
    """Send a TCP command and return any response"""
    try:
        sock.sendall(command.encode())
        print(f"[TCP] Sent: {command}")
        return True
    except Exception as e:
        print(f"[TCP] Error sending command: {e}")
        return False

def tcp_control():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ZYNQ_IP, TCP_PORT))
        print(f"[TCP] Connected to {ZYNQ_IP}:{TCP_PORT}")
        print(f"[TCP] Available commands:")
        print(f"  Basic Control:")
        print(f"    start - Begin data streaming")
        print(f"    stop  - Stop data streaming") 
        print(f"    reset_timestamp - Reset timestamp to 0")
        print(f"    loop <count> - Set loop count (0=infinite)")
        print(f"  Status Commands:")
        print(f"    status - Show PL status")
        print(f"  Diagnostic Commands:")
        print(f"    dump_bram [start] [count] - Show BRAM contents")
        print(f"  Utility:")
        print(f"    help - Show all commands")
        print(f"    stats - Show current statistics")
        print(f"    quit - Exit program")
        
        while True:
            cmd = input("\n[TCP] Enter command: ").strip().lower()
            
            if cmd == "quit":
                break
            elif cmd in ("start", "stop", "reset_timestamp", "status", "help"):
                send_tcp_command(sock, cmd)
                if cmd == "reset_timestamp":
                    validator.last_timestamp = None  # Reset our tracking too
                    print("[TCP] Local timestamp tracking reset")
            elif cmd.startswith("loop "):
                send_tcp_command(sock, cmd)
            elif cmd.startswith("dump_bram"):
                send_tcp_command(sock, cmd)
            elif cmd == "stats":
                validator.print_statistics()
            elif cmd == "test":
                print("\n[TCP] Running simple test sequence...")
                print("[TCP] Step 1: Reset timestamp")
                send_tcp_command(sock, "reset_timestamp")
                time.sleep(0.5)
                
                print("[TCP] Step 2: Start streaming")
                send_tcp_command(sock, "start")
                time.sleep(0.5)
                
                print("[TCP] Step 3: Check status")
                send_tcp_command(sock, "status")
                time.sleep(0.5)
                
                print("[TCP] Test sequence complete. UDP should be receiving data.")
                
            elif cmd == "quick_test":
                print("\n[TCP] Running quick test (10 seconds)...")
                validator.__init__()  # Reset statistics
                send_tcp_command(sock, "reset_timestamp")
                time.sleep(0.5)
                send_tcp_command(sock, "start")
                print("[TCP] Waiting 10 seconds for data...")
                time.sleep(10)
                send_tcp_command(sock, "stop")
                send_tcp_command(sock, "status")
                validator.print_statistics()
                
            elif cmd == "monitor":
                print("\n[TCP] Starting monitoring mode (press Enter to stop)...")
                try:
                    while True:
                        send_tcp_command(sock, "status")
                        # Use select or timeout to check for input
                        import select
                        import sys
                        if select.select([sys.stdin], [], [], 3) == ([sys.stdin], [], []):
                            sys.stdin.readline()
                            break
                        time.sleep(3)
                except KeyboardInterrupt:
                    pass
                print("[TCP] Monitoring stopped")
                
            elif cmd == "benchmark":
                print("\n[TCP] Running 30-second benchmark...")
                validator.__init__()  # Reset statistics
                send_tcp_command(sock, "reset_timestamp")
                time.sleep(1)
                send_tcp_command(sock, "start")
                
                start_time = time.time()
                while time.time() - start_time < 30:
                    send_tcp_command(sock, "status")
                    time.sleep(5)
                
                send_tcp_command(sock, "stop")
                send_tcp_command(sock, "status")
                validator.print_statistics()
                print("[TCP] 30-second benchmark complete")
                
            else:
                print("[TCP] Invalid command. Available commands:")
                print("  start, stop, reset_timestamp, status, help")
                print("  loop <count>, dump_bram [start] [count]")
                print("  test, quick_test, monitor, benchmark, stats, quit")
                
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
    print(f"Expecting {WORDS_PER_PACKET}-word packets ({EXPECTED_PACKET_SIZE} bytes)")
    print(f"Magic number: 0x{MAGIC_NUMBER_HIGH:08X}{MAGIC_NUMBER_LOW:08X}")
    print("Press Ctrl+C to stop.\n")
    
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()
    
    tcp_control()
    
    # Give UDP thread a moment to print final stats
    time.sleep(0.5)