import socket
import threading
import struct
import time

ZYNQ_IP = "192.168.1.10"  # IP of the Zynq board
TCP_PORT = 6000  # Must match your board's TCP_PORT
UDP_PORT = 5000  # Must match your board's UDP_PORT

# Data generator constants
MAGIC_NUMBER = 0xDEADBEEFCAFEBABE
DUMMY_DATA_WORD = 0x123456789ABCDEF0
EXPECTED_PACKET_SIZE = 296  # 37 words * 8 bytes = 296 bytes
WORDS_PER_PACKET = 37

class DataValidator:
    def __init__(self):
        self.last_timestamp = None
        self.packet_count = 0
        self.error_count = 0
        self.start_time = None
        self.timestamp_errors = 0
        self.magic_errors = 0
        self.dummy_errors = 0
        self.size_errors = 0
        
    def validate_packet(self, data):
        self.packet_count += 1
        
        if self.start_time is None:
            self.start_time = time.time()
        
        # Check packet size
        if len(data) != EXPECTED_PACKET_SIZE:
            self.size_errors += 1
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Wrong size {len(data)}, expected {EXPECTED_PACKET_SIZE}")
            return False
        
        try:
            # Unpack as little-endian (ARM processors are typically little-endian)
            words = struct.unpack('<37Q', data)  # 37 x 64-bit unsigned integers, little-endian
            
            # Validate magic number (word 0)
            magic = words[0]
            if magic != MAGIC_NUMBER:
                self.magic_errors += 1
                self.error_count += 1
                print(f"[ERROR] Packet {self.packet_count}: Wrong magic number 0x{magic:016X}, expected 0x{MAGIC_NUMBER:016X}")
                return False
            
            # Extract and validate timestamp (word 1)
            timestamp = words[1]
            
            if self.last_timestamp is not None:
                expected_timestamp = self.last_timestamp + 1
                if timestamp != expected_timestamp:
                    self.timestamp_errors += 1
                    self.error_count += 1
                    print(f"[ERROR] Packet {self.packet_count}: Timestamp jump from {self.last_timestamp} to {timestamp}, expected {expected_timestamp}")
                    # Continue processing but note the error
            
            self.last_timestamp = timestamp
            
            # Validate dummy data (words 2-36)
            for i in range(2, WORDS_PER_PACKET):
                if words[i] != DUMMY_DATA_WORD:
                    self.dummy_errors += 1
                    self.error_count += 1
                    print(f"[ERROR] Packet {self.packet_count}: Wrong dummy data at word {i}: 0x{words[i]:016X}, expected 0x{DUMMY_DATA_WORD:016X}")
                    return False
            
            # Packet is valid
            if self.packet_count % 100 == 0:
                elapsed = time.time() - self.start_time
                rate = self.packet_count / elapsed if elapsed > 0 else 0
                print(f"[INFO] Packet {self.packet_count}: Timestamp {timestamp}, Rate: {rate:.1f} pkt/s, Errors: {self.error_count}")
            
            return True
            
        except struct.error as e:
            self.error_count += 1
            print(f"[ERROR] Packet {self.packet_count}: Failed to unpack data: {e}")
            return False
    
    def print_statistics(self):
        elapsed = time.time() - self.start_time if self.start_time else 0
        rate = self.packet_count / elapsed if elapsed > 0 else 0
        
        print(f"\n=== STATISTICS ===")
        print(f"Total packets received: {self.packet_count}")
        print(f"Total errors: {self.error_count}")
        print(f"  - Timestamp errors: {self.timestamp_errors}")
        print(f"  - Magic number errors: {self.magic_errors}")
        print(f"  - Dummy data errors: {self.dummy_errors}")
        print(f"  - Size errors: {self.size_errors}")
        print(f"Elapsed time: {elapsed:.1f}s")
        print(f"Average rate: {rate:.1f} packets/second")
        print(f"Last timestamp: {self.last_timestamp}")
        
        if self.error_count == 0:
            print("✅ All packets validated successfully!")
        else:
            error_rate = (self.error_count / self.packet_count) * 100 if self.packet_count > 0 else 0
            print(f"❌ Error rate: {error_rate:.2f}%")

validator = DataValidator()

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))
    sock.settimeout(1.0)  # Add timeout for clean shutdown
    print(f"[UDP] Listening on port {UDP_PORT}...")
    print(f"[UDP] Expected packet size: {EXPECTED_PACKET_SIZE} bytes")
    print(f"[UDP] Magic number: 0x{MAGIC_NUMBER:016X}")
    print(f"[UDP] Dummy data word: 0x{DUMMY_DATA_WORD:016X}")
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(2048)  # Increased buffer size
                validator.validate_packet(data)
            except socket.timeout:
                continue  # Keep checking for shutdown
            except KeyboardInterrupt:
                break
    except KeyboardInterrupt:
        print("\n[UDP] Stopping UDP listener")
    finally:
        sock.close()
        validator.print_statistics()

def tcp_control():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ZYNQ_IP, TCP_PORT))
        print(f"[TCP] Connected to {ZYNQ_IP}:{TCP_PORT}")
        print(f"[TCP] Available commands:")
        print(f"  start - Begin data streaming")
        print(f"  stop  - Stop data streaming") 
        print(f"  reset_timestamp - Reset timestamp to 0")
        print(f"  status - Show PL status registers")
        print(f"  loop <count> - Set loop count (0=infinite)")
        print(f"  stats - Show current statistics")
        print(f"  quit  - Exit program")
        
        while True:
            cmd = input("\n[TCP] Enter command: ").strip().lower()
            
            if cmd == "quit":
                break
            elif cmd in ("start", "stop", "reset_timestamp", "status"):
                sock.sendall(cmd.encode())
                print(f"[TCP] Sent: {cmd}")
                if cmd == "reset_timestamp":
                    validator.last_timestamp = None  # Reset our tracking too
                    print("[TCP] Local timestamp tracking reset")
            elif cmd.startswith("loop "):
                # Send loop command with count
                sock.sendall(cmd.encode())
                print(f"[TCP] Sent: {cmd}")
            elif cmd == "stats":
                validator.print_statistics()
            elif cmd == "help":
                print(f"[TCP] Available commands: start, stop, reset_timestamp, status, loop <count>, stats, quit")
            else:
                print("[TCP] Invalid command. Type 'help' for available commands.")
                
    except ConnectionRefusedError:
        print(f"[TCP] Could not connect to {ZYNQ_IP}:{TCP_PORT}")
        print(f"[TCP] Make sure the Zynq board is running and reachable")
    except KeyboardInterrupt:
        print("\n[TCP] Closing TCP connection")
    finally:
        sock.close()

if __name__ == "__main__":
    print("=== Zynq Data Generator Validator ===")
    print("This program validates the data structure from your Zynq data generator.")
    print("Press Ctrl+C to stop.\n")
    
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()
    
    tcp_control()
    
    # Give UDP thread a moment to print final stats
    time.sleep(0.5)