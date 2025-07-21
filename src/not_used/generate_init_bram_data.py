# generate_coe.py

NUM_WORDS = 65536
RADIX = 16
FILENAME = "init_bram.coe"

def generate_data(num_words):
    return [i for i in range(num_words)]

def write_coe(filename, data, radix=16):
    with open(filename, "w") as f:
        f.write(f"memory_initialization_radix={radix};\n")
        f.write("memory_initialization_vector=\n")
        
        for i, word in enumerate(data):
            # Format the word as a 32-bit hexadecimal string
            formatted = f"{word:08X}"
            # Append comma unless it's the last word
            if i < len(data) - 1:
                f.write(f"{formatted},\n")
            else:
                f.write(f"{formatted};\n")

if __name__ == "__main__":
    data = generate_data(NUM_WORDS)
    write_coe(FILENAME, data, RADIX)
    print(f"COE file written to {FILENAME}")
