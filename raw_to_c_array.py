import sys

if len(sys.argv) != 3:
    print("Usage: python raw_to_c_array.py <input.raw> <output.h>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, "rb") as f:
    data = f.read()

array_name = input_file.split('.')[0] + "_audio_data"

with open(output_file, "w") as f:
    f.write(f"unsigned char {array_name}[] = {{\n")
    for i, byte in enumerate(data):
        if i % 12 == 0:
            f.write("\n  ")
        f.write(f"0x{byte:02x}, ")
    f.write("\n};\n")
    f.write(f"unsigned int {array_name}_len = {len(data)};\n")

print(f"Header file '{output_file}' has been created.")
