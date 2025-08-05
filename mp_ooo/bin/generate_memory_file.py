#!/usr/bin/python3

import sys
import os
import shutil
import pathlib
import subprocess
import math

RED    = "31"
YELLOW = "33"
GREEN  = "32"
BLUE   = "36"
def sprint_color(s, color):
    return f"\033[{color}m" + s + "\033[0m"

input_file = []
addressability = []

sys.argv.append('-32');

for v in sys.argv[1:]:
    if v.startswith('-'):
        addressability.append(int(v[1:]))
    else:
        input_file.append(os.path.abspath(v))

if len(input_file) == 0 or len(addressability) == 0:
    print(sprint_color("[ERROR] ", RED) + " Missing Argument.")
    print("[INFO]  Compile a C source files or a RISC-V assembly file, or convert a RISC-V ELF file, into a memory file for simulation.")
    print("[INFO]  Usage: python3 generate_memory_file.py [s/c/elf file] -[addressabilities]")
    print("[INFO]  Example: python3 generate_memory_file.py -4 test.s")
    print("[INFO]  Example: python3 generate_memory_file.py -1 -4 -8 -32 matrix.c helper.c")
    exit(1)

own_path = os.path.abspath(__file__)
script_dir = os.path.dirname(own_path)
work_dir = os.path.join(script_dir, "../sim/bin")
start_file = os.path.join(script_dir, "startup.s")
linker_script = os.path.join(script_dir, "link.ld")
compile = True

assembler="riscv64-unknown-elf-gcc"
objdump="riscv64-unknown-elf-objdump"
objcopy="riscv64-unknown-elf-objcopy"

result = subprocess.run(f"python3 {script_dir}/get_options.py arch", shell=True, stdout=subprocess.PIPE)
arch = result.stdout.decode().split('\n')[0]
result = subprocess.run(f"python3 {script_dir}/get_options.py abi", shell=True, stdout=subprocess.PIPE)
abi = result.stdout.decode().split('\n')[0]

opt = "-Ofast -flto"
warn = "-Wall -Wextra -Wno-unused"
include = "" if len(input_file) == 1 else f"-I {os.path.dirname(os.path.abspath(input_file[0]))}"
assembler_args = f"-mcmodel=medany -ffreestanding -nostartfiles -static -static-libgcc -lm -lgcc -lc -Wl,--no-relax -march={arch} -mabi={abi} {opt} {warn} -T {linker_script} {include}"

out_elf_file = os.path.join(work_dir, pathlib.Path(input_file[0]).stem + ".elf")
out_dis_file = os.path.join(work_dir, pathlib.Path(input_file[0]).stem + ".dis")
temp_bin_file = os.path.join(work_dir, pathlib.Path(input_file[0]).stem + ".bin")
spike_elf_link = os.path.join(work_dir, "spike_dpi.elf")

for f in [os.path.join(work_dir, f"memory_{x}.lst") for x in addressability] + [out_elf_file, out_dis_file, temp_bin_file, spike_elf_link]:
    if os.path.isfile(f):
        os.remove(f)
if not os.path.isdir(work_dir):
    os.system(f"mkdir -p {work_dir}")

if len(input_file) == 1:
    if pathlib.Path(input_file[0]).suffix.lower() in [".s", ".asm"]:
        start_file = ""
    if pathlib.Path(input_file[0]).suffix.lower() == ".elf":
        compile = False
        out_elf_file = input_file[0]

if compile:
    result = subprocess.run(f"{assembler} {assembler_args} {start_file} {' '.join(input_file)} -o {out_elf_file}", shell=True, stdout=subprocess.PIPE)
    if result.returncode != 0 or not os.path.isfile(out_elf_file):
        print(sprint_color("[ERROR] ", RED) + "Error compiling")
        exit(1)
    else:
        print(f"[INFO]  Compiled source to {out_elf_file}")

result = subprocess.run(f"ln -sf {out_elf_file} {spike_elf_link}", shell=True, stdout=subprocess.PIPE)
if result.returncode != 0:
    print(sprint_color("[ERROR] ", RED) + "Unable to create elf link")
    exit(1)

result = subprocess.run(f"{objdump} -D -Mnumeric {out_elf_file} > {out_dis_file}", shell=True, stdout=subprocess.PIPE)
if result.returncode != 0:
    print(sprint_color("[ERROR] ", RED) + "Error disassembling")
    exit(1)
else:
    print(f"[INFO]  Disassembling {os.path.basename(out_elf_file)} to {out_dis_file}")

result = subprocess.run(f"{objdump} -h {out_elf_file}", shell=True, stdout=subprocess.PIPE)
if result.returncode != 0:
    print(sprint_color("[ERROR] ", RED) + "Error objdumping")
    exit(1)
sections = [x.strip().split() for x in result.stdout.decode().splitlines()[5::2]]

for a in addressability:
    fname = os.path.join(work_dir, f"memory_{a}.lst")
    with open(fname, 'w') as f:
        for s in sections:
            section_start = int(s[4], 16)
            section_size = int(s[2], 16)
            section_end = section_start + section_size
            if section_start % a != 0 or section_size % a != 0 or section_end % a != 0:
                print(sprint_color("[ERROR] ", RED) + "Non aligned section not supported")
                exit(1)
            os.system(f"{objcopy} -O binary -j {s[1]} {out_elf_file} {temp_bin_file}")
            if not os.path.isfile(temp_bin_file):
                print(sprint_color("[ERROR] ", RED) + "Error binarizing")
                exit(1)
            with open(temp_bin_file, 'rb') as f2:
                binary = f2.read()
            if len(binary) != 0:
                f.write(f"@{section_start >> int(math.log2(a)):08x}\n")
                temp_string = "".zfill(2*(section_start % a))
                for i in range(len(binary)):
                    temp_string += f"{binary[i]:02x}"
                    if len(temp_string) == 2*a:
                        f.write("".join(reversed([temp_string[i:i+2] for i in range(0, len(temp_string), 2)])) + '\n')
                        temp_string = ""
                if len(temp_string) != 0:
                    f.write("".join(reversed([temp_string[i:i+2] for i in range(0, len(temp_string), 2)])).zfill(2*a) + '\n')
                f.write('\n')
            os.remove(temp_bin_file)
    print(f"[INFO]  Wrote memory contents to {fname}")
