#!/usr/bin/python3

import json
import os
import sys
import string

if len(sys.argv) not in [2, 3]:
    print("Usage:   python3 rvfi_reference.py channels [no_float]")
    print("Example: python3 rvfi_reference.py 1")
    print("Example: python3 rvfi_reference.py 8 no_float")
    exit(1)

channels = int(sys.argv[1])

if len(sys.argv) == 3:
    no_float = True
else:
    no_float = False

rvfi_list = [
    "valid"     ,
    "order"     ,
    "inst"      ,
    "rs1_addr"  ,
    "rs2_addr"  ,
    "rs1_rdata" ,
    "rs2_rdata" ,
    "rd_addr"   ,
    "rd_wdata"  ,
    "pc_rdata"  ,
    "pc_wdata"  ,
    "mem_addr"  ,
    "mem_rmask" ,
    "mem_wmask" ,
    "mem_rdata" ,
    "mem_wdata"
]

if not no_float:
    rvfi_list += [
        "frs1_addr" ,
        "frs2_addr" ,
        "frs3_addr" ,
        "frs1_rdata",
        "frs2_rdata",
        "frs3_rdata",
        "frd_addr"  ,
        "frd_wdata"
    ]

required_list = []

for i in range(channels):
    required_list += [x + f"[{i}]" for x in rvfi_list]

allowed_char = set(string.ascii_lowercase + string.ascii_uppercase + string.digits + "._'[]")

os.chdir(os.path.dirname(os.path.abspath(__file__)))
os.chdir("../hvl/common")

if os.path.isfile("rvfi_reference.svh"):
    os.remove("rvfi_reference.svh")

with open("rvfi_reference.json") as f:
    j = json.load(f)

if not all([x in j for x in required_list]):
    print("incomplete list in rvfi_reference.json", file=sys.stderr)
    exit(1)

if not all([x in required_list for x in j]):
    print("spurious item in rvfi_reference.json", file=sys.stderr)
    exit(1)

if not all([set(j[x]) <= allowed_char for x in j]):
    print("illegal character in rvfi_reference.json", file=sys.stderr)
    exit(1)

with open("rvfi_reference.svh", 'w') as f:
    f.write("always_comb begin\n")
    for x in j:
        f.write(f"    mon_itf.{x} = {j[x]};\n")
    f.write("end\n")
