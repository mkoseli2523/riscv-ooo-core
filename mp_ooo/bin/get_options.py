#!/usr/bin/python3

import json
import sys
import os
import string
import subprocess

os.chdir(os.path.dirname(os.path.abspath(__file__)))
os.chdir("..")

with open("options.json") as f:
    j = json.load(f)

if sys.argv[1] == "no_float":
    if j["f_ext"]:
        retval = ""
    else:
        retval = "+define+ECE411_NO_FLOAT"
    print(retval)

if sys.argv[1] == "arch":
    retval = "rv32im"
    if j["f_ext"]:
        retval += 'f'
    if j["c_ext"]:
        retval += 'c'
    print(retval)

if sys.argv[1] == "abi":
    retval = "ilp32"
    if j["f_ext"]:
        retval += 'f'
    print(retval)

if sys.argv[1] == "clock":
    if j["clock"] % 2 != 0 or j["clock"] < 0:
        print("Error: clock period must be a even positive number")
        exit(1)
    print(int(j["clock"]))

if sys.argv[1] == "bmem_x":
    print(int(j["bmem_0_on_x"]))

if sys.argv[1] == "dw_ip":
    allowed_char = set(string.ascii_lowercase + string.ascii_uppercase + string.digits + "._")
    if not all([set(x) <= allowed_char for x in j["dw_ip"]]):
        print("illegal character in options.json dw_ip", file=sys.stderr)
        exit(1)
    for i in j["dw_ip"]:
        result = subprocess.run(f"grep -nw {i} sim/vcs_warn.config", shell=True, stdout=subprocess.PIPE)
        if result.returncode == 1:
            with open("sim/vcs_warn.config", "a") as f:
                f.write("{\n    +lint=none;\n    +module=" + i + ";\n}\n")
        result = subprocess.run(f"grep -nw {i} sim/xprop.config", shell=True, stdout=subprocess.PIPE)
        if result.returncode == 1:
            with open("sim/xprop.config", "a") as f:
                f.write("module {" + i + "} {xpropOff};\n")
    print(' '.join([os.environ["DW"] + '/sim_ver/' + x + '.v' for x in j['dw_ip']]))

if sys.argv[1] == "min_power":
    print(int(j["synth"]["min_power"]))

if sys.argv[1] == "synth_inc_iter":
    iter = int(j["synth"]["inc_iter"])
    if iter > 10 or iter < 0:
        print("Error: Synthesis incremental iterations need to be within 0 and 10", file=sys.stderr)
        exit(1)
    print(int(j["synth"]["inc_iter"]))

if sys.argv[1] == "synth_cmd" or sys.argv[1] == "synth_cmd_inc":
    cmd = ""

    if j["synth"]["compile_ultra"]:
        cmd += "compile_ultra"
        if sys.argv[1] == "synth_cmd_inc":
            cmd += " -incremental"
    else:
        cmd += "compile"
        if sys.argv[1] == "synth_cmd_inc":
            cmd += " -incremental_mapping"

    if j["synth"]["compile_ultra"]:
        if not j["synth"]["ungroup"]:
            cmd += " -no_autoungroup"
    else:
        if j["synth"]["ungroup"]:
            cmd += " -ungroup_all"

    if j["synth"]["gate_clock"]:
        cmd += " -gate_clock"

    if j["synth"]["retime"]:
        if j["synth"]["compile_ultra"]:
            cmd += " -retime"
        else:
            print("Error: compile non ultra does not support retime", file=sys.stderr)
            exit(1)

    print(cmd)
