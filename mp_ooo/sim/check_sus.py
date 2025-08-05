#!/usr/bin/python3

import subprocess
import os
import sys
import string

allowed_sv_tasks = [
    "signed",
    "unsigned",
    "cast",
    "bits",
    "unpacked_dimensions",
    "dimensions",
    "left",
    "right",
    "low",
    "high",
    "increment",
    "size",
    "clog2",
    "countbits",
    "countones",
    "onehot",
    "onehot0",
    "fatal",
]
sus_keywords = ["DPI", "`", "verilator", "xprop_off", "translate_off", "dc_tcl_script_begin", "dc_script_begin"]
banned_wholewords = ["bit", "int", "byte", "shortint", "longint", "real", "shortreal", "time"]
search_dir = ["hdl", "pkg"]
allowed_char = set(string.ascii_lowercase + string.ascii_uppercase + string.digits + "._-/")

os.chdir(os.path.dirname(os.path.abspath(__file__)))
os.chdir("..")

found = False
log = ""

for d in search_dir:
    result = subprocess.run(f"find {d}", shell=True, stdout=subprocess.PIPE)
    files = result.stdout.decode().split('\n')
    for f in files:
        if not set(f.strip()) <= allowed_char:
            found = True
            log += f + '\n'

allowed_sv_tasks = '|'.join(allowed_sv_tasks)
search_dir = ' '.join(search_dir)

result = subprocess.run(f"grep -Rn -P '\$(?!({allowed_sv_tasks}))' {search_dir}", shell=True, stdout=subprocess.PIPE)
if result.returncode != 1:
    found = True
    log += result.stdout.decode()

for s in sus_keywords:
    result = subprocess.run(f"grep -Rn -P '{s}' {search_dir}", shell=True, stdout=subprocess.PIPE)
    if result.returncode != 1:
        found = True
        log += result.stdout.decode()

for s in banned_wholewords:
    result = subprocess.run(f"grep -Rnw '{s}' {search_dir}", shell=True, stdout=subprocess.PIPE)
    if result.returncode != 1:
        found = True
        log += result.stdout.decode()

if found:
    print("\033[31m" + "Forbidden Keyword Found: " + "\033[0m", file=sys.stderr)
    print(log, file=sys.stderr)
    exit(1)
else:
    exit(0)
