#!/usr/bin/python3

import json
import os
import pathlib
import string

allowed_char = set(string.ascii_lowercase + string.ascii_uppercase + string.digits + "._")

own_path = (os.path.dirname(os.path.abspath(__file__)))
os.chdir(own_path)

flag = False

for c in os.listdir(os.path.join(own_path, "config")):
    if not c.endswith(".json"):
        continue
    if not set(c) <= allowed_char:
        print("illegal character in config filename")
        exit(1)
    name = pathlib.Path(c).stem
    with open(os.path.join(own_path, "config", c)) as f:
        j = json.load(f)
    opensram_config = f'''
tech_name = "freepdk45"
num_rw_ports = {int(j["num_rw_ports"])}
num_r_ports = 0
num_w_ports = 0
word_size = {int(j["word_size"])}
write_size = {"None" if j["word_size"] == j["write_size"] else int(j["write_size"])}
num_words = {int(j["num_words"])}
sim_random_init = {bool(j["random_init"])}
nominal_corner_only = True
process_corners = ["TT"]
supply_voltages = [1.0]
temperatures = [25]
netlist_only = False
route_supplies = False
check_lvsdrc = False
perimeter_pins = False
load_scales = [0.5, 1, 4]
slew_scales = [0.5, 1]
output_name = "{name}"
output_path = "output/{name}"
print_banner = False
num_threads = 4
output_extended_config = True
'''
    target_file = os.path.join(own_path, "output", name, name+".py")
    os.system("mkdir -p " + os.path.dirname(target_file))
    with open(target_file, 'w') as f:
        f.write(opensram_config)
    flag = True

if not flag:
    print("Error: no sram config found")
    exit(1)
