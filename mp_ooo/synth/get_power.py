#!/usr/bin/python3

import re

with open("reports/power2.rpt") as f:
    dynamic_power_re = re.compile("Total Dynamic Power += +(\d+\.\d+) +(\w+)")
    leakage_power_re = re.compile("Cell Leakage Power += +(\d+\.\d+) +(\w+)")
    dynamic_power = None
    leakage_power = None
    for l in f.readlines():
        for match in re.finditer(dynamic_power_re, l):
            if match.group(2) == "nW":
                dynamic_power = float(match.group(1))/1000/1000
            elif match.group(2) == "uW":
                dynamic_power = float(match.group(1))/1000
            elif match.group(2) == "mW":
                dynamic_power = float(match.group(1))
            elif match.group(2) == "W":
                dynamic_power = float(match.group(1))*1000
            else:
                raise AssertionError
        for match in re.finditer(leakage_power_re, l):
            if match.group(2) == "nW":
                leakage_power = float(match.group(1))/1000/1000
            elif match.group(2) == "uW":
                leakage_power = float(match.group(1))/1000
            elif match.group(2) == "mW":
                leakage_power = float(match.group(1))
            elif match.group(2) == "W":
                dynamic_power = float(match.group(1))*1000
            else:
                raise AssertionError

    if dynamic_power is None or leakage_power is None:
        raise AssertionError

    print(dynamic_power + leakage_power)
