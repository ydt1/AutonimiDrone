#!/usr/bin/python3
import re
import time
import numpy

NUM_TESTS = 10

print("Measure wifi signal levels in 5 seconds")
time.sleep(5)

def read_signal(builtInWiFi):
    with open("/proc/net/wireless") as fp:
        lines = fp.readlines()
        for line in lines:
            if builtInWiFi:
                match_obj = re.search(r'wlan0:\s*[^\s]*\s*[^\s]*\s*([^\s]*)', line)
            else:
                match_obj = re.search(r'wlan1:\s*[^\s]*\s*[^\s]*\s*([^\s]*)', line)
            if match_obj:
                return(float(match_obj[1]))
    raise Exception("Unable to read /proc/net/wireless")


def measureStats(builtInWiFi):
    results = []

    for i in range(NUM_TESTS):
        results.append(read_signal(builtInWiFi))
        time.sleep(1)

    print(f"{len(results)} measurements")
    print(f"Min:\t{min(results)}")
    print(f"Max:\t{max(results)}")
    print(f"Mean:\t{numpy.mean(results)}")
    print(f"StdDev:\t{numpy.std(results)}")
    print("Measurement done")

print("****** Builtin WIFI ******")
measureStats(True)
print("****** Dongle WIFI ******")
measureStats(False)
