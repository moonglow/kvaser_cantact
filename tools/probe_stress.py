#!/usr/bin/env python3
# This program is free software. It comes without any warranty, to
# the extent permitted by applicable law. You can redistribute it
# and/or modify it under the terms of the Do What The Fuck You Want
# To Public License, Version 2, as published by Sam Hocevar. See
# http://www.wtfpl.net/ for more details.
"""Regression test for the "kvaser_usb error -110: Cannot get software info" wedge.

Unbinds and rebinds kvaser_usb on an Xvaser adapter many times and requires every probe to produce a CAN
interface. With --busy each cycle first brings the interface up and floods it with frames (cangen from
can-utils), so the firmware's response buffer is full and the interface is torn down mid-traffic, the state
that used to leave the adapter answering no further commands.

Runs as root on the Linux host the adapter is plugged into. Only adapters with USB ID 0bfd:0120 and product
"Xvaser" are touched; real Kvasers (0bfd:0117) are never selected.

  probe_stress.py --list
  probe_stress.py <usb path, e.g. 3-4.3.4> [--cycles N] [--busy] [--bitrate 500000]
  probe_stress.py --all [--cycles N] [--busy]

Exit status 0 if every probe of every selected adapter succeeded.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

USB = "/sys/bus/usb/devices"
DRIVER = "/sys/bus/usb/drivers/kvaser_usb"


def read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def xvasers():
    found = []
    for name in sorted(os.listdir(USB)):
        d = f"{USB}/{name}"
        if read(f"{d}/idVendor") == "0bfd" and read(f"{d}/idProduct") == "0120" \
                and (read(f"{d}/product") or "").startswith("Xvaser"):
            found.append((name, read(f"{d}/serial")))
    return found


def netdev(path):
    try:
        names = os.listdir(f"{USB}/{path}:1.0/net")
    except OSError:
        return None
    return names[0] if names else None


def write(path, text):
    with open(path, "w") as f:
        f.write(text)


def probe_once(path, timeout=3.0):
    """Unbind then bind; True when a CAN interface appears within the timeout."""
    intf = f"{path}:1.0"
    try:
        write(f"{DRIVER}/unbind", intf)
    except OSError:
        pass
    time.sleep(0.2)
    try:
        write(f"{DRIVER}/bind", intf)
    except OSError:
        return False
    deadline = time.time() + timeout
    while time.time() < deadline:
        if netdev(path):
            return True
        time.sleep(0.05)
    return False


def load_bus(dev, bitrate, seconds=0.5):
    subprocess.run(["ip", "link", "set", dev, "down"], check=False, capture_output=True)
    subprocess.run(["ip", "link", "set", dev, "type", "can", "bitrate", str(bitrate)], check=False, capture_output=True)
    subprocess.run(["ip", "link", "set", dev, "up"], check=False, capture_output=True)
    gen = subprocess.Popen(["cangen", dev, "-g", "0", "-I", "i", "-L", "8"],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(seconds)
    gen.terminate()
    gen.wait()
    # deliberately no "ip link set down": the driver is unbound with the channel started and frames queued


def stress(path, cycles, busy, bitrate):
    failures = 0
    for n in range(1, cycles + 1):
        if busy:
            dev = netdev(path)
            if dev:
                load_bus(dev, bitrate)
        if not probe_once(path):
            failures += 1
            print(f"  cycle {n}: no interface (probe failed)")
            break
    return failures


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", nargs="?")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--cycles", type=int, default=25)
    ap.add_argument("--busy", action="store_true")
    ap.add_argument("--bitrate", type=int, default=500000)
    a = ap.parse_args()

    adapters = xvasers()
    if a.list:
        for p, s in adapters:
            print(p, s, netdev(p))
        return 0
    if os.geteuid() != 0:
        sys.exit("needs root")
    if a.busy and not shutil.which("cangen"):
        sys.exit("--busy needs cangen (apt install can-utils)")
    if a.all:
        targets = [p for p, _ in adapters]
    elif a.path:
        if a.path not in [p for p, _ in adapters]:
            sys.exit(f"{a.path} is not an Xvaser adapter; see --list")
        targets = [a.path]
    else:
        ap.error("give a USB path or --all")

    bad = 0
    for p in targets:
        f = stress(p, a.cycles, a.busy, a.bitrate)
        print(f"{p}: {'FAIL' if f else 'ok'} ({a.cycles} cycles{', busy' if a.busy else ''})")
        bad += f
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
