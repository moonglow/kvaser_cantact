#!/usr/bin/env python3
# This program is free software. It comes without any warranty, to
# the extent permitted by applicable law. You can redistribute it
# and/or modify it under the terms of the Do What The Fuck You Want
# To Public License, Version 2, as published by Sam Hocevar. See
# http://www.wtfpl.net/ for more details.
"""Send CMD_RESET_CARD_REQ to the emulated adapter with the given kv_serial
(or on the given network interface). On the STM32F072 builds the firmware reboots straight into the ST
DFU bootloader (no BOOT0 jumper needed); F042 builds simply reboot.

Runs as root on Linux: it unbinds kvaser_usb from the adapter's interface and
writes the 4-byte command on bulk OUT endpoint 0x02 through usbfs. Do not point
it at a real Kvaser.

  reset_card.py <kv_serial>
  reset_card.py --iface canX    for drivers without kv_serial (in-tree kvaser_usb)
  reset_card.py --auto [--exclude S1,S2,...]
      the one kvaser_usb interface whose kv_serial is not excluded
      (exclude every real Kvaser attached to the host)
"""
import ctypes
import fcntl
import os
import sys
import time

CMD_RESET_CARD_REQ = 25
EP_MSGOUT = 0x02

# <linux/usbdevice_fs.h> ioctls (64-bit layouts)
USBDEVFS_CLAIMINTERFACE = 0x8004550F
USBDEVFS_RESET = 0x5514


class BulkTransfer(ctypes.Structure):
    _fields_ = [("ep", ctypes.c_uint), ("len", ctypes.c_uint),
                ("timeout", ctypes.c_uint), ("data", ctypes.c_void_p)]


USBDEVFS_BULK = 0xC0000000 | (ctypes.sizeof(BulkTransfer) << 16) | (ord("U") << 8) | 2


def find_interface(serial):
    for name in os.listdir("/sys/class/net"):
        try:
            with open(f"/sys/class/net/{name}/kv_serial") as f:
                if f.read().strip() == serial:
                    return name, os.path.realpath(f"/sys/class/net/{name}/device")
        except OSError:
            continue
    sys.exit(f"no interface with kv_serial {serial}")


def main():
    args = sys.argv[1:]
    real = set()
    if len(args) == 2 and args[0] == "--iface":
        netdev = args[1]
        intf_path = os.path.realpath(f"/sys/class/net/{netdev}/device")
        if not os.path.exists(f"/sys/class/net/{netdev}/device"):
            sys.exit(f"no interface {netdev}")
        reset(netdev, intf_path, "(not reported)")
        return
    if len(args) == 3 and args[0] == "--auto" and args[1] == "--exclude":
        real = {s for s in args[2].split(",") if s}
        args = args[:1]
    if len(args) != 1:
        sys.exit(__doc__)
    serial = args[0]
    if serial == "--auto":
        candidates = set()
        for name in os.listdir("/sys/class/net"):
            try:
                with open(f"/sys/class/net/{name}/kv_serial") as f:
                    candidates.add(f.read().strip())
            except OSError:
                continue
        candidates -= real
        if len(candidates) != 1:
            sys.exit(f"cannot pick the emulated adapter automatically: {sorted(candidates)}")
        serial = candidates.pop()
    if serial in real:
        sys.exit(f"refusing: {serial} is excluded")

    netdev, intf_path = find_interface(serial)
    reset(netdev, intf_path, serial)


def reset(netdev, intf_path, serial):
    """Unbind kvaser_usb from the interface and send the reset command."""
    intf = os.path.basename(intf_path)          # e.g. 2-12:1.0
    usbdev = os.path.dirname(intf_path)
    with open(f"{usbdev}/busnum") as f:
        bus = int(f.read())
    with open(f"{usbdev}/devnum") as f:
        dev = int(f.read())
    ifnum = int(intf.rsplit(".", 1)[1])
    print(f"{netdev}: kv_serial {serial}, usb {bus:03d}/{dev:03d} interface {intf}")

    with open("/sys/bus/usb/drivers/kvaser_usb/unbind", "w") as f:
        f.write(intf)

    sent = False
    fd = -1
    try:
        fd = os.open(f"/dev/bus/usb/{bus:03d}/{dev:03d}", os.O_RDWR)
        fcntl.ioctl(fd, USBDEVFS_CLAIMINTERFACE, ctypes.c_uint(ifnum))
        cmd = (ctypes.c_uint8 * 4)(4, CMD_RESET_CARD_REQ, 0, 0)
        xfer = BulkTransfer(EP_MSGOUT, 4, 1000, ctypes.cast(cmd, ctypes.c_void_p))
        fcntl.ioctl(fd, USBDEVFS_BULK, xfer)
        sent = True
        print("reset sent")
        # Firmware without the USB detach reboots too fast for the host to
        # notice; a port reset forces re-enumeration either way.
        time.sleep(0.5)
        try:
            fcntl.ioctl(fd, USBDEVFS_RESET)
        except OSError:
            pass  # device already gone: the host saw the disconnect
    finally:
        if fd >= 0:
            os.close(fd)
        if not sent:
            # Give the adapter its driver back rather than stranding it unbound.
            with open("/sys/bus/usb/drivers/kvaser_usb/bind", "w") as f:
                f.write(intf)
            print(f"reset not sent; kvaser_usb rebound to {intf}", file=sys.stderr)


if __name__ == "__main__":
    main()
