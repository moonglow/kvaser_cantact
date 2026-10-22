#!/usr/bin/env python3
# This program is free software. It comes without any warranty, to
# the extent permitted by applicable law. You can redistribute it
# and/or modify it under the terms of the Do What The Fuck You Want
# To Public License, Version 2, as published by Sam Hocevar. See
# http://www.wtfpl.net/ for more details.
"""Validate the UCAN running this firmware against a real Kvaser on a two-node bus.

Runs as root on Linux (interface configuration needs CAP_NET_ADMIN), with the
emulated adapter (DUT) and a real Kvaser (peer) on an isolated two-node bus.
Interfaces are found by card serial (`kv_serial` in sysfs) or given by name.
Every case uses an exact loss oracle: each frame carries a sequence number,
captures start before any transmission, and a case passes only if the received
frames equal the sent ones in order with zero netdev drop/overrun deltas.

  validate_bench.py --peer-serial N [--dut-serial N] [--protect-serial N ...]
                    [--soak-seconds 300] [--cases abcdghstef]
  validate_bench.py --dut-iface canX --peer-iface canY ...
  validate_bench.py --self-test     check that the oracle itself can fail

Without --dut-serial the DUT is the one kvaser_usb interface that is neither
the peer nor a --protect-serial adapter (e.g. ones carrying other live buses).
"""
import argparse
import errno
import json
import os
import socket
import struct
import subprocess
import sys
import threading
import time

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
FRAME = struct.Struct("=IB3x8s")

# Card serials (kv_serial) that must never be touched (adapters on other live buses);
# filled from --protect-serial.
PROTECTED_SERIALS = set()


def kvaser_ifaces():
    """{interface: kv_serial} for every kvaser_usb interface."""
    found = {}
    for name in sorted(os.listdir("/sys/class/net")):
        try:
            with open(f"/sys/class/net/{name}/kv_serial") as f:
                found[name] = f.read().strip()
        except OSError:
            continue
    return found


def find_iface(serial):
    for name, kv_serial in kvaser_ifaces().items():
        if kv_serial == serial:
            return name
    sys.exit(f"no interface with kv_serial {serial}")


def find_dut_serial(peer_serial):
    candidates = sorted({s for s in kvaser_ifaces().values()
                         if s != peer_serial and s not in PROTECTED_SERIALS})
    if len(candidates) != 1:
        sys.exit(f"cannot pick the UCAN automatically, candidates {candidates}; pass --dut-serial")
    return candidates[0]


def ip(*args):
    subprocess.run(["ip", *args], check=True)


def configure(iface, bitrate):
    ip("link", "set", "dev", iface, "down")
    ip("link", "set", "dev", iface, "type", "can", "bitrate", str(bitrate), "restart-ms", "100")
    ip("link", "set", "dev", iface, "txqueuelen", "1000")
    ip("link", "set", "dev", iface, "up")


def counters(iface):
    """Netdev drop/overrun counters plus CAN error-state counters."""
    out = subprocess.run(["ip", "-j", "-d", "-s", "link", "show", "dev", iface],
                         check=True, capture_output=True, text=True).stdout
    link = json.loads(out)[0]
    rx = link["stats64"]["rx"]
    x = link.get("linkinfo", {}).get("info_xstats", {})
    return {
        "rx_dropped": rx.get("dropped", 0),
        "rx_over_errors": rx.get("over_errors", 0),
        "bus_error": x.get("bus_error", 0),
        "error_passive": x.get("error_passive", 0),
        "bus_off": x.get("bus_off", 0),
    }


def frame_bits(can_id, dlc, data):
    """Exact on-wire length of a classic CAN frame in bits, including stuff
    bits, the 15-bit CRC and the 3-bit interframe space."""
    ext = bool(can_id & CAN_EFF_FLAG)
    rtr = bool(can_id & CAN_RTR_FLAG)
    ident = can_id & (0x1FFFFFFF if ext else 0x7FF)
    bits = [0]                                               # SOF
    if ext:
        bits += [(ident >> (28 - i)) & 1 for i in range(11)]  # base ID
        bits += [1, 1]                                        # SRR, IDE
        bits += [(ident >> (17 - i)) & 1 for i in range(18)]  # extended ID
        bits += [int(rtr), 0, 0]                              # RTR, r1, r0
    else:
        bits += [(ident >> (10 - i)) & 1 for i in range(11)]
        bits += [int(rtr), 0, 0]                              # RTR, IDE, r0
    bits += [(dlc >> (3 - i)) & 1 for i in range(4)]
    if not rtr:
        for byte in data[:min(dlc, 8)]:
            bits += [(byte >> (7 - i)) & 1 for i in range(8)]
    crc = 0
    for b in bits:
        crc = ((crc << 1) ^ (0x4599 if ((crc >> 14) & 1) ^ b else 0)) & 0x7FFF
    bits += [(crc >> (14 - i)) & 1 for i in range(15)]
    stuffed, run, last = 0, 0, None
    for b in bits:                                            # SOF..CRC are stuffed
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            stuffed += 1
            last, run = 1 - b, 1                              # the stuff bit starts a new run
    return len(bits) + stuffed + 1 + 2 + 7 + 3               # CRC delim, ACK, EOF, IFS


def open_sock(iface, loopback=True):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    if not loopback:
        # Keep this socket's frames out of other sockets on the same interface,
        # or a bidirectional run would capture its own transmissions.
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_LOOPBACK, 0)
    s.bind((iface,))
    s.settimeout(0.2)
    return s


def send(sock, frame):
    can_id, dlc, data = frame
    raw = FRAME.pack(can_id, dlc, data.ljust(8, b"\0"))
    while True:
        try:
            sock.send(raw)
            return
        except OSError as e:
            if e.errno != errno.ENOBUFS:
                raise
            time.sleep(0.0005)


class Capture(threading.Thread):
    def __init__(self, iface):
        super().__init__(daemon=True)
        self.sock = open_sock(iface)
        self.frames = []
        self.times = []
        self.last_rx = time.monotonic()
        self.stop = threading.Event()
        self.ready = threading.Event()

    def run(self):
        self.ready.set()
        while not self.stop.is_set():
            try:
                raw = self.sock.recv(16)
            except socket.timeout:
                continue
            can_id, dlc, data = FRAME.unpack(raw)
            self.frames.append((can_id, dlc, b"" if can_id & CAN_RTR_FLAG else data[:dlc]))
            self.last_rx = time.monotonic()
            self.times.append(self.last_rx)

    def drain(self, quiet=0.5, limit=10.0):
        end = time.monotonic() + limit
        while time.monotonic() < end and time.monotonic() - self.last_rx < quiet:
            time.sleep(0.05)
        self.stop.set()
        self.join()
        self.sock.close()


def oracle(sent, received):
    """Exact comparison: same frames, same order, nothing missing or extra."""
    if sent == received:
        return True, f"{len(sent)}/{len(sent)} frames, exact match"
    sent_set = set(sent)
    missing = len(sent_set - set(received))
    first = next((i for i, (a, b) in enumerate(zip(sent, received)) if a != b),
                 min(len(sent), len(received)))
    return False, (f"sent {len(sent)} received {len(received)} "
                   f"missing {missing}, first divergence at index {first}")


def seq_frames(n, start=0, ext_every=0, node=0):
    """Sequence-numbered 8-byte frames. Each node gets its own ID range so two
    nodes transmitting at once never put identical IDs on the bus."""
    frames = []
    for i in range(start, start + n):
        if ext_every and i % ext_every == 0:
            can_id = (0x18DA0000 + (node << 16)) | (i & 0xFFFF) | CAN_EFF_FLAG
        else:
            can_id = 0x100 + node * 0x300 + (i % 0x300)
        frames.append((can_id, 8, struct.pack("<Q", i)))
    return frames


class Runner:
    def __init__(self, dut, peer):
        self.dut, self.peer = dut, peer
        self.results = []

    def transfer(self, tx_iface, rx_iface, frames, rate=None):
        cap = Capture(rx_iface)
        cap.start()
        cap.ready.wait()
        before = counters(rx_iface)
        tx = open_sock(tx_iface, loopback=False)
        t0 = time.monotonic()
        for i, frame in enumerate(frames):
            if rate:
                delay = t0 + i / rate - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            send(tx, frame)
        elapsed = time.monotonic() - t0
        tx.close()
        cap.drain()
        after = counters(rx_iface)
        ok, detail = oracle(frames, cap.frames)
        drops = {k: after[k] - before[k] for k in ("rx_dropped", "rx_over_errors")}
        if any(drops.values()):
            ok, detail = False, f"{detail}; netdev deltas {drops}"
        fps = len(frames) / elapsed if elapsed else 0
        return ok, f"{detail}; {fps:.0f} frames/s"

    def record(self, name, ok, detail):
        self.results.append((name, ok))
        print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}", flush=True)

    def both_ways(self, name, frames, rate=None):
        for tx, rx, label in ((self.dut, self.peer, "UCAN->Leaf"), (self.peer, self.dut, "Leaf->UCAN")):
            self.record(f"{name} {label}", *self.transfer(tx, rx, frames, rate))

    def case_frame_types(self):
        configure(self.dut, 500000); configure(self.peer, 500000)
        frames = [
            (0x123, 8, bytes(range(8))),
            (0x7FF, 8, b"\xff" * 8),
            (0x000, 0, b""),
            (0x1ABCDEF0 | CAN_EFF_FLAG, 8, bytes(range(8, 16))),
            (0x18DAF110 | CAN_EFF_FLAG, 0, b""),
            (0x456, 3, b"\x01\x02\x03"),
        ]
        self.both_ways("(a) std/ext, DLC 0/3/8 @500k", frames, rate=200)

    def case_rtr(self):
        frames = [
            (0x321 | CAN_RTR_FLAG, 0, b""),
            (0x321 | CAN_RTR_FLAG, 8, b""),
            (0x1F000001 | CAN_EFF_FLAG | CAN_RTR_FLAG, 4, b""),
        ]
        self.both_ways("(b) RTR std/ext @500k", frames, rate=200)

    def case_bitrates(self):
        for bitrate in (125000, 250000, 500000, 1000000):
            configure(self.dut, bitrate); configure(self.peer, bitrate)
            self.both_ways(f"(c) 100 frames @{bitrate // 1000}k", seq_frames(100), rate=500)

    def bidirectional(self, name, n, ext_every, rate=None):
        """Both nodes transmit at once (disjoint IDs); each direction is judged
        by the oracle, plus no new bus errors on either node."""
        before = {i: counters(i) for i in (self.dut, self.peer)}
        out = {}

        def leg(tx, rx, label, node):
            out[label] = self.transfer(tx, rx, seq_frames(n, ext_every=ext_every, node=node), rate=rate)

        legs = [threading.Thread(target=leg, args=(self.dut, self.peer, "UCAN->Leaf", 0)),
                threading.Thread(target=leg, args=(self.peer, self.dut, "Leaf->UCAN", 1))]
        for t in legs: t.start()
        for t in legs: t.join()
        for label, (ok, detail) in out.items():
            self.record(f"{name} {label}", ok, detail)
        for iface in (self.dut, self.peer):
            after = counters(iface)
            errs = {k: after[k] - before[iface][k] for k in ("bus_error", "error_passive", "bus_off")}
            self.record(f"{name} error counters {iface}", not any(errs.values()), str(errs))

    def case_burst(self):
        for bitrate in (500000, 1000000):
            configure(self.dut, bitrate); configure(self.peer, bitrate)
            self.both_ways(f"(d) 1000-frame back-to-back burst @{bitrate // 1000}k",
                           seq_frames(1000, ext_every=3))

    def case_bidir_burst(self):
        for bitrate in (500000, 1000000):
            configure(self.dut, bitrate); configure(self.peer, bitrate)
            self.bidirectional(f"(g) 2000-frame bidirectional full-load burst @{bitrate // 1000}k",
                               2000, ext_every=3)

    def sweep_rate(self, tx_dut=False):
        """Informational: highest paced rate per direction that loses nothing."""
        tx, rx, label = (self.dut, self.peer, "UCAN->Leaf") if tx_dut else (self.peer, self.dut, "Leaf->UCAN")
        for bitrate, rates in ((500000, (3500, 3000, 2500, 2000, 1000)),
                               (1000000, (7000, 6000, 5000, 4000, 3000, 2000))):
            configure(self.dut, bitrate); configure(self.peer, bitrate)
            best = None
            for rate in rates:
                ok, detail = self.transfer(tx, rx, seq_frames(3000, ext_every=3), rate=rate)
                print(f"INFO  @{bitrate // 1000}k {label} paced {rate} frames/s: "
                      f"{'lossless' if ok else 'LOSS'}; {detail}", flush=True)
                if ok and best is None:
                    best = rate
            print(f"INFO  @{bitrate // 1000}k highest lossless {label} rate tested: {best} frames/s",
                  flush=True)

    def sweep_rx_rate(self):
        self.sweep_rate(tx_dut=False)

    def sweep_tx_rate(self):
        self.sweep_rate(tx_dut=True)

    def case_canlib_rx(self, sender, channel, n=20000, min_load=0.90):
        """UCAN receive at full bus load. The real Kvaser is driven through
        Kvaser CANlib (`tools/canlib_sender.c`) instead of SocketCAN, whose
        one-frame-per-transfer path caps a Leaf at about 3,000 frames/s. The
        peer is not a SocketCAN interface here, so only the UCAN side's
        counters are checked."""
        can_id = 0x124
        expected = [(can_id, 8, struct.pack("<Q", i)) for i in range(n)]
        for bitrate in (500000, 1000000):
            configure(self.dut, bitrate)
            cap = Capture(self.dut)
            cap.start()
            cap.ready.wait()
            before = counters(self.dut)
            out = subprocess.run([sender, str(channel), str(bitrate), str(n), f"{can_id:x}"],
                                 capture_output=True, text=True)
            cap.drain(quiet=1.0)
            after = counters(self.dut)
            ok, detail = oracle(expected, cap.frames)
            deltas = {k: after[k] - before[k] for k in after if after[k] != before[k]}
            bits = sum(frame_bits(*f) for f in cap.frames)
            load = bits / ((cap.times[-1] - cap.times[0]) * bitrate) if len(cap.times) > 1 else 0.0
            # Not load-gated: a Leaf Light HS v2 alone peaks at about 7,140 frames/s
            # (one per 140 us), 87.6% of 1 Mbit/s; the bidirectional run below
            # is what saturates the bus.
            ok = ok and out.returncode == 0 and not deltas
            self.record(f"(i) {n}-frame CANlib Leaf->UCAN @{bitrate // 1000}k", ok,
                        f"{detail}; bus load {load:.1%} (not gated: Leaf TX ceiling)"
                        f"{'; counter deltas ' + str(deltas) if deltas else ''}"
                        f"{'; ' + out.stderr.strip() if out.returncode else ''}")

        # UCAN transmitting at its maximum into a Leaf that only receives, so a
        # loss here cannot be the Leaf's own TX+RX budget (8,000 messages/s).
        ucan_id = 0x123
        for bitrate in (500000, 1000000):
            configure(self.dut, bitrate)
            leaf = subprocess.Popen([sender, str(channel), str(bitrate), "0", f"{can_id:x}",
                                     f"{ucan_id:x}", str(n)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True)
            time.sleep(0.3)
            t0 = time.monotonic()
            ucan = subprocess.run(["cangen", self.dut, "-g", "0", "-p", "10", "-I", f"{ucan_id:x}",
                                   "-L", "8", "-D", "i", "-n", str(n), "-x"])
            leaf_out, leaf_err = leaf.communicate(timeout=120)
            res = dict(kv.split("=") for kv in leaf_out.split())
            span = float(res.get("rx_last", 0)) - float(res.get("rx_first", 0))
            load = int(res.get("rx", 0)) * frame_bits(ucan_id, 8, struct.pack("<Q", 0)) / (span * bitrate) \
                if span > 0 else 0.0
            ok = leaf.returncode == 0 and ucan.returncode == 0
            self.record(f"(i) {n}-frame UCAN->CANlib Leaf (receive-only) @{bitrate // 1000}k", ok,
                        f"rx {res.get('rx')}/{n} missing {res.get('rx_missing')} disorder {res.get('rx_disorder')} "
                        f"leaf-overruns {res.get('rx_overruns')}; bus load {load:.1%} (not gated: one TX mailbox)"
                        f"{'; ' + leaf_err.strip() if leaf_err.strip() else ''}")

        # Both at once: UCAN (cangen) and the CANlib Leaf each send at their
        # maximum, so together they hold the bus at 100% while the UCAN must
        # receive the Leaf's stream. The Leaf checks the UCAN's stream itself.
        for bitrate in (500000, 1000000):
            configure(self.dut, bitrate)
            cap = Capture(self.dut)
            cap.start()
            cap.ready.wait()
            before = counters(self.dut)
            leaf = subprocess.Popen([sender, str(channel), str(bitrate), str(n), f"{can_id:x}",
                                     f"{ucan_id:x}", str(n)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True)
            time.sleep(0.3)  # let the Leaf go bus-on before the UCAN starts
            ucan = subprocess.run(["cangen", self.dut, "-g", "0", "-p", "10", "-I", f"{ucan_id:x}",
                                   "-L", "8", "-D", "i", "-n", str(n), "-x"])
            leaf_out, leaf_err = leaf.communicate(timeout=120)
            cap.drain(quiet=1.0)
            after = counters(self.dut)
            res = dict(kv.split("=") for kv in leaf_out.split())
            ok_rx, detail = oracle(expected, cap.frames)
            # The UCAN's stream into the Leaf is verified by the receive-only run
            # above. Here the Leaf is past its rated 8,000 messages/s (sending
            # about 7,100/s while receiving), so its own receive side may overrun;
            # that is reported, not gated. The gate is the UCAN's receive.
            ok_tx = ucan.returncode == 0 and "rx" in res
            deltas = {k: after[k] - before[k] for k in after if after[k] != before[k]}
            # busy span: from the first frame either side saw to the last
            starts = [cap.times[0], float(res.get("rx_first", 0)) or cap.times[0]]
            ends = [cap.times[-1], float(res.get("rx_last", 0)) or cap.times[-1]]
            bits = (sum(frame_bits(*f) for f in cap.frames)
                    + int(res.get("rx", 0)) * frame_bits(ucan_id, 8, struct.pack("<Q", 0)))
            load = bits / ((max(ends) - min(starts)) * bitrate)
            ok = ok_rx and ok_tx and not deltas and load >= min_load
            self.record(f"(i) {n}+{n}-frame bidirectional CANlib Leaf + UCAN @{bitrate // 1000}k "
                        f"(gate: UCAN receive at >= {min_load:.0%} load)", ok,
                        f"Leaf->UCAN {detail}; [info] UCAN->Leaf rx {res.get('rx')}/{n} missing {res.get('rx_missing')} "
                        f"disorder {res.get('rx_disorder')} leaf-overruns {res.get('rx_overruns')} "
                        f"errframes {res.get('rx_errframes')}; "
                        f"bus load {load:.1%}{'; counter deltas ' + str(deltas) if deltas else ''}"
                        f"{'; ' + leaf_err.strip() if leaf_err.strip() else ''}")

    def case_saturation(self, n=10000):
        """cangen (C, no gap) over SocketCAN on both adapters: the heaviest
        load the standard Linux driver can produce. Every frame is checked by
        sequence; the bus load is measured from the exact bit length of each
        received frame and reported. It is not gated here because a SocketCAN
        Leaf sends at most about 3,000 frames/s; case (i) carries the gate."""
        ids = {self.dut: 0x123, self.peer: 0x124}

        def expected(can_id):
            return [(can_id, 8, struct.pack("<Q", i)) for i in range(n)]

        for bitrate in (500000, 1000000):
            for senders in ((self.dut,), (self.peer,), (self.dut, self.peer)):
                configure(self.dut, bitrate); configure(self.peer, bitrate)
                name = (f"(h) {n}-frame cangen saturation @{bitrate // 1000}k "
                        + " + ".join("UCAN->Leaf" if s == self.dut else "Leaf->UCAN" for s in senders))
                caps = {}
                before = {}
                for tx in senders:
                    rx = self.peer if tx == self.dut else self.dut
                    caps[tx] = Capture(rx)
                    caps[tx].start()
                    caps[tx].ready.wait()
                    before[rx] = counters(rx)
                errs_before = {i: counters(i) for i in (self.dut, self.peer)}
                # -p: poll when the driver queue is full (cangen's documented full-load mode)
                procs = [subprocess.Popen(["cangen", tx, "-g", "0", "-p", "10", "-I", f"{ids[tx]:x}",
                                           "-L", "8", "-D", "i", "-n", str(n), "-x"]) for tx in senders]
                for p in procs:
                    p.wait()
                for cap in caps.values():
                    cap.drain(quiet=1.0)

                ok_all, bits, t0, t1 = True, 0, None, None
                for tx, cap in caps.items():
                    rx = self.peer if tx == self.dut else self.dut
                    ok, detail = oracle(expected(ids[tx]), cap.frames)
                    after = counters(rx)
                    drops = {k: after[k] - before[rx][k] for k in ("rx_dropped", "rx_over_errors")}
                    if any(drops.values()):
                        ok, detail = False, f"{detail}; netdev deltas {drops}"
                    ok_all &= ok
                    print(f"      {'UCAN->Leaf' if tx == self.dut else 'Leaf->UCAN'}: {detail}", flush=True)
                    bits += sum(frame_bits(*f) for f in cap.frames)
                    if cap.times:
                        t0 = cap.times[0] if t0 is None else min(t0, cap.times[0])
                        t1 = cap.times[-1] if t1 is None else max(t1, cap.times[-1])
                errs = {}
                for iface in (self.dut, self.peer):
                    after = counters(iface)
                    errs.update({f"{iface}.{k}": after[k] - errs_before[iface][k]
                                 for k in ("bus_error", "error_passive", "bus_off") if after[k] != errs_before[iface][k]})
                load = bits / ((t1 - t0) * bitrate) if t0 is not None and t1 > t0 else 0.0
                ok_all &= not errs
                # Load is reported, not gated: with SocketCAN the Leaf sends at
                # most about 3,000 frames/s. Case (i) drives the Leaf through
                # Kvaser CANlib and carries the full-load gate.
                self.record(name, ok_all,
                            f"bus load {load:.1%}"
                            f"{'; errors ' + str(errs) if errs else ''}")

    def case_soak(self, seconds):
        configure(self.dut, 500000); configure(self.peer, 500000)
        self.bidirectional(f"(e) {seconds}s bidirectional soak @500k", 500 * seconds, ext_every=2, rate=500)

    def case_negative(self):
        """Mismatched bitrates must lose every frame; proves the oracle can fail."""
        configure(self.dut, 500000); configure(self.peer, 250000)
        ok, detail = self.transfer(self.dut, self.peer, seq_frames(20), rate=100)
        self.record("(f) control: 500k vs 250k must lose frames", not ok, detail)
        # The unacknowledged frames above must not survive the interface going
        # down and come out ahead of the next transfer.
        configure(self.dut, 500000); configure(self.peer, 500000)
        self.record("(f) no stale TX after the control", *self.transfer(
            self.dut, self.peer, seq_frames(6, start=1000), rate=200))

    def restore(self):
        for iface in (self.dut, self.peer):
            if iface:
                ip("link", "set", "dev", iface, "down")


def self_test():
    frames = seq_frames(50)
    dropped = frames[:20] + frames[21:]
    checks = [oracle(frames, frames)[0] is True,
              oracle(frames, dropped)[0] is False,
              oracle(frames, frames[:10] + frames[11:12] + frames[10:11] + frames[12:])[0] is False,
              oracle(frames, frames + frames[-1:])[0] is False]
    # Frame length: an 8-byte standard frame is 111 bits unstuffed and at most
    # 135 with worst-case stuffing; long zero runs must cost stuff bits.
    zeros = frame_bits(0x000, 8, bytes(8))
    alternating = frame_bits(0x555, 8, b"\x55" * 8)
    checks += [111 <= zeros <= 135, 111 <= alternating <= 135, zeros > alternating,
               frame_bits(0x18DA0000 | CAN_EFF_FLAG, 8, bytes(8)) > zeros]
    # A soak-sized mismatch must still be judged quickly (no quadratic path).
    big = seq_frames(150000)
    t0 = time.monotonic()
    checks.append(oracle(big, big[:-1] + big[:1])[0] is False)
    checks.append(time.monotonic() - t0 < 5)
    print("self-test:", "PASS" if all(checks) else f"FAIL {checks}")
    return 0 if all(checks) else 1


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dut-serial", default=None)
    p.add_argument("--peer-serial", help="kv_serial of the real Kvaser")
    p.add_argument("--protect-serial", action="append", default=[],
                   help="kv_serial of an adapter that must never be used (repeatable)")
    p.add_argument("--soak-seconds", type=int, default=300)
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--dut-iface", help="use this interface for the UCAN instead of finding it by "
                                       "kv_serial (in-tree kvaser_usb has no kv_serial attribute)")
    p.add_argument("--peer-iface", help="use this interface for the real Kvaser")
    p.add_argument("--canlib-sender", help="path to the built tools/canlib_sender.c, for case i")
    p.add_argument("--canlib-channel", type=int, default=0, help="CANlib channel of the real Kvaser")
    p.add_argument("--cases", default="abcdghstef",
                   help="subset to run: a b c d g(bidir burst) h(cangen saturation) "
                        "i(CANlib full load) s/t(rx/tx rate sweep) e(soak) f(control)")
    args = p.parse_args()
    if args.self_test:
        return self_test()
    PROTECTED_SERIALS.update(args.protect_serial)

    if "i" in args.cases and not args.canlib_sender:
        sys.exit("case i needs --canlib-sender (build tools/canlib_sender.c against linuxcan)")
    if args.dut_iface and not args.peer_iface and set(args.cases) <= {"i"}:
        # CANlib-only run: the real Kvaser is owned by CANlib, not SocketCAN
        dut, peer = args.dut_iface, None
        print(f"DUT {dut}, peer via CANlib channel {args.canlib_channel}", flush=True)
    elif args.dut_iface or args.peer_iface:
        if not (args.dut_iface and args.peer_iface):
            sys.exit("--dut-iface and --peer-iface go together")
        dut, peer = args.dut_iface, args.peer_iface
        print(f"DUT {dut}, peer {peer} (given explicitly)", flush=True)
    else:
        if not args.peer_serial:
            sys.exit("give --peer-serial (or --dut-iface/--peer-iface)")
        if args.dut_serial is None:
            args.dut_serial = find_dut_serial(args.peer_serial)
        for serial in (args.dut_serial, args.peer_serial):
            if serial in PROTECTED_SERIALS:
                sys.exit(f"refusing: serial {serial} is protected (--protect-serial)")
        dut, peer = find_iface(args.dut_serial), find_iface(args.peer_serial)
        print(f"DUT {dut} (kv_serial {args.dut_serial}), peer {peer} (kv_serial {args.peer_serial})", flush=True)

    r = Runner(dut, peer)
    steps = {
        "a": r.case_frame_types,
        "b": r.case_rtr,
        "c": r.case_bitrates,
        "d": r.case_burst,
        "g": r.case_bidir_burst,
        "h": r.case_saturation,
        "i": lambda: r.case_canlib_rx(args.canlib_sender, args.canlib_channel),
        "s": r.sweep_rx_rate,
        "t": r.sweep_tx_rate,
        "e": lambda: r.case_soak(args.soak_seconds),
        "f": r.case_negative,
    }
    try:
        for key in "abcdghistef":
            if key in args.cases:
                steps[key]()
    finally:
        r.restore()
    failed = [name for name, ok in r.results if not ok]
    print(f"\n{len(r.results) - len(failed)}/{len(r.results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
