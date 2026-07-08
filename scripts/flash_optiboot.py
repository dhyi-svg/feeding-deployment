#!/usr/bin/env python3
"""
flash_optiboot.py -- minimal STK500v1 flasher for an Arduino Uno (Optiboot).

Exists because the NUC (which owns the base Arduino) has no avrdude and no
sudo: this needs only pyserial. Safe by construction: the Uno's bootloader
lives in a fuse-protected flash section that this protocol cannot touch, so a
failed or interrupted flash is ALWAYS recoverable -- reset and run this again.

usage:
    python3 flash_optiboot.py [--port PORT] firmware.hex
    python3 flash_optiboot.py --verify-only firmware.hex   # compare, no write

After a successful flash the sketch is started and the first serial lines
(e.g. the "Ready v7 enc" banner) are echoed as an end-to-end check.
"""

import argparse
import sys
import time

import serial

DEFAULT_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00"

STK_INSYNC = 0x14
STK_OK = 0x10
CRC_EOP = 0x20
PAGE = 128  # ATmega328P flash page, bytes
SIGNATURE_M328P = bytes([0x1E, 0x95, 0x0F])


def parse_ihex(path):
    """Intel HEX -> contiguous bytes image (0xFF-padded)."""
    mem = {}
    ext = 0
    with open(path) as f:
        for ln, raw in enumerate(f, 1):
            s = raw.strip()
            if not s:
                continue
            if not s.startswith(":"):
                raise ValueError(f"{path}:{ln}: missing ':'")
            b = bytes.fromhex(s[1:])
            if (sum(b) & 0xFF) != 0:
                raise ValueError(f"{path}:{ln}: bad checksum")
            count, addr, rtype = b[0], (b[1] << 8) | b[2], b[3]
            data = b[4:4 + count]
            if rtype == 0x00:
                for i, d in enumerate(data):
                    mem[ext + addr + i] = d
            elif rtype == 0x01:
                break
            elif rtype == 0x02:
                ext = (((data[0] << 8) | data[1]) << 4)
            elif rtype == 0x04:
                ext = (((data[0] << 8) | data[1]) << 16)
            else:
                raise ValueError(f"{path}:{ln}: unsupported record type {rtype}")
    if not mem:
        raise ValueError(f"{path}: no data records")
    img = bytearray([0xFF] * (max(mem) + 1))
    for a, d in mem.items():
        img[a] = d
    return bytes(img)


class Optiboot:
    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baud, timeout=0.5)

    def close(self):
        self.ser.close()

    def _reset(self):
        """Pulse DTR: the Uno's auto-reset cap resets the MCU on the edge,
        dropping us into the ~1 s Optiboot window."""
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.dtr = True
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def _cmd(self, payload: bytes, resp_len: int = 0) -> bytes:
        self.ser.write(payload + bytes([CRC_EOP]))
        r = self.ser.read(resp_len + 2)
        if len(r) != resp_len + 2 or r[0] != STK_INSYNC or r[-1] != STK_OK:
            raise IOError(f"cmd 0x{payload[0]:02x}: bad response {r.hex() or '(timeout)'}")
        return r[1:-1]

    def sync(self, resets: int = 5):
        """Reset into the bootloader and get sync (the running sketch chats on
        the same port, so flush + retry within each reset window)."""
        for _ in range(resets):
            self._reset()
            for _ in range(4):
                self.ser.reset_input_buffer()
                try:
                    self._cmd(bytes([0x30]))  # STK_GET_SYNC
                    return
                except IOError:
                    time.sleep(0.05)
        raise IOError("could not sync with Optiboot (is something else on the port?)")

    def check_signature(self):
        sig = self._cmd(bytes([0x75]), resp_len=3)  # STK_READ_SIGN
        if sig != SIGNATURE_M328P:
            raise IOError(f"unexpected device signature {sig.hex()} (want ATmega328P)")

    def _load_address(self, byte_addr: int):
        wa = byte_addr // 2  # word-addressed, little-endian
        self._cmd(bytes([0x55, wa & 0xFF, (wa >> 8) & 0xFF]))

    def program(self, img: bytes):
        npages = (len(img) + PAGE - 1) // PAGE
        for i in range(npages):
            base = i * PAGE
            page = img[base:base + PAGE].ljust(PAGE, b"\xFF")
            self._load_address(base)
            # STK_PROG_PAGE: length is BIG-endian, memtype 'F' = flash.
            self._cmd(bytes([0x64, 0x00, PAGE, ord("F")]) + page)
            print(f"\r  writing {base + len(page):5d}/{len(img)} bytes "
                  f"({i + 1}/{npages} pages)", end="", flush=True)
        print()

    def verify(self, img: bytes) -> bool:
        npages = (len(img) + PAGE - 1) // PAGE
        for i in range(npages):
            base = i * PAGE
            want = img[base:base + PAGE].ljust(PAGE, b"\xFF")
            self._load_address(base)
            got = self._cmd(bytes([0x74, 0x00, PAGE, ord("F")]), resp_len=PAGE)
            if got != want:
                off = next(j for j in range(PAGE) if got[j] != want[j])
                print(f"\nVERIFY MISMATCH at 0x{base + off:04x}: "
                      f"want {want[off]:02x} got {got[off]:02x}")
                return False
            print(f"\r  verifying page {i + 1}/{npages}", end="", flush=True)
        print()
        return True

    def start_app(self):
        self._cmd(bytes([0x51]))  # STK_LEAVE_PROGMODE -> jumps to the sketch


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("hex", help="Intel HEX file to flash")
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--verify-only", action="store_true",
                   help="compare flash against the hex, write nothing")
    args = p.parse_args()

    img = parse_ihex(args.hex)
    print(f"{args.hex}: {len(img)} bytes "
          f"({(len(img) + PAGE - 1) // PAGE} pages of {PAGE})")

    ob = Optiboot(args.port, args.baud)
    try:
        print("Resetting into Optiboot...")
        ob.sync()
        ob.check_signature()
        print("Synced; ATmega328P signature OK.")
        if not args.verify_only:
            ob.program(img)
        if not ob.verify(img):
            print("FAILED: verify mismatch -- flash again (bootloader is intact).")
            return 1
        print("Verify OK. Starting sketch...")
        ob.start_app()
        # End-to-end check: echo the sketch's first lines (banner).
        t_end = time.time() + 4.0
        while time.time() < t_end:
            line = ob.ser.readline()
            if line.endswith(b"\n"):
                text = line.decode("utf-8", errors="replace").strip()
                if text:
                    print(f"  sketch: {text}")
                    if text.startswith("Ready"):
                        break
        return 0
    finally:
        ob.close()


if __name__ == "__main__":
    sys.exit(main())
