#!/usr/bin/env python3
"""Capture whatever is typed/pasted into the terminal and save it on Ctrl+C.

Usage:
    ./capture.py [output_file]

Type or paste text, then press Ctrl+C to write everything to the file
(default: capture.txt in the current directory). Ctrl+D also saves and exits.
"""
import sys
import termios
import tty

OUT = sys.argv[1] if len(sys.argv) > 1 else "capture.txt"


def main():
    fd = sys.stdin.fileno()
    buf = []

    if not sys.stdin.isatty():
        # Piped/redirected input: no terminal, just read it all.
        buf.append(sys.stdin.buffer.read())
    else:
        capture_tty(fd, buf)

    text = b"".join(buf).replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    with open(OUT, "wb") as f:
        f.write(text)
    print(f"\nSaved {len(text)} bytes to {OUT}")


def capture_tty(fd, buf):
    old = termios.tcgetattr(fd)
    try:
        # Raw mode so we receive Ctrl+C (0x03) as data instead of a signal,
        # and so a final line without Enter is still captured. TCSADRAIN (not
        # the setraw default TCSAFLUSH) so any type-ahead/paste that arrived a
        # hair early isn't discarded.
        tty.setraw(fd, termios.TCSADRAIN)
        while True:
            ch = sys.stdin.buffer.read(1)
            if not ch or ch == b"\x03":  # EOF / Ctrl+C -> stop and save
                break
            buf.append(ch)
            if ch == b"\r":  # echo paste/typing so you can see it
                sys.stdout.write("\r\n")
            else:
                sys.stdout.write(ch.decode("utf-8", "replace"))
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


if __name__ == "__main__":
    main()
