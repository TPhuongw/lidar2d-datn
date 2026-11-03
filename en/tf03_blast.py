"""
tf03_blast.py - Send a TF03 command many times (for a sensor that accepts commands only intermittently),
and measure the reply rate.

Usage:
  python tf03_blast.py --port COM12 --baud 460800 rate              # send version 100x, count replies
  python tf03_blast.py --port COM12 --baud 460800 send "5A 06 03 E8 03 4E"        # 1000 Hz, 100x
  python tf03_blast.py --port COM12 --baud 460800 send "5A 08 06 00 C2 01 00 2B"  # baud 115200, 100x
  python tf03_blast.py --port COM12 --baud 460800 send "5A 04 11 6F"              # save, 100x
Options: -n (repeats, default 100), --gap (seconds between sends, default 0.03)
"""
import argparse
import time

import serial  # pip install pyserial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("-n", type=int, default=100)
    ap.add_argument("--gap", type=float, default=0.03)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("rate")
    sp = sub.add_parser("send"); sp.add_argument("hex")
    a = ap.parse_args()

    with serial.Serial(a.port, a.baud, timeout=0) as s:
        if a.cmd == "rate":
            cmd, ok = bytes.fromhex("5A 04 01 5F"), 0
            for _ in range(a.n):
                s.reset_input_buffer()
                s.write(cmd)
                t_end, buf = time.time() + 0.08, bytearray()
                while time.time() < t_end:
                    buf += s.read(4096)
                ok += b"\x5a\x07\x01" in buf
            print(f"version replies: {ok} / {a.n} at {a.baud} baud")
        else:
            cmd = bytes.fromhex(a.hex)
            echoes = 0
            for _ in range(a.n):
                s.write(cmd)
                time.sleep(a.gap)
                d = s.read(65536)
                echoes += any(d[i] == 0x5A and d[i + 2] == cmd[2] and 4 <= d[i + 1] <= 9
                              for i in range(len(d) - 2))
            print(f"sent {cmd.hex(' ').upper()} x{a.n} at {a.baud} baud; reply-like frames seen: {echoes}")


if __name__ == "__main__":
    main()
