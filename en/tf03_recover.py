"""
tf03_recover.py - Blind recovery attempt for a TF03 that outputs data but ignores commands.

Hypothesis: the sensor's RECEIVE side may be listening at a different baud than the one it
transmits at. We cannot read replies at an unknown baud, so we send "blind":
for each candidate baud, send  [set baud 115200] + [save]  (and optionally factory reset).
Then POWER CYCLE the sensor and run tf03_baud_scan.py to see if it now transmits at 115200.

Worst case: nothing changes. Commands are only accepted if the sensor understands them.

Usage (close BW_TFDS first):
  python tf03_recover.py --port COM8                 # set-baud 115200 + save at every baud
  python tf03_recover.py --port COM8 --factory       # also send factory reset at every baud
"""
import argparse
import struct
import time

import serial  # pip install pyserial

BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 512000, 921600]


def make_cmd(cmd_id, payload=b""):
    body = bytes([0x5A, 4 + len(payload), cmd_id]) + payload
    return body + bytes([sum(body) & 0xFF])


CMD_BAUD_115200 = make_cmd(0x06, struct.pack("<I", 115200))   # 5A 08 06 00 C2 01 00 2B
CMD_SAVE = make_cmd(0x11)                                      # 5A 04 11 6F
CMD_FACTORY = make_cmd(0x10)                                   # 5A 04 10 6E


def send_slow(ser, cmd, gap_s=0.002):
    for b in cmd:
        ser.write(bytes([b]))
        ser.flush()
        time.sleep(gap_s)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--factory", action="store_true", help="also send factory reset (5A 04 10 6E)")
    a = ap.parse_args()
    for b in BAUDS:
        with serial.Serial(a.port, b, timeout=0.05) as s:
            seq = [CMD_BAUD_115200, CMD_SAVE] + ([CMD_FACTORY, CMD_SAVE] if a.factory else [])
            for _ in range(2):                      # twice, normal and slow
                for c in seq:
                    s.write(c); s.flush(); time.sleep(0.15)
                for c in seq:
                    send_slow(s, c); time.sleep(0.15)
        print(f"  sent at {b:>6} baud: " + " | ".join(c.hex(' ').upper() for c in seq))
    print("\n-> Now POWER CYCLE the TF03 (unplug 5 s), then run:  python tf03_baud_scan.py --port COM8")


if __name__ == "__main__":
    main()
