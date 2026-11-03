"""
tf03_baud_scan.py - Do toc do baud thuc te cua TF03: thu lan luot cac baud, dem so khung hop le.

Chay:  python tf03_baud_scan.py --port COM8
Baud nao co nhieu khung hop le (checksum dung) -> do la baud TF03 dang dung.
"""
import argparse
import time

import serial  # pip install pyserial

from tf03_live import FrameParser

BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 256000, 460800, 512000, 921600]


def try_baud(port, baud, seconds=1.0):
    with serial.Serial(port, baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        t0 = time.time()
        while time.time() - t0 < 0.3:          # bo du lieu cu
            ser.read(65536)
        p, n_bytes, frames = FrameParser(), 0, []
        t0 = time.time()
        while time.time() - t0 < seconds:
            b = ser.read(4096)
            n_bytes += len(b)
            frames += p.feed(b)
    return n_bytes / seconds, len(frames) / seconds, frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    a = ap.parse_args()
    print(f"{'baud':>7} | {'byte/s':>7} | {'khung hop le/s':>14} | khoang cach (cm)")
    best = None
    for b in BAUDS:
        try:
            bps, fps, fr = try_baud(a.port, b)
        except serial.SerialException as e:
            print(f"{b:>7} | loi mo cong: {e}")
            continue
        dist = ""
        if fr:
            ds = sorted(d for d, _ in fr)
            dist = f"trung vi {ds[len(ds) // 2]}"
        print(f"{b:>7} | {bps:7.0f} | {fps:14.1f} | {dist}")
        if fps > 0 and (best is None or fps > best[1]):
            best = (b, fps)
    if best:
        print(f"\n-> TF03 dang phat o {best[0]} baud (~{best[1]:.0f} khung/s).")
        if best[0] != 115200:
            print(f"   Dung cac script voi --baud {best[0]}, hoac dat lai 115200 (can chieu TX):")
            print(f"   python tf03_tool.py --port {a.port} --baud {best[0]} set-baud 115200   (lenh nay tu luu)")
    else:
        print("\n-> Khong baud nao co khung hop le. Kiem tra day GND (den), day TX nau -> RXD, nguon 5 V.")


if __name__ == "__main__":
    main()
