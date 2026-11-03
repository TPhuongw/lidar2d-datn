"""
tf03_raw_view.py - Xem du lieu tho tu TF03 theo tung tang, de hieu giao thuc.

Che do:
  raw   : in tung byte dang hex, 16 byte/dong (chua hieu gi ca, chi la dong byte)
  frame : tim dau khung 59 59, cat 9 byte, kiem tra checksum, giai ma khoang cach/cuong do

Chay:
  python tf03_raw_view.py --port COM3 --mode raw
  python tf03_raw_view.py --port COM3 --mode frame -n 30
"""
import argparse
import time

import serial  # pip install pyserial

HEADER = 0x59
FRAME_LEN = 9


def drain(ser, window=0.05):
    """Bo du lieu cu ton trong bo dem driver: doc cho den khi 1 cua so chi con
    luong byte bang toc do thuc te cua duong truyen (khong con 'xa' du lieu cu)."""
    ser.reset_input_buffer()
    limit = 1.2 * ser.baudrate / 10 * window          # so byte toi da UART truyen duoc trong 'window' giay
    ser.timeout = window
    quiet = 0
    t0 = time.time()
    while quiet < 4 and time.time() - t0 < 30:
        n = len(ser.read(65536))
        quiet = quiet + 1 if n <= limit else 0


def checksum(frame8):
    """Byte thap cua tong 8 byte dau (da kiem chung tren TF03 cua minh)."""
    return sum(frame8) & 0xFF


def decode(frame):
    """frame: 9 byte. Tra ve dict, hoac None neu checksum sai."""
    if checksum(frame[:8]) != frame[8]:
        return None
    dist = frame[2] | (frame[3] << 8)        # little-endian: byte thap truoc
    strength = frame[4] | (frame[5] << 8)
    return {"dist_cm": dist, "strength": strength}


def mode_raw(ser, lines):
    """Tang 1: chi in byte. Hay tu tim cap 59 59 bang mat."""
    print("Moi dong 16 byte (hex). Tim cap '59 59' - do la dau khung.\n")
    count = 0
    buf = b""
    while count < lines:
        buf += ser.read(64)
        while len(buf) >= 16 and count < lines:
            chunk, buf = buf[:16], buf[16:]
            print(" ".join(f"{b:02X}" for b in chunk))
            count += 1


def mode_frame(ser, n):
    """Tang 2+3: dong bo theo dau khung, kiem tra checksum, giai ma."""
    print(f"{'#':>4}  {'9 byte nhan duoc (hex)':<28} {'CHK':>9}  {'kc (cm)':>8} {'cuong do':>9}  ghi chu")
    buf = bytearray()
    got, skipped, bad = 0, 0, 0
    t0 = time.time()
    while got < n:
        buf += ser.read(64)
        while len(buf) >= FRAME_LEN and got < n:
            # 1) Dong bo: dau buffer phai la 59 59, neu khong thi bo 1 byte va thu lai
            if not (buf[0] == HEADER and buf[1] == HEADER):
                buf.pop(0)
                skipped += 1
                continue
            frame = bytes(buf[:FRAME_LEN])
            res = decode(frame)
            hexs = " ".join(f"{b:02X}" for b in frame)
            calc = checksum(frame[:8])
            if res is None:
                # 2) Checksum sai: 59 59 nay co the chi la du lieu trung gia tri -> bo 1 byte, dong bo lai
                print(f"{'-':>4}  {hexs:<28} {calc:02X}!={frame[8]:02X}  {'':>8} {'':>9}  checksum sai -> dong bo lai")
                buf.pop(0)
                bad += 1
                continue
            del buf[:FRAME_LEN]
            got += 1
            note = ""
            if res["dist_cm"] >= 18000:
                note = "ngoai tam do"
            elif res["strength"] < 40:
                note = "tin hieu yeu, khong tin cay"
            print(f"{got:>4}  {hexs:<28} {calc:02X}=={frame[8]:02X}  {res['dist_cm']:>8} {res['strength']:>9}  {note}")
    dt = time.time() - t0
    print(f"\n{got} khung hop le trong {dt:.2f} s (~{got / dt:.1f} khung/s); "
          f"bo {skipped} byte de dong bo, {bad} lan checksum sai")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["raw", "frame"], default="frame")
    ap.add_argument("-n", type=int, default=20, help="so dong (raw) hoac so khung (frame)")
    a = ap.parse_args()
    with serial.Serial(a.port, a.baud, timeout=0.1) as ser:
        drain(ser)
        ser.timeout = 0.1
        (mode_raw if a.mode == "raw" else mode_frame)(ser, a.n)


if __name__ == "__main__":
    main()
