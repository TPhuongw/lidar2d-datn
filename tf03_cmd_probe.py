"""
tf03_cmd_probe.py - Tim dieu kien de TF03 NHAN LENH o baud cao.

Gui lenh 'version' (5A 04 01 5F) nhieu lan, voi nhieu cau hinh phia PC:
  - baud lech quanh baud danh nghia (do lech dong ho)
  - 1 hoac 2 stop bit
Ghi lai cau hinh nao co phan hoi 5A 07 01.

Chay (dong GUI truoc):  python tf03_cmd_probe.py --port COM8 --baud 460800
Luu y: CP2102 doi cu chi tao duoc mot so baud nhat dinh; baud le co the bi lam tron.
"""
import argparse
import time

import serial  # pip install pyserial

CMD_VERSION = bytes.fromhex("5A 04 01 5F")


def try_once(port, baud, stopbits, repeats=3):
    with serial.Serial(port, baud, timeout=0.05, stopbits=stopbits) as s:
        hits = 0
        for _ in range(repeats):
            s.reset_input_buffer()
            s.write(CMD_VERSION)
            t_end, buf = time.time() + 0.3, bytearray()
            while time.time() < t_end:
                buf += s.read(4096)
            i = buf.find(b"\x5A\x07\x01")
            if i >= 0 and len(buf) >= i + 7 and (sum(buf[i:i + 6]) & 0xFF) == buf[i + 6]:
                hits += 1
                ver = f"{buf[i+5]}.{buf[i+4]}.{buf[i+3]}"
        return hits, (ver if hits else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=460800)
    a = ap.parse_args()
    offsets = [0, -1, +1, -2, +2, -3, +3, -4, +4]          # %
    print(f"{'baud':>8} | {'lech':>5} | stop | phan hoi/3 | firmware")
    for off in offsets:
        b = round(a.baud * (1 + off / 100))
        for sb in (serial.STOPBITS_ONE, serial.STOPBITS_TWO):
            try:
                h, v = try_once(a.port, b, sb)
            except (serial.SerialException, ValueError) as e:
                print(f"{b:>8} | {off:+4d}% | {sb:>4} | loi: {e}")
                continue
            print(f"{b:>8} | {off:+4d}% | {sb:>4} | {h:>10} | {v}")


if __name__ == "__main__":
    main()
