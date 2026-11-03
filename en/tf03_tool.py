"""
tf03_tool.py - Cong cu kiem tra Benewake TF03 qua USB-UART (3.3 V) tren PC.

Nguon: TF03 UART/CAN Product Manual V1.2.2 (Benewake).
    - UART mac dinh 115200, 8N1, muc LVTTL 0-3.3 V
    - Khung du lieu 9 byte: 59 59 DIST_L DIST_H STR_L STR_H R R CHK   (DIST don vi cm)
    - Lenh: 5A LEN ID PAYLOAD CHK, CHK = byte thap cua tong cac byte truoc no
    - DIST = 18000 (nguong mac dinh) -> ngoai tam, phai loai bo
    - Strength < 40 -> khong tin cay; 40..1200 tin cay

LUU Y: manual ghi checksum khung du lieu la "Byte0 + Byte2 + ... + Byte7" (thieu Byte1),
nhieu kha nang la loi in. Lenh `check` ben duoi tu kiem tra quy tac nao dung voi cam bien that.

Cai dat:  pip install pyserial numpy
Vi du:
    python tf03_tool.py --port COM5 check            # kiem tra ket noi + quy tac checksum
    python tf03_tool.py --port COM5 version
    python tf03_tool.py --port COM5 stats -n 1000    # mean, std, strength, toc do khung, loi
    python tf03_tool.py --port COM5 log -n 5000 -o d_2m_100hz.csv --note "tam trang 2 m"
    python tf03_tool.py --port COM5 set-rate 1000
    python tf03_tool.py --port COM5 --baud 115200 set-baud 460800  # --baud = baud HIEN TAI; sau do mo lai voi --baud 460800
    python tf03_tool.py --port COM5 save
"""
import argparse
import csv
import struct
import sys
import time

import numpy as np

try:
    import serial
except ImportError:          # cho phep import module de test parser khi chua cai pyserial
    serial = None

HEADER = 0x59
OUT_OF_RANGE_CM = 18000
STRENGTH_MIN = 40
VALID_RATES = sorted({k * 10 ** n for k in range(1, 10) for n in range(5) if k * 10 ** n <= 10000})


# ----------------------------------------------------------------------------
# Lenh
# ----------------------------------------------------------------------------
def make_cmd(cmd_id, payload=b""):
    """5A LEN ID PAYLOAD CHK; LEN = tong so byte cua ca khung."""
    body = bytes([0x5A, 4 + len(payload), cmd_id]) + bytes(payload)
    return body + bytes([sum(body) & 0xFF])


CMD_VERSION = make_cmd(0x01)             # 5A 04 01 5F
CMD_RESET = make_cmd(0x02)               # 5A 04 02 60
CMD_TRIGGER = make_cmd(0x04)             # 5A 04 04 62
CMD_FACTORY = make_cmd(0x10)             # 5A 04 10 6E
CMD_SAVE = make_cmd(0x11)                # 5A 04 11 6F


def cmd_frame_rate(hz):
    return make_cmd(0x03, struct.pack("<H", hz))          # 5A 06 03 LL HH SU


def cmd_baud(baud):
    return make_cmd(0x06, struct.pack("<I", baud))        # 5A 08 06 b0 b1 b2 b3 SU


def cmd_output(enable):
    return make_cmd(0x07, bytes([1 if enable else 0]))


# ----------------------------------------------------------------------------
# Giai ma khung du lieu
# ----------------------------------------------------------------------------
def checksum_ok(frame, rule):
    if rule == "all":            # tong byte 0..7 (quy tac thong dung cua dong TF)
        s = sum(frame[0:8])
    else:                        # "skip1": dung nguyen van cong thuc in trong manual
        s = frame[0] + sum(frame[2:8])
    return (s & 0xFF) == frame[8]


class FrameParser:
    """Bo tach khung chiu loi: dong bo lai theo 59 59, dem khung hong."""

    def __init__(self, rule="all"):
        self.buf = bytearray()
        self.rule = rule
        self.n_ok = 0
        self.n_bad = 0

    def feed(self, data):
        """Nhan byte moi, tra ve list (dist_cm, strength)."""
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\x59\x59")
            if i < 0:
                del self.buf[:-1]               # giu byte cuoi phong truong hop 59 bi cat
                return out
            if len(self.buf) - i < 9:
                del self.buf[:i]
                return out
            frame = self.buf[i:i + 9]
            if checksum_ok(frame, self.rule):
                dist, strength = struct.unpack_from("<HH", frame, 2)
                out.append((dist, strength))
                self.n_ok += 1
                del self.buf[:i + 9]
            else:
                self.n_bad += 1
                del self.buf[:i + 1]            # truot 1 byte, tim header tiep theo


# ----------------------------------------------------------------------------
# Giao tiep
# ----------------------------------------------------------------------------
def open_port(port, baud):
    if serial is None:
        sys.exit("pyserial not installed: pip install pyserial")
    return serial.Serial(port, baud, timeout=0.05)


def send_cmd(ser, cmd, wait=0.5):
    """
    Gui lenh, doc cac khung phan hoi bat dau bang 5A (bo qua khung du lieu 59 59).
    Xa du lieu ton dong TRUOC khi gui: neu khong, phan hoi nam sau hang tram KB du lieu cu
    va khong kip doc ra trong thoi gian cho.
    """
    old = drain(ser)
    if old:
        print(f"  (discarded {old} stale bytes before sending the command)")
    gap = getattr(ser, "byte_gap_ms", 0)
    if gap > 0:                                   # gui tung byte, cach nhau gap ms
        for b in cmd:
            ser.write(bytes([b]))
            ser.flush()
            time.sleep(gap / 1000)
    else:
        ser.write(cmd)
    t_end = time.time() + wait
    buf = bytearray()
    while time.time() < t_end:
        buf += ser.read(max(1, ser.in_waiting))
    replies = []
    i = 0
    while True:
        i = buf.find(b"\x5A", i)
        if i < 0 or i + 2 > len(buf):
            break
        n = buf[i + 1]
        if 4 <= n <= 16 and i + n <= len(buf):
            fr = bytes(buf[i:i + n])
            if (sum(fr[:-1]) & 0xFF) == fr[-1]:
                replies.append(fr)
                i += n
                continue
        i += 1
    print("  sent:", cmd.hex(" ").upper())
    print("  recv:", ", ".join(r.hex(" ").upper() for r in replies) or "(no 5A reply)")
    return replies


def line_limit_bytes(ser, window):
    """So byte toi da UART co the truyen trong 'window' giay (8N1 = 10 bit/byte)."""
    return ser.baudrate / 10.0 * window


def drain(ser, window=0.05, quiet_windows=4, max_time=120.0):
    """
    Xa sach du lieu ton dong truoc khi do.

    Thuc te da gap: reset_input_buffer() KHONG xoa duoc du lieu nam san trong driver/chip
    USB-UART (module CP2102 clone tren Windows), va luong ton dong co the rat lon
    (hang tram KB neu cam bien chay 1000 Hz trong luc khong co chuong trinh nao doc).
    Du lieu ton dong duoc doc ra voi toc do USB (nhanh hon nhieu so voi UART).

    Cach lam: doc bo theo tung cua so 'window' giay. Chi khi lien tiep 'quiet_windows'
    cua so co luong byte <= gioi han vat ly cua UART thi moi coi la da het ton dong.
    """
    ser.reset_input_buffer()
    limit = 1.2 * line_limit_bytes(ser, window)
    total, quiet = 0, 0
    t_start = time.perf_counter()
    while quiet < quiet_windows:
        if time.perf_counter() - t_start > max_time:
            print(f"  WARNING: stale data still arriving after {max_time:.0f} s. Replug USB.")
            break
        t_end = time.perf_counter() + window
        n = 0
        while time.perf_counter() < t_end:
            n += len(ser.read(max(1, ser.in_waiting)))
        total += n
        quiet = quiet + 1 if n <= limit else 0
    return total


def check_realtime(n_frames, duration, ser):
    """Canh bao neu toc do khung vuot gioi han vat ly cua UART -> du lieu khong phai thoi gian thuc."""
    if duration <= 0:
        return
    rate = (n_frames - 1) / duration
    max_rate = ser.baudrate / 10.0 / 9.0
    if rate > 1.1 * max_rate:
        print(f"  WARNING: {rate:.0f} frames/s exceeds the UART limit ({max_rate:.0f} frames/s at "
              f"{ser.baudrate} baud). This is buffered stale data, NOT usable.")


def read_samples(ser, n, rule="all", timeout=None):
    """Doc n mau, tra ve mang (n, 3): t_host (s), dist (cm), strength + parser."""
    if timeout is None:
        timeout = max(30.0, n / 50.0)          # du cho 100 Hz voi he so an toan 2
    old = drain(ser)
    if old:
        print(f"  (discarded {old} stale bytes before measuring)")
    p = FrameParser(rule)
    rows = []
    t0 = time.perf_counter()
    while len(rows) < n:
        if time.perf_counter() - t0 > timeout:
            print(f"  Timeout: only {len(rows)}/{n} samples read")
            break
        data = ser.read(max(1, ser.in_waiting))
        t = time.perf_counter() - t0
        for d, s in p.feed(data):
            rows.append((t, d, s))
    return np.array(rows[:n], dtype=float).reshape(-1, 3), p


# ----------------------------------------------------------------------------
# Cac lenh dong lenh
# ----------------------------------------------------------------------------
def do_check(ser, args):
    """Thu ca hai quy tac checksum tren cung luong byte de biet cai nao dung."""
    old = drain(ser)
    t0 = time.perf_counter()
    raw = bytearray()
    while time.perf_counter() - t0 < 1.0:      # cua so do 1 giay, khong gioi han 4096 byte
        raw += ser.read(max(1, ser.in_waiting))
    raw = bytes(raw)
    print(f"  (discarded {old} old bytes from the buffer)")
    print(f"  Received {len(raw)} bytes in 1.0 s  ->  ~{len(raw) / 9:.0f} frames/s for 9-byte frames")
    if not raw:
        print("  NO data. Check: power, TX/RX swap, baud, common GND, 3.3 V levels.")
        return
    print("  first 20 bytes:", raw[:20].hex(" ").upper())
    for rule in ("all", "skip1"):
        p = FrameParser(rule)
        frames = p.feed(raw)
        print(f"  rule '{rule}': {p.n_ok} valid frames, {p.n_bad} checksum errors")
        if frames:
            print(f"    example: dist = {frames[0][0]} cm, strength = {frames[0][1]}")
    print("  -> Use the rule with many valid frames and ~0 errors (pass with --rule).")


def do_stats(ser, args):
    data, p = read_samples(ser, args.n, args.rule)
    if len(data) < 2:
        print("  Not enough data.")
        return
    t, d, s = data.T
    dur = t[-1] - t[0]
    valid = (d < OUT_OF_RANGE_CM) & (s >= STRENGTH_MIN)
    print(f"  Frames: {len(d)} in {dur:.2f} s  ->  ~{(len(d) - 1) / dur:.0f} frames/s")
    check_realtime(len(d), dur, ser)
    print(f"  Bad frames (checksum): {p.n_bad}")
    print(f"  Valid samples (dist < {OUT_OF_RANGE_CM}, strength >= {STRENGTH_MIN}): "
          f"{valid.sum()}/{len(d)}")
    if valid.sum() > 1:
        dv = d[valid]
        print(f"  Distance: mean = {dv.mean():.2f} cm, std = {dv.std(ddof=1):.2f} cm, "
              f"min = {dv.min():.0f}, max = {dv.max():.0f}")
        print(f"  Strength: mean = {s[valid].mean():.0f}, min = {s[valid].min():.0f}")


def do_log(ser, args):
    data, p = read_samples(ser, args.n, args.rule)
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# note: {args.note}", f"baud={args.baud}", f"rule={args.rule}",
                    f"bad_frames={p.n_bad}", time.strftime("%Y-%m-%d %H:%M:%S")])
        w.writerow(["t_host_s", "dist_cm", "strength"])
        for row in data:
            w.writerow([f"{row[0]:.6f}", int(row[1]), int(row[2])])
    print(f"  Wrote {len(data)} samples to {args.out} (bad frames: {p.n_bad})")
    if len(data) > 1:
        dur = data[-1, 0] - data[0, 0]
        print(f"  Logging rate: ~{(len(data) - 1) / dur:.0f} frames/s over {dur:.2f} s")
        check_realtime(len(data), dur, ser)


def do_set_rate(ser, args):
    if args.hz not in VALID_RATES:
        sys.exit(f"  {args.hz} Hz is not valid. Manual: only (1..9) x 10^N accepted; "
                 f"other values fall back to 100 Hz. Valid: {VALID_RATES}")
    load = args.hz * 90 / ser.baudrate
    print(f"  Link load at {ser.baudrate} baud: {load*100:.0f}%")
    if load > 0.8:
        print(f"  WARNING: > 80% -> lost frames / checksum errors. Need baud >= {int(args.hz*90/0.8)}")
    if args.hz > 1000:
        print("  Note: above 1000 Hz not verified for this model/firmware - verify with 'check'.")
    send_cmd(ser, cmd_frame_rate(args.hz))
    print("  Run 'save' to keep this setting after power-off.")


def do_set_baud(ser, args):
    """TF03: lenh doi baud chi co hieu luc sau khi LUU va KHOI DONG LAI.
    Vi vay lenh save phai gui o baud CU (cam bien chua doi). Neu khong phan hoi moi thu o baud moi."""
    old_baud = ser.baudrate
    print(f"  Port opened at {old_baud} baud (sensor CURRENT baud).")
    print(f"  After save and POWER CYCLE, the sensor will run at {args.new_baud} baud.")
    send_cmd(ser, cmd_baud(args.new_baud))
    time.sleep(0.2)
    r = send_cmd(ser, CMD_SAVE)
    if not r:
        print(f"  No reply at {old_baud}, retrying save at {args.new_baud}...")
        ser.baudrate = args.new_baud
        time.sleep(0.2)
        send_cmd(ser, CMD_SAVE)
    print(f"  -> POWER CYCLE the TF03, then verify:  --baud {args.new_baud} check")


def main():
    ap = argparse.ArgumentParser(description="TF03 test tool")
    ap.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--gap", type=float, default=0,
                    help="ms pause between command bytes")
    ap.add_argument("--rule", choices=["all", "skip1"], default="all",
                    help="data-frame checksum rule (see check)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    sub.add_parser("version")
    sub.add_parser("save")
    sub.add_parser("factory")
    sub.add_parser("trigger")
    st = sub.add_parser("stats"); st.add_argument("-n", type=int, default=1000)
    lg = sub.add_parser("log")
    lg.add_argument("-n", type=int, default=1000)
    lg.add_argument("-o", "--out", required=True)
    lg.add_argument("--note", default="")
    sr = sub.add_parser("set-rate"); sr.add_argument("hz", type=int)
    sb = sub.add_parser("set-baud"); sb.add_argument("new_baud", type=int, help="new baud")
    args = ap.parse_args()

    with open_port(args.port, args.baud) as ser:
        ser.byte_gap_ms = args.gap
        print(f"[{args.cmd}] {args.port} @ {args.baud}" + (f"  (gap {args.gap:g} ms/byte)" if args.gap else ""))
        if args.cmd == "check":
            do_check(ser, args)
        elif args.cmd == "version":
            r = send_cmd(ser, CMD_VERSION)
            for fr in r:
                if len(fr) == 7 and fr[2] == 0x01:
                    print(f"  Firmware: {fr[5]}.{fr[4]}.{fr[3]}")
        elif args.cmd == "save":
            send_cmd(ser, CMD_SAVE)
        elif args.cmd == "factory":
            send_cmd(ser, CMD_FACTORY)
        elif args.cmd == "trigger":
            send_cmd(ser, CMD_TRIGGER)
        elif args.cmd == "stats":
            do_stats(ser, args)
        elif args.cmd == "log":
            do_log(ser, args)
        elif args.cmd == "set-rate":
            do_set_rate(ser, args)
        elif args.cmd == "set-baud":
            do_set_baud(ser, args)


if __name__ == "__main__":
    main()
