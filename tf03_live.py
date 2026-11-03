"""
tf03_live.py - Doc TF03 lien tuc theo thoi gian thuc, chon duoc tan so.

Hai tan so KHAC NHAU:
  --sensor-hz : tan so TF03 do (1..1000 Hz, gia tri hop le (1..9)*10^N).
                Can chieu TX (CP2102 -> day xanh duong) thong. Khong dat -> TF03 giu tan so hien tai.
  --hz        : tan so CAP NHAT ket qua tren man hinh / file (vd 1, 5, 10, 20 Hz).
                Moi lan cap nhat gop tat ca khung nhan duoc trong khoang do -> trung binh, min, max.
                Khong can chieu TX.

Vi du:
  python tf03_live.py --port COM8 --hz 5                      # in 5 dong/giay
  python tf03_live.py --port COM8 --hz 10 --plot              # do thi cuon 10 s, cap nhat 10 lan/giay
  python tf03_live.py --port COM8 --hz 2 --sensor-hz 1000     # dat TF03 1000 Hz, in 2 dong/giay
  python tf03_live.py --port COM8 --hz 5 --log live.csv       # vua in vua ghi file
Dung: Ctrl+C
"""
import argparse
import collections
import time

import serial  # pip install pyserial

HEADER, FRAME_LEN = 0x59, 9
OUT_OF_RANGE, STRENGTH_MIN = 18000, 40
VALID_SENSOR_HZ = sorted({d * 10 ** k for k in range(4) for d in range(1, 10) if d * 10 ** k <= 1000})


# ---------------------------------------------------------------- giao thuc
def make_cmd(cmd_id, payload=b""):
    body = bytes([0x5A, 4 + len(payload), cmd_id]) + payload
    return body + bytes([sum(body) & 0xFF])


def cmd_frame_rate(hz):
    return make_cmd(0x03, int(hz).to_bytes(2, "little"))


class FrameParser:
    """Tach khung 59 59 ... CHK tu dong byte, dong bo lai khi checksum sai."""
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out = []
        while len(self.buf) >= FRAME_LEN:
            if self.buf[0] != HEADER or self.buf[1] != HEADER:
                self.buf.pop(0)
                continue
            f = self.buf[:FRAME_LEN]
            if sum(f[:8]) & 0xFF != f[8]:
                self.buf.pop(0)
                continue
            del self.buf[:FRAME_LEN]
            out.append((f[2] | f[3] << 8, f[4] | f[5] << 8))
        return out


def drain(ser, window=0.05):
    """Bo du lieu cu trong bo dem driver cho toi khi chi con luong byte that."""
    ser.reset_input_buffer()
    limit = 1.2 * ser.baudrate / 10 * window
    old_to, ser.timeout = ser.timeout, window
    quiet, t0 = 0, time.time()
    while quiet < 4 and time.time() - t0 < 30:
        quiet = quiet + 1 if len(ser.read(65536)) <= limit else 0
    ser.timeout = old_to


def measure_rate(ser, parser, seconds=1.0):
    n, t0 = 0, time.time()
    while time.time() - t0 < seconds:
        n += len(parser.feed(ser.read(4096)))
    return n / seconds


# ---------------------------------------------------------------- tong hop 1 cua so
def summarize(frames):
    """frames: list (dist, strength). Tra ve dict thong ke cho cac khung hop le."""
    good = [(d, s) for d, s in frames if d < OUT_OF_RANGE and s >= STRENGTH_MIN]
    res = {"n": len(frames), "n_ok": len(good)}
    if good:
        ds = [d for d, _ in good]
        res.update(mean=sum(ds) / len(ds), dmin=min(ds), dmax=max(ds),
                   strength=sum(s for _, s in good) / len(good))
    return res


def fmt_line(t, r, period):
    rate = r["n"] / period
    if r["n_ok"] == 0:
        return f"{t:8.2f}s | {'---':>8} cm | khong co khung hop le ({r['n']} khung) | {rate:6.0f} khung/s"
    return (f"{t:8.2f}s | {r['mean']:8.1f} cm | min {r['dmin']:5d} max {r['dmax']:5d} | "
            f"cuong do {r['strength']:6.0f} | {r['n_ok']:4d}/{r['n']:<4d} khung | {rate:6.0f} khung/s")


# ---------------------------------------------------------------- vong lap chinh
def run(ser, hz, sensor_hz=None, plot=False, log=None, duration=None):
    parser = FrameParser()
    drain(ser)
    ser.timeout = 0.005

    if sensor_hz:
        ser.write(cmd_frame_rate(sensor_hz))
        time.sleep(0.3)
        drain(ser)
        got = measure_rate(ser, parser)
        print(f"Da gui lenh dat {sensor_hz} Hz. TF03 dang phat ~{got:.0f} khung/s.")
        if abs(got - sensor_hz) > 0.15 * sensor_hz:
            print("  -> KHONG doi duoc tan so: kiem tra chieu TX (CP2102 TXD -> day xanh duong).")
        print("  (Cai dat mat khi tat nguon, tru khi dung lenh save cua tf03_tool.py)")
    else:
        got = measure_rate(ser, parser)
        print(f"TF03 dang phat ~{got:.0f} khung/s.")

    if got > 0 and hz > got:
        print(f"Canh bao: --hz {hz} lon hon toc do TF03 ({got:.0f}); nhieu lan cap nhat se khong co khung.")

    fh = None
    if log:
        fh = open(log, "w", encoding="utf-8")
        fh.write("t_s,n_frames,n_ok,mean_cm,min_cm,max_cm,strength\n")

    hist_t = collections.deque()
    hist_d = collections.deque()
    if plot:
        import matplotlib.pyplot as plt
        plt.ion()
        fig, ax = plt.subplots(figsize=(9, 4))
        line, = ax.plot([], [], "-", lw=1.2)
        ax.set_xlabel("thoi gian (s)")
        ax.set_ylabel("khoang cach (cm)")
        ax.grid(alpha=0.3)
        title = ax.set_title("")

    period = 1.0 / hz
    t_start = time.time()
    next_tick = t_start + period
    window = []
    print(f"Cap nhat {hz} lan/giay. Ctrl+C de dung.\n")
    try:
        while duration is None or time.time() - t_start < duration:
            window += parser.feed(ser.read(4096))
            now = time.time()
            if now < next_tick:
                continue
            t = now - t_start
            r = summarize(window)
            window = []
            next_tick += period
            if next_tick < now:                     # may cham, bo qua cac nhip bi tre
                next_tick = now + period
            print(fmt_line(t, r, period))
            if fh:
                if r["n_ok"]:
                    fh.write(f"{t:.3f},{r['n']},{r['n_ok']},{r['mean']:.2f},{r['dmin']},{r['dmax']},{r['strength']:.1f}\n")
                else:
                    fh.write(f"{t:.3f},{r['n']},0,,,,\n")
            if plot and r["n_ok"]:
                hist_t.append(t)
                hist_d.append(r["mean"])
                while hist_t and t - hist_t[0] > 10:
                    hist_t.popleft()
                    hist_d.popleft()
                line.set_data(hist_t, hist_d)
                ax.set_xlim(max(0, t - 10), max(10, t))
                lo, hi = min(hist_d), max(hist_d)
                pad = max(2.0, 0.1 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)
                title.set_text(f"{r['mean']:.1f} cm  (cap nhat {hz} Hz)")
                plt.pause(0.001)
    except KeyboardInterrupt:
        print("\nDa dung.")
    finally:
        if fh:
            fh.close()
            print(f"Da luu {log}")


def main():
    ap = argparse.ArgumentParser(description="Doc TF03 lien tuc, chon tan so cap nhat.")
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hz", type=float, default=5, help="tan so cap nhat ket qua (lan/giay)")
    ap.add_argument("--sensor-hz", type=int, default=None, help="dat tan so do cua TF03 (can chieu TX)")
    ap.add_argument("--plot", action="store_true", help="ve do thi cuon 10 s")
    ap.add_argument("--log", default=None, help="ghi ket qua moi lan cap nhat ra file CSV")
    a = ap.parse_args()
    if a.sensor_hz and a.sensor_hz not in VALID_SENSOR_HZ:
        ap.error(f"--sensor-hz phai la mot trong {VALID_SENSOR_HZ}")
    if not 0 < a.hz <= 100:
        ap.error("--hz trong khoang (0, 100]; in nhanh hon 100 dong/giay thi khong doc kip")
    with serial.Serial(a.port, a.baud, timeout=0.1) as ser:
        run(ser, a.hz, a.sensor_hz, a.plot, a.log)


if __name__ == "__main__":
    main()
