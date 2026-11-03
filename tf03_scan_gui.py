"""
tf03_scan_gui.py - Quet tay co HIEN THI THOI GIAN THUC.

Cua so ve:
  - Tia do  : khoang cach TF03 dang do, ve o goc hien tai (cap nhat ~10 lan/giay)
  - Cham xanh: cac diem da chot
Thao tac (bam phim khi cua so do thi dang duoc chon):
  Space / Enter : chot diem o goc hien tai, roi tang goc len 1 buoc
  Mui ten trai/phai : giam/tang goc 1 buoc       Mui ten len/xuong : +1 / -1 do
  u : xoa diem vua chot      s : luu CSV + PNG      q : luu va thoat

Chay:  python tf03_scan_gui.py --step 5 --offset 12 -o phong_1.csv --rmax 8
--offset (cm): quang duong tia tu TF03 xuong tam guong. Khoang cach ve = do duoc - offset
              (= khoang cach tinh tu TRUC QUAY guong). Cach do: xem huong dan hieu chuan offset.
Goc do em DAT bang tay (chua co encoder): xoay kim toi dung goc hien tren cua so roi moi bam Space.
"""
import argparse
import collections
import csv
import math
import statistics
import threading
import time

import serial  # pip install pyserial
import matplotlib.pyplot as plt

from tf03_live import FrameParser, drain, OUT_OF_RANGE, STRENGTH_MIN


class Reader(threading.Thread):
    """Doc TF03 lien tuc o luong rieng, giu cac khung cua ~0.5 s gan nhat."""
    def __init__(self, ser, keep_s=0.5):
        super().__init__(daemon=True)
        self.ser, self.keep_s = ser, keep_s
        self.buf = collections.deque()
        self.lock = threading.Lock()
        self.running = True
        self.rate = 0.0

    def run(self):
        p = FrameParser()
        drain(self.ser)
        self.ser.timeout = 0.02
        n, t_rate = 0, time.time()
        while self.running:
            frames = p.feed(self.ser.read(4096))
            now = time.time()
            with self.lock:
                for f in frames:
                    self.buf.append((now, f[0], f[1]))
                while self.buf and now - self.buf[0][0] > self.keep_s:
                    self.buf.popleft()
            n += len(frames)
            if now - t_rate >= 1.0:
                self.rate, n, t_rate = n / (now - t_rate), 0, now

    def latest(self, seconds):
        """(trung vi kc cm, do dao dong cm, cuong do TB, so khung hop le) trong `seconds` giay gan nhat."""
        now = time.time()
        with self.lock:
            fr = [(d, s) for t, d, s in self.buf if now - t <= seconds]
        good = [(d, s) for d, s in fr if d < OUT_OF_RANGE and s >= STRENGTH_MIN]
        if not good:
            return None, None, 0.0, 0
        ds = [d for d, _ in good]
        return statistics.median(ds), max(ds) - min(ds), sum(s for _, s in good) / len(good), len(good)


class ScanGUI:
    def __init__(self, reader, step, rmax, out, offset=0.0):
        self.r, self.step, self.rmax, self.out, self.offset = reader, step, rmax, out, offset
        self.angle = 0.0
        self.points = []                       # dict angle, dist, spread, strength, n
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-rmax, rmax)
        self.ax.set_ylim(-rmax, rmax)
        self.ax.grid(alpha=0.3)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.plot(0, 0, "k^", ms=10)
        self.pts, = self.ax.plot([], [], "o", color="#1F5FA8", ms=6)
        self.ray, = self.ax.plot([], [], "-", color="#C0392B", lw=2)
        self.tip, = self.ax.plot([], [], "o", color="#C0392B", ms=9)
        self.dirline, = self.ax.plot([], [], ":", color="0.5", lw=1)
        self.info = self.ax.text(0.02, 0.98, "", transform=self.ax.transAxes, va="top", family="monospace",
                                 fontsize=10, bbox=dict(boxstyle="round", fc="white", alpha=0.85))
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.timer = self.fig.canvas.new_timer(interval=100)
        self.timer.add_callback(self.refresh)
        self.timer.start()

    # ---------- ve ----------
    def refresh(self):
        d, spread, s, n = self.r.latest(0.1)
        t = math.radians(self.angle)
        self.dirline.set_data([0, self.rmax * math.cos(t)], [0, self.rmax * math.sin(t)])
        if d is not None:
            d = d - self.offset
            rm = d / 100
            x, y = rm * math.cos(t), rm * math.sin(t)
            self.ray.set_data([0, x], [0, y])
            self.tip.set_data([x], [y])
            meas = f"{d:7.1f} cm (offset {self.offset:g})  dao dong {spread:3.0f} cm  cuong do {s:5.0f}"
        else:
            self.ray.set_data([], [])
            self.tip.set_data([], [])
            meas = "   --- khong co khung hop le"
        self.info.set_text(f"Goc hien tai : {self.angle:6.1f} do   (buoc {self.step:g})\n"
                           f"Dang do      : {meas}\n"
                           f"TF03         : {self.r.rate:5.0f} khung/s    Da chot: {len(self.points)} diem\n"
                           f"Space=chot  <-/->=doi goc  u=xoa  s=luu  q=thoat")
        self.fig.canvas.draw_idle()

    def redraw_points(self):
        xs = [p["dist_cm"] / 100 * math.cos(math.radians(p["angle_deg"])) for p in self.points]
        ys = [p["dist_cm"] / 100 * math.sin(math.radians(p["angle_deg"])) for p in self.points]
        self.pts.set_data(xs, ys)

    # ---------- phim ----------
    def on_key(self, ev):
        k = ev.key
        if k in (" ", "enter"):
            self.capture()
        elif k == "right":
            self.angle = (self.angle + self.step) % 360
        elif k == "left":
            self.angle = (self.angle - self.step) % 360
        elif k == "up":
            self.angle = (self.angle + 1) % 360
        elif k == "down":
            self.angle = (self.angle - 1) % 360
        elif k == "u" and self.points:
            p = self.points.pop()
            self.angle = p["angle_deg"]
            self.redraw_points()
        elif k == "s":
            self.save()
        elif k == "q":
            self.save()
            plt.close(self.fig)
        self.refresh()

    def capture(self):
        d, spread, s, n = self.r.latest(0.2)        # 0,2 s ~ 200 khung o 1000 Hz
        if d is None:
            print(f"{self.angle:6.1f} do: khong co khung hop le - chua chot")
            return
        raw = d
        d = d - self.offset
        self.points.append({"angle_deg": round(self.angle, 2), "dist_cm": d, "raw_cm": raw, "spread_cm": spread,
                            "strength": round(s, 1), "n_ok": n})
        print(f"{self.angle:6.1f} do: {d:7.1f} cm | dao dong {spread:3.0f} cm | cuong do {s:5.0f} | {n} khung")
        self.redraw_points()
        self.angle = (self.angle + self.step) % 360

    def save(self):
        with open(self.out, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=["angle_deg", "dist_cm", "raw_cm", "spread_cm", "strength", "n_ok"])
            w.writeheader()
            w.writerows(self.points)
        png = self.out.rsplit(".", 1)[0] + ".png"
        self.fig.savefig(png, dpi=130)
        print(f"Da luu {self.out} va {png} ({len(self.points)} diem)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--step", type=float, default=10)
    ap.add_argument("--rmax", type=float, default=8.0, help="ban kinh khung nhin (m)")
    ap.add_argument("-o", "--out", default="quet_gui.csv")
    ap.add_argument("--offset", type=float, default=0.0, help="cm, quang duong TF03 -> tam guong, se bi tru")
    a = ap.parse_args()
    with serial.Serial(a.port, a.baud, timeout=0.1) as ser:
        rd = Reader(ser)
        rd.start()
        gui = ScanGUI(rd, a.step, a.rmax, a.out, a.offset)
        plt.show()
        rd.running = False
        rd.join(timeout=1)


if __name__ == "__main__":
    main()
