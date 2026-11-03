"""
tf03_manual_scan.py - Quet 2D BANG TAY: xoay guong toi tung goc, nhan Enter de do.

Moi goc: bo du lieu cu trong bo dem, lay N khung moi, ghi trung vi khoang cach + cuong do.
Ket thuc: luu CSV va ve anh (cuc + Descartes).

Chay:
  python tf03_manual_scan.py --port COM8 --step 10 -o scan1.csv
  python tf03_manual_scan.py --port COM8 --start 0 --end 180 --step 5 -n 50 -o scan_180.csv
Phim khi do:  Enter = do tai goc dang hien
              so (vd 37.5) + Enter = do tai goc do (thay vi goc goi y)
              s = bo qua goc nay, r = do lai goc vua roi, q = ket thuc va luu

Quy uoc goc: nhin tu TREN xuong, goc tang NGUOC chieu kim dong ho, 0 do = huong moc em chon.
Xoay guong 45 do quanh truc dung mot goc theta thi tia ra cung xoay dung theta.
"""
import argparse
import csv
import math
import statistics
import time

import serial  # pip install pyserial

from tf03_live import FrameParser, drain, OUT_OF_RANGE, STRENGTH_MIN


def capture(ser, parser, n, timeout=3.0):
    """Bo du lieu cu roi lay n khung moi. Tra ve (trung vi kc, do lech, cuong do TB, so khung hop le, tong)."""
    drain(ser)
    parser.buf.clear()
    ser.timeout = 0.02
    frames, t0 = [], time.time()
    while len(frames) < n and time.time() - t0 < timeout:
        frames += parser.feed(ser.read(4096))
    good = [(d, s) for d, s in frames if d < OUT_OF_RANGE and s >= STRENGTH_MIN]
    if not good:
        return None, None, (sum(s for _, s in frames) / len(frames) if frames else 0), 0, len(frames)
    ds = [d for d, _ in good]
    spread = (max(ds) - min(ds)) if len(ds) > 1 else 0
    return statistics.median(ds), spread, sum(s for _, s in good) / len(good), len(good), len(frames)


def plot(rows, out_png, rmax=15.0):
    """Ve anh quet. Diem xa hon rmax (m) KHONG ve, chi liet ke - tranh 1 diem bat thuong lam be ca anh."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ok = [r for r in rows if r["dist_cm"] not in (None, "")]
    for r in ok:
        r["dist_cm"] = float(r["dist_cm"]); r["angle_deg"] = float(r["angle_deg"])
    far = [r for r in ok if r["dist_cm"] / 100 > rmax]
    ok = [r for r in ok if r["dist_cm"] / 100 <= rmax]
    th = [math.radians(r["angle_deg"]) for r in ok]
    rm = [r["dist_cm"] / 100 for r in ok]
    fig = plt.figure(figsize=(13, 6))
    ax1 = fig.add_subplot(1, 2, 1, projection="polar")
    ax1.plot(th, rm, "o-", ms=4, lw=0.8)
    ax1.set_title("Toa do cuc (r [m], goc)")
    ax2 = fig.add_subplot(1, 2, 2)
    xs = [r * math.cos(t) for r, t in zip(rm, th)]
    ys = [r * math.sin(t) for r, t in zip(rm, th)]
    ax2.plot(xs, ys, "o", ms=5)
    ax2.plot(0, 0, "r^", ms=10)
    for r, x, y in zip(ok, xs, ys):
        ax2.annotate(f"{r['angle_deg']:.0f}", (x, y), fontsize=7, color="0.35", xytext=(3, 3), textcoords="offset points")
    ax2.set_aspect("equal")
    ax2.grid(alpha=0.3)
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_title("Descartes: x = r·cosθ, y = r·sinθ  (tam giac do = LiDAR, so = goc)")
    if far:
        txt = "Bo khoi anh (> %.0f m): " % rmax + ", ".join(f"{r['angle_deg']:.0f}°={r['dist_cm']/100:.1f} m" for r in far)
        fig.text(0.5, 0.01, txt, ha="center", fontsize=9, color="#B04A3A")
        print("  " + txt)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    fig.savefig(out_png, dpi=130)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--start", type=float, default=0)
    ap.add_argument("--end", type=float, default=355)
    ap.add_argument("--step", type=float, default=10)
    ap.add_argument("-n", type=int, default=50, help="so khung lay o moi goc")
    ap.add_argument("-o", "--out", default="manual_scan.csv")
    ap.add_argument("--rmax", type=float, default=15.0, help="chi ve diem gan hon rmax (m)")
    a = ap.parse_args()

    angles, x = [], a.start
    while x <= a.end + 1e-9:
        angles.append(round(x, 3))
        x += a.step
    rows = []
    with serial.Serial(a.port, a.baud, timeout=0.1) as ser:
        parser = FrameParser()
        i = 0
        print(f"{len(angles)} goc, buoc {a.step} do. Enter=do, so=goc khac, s=bo qua, r=do lai, q=ket thuc.\n")
        while i < len(angles):
            cmd = input(f"Xoay toi {angles[i]:6.1f} do roi nhan Enter > ").strip().lower()
            if cmd == "q":
                break
            if cmd == "s":
                i += 1
                continue
            if cmd == "r":
                if rows:
                    rows.pop()
                    i = max(0, i - 1)
                continue
            ang = angles[i]
            if cmd:
                try:
                    ang = float(cmd)
                except ValueError:
                    print("  Khong hieu lenh.")
                    continue
            d, spread, s, n_ok, n_all = capture(ser, parser, a.n)
            if d is None:
                print(f"  {ang:6.1f} do: KHONG co khung hop le ({n_all} khung, cuong do TB {s:.0f}) - ngoai tam/tin hieu yeu")
            else:
                print(f"  {ang:6.1f} do: {d:7.1f} cm | dao dong {spread:3.0f} cm | cuong do {s:6.0f} | {n_ok}/{n_all} khung")
            rows.append({"angle_deg": ang, "dist_cm": d, "spread_cm": spread, "strength": round(s, 1),
                         "n_ok": n_ok, "n_all": n_all})
            i += 1

    with open(a.out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["angle_deg", "dist_cm", "spread_cm", "strength", "n_ok", "n_all"])
        w.writeheader()
        w.writerows(rows)
    png = a.out.rsplit(".", 1)[0] + ".png"
    if any(r["dist_cm"] is not None for r in rows):
        plot(rows, png, a.rmax)
        print(f"\nDa luu {a.out} va {png}")
    else:
        print(f"\nDa luu {a.out} (khong co diem hop le de ve)")


if __name__ == "__main__":
    main()
