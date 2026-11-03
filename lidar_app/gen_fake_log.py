"""
gen_fake_log.py - Tao thu muc log gia lap de thu app.py khi khong co phan cung.

Canh: phong 8 x 6 m, LiDAR dat lech tam, co 1 cot. Mot nguoi (tru tron ban kinh 0,2 m) di tu ngoai
vao vung giua phong o giay 10, dung den giay 18 roi di ra. Do tre USB ngau nhien 0-16 ms, dong ho UNO
nhanh 0,3 %, TF03 ~1025 Hz, nhieu khoang cach 1 cm.
Chay:  python gen_fake_log.py        -> tao thu muc log_gia_lap/
"""
import math, os

import numpy as np

rng = np.random.default_rng(7)
OUT = "log_gia_lap"
RPS, HZ, DUR = 1.0, 1025.0, 30.0
LID = (1.0, 0.5)                               # vi tri LiDAR trong phong
WALLS = (-3.0, 5.0, -2.5, 3.5)                 # xmin, xmax, ymin, ymax (toa do phong)
POLE = (3.0, 2.0, 0.08)


def person(t):
    """Vi tri nguoi theo thoi gian (hoac None)."""
    if t < 6 or t > 24:
        return None
    if t < 10:                                  # di vao tu ben phai
        return (4.6 - (t - 6) * 0.45, -1.5)
    if t < 18:
        return (2.8, -1.5)                       # dung trong vung cam
    return (2.8 + (t - 18) * 0.3, -1.5 - (t - 18) * 0.15)


def ray(px, py, th, circles):
    c, s = math.cos(th), math.sin(th)
    best = 1e9
    x0, x1, y0, y1 = WALLS
    for X in (x0, x1):
        if abs(c) > 1e-9:
            t = (X - px) / c
            if t > 0 and y0 <= py + t * s <= y1: best = min(best, t)
    for Y in (y0, y1):
        if abs(s) > 1e-9:
            t = (Y - py) / s
            if t > 0 and x0 <= px + t * c <= x1: best = min(best, t)
    for qx, qy, r in circles:
        fx, fy = px - qx, py - qy
        b = fx * c + fy * s
        disc = b * b - (fx * fx + fy * fy - r * r)
        if disc >= 0:
            t = -b - math.sqrt(disc)
            if t > 0: best = min(best, t)
    return best


def main():
    os.makedirs(OUT, exist_ok=True)
    ft = open(os.path.join(OUT, "tf03_raw.csv"), "w"); ft.write("n,dist_cm,strength,t_pc_s\n")
    fi = open(os.path.join(OUT, "index.csv"), "w"); fi.write("t_uno_s,t_pc_s\n")
    T0 = 1000.0                                  # dong ho PC bat dau tu 1000 s
    n, t, pend = 0, 0.0, []
    rows = []
    while t < DUR:
        circles = [POLE] + ([(*person(t), 0.2)] if person(t) else [])
        th = 2 * math.pi * RPS * t
        d = ray(*LID, th, circles) + rng.normal(0, 0.01)
        pend.append((n, int(round(d * 100)), int(rng.uniform(800, 1500))))
        n += 1; t += 1 / HZ
        if rng.random() < 0.15 or len(pend) >= 16:
            ta = T0 + t + rng.uniform(0, 0.016)
            rows += [(a, b, c, ta) for a, b, c in pend]; pend = []
    rows.sort(key=lambda r: r[3])
    for r in rows:
        ft.write("%d,%d,%d,%.6f\n" % r)
    k = 0
    while k / RPS < DUR:
        ti = k / RPS
        fi.write("%.6f,%.6f\n" % (5.0 + ti * 1.003, T0 + ti + 0.009 + rng.uniform(0, 0.003)))
        k += 1
    ft.close(); fi.close()
    print("Da tao", OUT, "| khung:", n, "| vong:", k)


if __name__ == "__main__":
    main()
