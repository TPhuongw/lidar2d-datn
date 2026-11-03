"""
calib_2lidar.py - Hieu chuan khong gian giua 2 LiDAR bang cot moc chung (phuong an 2.b).

Y tuong:
  1. Dat 3 cot tron (ong nhua, xo...) ban kinh R biet truoc, ca 2 LiDAR cung nhin thay.
  2. Moi LiDAR: tach cum diem -> giu cum co do rong hop voi cot -> khop duong tron ban kinh R
     -> tam cot trong he toa do cua LiDAR do.
  3. Ghep cot cua A voi cot cua B bang khoang cach doi mot giua cac cot (khong doi khi xoay/dich).
  4. Giai phep bien doi cung (xoay theta + dich x, y) dua tam cot cua B trung tam cot cua A
     (thuat toan Kabsch 2D, binh phuong toi thieu). Ket qua = vi tri va huong cua LiDAR B trong he A.

Dau vao: file CSV luu bang phim 's' cua lidar_map_pc.py (cot x_m, y_m), co the nhieu file / LiDAR
         (nhieu vong -> tam cot trung binh, it nhieu hon).
Chay:
  python calib_2lidar.py --a vongA_*.csv --b vongB_*.csv --R 0.055
  python calib_2lidar.py --demo          (mo phong, khong can phan cung)
Ket qua ghi ra calib_B_in_A.json: {"x": .., "y": .., "theta_deg": .., "rms_m": ..}
CODE THU NGHIEM: da thu bang du lieu mo phong, CHUA thu voi du lieu that.
"""
import argparse, glob, itertools, json, math

import numpy as np


# ------------------------------------------------------------ tim cot trong 1 vong quet
def split_clusters(x, y, gap=0.08):
    """Tach diem theo thu tu goc thanh cac cum; 2 diem lien tiep cach nhau > gap (m) -> cum moi."""
    ang = np.arctan2(y, x)
    o = np.argsort(ang)
    x, y = x[o], y[o]
    d = np.hypot(np.diff(x), np.diff(y))
    cuts = np.where(d > gap)[0] + 1
    return [(x[a:b], y[a:b]) for a, b in zip(np.r_[0, cuts], np.r_[cuts, len(x)])]


def fit_circle_known_r(px, py, R, iters=20):
    """Khop duong tron ban kinh R biet truoc (Gauss-Newton). LiDAR o goc toa do nhin vao mat truoc cot."""
    mx, my = px.mean(), py.mean()
    n = math.hypot(mx, my)
    cx, cy = mx + R * mx / n, my + R * my / n          # tam nam sau mat cot, theo huong tia
    for _ in range(iters):
        dx, dy = px - cx, py - cy
        rr = np.hypot(dx, dy) + 1e-12
        res = rr - R
        J = np.c_[-dx / rr, -dy / rr]
        step = np.linalg.lstsq(J, -res, rcond=None)[0]
        cx, cy = cx + step[0], cy + step[1]
        if np.hypot(*step) < 1e-6:
            break
    rms = float(np.sqrt(np.mean((np.hypot(px - cx, py - cy) - R) ** 2)))
    return cx, cy, rms


def find_poles(x, y, R, min_pts=4, max_rms=0.03):
    """Tra ve list (cx, cy, so diem, rms) cho cac cum giong cot ban kinh R."""
    poles = []
    for cx_, cy_ in split_clusters(x, y):
        if len(cx_) < min_pts:
            continue
        chord = math.hypot(cx_[-1] - cx_[0], cy_[-1] - cy_[0])
        if not (0.3 * R <= chord <= 2.4 * R):           # rong qua (tuong) hoac hep qua (nhieu)
            continue
        c = fit_circle_known_r(cx_, cy_, R)
        if c[2] <= max_rms:
            poles.append((c[0], c[1], len(cx_), c[2]))
    return poles


def average_poles(scans, R, merge=0.15):
    """Gop cot tim duoc qua nhieu vong (cach nhau < merge m la cung 1 cot), lay trung binh tam."""
    groups = []
    for x, y in scans:
        for cx, cy, n, _ in find_poles(x, y, R):
            for g in groups:
                if math.hypot(g["x"][0] - cx, g["y"][0] - cy) < merge:
                    g["x"].append(cx); g["y"].append(cy); break
            else:
                groups.append(dict(x=[cx], y=[cy]))
    need = max(1, len(scans) // 2)                       # phai thay o it nhat nua so vong
    out = [(np.mean(g["x"]), np.mean(g["y"]), np.std(g["x"]), np.std(g["y"]), len(g["x"]))
           for g in groups if len(g["x"]) >= need]
    return out


# ------------------------------------------------------------ ghep cot va giai bien doi
def kabsch2d(P, Q):
    """Tim R, t sao cho R @ q + t ~ p (P, Q: Nx2, cung thu tu). Tra ve theta (rad), t, rms."""
    pc, qc = P.mean(0), Q.mean(0)
    H = (Q - qc).T @ (P - pc)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1, np.sign(np.linalg.det(Vt.T @ U.T))])  # loai phep doi xung guong
    Rm = Vt.T @ D @ U.T
    t = pc - Rm @ qc
    rms = float(np.sqrt(np.mean(np.sum((P - (Q @ Rm.T + t)) ** 2, 1))))
    return math.atan2(Rm[1, 0], Rm[0, 0]), t, rms


def match_and_solve(PA, PB):
    """Thu moi cach ghep k cot cua B voi k cot cua A, chon cach co sai so nho nhat."""
    PA, PB = np.asarray(PA, float), np.asarray(PB, float)
    k = min(len(PA), len(PB))
    if k < 2:
        raise SystemExit("Can it nhat 3 cot ca hai LiDAR cung thay.")
    if k == 2:
        print("CANH BAO: chi co 2 cot chung -> co 2 nghiem doi nhau 180 do, ket qua co the sai. "
              "Can 3 cot, dat khong thang hang, khoang cach doi mot khac nhau.")
    best = None
    for ia in itertools.combinations(range(len(PA)), k):
        for ib in itertools.permutations(range(len(PB)), k):
            th, t, rms = kabsch2d(PA[list(ia)], PB[list(ib)])
            if best is None or rms < best[2]:
                best = (th, t, rms, ia, ib)
    return best


# ------------------------------------------------------------ doc file / mo phong
def load_scans(pattern):
    scans = []
    for f in sorted(glob.glob(pattern)):
        a = np.genfromtxt(f, delimiter=",", names=True)
        scans.append((np.asarray(a["x_m"]), np.asarray(a["y_m"])))
    if not scans:
        raise SystemExit("Khong tim thay file: " + pattern)
    return scans


def simulate(pose_b, poles_world, R, n_rev=10, pts=1000, noise=0.02, seed=0):
    """Tao vong quet gia lap cho LiDAR o pose (x, y, theta): phong 12 x 8 m + cac cot."""
    rng = np.random.default_rng(seed)
    px, py, th = pose_b
    walls = [(-4, None), (8, None), (None, -3), (None, 5)]
    scans = []
    for _ in range(n_rev):
        a = np.linspace(0, 2 * math.pi, pts, endpoint=False)
        xs, ys = [], []
        for al in a:
            wa = al + th                                   # huong tia trong he the gioi
            c, s = math.cos(wa), math.sin(wa)
            best = 1e9
            for X, Y in walls:
                if X is not None and abs(c) > 1e-9:
                    t = (X - px) / c
                    if t > 0 and -3 <= py + t * s <= 5: best = min(best, t)
                if Y is not None and abs(s) > 1e-9:
                    t = (Y - py) / s
                    if t > 0 and -4 <= px + t * c <= 8: best = min(best, t)
            for qx, qy in poles_world:                     # giao tia voi duong tron
                fx, fy = px - qx, py - qy
                bq = fx * c + fy * s
                disc = bq * bq - (fx * fx + fy * fy - R * R)
                if disc >= 0:
                    t = -bq - math.sqrt(disc)
                    if t > 0: best = min(best, t)
            r = best + rng.normal(0, noise)
            xs.append(r * math.cos(al)); ys.append(r * math.sin(al))
        scans.append((np.array(xs), np.array(ys)))
    return scans


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--a"); p.add_argument("--b")
    p.add_argument("--R", type=float, default=0.055, help="ban kinh cot (m), vd ong PVC phi 110 -> 0.055")
    p.add_argument("--min-pts", type=int, default=4)
    p.add_argument("--demo", action="store_true")
    args = p.parse_args()

    if args.demo:
        poles = [(2.0, 2.5), (4.5, -1.0), (1.0, -1.8)]
        truth = (6.0, 1.5, math.radians(150.0))            # LiDAR B dat o (6; 1,5), xoay 150 do
        scA = simulate((0, 0, 0), poles, args.R)
        scB = simulate(truth, poles, args.R, seed=1)
        print("MO PHONG: LiDAR B that o x=6.000 y=1.500 theta=150.00 do, nhieu do xa 2 cm")
    else:
        scA, scB = load_scans(args.a), load_scans(args.b)

    PA, PB = average_poles(scA, args.R), average_poles(scB, args.R)
    for name, P in (("A", PA), ("B", PB)):
        print("LiDAR %s thay %d cot:" % (name, len(P)))
        for cx, cy, sx, sy, n in P:
            print("   tam (%.3f, %.3f) m | do lech chuan (%.3f, %.3f) | %d vong" % (cx, cy, sx, sy, n))
    th, t, rms, ia, ib = match_and_solve([q[:2] for q in PA], [q[:2] for q in PB])
    print("\nKET QUA: LiDAR B trong he toa do A: x=%.3f m, y=%.3f m, theta=%.2f do | sai so khop RMS %.1f mm"
          % (t[0], t[1], math.degrees(th), rms * 1000))
    print("Ghep cot (A <- B):", list(zip(ia, ib)))
    json.dump(dict(x=float(t[0]), y=float(t[1]), theta_deg=math.degrees(th), rms_m=rms),
              open("calib_B_in_A.json", "w"), indent=2)
    print("Da ghi calib_B_in_A.json. Diem cua B doi sang he A: [x_A, y_A] = Rot(theta) @ [x_B, y_B] + [x, y]")


if __name__ == "__main__":
    main()
