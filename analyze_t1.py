"""
analyze_t1.py - Phan tich thi nghiem T1: do chinh xac tinh theo khoang cach.

Dau vao: cac file CSV do tf03_tool.py (lenh log) tao ra, ten file chua khoang cach that,
vi du T1_050cm.csv, T1_100cm.csv, T1_200cm.csv ...
(khoang cach that do bang thuoc tu MAT KINH TRUOC cua TF03 den tuong)

Dau ra:
    - Bang ket qua in ra man hinh va luu T1_summary.csv
    - Hinh T1_result.png: (a) khoang cach do vs that + duong fit, (b) bias, (c) std va strength
    - He so fit: d_do = a * d_that + b
        a : he so ti le (ly tuong = 1)
        b : offset (cm) ~ khoang cach tu goc do noi bo cua cam bien den mat kinh

Chay:  python analyze_t1.py T1_*.csv
       (tren Windows PowerShell neu dau * khong tu mo rong: python analyze_t1.py)
"""
import csv
import glob
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_OF_RANGE_CM = 18000
STRENGTH_MIN = 40


def load(path):
    """Doc CSV cua tf03_tool: dong 1 la ghi chu, dong 2 la tieu de."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    note = rows[0][0] if rows and rows[0] and rows[0][0].startswith("#") else ""
    data = np.array([[float(x) for x in r] for r in rows[2:] if len(r) == 3])
    return note, data


def true_distance(path):
    m = re.search(r"(\d+(?:\.\d+)?)\s*cm", path, re.IGNORECASE)
    if not m:
        raise ValueError(f"Khong doc duoc khoang cach that tu ten file: {path} (can dang ..._123cm.csv)")
    return float(m.group(1))


def main(paths):
    paths = sorted(set(paths), key=true_distance)
    if not paths:
        sys.exit("Khong tim thay file. Vi du: python analyze_t1.py T1_050cm.csv T1_100cm.csv")

    rows = []
    for p in paths:
        note, data = load(p)
        t, d, s = data.T
        valid = (d < OUT_OF_RANGE_CM) & (s >= STRENGTH_MIN)
        dv = d[valid]
        rate = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 1 else float("nan")
        codes = np.unique(dv)
        rows.append(dict(
            file=p, d_true=true_distance(p), n=len(d), n_valid=int(valid.sum()),
            mean=dv.mean(), std=dv.std(ddof=1), dmin=dv.min(), dmax=dv.max(),
            n_codes=len(codes), strength=s[valid].mean(), rate=rate, note=note))

    d_true = np.array([r["d_true"] for r in rows])
    d_mean = np.array([r["mean"] for r in rows])
    a, b = np.polyfit(d_true, d_mean, 1) if len(rows) >= 2 else (np.nan, np.nan)
    resid = d_mean - (a * d_true + b)

    # ---- in bang ----
    print(f"{'d_that':>7} {'n_hl/n':>10} {'mean':>8} {'bias':>7} {'std':>6} {'min':>5} {'max':>5}"
          f" {'so_ma':>5} {'strength':>8} {'khung/s':>7}")
    for r in rows:
        print(f"{r['d_true']:7.1f} {r['n_valid']:5d}/{r['n']:<4d} {r['mean']:8.2f} "
              f"{r['mean'] - r['d_true']:7.2f} {r['std']:6.2f} {r['dmin']:5.0f} {r['dmax']:5.0f}"
              f" {r['n_codes']:5d} {r['strength']:8.0f} {r['rate']:7.1f}")
    print(f"\nFit: d_do = {a:.4f} * d_that + {b:.2f} cm")
    print(f"Sai lech sau fit (RMS): {np.sqrt(np.mean(resid ** 2)):.2f} cm")
    print("Luu y: neu 'so_ma' = 1-2 thi nhieu that nho hon do phan giai 1 cm, std khong phan anh nhieu that.")

    with open("T1_summary.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["d_true_cm", "n", "n_valid", "mean_cm", "bias_cm", "std_cm", "min_cm", "max_cm",
                    "n_codes", "strength_mean", "frame_rate", "resid_after_fit_cm", "file", "note"])
        for r, e in zip(rows, resid):
            w.writerow([r["d_true"], r["n"], r["n_valid"], f"{r['mean']:.3f}",
                        f"{r['mean'] - r['d_true']:.3f}", f"{r['std']:.3f}", r["dmin"], r["dmax"],
                        r["n_codes"], f"{r['strength']:.1f}", f"{r['rate']:.2f}", f"{e:.3f}",
                        r["file"], r["note"]])

    # ---- ve hinh ----
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.5))
    ax[0].plot(d_true, d_mean, "o", label="trung binh do")
    xx = np.linspace(0, d_true.max() * 1.05, 50)
    ax[0].plot(xx, xx, "--", color="0.6", label="ly tuong d_do = d_that")
    ax[0].plot(xx, a * xx + b, "-", label=f"fit: {a:.4f}·d + {b:.2f}")
    ax[0].set_xlabel("khoang cach that (cm)"); ax[0].set_ylabel("khoang cach do (cm)")
    ax[0].set_title("(a) Do so voi that"); ax[0].legend(fontsize=8)

    ax[1].errorbar(d_true, d_mean - d_true, yerr=[r["std"] for r in rows], fmt="o", capsize=4)
    ax[1].axhline(0, color="0.6", ls="--")
    ax[1].set_xlabel("khoang cach that (cm)"); ax[1].set_ylabel("bias = do - that (cm)")
    ax[1].set_title("(b) Sai so he thong (thanh = ±1 std)")

    ax[2].plot(d_true, [r["std"] for r in rows], "o-", label="std (cm)")
    ax[2].set_xlabel("khoang cach that (cm)"); ax[2].set_ylabel("std (cm)")
    ax2 = ax[2].twinx()
    ax2.plot(d_true, [r["strength"] for r in rows], "s--", color="C1", label="strength")
    ax2.set_ylabel("strength", color="C1")
    ax[2].set_title("(c) Do lap va cuong do tin hieu")
    for a_ in ax:
        a_.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig("T1_result.png", dpi=130)
    print("Da luu T1_summary.csv va T1_result.png")


if __name__ == "__main__":
    args = sys.argv[1:] or glob.glob("T1_*cm.csv")
    expanded = []
    for a_ in args:
        expanded += glob.glob(a_) or [a_]
    main(expanded)
