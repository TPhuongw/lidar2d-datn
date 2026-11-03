"""
plot_scan.py - Ve lai anh tu file CSV quet tay (khong can ket noi TF03).
Chay:  python plot_scan.py quet_tay_1.csv --rmax 10
"""
import argparse
import csv
from tf03_manual_scan import plot

ap = argparse.ArgumentParser()
ap.add_argument("csv")
ap.add_argument("--rmax", type=float, default=15.0, help="chi ve diem gan hon rmax (m)")
a = ap.parse_args()
with open(a.csv, encoding="utf-8") as f:
    rows = list(csv.DictReader(f))
png = a.csv.rsplit(".", 1)[0] + f"_r{a.rmax:g}.png"
plot(rows, png, a.rmax)
print(f"Da luu {png}")
