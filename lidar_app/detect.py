"""
detect.py - Phat hien vat can trong vung cam.

Cac buoc cho moi vong quet:
  1. Nen (background): LiDAR dung yen nen moi huong goc co mot khoang cach "binh thuong"
     (tuong, cot, rao). Hoc trong N vong: chia 360 do thanh cac o goc, lay trung vi khoang cach.
  2. Diem tien canh: diem gan hon nen it nhat 'margin' (m) -> vat moi xuat hien.
     O goc chua co nen (khong thay gi khi hoc) -> moi diem deu la tien canh.
  3. Giu diem tien canh nam trong vung cam (da giac).
  4. Gom cum theo thu tu goc: 2 diem lien tiep cach nhau > gap -> cum moi.
     Bo cum it diem hoac nho hon kich thuoc toi thieu.
  5. Chong bao dong gia: chi BAO DONG khi co vat hop le lien tiep 'on_revs' vong;
     het bao dong khi khong con vat 'off_revs' vong lien tiep.
"""
import math
import os
import time
from dataclasses import dataclass, field

import numpy as np


def point_in_polygon(x, y, poly):
    """Ray casting, x/y la mang. poly: list[(x, y)] >= 3 dinh."""
    x, y = np.asarray(x), np.asarray(y)
    inside = np.zeros(x.shape, bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % n]
        cond = (y1 > y) != (y2 > y)
        with np.errstate(divide="ignore", invalid="ignore"):
            xint = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        inside ^= cond & (x < xint)
    return inside


def transform_zone(zone, off_old, dir_old, off_new, dir_new):
    """Doi dinh vung cam khi doi Xoay / Dao chieu de vung van o dung vi tri that so voi LiDAR.
    Xoay va lat giu nguyen duong thang nen chi can doi cac dinh. r0 khong lam vung thay doi."""
    out = []
    for x, y in zone:
        r, th = math.hypot(x, y), math.degrees(math.atan2(y, x))
        th = dir_new * dir_old * (th - off_old) + off_new     # th = dir * goc tho + off
        out.append((r * math.cos(math.radians(th)), r * math.sin(math.radians(th))))
    return out


@dataclass
class Obj:
    cx: float
    cy: float
    size: float
    n: int
    xs: np.ndarray = field(repr=False, default=None)
    ys: np.ndarray = field(repr=False, default=None)
    a0: float = 0.0                     # goc tho nho nhat cua cum (do), de tinh thoi diem tia cham vat


class Background:
    def __init__(self, bin_deg=0.5):
        self.nb = int(round(360 / bin_deg))
        self.bin_deg = bin_deg
        self.samples = []
        self.bg = None                      # khoang cach nen (m) theo o goc, inf = trong
        self.learning_left = 0

    def start_learning(self, n_revs=20):
        self.samples, self.learning_left, self.bg = [], n_revs, None

    def feed(self, ang_deg, r):
        """Cho 1 vong vao khi dang hoc. Tra ve True khi vua hoc xong."""
        if self.learning_left <= 0:
            return False
        b = (np.asarray(ang_deg) / self.bin_deg).astype(int) % self.nb
        row = np.full(self.nb, np.nan)
        order = np.argsort(r)[::-1]                     # neu nhieu diem cung o: giu diem gan nhat
        row[b[order]] = np.asarray(r)[order]
        self.samples.append(row)
        self.learning_left -= 1
        if self.learning_left == 0:
            S = np.array(self.samples)
            with np.errstate(all="ignore"):
                bg = np.nanmedian(S, axis=0)
            valid = np.sum(~np.isnan(S), axis=0) >= max(2, len(S) // 3)
            self.bg = np.where(valid, bg, np.inf)
            # lap lo trong giua cac o co nen: lay min hai ben (tranh bao gia o mep vat)
            self.bg = np.minimum(self.bg, np.minimum(np.roll(self.bg, 1), np.roll(self.bg, -1)))
            return True
        return False

    def ready(self):
        return self.bg is not None

    def save(self, path, rps, r0):
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            np.savez(f, bg=self.bg, bin_deg=self.bin_deg, rps=rps, r0=r0, t=time.time())
        os.replace(tmp, path)                           # app chet giua chung khong lam hong file cu

    @staticmethod
    def read(path):
        with np.load(path) as d:
            return dict(bg=d["bg"], bin_deg=float(d["bin_deg"]), rps=float(d["rps"]), r0=float(d["r0"]), t=float(d["t"]))

    def apply(self, data, r0_now):
        """Dung nen da doc. Nen luu khoang cach da tru r0 nen phai bu neu r0 da doi."""
        if data["bin_deg"] != self.bin_deg or len(data["bg"]) != self.nb:
            raise ValueError("File nền không đúng độ phân giải góc.")
        self.bg, self.samples, self.learning_left = data["bg"] + data["r0"] - r0_now, [], 0

    def foreground(self, ang_deg, r, margin):
        if self.bg is None:
            return np.ones(len(r), bool)
        b = (np.asarray(ang_deg) / self.bin_deg).astype(int) % self.nb
        return np.asarray(r) < self.bg[b] - margin


def cluster(ang, x, y, gap=0.2, min_pts=3, min_size=0.1):
    if len(x) == 0:
        return []
    o = np.argsort(ang)
    a_, x, y = np.asarray(ang)[o], x[o], y[o]
    d = np.hypot(np.diff(x), np.diff(y))
    cuts = np.r_[0, np.where(d > gap)[0] + 1, len(x)]
    objs = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b - a < min_pts:
            continue
        cx_, cy_ = x[a:b], y[a:b]
        size = float(math.hypot(np.ptp(cx_), np.ptp(cy_)))
        if size >= min_size or b - a >= 3 * min_pts:
            objs.append(Obj(float(cx_.mean()), float(cy_.mean()), size, b - a, cx_, cy_, float(a_[a])))
    return objs


class Detector:
    def __init__(self):
        self.zone = []                  # da giac [(x, y)]
        self.bg = Background()
        self.margin = 0.15              # m
        self.gap = 0.2
        self.min_pts = 3
        self.min_size = 0.10
        self.on_revs, self.off_revs = 3, 3
        self.use_bg = True
        self.alarm = False
        self._on = self._off = 0
        self.events = []                # (thoi gian, kieu, mo ta)

    def step(self, xy):
        """xy: dict tu core.to_xy. Tra ve (objs, mask_fg, alarm_moi_bat)."""
        x, y, r, ang = xy["x"], xy["y"], xy["r"], xy["ang"]
        if self.bg.learning_left > 0:
            done = self.bg.feed(ang, r)
            if done:
                self.events.append((time.time(), "INFO", "Học nền xong"))
            return [], np.zeros(len(x), bool), False
        if self.use_bg and not self.bg.ready():          # chua hoc nen -> khong phan biet duoc tuong voi vat
            self.alarm, self._on, self._off = False, 0, 0
            return [], np.zeros(len(x), bool), False
        fg = self.bg.foreground(ang, r, self.margin) if self.use_bg else np.ones(len(x), bool)
        if len(self.zone) >= 3:
            fg &= point_in_polygon(x, y, self.zone)
        else:
            fg &= False                                  # chua co vung cam -> khong bao dong
        objs = cluster(ang[fg], x[fg], y[fg], self.gap, self.min_pts, self.min_size)
        newly = False
        if objs:
            self._on += 1; self._off = 0
            if not self.alarm and self._on >= self.on_revs:
                self.alarm, newly = True, True
                big = max(objs, key=lambda o: o.size)
                self.events.append((time.time(), "BÁO ĐỘNG",
                                    "%d vật | lớn nhất %.2f m tại (%.2f, %.2f)" % (len(objs), big.size, big.cx, big.cy)))
        else:
            self._off += 1; self._on = 0
            if self.alarm and self._off >= self.off_revs:
                self.alarm = False
                self.events.append((time.time(), "HẾT", "Vùng cấm trống"))
        return objs, fg, newly


class AlarmLog:
    """Ghi moc thoi gian tung bao dong ra CSV de do tre tung khau. Cac moc t_* cung dong ho voi t_pc trong tf03_raw.csv.

    t_first_hit : tia cham vat o vong dau cua chuoi vong lien tiep (uoc luong tu goc: t moc + a0/360 * T)
    t_scan_end  : het vong gay bao dong (moc 0 do tiep theo)
    t_ready     : Mapper tra ve vong do
    t_alarm     : Detector bat bao dong
    t_snap      : xong beep + luu anh
    t_shown     : da ve lai o trang thai mau do
    t_wall      : gio he thong (s), de doi chieu voi video
    Khong do duoc luc vat thuc su vao vung: khoang do can camera hoac cam bien ngoai.
    """
    HEAD = ("kieu,seq,seq_first,n_revs,on_revs,T_s,t_first_hit,t_scan_end,t_ready,t_alarm,t_snap,t_shown,"
            "tre_hit_den_shown_s,t_wall,cx_m,cy_m,size_m,n_diem\n")

    def __init__(self, path):
        self.path, self.run, self.row, self.was_alarm = path, None, None, False

    def _write(self, line):
        new = not os.path.exists(self.path)
        with open(self.path, "a", encoding="utf-8") as f:   # mo-dong moi dong: app chet giua chung van con log
            f.write((self.HEAD if new else "") + line)

    def on_scan(self, sc, objs, newly, det, t_ready, t_alarm):
        if objs and self.run is None:                       # vong dau cua chuoi lien tiep (cung logic _on cua Detector)
            self.run = (sc.seq, sc.t + min(o.a0 for o in objs) / 360.0 * sc.period)
        elif not objs:
            self.run = None
        if newly and self.run:
            big = max(objs, key=lambda o: o.size)
            self.row = dict(sc=sc, on=det.on_revs, first=self.run, t_ready=t_ready, t_alarm=t_alarm, t_snap=t_alarm, big=big)
        elif self.was_alarm and not det.alarm:
            self._write("HET,%d,,,,%.6f,,%.6f,%.6f,%.6f,,,,%.3f,,,,\n" % (
                sc.seq, sc.period, sc.t + sc.period, t_ready, t_alarm, time.time()))
        self.was_alarm = det.alarm

    def mark_snap(self, t):
        if self.row:
            self.row["t_snap"] = t

    def finish(self, t_shown):
        """Goi sau khi da ve o trang thai; tra ve do tre tu luc tia cham vat (s) hoac None."""
        r, self.row = self.row, None
        if r is None:
            return None
        sc, big, (seq0, t_hit) = r["sc"], r["big"], r["first"]
        self._write("BAO_DONG,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n" % (
            sc.seq, seq0, sc.seq - seq0 + 1, r["on"], sc.period, t_hit, sc.t + sc.period, r["t_ready"], r["t_alarm"],
            r["t_snap"], t_shown, t_shown - t_hit, time.time(), big.cx, big.cy, big.size, big.n))
        return t_shown - t_hit
