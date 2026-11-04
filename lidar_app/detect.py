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


@dataclass
class Obj:
    cx: float
    cy: float
    size: float
    n: int
    xs: np.ndarray = field(repr=False, default=None)
    ys: np.ndarray = field(repr=False, default=None)


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

    def foreground(self, ang_deg, r, margin):
        if self.bg is None:
            return np.ones(len(r), bool)
        b = (np.asarray(ang_deg) / self.bin_deg).astype(int) % self.nb
        return np.asarray(r) < self.bg[b] - margin


def cluster(ang, x, y, gap=0.2, min_pts=3, min_size=0.1):
    if len(x) == 0:
        return []
    o = np.argsort(ang)
    x, y = x[o], y[o]
    d = np.hypot(np.diff(x), np.diff(y))
    cuts = np.r_[0, np.where(d > gap)[0] + 1, len(x)]
    objs = []
    for a, b in zip(cuts[:-1], cuts[1:]):
        if b - a < min_pts:
            continue
        cx_, cy_ = x[a:b], y[a:b]
        size = float(math.hypot(np.ptp(cx_), np.ptp(cy_)))
        if size >= min_size or b - a >= 3 * min_pts:
            objs.append(Obj(float(cx_.mean()), float(cy_.mean()), size, b - a, cx_, cy_))
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
