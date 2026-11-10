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


class SpeedGate:
    """Chi coi la on dinh khi n vong lien tiep co chu ky lech nhau duoi tol, va lech chu ky luc hoc nen duoi tol.
    Luc dong co tang toc, doi toc do hoac bi vuong, goc tinh theo thoi gian bi sai nen khong duoc phat hien va hoc nen.
    Log 17:14: binh thuong chu ky chi dao dong 0,02 %; chu ky lech 0,5 % da lam 23 % so diem khac nen."""

    def __init__(self, n=3, tol=0.003, rot_max=5.0):
        self.n, self.tol, self.rot_max = n, tol, rot_max
        self.reset()

    def reset(self):
        self.hist, self.ok, self.bg_mismatch, self.sync_bad, self.rot, self.steady = [], False, False, False, None, False

    def update(self, period, bg_period=None, sync_ok=True, rot_deg=None):
        """sync_ok=False: lau khong nhan duoc moc 0 do dung gio. rot_deg: vong quet xoay so voi nen (do).
        Ca hai deu nghia la goc dang sai -> khong phat hien (vat gia o mep tuong)."""
        self.hist = (self.hist + [period])[-self.n:]
        steady = len(self.hist) == self.n and (max(self.hist) - min(self.hist)) <= self.tol * min(self.hist)
        self.bg_mismatch = bg_period is not None and abs(period - bg_period) > self.tol * bg_period
        self.sync_bad = not sync_ok
        self.rot = rot_deg if rot_deg is not None and abs(rot_deg) > self.rot_max else None
        self.steady = steady and sync_ok and not self.bg_mismatch     # goc dang tin duoc ve mat thoi gian
        self.ok = self.steady and self.rot is None
        return self.ok


def poly_edge_dist(x, y, poly):
    """Khoang cach tu diem toi canh gan nhat cua da giac (m)."""
    best = math.inf
    for (x1, y1), (x2, y2) in zip(poly, list(poly[1:]) + [poly[0]]):
        dx, dy = x2 - x1, y2 - y1
        u = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / max(dx * dx + dy * dy, 1e-12)))
        best = min(best, math.hypot(x - x1 - u * dx, y - y1 - u * dy))
    return best


def polygon_area(poly):
    """Dien tich da giac (cong thuc Gauss), m2. Chi dung voi da giac khong tu cat."""
    return 0.5 * abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, list(poly[1:]) + [poly[0]])))


def polygon_self_intersects(poly):
    """True neu hai canh khong ke nhau cat nhau."""
    side = lambda u, v, w: (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        for j in range(i + 2, n):
            if i == 0 and j == n - 1:                   # hai canh chung dinh dau
                continue
            p, q = poly[j], poly[(j + 1) % n]
            if side(a, b, p) * side(a, b, q) < 0 and side(p, q, a) * side(p, q, b) < 0:
                return True
    return False


def visible_area(poly, ang_deg, r, off_deg, direction, step=0.02):
    """Phan da giac LiDAR nhin thay duoc: diem gan hon duong bao (ang_deg, r) theo huong cua no. Tinh bang luoi step (m), m2."""
    xs, ys = zip(*poly)
    gx, gy = np.meshgrid(np.arange(min(xs) + step / 2, max(xs), step), np.arange(min(ys) + step / 2, max(ys), step))
    gx, gy = gx.ravel(), gy.ravel()
    k = point_in_polygon(gx, gy, poly)
    gx, gy = gx[k], gy[k]
    raw = direction * (np.degrees(np.arctan2(gy, gx)) - off_deg) % 360.0
    return float(np.sum(np.hypot(gx, gy) < np.interp(raw, ang_deg, r, period=360.0))) * step * step


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
    ang: np.ndarray = field(repr=False, default=None)
    gap: float = math.inf               # trung vi (nen - khoang cach): cum nam truoc nen bao xa (m)


class Background:
    def __init__(self, bin_deg=0.5):
        self.nb = int(round(360 / bin_deg))
        self.bin_deg = bin_deg
        self.samples = []
        self.bg = None                      # khoang cach nen (m) theo o goc, inf = trong
        self.learning_left = 0
        self.t_learn, self.rps = None, None # hoc luc nao, o toc do nao (de ghi vao session.json)

    def start_learning(self, n_revs=20):
        self.samples, self.learning_left, self.bg = [], n_revs, None
        self.t_learn, self.rps = None, None

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
            self.t_learn = time.time()
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
        self.t_learn, self.rps = data["t"], data["rps"]

    def rotation(self, ang_deg, r, max_deg=60.0, min_frac=0.3):
        """Uoc luong vong quet bi xoay bao nhieu do so voi nen (trung vi |r - nen| nho nhat theo do xoay).
        Dung de phat hien goc sai do dong bo thoi gian, khong dung de sua goc. Tra ve (do, sai so trung vi m) hoac None."""
        if self.bg is None:
            return None
        b = (np.asarray(ang_deg) / self.bin_deg).astype(int) % self.nb
        row = np.full(self.nb, np.nan)
        order = np.argsort(r)[::-1]
        row[b[order]] = np.asarray(r)[order]
        bg = np.where(np.isfinite(self.bg), self.bg, np.nan)
        ks = np.arange(-int(max_deg / self.bin_deg), int(max_deg / self.bin_deg) + 1)
        sh = row[(np.arange(self.nb)[None, :] - ks[:, None]) % self.nb]      # sh[i] = row xoay ks[i] o
        d = np.abs(sh - bg[None, :])
        cnt = np.sum(np.isfinite(d), axis=1)
        if cnt.max() < min_frac * self.nb:
            return None
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            score = np.nanmedian(d, axis=1)
        score[cnt < min_frac * self.nb] = np.inf
        i = int(np.argmin(score))
        k = float(ks[i])
        if 0 < i < len(ks) - 1 and np.isfinite(score[i - 1]) and np.isfinite(score[i + 1]):
            den = score[i - 1] - 2 * score[i] + score[i + 1]      # noi suy parabol: duoi nua o, khong lech ve phia am khi hai o bang diem
            if den > 1e-12:
                k += float(np.clip(0.5 * (score[i - 1] - score[i + 1]) / den, -0.5, 0.5))
        return k * self.bin_deg, float(score[i])

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
            objs.append(Obj(float(cx_.mean()), float(cy_.mean()), size, b - a, cx_, cy_, float(a_[a]), a_[a:b]))
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
        self.all_objs = []              # cum khac nen tren toan phong (ca ngoai vung cam), de theo doi
        self.hold = False               # True: con vat co the dang khuat duoi mat quet, chua duoc het bao dong

    def step(self, xy):
        """xy: dict tu core.to_xy. Tra ve (objs, mask_fg, alarm_moi_bat)."""
        x, y, r, ang = xy["x"], xy["y"], xy["r"], xy["ang"]
        self.all_objs = []
        if self.bg.learning_left > 0:
            done = self.bg.feed(ang, r)
            if done:
                self.events.append((time.time(), "INFO", "Học nền xong"))
            return [], np.zeros(len(x), bool), False
        if self.use_bg and not self.bg.ready():          # chua hoc nen -> khong phan biet duoc tuong voi vat
            self.alarm, self._on, self._off = False, 0, 0
            return [], np.zeros(len(x), bool), False
        fg = self.bg.foreground(ang, r, self.margin) if self.use_bg else np.ones(len(x), bool)
        if self.use_bg:                                  # khong co nen thi tuong cung la cum, khong theo doi duoc
            self.all_objs = cluster(ang[fg], x[fg], y[fg], self.gap, self.min_pts, self.min_size)
            for o in self.all_objs:
                b = (o.ang / self.bg.bin_deg).astype(int) % self.bg.nb
                o.gap = float(np.median(self.bg.bg[b] - np.hypot(o.xs, o.ys)))
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
            if self.alarm and self._off >= self.off_revs and not self.hold:
                self.alarm = False
                self.events.append((time.time(), "HẾT", "Vùng cấm trống"))
        return objs, fg, newly


@dataclass
class Track:
    id: int
    t: list
    x: list
    y: list
    size: float
    in_zone: bool
    hits: int = 1
    vx: float = 0.0
    vy: float = 0.0
    speed: float = None                 # m/s, None khi chua du du lieu
    still_since: float = None           # thoi diem bat dau dung yen
    zone_since: float = None            # thoi diem vao vung cam
    merged: int = 0                     # so doi tuong khac da gop vao
    absorb_t: float = None              # lan cuoi con gom them cum khac
    v_t0: float = None                  # chi tinh toc do tu sau moc nay (tam nhay khi vua gop)
    t0: float = None                    # thoi diem thay lan dau
    gaps: list = field(default_factory=list)   # vai gia tri gap gan nhat
    lost_since: float = None            # thoi diem lan cuoi thay, khi dang giu vi bien mat giua vung
    lost_s: float = 0.0                 # da mat tin hieu bao lau
    moving: bool = False                # dang di chuyen (nguong bat move_v, nguong tat move_v / 2)
    entry: dict = None                  # luot vao vung dang mo


class Tracker:
    """Noi cum giua cac vong thanh doi tuong: toc do, huong, thoi gian dung, quy dao.

    Moi cum co thoi diem rieng (t moc + a0/360 * T) vi tia quet toi moi huong vao mot luc khac nhau.
    Khong anh huong logic bao dong cua Detector.
    Tam cum la tam mat nhin thay, khong phai tam vat. Toc do chua duoc doi chieu voi phep do doc lap.

    Gop doi tuong: 2 doi tuong gan nhau (< merge_d) va cung van toc (lech < merge_dv) lien tuc merge_s giay
    thi tinh la mot, giu so thu tu cu hon. Sau khi gop, cum nam gan (< 1,5 * merge_d) duoc gom vao doi tuong
    do; cum di xa hon thi tach thanh doi tuong moi. merge_d = 0 la tat.

    Giu doi tuong: LiDAR chi quet mot mat phang nen vat thap xuong (cui, nga) la khong thay nua. Doi tuong da
    ton tai du lau ma bien mat giua vung cam (khong sat mep, khong dang di ra, nam truoc nen ro rang) duoc giu
    toi da hold_s giay. Xuat hien lai trong ban kinh reacq_r thi noi lai, giu so thu tu va thoi gian dung.
    Day chi la suy luan "co the van con do", LiDAR khong thay duoc vat. hold_s = 0 la tat.
    """

    def __init__(self, gate=0.4, vmax=3.0, merge=0.35, miss_s=1.5, confirm=3, win_s=2.0, still_r=0.15, still_v=0.1, hist_s=30.0,
                 merge_d=0.6, merge_dv=0.25, merge_s=1.5, absorb_s=3.0,
                 hold_s=60.0, reacq_r=0.6, min_life=3.0, edge_m=0.5, min_gap=0.5, move_v=0.2, ghost_s=3.0):
        self.gate, self.vmax, self.merge = gate, vmax, merge    # m, m/s, m
        self.merge_d, self.merge_dv, self.merge_s, self.absorb_s = merge_d, merge_dv, merge_s, absorb_s
        self.hold_s, self.reacq_r, self.min_life, self.edge_m, self.min_gap = hold_s, reacq_r, min_life, edge_m, min_gap
        self.events = []                                        # (kieu, id, t, x, y, mo ta), giao dien lay roi xoa
        self.move_v = move_v                                    # m/s
        self.ghost_s = ghost_s                                  # doi tuong moi sinh trong bay nhieu s co the la phan tach cua vat khac
        self.entries, self.closed = [], []                      # moi luot vao vung; cac luot vua dong, giao dien lay roi xoa
        self.miss_s, self.confirm = miss_s, confirm             # bo sau bay nhieu s khong thay; xac nhan sau bay nhieu lan
        self.win_s, self.still_r, self.still_v, self.hist_s = win_s, still_r, still_v, hist_s
        self.reset()

    def reset(self, keep_entries=False):
        if keep_entries:
            self.close_open()
        else:
            self.entries, self.closed = [], []
        self.tracks, self.next_id, self._pairs = [], getattr(self, "next_id", 1) if keep_entries else 1, {}

    def _open_entry(self, tr):
        tr.entry = dict(id=tr.id, t0=tr.t0 if tr.t0 is not None else tr.t[0], t_in=tr.zone_since if tr.zone_since is not None else tr.t[0],
                        t_out=None, t_last=None, x_in=tr.x[-1], y_in=tr.y[-1], x_last=tr.x[-1], y_last=tr.y[-1],
                        v_in=None, v_max=0.0, v_sum=0.0, v_n=0, size=tr.size, n_lost=0, t_lost=None, note="")
        self.entries.append(tr.entry)

    def _touch_entry(self, tr):
        """Cap nhat luot dang mo bang phep do moi nhat cua doi tuong."""
        e = tr.entry
        if e is None or e["t_last"] == tr.t[-1]:
            return
        e.update(t_last=tr.t[-1], x_last=tr.x[-1], y_last=tr.y[-1], size=max(e["size"], tr.size))
        if tr.speed is not None:
            if e["v_in"] is None:
                e["v_in"] = tr.speed
            e["v_max"] = max(e["v_max"], tr.speed); e["v_sum"] += tr.speed; e["v_n"] += 1

    def _close_entry(self, tr, note=""):
        if tr.entry is not None:
            self._touch_entry(tr)
            tr.entry.update(t_out=tr.t[-1], note=note)
            self.closed.append(tr.entry)
            tr.entry = None

    def close_open(self, note="Dừng đo"):
        """Dong moi luot dang mo (dung do, doi Xoay)."""
        for tr in getattr(self, "tracks", []):
            self._close_entry(tr, note)

    def confirmed(self):
        return [tr for tr in self.tracks if tr.hits >= self.confirm]

    def visible(self):
        return [tr for tr in self.confirmed() if tr.lost_since is None]

    def lost_in_zone(self):
        return [tr for tr in self.confirmed() if tr.lost_since is not None and tr.in_zone]

    def _can_hold(self, tr, zone):
        """Bien mat luc nay co dang ngo khong: da xac nhan, du lau, trong vung, xa mep, khong dang di ra, khong sat nen."""
        if self.hold_s <= 0 or tr.hits < self.confirm or not tr.in_zone or len(zone) < 3 or tr.t0 is None:
            return False
        if tr.t[-1] - tr.t0 < self.min_life or (tr.gaps and float(np.median(tr.gaps)) < self.min_gap):
            return False
        x, y = tr.x[-1], tr.y[-1]
        if poly_edge_dist(x, y, zone) < self.edge_m:
            return False
        return bool(point_in_polygon(np.array([x + tr.vx]), np.array([y + tr.vy]), zone)[0])   # sau 1 s van trong vung

    def hold_active(self, zone):
        """Con doi tuong dang duoc giu, hoac neu bien mat ngay thi se duoc giu. Detector khong duoc het bao dong."""
        return any(tr.lost_since is not None and tr.in_zone or self._can_hold(tr, zone) for tr in self.confirmed())

    def clear_lost(self):
        """Nguoi van hanh xac nhan vung trong: bo cac doi tuong dang giu."""
        for tr in self.lost_in_zone():
            self.events.append(("HET_GIU", tr.id, tr.t[-1], tr.x[-1], tr.y[-1], "#%d: người vận hành xác nhận vùng trống" % tr.id))
            self._close_entry(tr, "Xác nhận trống")
        keep = set(id(tr) for tr in self.lost_in_zone())
        self.tracks = [tr for tr in self.tracks if id(tr) not in keep]

    def _measure(self, sc, objs):
        """Gop cum sat nhau (mot vat bi tach, vi du tay va than) thanh mot phep do (t, x, y, size, xs, ys)."""
        groups = []
        for o in objs:
            hit = [g for g in groups if any(np.min(np.hypot(p.xs[:, None] - o.xs, p.ys[:, None] - o.ys)) < self.merge for p in g)]
            for g in hit:
                groups.remove(g)
            groups.append(sum(hit, []) + [o])
        out = []
        for g in groups:
            xs, ys = np.concatenate([o.xs for o in g]), np.concatenate([o.ys for o in g])
            a0 = min(o.a0 for o in g)
            out.append(self._meas(sc.t + a0 / 360.0 * sc.period, xs, ys, max(o.gap for o in g)))
        return out

    @staticmethod
    def _meas(t, xs, ys, gap):
        return t, float(xs.mean()), float(ys.mean()), float(math.hypot(np.ptp(xs), np.ptp(ys))), xs, ys, gap

    def _find_split(self, tr):
        """tr vua het miss_s. Tim doi tuong dang thay ngay canh vi tri cuoi cua tr ma mot trong hai moi sinh: mot vat bi tach
        cum roi cum phu nhan vi tri cua doi tuong cu. Hai doi tuong da song lau canh nhau thi khong tinh, de van giu vat cui."""
        t_end = tr.t[-1]
        for o in self.tracks:
            if o is tr or o.lost_since is not None or o.t[-1] <= t_end or o.t0 is None or tr.t0 is None:
                continue
            k = int(np.searchsorted(o.t, t_end))
            k = min(k, len(o.t) - 1)
            if math.hypot(o.x[k] - tr.x[-1], o.y[k] - tr.y[-1]) >= self.reacq_r:
                continue
            older_is_tr = tr.t0 <= o.t0
            if (older_is_tr and o.t0 >= t_end - self.ghost_s) or (not older_is_tr and t_end - tr.t0 < self.ghost_s):
                return o, older_is_tr
        return None, False

    def _join_split(self, old, new):
        """Doi tuong cu nhan lai quy dao tu phan tach, giu so thu tu va luot vao vung."""
        k = int(np.searchsorted(new.t, old.t[-1], side="right"))
        old.t += new.t[k:]; old.x += new.x[k:]; old.y += new.y[k:]
        old.hits += new.hits; old.size, old.gaps = new.size, new.gaps
        old.vx, old.vy, old.speed, old.still_since, old.moving, old.v_t0 = new.vx, new.vy, new.speed, new.still_since, new.moving, new.v_t0
        old.merged, old.absorb_t = max(old.merged, new.merged), new.absorb_t if new.merged else old.absorb_t
        self._drop_entry(new, old)
        self.tracks.remove(new)
        self.events.append(("GOP_TACH", old.id, old.t[-1], old.x[-1], old.y[-1], "#%d là phần tách của #%d" % (new.id, old.id)))

    def _drop_entry(self, ghost, keeper):
        if ghost.entry is not None:
            if keeper.entry is None:
                keeper.entry, keeper.entry["id"] = ghost.entry, keeper.id
            else:
                self.entries.remove(ghost.entry)
            ghost.entry = None

    def _pred_dist(self, tr, m):
        dt = max(m[0] - tr.t[-1], 0.0)
        return math.hypot(m[1] - tr.x[-1] - tr.vx * dt, m[2] - tr.y[-1] - tr.vy * dt), dt

    def _merge_tracks(self, sc, now):
        conf = [tr for tr in self.confirmed() if tr.speed is not None and tr.t[-1] >= sc.t]
        close = set()
        for k, a in enumerate(conf):
            for b in conf[k + 1:]:
                if math.hypot(a.x[-1] - b.x[-1], a.y[-1] - b.y[-1]) < self.merge_d and \
                        math.hypot(a.vx - b.vx, a.vy - b.vy) < self.merge_dv:
                    close.add((min(a.id, b.id), max(a.id, b.id)))
        self._pairs = {k: self._pairs.get(k, now) for k in close}
        by_id = {tr.id: tr for tr in self.tracks}
        for (ia, ib), since in sorted(self._pairs.items()):
            a, b = by_id.get(ia), by_id.get(ib)
            if a is None or b is None or now - since < self.merge_s:
                continue
            a.merged += b.merged + 1; a.absorb_t = now; a.v_t0 = a.t[-1]
            for name in ("still_since", "zone_since"):
                vals = [v for v in (getattr(a, name), getattr(b, name)) if v is not None]
                setattr(a, name, min(vals) if vals else None)
            if b.entry is not None:                             # da gop thi tinh la mot doi tuong, bo luot cua doi tuong bi gop
                if a.entry is None:
                    a.entry, a.entry["id"] = b.entry, a.id
                else:
                    self.entries.remove(b.entry)
                b.entry = None
            self.tracks.remove(b); del by_id[ib]
        self._pairs = {k: v for k, v in self._pairs.items() if k[0] in by_id and k[1] in by_id}

    def _refresh(self, tr):
        t = np.array(tr.t)
        # it nhat 3 diem gan nhat: quay cham hoac mat vai vong thi cua so thoi gian co the chi con 2 diem
        i0 = max(0, min(int(np.searchsorted(t, t[-1] - self.win_s)), len(t) - 3))
        if tr.v_t0 is not None:
            i0 = max(i0, int(np.searchsorted(t, tr.v_t0, side="right")))
        tw, xw, yw = t[i0:], np.array(tr.x)[i0:], np.array(tr.y)[i0:]
        if len(tw) >= 3 and tw[-1] - tw[0] >= 1.0:
            tr.vx, tr.vy = float(np.polyfit(tw, xw, 1)[0]), float(np.polyfit(tw, yw, 1)[0])
            tr.speed = math.hypot(tr.vx, tr.vy)
            spread = np.max(np.hypot(xw - xw.mean(), yw - yw.mean()))
            # tam cum dao dong theo mat nhin thay, nen cho phep rong hon neu tam khong troi di dau;
            # gioi han 2 * still_r de nguoi di qua lai (van toc trung binh ~ 0) khong bi tinh la dung
            still = spread < self.still_r or (tr.speed < self.still_v and spread < 2 * self.still_r)
            tr.still_since = (tr.still_since if tr.still_since is not None else float(tw[0])) if still else None
        keep = t >= t[-1] - self.hist_s
        tr.t, tr.x, tr.y = [v for v, c in zip(tr.t, keep) if c], [v for v, c in zip(tr.x, keep) if c], [v for v, c in zip(tr.y, keep) if c]

    def update(self, sc, objs, zone=()):
        """Cap nhat bang mot vong quet. Tra ve danh sach doi tuong da xac nhan."""
        meas = self._measure(sc, objs)
        pairs = []
        for i, tr in enumerate(self.tracks):
            for j, m in enumerate(meas):
                if tr.lost_since is not None:                   # dang giu: chi noi lai quanh vi tri cuoi
                    d = math.hypot(m[1] - tr.x[-1], m[2] - tr.y[-1])
                    if d <= self.reacq_r:
                        pairs.append((d, i, j))
                    continue
                d, dt = self._pred_dist(tr, m)
                # doi tuong da co huong: du doan tot hon nen cua so hep hon
                if d <= self.gate + self.vmax * dt * (1.0 if tr.hits < self.confirm else 0.5):
                    pairs.append((d, i, j))
        now = sc.t + sc.period
        got, used_m = {}, set()
        for d, i, j in sorted(pairs):
            if i in got or j in used_m:
                continue
            got[i] = [j]; used_m.add(j)
        for i, tr in enumerate(self.tracks):                    # doi tuong da gop gom them cac cum con gan
            if tr.merged and now - tr.absorb_t > self.absorb_s:
                tr.merged = 0                                   # lau khong con cum thu hai: da la mot vat
            if not tr.merged or tr.lost_since is not None:
                continue
            for j, m in enumerate(meas):
                if j not in used_m and self._pred_dist(tr, m)[0] < 1.5 * self.merge_d:
                    got.setdefault(i, []).append(j); used_m.add(j); tr.absorb_t = now
        for i, js in got.items():
            tr = self.tracks[i]
            gap = max(meas[j][6] for j in js)
            t, x, y, size = self._meas(min(meas[j][0] for j in js), np.concatenate([meas[j][4] for j in js]),
                                       np.concatenate([meas[j][5] for j in js]), gap)[:4]
            if tr.lost_since is not None:
                self.events.append(("NOI_LAI", tr.id, t, x, y, "#%d xuất hiện lại sau %.0f s" % (tr.id, t - tr.lost_since)))
                tr.lost_since, tr.lost_s, tr.v_t0 = None, 0.0, tr.t[-1]   # khong tinh toc do qua khoang mat tin hieu
            tr.t.append(t); tr.x.append(x); tr.y.append(y); tr.size = size; tr.hits += 1
            tr.gaps = (tr.gaps + [gap])[-5:]
            self._refresh(tr)
            if tr.still_since is not None or (tr.speed is not None and tr.speed < self.move_v / 2):
                tr.moving = False                               # nguong tat thap hon nguong bat de khong bao lien tuc
            elif not tr.moving and tr.hits >= self.confirm and tr.speed is not None and tr.speed >= self.move_v:
                tr.moving = True
        for j, m in enumerate(meas):
            if j not in used_m:
                self.tracks.append(Track(self.next_id, [m[0]], [m[1]], [m[2]], m[3], False, t0=m[0], gaps=[m[6]]))
                self.next_id += 1
        for tr in list(self.tracks):                            # truoc khi coi la bien mat: co phai bi tach cum khong
            if tr in self.tracks and tr.lost_since is None and tr.hits >= self.confirm and now - tr.t[-1] > self.miss_s:
                o, older_is_tr = self._find_split(tr)
                if o is not None and older_is_tr:
                    self._join_split(tr, o)
                elif o is not None:
                    self._drop_entry(tr, o)
                    self.tracks.remove(tr)
                    self.events.append(("GOP_TACH", o.id, tr.t[-1], tr.x[-1], tr.y[-1], "#%d là phần tách của #%d" % (tr.id, o.id)))
        keep = []
        for tr in self.tracks:
            if now - tr.t[-1] <= self.miss_s:
                keep.append(tr)
            elif tr.lost_since is not None:
                tr.lost_s = now - tr.lost_since
                if tr.lost_s <= self.hold_s:
                    keep.append(tr)
                else:
                    self.events.append(("HET_GIU", tr.id, now, tr.x[-1], tr.y[-1], "#%d: hết thời gian giữ, không thấy lại" % tr.id))
                    self._close_entry(tr, "Hết giờ giữ")
            elif self._can_hold(tr, zone):
                tr.lost_since, tr.lost_s, tr.vx, tr.vy, tr.moving = tr.t[-1], now - tr.t[-1], 0.0, 0.0, False
                self.events.append(("MAT_TIN_HIEU", tr.id, tr.t[-1], tr.x[-1], tr.y[-1], "#%d biến mất giữa vùng tại (%.2f, %.2f)" % (tr.id, tr.x[-1], tr.y[-1])))
                if tr.entry is not None:
                    tr.entry["t_lost"] = tr.t[-1]; tr.entry["n_lost"] += 1
                keep.append(tr)
            else:
                if tr.entry is not None:
                    tr.entry["t_lost"] = tr.t[-1]
                self._close_entry(tr, "Mất tín hiệu")
        self.tracks = keep
        if self.merge_d > 0:
            self._merge_tracks(sc, now)
        for tr in self.tracks:
            tr.in_zone = len(zone) >= 3 and bool(point_in_polygon(np.array([tr.x[-1]]), np.array([tr.y[-1]]), zone)[0])
            tr.zone_since = (tr.zone_since if tr.zone_since is not None else tr.t[-1]) if tr.in_zone else None
            inside = tr.in_zone or len(zone) < 3                # chua co vung cam: ghi lich su tren toan phong
            if inside and tr.entry is None and tr.hits >= self.confirm:
                self._open_entry(tr)
            elif not inside:
                self._close_entry(tr, "Đã ra khỏi vùng")
            self._touch_entry(tr)
        return self.confirmed()


class TrackLog:
    """Ghi vi tri va toc do doi tuong da xac nhan, moi lan do mot dong. t cung dong ho voi t_pc."""
    HEAD = "id,t,x_m,y_m,vx_m_s,vy_m_s,v_m_s,dung_s,trong_vung,trong_vung_s,size_m,so_da_gop\n"

    def __init__(self, path):
        self.path = path

    def write(self, sc, tracks):
        rows = []
        for tr in tracks:
            if tr.t[-1] < sc.t:                         # vong nay khong thay doi tuong, khong co phep do moi
                continue
            t = tr.t[-1]
            rows.append("%d,%.6f,%.3f,%.3f,%.3f,%.3f,%s,%s,%d,%s,%.3f,%d\n" % (
                tr.id, t, tr.x[-1], tr.y[-1], tr.vx, tr.vy, "" if tr.speed is None else "%.3f" % tr.speed,
                "" if tr.still_since is None else "%.2f" % (t - tr.still_since), tr.in_zone,
                "" if tr.zone_since is None else "%.2f" % (t - tr.zone_since), tr.size, tr.merged))
        if rows:
            new = not os.path.exists(self.path)
            with open(self.path, "a", encoding="utf-8") as f:
                f.write((self.HEAD if new else "") + "".join(rows))


class EntryLog:
    """Ghi tung luot vao vung khi luot do ket thuc. t_in, t_out cung dong ho voi t_pc."""
    HEAD = ("id,t_xuat_hien,t_vao,t_ra,trong_vung_s,v_vao_m_s,v_tb_m_s,v_max_m_s,rong_m,x_vao,y_vao,x_cuoi,y_cuoi,"
            "so_lan_mat_tin_hieu,t_mat_tin_hieu,trang_thai\n")

    def __init__(self, path):
        self.path = path

    def write(self, entries):
        if entries:
            new = not os.path.exists(self.path)
            with open(self.path, "a", encoding="utf-8") as f:
                f.write((self.HEAD if new else "") + "".join(self._row(e) for e in entries))

    @staticmethod
    def _row(e):
        v = lambda x: "" if x is None else "%.3f" % x
        return "%d,%.6f,%.6f,%.6f,%.2f,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%s,%s\n" % (
            e["id"], e["t0"], e["t_in"], e["t_out"], e["t_out"] - e["t_in"], v(e["v_in"]), v(e["v_sum"] / e["v_n"] if e["v_n"] else None),
            v(e["v_max"] if e["v_n"] else None), e["size"], e["x_in"], e["y_in"], e["x_last"], e["y_last"], e["n_lost"],
            "" if e["t_lost"] is None else "%.6f" % e["t_lost"], e["note"])


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

    def note(self, kind, seq, period, t, x, y):
        """Su kien cua Tracker (MAT_TIN_HIEU, NOI_LAI, HET_GIU): t nam o cot t_alarm, vi tri o cx, cy."""
        self._write("%s,%d,,,,%.6f,,,,%.6f,,,,%.3f,%.3f,%.3f,,\n" % (kind, seq, period, t, time.time(), x, y))

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
