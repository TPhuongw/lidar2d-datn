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
        self.all_objs = []              # cum khac nen tren toan phong (ca ngoai vung cam), de theo doi

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


class Tracker:
    """Noi cum giua cac vong thanh doi tuong: toc do, huong, thoi gian dung, quy dao.

    Moi cum co thoi diem rieng (t moc + a0/360 * T) vi tia quet toi moi huong vao mot luc khac nhau.
    Khong anh huong logic bao dong cua Detector.
    Tam cum la tam mat nhin thay, khong phai tam vat. Toc do chua duoc doi chieu voi phep do doc lap.
    """

    def __init__(self, gate=0.4, vmax=3.0, merge=0.35, miss_s=1.5, confirm=3, win_s=2.0, still_r=0.15, still_v=0.1, hist_s=30.0):
        self.gate, self.vmax, self.merge = gate, vmax, merge    # m, m/s, m
        self.miss_s, self.confirm = miss_s, confirm             # bo sau bay nhieu s khong thay; xac nhan sau bay nhieu lan
        self.win_s, self.still_r, self.still_v, self.hist_s = win_s, still_r, still_v, hist_s
        self.reset()

    def reset(self):
        self.tracks, self.next_id = [], 1

    def confirmed(self):
        return [tr for tr in self.tracks if tr.hits >= self.confirm]

    def _measure(self, sc, objs):
        """Gop cum sat nhau (mot vat bi tach, vi du tay va than) thanh mot phep do (t, x, y, size)."""
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
            out.append((sc.t + a0 / 360.0 * sc.period, float(xs.mean()), float(ys.mean()), float(math.hypot(np.ptp(xs), np.ptp(ys)))))
        return out

    def _refresh(self, tr):
        t = np.array(tr.t)
        # it nhat 3 diem gan nhat: quay cham hoac mat vai vong thi cua so thoi gian co the chi con 2 diem
        i0 = max(0, min(int(np.searchsorted(t, t[-1] - self.win_s)), len(t) - 3))
        tw, xw, yw = t[i0:], np.array(tr.x)[i0:], np.array(tr.y)[i0:]
        span = tw[-1] - tw[0]
        if len(tw) >= 3 and span >= 1.0:
            tr.vx, tr.vy = float(np.polyfit(tw, xw, 1)[0]), float(np.polyfit(tw, yw, 1)[0])
            tr.speed = math.hypot(tr.vx, tr.vy)
            spread = np.max(np.hypot(xw - xw.mean(), yw - yw.mean()))
            # tam cum dao dong theo mat nhin thay, nen cho phep rong hon neu tam khong troi di dau;
            # gioi han 2 * still_r de nguoi di qua lai (van toc trung binh ~ 0) khong bi tinh la dung
            still = span >= 1.5 and (spread < self.still_r or (tr.speed < self.still_v and spread < 2 * self.still_r))
            tr.still_since = (tr.still_since if tr.still_since is not None else float(tw[0])) if still else None
        keep = t >= t[-1] - self.hist_s
        tr.t, tr.x, tr.y = [v for v, c in zip(tr.t, keep) if c], [v for v, c in zip(tr.x, keep) if c], [v for v, c in zip(tr.y, keep) if c]

    def update(self, sc, objs, zone=()):
        """Cap nhat bang mot vong quet. Tra ve danh sach doi tuong da xac nhan."""
        meas = self._measure(sc, objs)
        pairs = []
        for i, tr in enumerate(self.tracks):
            for j, (t, x, y, _) in enumerate(meas):
                dt = max(t - tr.t[-1], 0.0)
                d = math.hypot(x - tr.x[-1] - tr.vx * dt, y - tr.y[-1] - tr.vy * dt)
                # doi tuong da co huong: du doan tot hon nen cua so hep hon
                if d <= self.gate + self.vmax * dt * (1.0 if tr.hits < self.confirm else 0.5):
                    pairs.append((d, i, j))
        used_t, used_m = set(), set()
        for d, i, j in sorted(pairs):
            if i in used_t or j in used_m:
                continue
            used_t.add(i); used_m.add(j)
            tr, (t, x, y, size) = self.tracks[i], meas[j]
            tr.t.append(t); tr.x.append(x); tr.y.append(y); tr.size = size; tr.hits += 1
            self._refresh(tr)
        for j, (t, x, y, size) in enumerate(meas):
            if j not in used_m:
                self.tracks.append(Track(self.next_id, [t], [x], [y], size, False))
                self.next_id += 1
        now = sc.t + sc.period
        self.tracks = [tr for tr in self.tracks if now - tr.t[-1] <= self.miss_s]
        for tr in self.tracks:
            tr.in_zone = len(zone) >= 3 and bool(point_in_polygon(np.array([tr.x[-1]]), np.array([tr.y[-1]]), zone)[0])
            tr.zone_since = (tr.zone_since if tr.zone_since is not None else tr.t[-1]) if tr.in_zone else None
        return self.confirmed()


class TrackLog:
    """Ghi vi tri va toc do doi tuong da xac nhan, moi lan do mot dong. t cung dong ho voi t_pc."""
    HEAD = "id,t,x_m,y_m,vx_m_s,vy_m_s,v_m_s,dung_s,trong_vung,trong_vung_s,size_m\n"

    def __init__(self, path):
        self.path = path

    def write(self, sc, tracks):
        rows = []
        for tr in tracks:
            if tr.t[-1] < sc.t:                         # vong nay khong thay doi tuong, khong co phep do moi
                continue
            t = tr.t[-1]
            rows.append("%d,%.6f,%.3f,%.3f,%.3f,%.3f,%s,%s,%d,%s,%.3f\n" % (
                tr.id, t, tr.x[-1], tr.y[-1], tr.vx, tr.vy, "" if tr.speed is None else "%.3f" % tr.speed,
                "" if tr.still_since is None else "%.2f" % (t - tr.still_since), tr.in_zone,
                "" if tr.zone_since is None else "%.2f" % (t - tr.zone_since), tr.size))
        if rows:
            new = not os.path.exists(self.path)
            with open(self.path, "a", encoding="utf-8") as f:
                f.write((self.HEAD if new else "") + "".join(rows))


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
