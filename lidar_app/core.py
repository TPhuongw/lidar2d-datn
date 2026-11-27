"""
core.py - Loi xu ly du lieu LiDAR 2D (khong phu thuoc giao dien).

  parse_tf03   : tach khung TF03 9 byte tu luong byte
  tf03_probe / tf03_set_rate / tf03_set_baud : do baud, dat tan so, dat baud TF03
  ClockSync    : khop dong ho nguon (so thu tu khung / micros UNO) sang dong ho PC theo bien duoi
  Mapper       : gan goc cho tung khung theo moc 0 do, gom thanh tung vong quet (Scan)
  LiveSource   : doc that tu TF03 (FT232) + UNO, ghi log
  ReplaySource : phat lai thu muc log_*/ nhu dang chay that

Khi co STM32 chi can them mot Source moi tra ve thang (goc, khoang cach); Mapper/giao dien giu nguyen.
"""
import collections, csv, json, math, os, re, struct, threading, time
from dataclasses import dataclass, field

import numpy as np


# ------------------------------------------------------------------ TF03
def parse_tf03(buf):
    """Tra ve (list[(dist_cm, strength)], phan du, so khung loi checksum)."""
    out, bad, i, n = [], 0, 0, len(buf)
    while i + 9 <= n:
        if buf[i] == 0x59 and buf[i + 1] == 0x59:
            if (sum(buf[i:i + 8]) & 0xFF) == buf[i + 8]:
                out.append((buf[i + 2] | buf[i + 3] << 8, buf[i + 4] | buf[i + 5] << 8))
                i += 9
                continue
            bad += 1
        i += 1
    return out, buf[i:], bad


# ------------------------------------------------------------------ cau hinh TF03 (chi khi cong dang ranh)
TF03_BAUDS = (115200, 230400, 460800, 921600)
TF03_RATES = (100, 200, 500, 1000, 2000, 5000)   # da chay that toi 2000 Hz; 5000 Hz chua kiem chung
TF03_BAUD_PER_HZ = 112                           # baud >= 112 x Hz, vuot muc nay TF03 qua tai va bo lenh


def _tf03_cmd(cmd_id, payload=b""):
    body = bytes([0x5A, 4 + len(payload), cmd_id]) + bytes(payload)
    return body + bytes([sum(body) & 0xFF])


def _tf03_send(ser, cmd, wait=0.4):
    """Gui lenh, tra ve cac khung phan hoi 5A hop le (bo qua khung du lieu 59 59)."""
    ser.reset_input_buffer()
    ser.write(cmd)
    buf, t_end = bytearray(), time.perf_counter() + wait
    while time.perf_counter() < t_end:
        buf += ser.read(max(1, ser.in_waiting))
    replies, i = [], 0
    while True:
        i = buf.find(b"\x5A", i)
        if i < 0 or i + 2 > len(buf):
            return replies
        n = buf[i + 1]
        if 4 <= n <= 16 and i + n <= len(buf) and (sum(buf[i:i + n - 1]) & 0xFF) == buf[i + n - 1]:
            replies.append(bytes(buf[i:i + n])); i += n
        else:
            i += 1


def _tf03_save(ser):
    """Luu cau hinh. True neu TF03 tra loi 5A 05 11 00 70."""
    return any(r[2] == 0x11 and r[3] == 0 for r in _tf03_send(ser, _tf03_cmd(0x11)) if len(r) >= 5)


def tf03_measure(ser, sec=1.0):
    """Do toc do khung that. Tra ve (khung/s, so khung loi)."""
    ser.reset_input_buffer()
    buf, n, bad, t0 = bytearray(), 0, 0, time.perf_counter()
    while time.perf_counter() - t0 < sec:
        buf += ser.read(max(1, ser.in_waiting))
        frames, rest, b = parse_tf03(buf)
        buf = bytearray(rest); n += len(frames); bad += b
    return n / sec, bad


def tf03_probe(port, bauds=TF03_BAUDS, sec=0.4):
    """Do baud hien tai cua TF03. Tra ve (baud, khung/s) hoac None."""
    import serial
    for baud in bauds:
        with serial.Serial(port, baud, timeout=0.05) as ser:
            rate, bad = tf03_measure(ser, sec)
        if rate * sec >= 10 and rate * sec > 10 * bad:   # sai baud chi con vai khung trung checksum ngau nhien
            return baud, rate
    return None


def tf03_set_rate(port, baud, hz):
    """Dat va luu tan so. Tra ve (da nhan lenh luu, khung/s do lai, so khung loi)."""
    import serial
    if hz not in TF03_RATES:
        raise ValueError("Tần số %d Hz không nằm trong danh sách cho phép." % hz)
    if hz * TF03_BAUD_PER_HZ > baud:
        raise ValueError("%d Hz cần baud ≥ %d, hiện là %d. Hãy đổi baud trước." % (hz, hz * TF03_BAUD_PER_HZ, baud))
    with serial.Serial(port, baud, timeout=0.05) as ser:
        _tf03_send(ser, _tf03_cmd(0x03, struct.pack("<H", hz)))
        saved = _tf03_save(ser)
        time.sleep(0.2)
        rate, bad = tf03_measure(ser)
    return saved, rate, bad


# --- TF03 noi vao Nucleo (firmware nucleo_lidar): cau hinh qua lenh chu tren cong VCP
STM_VCP_BAUD = 460800


def _stm_ask(ser, cmd, prefixes, timeout):
    """Gui mot lenh chu, tra ve dong dau tien bat dau bang mot trong prefixes (None neu het gio)."""
    ser.write(b"b 0\n"); time.sleep(0.15); ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    buf, t_end = b"", time.perf_counter() + timeout
    while time.perf_counter() < t_end:
        buf += ser.read(max(1, ser.in_waiting))
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            s = line.decode("ascii", "replace").strip()
            if s.startswith(prefixes):
                return s
    return None


def stm_parse_probe(line):
    """'TF03 DO baud 460800 khung/s 1000' -> (460800, 1000.0); 'TF03 DO khong thay ...' hoac None -> None."""
    m = re.match(r"TF03 DO baud (\d+) khung/s (\d+)", line or "")
    return (int(m.group(1)), float(m.group(2))) if m else None


def stm_tf03_probe(port):
    """Nucleo tu do baud TF03 (lenh p). Tra ve (baud, khung/s) hoac None."""
    import serial
    with serial.Serial(port, STM_VCP_BAUD, timeout=0.05) as ser:
        line = _stm_ask(ser, "p", ("TF03 DO",), 4.0)
    if line is None:
        raise ValueError("Nucleo không trả lời lệnh dò. Kiểm tra cổng COM và firmware nucleo_lidar mới nhất.")
    return stm_parse_probe(line)


def stm_tf03_set_rate(port, hz):
    """Lenh f <Hz> qua Nucleo. Tra ve (da nhan lenh luu, khung/s do lai, 0)."""
    import serial
    if hz not in TF03_RATES:
        raise ValueError("Tần số %d Hz không nằm trong danh sách cho phép." % hz)
    with serial.Serial(port, STM_VCP_BAUD, timeout=0.05) as ser:
        if _stm_ask(ser, "p", ("TF03 DO",), 4.0) is None:   # baud USART1 cua Nucleo phai dung baud TF03
            raise ValueError("Nucleo không trả lời lệnh dò. Kiểm tra cổng COM và firmware nucleo_lidar mới nhất.")
        line = _stm_ask(ser, "f %d" % hz, ("TF03:", "Chi cho phep", "%d Hz can" % hz), 3.0)
    m = re.match(r"TF03: dat \d+ Hz, lenh luu (.+), do lai (\d+) khung/s", line or "")
    if not m:
        raise ValueError(line or "Nucleo không trả lời lệnh đặt tần số.")
    return not m.group(1).startswith("KHONG"), float(m.group(2)), 0


def stm_tf03_set_baud(port, new_baud):
    """Lenh B <baud> qua Nucleo. True neu TF03 nhan lenh luu. Co hieu luc sau khi rut nguon TF03."""
    import serial
    if new_baud not in TF03_BAUDS:
        raise ValueError("Baud %d không nằm trong danh sách cho phép." % new_baud)
    with serial.Serial(port, STM_VCP_BAUD, timeout=0.05) as ser:
        if _stm_ask(ser, "p", ("TF03 DO",), 4.0) is None:
            raise ValueError("Nucleo không trả lời lệnh dò. Kiểm tra cổng COM và firmware nucleo_lidar mới nhất.")
        line = _stm_ask(ser, "B %d" % new_baud, ("TF03 BAUD",), 3.0)
    if line is None or line.startswith("TF03 BAUD loi"):
        raise ValueError(line or "Nucleo không trả lời lệnh đổi baud.")
    return "KHONG" not in line


def tf03_set_baud(port, baud, new_baud):
    """Dat va luu baud moi; chi co hieu luc sau khi rut nguon TF03. Tra ve True neu TF03 nhan lenh luu."""
    import serial
    if new_baud not in TF03_BAUDS:
        raise ValueError("Baud %d không nằm trong danh sách cho phép." % new_baud)
    with serial.Serial(port, baud, timeout=0.05) as ser:
        rate, _ = tf03_measure(ser, 0.5)
        if rate == 0:
            raise ValueError("Không nhận được dữ liệu TF03 ở %d baud. Hãy bấm Dò TF03 trước." % baud)
        if rate * TF03_BAUD_PER_HZ > new_baud * 1.02:     # chua 2 % vi TF03 thuc te chay nhanh hon danh dinh
            raise ValueError("TF03 đang chạy khoảng %.0f Hz, %d baud không đủ. Hãy giảm tần số trước." % (rate, new_baud))
        _tf03_send(ser, _tf03_cmd(0x06, struct.pack("<I", new_baud)))
        time.sleep(0.2)
        saved = _tf03_save(ser)
        if not saved:                                     # phong khi TF03 da doi baud ngay
            ser.baudrate = new_baud
            time.sleep(0.2)
            saved = _tf03_save(ser)
    return saved


# ------------------------------------------------------------------ dong bo dong ho
def lower_line(R, L):
    """Duong thang nam duoi moi diem (R, L) va sat chung nhat: canh bao loi duoi chua R trung binh
    (bai toan LP uoc luong lech dong ho cua Moon, Skelly, Towsley 1999). Tre truyen chi lam L lon them,
    nen goi tin ve tre, ke ca ve don cuc vai giay, khong keo lech duong nay mien la con vai goi ve dung gio.
    Tra ve (a, b) voi L ~ a + b*R."""
    o = np.lexsort((L, R))                          # theo R, cung R thi L nho truoc
    R, L = R[o], L[o]
    h, last = [], None                              # bao loi duoi (monotone chain)
    for p in zip(R.tolist(), L.tolist()):
        if p[0] == last:                            # cung R: chi giu diem thap nhat
            continue
        last = p[0]
        while len(h) >= 2 and (h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0]) <= 0:
            h.pop()
        h.append(p)
    if len(h) < 2:
        b = 1.0
    else:
        m = float(R.mean())
        k = next((i for i in range(len(h) - 1) if h[i + 1][0] >= m), len(h) - 2)
        b = (h[k + 1][1] - h[k][1]) / (h[k + 1][0] - h[k][0])
    return float(np.min(L - b * R)), float(b)


class ClockSync:
    """t_PC = a + b*remote + tre (tre >= 0).
    Mac dinh (max_age = 0): b hoi quy, a bien duoi trong cua so n mau gan nhat (hull=True: ca a, b theo bien duoi).
    max_age > 0 (dung cho moc 0 do, it mau nhung chinh xac): tach hai phan
      - b (do troi dong ho, doi rat cham): hoi quy chi tren cac mau ve dung gio (tre < ontime) trong cua so dai n_long,
        chi cap nhat khi moc dang ve tot (>= 80 % mau gan day dung gio). Hoi quy tren cua so ngan khi moc thua mau dung gio
        lam b lech ~1e-3, ban do xoay ~0,7 do/s (log 06/11 11:44).
      - a: bien duoi trong cua so ngan n; neu max_age giay (dong ho nguon) khong co mau dung gio thi giu a cu,
        toi da hold_max giay."""

    def __init__(self, n=400, every=10, max_age=0.0, ontime=0.04, hold_max=120.0, n_long=1500, hull=False):
        self.r, self.l = collections.deque(maxlen=n), collections.deque(maxlen=n)
        self.base = self.a = self.b = None; self.every = every
        self.cnt = 0
        self.max_age, self.ontime, self.hold_max, self.hull = max_age, ontime, hold_max, hull
        self.lr, self.ll, self.lok = (collections.deque(maxlen=n_long) for _ in range(3))
        self.recent_ok = collections.deque(maxlen=25)
        self.b_long = False        # b da lay tu cua so dai
        self.age = 0.0             # tuoi (s nguon) cua mau ve dung gio gan nhat
        self.delay = 0.0           # tre cua mau moi nhat so voi mo hinh (s)
        self.held = 0.0            # da giu a cu bao lau (s nguon)
        self.t_ontime = None

    def add(self, remote, local):
        if self.base is None:
            self.base = remote
        x = remote - self.base
        self.r.append(x); self.l.append(local); self.cnt += 1
        if self.max_age <= 0:
            if len(self.r) >= 5 and (self.b is None or self.cnt % self.every == 0):
                R, L = np.array(self.r), np.array(self.l)
                if np.ptp(R) > 0:
                    if self.hull and len(R) >= 10:
                        self.a, self.b = lower_line(R, L)
                    else:
                        self.b = float(np.polyfit(R, L, 1)[0])
                        self.a = float(np.min(L - self.b * R))
            return
        # ---- che do moc 0 do
        ok = True
        if self.b is not None:
            self.delay = local - (self.a + self.b * x)
            ok = self.delay < self.ontime
            if ok:
                self.t_ontime = x
            self.age = x - (self.t_ontime if self.t_ontime is not None else self.r[0])
        self.lr.append(x); self.ll.append(local); self.lok.append(ok); self.recent_ok.append(ok)
        if len(self.r) < 5:
            return
        R, L = np.array(self.r), np.array(self.l)
        if np.ptp(R) <= 0:
            return
        # b: cua so dai, chi mau dung gio, chi khi moc dang ve tot
        if self.b is None or (self.cnt % 25 == 0 and np.mean(self.recent_ok) >= 0.8) or not self.b_long:
            m = np.array(self.lok)
            LR, LL = np.array(self.lr)[m], np.array(self.ll)[m]
            if len(LR) >= 50 and np.ptp(LR) >= 60.0:
                self.b = float(np.polyfit(LR, LL, 1)[0]); self.b_long = True
            elif not self.b_long:                          # luc dau: hoi quy cua so ngan nhu cu
                self.b = float(np.polyfit(R, L, 1)[0])
        # a: bien duoi cua so ngan; giu nguyen neu lau khong co moc dung gio
        if self.a is not None and self.age > self.max_age and self.held < self.hold_max:
            self.held = self.age - self.max_age
            return
        self.held = 0.0
        self.a = float(np.min(L - self.b * R))

    def ready(self):
        return self.b is not None

    def to_local(self, remote):
        return self.a + self.b * (np.asarray(remote, float) - self.base)


# ------------------------------------------------------------------ gom vong quet
@dataclass
class Scan:
    seq: int                 # so thu tu vong
    t: float                 # thoi diem moc 0 do dau vong (dong ho PC)
    period: float            # chu ky vong (s)
    angle_deg: np.ndarray    # goc tho (chua offset/dir)
    dist_cm: np.ndarray
    strength: np.ndarray
    n_total: int = 0
    extra: dict = field(default_factory=dict)


class Mapper:
    def __init__(self):
        self.frames = collections.deque()      # (n, dist, strength, t_khung theo dong ho PC)
        # Thoi diem tung khung: "arrival" = thoi diem PC nhan goi chua no, lui 1 chu ky khung cho moi khung dung sau no trong goi
        # (PC dung lai thi cac khung don lai van duoc dat ve dung cho). "model" = duong thang theo so thu tu khung (ban cu).
        # Log 06/11 14:48: PC nhan goi deu (3,5 ms, p99 4,7 ms) nhung TF03 xuat khung khong deu, nen "model" sai theo tung luc:
        # vong lech > 3 do: model 76/5962, arrival 11/5963.
        self.frame_time = "arrival"
        self.late_thr = 0.015                  # s: goi ve tre hon mo hinh qua muc nay (PC dang khung) thi dung mo hinh
        self.n_late = 0                        # so khung da phai dung mo hinh
        self.t_last_ft = None                  # thoi diem khung moi nhat
        self.idx = []                          # moc 0 do theo dong ho nguon
        self.idx_t = []                        # thoi diem PC nhan tung moc
        # Bo vong dang ngo (theo ban sua 09/11 53fa5f1): chu ky lech > 3 % trung vi, moc ve tre > 0,1 s so voi mo hinh,
        # hoac trong vong co khoang trong du lieu > 80 ms. Vong bi bo khong ve, khong dua vao phat hien.
        self.skip_bad = True
        self.n_skip = 0
        self.skip_why = collections.Counter()  # chu_ky / moc_tre / trong_du_lieu
        # moc thua (vai moc/s) nen cap nhat moi moc; cua so 300 moc (~2 phut o 2,5 vong/s).
        # Log 06/11 11:44: tu phut 16 PC nhan moc don cuc 4 dong moi ~1,7 s (tre toi 4 s) trong khi UNO van deu 0,400 s;
        # khop theo thoi diem nhan lam ban do xoay hang tram do. Bien duoi + giu mo hinh cu khi lau khong co moc dung gio.
        self.cf, self.ci = ClockSync(400, hull=True), ClockSync(300, every=1, max_age=10.0, hold_max=600.0)
        self.done_key = None                   # moc dau cua vong da xu ly gan nhat
        self.seq = 0
        self.lock = threading.Lock()
        # thong ke
        self.n_frames = self.n_idx = 0
        self.t_last_frame = self.t_last_idx = None
        self.last_n = None                     # so thu tu khung moi nhat da nhan

    def add_frames(self, items, t_arr):
        if not items:
            return
        with self.lock:
            self.cf.add(items[-1][0], t_arr)
            nl = items[-1][0]
            if self.frame_time == "arrival":
                b = self.cf.b if self.cf.ready() and 1e-5 < self.cf.b < 0.1 else 0.001     # s / khung
                lo = self.t_last_ft if self.t_last_ft is not None else -math.inf
                late = self.cf.ready() and len(self.cf.r) >= 10 and t_arr - self.cf.to_local(nl) > self.late_thr
                for it in items:
                    tt = self.cf.to_local(it[0]) if late else t_arr - (nl - it[0]) * b
                    tt = max(tt, lo)                                                       # khong lui qua khung truoc
                    self.frames.append((it[0], it[1], it[2], tt))
                if late:
                    self.n_late += len(items)
                self.t_last_ft = t_arr
            else:
                self.frames.extend(items)
            self.n_frames += len(items); self.t_last_frame = t_arr
            self.last_n = nl

    def add_index(self, remote, t_arr):
        with self.lock:
            self.idx.append(remote); self.idx_t.append(t_arr); self.n_idx += 1; self.t_last_idx = t_arr
            self.ci.add(remote, t_arr)
            if len(self.idx) > 60:
                self.idx, self.idx_t = self.idx[-60:], self.idx_t[-60:]

    def sync_state(self):
        """(tuoi moc dung gio gan nhat s, tre nhan moc moi nhat s, dang giu mo hinh cu)."""
        with self.lock:
            return self.ci.age, self.ci.delay, self.ci.held > 0

    def periods(self):
        with self.lock:
            if len(self.idx) < 2 or not self.ci.ready():
                return np.array([])
            return np.diff(self.ci.to_local(self.idx[-20:]))

    def process(self):
        """Tra ve danh sach Scan vua hoan thanh (co the nhieu vong khi phat lai nhanh)."""
        out = []
        with self.lock:
            if len(self.idx) < 3 or not (self.cf.ready() and self.ci.ready()):
                return out
            I = self.ci.to_local(self.idx)
            if self.frame_time == "arrival":
                tf1 = lambda f: f[3]
                t_end = self.t_last_ft
            else:
                tf1 = lambda f: self.cf.to_local(f[0])
                t_end = self.cf.to_local(self.last_n)
            for j in range(len(self.idx) - 1):
                key = self.idx[j]
                if self.done_key is not None and key <= self.done_key:
                    continue
                if j == 0:                       # chua biet vong truoc do -> chi dung de dong bo
                    self.done_key = key
                    continue
                i0, i1 = I[j], I[j + 1]
                if t_end < i1:                   # khung cuoi vong chua ve het -> cho lan goi sau
                    break
                d = np.diff(I[max(0, j - 5):j + 2])
                T = float(np.median(d))
                while self.frames and tf1(self.frames[0]) < i0:
                    self.frames.popleft()
                rev = []
                while self.frames and tf1(self.frames[0]) < i1:
                    rev.append(self.frames.popleft())
                self.done_key = key
                if not rev:
                    continue
                ft = np.array([tf1(f) for f in rev], float)
                why = None
                if abs((i1 - i0) - T) > 0.03 * T:
                    why = "chu_ky"
                elif max(self.idx_t[j] - i0, self.idx_t[j + 1] - i1) > 0.1:
                    why = "moc_tre"
                elif np.max(np.diff(np.r_[i0, ft, i1])) > 0.08:
                    why = "trong_du_lieu"
                if self.skip_bad and why:
                    self.n_skip += 1; self.skip_why[why] += 1
                    continue
                ang = 360.0 * (ft - i0) / T
                self.seq += 1
                out.append(Scan(self.seq, float(i0), T, ang,
                                np.array([f[1] for f in rev], float),
                                np.array([f[2] for f in rev], float), len(rev)))
        return out


def to_xy(scan, r0=0.0, offset_deg=0.0, direction=1, max_cm=3000, min_str=0):
    """Loc diem hop le va doi sang (x, y) met. Tra ve dict."""
    d, s, a = scan.dist_cm, scan.strength, scan.angle_deg
    ok = (d > 0) & (d <= max_cm) & (s >= min_str) & (a >= 0) & (a < 360)
    th = np.radians(direction * a[ok] + offset_deg)
    d_m = d[ok] / 100.0                     # khoang cach tho: nen hoc theo d_m nen khong phu thuoc r0 (ban sua 09/11)
    r = d_m - r0
    return dict(x=r * np.cos(th), y=r * np.sin(th), r=r, d_m=d_m, ang=a[ok], s=s[ok], ok=ok)


def measure_area(revs, max_r=8.0, bin_deg=0.5, med_bins=5, min_revs=10):
    """Dien tich nhin thay duoc tu vi tri LiDAR, o do cao mat quet. Khong phai dien tich san.
    revs: list[(goc tho, r)] moi vong. Trung vi theo vong de bo nguoi di qua, trung vi theo goc de bo tia le,
    bo huong xa hon max_r (cua kinh, cua mo) roi noi suy tu hai ben. Dien tich = 1/2 * tong r^2 * dtheta.
    Tra ve dict, hoac None neu chua du min_revs vong."""
    if len(revs) < min_revs:
        return None
    nb = int(round(360 / bin_deg))
    arr = np.full((len(revs), nb), np.nan)
    for i, (ang, r) in enumerate(revs):
        ang, r = np.asarray(ang), np.asarray(r)
        o = np.argsort(r)[::-1]                        # nhieu diem cung o: giu diem gan nhat
        arr[i, (ang[o] / bin_deg).astype(int) % nb] = r[o]
    with np.errstate(all="ignore"):
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            med = np.nanmedian(arr, axis=0)
            half = med_bins // 2
            filt = np.nanmedian(np.array([np.roll(med, k) for k in range(-half, half + 1)]), axis=0)
    bad =~np.isfinite(filt) | (filt > max_r) | (filt <= 0)
    if bad.all():
        return None
    th = (np.arange(nb) + 0.5) * bin_deg
    r = np.interp(th, th[~bad], filt[~bad], period=360.0)   # huong khong dung duoc: noi suy tu hai ben
    dth = math.radians(bin_deg)
    area = 0.5 * float(np.sum(r ** 2)) * dth
    t = np.radians(th)
    x, y = r * np.cos(t), r * np.sin(t)
    perim = float(np.sum(np.hypot(np.diff(np.r_[x, x[0]]), np.diff(np.r_[y, y[0]]))))
    return dict(area_m2=area, perim_m=perim, ang=th, r=r, n_revs=len(revs), gap_deg=float(bad.sum()) * bin_deg, max_r=max_r)


# ------------------------------------------------------------------ thong so phien do
class SessionLog:
    """Ghi thong so cua mot phien do vao JSON, kem moi thay doi trong luc chay. t tinh tu luc bat dau (s)."""

    def __init__(self, path, cfg, nguon, phien_ban=""):
        self.path = path
        self.d = dict(bat_dau=time.strftime("%Y-%m-%d %H:%M:%S"), ket_thuc=None, nguon=nguon, phien_ban=phien_ban,
                      thong_so=json.loads(json.dumps(cfg)), su_kien=[])
        self._write()

    def _write(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(self.d, f, indent=2, ensure_ascii=False)
        os.replace(tmp, self.path)                     # app chet giua chung khong lam hong file cu

    def diff(self, old, new, t):
        old, new = json.loads(json.dumps(old)), json.loads(json.dumps(new))
        ch = [dict(t=round(t, 2), kieu="doi", ten=k, cu=old.get(k), moi=new[k]) for k in new if old.get(k) != new[k]]
        if ch:
            self.d["su_kien"] += ch
            self._write()

    def note(self, kieu, t, **kw):
        self.d["su_kien"].append(dict(t=round(t, 2), kieu=kieu, **kw))
        self._write()

    def end(self):
        self.d["ket_thuc"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self._write()


# ------------------------------------------------------------------ nguon du lieu
class LiveSource:
    """TF03 qua FT232 + UNO chay index_flag_test.ino. Ghi log vao logdir."""

    def __init__(self, mapper, tf_port, uno_port, rps=1.0, tf_baud=115200, logdir=None):
        self.m, self.tf_port, self.uno_port, self.rps, self.tf_baud = mapper, tf_port, uno_port, rps, tf_baud
        self.logdir = logdir or time.strftime("log_%Y%m%d_%H%M%S")
        self.stop_ev = threading.Event()
        self.err, self.bad_frames, self.uno_lines = "", 0, collections.deque(maxlen=50)
        self.uno = None

    def start(self):
        os.makedirs(self.logdir, exist_ok=True)
        self.flog = open(os.path.join(self.logdir, "tf03_raw.csv"), "w"); self.flog.write("n,dist_cm,strength,t_pc_s\n")
        self.ilog = open(os.path.join(self.logdir, "index.csv"), "w"); self.ilog.write("t_uno_s,t_pc_s\n")
        self.th = [threading.Thread(target=self._guard, args=(self._tf03,), daemon=True),
                   threading.Thread(target=self._guard, args=(self._uno,), daemon=True)]
        for t in self.th:
            t.start()

    def _guard(self, fn):
        try:
            fn()
        except Exception as e:
            self.err = "%s: %s" % (fn.__name__.strip("_"), e)

    def _tf03(self):
        import serial
        with serial.Serial(self.tf_port, self.tf_baud, timeout=0.01) as ser:
            ser.reset_input_buffer()
            buf, n, t_flush = bytearray(), 0, time.perf_counter()
            while not self.stop_ev.is_set():
                data = ser.read(max(1, ser.in_waiting))
                if time.perf_counter() - t_flush >= 1.0:      # app chet thi chi mat toi da khoang 1 s du lieu tho
                    self.flog.flush(); t_flush = time.perf_counter()
                if not data:
                    continue
                if ser.in_waiting:                           # hut het byte den trong luc cho GIL truoc khi lay moc thoi gian
                    data += ser.read(ser.in_waiting)
                t_arr = time.perf_counter()
                buf += data
                frames, rest, bad = parse_tf03(buf)
                buf = bytearray(rest); self.bad_frames += bad; n += bad
                items = []
                for d, s in frames:
                    items.append((n, d, s)); n += 1
                self.m.add_frames(items, t_arr)
                for it in items:
                    self.flog.write("%d,%d,%d,%.6f\n" % (it + (t_arr,)))

    def _uno(self):
        import serial
        with serial.Serial(self.uno_port, 115200, timeout=0.1) as ser:
            self.uno = ser
            time.sleep(2.5)
            ser.reset_input_buffer()
            self.set_speed(self.rps)
            last_us, wrap = None, 0
            try:
                while not self.stop_ev.is_set():
                    line = ser.readline()
                    if not line:
                        continue
                    t = time.perf_counter() - len(line) * 10 / 115200
                    txt = line.decode(errors="ignore").strip()
                    if txt.startswith("MOC 0"):
                        mm = re.search(r"t=(\d+)", txt)
                        if mm:
                            us = int(mm.group(1))
                            if last_us is not None and us < last_us:
                                wrap += 1 << 32
                            last_us = us
                            remote = (us + wrap) * 1e-6
                        else:
                            remote = t
                        self.m.add_index(remote, t)
                        self.ilog.write("%.6f,%.6f\n" % (remote, t)); self.ilog.flush()
                    elif txt:
                        self.uno_lines.append(txt)
            finally:                                          # loi giua chung van dung dong co
                self.uno = None
                try:
                    ser.write(b"r 0\n"); time.sleep(0.2)
                except Exception:
                    pass

    def set_speed(self, rps):
        self.rps = rps
        if self.uno is not None:
            self.uno.write(("r %.2f\n" % rps).encode())

    def stop(self):
        self.stop_ev.set()
        for t in getattr(self, "th", []):
            t.join(timeout=1.5)
        for f in (getattr(self, "flog", None), getattr(self, "ilog", None)):
            if f:
                f.close()

    def tick(self):
        pass

    def now(self):
        return time.perf_counter()


class ReplaySource:
    """Phat lai thu muc log. Ho tro dinh dang moi (n,dist,str,t_pc / t_uno,t_pc) va cu (t,dist,str / t)."""

    def __init__(self, mapper, logdir, speed=1.0):
        self.m, self.logdir, self.speed = mapper, logdir, speed
        self.err, self.bad_frames, self.uno_lines = "", 0, collections.deque(maxlen=50)
        self.paused, self.finished = False, False
        ev = []
        with open(os.path.join(logdir, "tf03_raw.csv")) as f:
            rd = csv.reader(f); head = next(rd)
            rows = list(rd)
        if head[0] == "n":                                # moi: gom theo goi den (cung t_pc)
            chunk, t_prev = [], None
            for r in rows:
                n, d, s, t = int(r[0]), int(r[1]), int(r[2]), float(r[3])
                if t_prev is not None and t != t_prev:
                    ev.append((t_prev, 0, chunk)); chunk = []
                chunk.append((n, d, s)); t_prev = t
            if chunk:
                ev.append((t_prev, 0, chunk))
        else:                                             # cu: moi dong 1 khung, t da rai nguoc
            for k, r in enumerate(rows):
                ev.append((float(r[0]), 0, [(k, int(r[1]), int(r[2]))]))
        with open(os.path.join(logdir, "index.csv")) as f:
            rd = csv.reader(f); head = next(rd)
            for r in rd:
                if len(r) >= 2:
                    ev.append((float(r[1]), 1, float(r[0])))
                else:
                    ev.append((float(r[0]), 1, float(r[0])))
        ev.sort(key=lambda e: (e[0], e[1]))
        self.ev, self.k = ev, 0
        self.t0_log = ev[0][0] if ev else 0.0
        self.duration = (ev[-1][0] - self.t0_log) if ev else 0.0
        self.pos = 0.0                                    # thoi gian log da phat (s)
        self.t_wall = None

    def start(self):
        self.t_wall = time.perf_counter()

    def tick(self):
        """Goi dinh ky tu giao dien: day cac su kien den thoi diem hien tai vao Mapper."""
        now = time.perf_counter()
        if self.t_wall is None:
            self.t_wall = now
        if not self.paused and not self.finished:
            self.pos += (now - self.t_wall) * self.speed
        self.t_wall = now
        lim = self.t0_log + self.pos
        while self.k < len(self.ev) and self.ev[self.k][0] <= lim:
            t, typ, p = self.ev[self.k]
            if typ == 0:
                self.m.add_frames(p, t)
            else:
                self.m.add_index(p, t)
            self.k += 1
        if self.k >= len(self.ev):
            self.finished = True

    def set_speed(self, s):
        self.speed = s

    def now(self):
        return self.t0_log + self.pos

    def stop(self):
        self.finished = True


# ------------------------------------------------------------------ Nucleo (buoc 3): TF03 noi vao STM32, goc theo so buoc
# Firmware: firmware/nucleo_lidar. Moi khung TF03 mang so buoc ke tu moc 0 do gan nhat, nen goc = buoc x 360 / 3200,
# khong con ghep goc theo thoi diem PC nhan du lieu (nguon cua moi loi xoay ban do truoc day).
STM_FRAME, STM_INDEX = 0x55, 0x56
STM_PKT = 16
_CRC8 = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = ((_c << 1) ^ 0x07) & 0xFF if _c & 0x80 else (_c << 1) & 0xFF
    _CRC8.append(_c)


def crc8(data):
    """CRC-8, da thuc 0x07, khoi dau 0 (giong firmware)."""
    c = 0
    for b in data:
        c = _CRC8[c ^ b]
    return c


class StmParser:
    """Tach goi 16 byte tu luong USART cua Nucleo. Byte nam ngoai goi la dong chu (phan hoi lenh)."""

    def __init__(self):
        self.buf, self.text, self.bad = bytearray(), bytearray(), 0

    def feed(self, data):
        """Tra ve (goi, dong_chu). goi: ('f', seq, t_us, step, dist, str, rev, flags) hoac ('i', seq, t_us, rev_steps, n_khung, tf_loi, rev, flags)."""
        self.buf += data
        b, i, n, pk, lines = self.buf, 0, len(self.buf), [], []
        while i < n:
            if b[i] == 0xAA:
                if i + 1 >= n:
                    break
                if b[i + 1] in (STM_FRAME, STM_INDEX):
                    if i + STM_PKT > n:
                        break
                    if crc8(b[i:i + 15]) == b[i + 15]:
                        if b[i + 1] == STM_FRAME:
                            seq, t, st, d, s, rev, fl = struct.unpack_from("<BIhHHBB", b, i + 2)
                            pk.append(("f", seq, t, st, d, s, rev, fl))
                        else:
                            seq, t, rs, nf, tl, rev, fl = struct.unpack_from("<BIhHHBB", b, i + 2)
                            pk.append(("i", seq, t, rs, nf, tl, rev, fl))
                        i += STM_PKT
                        continue
                    self.bad += 1
                i += 1
                continue
            c = b[i]
            if c == 0x0A:
                if self.text:
                    lines.append(self.text.decode(errors="ignore").strip()); self.text = bytearray()
            elif 32 <= c < 127 and len(self.text) < 200:
                self.text.append(c)
            i += 1
        self.buf = b[i:]
        return pk, lines


class StepMapper:
    """Gom khung theo so thu tu moc (rev) thanh vong quet. Cung giao dien Mapper (process, periods, sync_state, n_idx...).
    Bo vong: thieu goi moc dau/cuoi (thieu_moc), so buoc/vong lech 3200 qua 2 buoc (buoc_vong), mat > 2 % goi (mat_goi)."""

    def __init__(self, steps_per_rev=3200):
        self.spr = steps_per_rev
        self.lock = threading.Lock()
        self.cs = ClockSync(400, hull=True)               # thoi gian Nucleo (s) -> dong ho PC, chi de gan nhan thoi gian
        self.out, self.pending = [], []
        self.cur_rev, self.cur, self.cur_lost = None, [], 0
        self.first_rev = None
        self.info = {}                                     # rev -> (t_s, rev_steps, n_khung)
        self.idx_t = collections.deque(maxlen=60)
        self._t_last = self._rev_last = None              # bo tro mo vong 32 bit / 8 bit
        self._seq_last = None
        self.seq = 0
        self.n_frames = self.n_idx = 0
        self.t_last_frame = self.t_last_idx = None
        self.n_lost = 0                                    # goi mat (theo seq)
        self.dev_bad = 0                                   # loi TF03 tren Nucleo (checksum + tran)
        self.n_skip = 0
        self.skip_why = collections.Counter()

    def _unwrap_t(self, t):
        if self._t_last is None:
            self._t_last = t
            return t * 1e-6
        d = (t - (self._t_last & 0xFFFFFFFF)) & 0xFFFFFFFF
        if d >= 1 << 31:
            d -= 1 << 32
        full = self._t_last + d
        self._t_last = max(self._t_last, full)
        return full * 1e-6

    def _unwrap_rev(self, r):
        if self._rev_last is None:
            self._rev_last = r
            return r
        d = (r - (self._rev_last & 0xFF)) & 0xFF
        if d >= 128:
            d -= 256
        full = self._rev_last + d
        self._rev_last = max(self._rev_last, full)
        return full

    def add(self, pkts, t_arr):
        if not pkts:
            return
        with self.lock:
            t_s = None
            for p in pkts:
                seq = p[1]
                if self._seq_last is not None:
                    gap = (seq - self._seq_last - 1) & 0xFF
                    if gap:
                        self.n_lost += gap; self.cur_lost += gap
                self._seq_last = seq
                t_s = self._unwrap_t(p[2])
                rev = self._unwrap_rev(p[6])
                if p[0] == "i":
                    self.info[rev] = (t_s, p[3], p[4])
                    self.idx_t.append(t_s); self.n_idx += 1; self.t_last_idx = t_arr
                    self.dev_bad = p[5]
                    continue
                self.n_frames += 1; self.t_last_frame = t_arr
                if not p[7] & 1:                           # chua co moc: chua biet goc
                    continue
                if self.cur_rev is None:
                    self.cur_rev = self.first_rev = rev
                if rev != self.cur_rev:
                    self.pending.append((self.cur_rev, self.cur, self.cur_lost))
                    self.cur_rev, self.cur, self.cur_lost = rev, [], 0
                self.cur.append((p[3], p[4], p[5]))
            if t_s is not None:
                self.cs.add(t_s, t_arr)
            self._emit()

    def add_frames(self, p, t_arr):                        # cho StmReplaySource (dung chung ReplaySource.tick)
        self.add(p, t_arr)

    def add_index(self, p, t_arr):
        self.add(p, t_arr)

    def _skip(self, why):
        self.n_skip += 1; self.skip_why[why] += 1

    def _emit(self):
        keep = []
        for r, fr, lost in self.pending:
            a, b = self.info.get(r), self.info.get(r + 1)
            if a is None or b is None:
                if self.cur_rev - r > 2:                   # doi 2 vong ma van thieu goi moc
                    if r != self.first_rev:                # vong dau phien bat dau giua chung: bo im lang
                        self._skip("thieu_moc")
                else:
                    keep.append((r, fr, lost))
                continue
            T = b[0] - a[0]
            if abs(b[1] - self.spr) > 2:
                self._skip("buoc_vong"); continue
            if lost > 0.02 * max(len(fr), 1) or not fr or T <= 0:
                self._skip("mat_goi"); continue
            st = np.array([f[0] for f in fr], float)
            ang = (st % self.spr) * (360.0 / self.spr)
            t0 = float(self.cs.to_local(a[0])) if self.cs.ready() else a[0]
            self.seq += 1
            self.out.append(Scan(self.seq, t0, T, ang, np.array([f[1] for f in fr], float),
                                 np.array([f[2] for f in fr], float), len(fr), extra=dict(rev=r, n_khung_nucleo=b[2])))
        self.pending = keep
        if len(self.info) > 50:
            for k in sorted(self.info)[:-20]:
                del self.info[k]

    def process(self):
        with self.lock:
            out, self.out = self.out, []
        return out

    def periods(self):
        with self.lock:
            return np.diff(np.array(self.idx_t)[-20:]) if len(self.idx_t) >= 2 else np.array([])

    def sync_state(self):
        """Goc khong phu thuoc dong ho PC: luon 'dung gio'."""
        return 0.0, 0.0, False


class StmLiveSource(LiveSource):
    """Nucleo chay firmware nucleo_lidar: mot cong COM (ST-Link VCP). Ghi stm_raw.csv + stm_idx.csv."""

    def __init__(self, mapper, port, rps=1.0, baud=460800, logdir=None):
        super().__init__(mapper, None, port, rps, logdir=logdir)
        self.baud = baud
        self.tf_probe = None    # (baud, khung/s) TF03 do duoc luc bat dau

    def start(self):
        os.makedirs(self.logdir, exist_ok=True)
        self.flog = open(os.path.join(self.logdir, "stm_raw.csv"), "w")
        self.flog.write("k,seq,t_us,step,dist_cm,strength,rev,flags,t_pc_s\n")
        self.ilog = open(os.path.join(self.logdir, "stm_idx.csv"), "w")
        self.ilog.write("k,seq,t_us,rev_steps,n_khung,tf_loi,rev,flags,t_pc_s\n")
        self.th = [threading.Thread(target=self._guard, args=(self._stm,), daemon=True)]
        self.th[0].start()

    def _stm(self):
        import serial
        par, k, t_flush = StmParser(), 0, time.perf_counter()
        with serial.Serial(self.uno_port, self.baud, timeout=0.01) as ser:
            self.uno = ser
            # Nucleo khoi dong luon doc TF03 o 460800: do lai baud moi lan chay (TF03 co the da doi baud)
            self.tf_probe = stm_parse_probe(_stm_ask(ser, "p", ("TF03 DO",), 4.0))
            ser.write(b"b 0\n"); time.sleep(0.2); ser.reset_input_buffer()
            ser.write(b"b 1\n")
            self.set_speed(self.rps)
            try:
                while not self.stop_ev.is_set():
                    data = ser.read(max(1, ser.in_waiting))
                    if time.perf_counter() - t_flush >= 1.0:
                        self.flog.flush(); self.ilog.flush(); t_flush = time.perf_counter()
                    if not data:
                        continue
                    if ser.in_waiting:
                        data += ser.read(ser.in_waiting)
                    t_arr = time.perf_counter()
                    pkts, lines = par.feed(data)
                    self.uno_lines.extend(lines)
                    self.m.add(pkts, t_arr)
                    for p in pkts:
                        (self.flog if p[0] == "f" else self.ilog).write("%d,%s,%.6f\n" % (k, ",".join(str(v) for v in p[1:]), t_arr))
                        k += 1
                    self.bad_frames = par.bad + getattr(self.m, "dev_bad", 0)
            finally:
                self.uno = None
                try:
                    ser.write(b"r 0\nb 0\n"); time.sleep(0.2)
                except Exception:
                    pass


def is_stm_log(logdir):
    return os.path.exists(os.path.join(logdir, "stm_raw.csv"))


class StmReplaySource(ReplaySource):
    """Phat lai log cua StmLiveSource: tron hai file theo cot k (thu tu goi), gom theo t_pc nhu luc nhan."""

    def __init__(self, mapper, logdir, speed=1.0):
        self.m, self.logdir, self.speed = mapper, logdir, speed
        self.err, self.bad_frames, self.uno_lines = "", 0, collections.deque(maxlen=50)
        self.paused, self.finished = False, False
        rows = []
        for name, kind in (("stm_raw.csv", "f"), ("stm_idx.csv", "i")):
            path = os.path.join(logdir, name)
            if not os.path.exists(path):
                continue
            with open(path) as f:
                rd = csv.reader(f); next(rd)
                for r in rd:
                    if len(r) == 9:
                        rows.append((int(r[0]), float(r[8]), (kind,) + tuple(int(v) for v in r[1:8])))
        rows.sort()
        ev, chunk, t_prev = [], [], None
        for _, t, p in rows:
            if t_prev is not None and t != t_prev:
                ev.append((t_prev, 0, chunk)); chunk = []
            chunk.append(p); t_prev = t
        if chunk:
            ev.append((t_prev, 0, chunk))
        self.ev, self.k = ev, 0
        self.t0_log = ev[0][0] if ev else 0.0
        self.duration = (ev[-1][0] - self.t0_log) if ev else 0.0
        self.pos = 0.0
        self.t_wall = None
