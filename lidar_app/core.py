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
import collections, csv, math, os, re, struct, threading, time
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
class ClockSync:
    """t_PC = a + b*remote + tre (tre >= 0). b: do doc bien duoi theo cum; a: bien duoi trong cua so."""

    def __init__(self, n=400, every=10):         # every: cap nhat a, b sau moi bay nhieu mau
        self.r, self.l = collections.deque(maxlen=n), collections.deque(maxlen=n)
        self.base = self.a = self.b = None; self.every = every
        self.cnt = 0

    def add(self, remote, local):
        if self.base is None:
            self.base = remote
        self.r.append(remote - self.base); self.l.append(local); self.cnt += 1
        if len(self.r) >= 5 and (self.b is None or self.cnt % self.every == 0):
            R, L = np.array(self.r, float), np.array(self.l, float)
            if np.ptp(R) > 0:
                k = min(8, len(R) // 2)
                edges = np.linspace(0, len(R), k + 1, dtype=int)
                b0 = self.b if self.b is not None else (L[-1] - L[0]) / (R[-1] - R[0])
                im = [u + int(np.argmin(L[u:v] - b0 * R[u:v])) for u, v in zip(edges[:-1], edges[1:]) if v > u]
                rb, lb = R[im], L[im]
                sl = [(lb[j] - lb[i]) / (rb[j] - rb[i]) for i in range(len(rb)) for j in range(i + 1, len(rb)) if rb[j] > rb[i]]
                self.b = float(np.median(sl)) if sl else float(np.polyfit(R, L, 1)[0])
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
        self.pkts = collections.deque()        # (items, t_arr, m_cum)
        self.idx = []                          # (remote, t_arr) moc 0 do
        self.cf, self.ci = ClockSync(400, every=20), ClockSync(60, every=1)
        self.done_key = None                   # moc dau cua vong da xu ly gan nhat
        self.seq = 0
        self.lock = threading.Lock()
        # thong ke
        self.n_frames = self.n_idx = 0
        self.t_last_frame = self.t_last_idx = None
        self.last_n = None                     # so thu tu khung moi nhat da nhan
        self._m_cum = 0.0
        self._t_env = None

    def add_frames(self, items, t_arr):
        if not items:
            return
        with self.lock:
            self.n_frames += len(items); self.t_last_frame = t_arr
            self.cf.add(items[-1][0], t_arr)
            self.last_n = items[-1][0]
            b = self.cf.b or 0.001
            mk = float(len(items))
            if self._t_env is None:
                self._t_env = t_arr
                self._m_cum += mk
            else:
                dt = max(0.0, t_arr - self._t_env)
                self._m_cum += min(mk, max(1.0, (dt + 0.018) / b))
                self._t_env = min(t_arr, self._t_env + mk * b)
            self.pkts.append((items, t_arr, self._m_cum))

    def add_index(self, remote, t_arr):
        with self.lock:
            self.idx.append((remote, t_arr)); self.n_idx += 1; self.t_last_idx = t_arr
            self.ci.add(remote, t_arr)
            if len(self.idx) > 60:
                self.idx = self.idx[-60:]

    def periods(self):
        with self.lock:
            if len(self.idx) < 2 or not self.ci.ready():
                return np.array([])
            return np.diff(self.ci.to_local([r for r, _ in self.idx[-20:]]))

    def process(self):
        """Tra ve danh sach Scan vua hoan thanh (co the nhieu vong khi phat lai nhanh)."""
        out = []
        with self.lock:
            if len(self.idx) < 3 or not (self.cf.ready() and self.ci.ready()) or not self.pkts:
                return out
            rem = [r for r, _ in self.idx]
            I = self.ci.to_local(rem)
            b = self.cf.b
            T_p = np.array([p[1] for p in self.pkts], float)
            M_p = np.array([p[2] for p in self.pkts], float)
            u = T_p - b * M_p
            for w in range(1, min(16, len(u))):
                u[:-w] = np.minimum(u[:-w], u[w:])
            T_end = u + b * M_p
            for j in range(len(self.idx) - 1):
                key = rem[j]
                if self.done_key is not None and key <= self.done_key:
                    continue
                if j == 0:                       # chua biet vong truoc do -> chi dung de dong bo
                    self.done_key = key
                    continue
                i0, i1 = float(I[j]), float(I[j + 1])
                if T_end[-1] < i1 + 0.02:        # cho them vai goi sau i1 de khu tre goi cuoi vong
                    break
                T = i1 - i0
                d_per = np.diff(I[max(0, j - 5):j + 2])
                T_med = float(np.median(d_per))
                while len(self.pkts) > 1 and T_end[0] < i0:
                    self.pkts.popleft(); T_end = T_end[1:]
                ft, fd, fs = [], [], []
                for (items, _, _), te in zip(self.pkts, T_end):
                    k = len(items)
                    if te - k * b >= i1:
                        break
                    for r, (_, d, s) in enumerate(items):
                        tf = te - (k - 1 - r) * b
                        if i0 <= tf < i1:
                            ft.append(tf); fd.append(d); fs.append(s)
                self.done_key = key
                t_idx0, t_idx1 = self.idx[j][1], self.idx[j + 1][1]
                # bo qua vong dang tang/giam toc (>3%) hoac bi treo PC (>80 ms mat du lieu)
                if abs(T - T_med) / T_med > 0.03 or max(t_idx0 - i0, t_idx1 - i1) > 0.1 or not ft:
                    continue
                ft = np.array(ft, float)
                if np.max(np.diff(np.r_[i0, ft, i1])) > 0.08:
                    continue
                ang = 360.0 * (ft - i0) / T
                self.seq += 1
                out.append(Scan(self.seq, i0, T, ang, np.array(fd, float), np.array(fs, float), len(ft)))
        return out


def to_xy(scan, r0=0.0, offset_deg=0.0, direction=1, max_cm=3000, min_str=0):
    """Loc diem hop le va doi sang (x, y) met. Tra ve dict."""
    d, s, a = scan.dist_cm, scan.strength, scan.angle_deg
    ok = (d > 0) & (d <= max_cm) & (s >= min_str) & (a >= 0) & (a < 360)
    th = np.radians(direction * a[ok] + offset_deg)
    d_m = d[ok] / 100.0
    r = d_m - r0
    return dict(x=r * np.cos(th), y=r * np.sin(th), r=r, d_m=d_m, ang=a[ok], s=s[ok], ok=ok)


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
            buf, n = bytearray(), 0
            while not self.stop_ev.is_set():
                data = ser.read(max(1, ser.in_waiting))
                if not data:
                    continue
                if ser.in_waiting:               # hut het byte den trong luc cho GIL truoc khi lay moc thoi gian
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
                        self.ilog.write("%.6f,%.6f\n" % (remote, t))
                    elif txt:
                        self.uno_lines.append(txt)
            finally:
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
