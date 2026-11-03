"""
lidar_map_pc.py - Ban do 2D thu nghiem tren PC (chua co STM32).

Nguon du lieu:
  - TF03 qua FT232RL (vd COM12): khung 9 byte 59 59 DL DH SL SH 00 00 CHK, 1000 Hz.
  - Arduino UNO chay index_flag_test.ino (vd COM10): quay dong co, in dong "MOC 0 ..." moi lan
    la chan qua cam bien khe (moc 0 do).

Cach tinh goc (tam thoi, do STM32 chua co):
      goc = 360 * (t_khung - t_moc_truoc) / T_vong
  De khu do tre USB ngau nhien (vai ms den 16 ms), KHONG dung thang thoi diem PC nhan duoc:
  - Khung TF03: danh so thu tu n (TF03 do deu ~1000 Hz). Khop t_PC = a + b*n theo "bien duoi"
    (moi goi den PC chi co the tre, khong the som) -> thoi diem do that cua tung khung.
  - Moc 0 do: UNO gui kem t=<micros> do trong ngat (chinh xac us). Khop dong ho UNO -> dong ho PC
    cung theo bien duoi (co tinh ca sai lech tan so thach anh cua UNO).
  Phan tre CO DINH con lai chi lam xoay ca ban do mot goc khong doi -> chinh bang --offset.
  Van la ban THU: so lieu danh gia do chinh xac phai do tren STM32.

Cai dat:  pip install pyserial numpy matplotlib
Chay:     python lidar_map_pc.py --tf03 COM12 --uno COM10 --rps 1
  Phim trong cua so do thi:  s = luu anh + CSV vong hien tai,  q = thoat.
  Moi lan chay ghi them du lieu tho vao thu muc log_<thoi gian>/ de xu ly lai sau.

TRUOC KHI CHAY: dat Latency Timer cua FT232RL = 1 ms
  (Device Manager > Ports > USB Serial Port (COM12) > Properties > Port Settings > Advanced).
CODE THU NGHIEM: ban 1 da chay voi phan cung; phan dong bo dong ho moi thu bang du lieu gia lap.
"""
import argparse, collections, math, os, re, threading, time

import numpy as np


# ---------------------------------------------------------------- xu ly (khong phu thuoc serial)
def parse_tf03(buf):
    """Tach cac khung hop le tu bytearray. Tra ve (list[(dist_cm, strength)], phan du chua du khung, so khung loi)."""
    out, bad, i = [], 0, 0
    n = len(buf)
    while i + 9 <= n:
        if buf[i] == 0x59 and buf[i + 1] == 0x59:
            if (sum(buf[i:i + 8]) & 0xFF) == buf[i + 8]:
                out.append((buf[i + 2] | buf[i + 3] << 8, buf[i + 4] | buf[i + 5] << 8))
                i += 9
                continue
            bad += 1
        i += 1
    return out, buf[i:], bad


class ClockSync:
    """Khop dong ho nguon (so thu tu khung, hoac micros cua UNO) sang dong ho PC.
    Moi cap (remote, t_PC) thoa t_PC = a + b*remote + tre, voi tre >= 0. Do doc b lay bang hoi quy,
    a lay theo bien duoi (tre nho nhat) trong cua so gan nhat."""

    def __init__(self, n=400):
        self.r, self.l = collections.deque(maxlen=n), collections.deque(maxlen=n)
        self.base, self.a, self.b, self.cnt = None, None, None, 0

    def add(self, remote, local):
        if self.base is None:
            self.base = remote
        self.r.append(remote - self.base); self.l.append(local); self.cnt += 1
        if len(self.r) >= 5 and (self.b is None or self.cnt % 10 == 0):
            R, L = np.array(self.r), np.array(self.l)
            if np.ptp(R) > 0:
                self.b = float(np.polyfit(R, L, 1)[0])
                self.a = float(np.min(L - self.b * R))

    def ready(self):
        return self.b is not None

    def to_local(self, remote):
        return self.a + self.b * (np.asarray(remote, float) - self.base)


class Mapper:
    """Gan goc cho tung khung theo cac moc 0 do; gom thanh tung vong quet."""

    def __init__(self, direction=1, offset_deg=0.0, r0_m=0.0, max_cm=3000, min_strength=0):
        self.dir, self.off, self.r0 = direction, offset_deg, r0_m
        self.max_cm, self.min_str = max_cm, min_strength
        self.frames = collections.deque()          # (n, dist_cm, strength) chua gan goc, n = so thu tu khung
        self.idx = []                              # thoi diem moc 0 do theo dong ho nguon (s)
        self.cf, self.ci = ClockSync(400), ClockSync(30)   # khung TF03, moc 0 do
        self.last_rev = None                       # dict: angle_deg, r_m, strength, x, y
        self.rev_count = 0
        self.err = ""                              # loi tu luong doc serial (hien len tieu de)
        self.n_frames = 0
        self.n_idx = 0                             # tong so moc da nhan (khong bi cat nhu self.idx)
        self.lock = threading.Lock()

    def add_frames(self, items, t_arr):
        with self.lock:
            self.frames.extend(items)
            self.n_frames += len(items)
            if items:
                self.cf.add(items[-1][0], t_arr)    # khung cuoi cua goi vua den

    def add_index(self, remote, t_arr):
        with self.lock:
            self.idx.append(remote)
            self.ci.add(remote, t_arr)
            self.n_idx += 1
            if len(self.idx) > 50:
                self.idx = self.idx[-50:]

    def period(self, I):
        d = np.diff(I[-6:])
        return float(np.median(d)) if len(d) else None

    def process(self):
        """Gan goc cho cac khung nam giua 2 moc da biet. Tra ve True neu vua xong 1 vong moi."""
        with self.lock:
            if len(self.idx) < 3 or not (self.cf.ready() and self.ci.ready()):
                return False
            I = self.ci.to_local(self.idx)               # moc 0 do theo dong ho PC
            T = self.period(I)
            i0, i1, key = I[-2], I[-1], self.idx[-2]
            tf = self.cf.to_local
            # bo khung cu hon moc ap chot
            while self.frames and tf(self.frames[0][0]) < i0:
                self.frames.popleft()
            rev = []
            while self.frames and tf(self.frames[0][0]) < i1:
                rev.append(self.frames.popleft())
            if not rev or (self.last_rev is not None and self.last_rev["key"] == key):
                return False
            t = tf(np.array([f[0] for f in rev], float))
        d = np.array([f[1] for f in rev], float)
        s = np.array([f[2] for f in rev], float)
        ang = 360.0 * (t - i0) / T
        ok = (d > 0) & (d <= self.max_cm) & (s >= self.min_str) & (ang < 360.0)
        a = np.radians(self.dir * ang[ok] + self.off)
        r = d[ok] / 100.0 - self.r0
        self.last_rev = dict(key=key, T=T, angle_deg=ang[ok], r_m=r, strength=s[ok],
                             x=r * np.cos(a), y=r * np.sin(a), n_total=len(rev), n_ok=int(ok.sum()))
        self.rev_count += 1
        return True


# ---------------------------------------------------------------- doc serial (2 luong)
def guarded(fn):
    def w(*a):
        try:
            fn(*a)
        except Exception as e:                     # vd: cong dang bi Arduino IDE chiem
            msg = "%s: %s" % (fn.__name__, e)
            print("LOI", msg); a[-3].err = msg   # tham so mapper luon dung vi tri thu 3 tu cuoi
    return w


@guarded
def tf03_reader(port, baud, hz, mapper, stop, log):
    import serial
    ser = serial.Serial(port, baud, timeout=0.01)
    ser.reset_input_buffer()
    buf, bad_total, n = bytearray(), 0, 0
    while not stop.is_set():
        data = ser.read(max(1, ser.in_waiting))
        if not data:
            continue
        t_arr = time.perf_counter()
        buf += data
        frames, rest, bad = parse_tf03(buf)
        buf = bytearray(rest); bad_total += bad
        n += bad                                  # khung loi: coi nhu mat 1 khung, giu dung so thu tu
        items = []
        for d, s in frames:
            items.append((n, d, s)); n += 1
        mapper.add_frames(items, t_arr)
        for it in items:
            log.write("%d,%d,%d,%.6f\n" % (it + (t_arr,)))
    ser.close()
    print("TF03: so khung loi checksum =", bad_total)


@guarded
def uno_reader(port, rps, mapper, stop, log):
    import serial
    ser = serial.Serial(port, 115200, timeout=0.1)
    time.sleep(2.5)                      # UNO tu reset khi mo cong
    ser.reset_input_buffer()
    ser.write(("r %.2f\n" % rps).encode())
    print("UNO: da gui lenh r %.2f, cho dong co dat toc do..." % rps)
    last_us, wrap = None, 0
    while not stop.is_set():
        line = ser.readline()
        if not line:
            continue
        # tru thoi gian truyen ca dong qua UART 115200 (10 bit/ky tu): moc xay ra truoc khi dong bat dau gui
        t = time.perf_counter() - len(line) * 10 / 115200
        txt = line.decode(errors="ignore").strip()
        if txt.startswith("MOC 0"):
            m = re.search(r"t=(\d+)", txt)
            if m:                                 # micros() cua UNO, quay vong sau ~71.6 phut
                us = int(m.group(1))
                if last_us is not None and us < last_us:
                    wrap += 1 << 32
                last_us = us
                remote = (us + wrap) * 1e-6
            else:                                 # sketch cu khong co t=: dung thoi diem PC nhan
                remote = t
            mapper.add_index(remote, t)
            log.write("%.6f,%.6f\n" % (remote, t))
        elif txt:
            print("UNO:", txt)
    ser.write(b"r 0\n"); time.sleep(0.2)
    ser.close()


# ---------------------------------------------------------------- ve
def run(args):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    logdir = time.strftime("log_%Y%m%d_%H%M%S"); os.makedirs(logdir, exist_ok=True)
    flog = open(os.path.join(logdir, "tf03_raw.csv"), "w"); flog.write("n,dist_cm,strength,t_pc_s\n")
    ilog = open(os.path.join(logdir, "index.csv"), "w"); ilog.write("t_uno_s,t_pc_s\n")
    mapper = Mapper(args.dir, args.offset, args.r0, args.maxcm, args.minstr)
    stop = threading.Event()
    th = [threading.Thread(target=tf03_reader, args=(args.tf03, args.baud, args.hz, mapper, stop, flog), daemon=True),
          threading.Thread(target=uno_reader, args=(args.uno, args.rps, mapper, stop, ilog), daemon=True)]
    for x in th:
        x.start()

    fig, ax = plt.subplots(figsize=(8, 8))
    sc = ax.scatter([], [], s=4, c=[], cmap="viridis")
    ax.plot([0], [0], "r+", ms=14)
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.set_xlim(-args.view, args.view); ax.set_ylim(-args.view, args.view)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    title = ax.set_title("Dang cho moc 0 do...")

    def update(_):
        mapper.process()
        rv = mapper.last_rev
        if mapper.err:
            title.set_text("LOI: " + mapper.err[:80])
        elif rv is None:
            title.set_text("Dang cho moc 0 do... (moc: %d, khung TF03: %d)" % (mapper.n_idx, mapper.n_frames))
        if rv is not None and len(rv["x"]):
            sc.set_offsets(np.c_[rv["x"], rv["y"]])
            sc.set_array(rv["strength"])
            sc.set_clim(rv["strength"].min(), max(rv["strength"].max(), rv["strength"].min() + 1))
            title.set_text("Vong #%d | %.3f vong/s | %d/%d diem hop le"
                           % (mapper.rev_count, 1.0 / rv["T"], rv["n_ok"], rv["n_total"]))
        return sc, title

    def on_key(ev):
        rv = mapper.last_rev
        if ev.key == "s" and rv is not None:
            stem = os.path.join(logdir, "vong_%d" % mapper.rev_count)
            fig.savefig(stem + ".png", dpi=150)
            np.savetxt(stem + ".csv", np.c_[rv["angle_deg"], rv["r_m"], rv["strength"], rv["x"], rv["y"]],
                       delimiter=",", header="angle_deg,r_m,strength,x_m,y_m", comments="", fmt="%.4f")
            print("Da luu", stem + ".png/.csv")
        elif ev.key == "q":
            plt.close(fig)

    fig.canvas.mpl_connect("key_press_event", on_key)
    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()
    stop.set()
    for x in th:
        x.join(timeout=1)
    flog.close(); ilog.close()
    print("Du lieu tho da luu trong", logdir)


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Ban do 2D thu nghiem: TF03 (FT232) + moc 0 do (UNO)")
    p.add_argument("--tf03", default="COM12"); p.add_argument("--uno", default="COM10")
    p.add_argument("--baud", type=int, default=115200, help="baud TF03")
    p.add_argument("--hz", type=float, default=1000, help="tan so do TF03")
    p.add_argument("--rps", type=float, default=1.0, help="toc do quay (vong/s); 1 cho sai so goc nho nhat")
    p.add_argument("--dir", type=int, default=1, choices=(1, -1), help="dao chieu goc neu ban do bi lat guong")
    p.add_argument("--offset", type=float, default=0.0, help="xoay ban do (do) de huong 0 trung khung")
    p.add_argument("--r0", type=float, default=0.0, help="tru quang duong TF03 -> guong (m)")
    p.add_argument("--maxcm", type=int, default=3000, help="bo diem xa hon (cm)")
    p.add_argument("--minstr", type=int, default=0, help="bo diem co cuong do nho hon")
    p.add_argument("--view", type=float, default=5.0, help="nua be rong khung nhin (m)")
    run(p.parse_args())
