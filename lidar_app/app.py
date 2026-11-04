"""
app.py - Ung dung giam sat LiDAR 2D (ban thu tren PC, chua co STM32).

Chuc nang:
  Dot 1: chon nguon Live (TF03 + UNO) hoac Phat lai thu muc log; bang trang thai va canh bao;
         chinh r0 / offset / chieu / loc diem ngay khi dang chay; cong don N vong; luu anh + CSV.
  Dot 2: ve vung cam bang chuot, hoc nen, gom cum vat the, bao dong (mau do + tieng beep),
         nhat ky su kien kem anh chup.
  Dot 3 (mot phan): do baud, dat baud va tan so TF03 khi dang dung.
  Cau hinh: ty lan chuot khong doi thong so, khoa hieu chinh, vung cam di theo Xoay / Dao chieu,
            tu luu settings khi doi, luu va nap nen.

Cai dat:  pip install PySide6 pyqtgraph pyserial numpy
Chay:     python app.py
CODE THU NGHIEM: da chay voi log gia lap va chay Live voi UNO + TF03 that (04/11); chua dung lam so lieu danh gia.
"""
import json, math, os, struct, subprocess, sys, tempfile, time, wave

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

import core
import detect

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS = os.path.join(HERE, "settings.json")
BG_FILE = os.path.join(HERE, "nen.npz")
BEEP_ZONE, BEEP_MOVE, BEEP_OFF = "Có vật trong vùng", "Có vật đang đi", "Tắt"
DEFAULTS = dict(tf_port="COM12", uno_port="COM10", tf_baud=115200, tf_hz=1000, rps=1.0, r0=0.0, offset=0.0, dir=1, maxcm=3000, minstr=0,
                accum=1, color="cường độ", view=5.0, zone=[], margin=0.15, min_size=0.10, on_revs=3,
                learn_revs=20, replay_dir="", replay_speed=1.0, lock_cal=False, merge_d=0.6, merge_dv=0.25, hold_s=60.0, beep_mode="Có vật trong vùng")


class BeepPlayer:
    """Phat lap tieng keo dai (long) hoac ngat quang (pulse) qua loa, khong lam dung giao dien. Khong co winsound thi ok = False."""
    RATE = 22050

    def __init__(self):
        self.state, self.ok, self.files = "off", False, {}
        try:
            import winsound
            self.ws = winsound
            d = tempfile.gettempdir()
            self.files = dict(long=os.path.join(d, "lidar_app_bip_dai.wav"), pulse=os.path.join(d, "lidar_app_bip_ngat.wav"))
            self._wav(self.files["long"], [(1000, 1.0)])          # so chu ky nguyen, lap lai lien mach
            self._wav(self.files["pulse"], [(1500, 0.15), (0, 0.15)])
            self.ok = True
        except Exception:
            pass

    def _wav(self, path, parts):
        data = bytearray()
        for hz, sec in parts:
            n = int(self.RATE * sec)
            for i in range(n):
                edge = min(1.0, i / 100.0, (n - i) / 100.0) if len(parts) > 1 else 1.0   # tieng ngan: vuot mep tranh tieng tach
                data += struct.pack("<h", int(14000 * edge * math.sin(2 * math.pi * hz * i / self.RATE)) if hz else 0)
        with wave.open(path, "wb") as f:
            f.setnchannels(1); f.setsampwidth(2); f.setframerate(self.RATE); f.writeframes(bytes(data))

    def set(self, state):
        if state == self.state:
            return
        self.state = state
        if not self.ok:
            return
        if state == "off":
            self.ws.PlaySound(None, self.ws.SND_PURGE)
        else:
            self.ws.PlaySound(self.files[state], self.ws.SND_FILENAME | self.ws.SND_ASYNC | self.ws.SND_LOOP)


def code_version():
    """Phien ban code de ghi vao session.json; khong co git thi tra ve chuoi rong."""
    try:
        return subprocess.run(["git", "-C", HERE, "describe", "--always", "--dirty"], capture_output=True, text=True, timeout=2,
                              creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout.strip()
    except Exception:
        return ""


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [p.device for p in lp.comports()]
    except Exception:
        return []


def dspin(lo, hi, step, val, dec=2):
    w = QtWidgets.QDoubleSpinBox(); w.setDecimals(dec); w.setRange(lo, hi); w.setSingleStep(step); w.setValue(val)
    return w


def ispin(lo, hi, step, val):
    w = QtWidgets.QSpinBox(); w.setRange(lo, hi); w.setSingleStep(step); w.setValue(val)
    return w


def combo(editable=False):
    w = QtWidgets.QComboBox(); w.setEditable(editable)
    return w


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR 2D - giám sát vật cản (bản thử)")
        self.cfg = dict(DEFAULTS)
        if os.path.exists(SETTINGS):
            try:
                self.cfg.update(json.load(open(SETTINGS)))
            except Exception:
                pass
        self.mapper = core.Mapper()
        self.src = None
        self.det = detect.Detector()
        self.det.zone = [tuple(p) for p in self.cfg["zone"]]
        self.recent = []                    # cac vong da doi sang xy (de cong don)
        self.last_scan = None
        self.drawing = False
        self.draft = []
        self.n_ev_shown = 0
        self.rate_hist = []                 # (t, n_frames) tinh toc do khung
        self.alog = None                    # log moc thoi gian bao dong
        self.tracker, self.tlog, self.elog = detect.Tracker(), None, None
        self.session = None                 # thong so cua phien do dang chay
        self.t_start, self.wall_off, self.entry_view = 0.0, None, []   # wall_off: gio he thong - dong ho nguon (Truc tiep)
        self._save_timer = QtCore.QTimer(self, singleShot=True, interval=500, timeout=self._save_settings)
        self._build_ui()
        self._apply_detect_params()
        self._draw_zone()
        self.timer = QtCore.QTimer(self, interval=50, timeout=self.on_tick)
        self.timer.start()
        self.player = BeepPlayer()
        self.beep_timer = QtCore.QTimer(self, interval=200, timeout=self._beep_tick)
        self.beep_timer.start()

    # ------------------------------------------------------------ giao dien
    def _build_ui(self):
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget(background="w")
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "x (m)"); self.plot.setLabel("left", "y (m)")
        v = self.cfg["view"]; self.plot.setRange(xRange=(-v, v), yRange=(-v, v))
        self.sc = pg.ScatterPlotItem(size=4, pen=None)
        self.sc_fg = pg.ScatterPlotItem(size=7, pen=None, brush=pg.mkBrush(220, 30, 30))
        self.zone_item = pg.PlotDataItem(pen=pg.mkPen((230, 120, 0), width=2), fillLevel=None)
        self.zone_fill = pg.PlotDataItem(pen=None)
        self.draft_item = pg.PlotDataItem(pen=pg.mkPen((230, 120, 0), width=1, style=QtCore.Qt.DashLine),
                                          symbol="o", symbolSize=6, symbolBrush=(230, 120, 0))
        self.obj_items = []
        origin = pg.ScatterPlotItem([0], [0], symbol="+", size=18, pen=pg.mkPen("r", width=2))
        self.sel_mark = pg.ScatterPlotItem(symbol="x", size=18, pen=pg.mkPen((120, 0, 160), width=3), brush=pg.mkBrush(120, 0, 160))
        self.sel_text = pg.TextItem("", color=(120, 0, 160), anchor=(0, 1))     # diem cuoi cua dong dang chon trong bang
        for it in (self.zone_item, self.sc, self.sc_fg, self.draft_item, origin, self.sel_mark, self.sel_text):
            self.plot.addItem(it)
        self.plot.scene().sigMouseClicked.connect(self.on_click)
        self.lb_ov = QtWidgets.QLabel(self.plot)        # thoi gian va so doi tuong, nam tren ban do
        self.lb_ov.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.lb_ov.setStyleSheet("background:rgba(255,255,255,210);color:#222;font-size:15px;font-weight:bold;padding:4px 8px;border-radius:4px")
        self.lb_ov.move(70, 10); self.lb_ov.hide()
        self.lut = np.array([pg.mkBrush(c) for c in pg.colormap.get("viridis").map(np.linspace(0, 1, 64), mode="qcolor")],
                            dtype=object)        # bang mau dung san: tao brush tung diem qua cham, lam tre luong doc COM

        side = QtWidgets.QWidget(); lay = QtWidgets.QVBoxLayout(side)
        side.setMinimumWidth(330); side.setMaximumWidth(380)

        # --- nguon
        g = QtWidgets.QGroupBox("1. Nguồn dữ liệu"); f = QtWidgets.QFormLayout(g)
        self.rb_live = QtWidgets.QRadioButton("Trực tiếp (TF03 + UNO)")
        self.rb_replay = QtWidgets.QRadioButton("Phát lại thư mục log")
        self.rb_replay.setChecked(True) if self.cfg["replay_dir"] else self.rb_live.setChecked(True)
        f.addRow(self.rb_live); f.addRow(self.rb_replay)
        self.cb_tf = combo(True); self.cb_uno = combo(True)
        self._fill_ports()
        bt_ref = QtWidgets.QPushButton("Quét cổng"); bt_ref.clicked.connect(self._fill_ports)
        f.addRow("COM TF03", self.cb_tf); f.addRow("COM UNO", self.cb_uno); f.addRow(bt_ref)
        self.sp_rps = dspin(0.1, 5, 0.25, self.cfg["rps"])
        self.sp_rps.valueChanged.connect(lambda v: self.src and isinstance(self.src, core.LiveSource) and self.src.set_speed(v))
        f.addRow("Tốc độ quay (vòng/s)", self.sp_rps)
        h = QtWidgets.QHBoxLayout()
        self.le_dir = QtWidgets.QLineEdit(self.cfg["replay_dir"]); bt_dir = QtWidgets.QPushButton("...")
        bt_dir.clicked.connect(self._pick_dir); h.addWidget(self.le_dir); h.addWidget(bt_dir)
        f.addRow("Thư mục log", h)
        self.cb_speed = QtWidgets.QComboBox(); self.cb_speed.addItems(["0.5", "1", "2", "5", "10"])
        self.cb_speed.setCurrentText(str(self.cfg["replay_speed"]).rstrip("0").rstrip(".") or "1")
        self.cb_speed.currentTextChanged.connect(lambda t: self.src and isinstance(self.src, core.ReplaySource) and self.src.set_speed(float(t)))
        f.addRow("Tốc độ phát lại (x)", self.cb_speed)
        h2 = QtWidgets.QHBoxLayout()
        self.bt_start = QtWidgets.QPushButton("Bắt đầu"); self.bt_start.clicked.connect(self.toggle_run)
        self.bt_pause = QtWidgets.QPushButton("Tạm dừng"); self.bt_pause.clicked.connect(self.toggle_pause)
        h2.addWidget(self.bt_start); h2.addWidget(self.bt_pause); f.addRow(h2)
        lay.addWidget(g)

        # --- cau hinh TF03
        g = QtWidgets.QGroupBox("TF03 (bấm Dừng trước khi đổi)"); f = QtWidgets.QFormLayout(g)
        self.cb_baud = QtWidgets.QComboBox(); self.cb_baud.addItems([str(b) for b in core.TF03_BAUDS])
        self.cb_baud.setCurrentText(str(self.cfg["tf_baud"]))
        f.addRow("Baud hiện tại", self.cb_baud)
        bt_probe = QtWidgets.QPushButton("Dò TF03"); bt_probe.clicked.connect(self.tf03_probe)
        self.lb_tf = QtWidgets.QLabel("-"); self.lb_tf.setWordWrap(True)
        f.addRow(bt_probe, self.lb_tf)
        h = QtWidgets.QHBoxLayout()
        self.cb_hz = QtWidgets.QComboBox(); self.cb_hz.addItems([str(r) for r in core.TF03_RATES])
        self.cb_hz.setCurrentText(str(self.cfg["tf_hz"]))
        bt_hz = QtWidgets.QPushButton("Đặt tần số"); bt_hz.clicked.connect(self.tf03_set_rate)
        h.addWidget(self.cb_hz); h.addWidget(bt_hz); f.addRow("Tần số (Hz)", h)
        h = QtWidgets.QHBoxLayout()
        self.cb_newbaud = QtWidgets.QComboBox(); self.cb_newbaud.addItems([str(b) for b in core.TF03_BAUDS])
        self.cb_newbaud.setCurrentText(str(self.cfg["tf_baud"]))
        bt_baud = QtWidgets.QPushButton("Đặt baud"); bt_baud.clicked.connect(self.tf03_set_baud)
        h.addWidget(self.cb_newbaud); h.addWidget(bt_baud); f.addRow("Baud mới", h)
        lay.addWidget(g)

        # --- trang thai
        g = QtWidgets.QGroupBox("2. Trạng thái"); grid = QtWidgets.QGridLayout(g)
        self.st = {}
        for i, (k, name) in enumerate([("rps", "Tốc độ quay"), ("jit", "Dao động chu kỳ"), ("pts", "Điểm / vòng"),
                                       ("valid", "Điểm hợp lệ"), ("rate", "Khung TF03 / s"), ("bad", "Khung lỗi"),
                                       ("idx", "Số mốc 0°"), ("rev", "Vòng"), ("time", "Thời gian")]):
            grid.addWidget(QtWidgets.QLabel(name), i, 0)
            self.st[k] = QtWidgets.QLabel("-"); self.st[k].setAlignment(QtCore.Qt.AlignRight)
            grid.addWidget(self.st[k], i, 1)
        self.lb_warn = QtWidgets.QLabel(""); self.lb_warn.setWordWrap(True)
        self.lb_warn.setStyleSheet("color:#b00020;font-weight:bold")
        grid.addWidget(self.lb_warn, 9, 0, 1, 2)
        lay.addWidget(g)

        # --- hien thi
        g = QtWidgets.QGroupBox("3. Hiển thị / hiệu chỉnh"); f = QtWidgets.QFormLayout(g)
        self.sp_r0 = dspin(-1, 1, 0.005, self.cfg["r0"], 3)
        self.sp_off = dspin(-360, 360, 1, self.cfg["offset"], 1)
        self.ck_dir = QtWidgets.QCheckBox("Đảo chiều (lật gương)"); self.ck_dir.setChecked(self.cfg["dir"] == -1)
        self.sp_max = ispin(50, 18000, 100, self.cfg["maxcm"])
        self.sp_str = ispin(0, 65535, 10, self.cfg["minstr"])
        self.sp_acc = ispin(1, 20, 1, self.cfg["accum"])
        self.cb_color = QtWidgets.QComboBox(); self.cb_color.addItems(["cường độ", "khoảng cách"])
        self.cb_color.setCurrentText(self.cfg["color"])
        for w_, lab in ((self.sp_r0, "r0: TF03 → gương (m)"), (self.sp_off, "Xoay (độ)"), (self.ck_dir, ""),
                        (self.sp_max, "Bỏ điểm xa hơn (cm)"), (self.sp_str, "Cường độ tối thiểu"),
                        (self.sp_acc, "Cộng dồn (vòng)"), (self.cb_color, "Tô màu theo")):
            f.addRow(lab, w_) if lab else f.addRow(w_)
        self._cal = self._cal_now()
        self.sp_off.valueChanged.connect(self._on_cal); self.ck_dir.toggled.connect(self._on_cal)   # doi vung truoc khi ve lai
        for w_ in (self.sp_r0, self.sp_off, self.sp_max, self.sp_str, self.sp_acc):
            w_.valueChanged.connect(self._redraw_recent)
        self.ck_dir.toggled.connect(self._redraw_recent); self.cb_color.currentTextChanged.connect(self._redraw_recent)
        self.ck_lock = QtWidgets.QCheckBox("Khóa hiệu chỉnh (r0, Xoay, chiều)")
        self.ck_lock.toggled.connect(self._apply_lock); self.ck_lock.setChecked(self.cfg["lock_cal"])
        f.addRow(self.ck_lock)
        h = QtWidgets.QHBoxLayout()
        b1 = QtWidgets.QPushButton("Lưu ảnh"); b1.clicked.connect(self.save_png)
        b2 = QtWidgets.QPushButton("Lưu CSV vòng này"); b2.clicked.connect(self.save_csv)
        h.addWidget(b1); h.addWidget(b2); f.addRow(h)
        lay.addWidget(g)

        # --- giam sat
        g = QtWidgets.QGroupBox("4. Giám sát vùng cấm"); f = QtWidgets.QFormLayout(g)
        h = QtWidgets.QHBoxLayout()
        self.bt_draw = QtWidgets.QPushButton("Vẽ vùng cấm"); self.bt_draw.setCheckable(True)
        self.bt_draw.toggled.connect(self.toggle_draw)
        bt_clr = QtWidgets.QPushButton("Xóa vùng"); bt_clr.clicked.connect(self.clear_zone)
        h.addWidget(self.bt_draw); h.addWidget(bt_clr); f.addRow(h)
        h = QtWidgets.QHBoxLayout()
        self.sp_learn = ispin(3, 200, 1, self.cfg["learn_revs"])
        bt_learn = QtWidgets.QPushButton("Học nền"); bt_learn.clicked.connect(self.learn_bg)
        self.ck_bg = QtWidgets.QCheckBox("Dùng nền"); self.ck_bg.setChecked(True)
        self.ck_bg.toggled.connect(self._apply_detect_params)
        h.addWidget(bt_learn); h.addWidget(self.sp_learn); h.addWidget(QtWidgets.QLabel("vòng")); h.addWidget(self.ck_bg)
        f.addRow(h)
        bt_loadbg = QtWidgets.QPushButton("Nạp nền đã lưu"); bt_loadbg.clicked.connect(self.load_bg)
        f.addRow(bt_loadbg)
        self.sp_margin = dspin(0.02, 2, 0.05, self.cfg["margin"])
        self.sp_size = dspin(0.0, 5, 0.05, self.cfg["min_size"])
        self.sp_on = ispin(1, 50, 1, self.cfg["on_revs"])
        self.sp_mg_d = dspin(0, 3, 0.05, self.cfg["merge_d"])
        self.sp_mg_dv = dspin(0, 3, 0.05, self.cfg["merge_dv"])
        self.sp_hold = dspin(0, 3600, 10, self.cfg["hold_s"], 0)
        for w_, lab in ((self.sp_margin, "Gần hơn nền (m)"), (self.sp_size, "Vật nhỏ nhất (m)"), (self.sp_on, "Báo khi liên tiếp (vòng)"),
                        (self.sp_mg_d, "Gộp khi cách dưới (m), 0 = tắt"), (self.sp_mg_dv, "Gộp khi lệch tốc độ dưới (m/s)"),
                        (self.sp_hold, "Giữ vật biến mất (s), 0 = tắt")):
            f.addRow(lab, w_); w_.valueChanged.connect(self._apply_detect_params)
        self.lb_alarm = QtWidgets.QLabel("CHƯA CÓ VÙNG CẤM"); self.lb_alarm.setAlignment(QtCore.Qt.AlignCenter)
        self.lb_alarm.setMinimumHeight(44); self._set_alarm_style(None)
        f.addRow(self.lb_alarm)
        bt_ack = QtWidgets.QPushButton("Xác nhận vùng trống"); bt_ack.clicked.connect(self.ack_clear)
        f.addRow(bt_ack)
        self.cb_beep = QtWidgets.QComboBox(); self.cb_beep.addItems([BEEP_ZONE, BEEP_MOVE, BEEP_OFF])
        self.cb_beep.setCurrentText(self.cfg["beep_mode"])
        f.addRow("Bíp liên tục", self.cb_beep)
        self.lst = QtWidgets.QListWidget(); self.lst.setMaximumHeight(140)
        f.addRow(self.lst)
        self.lb_tracks = QtWidgets.QLabel("Đối tượng: chưa có"); self.lb_tracks.setWordWrap(True)
        f.addRow(self.lb_tracks)
        lay.addWidget(g)
        lay.addStretch(1)

        # --- lich su doi tuong, ben duoi ban do
        hist = QtWidgets.QGroupBox("Lịch sử đối tượng vào vùng"); v = QtWidgets.QVBoxLayout(hist)
        h = QtWidgets.QHBoxLayout()
        self.lb_entry = QtWidgets.QLabel("Tổng lượt vào: 0"); self.lb_entry.setStyleSheet("font-weight:bold")
        bt_clr_e = QtWidgets.QPushButton("Xóa bảng"); bt_clr_e.clicked.connect(self.clear_entries)
        h.addWidget(self.lb_entry, 1); h.addWidget(bt_clr_e)
        v.addLayout(h)
        heads = ["Đối tượng", "Xuất hiện", "Vào vùng", "Ra / mất", "Trong vùng (s)", "Tốc độ vào (m/s)", "Tốc độ TB (m/s)",
                 "Tốc độ lớn nhất (m/s)", "Rộng (m)", "Điểm cuối (x, y)", "Số lần mất", "Trạng thái"]
        self.tb_entry = QtWidgets.QTableWidget(0, len(heads)); self.tb_entry.setHorizontalHeaderLabels(heads)
        self.tb_entry.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        self.tb_entry.horizontalHeader().setStretchLastSection(True); self.tb_entry.verticalHeader().hide()
        self.tb_entry.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.tb_entry.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.tb_entry.cellClicked.connect(self._on_entry_click)
        v.addWidget(self.tb_entry)

        spins = side.findChildren(QtWidgets.QSpinBox) + side.findChildren(QtWidgets.QDoubleSpinBox)
        combos = side.findChildren(QtWidgets.QComboBox)
        for w_ in spins + combos:                       # lan chuot chi cuon bang, khong doi thong so
            w_.installEventFilter(self)
            w_.setFocusPolicy(QtCore.Qt.StrongFocus)
        for w_ in spins:                                # tu luu khi doi
            w_.valueChanged.connect(self._mark_dirty)
        for w_ in combos:
            w_.currentTextChanged.connect(self._mark_dirty)
        for w_ in side.findChildren(QtWidgets.QCheckBox):
            w_.toggled.connect(self._mark_dirty)
        self.le_dir.textChanged.connect(self._mark_dirty)

        scroll =QtWidgets.QScrollArea(); scroll.setWidget(side); scroll.setWidgetResizable(True)
        left = QtWidgets.QSplitter(QtCore.Qt.Vertical); left.addWidget(self.plot); left.addWidget(hist)
        left.setStretchFactor(0, 3); left.setStretchFactor(1, 1); left.setSizes([640, 220])
        split = QtWidgets.QSplitter(); split.addWidget(left); split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        self.setCentralWidget(split)
        self.resize(1300, 860)

    def eventFilter(self, obj, ev):
        if ev.type() == QtCore.QEvent.Wheel and isinstance(obj, (QtWidgets.QAbstractSpinBox, QtWidgets.QComboBox)):
            ev.ignore()                                 # chuyen len khung cuon
            return True
        return super().eventFilter(obj, ev)

    def _mark_dirty(self, *_):
        self._save_timer.start()

    def _apply_lock(self, on):
        for w_ in (self.sp_r0, self.sp_off, self.ck_dir):
            w_.setEnabled(not on)

    def _cal_now(self):
        return self.sp_off.value(), -1 if self.ck_dir.isChecked() else 1

    def _on_cal(self, *_):
        new = self._cal_now()
        self.det.zone = detect.transform_zone(self.det.zone, *self._cal, *new)
        if self.draft:
            self.draft = detect.transform_zone(self.draft, *self._cal, *new)
            self.draft_item.setData([p[0] for p in self.draft], [p[1] for p in self.draft])
        self._cal = new
        self.tracker.reset(keep_entries=True)           # toa do cu khong con noi tiep voi toa do moi
        self._drain_entries()
        self._draw_zone()

    def _fill_ports(self):
        ports = list_ports()
        for cb, key in ((self.cb_tf, "tf_port"), (self.cb_uno, "uno_port")):
            cur = cb.currentText() or self.cfg[key]
            cb.clear(); cb.addItems(sorted(set(ports + [self.cfg[key]])))
            cb.setCurrentText(cur)

    def _pick_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(self, "Chọn thư mục log_...", self.le_dir.text() or HERE)
        if d:
            self.le_dir.setText(d); self.rb_replay.setChecked(True)

    # ------------------------------------------------------------ chay / dung
    def toggle_run(self):
        if self.src:
            self.tracker.close_open(); self._drain_entries(); self._update_entries()
            self._end_session()
            self.src.stop(); self.src = None
            self.lb_ov.hide()
            self.bt_start.setText("Bắt đầu"); return
        self.mapper = core.Mapper(); self.recent = []; self.rate_hist = []
        try:
            if self.rb_replay.isChecked():
                self.src = core.ReplaySource(self.mapper, self.le_dir.text(), float(self.cb_speed.currentText()))
            else:
                self.src = core.LiveSource(self.mapper, self.cb_tf.currentText(), self.cb_uno.currentText(),
                                           self.sp_rps.value(), tf_baud=int(self.cb_baud.currentText()),
                                           logdir=os.path.join(HERE, time.strftime("log_%Y%m%d_%H%M%S")))
            self.src.start()
            name = "alarm_replay.csv" if isinstance(self.src, core.ReplaySource) else "alarm.csv"   # phat lai khong tron voi so do that
            self.alog = detect.AlarmLog(os.path.join(self.src.logdir, name))
            self.tlog = detect.TrackLog(os.path.join(self.src.logdir, name.replace("alarm", "tracks")))
            self.elog = detect.EntryLog(os.path.join(self.src.logdir, name.replace("alarm", "luot_vao")))
            self.tracker.reset()
            self.t_start = self.src.now()
            self.wall_off = None if isinstance(self.src, core.ReplaySource) else time.time() - self.src.now()
            self.sel_mark.setData([], []); self.sel_text.setText(""); self._update_entries()
            self._read_ui()
            replay = isinstance(self.src, core.ReplaySource)            # phat lai khong ghi de thong so cua lan do that
            self.session = core.SessionLog(os.path.join(self.src.logdir, "session_replay.json" if replay else "session.json"),
                                           self.cfg, "Phát lại" if replay else "Trực tiếp", code_version())
            self.lb_ov.show()
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Lỗi", str(e)); self.src = None; return
        self.bt_start.setText("Dừng")

    def toggle_pause(self):
        if isinstance(self.src, core.ReplaySource):
            self.src.paused = not self.src.paused
            self.bt_pause.setText("Tiếp tục" if self.src.paused else "Tạm dừng")

    # ------------------------------------------------------------ cau hinh TF03
    def _tf03_run(self, fn, *args):
        """Chay lenh cau hinh tren cong TF03. Tra ve (ok, ket qua); cong dang dung thi tu choi."""
        if self.src:
            QtWidgets.QMessageBox.warning(self, "TF03", "Hãy bấm Dừng trước, vì cổng TF03 đang được dùng.")
            return False, None
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try:
            return True, fn(self.cb_tf.currentText(), *args)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "TF03", str(e))
            return False, None
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()

    def tf03_probe(self):
        ok, res = self._tf03_run(core.tf03_probe)
        if not ok:
            return
        if res is None:
            self.lb_tf.setText("Không thấy dữ liệu TF03 ở các baud đã thử.")
            return
        self.cb_baud.setCurrentText(str(res[0]))
        self.lb_tf.setText("%d baud, khoảng %.0f Hz" % res)

    def tf03_set_rate(self):
        hz = int(self.cb_hz.currentText())
        if hz > 1000 and QtWidgets.QMessageBox.question(self, "TF03", "Trên 1000 Hz chưa được kiểm chứng với TF03 này. Vẫn đặt?") \
                != QtWidgets.QMessageBox.Yes:
            return
        ok, res = self._tf03_run(core.tf03_set_rate, int(self.cb_baud.currentText()), hz)
        if ok:
            saved, rate, bad = res
            self.lb_tf.setText("Đo lại: khoảng %.0f Hz, %d khung lỗi%s" % (rate, bad, "" if saved else ". TF03 không xác nhận lưu!"))

    def tf03_set_baud(self):
        new = int(self.cb_newbaud.currentText())
        if QtWidgets.QMessageBox.question(self, "TF03", "Đổi TF03 sang %d baud? Sau đó phải rút nguồn TF03 rồi cắm lại." % new) \
                != QtWidgets.QMessageBox.Yes:
            return
        ok, saved = self._tf03_run(core.tf03_set_baud, int(self.cb_baud.currentText()), new)
        if ok:
            self.lb_tf.setText("Đã gửi lệnh. Rút nguồn TF03, cắm lại, rồi bấm Dò TF03." if saved
                               else "TF03 không xác nhận lưu. Rút nguồn, cắm lại, rồi bấm Dò TF03 để kiểm tra.")

    # ------------------------------------------------------------ vong lap chinh
    def on_tick(self):
        if not self.src:
            return
        self.src.tick()
        scans = self.mapper.process()
        t_ready = self.src.now()
        for sc in scans:
            xy = core.to_xy(sc, **self._xy_params())
            was_learning = self.det.bg.learning_left > 0
            self.det.hold = self.tracker.hold_active(self.det.zone)
            objs, fg, newly = self.det.step(xy)
            if was_learning and self.det.bg.ready():
                self.session.note("hoc_nen", self.src.now() - self.t_start, so_vong=self.sp_learn.value(), vong_s=round(1.0 / sc.period, 3))
                if isinstance(self.src, core.LiveSource):
                    self._save_bg(sc)                   # phat lai khong ghi de nen cua phan cung
            self.alog.on_scan(sc, objs, newly, self.det, t_ready, self.src.now())
            self.tlog.write(sc, self.tracker.update(sc, self.det.all_objs, self.det.zone))
            self._drain_tracker_events(sc)
            self._drain_entries()
            xy["fg"], xy["objs"] = fg, objs
            self.last_scan = sc
            self.recent.append((sc, xy))
            self.recent = self.recent[-20:]
            if newly:
                if self.cb_beep.currentText() == BEEP_OFF:
                    QtWidgets.QApplication.beep()       # con lai da co tieng keo dai
                self._snapshot()
                self.alog.mark_snap(self.src.now())
        if scans:
            self._draw()
            self._update_entries()
        self._update_status()
        self._update_events()
        if self.alog.row:
            self.lb_alarm.repaint()                     # ve ngay de t_shown sat luc o chuyen do
            self.alog.finish(self.src.now())
        if isinstance(self.src, core.ReplaySource) and self.src.finished and not scans:
            self.lb_warn.setText("Phát lại xong.")

    def _xy_params(self):
        return dict(r0=self.sp_r0.value(), offset_deg=self.sp_off.value(), direction=-1 if self.ck_dir.isChecked() else 1,
                    max_cm=self.sp_max.value(), min_str=self.sp_str.value())

    def _redraw_recent(self, *_):
        p = self._xy_params()
        self.recent = [(sc, dict(core.to_xy(sc, **p), fg=np.zeros(0, bool), objs=[])) for sc, _ in self.recent]
        self._draw()

    def _draw(self):
        if not self.recent:
            return
        sel = self.recent[-self.sp_acc.value():]
        x = np.concatenate([v["x"] for _, v in sel]); y = np.concatenate([v["y"] for _, v in sel])
        val = np.concatenate([v["s"] if self.cb_color.currentText() == "cường độ" else v["r"] for _, v in sel])
        if len(val):
            lo, hi = np.percentile(val, 2), np.percentile(val, 98)
            idx = (np.clip((val - lo) / max(hi - lo, 1e-9), 0, 1) * (len(self.lut) - 1)).astype(int)
            self.sc.setData(x, y, brush=self.lut[idx])
        last = sel[-1][1]
        fg = last.get("fg", np.zeros(0, bool))
        if len(fg) == len(last["x"]) and fg.any():
            self.sc_fg.setData(last["x"][fg], last["y"][fg])
        else:
            self.sc_fg.setData([], [])
        for it in self.obj_items:
            self.plot.removeItem(it)
        self.obj_items = []
        for o in last.get("objs", []):
            r = max(o.size / 2 + 0.1, 0.15)
            c = QtWidgets.QGraphicsEllipseItem(o.cx - r, o.cy - r, 2 * r, 2 * r)
            c.setPen(pg.mkPen((220, 30, 30), width=2)); self.plot.addItem(c); self.obj_items.append(c)
            t = pg.TextItem("%.2f m" % o.size, color=(220, 30, 30), anchor=(0, 1)); t.setPos(o.cx + r, o.cy + r)
            self.plot.addItem(t); self.obj_items.append(t)
        lines = []
        for tr in self.tracker.confirmed():
            col = (230, 120, 0) if tr.lost_since is not None else (220, 30, 30) if tr.in_zone else (30, 90, 200)
            path = pg.PlotDataItem(tr.x, tr.y, pen=pg.mkPen(col, width=2))
            self.plot.addItem(path); self.obj_items.append(path)
            txt = self._track_text(tr)
            if tr.lost_since is not None:                # danh dau diem cuoi truoc khi mat tin hieu
                mark = pg.ScatterPlotItem([tr.x[-1]], [tr.y[-1]], symbol="x", size=18, pen=pg.mkPen(col, width=3), brush=pg.mkBrush(*col))
                self.plot.addItem(mark); self.obj_items.append(mark)
                txt += "\nđiểm cuối (%.2f, %.2f)" % (tr.x[-1], tr.y[-1])
            lab = pg.TextItem(txt, color=col, anchor=(0, 0)); lab.setPos(tr.x[-1], tr.y[-1])
            self.plot.addItem(lab); self.obj_items.append(lab)
            lines.append("%s, rộng %.2f m%s" % (self._track_text(tr), tr.size, ", trong vùng %.0f s" % (tr.t[-1] - tr.zone_since) if tr.in_zone else ""))
        self.lb_tracks.setText("Đối tượng: %d" % len(self.tracker.visible()) + ("\n" + "\n".join(lines) if lines else ""))

    @staticmethod
    def _track_text(tr):
        gop = " (gộp %d)" % (tr.merged + 1) if tr.merged else ""
        if tr.lost_since is not None:
            return "#%d%s mất tín hiệu %.0f s" % (tr.id, gop, tr.lost_s)
        if tr.still_since is not None:
            return "#%d%s đứng yên %.0f s" % (tr.id, gop, tr.t[-1] - tr.still_since)
        return "#%d%s %s" % (tr.id, gop, "%.2f m/s" % tr.speed if tr.speed is not None else "đang đo")

    @staticmethod
    def _fmt_time(sec):
        sec = int(sec)
        return "%d:%02d:%02d" % (sec // 3600, sec // 60 % 60, sec % 60) if sec >= 3600 else "%02d:%02d" % (sec // 60, sec % 60)

    def _time_text(self):
        s = self.src
        if isinstance(s, core.ReplaySource):
            return "%s / %s" % (self._fmt_time(min(s.pos, s.duration)), self._fmt_time(s.duration))
        return self._fmt_time(s.now() - self.t_start)

    def _overlay_text(self):
        if self.det.use_bg and not self.det.bg.ready():
            count = "Đối tượng: chưa học nền"
        elif not self.det.use_bg:
            count = "Đối tượng: cần bật dùng nền"
        else:
            conf = self.tracker.visible()
            count = "Đối tượng: %d (trong vùng: %d)" % (len(conf), sum(tr.in_zone for tr in conf))
            lost = len(self.tracker.lost_in_zone())
            if lost:
                count += "\nĐang giữ %d vật biến mất trong vùng" % lost
        return self._time_text() + "\n" + count

    def _update_status(self):
        m, s = self.mapper, self.src
        P = m.periods()
        now = s.now()
        if len(P):
            T = float(np.median(P))
            self.st["rps"].setText("%.3f vòng/s" % (1 / T))
            self.st["jit"].setText("%.2f ms" % (np.std(P) * 1000))
        if self.recent:
            sc, xy = self.recent[-1]
            self.st["pts"].setText(str(sc.n_total))
            self.st["valid"].setText("%.1f %%" % (100 * xy["ok"].sum() / max(1, sc.n_total)))
            self.st["rev"].setText(str(sc.seq))
        self.rate_hist.append((now, m.n_frames)); self.rate_hist = [r for r in self.rate_hist if now - r[0] <= 1.0]
        if len(self.rate_hist) > 1 and self.rate_hist[-1][0] > self.rate_hist[0][0]:
            (t0, n0), (t1, n1) = self.rate_hist[0], self.rate_hist[-1]
            self.st["rate"].setText("%.0f" % ((n1 - n0) / (t1 - t0)))
        self.st["bad"].setText(str(s.bad_frames)); self.st["idx"].setText(str(m.n_idx))
        self.st["time"].setText(self._time_text())
        self.lb_ov.setText(self._overlay_text()); self.lb_ov.adjustSize()
        warn = []
        if s.err:
            warn.append("LỖI: " + s.err)
        if m.t_last_frame is None or now - m.t_last_frame > 0.5:
            warn.append("TF03 không gửi dữ liệu")
        T = float(np.median(P)) if len(P) else 1.0
        if m.t_last_idx is None or now - m.t_last_idx > 3 * T + 1:
            warn.append("Không nhận được mốc 0°")
        if self.det.bg.learning_left > 0:
            warn.append("Đang học nền: còn %d vòng" % self.det.bg.learning_left)
        if not (isinstance(s, core.ReplaySource) and s.finished):
            self.lb_warn.setText("\n".join(warn))
        if len(self.det.zone) < 3:
            self.lb_alarm.setText("CHƯA CÓ VÙNG CẤM"); self._set_alarm_style(None)
        elif self.det.use_bg and not self.det.bg.ready():
            self.lb_alarm.setText("CHƯA HỌC NỀN"); self._set_alarm_style(None)
        elif self.tracker.lost_in_zone():                # khong bao gio bao "trong" khi con vat co the dang khuat
            self.lb_alarm.setText("BÁO ĐỘNG: VẬT BIẾN MẤT TRONG VÙNG"); self._set_alarm_style(True)
        elif self.det.alarm:
            self.lb_alarm.setText("BÁO ĐỘNG: CÓ VẬT CẢN"); self._set_alarm_style(True)
        else:
            self.lb_alarm.setText("VÙNG CẤM TRỐNG"); self._set_alarm_style(False)

    def _set_alarm_style(self, alarm):
        bg = {None: "#9e9e9e", True: "#c62828", False: "#2e7d32"}[alarm]
        self.lb_alarm.setStyleSheet("background:%s;color:white;font-size:16px;font-weight:bold;border-radius:6px" % bg)

    def _sound_state(self):
        """long: dang thay doi tuong theo dieu kien da chon; pulse: khong con thay ai ma con vat bien mat trong vung."""
        s, mode = self.src, self.cb_beep.currentText()
        if not s or mode == BEEP_OFF or (isinstance(s, core.ReplaySource) and (s.paused or s.finished)):
            return "off"
        vis = self.tracker.visible()
        if mode == BEEP_MOVE:
            active = any(tr.moving for tr in vis)
        else:
            active = any(tr.in_zone for tr in vis) if len(self.det.zone) >= 3 else bool(vis)
        return "long" if active else "pulse" if self.tracker.lost_in_zone() else "off"

    def _beep_tick(self):
        state = self._sound_state()
        self.player.set(state)
        if not self.player.ok and state != "off":
            QtWidgets.QApplication.beep()

    def _drain_entries(self):
        if self.elog:
            self.elog.write(self.tracker.closed)
        self.tracker.closed = []

    def clear_entries(self):
        self.tracker.entries = []
        for tr in self.tracker.tracks:
            tr.entry = None                             # doi tuong con trong vung se duoc tinh thanh luot moi
        self.sel_mark.setData([], []); self.sel_text.setText("")
        self._update_entries()

    def _clock(self, t):
        """Truc tiep: gio he thong. Phat lai: thoi gian tinh tu luc bat dau phat."""
        if self.wall_off is not None:
            return time.strftime("%H:%M:%S", time.localtime(t + self.wall_off))
        return self._fmt_time(max(t - self.t_start, 0))

    def _update_entries(self):
        es = self.tracker.entries
        lost_now = {id(tr.entry) for tr in self.tracker.lost_in_zone()}
        inside = sum(e["t_out"] is None for e in es)
        self.lb_entry.setText("Tổng lượt vào: %d | Số đối tượng: %d | Đang trong vùng: %d | Đang mất tín hiệu: %d"
                              % (len(es), len({e["id"] for e in es}), inside, len(lost_now)))
        self.entry_view = list(reversed(es[-200:]))     # luot moi nhat o tren
        v = lambda x: "-" if x is None else "%.2f" % x
        self.tb_entry.setRowCount(len(self.entry_view))
        for row, e in enumerate(self.entry_view):
            end = e["t_out"] if e["t_out"] is not None else e["t_last"]
            if id(e) in lost_now:
                state = "Đang mất tín hiệu"
            elif e["t_out"] is None:
                state = "Đang ở trong vùng"
            else:
                state = e["note"]
            cells = ("#%d" % e["id"], self._clock(e["t0"]), self._clock(e["t_in"]), self._clock(e["t_out"]) if e["t_out"] is not None else "-",
                     "%.1f" % (end - e["t_in"]), v(e["v_in"]), v(e["v_sum"] / e["v_n"] if e["v_n"] else None), v(e["v_max"] if e["v_n"] else None),
                     "%.2f" % e["size"], "(%.2f, %.2f)" % (e["x_last"], e["y_last"]), str(e["n_lost"]), state)
            for col, txt in enumerate(cells):
                it = QtWidgets.QTableWidgetItem(txt)
                if state in ("Đang mất tín hiệu", "Mất tín hiệu", "Hết giờ giữ"):
                    it.setForeground(QtGui.QColor("#c25e00"))
                self.tb_entry.setItem(row, col, it)

    def _on_entry_click(self, row, _col):
        if 0 <= row < len(self.entry_view):
            e = self.entry_view[row]
            self.sel_mark.setData([e["x_last"]], [e["y_last"]])
            self.sel_text.setText("#%d điểm cuối (%.2f, %.2f)" % (e["id"], e["x_last"], e["y_last"]))
            self.sel_text.setPos(e["x_last"], e["y_last"])

    def _drain_tracker_events(self, sc=None):
        name = {"MAT_TIN_HIEU": "MẤT TÍN HIỆU", "NOI_LAI": "NỐI LẠI", "HET_GIU": "HẾT GIỮ"}
        for kind, _, t, x, y, msg in self.tracker.events:
            self.det.events.append((time.time(), name[kind], msg))
            if self.alog:
                self.alog.note(kind, sc.seq if sc else -1, sc.period if sc else 0.0, t, x, y)
        self.tracker.events = []

    def ack_clear(self):
        self.tracker.clear_lost()
        self._drain_tracker_events()

    def _update_events(self):
        for t, kind, msg in self.det.events[self.n_ev_shown:]:
            it = QtWidgets.QListWidgetItem("%s  %s  %s" % (time.strftime("%H:%M:%S", time.localtime(t)), kind, msg))
            if kind in ("BÁO ĐỘNG", "MẤT TÍN HIỆU"):
                it.setForeground(QtGui.QColor("#c62828"))
            self.lst.insertItem(0, it)
        self.n_ev_shown = len(self.det.events)

    # ------------------------------------------------------------ vung cam + nen
    def toggle_draw(self, on):
        self.drawing = on
        if on:
            self.draft = []; self.bt_draw.setText("Xong (bấm chuột để thêm đỉnh)")
        else:
            self.bt_draw.setText("Vẽ vùng cấm")
            if len(self.draft) >= 3:
                self.det.zone = list(self.draft)
                self._mark_dirty()
            self.draft = []; self.draft_item.setData([], []); self._draw_zone()

    def on_click(self, ev):
        if not self.drawing or ev.button() != QtCore.Qt.LeftButton:
            return
        p = self.plot.getPlotItem().vb.mapSceneToView(ev.scenePos())
        self.draft.append((p.x(), p.y()))
        xs = [q[0] for q in self.draft]; ys = [q[1] for q in self.draft]
        self.draft_item.setData(xs, ys)

    def clear_zone(self):
        self.det.zone = []; self._draw_zone(); self._mark_dirty()

    def _draw_zone(self):
        z = self.det.zone
        if len(z) >= 3:
            xs = [p[0] for p in z] + [z[0][0]]; ys = [p[1] for p in z] + [z[0][1]]
            self.zone_item.setData(xs, ys)
        else:
            self.zone_item.setData([], [])

    def learn_bg(self):
        self.det.bg.start_learning(self.sp_learn.value())

    def _save_bg(self, sc):
        try:
            self.det.bg.save(BG_FILE, 1.0 / sc.period, self.sp_r0.value())
            self.det.events.append((time.time(), "INFO", "Đã lưu nền"))
        except Exception as e:
            self.det.events.append((time.time(), "INFO", "Không lưu được nền: %s" % e))

    def load_bg(self):
        try:
            data = detect.Background.read(BG_FILE)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Nền", "Không đọc được nền đã lưu: %s" % e)
            return
        P = self.mapper.periods() if self.src else []
        rps = 1.0 / float(np.median(P)) if len(P) else self.sp_rps.value()
        if abs(rps - data["rps"]) > 0.05 * data["rps"] and QtWidgets.QMessageBox.question(
                self, "Nền", "Nền học ở %.2f vòng/s, hiện là %.2f vòng/s. Mốc 0° dịch theo tốc độ nên nền có thể lệch góc. Vẫn nạp?"
                % (data["rps"], rps)) != QtWidgets.QMessageBox.Yes:
            return
        try:
            self.det.bg.apply(data, self.sp_r0.value())
        except ValueError as e:
            QtWidgets.QMessageBox.warning(self, "Nền", str(e))
            return
        self.det.events.append((time.time(), "INFO", "Đã nạp nền học lúc %s, chỉ đúng nếu LiDAR chưa bị dời"
                                % time.strftime("%H:%M %d/%m", time.localtime(data["t"]))))
        if self.session:
            self.session.note("nap_nen", self.src.now() - self.t_start, hoc_luc=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(data["t"])),
                              vong_s=round(data["rps"], 3))

    def _apply_detect_params(self, *_):
        self.det.margin = self.sp_margin.value(); self.det.min_size = self.sp_size.value()
        self.det.on_revs = self.sp_on.value(); self.det.use_bg = self.ck_bg.isChecked()
        self.tracker.merge_d, self.tracker.merge_dv = self.sp_mg_d.value(), self.sp_mg_dv.value()
        self.tracker.hold_s = self.sp_hold.value()

    # ------------------------------------------------------------ luu
    def _outdir(self, sub):
        base = self.src.logdir if self.src and hasattr(self.src, "logdir") else HERE
        d = os.path.join(base, sub); os.makedirs(d, exist_ok=True); return d

    def save_png(self):
        if not self.recent:
            return
        fn = os.path.join(self._outdir("anh"), "vong_%d.png" % self.recent[-1][0].seq)
        self.plot.grab().save(fn); self.det.events.append((time.time(), "INFO", "Đã lưu " + fn))

    def _snapshot(self):
        if self.recent:
            fn = os.path.join(self._outdir("su_kien"), time.strftime("baodong_%H%M%S.png"))
            self.plot.grab().save(fn)

    def save_csv(self):
        if not self.recent:
            return
        sc, xy = self.recent[-1]
        fn = os.path.join(self._outdir("anh"), "vong_%d.csv" % sc.seq)
        np.savetxt(fn, np.c_[xy["ang"], xy["r"], xy["s"], xy["x"], xy["y"]], delimiter=",",
                   header="angle_deg,r_m,strength,x_m,y_m", comments="", fmt="%.4f")
        self.det.events.append((time.time(), "INFO", "Đã lưu " + fn))

    def _end_session(self):
        if self.session:
            self._save_timer.stop()
            self._save_settings()                       # ghi not thay doi trong nua giay cuoi
            self.session.end()
            self.session = None

    def _read_ui(self):
        self.cfg.update(tf_port=self.cb_tf.currentText(), uno_port=self.cb_uno.currentText(), rps=self.sp_rps.value(),
                        tf_baud=int(self.cb_baud.currentText()), tf_hz=int(self.cb_hz.currentText()),
                        r0=self.sp_r0.value(), offset=self.sp_off.value(), dir=-1 if self.ck_dir.isChecked() else 1,
                        maxcm=self.sp_max.value(), minstr=self.sp_str.value(), accum=self.sp_acc.value(),
                        color=self.cb_color.currentText(), zone=self.det.zone, margin=self.sp_margin.value(),
                        min_size=self.sp_size.value(), on_revs=self.sp_on.value(), learn_revs=self.sp_learn.value(),
                        replay_dir=self.le_dir.text(), replay_speed=float(self.cb_speed.currentText()),
                        lock_cal=self.ck_lock.isChecked(), merge_d=self.sp_mg_d.value(), merge_dv=self.sp_mg_dv.value(),
                        hold_s=self.sp_hold.value(), beep_mode=self.cb_beep.currentText())

    def _save_settings(self):
        old = dict(self.cfg)
        self._read_ui()
        try:
            if self.session and self.src:               # t tre toi da khoang 0,5 s so voi luc doi
                self.session.diff(old, self.cfg, self.src.now() - self.t_start)
        except Exception as e:
            self.det.events.append((time.time(), "INFO", "Không ghi được session.json: %s" % e))
        try:
            tmp = SETTINGS + ".tmp"
            with open(tmp, "w") as f:
                json.dump(self.cfg, f, indent=2)
            os.replace(tmp, SETTINGS)                   # app chet giua chung khong lam hong file cu
        except Exception as e:
            self.det.events.append((time.time(), "INFO", "Không lưu được cấu hình: %s" % e))

    def closeEvent(self, e):
        self.beep_timer.stop(); self.player.set("off")
        self._save_timer.stop()
        if self.src:
            self.tracker.close_open(); self._drain_entries()
            self._end_session()
            self.src.stop()
        self._save_settings()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.show()
    sys.exit(app.exec())
