"""
app.py - Ung dung giam sat LiDAR 2D (ban thu tren PC, chua co STM32).

Chuc nang:
  Dot 1: chon nguon Live (TF03 + UNO) hoac Phat lai thu muc log; bang trang thai va canh bao;
         chinh r0 / offset / chieu / loc diem ngay khi dang chay; cong don N vong; luu anh + CSV.
  Dot 2: ve vung cam bang chuot, hoc nen, gom cum vat the, bao dong (mau do + tieng beep),
         nhat ky su kien kem anh chup.
  Dot 3 (mot phan): do baud, dat baud va tan so TF03 khi dang dung.

Cai dat:  pip install PySide6 pyqtgraph pyserial numpy
Chay:     python app.py
CODE THU NGHIEM: da chay voi log gia lap va chay Live voi UNO + TF03 that (04/11); chua dung lam so lieu danh gia.
"""
import json, os, sys, time

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets

import core
import detect

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS = os.path.join(HERE, "settings.json")
DEFAULTS = dict(tf_port="COM12", uno_port="COM10", tf_baud=115200, tf_hz=1000, rps=1.0, r0=0.0, offset=0.0, dir=1, maxcm=3000, minstr=0,
                accum=1, color="cường độ", view=5.0, zone=[], margin=0.15, min_size=0.10, on_revs=3,
                learn_revs=20, replay_dir="", replay_speed=1.0)


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
        self._build_ui()
        self._apply_detect_params()
        self._draw_zone()
        self.timer = QtCore.QTimer(self, interval=50, timeout=self.on_tick)
        self.timer.start()

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
        for it in (self.zone_item, self.sc, self.sc_fg, self.draft_item, origin):
            self.plot.addItem(it)
        self.plot.scene().sigMouseClicked.connect(self.on_click)
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
                                       ("idx", "Số mốc 0°"), ("rev", "Vòng")]):
            grid.addWidget(QtWidgets.QLabel(name), i, 0)
            self.st[k] = QtWidgets.QLabel("-"); self.st[k].setAlignment(QtCore.Qt.AlignRight)
            grid.addWidget(self.st[k], i, 1)
        self.lb_warn = QtWidgets.QLabel(""); self.lb_warn.setWordWrap(True)
        self.lb_warn.setStyleSheet("color:#b00020;font-weight:bold")
        grid.addWidget(self.lb_warn, 8, 0, 1, 2)
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
        for w_ in (self.sp_r0, self.sp_off, self.sp_max, self.sp_str, self.sp_acc):
            w_.valueChanged.connect(self._redraw_recent)
        self.ck_dir.toggled.connect(self._redraw_recent); self.cb_color.currentTextChanged.connect(self._redraw_recent)
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
        self.sp_margin = dspin(0.02, 2, 0.05, self.cfg["margin"])
        self.sp_size = dspin(0.0, 5, 0.05, self.cfg["min_size"])
        self.sp_on = ispin(1, 50, 1, self.cfg["on_revs"])
        for w_, lab in ((self.sp_margin, "Gần hơn nền (m)"), (self.sp_size, "Vật nhỏ nhất (m)"), (self.sp_on, "Báo khi liên tiếp (vòng)")):
            f.addRow(lab, w_); w_.valueChanged.connect(self._apply_detect_params)
        self.lb_alarm = QtWidgets.QLabel("CHƯA CÓ VÙNG CẤM"); self.lb_alarm.setAlignment(QtCore.Qt.AlignCenter)
        self.lb_alarm.setMinimumHeight(44); self._set_alarm_style(None)
        f.addRow(self.lb_alarm)
        self.lst = QtWidgets.QListWidget(); self.lst.setMaximumHeight(140)
        f.addRow(self.lst)
        lay.addWidget(g)
        lay.addStretch(1)

        scroll = QtWidgets.QScrollArea(); scroll.setWidget(side); scroll.setWidgetResizable(True)
        split = QtWidgets.QSplitter(); split.addWidget(self.plot); split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        self.setCentralWidget(split)
        self.resize(1300, 860)

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
            self.src.stop(); self.src = None
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
        for sc in scans:
            xy = core.to_xy(sc, **self._xy_params())
            objs, fg, newly = self.det.step(xy)
            xy["fg"], xy["objs"] = fg, objs
            self.last_scan = sc
            self.recent.append((sc, xy))
            self.recent = self.recent[-20:]
            if newly:
                QtWidgets.QApplication.beep()
                self._snapshot()
        if scans:
            self._draw()
        self._update_status()
        self._update_events()
        if isinstance(self.src, core.ReplaySource) and self.src.finished and not scans:
            self.lb_warn.setText("Phát lại xong.")

    def _xy_params(self):
        return dict(r0=self.sp_r0.value(), offset_deg=self.sp_off.value(), direction=-1 if self.ck_dir.isChecked() else 1,
                    max_cm=self.sp_max.value(), min_str=self.sp_str.value())

    def _redraw_recent(self, *_):
        p = self._xy_params()
        new_rec = []
        for sc, _ in self.recent:
            xy = core.to_xy(sc, **p)
            objs, fg = self.det.classify(xy)
            xy["fg"], xy["objs"] = fg, objs
            new_rec.append((sc, xy))
        self.recent = new_rec
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
        elif self.det.alarm:
            self.lb_alarm.setText("BÁO ĐỘNG: CÓ VẬT CẢN"); self._set_alarm_style(True)
        else:
            self.lb_alarm.setText("VÙNG CẤM TRỐNG"); self._set_alarm_style(False)

    def _set_alarm_style(self, alarm):
        bg = {None: "#9e9e9e", True: "#c62828", False: "#2e7d32"}[alarm]
        self.lb_alarm.setStyleSheet("background:%s;color:white;font-size:16px;font-weight:bold;border-radius:6px" % bg)

    def _update_events(self):
        for t, kind, msg in self.det.events[self.n_ev_shown:]:
            it = QtWidgets.QListWidgetItem("%s  %s  %s" % (time.strftime("%H:%M:%S", time.localtime(t)), kind, msg))
            if kind == "BÁO ĐỘNG":
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
            self.draft = []; self.draft_item.setData([], []); self._draw_zone(); self._redraw_recent()

    def on_click(self, ev):
        if not self.drawing or ev.button() != QtCore.Qt.LeftButton:
            return
        p = self.plot.getPlotItem().vb.mapSceneToView(ev.scenePos())
        self.draft.append((p.x(), p.y()))
        xs = [q[0] for q in self.draft]; ys = [q[1] for q in self.draft]
        self.draft_item.setData(xs, ys)

    def clear_zone(self):
        self.det.zone = []; self._draw_zone(); self._redraw_recent()

    def _draw_zone(self):
        z = self.det.zone
        if len(z) >= 3:
            xs = [p[0] for p in z] + [z[0][0]]; ys = [p[1] for p in z] + [z[0][1]]
            self.zone_item.setData(xs, ys)
        else:
            self.zone_item.setData([], [])

    def learn_bg(self):
        self.det.bg.start_learning(self.sp_learn.value())

    def _apply_detect_params(self, *_):
        self.det.margin = self.sp_margin.value(); self.det.min_size = self.sp_size.value()
        self.det.on_revs = self.sp_on.value(); self.det.use_bg = self.ck_bg.isChecked()
        self._redraw_recent()

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
            fn = os.path.join(self._outdir("su_kien"), "%s_vong%d.png" % (time.strftime("baodong_%H%M%S"), self.recent[-1][0].seq))
            self.plot.grab().save(fn)

    def save_csv(self):
        if not self.recent:
            return
        sc, xy = self.recent[-1]
        fn = os.path.join(self._outdir("anh"), "vong_%d.csv" % sc.seq)
        np.savetxt(fn, np.c_[xy["ang"], xy["r"], xy["s"], xy["x"], xy["y"]], delimiter=",",
                   header="angle_deg,r_m,strength,x_m,y_m", comments="", fmt="%.4f")
        self.det.events.append((time.time(), "INFO", "Đã lưu " + fn))

    def closeEvent(self, e):
        if self.src:
            self.src.stop()
        self.cfg.update(tf_port=self.cb_tf.currentText(), uno_port=self.cb_uno.currentText(), rps=self.sp_rps.value(),
                        tf_baud=int(self.cb_baud.currentText()), tf_hz=int(self.cb_hz.currentText()),
                        r0=self.sp_r0.value(), offset=self.sp_off.value(), dir=-1 if self.ck_dir.isChecked() else 1,
                        maxcm=self.sp_max.value(), minstr=self.sp_str.value(), accum=self.sp_acc.value(),
                        color=self.cb_color.currentText(), zone=self.det.zone, margin=self.sp_margin.value(),
                        min_size=self.sp_size.value(), on_revs=self.sp_on.value(), learn_revs=self.sp_learn.value(),
                        replay_dir=self.le_dir.text(), replay_speed=float(self.cb_speed.currentText()))
        try:
            json.dump(self.cfg, open(SETTINGS, "w"), indent=2)
        except Exception:
            pass
        super().closeEvent(e)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.show()
    sys.exit(app.exec())
