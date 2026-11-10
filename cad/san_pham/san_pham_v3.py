"""
san_pham_v3.py - Mo hinh 3D Y TUONG, ban QUET 360 DO: LiDAR 2D + camera gan TREN DINH COT.
CODE THU NGHIEM - chi la mo hinh hinh dang/bo tri, KHONG phai ban ve che tao.
Moi kich thuoc la [GIA DINH] tu dat, tru khoi TF03 lay tu lidar_frame.py (cung la gia dinh).

Khac san_pham_v2.py (ban dai GT2, gan hong cot):
  * Gan tren DINH cot qua mat bich + ong ap -> khong co cot che phia sau, cap di xuong trong long cot.
  * Dong co BLDC TRUC RONG om quanh duong tia, truyen THANG vao guong (khong dai, khong puly, khong o bi rieng).
  * Encoder tu dang VONG tren phan quay + dau doc co dinh -> doc thang goc guong, khong can la chan moc 0.
  * TF03 van o de chieu LEN; khong co day/cot nao cat ngang mat quet -> khong vung mu tren 360 do.

Toa do: goc = tam truc quay tren mat day de. Z huong len. Mat quet o Z = SCAN_Z. Camera nhin huong +X.
Chay:  python san_pham_v3.py   -> san_pham_v3.step (Fusion 360), san_pham_v3.glb (trang xem 3D)
"""
import math
import cadquery as cq

# ---------------- kich thuoc chinh [GIA DINH] ----------------
BASE_D, BASE_H, WALL = 150.0, 150.0, 3.0      # vo de
DECK_Z = 128.0                                 # mat duoi tam do dong co
WIN_OD, WIN_T, WIN_H = 140.0, 3.0, 75.0        # ong cua so
SCAN_Z = 185.0                                 # cao do tam guong = mat quet
TOP_H = 22.0                                   # nap tren
BORE = 36.0                                    # lo cho tia di len (2 thau kinh TF03 trai ~34 mm, gia dinh)
MOT_OD, MOT_ID, MOT_H = 86.0, 46.0, 18.0       # dong co BLDC truc rong (stator + vo)
TF_W, TF_T, TF_L = 44.0, 31.8, 42.9            # TF03 (lay theo lidar_frame.py, (*))
TF_TOP = 118.0                                 # mat kinh TF03 (huong len)
PCB_Z, PCB_T = 12.0, 1.6
POLE_D = 76.0                                  # cot lap dat

C = {  # mau (r, g, b, a)
    "vo": (0.55, 0.58, 0.62, 1), "vo_toi": (0.30, 0.32, 0.35, 1), "kinh": (0.55, 0.75, 0.95, 0.25),
    "pcb": (0.05, 0.40, 0.20, 1), "ic": (0.08, 0.08, 0.09, 1), "kim": (0.80, 0.80, 0.82, 1),
    "relay": (0.15, 0.35, 0.80, 1), "dauNoi": (0.95, 0.95, 0.92, 1), "xanhla": (0.20, 0.65, 0.30, 1),
    "tf03": (0.12, 0.12, 0.14, 1), "thau_kinh": (0.25, 0.10, 0.10, 1), "guong": (0.90, 0.92, 0.95, 1),
    "nhom": (0.75, 0.76, 0.78, 1), "dongco": (0.20, 0.20, 0.22, 1), "nam_cham": (0.45, 0.20, 0.20, 1),
    "cam": (0.92, 0.92, 0.90, 1), "cot": (0.45, 0.47, 0.50, 1),
}

assy = cq.Assembly(name="san_pham_v3")
nhom = {}  # ten chi tiet -> nhom (de trang 3D tach roi theo nhom)


def add(obj, name, color, group):
    assy.add(obj, name=name, color=cq.Color(*C[color]))
    nhom[name] = group


def box(x, y, z, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").box(x, y, z, centered=(True, True, False)).translate((cx, cy, z0))


def cyl(d, h, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").circle(d / 2).extrude(h).translate((cx, cy, z0))


def tube(od, idd, h, z0=0):
    return cq.Workplane("XY").circle(od / 2).circle(idd / 2).extrude(h).translate((0, 0, z0))


# ======================= VO DE =======================
shell = tube(BASE_D, BASE_D - 2 * WALL, BASE_H - 5, z0=5)
floor = cyl(BASE_D, 5).cut(cyl(30, 10, z0=-1))                 # lo giua: cap di xuong trong long cot
lid = cyl(BASE_D, 3, z0=BASE_H - 3).cut(cyl(MOT_OD + 4, 10, z0=BASE_H - 5))
add(shell.union(floor).union(lid), "vo_de", "vo", "vo")
add(tube(30, 22, 6, z0=-0.5), "dem_cao_su_cap", "vo_toi", "vo")

# tam do dong co (deck)
deck = cyl(BASE_D - 2 * WALL - 1, 4, z0=DECK_Z).cut(cyl(MOT_ID, 10, z0=DECK_Z - 1))
add(deck, "tam_do", "nhom", "co_khi")
for a in (45, 135, 225, 315):                                   # 4 tru do tam
    x, y = 62 * math.cos(math.radians(a)), 62 * math.sin(math.radians(a))
    add(cyl(8, DECK_Z - 5, x, y, 5), f"tru_do_{a}", "kim", "co_khi")

# ======================= PCB CHINH + LINH KIEN =======================
pcb = cyl(BASE_D - 2 * WALL - 6, PCB_T, z0=PCB_Z).cut(cyl(30, 5, z0=PCB_Z - 1))   # lo giua cho cap vao
for a in (45, 135, 225, 315):
    pcb = pcb.cut(cyl(9, 5, 62 * math.cos(math.radians(a)), 62 * math.sin(math.radians(a)), PCB_Z - 1))
add(pcb, "pcb_chinh", "pcb", "pcb")
zt = PCB_Z + PCB_T
parts = [  # (ten, (x,y,z) kich thuoc, (cx,cy), mau)
    ("stm32", (10, 10, 1.4), (20, 22), "ic"),
    ("w5500_ethernet", (7, 7, 0.9), (22, -8), "ic"),
    ("switch_ethernet_ic", (8, 8, 1.0), (-16, 26), "ic"),
    ("rj45_ra_ngoai", (16, 21, 13.5), (-30, 4), "kim"),
    ("rj45_camera", (16, 21, 13.5), (-30, -20), "kim"),
    ("cau_dau_24v", (10.2, 8, 10), (-15, 42), "xanhla"),
    ("cau_chi", (10, 5, 5), (-2, 44), "kim"),
    ("buck_ic", (3, 3, 1.0), (10, 44), "ic"),
    ("cuon_cam_buck", (12, 12, 6), (24, 44), "ic"),
    ("driver_foc", (6, 6, 1.0), (32, 8), "ic"),
    ("jst_dong_co", (8, 6, 6), (47, 8), "dauNoi"),
    ("jst_encoder", (14, 6, 6), (47, -10), "dauNoi"),
    ("jst_tf03", (10, 6, 6), (15, -26), "dauNoi"),
    ("swd", (12.7, 6, 5), (34, -26), "ic"),
    ("relay_1", (15.5, 10.5, 11.5), (-30, -45), "relay"),
    ("relay_2", (15.5, 10.5, 11.5), (-12, -45), "relay"),
    ("relay_3", (15.5, 10.5, 11.5), (6, -45), "relay"),
    ("relay_4", (15.5, 10.5, 11.5), (24, -45), "relay"),
    ("cau_dau_relay", (40.6, 8, 10), (-3, -60), "xanhla"),
]
for name, (sx, sy, sz), (cx, cy), col in parts:
    add(box(sx, sy, sz, cx, cy, zt), name, col, "pcb")
for i, (cx, cy) in enumerate([(42, 20), (42, 30), (-2, 32)]):     # tu dien
    add(cyl(8, 10, cx, cy, zt), f"tu_{i}", "kim", "pcb")
for i, cy in enumerate((-4, 0, 4)):                                # LED trang thai
    add(box(0.8, 1.6, 0.6, -48, cy, zt), f"led_{i}", "relay", "pcb")

# ======================= TF03 (chieu len) + gia =======================
tf = box(TF_W, TF_T, TF_L, z0=TF_TOP - TF_L)
for dx in (-10, 10):  # 2 thau kinh phat/thu (gia dinh)
    tf = tf.cut(cyl(15, 2, dx, 0, TF_TOP - 1.5))
add(tf, "tf03", "tf03", "tf03")
for i, dx in enumerate((-10, 10)):
    add(cyl(14, 0.5, dx, 0, TF_TOP - 1.5), f"thau_kinh_{i}", "thau_kinh", "tf03")
for sy in (-1, 1):  # 2 tam treo TF03 duoi deck
    y = sy * (TF_T / 2 + 1.5)
    add(box(TF_W + 10, 3, DECK_Z - (TF_TOP - TF_L) + 2, 0, y, TF_TOP - TF_L - 2), f"gia_tf03_{sy}", "nhom", "tf03")

# ======================= DONG CO BLDC TRUC RONG + ENCODER VONG =======================
MZ0 = DECK_Z + 4                                                  # mat duoi dong co (tren deck)
add(tube(MOT_OD, MOT_ID, MOT_H, z0=MZ0), "dong_co_truc_rong", "dongco", "truyen_dong")
add(box(10, 8, 5, -(26 + 6), 0, BASE_H), "dau_doc_encoder", "pcb", "truyen_dong")

# ======================= PHAN QUAY: mat bich rotor + vong nam cham + ong + guong =======================
RZ0 = MZ0 + MOT_H                                                  # mat tren dong co = mat bich rotor
flange = tube(60, BORE, 4, z0=RZ0)
magring = tube(52, 44.5, 2, z0=RZ0 + 4)
stem = tube(44, BORE, SCAN_Z - 22 - (RZ0 + 4), z0=RZ0 + 4)
cup_z0 = SCAN_Z - 22
cup = cyl(48, 52, z0=cup_z0)
# mat phang 45 do: z = x + SCAN_Z ; bo phan phia truoc-duoi guong (z < x + SCAN_Z)
cutter = (cq.Workplane("XZ").polyline([(-80, SCAN_Z - 80), (80, SCAN_Z + 80), (80, 0), (-80, 0)]).close()
          .extrude(60, both=True))
cup = cup.cut(cutter)
# doi trong phia truoc (+X), thap hon duong tia ra, can bang khoi nem lech ve phia sau (gia dinh)
cw = box(10, 30, 6, 27, 0, RZ0 + 4)
add(flange.union(stem).union(cup).union(cw), "phan_quay", "nhom", "phan_quay")
add(magring, "vong_nam_cham_encoder", "nam_cham", "phan_quay")
mir = (cq.Workplane("XY").ellipse(22 * math.sqrt(2), 22).extrude(2.5)
       .rotate((0, 0, 0), (0, 1, 0), -45).translate((0, 0, SCAN_Z)))
add(mir, "guong_45", "guong", "phan_quay")

# ======================= CUA SO + NAP =======================
add(tube(WIN_OD, WIN_OD - 2 * WIN_T, WIN_H, z0=BASE_H), "ong_cua_so", "kinh", "cua_so")
top_z = BASE_H + WIN_H
top = cyl(BASE_D, TOP_H - 6, z0=top_z).union(
    cq.Workplane("XY").sphere(BASE_D / 2 * 1.4).translate((0, 0, top_z + TOP_H - 6 - BASE_D / 2 * 1.4 + 8))
    .intersect(cyl(BASE_D, 30, z0=top_z + TOP_H - 6)))
top = top.union(tube(BASE_D + 10, BASE_D - 2, 4, z0=top_z))                    # vanh che nang
add(top, "nap_tren", "vo", "nap")

# ======================= CAMERA (mat truoc +X, duoi mat quet) =======================
CAM_Z = 95.0
cam = box(70, 50, 48).translate((0, 0, -24))
cam = cam.union(cyl(26, 6, z0=0).rotate((0, 0, 0), (0, 1, 0), 90).translate((35, 0, 0)))
cam = cam.union(box(80, 58, 3).translate((0, 0, 24)))                              # mai che
cam = cam.rotate((0, 0, 0), (0, 1, 0), 10)                                        # cui xuong 10 do
add(cam.translate((BASE_D / 2 + 45, 0, CAM_Z)), "camera", "cam", "camera")
add(cyl(18, 1, z0=0).rotate((0, 0, 0), (0, 1, 0), 90).rotate((0, 0, 0), (0, 1, 0), 10)
    .translate((BASE_D / 2 + 45 + 41 * math.cos(math.radians(10)), 0, CAM_Z - 41 * math.sin(math.radians(10)))),
    "ong_kinh_camera", "ic", "camera")
add(box(30, 30, 12, BASE_D / 2 + 8, 0, CAM_Z - 36), "gia_camera", "vo_toi", "camera")

# ======================= GA DINH COT =======================
add(cyl(POLE_D, 640, z0=-650), "cot_lap_dat", "cot", "ga_cot")
add(cyl(BASE_D, 8, z0=-8).cut(cyl(30, 20, z0=-15)), "mat_bich", "vo_toi", "ga_cot")
add(tube(POLE_D + 14, POLE_D + 0.5, 50, z0=-58), "ong_ap_cot", "vo_toi", "ga_cot")
for a in (45, 135, 225, 315):                                      # 4 bulong mat bich
    x, y = 62 * math.cos(math.radians(a)), 62 * math.sin(math.radians(a))
    add(cyl(10, 5, x, y, -13), f"bulong_{a}", "kim", "ga_cot")
for a in (0, 90, 180, 270):                                        # 4 gan tang cung
    g = (cq.Workplane("XZ").polyline([(0, 0), (28, 0), (0, -40)]).close().extrude(2.5, both=True)
         .translate(((POLE_D + 14) / 2, 0, -8)).rotate((0, 0, 0), (0, 0, 1), a))
    add(g, f"gan_{a}", "vo_toi", "ga_cot")

# ======================= xuat file =======================
if __name__ == "__main__":
    import json, os
    here = os.path.dirname(os.path.abspath(__file__))
    assy.save(os.path.join(here, "san_pham_v3.step"))
    assy.save(os.path.join(here, "san_pham_v3.glb"))
    json.dump(nhom, open(os.path.join(here, "nhom_chi_tiet_v3.json"), "w"), ensure_ascii=False, indent=1)
    bb = assy.toCompound().BoundingBox()
    print("so chi tiet:", len(nhom), "| kich thuoc bao (mm): %.0f x %.0f x %.0f" % (bb.xlen, bb.ylen, bb.zlen))
