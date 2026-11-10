"""
san_pham_v2.py - Mo hinh 3D Y TUONG cho san pham thuong mai (phien ban 2): LiDAR 2D + camera, lap cot.
CODE THU NGHIEM - chi la mo hinh hinh dang/bo tri, KHONG phai ban ve che tao.
Moi kich thuoc la [GIA DINH] tu dat, tru cac khoi lay tu lidar_frame.py (TF03, cung la gia dinh).

Y tuong chinh (khac ban do an):
  * TF03 nam trong DE, chieu LEN; guong 45 do quay tren o bi lo to (thin-section), dong co BLDC
    dat lech truc keo bang dai GT2 -> KHONG co day hay cot nao cat ngang mat quet => khong vung mu.
  * Ong cua so trong suot do nap tren (nap thu dong, khong co dien).
  * Camera gan o mat truoc cua de, THAP hon mat quet -> khong che tia.
  * Mot bo mach chinh: STM32 + W5500 (Ethernet) + driver FOC + buck 24V->5V + 4 relay + switch Ethernet
    (1 cap ra ngoai cho ca LiDAR va camera).

Toa do: goc = tam truc quay tren mat day de. Z huong len. Mat quet o Z = SCAN_Z. Camera nhin huong +X.
Chay:  python san_pham_v2.py   -> san_pham_v2.step (Fusion 360), san_pham_v2.glb (trang xem 3D)
"""
import math
import cadquery as cq

# ---------------- kich thuoc chinh [GIA DINH] ----------------
BASE_D, BASE_H, WALL = 150.0, 150.0, 3.0      # vo de
DECK_Z = 128.0                                 # mat duoi tam do o bi
WIN_OD, WIN_T, WIN_H = 140.0, 3.0, 75.0        # ong cua so
SCAN_Z = 185.0                                 # cao do tam guong = mat quet
TOP_H = 22.0                                   # nap tren
BORE = 36.0                                    # lo cho tia di len (2 thau kinh TF03 trai ~34 mm, gia dinh)
BRG_ID, BRG_OD, BRG_W = 40.0, 52.0, 7.0        # o bi thanh mong
TF_W, TF_T, TF_L = 44.0, 31.8, 42.9            # TF03 (lay theo lidar_frame.py, (*))
TF_TOP = 118.0                                 # mat kinh TF03 (huong len)
MOTOR_X = 48.0                                 # dong co lech truc
PCB_Z, PCB_T = 12.0, 1.6

C = {  # mau (r, g, b, a)
    "vo": (0.55, 0.58, 0.62, 1), "vo_toi": (0.30, 0.32, 0.35, 1), "kinh": (0.55, 0.75, 0.95, 0.25),
    "pcb": (0.05, 0.40, 0.20, 1), "ic": (0.08, 0.08, 0.09, 1), "kim": (0.80, 0.80, 0.82, 1),
    "relay": (0.15, 0.35, 0.80, 1), "dauNoi": (0.95, 0.95, 0.92, 1), "xanhla": (0.20, 0.65, 0.30, 1),
    "tf03": (0.12, 0.12, 0.14, 1), "thau_kinh": (0.25, 0.10, 0.10, 1), "guong": (0.90, 0.92, 0.95, 1),
    "nhom": (0.75, 0.76, 0.78, 1), "dai": (0.10, 0.10, 0.10, 1), "dongco": (0.20, 0.20, 0.22, 1),
    "cam": (0.92, 0.92, 0.90, 1), "cot": (0.45, 0.47, 0.50, 1), "cam_bien": (0.10, 0.10, 0.10, 1),
}

assy = cq.Assembly(name="san_pham_v2")
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
floor = cyl(BASE_D, 5)
GLANDS = [(55 * math.cos(math.radians(a)), 55 * math.sin(math.radians(a))) for a in (160, 180, 200)]
for a in (-20, 0, 20):  # 3 lo ong luon cap o day
    r = 55
    floor = floor.cut(cyl(16.5, 10, r * math.cos(math.radians(180 + a)), r * math.sin(math.radians(180 + a)), -1))
lid = cyl(BASE_D, 3, z0=BASE_H - 3).cut(cyl(46, 10, z0=BASE_H - 5))
# o lap camera o mat truoc (+X): bo tron mat cat
add(shell.union(floor).union(lid), "vo_de", "vo", "vo")

# tam do o bi + dong co (deck)
deck = cyl(BASE_D - 2 * WALL - 1, 4, z0=DECK_Z).cut(cyl(BRG_OD - 0.2, 10, z0=DECK_Z - 1))
deck = deck.cut(cyl(8, 10, MOTOR_X, 0, DECK_Z - 1))
add(deck, "tam_do", "nhom", "co_khi")
for a in (45, 135, 225, 315):                                   # 4 tru do tam
    x, y = 62 * math.cos(math.radians(a)), 62 * math.sin(math.radians(a))
    add(cyl(8, DECK_Z - 5, x, y, 5), f"tru_do_{a}", "kim", "co_khi")

# ======================= PCB CHINH + LINH KIEN =======================
pcb = cyl(BASE_D - 2 * WALL - 6, PCB_T, z0=PCB_Z)
for a in (45, 135, 225, 315):
    pcb = pcb.cut(cyl(9, 5, 62 * math.cos(math.radians(a)), 62 * math.sin(math.radians(a)), PCB_Z - 1))
for gx, gy in GLANDS:
    pcb = pcb.cut(cyl(20, 5, gx, gy, PCB_Z - 1))
add(pcb, "pcb_chinh", "pcb", "pcb")
for i, (gx, gy) in enumerate(GLANDS):  # ong luon cap M16 (phan nhin thay duoi day)
    add(cyl(22, 8, gx, gy, -8).union(cyl(16, 3, gx, gy, -11)), f"ong_luon_cap_{i}", "vo_toi", "vo")
zt = PCB_Z + PCB_T
parts = [  # (ten, (x,y,z) kich thuoc, (cx,cy), mau)
    ("stm32", (10, 10, 1.4), (5, 10), "ic"),
    ("w5500_ethernet", (7, 7, 0.9), (5, -8), "ic"),
    ("switch_ethernet_ic", (8, 8, 1.0), (-10, -8), "ic"),
    ("rj45_ra_ngoai", (16, 21, 13.5), (-30, 12), "kim"),
    ("rj45_camera", (16, 21, 13.5), (-30, -12), "kim"),
    ("cau_dau_24v", (10.2, 8, 10), (-15, 40), "xanhla"),
    ("cau_chi", (10, 5, 5), (-2, 42), "kim"),
    ("buck_ic", (3, 3, 1.0), (10, 42), "ic"),
    ("cuon_cam_buck", (12, 12, 6), (24, 42), "ic"),
    ("driver_foc", (6, 6, 1.0), (30, 8), "ic"),
    ("jst_dong_co", (8, 6, 6), (45, 8), "dauNoi"),
    ("jst_encoder", (14, 6, 6), (45, -10), "dauNoi"),
    ("jst_tf03", (10, 6, 6), (15, -26), "dauNoi"),
    ("jst_moc0", (8, 6, 6), (-5, -26), "dauNoi"),
    ("swd", (12.7, 6, 5), (32, -26), "ic"),
    ("relay_1", (15.5, 10.5, 11.5), (-30, -45), "relay"),
    ("relay_2", (15.5, 10.5, 11.5), (-12, -45), "relay"),
    ("relay_3", (15.5, 10.5, 11.5), (6, -45), "relay"),
    ("relay_4", (15.5, 10.5, 11.5), (24, -45), "relay"),
    ("cau_dau_relay", (40.6, 8, 10), (-3, -60), "xanhla"),
]
for name, (sx, sy, sz), (cx, cy), col in parts:
    add(box(sx, sy, sz, cx, cy, zt), name, col, "pcb")
for i, (cx, cy) in enumerate([(40, 18), (40, 28), (-2, 30)]):     # tu dien
    add(cyl(8, 10, cx, cy, zt), f"tu_{i}", "kim", "pcb")
for i, cx in enumerate((-12, -8, -4)):                                   # LED
    add(box(1.6, 0.8, 0.6, cx, 24, zt), f"led_{i}", "relay", "pcb")

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

# ======================= DONG CO BLDC + DAI =======================
add(cyl(40, 18, MOTOR_X, 0, DECK_Z - 18), "dong_co_bldc", "dongco", "truyen_dong")
add(cyl(25, 1.6, MOTOR_X, 0, DECK_Z - 22), "encoder_as5048", "pcb", "truyen_dong")
add(cyl(4, 14, MOTOR_X, 0, DECK_Z), "truc_dong_co", "kim", "truyen_dong")
R_SMALL, R_BIG = 20 * 2 / math.pi / 2, 80 * 2 / math.pi / 2           # GT2 20T / 80T (ban kinh buoc)
PZ = DECK_Z + 4 + BRG_W + 1                                              # cao do dai
add(cyl(2 * R_SMALL + 2, 8, MOTOR_X, 0, PZ - 1), "puly_20T", "nhom", "truyen_dong")
# vo bao loi cua 2 duong tron = vong dai
def hull2(r1, r2, d, h):
    a = math.asin((r1 - r2) / d)
    p1 = (r1 * math.sin(a), r1 * math.cos(a)); p2 = (d + r2 * math.sin(a), r2 * math.cos(a))
    w = (cq.Workplane("XY").moveTo(*p1).lineTo(*p2)
         .threePointArc((d + r2, 0), (p2[0], -p2[1])).lineTo(p1[0], -p1[1])
         .threePointArc((-r1, 0), p1).close().extrude(h))
    return w
belt = hull2(R_BIG + 0.8, R_SMALL + 0.8, MOTOR_X, 6).cut(hull2(R_BIG - 0.2, R_SMALL - 0.2, MOTOR_X, 6))
add(belt.translate((0, 0, PZ)), "dai_gt2", "dai", "truyen_dong")
add(tube(BRG_OD, BRG_ID, BRG_W, z0=DECK_Z), "o_bi_mong", "kim", "truyen_dong")

# ======================= PHAN QUAY: puly 80T + ong + guong =======================
rot_z0 = PZ - 1
puly = tube(2 * R_BIG + 2, BORE, 8, z0=rot_z0)
sleeve = tube(BRG_ID, BORE, rot_z0 - DECK_Z, z0=DECK_Z)                 # ong lot trong vong trong o bi
flag = box(15, 6, 2, -(R_BIG + 7), 0, rot_z0 + 3)                       # la chan moc 0 nam ngang (huong -X)
stem = tube(BRG_ID + 4, BORE, SCAN_Z - 22 - (rot_z0 + 8), z0=rot_z0 + 8)
# khoi nem guong: tru D=48, cat bo phan phia truoc mat 45 do (x > z - SCAN_Z)
cup_z0 = SCAN_Z - 22
cup = cyl(48, 52, z0=cup_z0)
# mat phang x = z - SCAN_Z  <=> z = x + SCAN_Z ; vung bo: z < x + SCAN_Z (phia truoc-duoi guong)
cutter = (cq.Workplane("XZ").polyline([(-80, SCAN_Z - 80), (80, SCAN_Z + 80), (80, 0), (-80, 0)]).close()
          .extrude(60, both=True))
cup = cup.cut(cutter).cut(cyl(BORE, SCAN_Z, z0=cup_z0 - 1).intersect(cutter))
rotor = puly.union(sleeve).union(flag).union(stem).union(cup)
add(rotor, "phan_quay", "nhom", "phan_quay")
# guong elip nam tren mat 45 do, tam (0,0,SCAN_Z)
mir = (cq.Workplane("XY").ellipse(22 * math.sqrt(2), 22).extrude(2.5)
       .rotate((0, 0, 0), (0, 1, 0), -45).translate((0, 0, SCAN_Z)))
add(mir, "guong_45", "guong", "phan_quay")
# cam bien khe (moc 0) tren deck, phia -X
sk = box(10, 10, 14, -(R_BIG + 15.5), 0, DECK_Z + 4).cut(box(8, 12, 3, -(R_BIG + 12.5), 0, rot_z0 + 2.5))
add(sk, "cam_bien_khe_moc0", "cam_bien", "phan_quay")

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
arm = box(30, 30, 12, BASE_D / 2 + 8, 0, CAM_Z - 36)
add(arm, "gia_camera", "vo_toi", "camera")

# ======================= GA COT (mat sau -X) =======================
POLE_D, POLE_X = 76.0, -(BASE_D / 2 + 70)
add(cyl(POLE_D, 600, POLE_X, 0, -250), "cot_lap_dat", "cot", "ga_cot")
plate = box(10, 120, 140, -(BASE_D / 2 + 5), 0, 5)
add(plate, "tam_ga", "vo_toi", "ga_cot")
for z in (30, 120):
    arm2 = box(abs(POLE_X) - BASE_D / 2 - 10, 20, 10, (POLE_X - BASE_D / 2 - 15) / 2, 0, z)
    clamp = tube(POLE_D + 12, POLE_D, 10, z0=z).translate((POLE_X, 0, 0))
    add(arm2.union(clamp), f"kep_cot_{z}", "vo_toi", "ga_cot")

# ======================= xuat file =======================
if __name__ == "__main__":
    import json, os
    here = os.path.dirname(os.path.abspath(__file__))
    assy.save(os.path.join(here, "san_pham_v2.step"))
    assy.save(os.path.join(here, "san_pham_v2.glb"))
    json.dump(nhom, open(os.path.join(here, "nhom_chi_tiet.json"), "w"), ensure_ascii=False, indent=1)
    bb = assy.toCompound().BoundingBox()
    print("so chi tiet:", len(nhom), "| kich thuoc bao (mm): %.0f x %.0f x %.0f" % (bb.xlen, bb.ylen, bb.zlen))
