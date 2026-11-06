"""
lidar_v4.py - Mo hinh 3D Y TUONG khoi LiDAR 2D (khong camera): TF03 O TREN chieu xuong, guong 45 do + dong co O DUOI.
CODE THU NGHIEM - mo hinh hinh dang/bo tri, KHONG phai ban ve che tao. Moi kich thuoc la [GIA DINH] tu dat,
tru khoi TF03 lay tu lidar_frame.py (cung la gia dinh) va mau day TF03 (do/den/nau/xanh duong, da xac nhan).

Bo tri (tu duoi len):
  DE (z 0..110): tam gan day + vo de; bo mach chinh; tam do dong co; dong co BLDC gimbal + encoder AS5048A;
                 dau noi nguon M12 24 V, dau noi Ethernet, van tho o mat sau.
  VUNG CUA SO (z 110..180): ong trong suot; guong 45 do quay thang tren rotor dong co; GAN LUON DAY TF03 o phia sau (-X).
  NAP (z 180..255): TF03 treo chieu xuong + gia; tam day nap co lo cho tia.
Day TF03 phai cat qua mat quet -> di trong gan mong o phia sau -> co vung mu (uoc ~18 do voi chum 15 mm, gia dinh).

Toa do: goc = tam truc quay tren mat day de. Z len. Tia ra o goc 0 huong +X. Mat sau = -X.
Chay: python lidar_v4.py -> lidar_v4.step (Fusion 360), lidar_v4.glb, nhom_chi_tiet_v4.json
"""
import math
import cadquery as cq
from cadquery import Vector, Solid, Compound

# ---------------- kich thuoc chinh [GIA DINH] ----------------
D, WALL = 150.0, 3.0                 # duong kinh vo
BASE_H = 110.0                       # chieu cao de
WIN_OD, WIN_T, WIN_H = 140.0, 3.0, 70.0
SCAN_Z = 145.0                       # tam guong = mat quet
CAP_Z0, CAP_H = BASE_H + WIN_H, 75.0 # nap chua TF03
TF_W, TF_T, TF_L = 44.0, 31.8, 42.9  # TF03 (*)
TF_LENS_Z = 205.0                    # mat kinh TF03 (huong xuong)
PLATE_Z = 88.0                       # tam do dong co
MOT_D, MOT_H = 40.0, 18.0            # BLDC gimbal (dang GM3506)
PCB_Z, PCB_T = 10.0, 1.6
RIB_X = -62.0                        # gan luon day TF03 (phia sau)

C = {
    "vo": (0.56, 0.59, 0.63, 1), "vo_toi": (0.28, 0.30, 0.33, 1), "kinh": (0.55, 0.75, 0.95, 0.25),
    "pcb": (0.05, 0.40, 0.20, 1), "ic": (0.08, 0.08, 0.09, 1), "kim": (0.80, 0.80, 0.82, 1),
    "relay": (0.15, 0.35, 0.80, 1), "trang": (0.95, 0.95, 0.92, 1), "xanhla": (0.20, 0.65, 0.30, 1),
    "tf03": (0.12, 0.12, 0.14, 1), "thau_kinh": (0.25, 0.10, 0.10, 1), "guong": (0.90, 0.92, 0.95, 1),
    "nhom": (0.76, 0.77, 0.79, 1), "dongco": (0.18, 0.18, 0.20, 1), "nam_cham": (0.50, 0.20, 0.20, 1),
    "do": (0.85, 0.12, 0.10, 1), "den": (0.06, 0.06, 0.06, 1), "nau": (0.45, 0.25, 0.10, 1),
    "xanh": (0.10, 0.30, 0.85, 1), "vang": (0.90, 0.75, 0.10, 1), "xam": (0.55, 0.55, 0.58, 1),
    "cam_d": (0.95, 0.50, 0.10, 1), "tim": (0.45, 0.25, 0.65, 1),
}
assy = cq.Assembly(name="lidar_v4")
nhom = {}


def add(obj, name, color, group):
    assy.add(obj, name=name, color=cq.Color(*C[color]))
    nhom[name] = group


def box(x, y, z, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").box(x, y, z, centered=(True, True, False)).translate((cx, cy, z0))


def cyl(d, h, cx=0, cy=0, z0=0):
    return cq.Workplane("XY").circle(d / 2).extrude(h).translate((cx, cy, z0))


def tube(od, idd, h, z0=0, cx=0, cy=0):
    return cq.Workplane("XY").circle(od / 2).circle(idd / 2).extrude(h).translate((cx, cy, z0))


def wire(pts, d=1.6):
    """Day dien = chuoi tru noi cac diem + cau o khop (compound, khong boolean cho nhanh)."""
    r, sol = d / 2, []
    for a, b in zip(pts[:-1], pts[1:]):
        va, vb = Vector(*a), Vector(*b)
        L = (vb - va).Length
        if L > 1e-6:
            sol.append(Solid.makeCylinder(r, L, va, (vb - va).normalized()))
    for p in pts[1:-1]:
        sol.append(Solid.makeSphere(r, Vector(*p), angleDegrees1=-90, angleDegrees2=90))
    return Compound.makeCompound(sol)


def bundle(pts, colors, name, group, d=1.4, axis="y"):
    """Bo nhieu day song song, lech nhau theo truc y (hoac x)."""
    n = len(colors)
    for i, col in enumerate(colors):
        o = (i - (n - 1) / 2) * d * 1.05
        p2 = [(x, y + o, z) if axis == "y" else (x + o, y, z) for (x, y, z) in pts]
        add(wire(p2, d), f"{name}_{i}", col, group)


# ======================= DE =======================
shell = tube(D, D - 2 * WALL, BASE_H - 5, z0=5)
floor = cyl(D, 5)
lid = cyl(D, 3, z0=BASE_H - 3).cut(cyl(MOT_D + 6, 10, z0=BASE_H - 5)).cut(box(8, 6, 10, RIB_X, 0, BASE_H - 5))
base = shell.union(floor).union(lid)
# lo cho dau noi o mat sau (-X): nguon (y=+25), Ethernet (y=-25), van tho (y=0, cao hon)
for y, zc, dd in ((25, 50, 16.5), (-25, 50, 16.5), (0, 80, 12.5)):
    base = base.cut(cq.Workplane("YZ").circle(dd / 2).extrude(20).translate((-D / 2 - 5, y, zc)))
add(base, "vo_de", "vo", "vo")
add(box(190, 120, 6, 0, 0, -6).cut(cq.Workplane("XY").pushPoints([(-80, -45), (-80, 45), (80, -45), (80, 45)])
    .circle(4.5).extrude(10).translate((0, 0, -8))), "tam_gan_day", "vo_toi", "vo")

# dau noi o mat sau
def panel_conn(y, z, name, label_col):
    body = (cq.Workplane("YZ").circle(9).extrude(14).translate((-D / 2 - 14 + 1, y, z))
            .union(cq.Workplane("YZ").polygon(6, 24).extrude(3).translate((-D / 2 - 1, y, z)))
            .union(cq.Workplane("YZ").circle(7.5).extrude(10).translate((-D / 2 + 2, y, z))))
    add(body, name, "kim", "ket_noi")
    add(cq.Workplane("YZ").circle(10).circle(9).extrude(2).translate((-D / 2 - 14, y, z)), name + "_vach", label_col, "ket_noi")
panel_conn(25, 50, "m12_nguon_24v", "do")
panel_conn(-25, 50, "m12_ethernet", "xanh")
add(cq.Workplane("YZ").circle(7).extrude(8).translate((-D / 2 - 7, 0, 80))
    .union(cq.Workplane("YZ").polygon(6, 16).extrude(3).translate((-D / 2 - 1, 0, 80))), "van_tho", "vo_toi", "ket_noi")

# ======================= BO MACH CHINH =======================
SO = [(60 * math.cos(math.radians(a)), 60 * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]
pcb = cyl(D - 2 * WALL - 6, PCB_T, z0=PCB_Z)
for x, y in SO:
    pcb = pcb.cut(cyl(4.5, 5, x, y, PCB_Z - 1))
add(pcb, "pcb_chinh", "pcb", "pcb")
zt = PCB_Z + PCB_T
parts = [  # ten, (x,y,z), (cx,cy), mau
    ("stm32f4", (10, 10, 1.4), (5, 0), "ic"),
    ("thach_anh", (5, 3.2, 1), (5, 10), "kim"),
    ("w5500_ethernet", (7, 7, 0.9), (-12, -22), "ic"),
    ("bien_ap_ethernet", (12, 10, 6), (-30, -24), "ic"),
    ("drv_foc", (6, 6, 1.0), (28, 12), "ic"),
    ("mosfet_1", (5, 6, 1.2), (36, 18), "ic"), ("mosfet_2", (5, 6, 1.2), (36, 10), "ic"),
    ("mosfet_3", (5, 6, 1.2), (36, 2), "ic"),
    ("buck_5v_ic", (3, 3, 1.0), (-14, 30), "ic"),
    ("cuon_cam_5v", (10, 10, 5), (-2, 32), "ic"),
    ("ldo_3v3", (6.5, 3.5, 1.6), (12, 30), "ic"),
    ("cau_chi_24v", (10, 5, 5), (-32, 44), "kim"),
    ("tvs_24v", (5, 3, 2.3), (-22, 44), "ic"),
    ("diode_nguoc_cuc", (5, 3, 2.3), (-22, 38), "ic"),
    ("cau_dau_24v", (10.2, 8, 10), (-44, 34), "xanhla"),
    ("jst_tf03_4p", (10, 6, 6), (-46, -2), "trang"),
    ("jst_dongco_3p", (8, 6, 6), (46, 10), "trang"),
    ("jst_encoder_6p", (14, 6, 6), (30, -18), "trang"),
    ("jst_ethernet", (10, 6, 6), (-46, -22), "trang"),
    ("swd", (12.7, 6, 5), (12, -34), "ic"),
    ("relay_1", (15.5, 10.5, 11.5), (-20, -48), "relay"),
    ("relay_2", (15.5, 10.5, 11.5), (0, -50), "relay"),
    ("relay_3", (15.5, 10.5, 11.5), (20, -48), "relay"),
    ("cau_dau_relay", (30.5, 8, 10), (0, -62), "xanhla"),
]
for name, (sx, sy, sz), (cx, cy), col in parts:
    add(box(sx, sy, sz, cx, cy, zt), name, col, "pcb")
for i, (cx, cy) in enumerate([(-30, 30), (14, 40), (46, -4), (-6, 44)]):   # tu dien
    add(cyl(8, 10, cx, cy, zt), f"tu_{i}", "kim", "pcb")
for i, cx in enumerate((-12, -8, -4)):
    add(box(1.6, 0.8, 0.6, cx, 14, zt), f"led_{i}", "relay", "pcb")

# ======================= DONG CO + ENCODER =======================
plate = cyl(D - 2 * WALL - 4, 4, z0=PLATE_Z).cut(cyl(12, 10, z0=PLATE_Z - 1))
for x, y in SO:
    plate = plate.cut(cyl(4, 10, x, y, PLATE_Z - 1))
# khoet cho bo day di len
plate = plate.cut(box(14, 24, 10, -60, 0, PLATE_Z - 1)).cut(box(14, 18, 10, 50, 20, PLATE_Z - 1))
add(plate, "tam_do_dong_co", "nhom", "dong_co")
for i, (x, y) in enumerate(SO):
    add(cyl(8, PLATE_Z - PCB_Z - PCB_T, x, y, zt), f"tru_do_{i}", "kim", "dong_co")
MZ0 = PLATE_Z + 4
add(cyl(MOT_D - 6, 6, z0=MZ0), "stator_de", "dongco", "dong_co")
add(cyl(MOT_D, MOT_H - 6, z0=MZ0 + 6).cut(cyl(MOT_D - 8, 2, z0=MZ0 + MOT_H - 1)), "rotor_dong_co", "dongco", "dong_co")
add(cyl(6, 3, z0=PLATE_Z - 3), "nam_cham_truc", "nam_cham", "dong_co")
add(cyl(26, 1.6, z0=PLATE_Z - 7), "encoder_as5048a", "pcb", "dong_co")
add(box(4, 4, 1, 0, 0, PLATE_Z - 5.4), "chip_as5048a", "ic", "dong_co")
for i, a in enumerate((90, 210, 330)):   # 3 tru treo bo encoder
    add(cyl(3, 5.4, 9 * math.cos(math.radians(a)), 9 * math.sin(math.radians(a)), PLATE_Z - 5.4), f"tru_enc_{i}", "kim", "dong_co")

# ======================= PHAN QUAY: coc guong (TF03 chieu xuong) =======================
CUP0 = BASE_H + 0.5
cup = cyl(48, SCAN_Z + 26 - CUP0, z0=CUP0)
# mat guong: z = SCAN_Z - x ; giu phan duoi mat (z < SCAN_Z - x), bo phan tren
cutter = (cq.Workplane("XZ").polyline([(-90, SCAN_Z + 90), (90, SCAN_Z - 90), (90, 400), (-90, 400)]).close()
          .extrude(60, both=True))
cup = cup.cut(cutter)
cup = cup.union(cyl(30, 4, z0=CUP0 - 0.5))                            # mat bich bat vao rotor
add(cup, "coc_guong", "nhom", "phan_quay")
nrm = Vector(1, 0, 1).normalized()
mir = (cq.Workplane("XY").ellipse(22 * math.sqrt(2), 22).extrude(2.5)
       .rotate((0, 0, 0), (0, 1, 0), 45).translate((0, 0, SCAN_Z)).translate(tuple(-2.5 * nrm)))
add(mir, "guong_45", "guong", "phan_quay")

# ======================= CUA SO + GAN LUON DAY =======================
add(tube(WIN_OD, WIN_OD - 2 * WIN_T, WIN_H, z0=BASE_H), "ong_cua_so", "kinh", "cua_so")
# gan mong: 4 mm theo phuong tiep tuyen (y), 10 mm theo ban kinh (x) -> it che tia nhat
rib = box(10, 4, WIN_H, RIB_X, 0, BASE_H).cut(box(6, 2.4, WIN_H + 2, RIB_X, 0, BASE_H - 1))
add(rib, "gan_luon_day", "vo_toi", "cua_so")

# ======================= NAP + TF03 =======================
cap = tube(D, D - 2 * WALL, CAP_H - 6, z0=CAP_Z0)
cap = cap.union(cyl(D, 6, z0=CAP_Z0 + CAP_H - 6))
cap = cap.union(cq.Workplane("XY").sphere(D * 0.75).translate((0, 0, CAP_Z0 + CAP_H - D * 0.75 + 10))
                .intersect(cyl(D, 20, z0=CAP_Z0 + CAP_H)))
cap = cap.union(tube(D + 10, D - 2, 4, z0=CAP_Z0))                     # vanh che nang (che cua so)
add(cap, "nap", "vo", "nap")
floorp = cyl(D - 2 * WALL, 3, z0=CAP_Z0).cut(cyl(40, 10, z0=CAP_Z0 - 1)).cut(box(8, 6, 10, RIB_X, 0, CAP_Z0 - 1))
add(floorp, "tam_day_nap", "vo_toi", "nap")
tf = box(TF_W, TF_T, TF_L, z0=TF_LENS_Z)
for dx in (-10, 10):
    tf = tf.cut(cyl(15, 2, dx, 0, TF_LENS_Z - 0.5))
add(tf, "tf03", "tf03", "tf03")
for i, dx in enumerate((-10, 10)):
    add(cyl(14, 0.5, dx, 0, TF_LENS_Z + 1), f"thau_kinh_{i}", "thau_kinh", "tf03")
top_in = CAP_Z0 + CAP_H - 6
for sy in (-1, 1):
    add(box(TF_W + 10, 3, top_in - TF_LENS_Z + 2, 0, sy * (TF_T / 2 + 1.5), TF_LENS_Z - 2), f"gia_tf03_{sy}", "nhom", "tf03")

# ======================= DAY DIEN =======================
# TF03 (4 day: do VCC, den GND, nau TX, xanh duong RX) - tu dinh TF03 -> gan -> xuong de -> JST tren PCB
tf_top = TF_LENS_Z + TF_L
p_tf = [(-TF_W / 2 + 1, 0, tf_top - 12), (-TF_W / 2 - 8, 0, tf_top - 12), (RIB_X + 4, 0, tf_top - 20),
        (RIB_X, 0, CAP_Z0 + 5), (RIB_X, 0, BASE_H - 5), (RIB_X, 0, PLATE_Z + 6), (-60, 0, PLATE_Z - 10),
        (-56, -2, 30), (-48, -2, zt + 8), (-46, -2, zt + 6)]
bundle(p_tf, ["do", "den", "nau", "xanh"], "day_tf03", "day_dien", d=1.2, axis="x")
# dong co 3 pha
p_m = [(18, 0, MZ0 + 3), (30, 8, MZ0 + 3), (50, 20, PLATE_Z + 6), (50, 20, PLATE_Z - 6), (52, 14, 40), (47, 10, zt + 8), (46, 10, zt + 6)]
bundle(p_m, ["vang", "xanhla", "xanh"], "day_dong_co", "day_dien", d=1.2, axis="x")
# encoder (6 soi xam)
p_e = [(10, -8, PLATE_Z - 6.5), (24, -14, PLATE_Z - 10), (32, -18, 50), (31, -18, zt + 8), (30, -18, zt + 6)]
bundle(p_e, ["xam"] * 6, "day_encoder", "day_dien", d=0.9, axis="x")
# nguon 24 V: M12 -> cau dau
p_p = [(-D / 2 + 12, 25, 50), (-60, 30, 45), (-50, 34, 30), (-45, 34, zt + 12), (-44, 34, zt + 9)]
bundle(p_p, ["do", "den"], "day_nguon_24v", "day_dien", d=2.0, axis="x")
# Ethernet: M12 -> JST (4 soi)
p_n = [(-D / 2 + 12, -25, 50), (-60, -24, 40), (-50, -22, 25), (-47, -22, zt + 8), (-46, -22, zt + 6)]
bundle(p_n, ["cam_d", "trang", "xanhla", "tim"], "day_ethernet", "day_dien", d=1.0, axis="y")

if __name__ == "__main__":
    import json, os
    here = os.path.dirname(os.path.abspath(__file__))
    assy.save(os.path.join(here, "lidar_v4.step"))
    assy.save(os.path.join(here, "lidar_v4.glb"), tolerance=0.3, angularTolerance=0.4)
    json.dump(nhom, open(os.path.join(here, "nhom_chi_tiet_v4.json"), "w"), ensure_ascii=False, indent=1)
    bb = assy.toCompound().BoundingBox()
    print("so chi tiet:", len(nhom), "| bao (mm): %.0f x %.0f x %.0f" % (bb.xlen, bb.ylen, bb.zlen))
