"""
lidar_frame.py - Cac chi tiet in 3D cho khung LiDAR 2D (42CME06 + coc guong SICK + TF03 + cot nhom 2020).

Toa do: goc (0,0,0) = tam truc quay tren mat tam mica. Z huong len. Cot 2020 dat tai x = POST_X.
Xuat:
  tf03_bracket.step/.stl   : gia treo TF03, kep vao cot 2020 bang 2 vit M5 + dai oc chu T
  opb871_bracket.step/.stl : gia OPB871, kep vao cot 2020, do vach dia encoder
  assembly_preview.png     : anh lap rap (dong co, coc guong, TF03 la khoi gia dinh)

CAC SO DANH DAU (*) LA UOC LUONG - DO LAI ROI SUA O DAY.
"""
import cadquery as cq

# ---------------- bo tri chung ----------------
POST_X      = -80.0     # tam cot 2020 (so voi truc quay)
POST_W      = 20.0
SLOT_OPEN   = 6.2       # do mo ranh 2020
M5_CLR      = 5.5

# ---------------- TF03 (*) ----------------
TF_W        = 44.0      # canh song song mat lap, vuong goc tia (*)
TF_H        = 42.9      # canh doc theo tia (*)
TF_T        = 31.8      # be day (*)
TF_ROW_GAP  = 38.0      # 2 hang lo M3 cach nhau (*)
TF_PITCH    = 10.0      # buoc lo trong 1 hang (*)
TF_HOLE0    = 7.0       # lo dau cach mat kinh (*)
LENS_Z      = 200.0     # cao do mat kinh TF03 (*) - chinh bang cach truot tren cot

# ---------------- dia encoder + OPB871 (*) ----------------
DISK_Z      = 113.0     # cao do mat dia encoder (*)
DISK_R_SLOT = 43.0      # ban kinh hang vach (*)

PLATE_T = 5.0


def post_clamp(z0, height, width=30.0):
    """Tam ap vao mat cot (mat x = POST_X + 10), co then 5.8 mm cam vao ranh, 2 lo M5."""
    x_face = POST_X + POST_W / 2
    plate = (cq.Workplane("YZ", origin=(x_face, 0, z0))
             .rect(width, height, centered=(True, False)).extrude(PLATE_T))
    key = (cq.Workplane("YZ", origin=(x_face, 0, z0))
           .rect(SLOT_OPEN - 0.4, height, centered=(True, False)).extrude(-1.5))
    holes = (cq.Workplane("YZ", origin=(x_face - 2, 0, z0))
             .pushPoints([(0, height * 0.25), (0, height * 0.75)])
             .circle(M5_CLR / 2).extrude(PLATE_T + 4))
    return plate.union(key).cut(holes), x_face + PLATE_T


# ======================= GIA TF03 =======================
tf_x_back = -TF_T / 2                      # mat lap cua TF03 nam o x = -TF_T/2, tam TF03 tren truc
z_lo, z_hi = LENS_Z - 2, LENS_Z + TF_H + 4
clamp, x_arm0 = post_clamp(z_lo, z_hi - z_lo)

# tam dung ap vao mat lap TF03
tf_plate = (cq.Workplane("YZ", origin=(tf_x_back - PLATE_T, 0, z_lo))
            .rect(TF_W + 8, z_hi - z_lo, centered=(True, False)).extrude(PLATE_T))
# 6 lo M3 dang ranh dung +-2 mm (bu sai so uoc luong)
pts = [(sy * TF_ROW_GAP / 2, LENS_Z + TF_HOLE0 + i * TF_PITCH) for sy in (-1, 1) for i in range(3)]
slots = (cq.Workplane("YZ", origin=(tf_x_back - PLATE_T - 1, 0, 0))
         .pushPoints(pts).slot2D(7.4, 3.4, angle=90).extrude(PLATE_T + 2))
tf_plate = tf_plate.cut(slots)

# 2 thanh ngang noi kep cot voi tam TF03 (de ho giua cho day cap)
arm_len = (tf_x_back - PLATE_T) - x_arm0
arms = (cq.Workplane("XY", origin=(x_arm0, 0, z_lo))
        .pushPoints([(arm_len / 2, -11), (arm_len / 2, 11)])
        .rect(arm_len, 6).extrude(z_hi - z_lo))
tf_bracket = clamp.union(tf_plate).union(arms)

# ======================= GIA OPB871 =======================
# OPB871 dat o phia cot (vung mu), khe cua no om lay mep dia tai ban kinh DISK_R_SLOT.
oz0 = DISK_Z - 30
oclamp, ox0 = post_clamp(oz0, 40, width=30)
x_sensor = -DISK_R_SLOT
shelf_len = x_sensor + 8 - ox0
shelf = (cq.Workplane("XY", origin=(ox0, 0, oz0 + 40 - PLATE_T))
         .center(shelf_len / 2, 0).rect(shelf_len, 24).extrude(PLATE_T))
# ranh M3 dai 14 mm theo X de chinh vi tri cam bien theo ban kinh
oslot = (cq.Workplane("XY", origin=(x_sensor, 0, oz0 + 30))
         .slot2D(14, 3.4, angle=0).extrude(20))
rib = (cq.Workplane("XZ", origin=(0, 2, 0))
       .polyline([(ox0, oz0 + 5), (ox0, oz0 + 40 - PLATE_T), (ox0 + shelf_len * 0.7, oz0 + 40 - PLATE_T)])
       .close().extrude(4))
opb_bracket = oclamp.union(shelf).union(rib).cut(oslot)

for name, part in (("tf03_bracket", tf_bracket), ("opb871_bracket", opb_bracket)):
    cq.exporters.export(part, f"{name}.step")
    cq.exporters.export(part, f"{name}.stl", tolerance=0.05, angularTolerance=0.1)
    bb = part.val().BoundingBox()
    print(f"{name}: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, the tich {part.val().Volume()/1000:.1f} cm3")

# ======================= khoi gia dinh de xem lap rap =======================
ref = {}
ref["mica"] = cq.Workplane("XY").box(240, 160, 5, centered=(True, True, False)).translate((-20, 0, -5))
ref["post"] = cq.Workplane("XY").box(20, 20, 300, centered=(True, True, False)).translate((POST_X, 0, 0))
ref["motor"] = cq.Workplane("XY").box(42.3, 42.3, 70, centered=(True, True, False)).translate((0, 0, 8))
ref["coupler"] = cq.Workplane("XY").circle(7).extrude(25).translate((0, 0, 88))
ref["disk"] = cq.Workplane("XY").circle(45).extrude(1.5).translate((0, 0, DISK_Z))
ref["cup"] = cq.Workplane("XY").circle(27).extrude(14).translate((0, 0, DISK_Z + 1.5))
ref["mirror"] = (cq.Workplane("XY").box(40, 3, 60).rotate((0, 0, 0), (0, 1, 0), 45)
                 .translate((0, 0, 160)))
ref["tf03"] = cq.Workplane("XY").box(TF_T, TF_W, TF_H, centered=(True, True, False)).translate((0, 0, LENS_Z))
tower = cq.importers.importStep("nema17_tower_mount.step")

import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def tris(shape):
    v, t = shape.val().tessellate(0.5)
    v = np.array([(p.x, p.y, p.z) for p in v])
    return v[np.array(t)]


items = [(tower, "#8fb3de"), (tf_bracket, "#4c8bd6"), (opb_bracket, "#d6874c"),
         (ref["post"], "#9aa5a5"), (ref["motor"], "#555555"),
         (ref["coupler"], "#bbbbbb"), (ref["disk"], "#cccccc"), (ref["cup"], "#e3b448"),
         (ref["mirror"], "#bfe0f5"), (ref["tf03"], "#3d6e5c")]
fig = plt.figure(figsize=(14, 7))
for k, (el, az) in enumerate([(18, -55), (18, 35)]):
    ax = fig.add_subplot(1, 2, k + 1, projection="3d")
    for shp, col in items:
        ax.add_collection3d(Poly3DCollection(tris(shp), facecolor=col, edgecolor=(0, 0, 0, 0.08), linewidths=0.2))
    ax.set_xlim(-140, 100); ax.set_ylim(-120, 120); ax.set_zlim(-5, 300)
    ax.set_box_aspect((240, 240, 305)); ax.view_init(el, az); ax.set_axis_off()
fig.suptitle("Lap rap khung LiDAR 2D (xanh dam: gia TF03 in 3D, cam: gia OPB871 in 3D, xanh nhat: thap dong co)")
plt.tight_layout(); plt.savefig("assembly_preview.png", dpi=120)

fig = plt.figure(figsize=(12, 5.5))
for k, (shp, col, title) in enumerate([(tf_bracket, "#4c8bd6", "Gia TF03"), (opb_bracket, "#d6874c", "Gia OPB871")]):
    ax = fig.add_subplot(1, 2, k + 1, projection="3d")
    t = tris(shp)
    ax.add_collection3d(Poly3DCollection(t, facecolor=col, edgecolor=(0, 0, 0, 0.25), linewidths=0.3))
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(22, 40 if k == 0 else -50); ax.set_axis_off(); ax.set_title(title)
plt.tight_layout(); plt.savefig("brackets_preview.png", dpi=120)
print("ok")
