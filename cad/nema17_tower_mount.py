"""
nema17_tower_mount.py - Gia in 3D giu dong co 42CME06 (NEMA 17) TRUC HUONG LEN, bat xuong mat phang.

Dong co bat tu DUOI len mat tam tren bang 4 vit M3 (vit vao tu tren, dau vit chim trong lo khoet).
Hai vach dung o 2 ben, hai phia con lai de ho cho cap dong co / encoder.
Hai chan de co ranh de bat xuong ban bang vit M4.

Tham so NEMA 17 (chuan, CAN DO LAI tren dong co that): lo cach nhau 31.0 mm, go dinh tam D22.
Chay: python nema17_tower_mount.py  -> xuat .step va .stl
"""
import cadquery as cq

# ---------------- tham so (mm) ----------------
HOLE_PITCH   = 31.0    # khoang cach tam 4 lo vit NEMA17
HOLE_D       = 3.4     # lo xuyen cho vit M3 (in 3D: co the khoan lai 3.4)
CBORE_D      = 6.5     # lo khoet dau vit M3 dau tru
CBORE_DEPTH  = 3.5
PILOT_D      = 22.6    # lo cho go dinh tam D22 (+0.6 cho in 3D)

PLATE_W      = 60.0    # tam tren: rong (X) x sau (Y)
PLATE_D      = 60.0
PLATE_T      = 6.0

MOTOR_LEN    = 70.0    # tong chieu dai dong co + encoder (do lai)
UNDER_CLEAR  = 8.0     # khe ho tu day dong co toi mat ban
INNER_GAP    = 48.0    # khoang trong giua 2 vach (than dong co 42.3 + du)
WALL_T       = 5.0

FOOT_W       = 20.0    # chan de nho ra ngoai moi ben
FOOT_T       = 6.0
SLOT_W       = 4.5     # ranh cho vit M4
SLOT_L       = 10.0
SLOT_PITCH_Y = 28.0    # 2 ranh moi chan, cach nhau theo Y (nam giua cac gan)

H_WALL = MOTOR_LEN + UNDER_CLEAR          # chieu cao vach (tu mat ban toi mat duoi tam tren)
Z_TOP = H_WALL + PLATE_T

# tam tren
plate = (cq.Workplane("XY").workplane(offset=H_WALL)
         .rect(PLATE_W, PLATE_D).extrude(PLATE_T))

# 2 vach dung (theo truc Y), dat sat mep trong INNER_GAP
x_wall = INNER_GAP / 2 + WALL_T / 2
walls = (cq.Workplane("XY")
         .pushPoints([(-x_wall, 0), (x_wall, 0)])
         .rect(WALL_T, PLATE_D).extrude(H_WALL))

# 2 chan de nho ra ngoai
x_foot = INNER_GAP / 2 + WALL_T + FOOT_W / 2
feet = (cq.Workplane("XY")
        .pushPoints([(-(x_foot - WALL_T / 2), 0), (x_foot - WALL_T / 2, 0)])
        .rect(FOOT_W + WALL_T, PLATE_D).extrude(FOOT_T))

# gan tang cung (tam giac) giua vach va chan, 2 cai moi ben
def gusset(sign, y):
    x0 = sign * (INNER_GAP / 2 + WALL_T)
    pts = [(x0, FOOT_T), (x0 + sign * (FOOT_W - 4), FOOT_T), (x0, FOOT_T + 25)]
    return (cq.Workplane("XZ", origin=(0, y + 2, 0)).polyline(pts).close().extrude(4))

body = plate.union(walls).union(feet)
for s in (-1, 1):
    for y in (-PLATE_D / 2 + 2, 0.0, PLATE_D / 2 - 2):   # gan o 2 dau va giua, tranh ranh vit
        body = body.union(gusset(s, y))

# lo dinh tam + 4 lo vit co khoet dau vit (tu mat tren)
hp = HOLE_PITCH / 2
pts4 = [(-hp, -hp), (-hp, hp), (hp, -hp), (hp, hp)]
body = (body.faces(">Z").workplane(origin=(0, 0, Z_TOP))
        .hole(PILOT_D)
        .pushPoints(pts4).cboreHole(HOLE_D, CBORE_D, CBORE_DEPTH))

# ranh bat vit M4 tren chan de
slot_pts = [(sx * x_foot, sy * SLOT_PITCH_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
slots = (cq.Workplane("XY").pushPoints(slot_pts)
         .slot2D(SLOT_L, SLOT_W, angle=0).extrude(FOOT_T))
body = body.cut(slots)


cq.exporters.export(body, "nema17_tower_mount.step")
cq.exporters.export(body, "nema17_tower_mount.stl", tolerance=0.05, angularTolerance=0.1)
bb = body.val().BoundingBox()
print(f"Kich thuoc bao: {bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm")
