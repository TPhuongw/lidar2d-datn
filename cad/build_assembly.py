"""
build_assembly.py - Xuat file lap rap LiDAR 2D (STEP nhieu component) de mo trong Fusion 360.
Component: Mica_base, Motor_tower, Motor_42CME06, Coupler, Rotor_SICK (dia + coc + guong + tia quet),
           Post_2020, TF03_bracket, Slot_sensor_bracket, TF03, Laser_down.
Kich thuoc dong co / coc guong / TF03 la UOC LUONG (khoi gia dinh) - thay bang so do that sau.
"""
import cadquery as cq

DISK_Z, MIRROR_Z, LENS_Z, POST_X = 113.0, 160.0, 200.0, -80.0

mica = cq.Workplane("XY").box(280, 240, 5, centered=(True, True, False)).translate((-40, 40, -5))
tower = cq.importers.importStep("nema17_tower_mount.step")
tf_br = cq.importers.importStep("tf03_bracket.step")
opb_br = cq.importers.importStep("opb871_bracket.step")

# dong co: than 42.3 vuong dai 70, go dinh tam D22 cao 2, truc D5 dai 24
motor = (cq.Workplane("XY").box(42.3, 42.3, 70, centered=(True, True, False)).translate((0, 0, 8))
         .union(cq.Workplane("XY").circle(11).extrude(2).translate((0, 0, 78)))
         .union(cq.Workplane("XY").circle(2.5).extrude(24).translate((0, 0, 80))))
coupler = (cq.Workplane("XY").circle(7).circle(2.5).extrude(25).translate((0, 0, 88)))

# rotor SICK (gia dinh): dia D90 day 1.5 + coc D54 cao 14 + than D36 + guong 40x60 nghieng 45 + tia quet
disk = cq.Workplane("XY").circle(45).extrude(1.5).translate((0, 0, DISK_Z))
cup = cq.Workplane("XY").circle(27).extrude(14).translate((0, 0, DISK_Z + 1.5))
# than coc: tru D36, dinh cat xien 45 do; guong nam TREN mat xien (khong cat vao than)
body = cq.Workplane("XY").circle(18).extrude(70).translate((0, 0, DISK_Z + 15.5))
n = 0.7071
cutter = (cq.Workplane("XY").box(300, 300, 300).translate((0, 0, 150))
          .rotate((0, 0, 0), (0, 1, 0), 45).translate((-1 * n, 0, MIRROR_Z - 1 * n)))
body = body.cut(cutter)
mirror = (cq.Workplane("XY").box(56, 40, 2).rotate((0, 0, 0), (0, 1, 0), 45)
          .translate((0, 0, MIRROR_Z)))
beam_out = (cq.Workplane("YZ").circle(0.8).extrude(400).translate((0, 0, MIRROR_Z)))   # tia ra theo +X
rotor_solid = disk.union(cup).union(body).union(mirror)

post = cq.Workplane("XY").box(20, 20, 300, centered=(True, True, False)).translate((POST_X, 0, 0))
tf03 = (cq.Workplane("XY").box(31.8, 44, 42.9, centered=(True, True, False)).translate((0, 0, LENS_Z))
        .cut(cq.Workplane("XY").pushPoints([(0, -10), (0, 10)]).circle(7).extrude(1.5)
             .translate((0, 0, LENS_Z))))
# cum mach: Nucleo (70 x 82.5) tren cot do 10 mm + shield 100 x 80 cam chong len (gia dinh)
PCB_C = (-75.0, 90.0)
nucleo = cq.Workplane("XY").box(82.5, 70, 1.6, centered=(True, True, False)).translate((PCB_C[0] + 8, PCB_C[1], 10))
shield = cq.Workplane("XY").box(100, 80, 1.6, centered=(True, True, False)).translate((PCB_C[0], PCB_C[1], 23))
standoffs = (cq.Workplane("XY").pushPoints([(PCB_C[0] + dx, PCB_C[1] + dy) for dx in (-46.5, 46.5) for dy in (-36.5, 36.5)])
             .circle(2.5).extrude(23))
beam_down = cq.Workplane("XY").circle(0.8).extrude(LENS_Z - MIRROR_Z).translate((0, 0, MIRROR_Z))

col = lambda r, g, b, a=1.0: cq.Color(r, g, b, a)
asm = cq.Assembly(name="LiDAR2D")
asm.add(mica, name="Mica_base", color=col(0.85, 0.85, 0.85, 0.6))
asm.add(tower, name="Motor_tower", color=col(0.55, 0.7, 0.87))
asm.add(motor, name="Motor_42CME06", color=col(0.25, 0.25, 0.25))
asm.add(coupler, name="Coupler", color=col(0.7, 0.7, 0.7))
rotor = cq.Assembly(name="Rotor_SICK")
rotor.add(rotor_solid, name="Rotor_body", color=col(0.89, 0.71, 0.28))
rotor.add(beam_out, name="Laser_scan", color=col(1, 0, 0))
asm.add(rotor, name="Rotor_SICK")
asm.add(post, name="Post_2020", color=col(0.6, 0.65, 0.65))
asm.add(tf_br, name="TF03_bracket", color=col(0.3, 0.55, 0.84))
asm.add(opb_br, name="Slot_sensor_bracket", color=col(0.84, 0.53, 0.3))
asm.add(nucleo, name="Nucleo_F411RE", color=col(0.95, 0.95, 0.95))
asm.add(shield, name="PCB_carrier", color=col(0.1, 0.45, 0.2))
asm.add(standoffs, name="PCB_standoffs", color=col(0.75, 0.75, 0.6))
asm.add(tf03, name="TF03", color=col(0.24, 0.43, 0.36))
asm.add(beam_down, name="Laser_down", color=col(1, 0, 0))
asm.save("lidar2d_assembly.step")
print("da xuat lidar2d_assembly.step")
