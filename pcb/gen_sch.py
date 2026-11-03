"""gen_sch.py - Sinh so do nguyen ly KiCad 7 (lidar_carrier.kicad_sch) tu design.py.
Kieu ve: moi chan co mot doan day ngan + nhan net (label). Hai chan cung ten nhan = noi voi nhau."""
import copy, math, uuid
from kiutils.schematic import Schematic
from kiutils.symbol import SymbolLib
from kiutils.items.common import Position, Property, Effects, Font, PageSettings, TitleBlock, Justify
from kiutils.items.schitems import (SchematicSymbol, SymbolProjectInstance, SymbolProjectPath, LocalLabel,
                                    Connection, NoConnect, HierarchicalSheetInstance, Text)
import design

PROJECT = "lidar_carrier"
U = lambda: str(uuid.uuid4())
LIBDIR = "/usr/share/kicad/symbols"
_libs = {}


def lib_symbol(lib, name):
    if lib not in _libs:
        _libs[lib] = SymbolLib.from_file(f"{LIBDIR}/{lib}.kicad_sym")
    L = _libs[lib]
    sym = [s for s in L.symbols if s.entryName == name][0]
    base = sym
    while base.extends:
        base = [s for s in L.symbols if s.entryName == base.extends][0]
    flat = copy.deepcopy(base)
    if base is not sym:                       # ky hieu ke thua: lay than cua ky hieu goc, thuoc tinh cua ky hieu con
        props = {p.key: p for p in sym.properties}
        for p in flat.properties:
            if p.key in props:
                p.value = props[p.key].value
        flat.extends = None
    flat.entryName = name
    for u in flat.units:
        u.entryName = name
    flat.libraryNickname = lib
    return flat


def pins_of(flat):
    out = {}
    for u in flat.units:
        for p in u.pins:
            out[p.number] = p
    return out


def xform(px, py, rot, sx, sy):
    """toa do thu vien (Y huong len) -> toa do so do (Y huong xuong), xoay rot do nguoc kim dong ho."""
    x, y = px, -py
    r = math.radians(rot)
    return (round(sx + x * math.cos(r) + y * math.sin(r), 2),
            round(sy - x * math.sin(r) + y * math.cos(r), 2))


def snap(v):
    return round(round(v / 1.27) * 1.27, 2)


sch = Schematic.create_new()
sch.version = "20230121"
sch.generator = "eeschema"
sch.uuid = U()
sch.paper = PageSettings(paperSize="A3")
sch.titleBlock = TitleBlock(title="Mach mang LiDAR 2D - shield cho NUCLEO-F411RE", date="2026-10-31", revision="0.1",
                            company="DATN - LiDAR 2D tu TF03",
                            comments={1: "Nguon 24V -> LM2596S-5.0 -> 5V; driver CL42C qua MOSFET AO3400A",
                                      2: "Ban nhap - can kiem tra lai truoc khi dat mach"})
sch.sheetInstances = [HierarchicalSheetInstance(instancePath="/", page="1")]

added_lib = {}
ENDPOINTS = {}   # diem cuoi doan day -> net, de phat hien hai net cham nhau
FONT = Effects(font=Font(width=1.27, height=1.27))


SCALE, OX, OY = 1.4, -8.0, -12.0
def S(v, o):
    return round(round((v * SCALE + o) / 2.54) * 2.54, 2)


def place(ref, lib, name, value, fp, x, y, rot, netmap, dnp=False):
    x, y = S(x, OX), S(y, OY)
    key = f"{lib}:{name}"
    if key not in added_lib:
        added_lib[key] = lib_symbol(lib, name)
        sch.libSymbols.append(added_lib[key])
    flat = added_lib[key]
    is_power = ref.startswith("#")
    s = SchematicSymbol(libraryNickname=lib, entryName=name, position=Position(X=x, Y=y, angle=rot), unit=1,
                        inBom=not is_power and not ref.startswith(("TP", "H")), onBoard=True, dnp=dnp, uuid=U())
    # thuoc tinh
    for i, (k, v, dy) in enumerate((("Reference", ref, -1), ("Value", value, 1), ("Footprint", fp, 2), ("Datasheet", "", 3))):
        eff = Effects(font=Font(width=1.27, height=1.27), hide=(k in ("Footprint", "Datasheet") or is_power and k == "Reference"))
        s.properties.append(Property(key=k, value=v, id=i, position=Position(X=round(x + 3.81, 2), Y=round(y + dy * 2.54, 2), angle=0), effects=eff))
    pins = pins_of(flat)
    for num in pins:
        s.pins[num] = U()
    s.instances = [SymbolProjectInstance(name=PROJECT, paths=[SymbolProjectPath(sheetInstancePath=f"/{sch.uuid}", reference=ref, unit=1)])]
    sch.schematicSymbols.append(s)
    # day + nhan cho tung chan
    for num, net in netmap.items():
        p = pins[num]
        ex, ey = xform(p.position.X, p.position.Y, rot, x, y)
        a = math.radians(p.position.angle)
        dx, dy = xform(-math.cos(a), -math.sin(a), rot, 0, 0)     # huong ra ngoai than
        if net is None:
            sch.noConnects.append(NoConnect(position=Position(X=ex, Y=ey), uuid=U()))
            continue
        tx, ty = round(ex + dx * 2.54, 2), round(ey + dy * 2.54, 2)
        for pt in ((ex, ey), (tx, ty)):
            old = ENDPOINTS.get(pt)
            if old and old != net:
                raise SystemExit(f"VA CHAM: {ref} chan {num} ({net}) cham net {old} tai {pt}")
            ENDPOINTS[pt] = net
        sch.graphicalItems.append(Connection(type="wire", points=[Position(X=ex, Y=ey), Position(X=tx, Y=ty)], uuid=U()))
        if abs(dx) > abs(dy):
            ang, just = (0, "left") if dx > 0 else (180, "right")
        else:
            ang, just = (90, "left") if dy < 0 else (270, "right")
        sch.labels.append(LocalLabel(text=net, position=Position(X=tx, Y=ty, angle=ang),
                                     effects=Effects(font=Font(width=1.27, height=1.27), justify=Justify(horizontally=just)),
                                     uuid=U()))


for ref, (lib, name), value, fp, x, y, rot, netmap in design.PARTS:
    place(ref, lib, name, value, fp, x, y, rot, netmap, dnp=ref in design.DNP)

for i, (net, x, y) in enumerate(design.PWR_FLAGS):
    place(f"#FLG0{i+1}", "power", "PWR_FLAG", "PWR_FLAG", "", x, y, 0, {"1": net})

# tieu de cac khoi
for txt, x, y in [("1. NGUON VAO 24V + BAO VE", 20.32, 25.4), ("2. BUCK LM2596S-5.0 -> 5V", 127, 25.4),
                  ("3. NUCLEO-F411RE (header Arduino)", 127, 96.52), ("4. DRIVER CL42C (chung duong)", 15.24, 101.6),
                  ("5. TF03", 170.18, 91.44), ("6. CAM BIEN KHE (moc 0 do)", 160.02, 124.46),
                  ("7. ENCODER DONG CO (tuy chon - DNP)", 160.02, 157.48), ("8. NUT + LED", 63.5, 76.2),
                  ("9. DIEM DO + LO BAT VIT", 238.76, 68.58)]:
    sch.texts.append(Text(text=txt, position=Position(X=S(x, OX), Y=S(y, OY) - 7.62, angle=0),
                          effects=Effects(font=Font(width=2, height=2, bold=True), justify=Justify(horizontally="left")), uuid=U()))
notes = ("GHI CHU:\n"
         "- Shield cam len header Arduino cua NUCLEO-F411RE. Nucleo cap nguon qua USB (ST-Link).\n"
         "  Muon chay doc lap: noi J2 (E5V_OUT) toi CN7 chan 6 (E5V), dat JP5 o vi tri 2-3 (UM1724).\n"
         "- PUL/DIR/ENA: driver dat jumper 5V; MOSFET keo PUL-/DIR-/ENA- xuong GND. Logic khong dao.\n"
         "- Gia tri buck theo datasheet TI LM2596 (SNVS124): Cin 470uF/50V, Cout 220uF/25V, L 68uH, diode 40V Schottky.\n"
         "- TF03: UART LVTTL 3.3V, nguon 5-24V, ~150 mA o 5V (datasheet TF03-180 V1.2).")
sch.texts.append(Text(text=notes.replace("\n", "\\n"), position=Position(X=S(205, OX), Y=S(178, OY), angle=0),
                      effects=Effects(font=Font(width=1.5, height=1.5), justify=Justify(horizontally="left")), uuid=U()))

sch.to_file(f"{PROJECT}.kicad_sch")
print("schematic:", len(sch.schematicSymbols), "symbols,", len(sch.labels), "labels")
