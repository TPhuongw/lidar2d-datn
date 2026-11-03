"""gen_pcb.py - Tao lidar_carrier.kicad_pcb (100 x 80 mm): vien mach, lo bat vit, dat linh kien, gan net.
Chua di day: nguoi dung tu di day trong KiCad. Footprint duoc lien ket voi ky hieu so do qua UUID,
nen 'Update PCB from Schematic' van khop."""
import json, re
import pcbnew
import design

W, H = 100.0, 80.0
FPDIR = "/usr/share/kicad/footprints"
mm = pcbnew.FromMM

# ---- vi tri tren PCB (mm, goc tren-trai cua mach la 0,0) + goc xoay + mat (F/B) ----
POS = {
    # nguon + buck: dai phia tren
    "J1": (6.5, 17.5, 90), "F1": (17.0, 4.5, 0), "D1": (18.5, 10.5, 180), "D2": (18.5, 17.5, 0),
    "C1": (29.0, 12.0, 0), "C2": (37.0, 4.0, 0),
    "U1": (47.0, 12.5, 0), "D3": (47.0, 21.5, 180), "L1": (64.5, 12.0, 0),
    "C3": (78.0, 10.0, 0), "C4": (78.0, 18.5, 0), "R1": (85.5, 16.0, 90), "D4": (85.5, 21.5, 90),
    "J2": (91.0, 7.5, 0),
    # Nucleo (header Arduino): goc footprint tai chan 1
    "A1": (40.0, 28.0, 0),
    # driver: cot ben trai
    "J3": (5.5, 40.0, 270),
    "Q1": (14.0, 37.0, 0), "Q2": (14.0, 47.0, 0), "Q3": (14.0, 57.0, 0),
    "R2": (22.0, 37.0, 0), "R3": (22.0, 47.0, 0), "R4": (22.0, 57.0, 0),
    "R5": (14.0, 41.5, 0), "R6": (14.0, 51.5, 0), "R7": (14.0, 61.5, 0),
    "R8": (12.0, 67.0, 0), "R9": (19.0, 67.0, 0), "C5": (26.0, 67.0, 0),
    # TF03, cam bien khe, encoder: cot ben phai
    "J4": (95.5, 33.0, 90), "R10": (86.0, 31.0, 0), "R11": (86.0, 35.0, 0), "C6": (86.0, 39.0, 0),
    "J5": (95.5, 50.0, 90), "R12": (86.0, 47.5, 0), "R13": (86.0, 51.5, 0), "C7": (86.0, 55.5, 0),
    "J6": (95.5, 65.0, 90), "R14": (86.0, 61.0, 0), "R15": (86.0, 64.0, 0), "R16": (86.0, 67.5, 0), "R17": (86.0, 70.5, 0),
    # nut + LED + diem do: trong vung giua 2 hang header
    "SW1": (30.0, 42.0, 0), "R20": (44.0, 42.0, 0), "C8": (44.0, 45.5, 0),
    "D5": (52.0, 42.0, 0), "R18": (58.0, 42.0, 0), "D6": (52.0, 46.0, 0), "R19": (58.0, 46.0, 0),
    "TP1": (66.0, 52.0, 0), "TP2": (70.0, 52.0, 0), "TP3": (66.0, 56.0, 0), "TP4": (70.0, 56.0, 0), "TP5": (74.0, 54.0, 0),
    "H1": (3.5, 3.5, 0), "H2": (96.5, 3.5, 0), "H3": (3.5, 76.5, 0), "H4": (96.5, 76.5, 0),
}
OX, OY = 100.0, 100.0           # dat mach tai (100,100) mm tren trang ve

# ---- UUID ky hieu tu so do ----
sch = open("lidar_carrier.kicad_sch").read()
uuids = {}
for m in re.finditer(r'\(symbol \(lib_id "[^"]+"\).*?\n  \)\n', sch, re.S):
    blk = m.group(0)
    ref = re.search(r'\(reference "([^"]+)"\)', blk).group(1)
    uuids[ref] = re.search(r'\n    \(uuid ([0-9a-f-]+)\)', blk).group(1)

board = pcbnew.BOARD()
board.SetCopperLayerCount(2)

# vien mach
def seg(x1, y1, x2, y2, layer=pcbnew.Edge_Cuts, w=0.1):
    s = pcbnew.PCB_SHAPE(board); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(mm(OX + x1), mm(OY + y1))); s.SetEnd(pcbnew.VECTOR2I(mm(OX + x2), mm(OY + y2)))
    s.SetLayer(layer); s.SetWidth(mm(w)); board.Add(s)
for a, b in (((0, 0), (W, 0)), ((W, 0), (W, H)), ((W, H), (0, H)), ((0, H), (0, 0))):
    seg(*a, *b)

# net
nets = {}
def net(name):
    if name not in nets:
        n = pcbnew.NETINFO_ITEM(board, name); board.Add(n); nets[name] = n
    return nets[name]

for ref, (lib, name), value, fp_id, *_rest, netmap in design.PARTS:
    flib, fname = fp_id.split(":")
    fp = pcbnew.FootprintLoad(f"{FPDIR}/{flib}.pretty", fname)
    fp.SetReference(ref); fp.SetValue(value)
    fp.SetFPIDAsString(fp_id)
    if ref in uuids:
        fp.SetPath(pcbnew.KIID_PATH(f"/{uuids[ref]}"))
    x, y, rot = POS[ref]
    fp.SetPosition(pcbnew.VECTOR2I(mm(OX + x), mm(OY + y)))
    fp.SetOrientationDegrees(rot)
    if ref in design.DNP:
        fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_DNP if hasattr(pcbnew, "FP_DNP") else fp.GetAttributes())
    if not ref.startswith(("J", "H")):           # gia tri linh kien chi in o lop Fab cho do roi
        fp.Value().SetLayer(pcbnew.F_Fab)
    for pad in fp.Pads():
        n = netmap.get(pad.GetNumber())
        if n:
            pad.SetNet(net(n))
    board.Add(fp)

# vung dong GND mat duoi (chua do - nhan B de do sau khi di day)
z = pcbnew.ZONE(board)
z.SetLayer(pcbnew.B_Cu); z.SetNet(net("GND")); z.SetLocalClearance(mm(0.4))
z.SetMinThickness(mm(0.25))
ol = z.Outline(); ol.NewOutline()
for x, y in ((0.5, 0.5), (W - 0.5, 0.5), (W - 0.5, H - 0.5), (0.5, H - 0.5)):
    ol.Append(mm(OX + x), mm(OY + y))
board.Add(z)

# chu tren silk
t = pcbnew.PCB_TEXT(board); t.SetText("LiDAR2D carrier v0.1 - NUCLEO-F411RE"); t.SetLayer(pcbnew.F_SilkS)
t.SetPosition(pcbnew.VECTOR2I(mm(OX + 50), mm(OY + 60.5))); t.SetTextSize(pcbnew.VECTOR2I(mm(1.2), mm(1.2)))
board.Add(t)

board.Save("lidar_carrier.kicad_pcb")

# ---- file du an: net class nguon ----
pro = json.load(open("/usr/share/kicad/template/kicad.kicad_pro"))
def cls(name, tw, cl, via_d=0.8, via_dr=0.4):
    return {"name": name, "track_width": tw, "clearance": cl, "via_diameter": via_d, "via_drill": via_dr,
            "microvia_diameter": 0.3, "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
            "diff_pair_via_gap": 0.25, "wire_width": 6, "bus_width": 12, "line_style": 0,
            "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)"}
pro["net_settings"] = {
    "classes": [cls("Default", 0.3, 0.2), cls("Power", 1.0, 0.3, 1.0, 0.5), cls("Supply_5V", 0.6, 0.25)],
    "meta": {"version": 3}, "net_colors": None, "netclass_assignments": None,
    "netclass_patterns": [{"netclass": "Power", "pattern": p} for p in ("+24V*", "SW")] +
                         [{"netclass": "Supply_5V", "pattern": p} for p in ("+5V", "GND", "+3V3")]}
pro["meta"]["filename"] = "lidar_carrier.kicad_pro"
json.dump(pro, open("lidar_carrier.kicad_pro", "w"), indent=2)

# ---- kiem tra chong lan courtyard ----
board = pcbnew.LoadBoard("lidar_carrier.kicad_pcb")
fps = list(board.GetFootprints())
def cy(fp):
    poly = fp.GetCourtyard(pcbnew.F_CrtYd)
    if poly.OutlineCount() == 0:
        fp.BuildCourtyardCaches(); poly = fp.GetCourtyard(pcbnew.F_CrtYd)
    return poly
over = []
for i in range(len(fps)):
    for j in range(i + 1, len(fps)):
        a, b = fps[i], fps[j]
        if "A1" in (a.GetReference(), b.GetReference()):
            continue    # A1 la cum header + vien Arduino: kiem tra rieng bang pad
        pa, pb = cy(a), cy(b)
        if pa.OutlineCount() and pb.OutlineCount() and pa.Collide(pb.Outline(0)):
            over.append((a.GetReference(), b.GetReference()))
# pad cua linh kien khac khong duoc cham vung pad header Arduino
a1 = [f for f in fps if f.GetReference() == "A1"][0]
hdr = [p.GetBoundingBox() for p in a1.Pads()]
for f in fps:
    if f.GetReference() == "A1":
        continue
    fb = cy(f)
    for bb in hdr:
        bb2 = pcbnew.BOX2I(bb.GetOrigin(), bb.GetSize()); bb2.Inflate(mm(0.5))
        if fb.OutlineCount() and fb.Collide(pcbnew.SHAPE_RECT(bb2)):
            over.append((f.GetReference(), "A1-header")); break
# trong vien mach
out = []
for f in fps:
    bb = f.GetBoundingBox(False, False)
    x0, y0 = pcbnew.ToMM(bb.GetX()) - OX, pcbnew.ToMM(bb.GetY()) - OY
    x1, y1 = x0 + pcbnew.ToMM(bb.GetWidth()), y0 + pcbnew.ToMM(bb.GetHeight())
    if f.GetReference() != "A1" and (x0 < -0.01 or y0 < -0.01 or x1 > W + 0.01 or y1 > H + 0.01):
        out.append((f.GetReference(), round(x0, 1), round(y0, 1), round(x1, 1), round(y1, 1)))
print("chong lan:", over)
print("ra ngoai mach:", out)
print("footprints:", len(fps), "nets:", len(nets))
board.BuildConnectivity()
print("so ket noi chua di day:", board.GetConnectivity().GetUnconnectedCount(True))
