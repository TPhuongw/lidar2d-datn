# Ve so do noi TF03 voi module USB-TTL FT232RL (SVG -> PNG bang playwright: node shot_tf03.js)
W, H = 1600, 900
o = []; a = o.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Arial, Helvetica, sans-serif"><rect width="{W}" height="{H}" fill="#fff"/>')
a('<text x="40" y="56" font-size="34" font-weight="bold" fill="#1b232c">Nối TF03 với module USB–TTL FT232RL</text>')
a('<text x="40" y="94" font-size="22" fill="#5b6670">TX và RX nối chéo · jumper FT232 đặt 3,3 V · TF03 lấy 5 V từ UNO · tất cả chung GND</text>')
def wire(d, c, w=7): a(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="{w}" stroke-linecap="round" stroke-linejoin="round"/>')
def pin(x, y, lab, side, fill="#c9a227", size=25):
    a(f'<rect x="{x-12}" y="{y-12}" width="24" height="24" fill="{fill}" stroke="#fff" stroke-width="2"/>')
    if side == "R": a(f'<text x="{x-24}" y="{y+9}" text-anchor="end" font-size="{size}" font-weight="bold" fill="#fff">{lab}</text>')
    else: a(f'<text x="{x+24}" y="{y+9}" font-size="{size}" font-weight="bold" fill="#fff">{lab}</text>')
# FT232
a('<rect x="60" y="170" width="380" height="560" rx="16" fill="#b8323a"/>')
a('<text x="250" y="222" text-anchor="middle" font-size="30" font-weight="bold" fill="#fff">FT232RL</text>')
a('<text x="250" y="252" text-anchor="middle" font-size="18" fill="#ffdfe0">(module USB–TTL) · cáp USB → PC</text>')
for k, y in (("GND", 300), ("RX", 460), ("TX", 540), ("VCC", 620)): pin(440, y, k, "R")
a('<text x="400" y="660" text-anchor="end" font-size="16" fill="#ffdfe0">VCC: để trống</text>')
a('<rect x="100" y="360" width="160" height="64" rx="8" fill="#7a1f26"/>')
a('<text x="180" y="387" text-anchor="middle" font-size="18" fill="#fff">Jumper</text><text x="180" y="412" text-anchor="middle" font-size="22" font-weight="bold" fill="#ffe08a">3,3 V</text>')
a('<text x="250" y="705" text-anchor="middle" font-size="16" fill="#ffdfe0">DTR, CTS: để trống</text>')
# TF03
a('<rect x="1140" y="170" width="400" height="560" rx="16" fill="#3a3f45"/>')
a('<text x="1340" y="222" text-anchor="middle" font-size="30" font-weight="bold" fill="#fff">TF03</text>')
a('<text x="1340" y="252" text-anchor="middle" font-size="18" fill="#c9c9c9">UART LVTTL 3,3 V · nguồn 5–24 V</text>')
for lab, y, c in (("ĐEN  GND", 300, "#222"), ("ĐỎ  VCC", 380, "#e03131"), ("NÂU  TX", 460, "#8b5a2b"),
                  ("XANH DƯƠNG  RX", 540, "#1c64c4"), ("TRẮNG / XANH LÁ  CAN", 620, "#9aa3ab")):
    pin(1140, y, lab, "L", fill=c, size=23)
a('<text x="1340" y="668" text-anchor="middle" font-size="16" fill="#c9c9c9">dây CAN: để trống, bọc đầu dây</text>')
# UNO nguon, dat giua 2 duong GND va RX
a('<rect x="640" y="330" width="300" height="100" rx="12" fill="#1f6fb2"/>')
a('<text x="760" y="372" text-anchor="middle" font-size="20" font-weight="bold" fill="#fff">Arduino UNO</text>')
a('<text x="760" y="398" text-anchor="middle" font-size="16" fill="#cfe3f5">(chỉ lấy nguồn 5 V)</text>')
pin(940, 380, "5V", "R", fill="#e03131", size=20)
a(f'<rect x="858" y="318" width="24" height="24" fill="#222" stroke="#fff" stroke-width="2"/><text x="870" y="358" text-anchor="middle" font-size="16" font-weight="bold" fill="#fff">GND</text>')
# day (khong cat nhau)
wire("M452 300H1128", "#222")                     # FT GND -> TF GND
wire("M870 318V300", "#222"); a('<circle cx="870" cy="300" r="9" fill="#222"/>')   # UNO GND -> GND chung
wire("M952 380H1128", "#e03131")                  # UNO 5V -> TF VCC
wire("M452 460H1128", "#2f9e44")                  # FT RX <- TF TX
wire("M452 540H1128", "#1c64c4")                  # FT TX -> TF RX
a('<text x="560" y="290" font-size="20" font-weight="bold" fill="#222">GND chung (cả 3 thiết bị)</text>')
a('<text x="1040" y="370" text-anchor="middle" font-size="20" font-weight="bold" fill="#e03131">5 V</text>')
a('<text x="790" y="450" text-anchor="middle" font-size="20" font-weight="bold" fill="#2f9e44">RX ← TX (dây nâu)</text>')
a('<text x="790" y="530" text-anchor="middle" font-size="20" font-weight="bold" fill="#1c64c4">TX → RX (dây xanh dương)</text>')
# ghi chu
a('<rect x="40" y="760" width="1520" height="116" rx="14" fill="#fff7d6" stroke="#f2b705" stroke-width="2"/>')
for i, (b, t) in enumerate((("Nối chéo:", " chân phát (TX) của bên này vào chân thu (RX) của bên kia."),
                            ("Jumper 3,3 V:", " tín hiệu từ FT232 vào RX của TF03 không vượt 3,3 V. Khi đó chân VCC của FT232 chỉ ra 3,3 V, không đủ cho TF03."),
                            ("Nguồn TF03:", " 5 V lấy từ UNO (hoặc nguồn 5–24 V bất kỳ), nhưng GND phải nối chung với FT232."))):
    a(f'<text x="64" y="{796+i*34}" font-size="21"><tspan font-weight="bold" fill="#8a6a00">{b}</tspan><tspan fill="#1b232c">{t}</tspan></text>')
a('</svg>')
open("tf03_ttl_wiring.svg", "w", encoding="utf-8").write("\n".join(o))
