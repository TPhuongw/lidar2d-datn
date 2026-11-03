W,H=1600,1000
o=[];a=o.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Arial, Helvetica, sans-serif">')
a(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
a('<text x="40" y="56" font-size="34" font-weight="bold" fill="#1b232c">Nối cảm biến khe (encoder) vào Arduino UNO qua breadboard</text>')
a('<text x="40" y="96" font-size="22" fill="#5b6670">Dây driver (D9→PUL+, D8→DIR+, PUL−/DIR−→GND) giữ nguyên, không vẽ lại</text>')
# UNO
a('<rect x="60" y="140" width="368" height="560" rx="18" fill="#1f6fb2"/>')
a('<text x="240" y="195" text-anchor="middle" font-size="32" font-weight="bold" fill="#fff">Arduino UNO</text>')
for n,y,c,note in [("D9",250,"#ffffff","→ PUL+ (giữ nguyên)"),("D8",300,"#ffffff","→ DIR+ (giữ nguyên)"),("D2",380,"#f2b705",""),("5V",560,"#e03131",""),("GND",630,"#222222","")]:
    a(f'<rect x="404" y="{y-12}" width="24" height="24" fill="#2b2b2b" stroke="{c}" stroke-width="3"/>')
    a(f'<text x="390" y="{y+9}" text-anchor="end" font-size="26" font-weight="bold" fill="#fff">{n}</text>')
    if note: a(f'<text x="96" y="{y+8}" font-size="19" fill="#cfe3f5">{note}</text>')
a('<text x="96" y="430" font-size="17" fill="#cfe3f5">D2: hàng DIGITAL, chân thứ 3</text>')
a('<text x="96" y="454" font-size="17" fill="#cfe3f5">tính từ góc (sau D0, D1)</text>')
a('<text x="96" y="610" font-size="17" fill="#cfe3f5">5V, GND: hàng POWER</text>')
# breadboard
BX,BY,BW,BH=640,220,520,480
a(f'<rect x="{BX}" y="{BY}" width="{BW}" height="{BH}" rx="12" fill="#f3efe4" stroke="#c9c2b0" stroke-width="2"/>')
a(f'<text x="{BX+BW/2}" y="{BY+BH+34}" text-anchor="middle" font-size="22" fill="#5b6670">Breadboard cắm dây</text>')
RP,RN=620,660
for y,col,lab in ((RP,"#e03131","+ (5V)"),(RN,"#1c64c4","− (GND)")):
    a(f'<line x1="{BX+16}" y1="{y-16}" x2="{BX+BW-16}" y2="{y-16}" stroke="{col}" stroke-width="3"/>')
    for x in range(BX+36,BX+BW-16,30): a(f'<rect x="{x-5}" y="{y-5}" width="10" height="10" fill="#9a9284"/>')
a(f'<text x="{BX-12}" y="{RP+8}" text-anchor="end" font-size="20" font-weight="bold" fill="#e03131"></text>')
cols=[690+42*i for i in range(11)]
rows=[300,340,380,420,460]
CA,CB=cols[2],cols[8]
a(f'<rect x="{CA-19}" y="280" width="38" height="200" rx="8" fill="#f2b705" fill-opacity="0.35" stroke="#c99700" stroke-width="2"/>')
a(f'<rect x="{CB-19}" y="280" width="38" height="200" rx="8" fill="#9c36b5" fill-opacity="0.22" stroke="#9c36b5" stroke-width="2"/>')
for x in cols:
    for y in rows: a(f'<rect x="{x-5}" y="{y-5}" width="10" height="10" fill="#9a9284"/>')
a(f'<text x="{CA}" y="268" text-anchor="middle" font-size="20" font-weight="bold" fill="#8a6a00"></text>')
def wire(d,c,w=7): a(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="{w}" stroke-linecap="round" stroke-linejoin="round"/>')
def dot(x,y,c): a(f'<circle cx="{x}" cy="{y}" r="9" fill="{c}" stroke="#222" stroke-width="2"/>')
def resistor(x,y1,y2,bands,label,lc,side=1):
    wire(f'M{x} {y1}V{y2}',"#8c8c8c",4); ym=(y1+y2)/2
    a(f'<rect x="{x-14}" y="{ym-30}" width="28" height="60" rx="10" fill="#e9d4a8" stroke="#8a7550" stroke-width="2"/>')
    for i,b in enumerate(bands): a(f'<rect x="{x-14}" y="{ym-22+i*12}" width="28" height="6" fill="{b}"/>')
    a(f'<text x="{x+24*side}" y="{ym+8}" text-anchor="{"start" if side>0 else "end"}" font-size="22" font-weight="bold" fill="{lc}">{label}</text>')
    dot(x,y1,"#8c8c8c"); dot(x,y2,"#8c8c8c")
# UNO -> rails (no crossings)
wire(f"M428 560H580V{RP}H{BX+36}","#e03131"); dot(BX+36,RP,"#e03131")
wire(f"M428 630H540V{RN}H{BX+66}","#222222"); dot(BX+66,RN,"#222222")
# D2 -> column A row c
wire(f"M428 380H{CA}","#f2b705"); dot(CA,380,"#f2b705")
a('<text x="470" y="368" font-size="20" font-weight="bold" fill="#8a6a00">dây D2</text>')
resistor(CA,460,RP,["#7a4a1c","#111111","#f08c00"],"10k","#8a6a00",1)
resistor(CB,460,RP,["#e03131","#e03131","#7a4a1c"],"220 Ω","#7a1f8f",-1)
a(f'<text x="{CA-52}" y="{rows[0]-30}" text-anchor="middle" font-size="20" font-weight="bold" fill="#8a6a00">Cột A</text>')
a(f'<text x="{CB+30}" y="{rows[0]-30}" text-anchor="middle" font-size="20" font-weight="bold" fill="#7a1f8f">Cột B</text>')
# sensor
SX=1290
a(f'<rect x="{SX}" y="200" width="270" height="440" rx="14" fill="#2b2b2b"/>')
a(f'<text x="{SX+135}" y="460" text-anchor="middle" font-size="26" font-weight="bold" fill="#fff">Cảm biến khe</text>')
a(f'<text x="{SX+135}" y="490" text-anchor="middle" font-size="18" fill="#bbbbbb">số = số chân</text>')
sp=[("3 XANH LÁ",250,"#2f9e44"),("1 TÍM",340,"#9c36b5"),("4 XANH DƯƠNG",560,"#1c64c4"),("2 TRẮNG",600,"#d0d0d0")]
for lab,y,c in sp:
    a(f'<rect x="{SX+18}" y="{y-13}" width="26" height="26" rx="4" fill="{c}"/>')
    a(f'<text x="{SX+54}" y="{y+8}" font-size="22" font-weight="bold" fill="#fff">{lab}</text>')
wire(f"M{SX} 250H{CA}V300","#2f9e44"); dot(CA,300,"#2f9e44")
wire(f"M{SX} 340H{CB}","#9c36b5"); dot(CB,340,"#9c36b5")
wire(f"M{SX} 560H1100V{RN}","#1c64c4"); dot(1100,RN,"#1c64c4")
wire(f"M{SX} 600H1130V{RN}","#c8c8c8"); dot(1130,RN,"#c8c8c8")
# explanation
EY=780
a(f'<rect x="40" y="{EY}" width="1520" height="196" rx="14" fill="#fff7d6" stroke="#f2b705" stroke-width="2"/>')
lines=[("Cột A (vàng) = nút D2:","#8a6a00"," 3 thứ cắm chung 1 cột dọc nên tự nối với nhau: dây từ D2 + dây XANH LÁ + 1 chân điện trở 10k."),
       ("Chân kia của 10k:","#8a6a00"," cắm hàng + (5V). Nhờ vậy D2 bình thường ở mức cao; tia sáng lọt qua khe đĩa kéo nó xuống thấp."),
       ("Cột B (tím) = LED:","#7a1f8f"," dây TÍM + 1 chân điện trở 220 Ω; chân kia của 220 Ω cắm hàng + (5V)."),
       ("TRẮNG, XANH DƯƠNG:","#1c64c4"," cắm thẳng hàng − (GND). Chân 5V và GND của UNO cắm vào hàng + và hàng −.")]
y=EY+42
for b,c,t in lines:
    a(f'<text x="66" y="{y}" font-size="23"><tspan font-weight="bold" fill="{c}">{b}</tspan><tspan fill="#1b232c">{t}</tspan></text>'); y+=44
a('</svg>')
open('enc_wiring.svg','w').write("\n".join(o))
