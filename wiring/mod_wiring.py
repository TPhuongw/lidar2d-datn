W,H=1600,900
o=[];a=o.append
a(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Arial, Helvetica, sans-serif"><rect width="{W}" height="{H}" fill="#fff"/>')
a('<text x="40" y="56" font-size="34" font-weight="bold" fill="#1b232c">Sơ đồ nối UNO với driver CL42C và module cảm biến khe LM393</text>')
a('<text x="40" y="94" font-size="22" fill="#5b6670">Không còn breadboard, không còn điện trở: module có sẵn mọi thứ</text>')
# UNO
a('<rect x="560" y="150" width="400" height="560" rx="18" fill="#1f6fb2"/>')
a('<text x="760" y="205" text-anchor="middle" font-size="32" font-weight="bold" fill="#fff">Arduino UNO</text>')
L={"D9":300,"D8":370,"GND":440}; R={"5V":300,"GND ":370,"D2":440}
for n,y in L.items():
    a(f'<rect x="548" y="{y-12}" width="24" height="24" fill="#2b2b2b" stroke="#fff" stroke-width="3"/><text x="590" y="{y+9}" font-size="26" font-weight="bold" fill="#fff">{n}</text>')
for n,y in R.items():
    a(f'<rect x="948" y="{y-12}" width="24" height="24" fill="#2b2b2b" stroke="#fff" stroke-width="3"/><text x="930" y="{y+9}" text-anchor="end" font-size="26" font-weight="bold" fill="#fff">{n.strip()}</text>')
a('<text x="760" y="560" text-anchor="middle" font-size="19" fill="#cfe3f5">D2: hàng DIGITAL, chân thứ 3 từ góc</text>')
a('<text x="760" y="588" text-anchor="middle" font-size="19" fill="#cfe3f5">5V, GND: hàng POWER</text>')
def wire(d,c,w=7): a(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="{w}" stroke-linecap="round" stroke-linejoin="round"/>')
# driver
a('<rect x="60" y="200" width="300" height="360" rx="14" fill="#3a3f45"/>')
a('<text x="210" y="248" text-anchor="middle" font-size="28" font-weight="bold" fill="#fff">Driver CL42C</text>')
a('<text x="210" y="278" text-anchor="middle" font-size="18" fill="#c9c9c9">(jumper tín hiệu ở 5V)</text>')
for n,y in (("PUL+",300),("DIR+",370),("PUL−",440),("DIR−",500)):
    a(f'<rect x="348" y="{y-12}" width="24" height="24" fill="#2b2b2b" stroke="#ddd" stroke-width="2"/><text x="330" y="{y+9}" text-anchor="end" font-size="24" font-weight="bold" fill="#fff">{n}</text>')
wire("M372 300H548","#f08c00"); wire("M372 370H548","#2f9e44")
wire("M372 440H548","#222"); wire("M372 500H470V440","#222")
a('<circle cx="470" cy="440" r="8" fill="#222"/>')
a('<text x="460" y="290" text-anchor="middle" font-size="18" fill="#5b6670">xung</text><text x="460" y="360" text-anchor="middle" font-size="18" fill="#5b6670">chiều</text>')
a('<text x="60" y="610" font-size="20" fill="#5b6670">Phần driver giữ nguyên như hiện tại</text>')
a('<text x="60" y="638" font-size="20" fill="#5b6670">(nguồn 24V, dây động cơ,</text><text x="60" y="664" font-size="20" fill="#5b6670">encoder động cơ)</text>')
# module
a('<rect x="1200" y="220" width="320" height="300" rx="14" fill="#1d4ea8"/>')
a('<rect x="1360" y="250" width="130" height="100" rx="6" fill="#111"/><rect x="1405" y="250" width="40" height="70" fill="#1d4ea8"/>')
a('<text x="1425" y="375" text-anchor="middle" font-size="18" fill="#cfd8f0">khe</text>')
a('<text x="1360" y="490" text-anchor="middle" font-size="24" font-weight="bold" fill="#fff">Module khe LM393</text>')
a('')
for n,y,c in (("VCC",300,"#e03131"),("GND",370,"#222"),("DO",440,"#f2b705")):
    a(f'<rect x="1188" y="{y-12}" width="24" height="24" fill="#c9a227" stroke="#fff" stroke-width="2"/><text x="1228" y="{y+9}" font-size="24" font-weight="bold" fill="#fff">{n}</text>')
wire("M972 300H1188","#e03131"); wire("M972 370H1188","#222"); wire("M972 440H1188","#f2b705")
a('<text x="1080" y="290" text-anchor="middle" font-size="18" fill="#5b6670">5V</text><text x="1080" y="360" text-anchor="middle" font-size="18" fill="#5b6670">GND</text><text x="1080" y="430" text-anchor="middle" font-size="18" fill="#5b6670">tín hiệu</text>')
a('<text x="1360" y="560" text-anchor="middle" font-size="19" fill="#5b6670">Nếu module có chân thứ 4 (AO): để trống</text>')
# notes
a('<rect x="40" y="740" width="1520" height="140" rx="14" fill="#fff7d6" stroke="#f2b705" stroke-width="2"/>')
for i,(b,t) in enumerate((("Đọc nhãn in trên module:"," thứ tự chân có thể khác ảnh này, nối theo CHỮ in trên mạch, không theo vị trí."),
                          ("Sau này chuyển sang STM32:"," VCC → 3,3V, DO → D3 (PB3). Không cấp 5V cho module khi dùng STM32."),
                          ("Code:"," encoder_only.ino / index_flag_test.ino dùng nguyên, D2 vẫn là chân tín hiệu."))):
    a(f'<text x="66" y="{784+i*40}" font-size="22"><tspan font-weight="bold" fill="#8a6a00">{b}</tspan><tspan fill="#1b232c">{t}</tspan></text>')
a('</svg>')
open('mod_wiring.svg','w').write("\n".join(o))
