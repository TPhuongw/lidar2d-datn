# CLAUDE.md: Đồ án LiDAR 2D từ LiDAR 1D (TF03)

Tài liệu ngữ cảnh cho Claude Code. Cập nhật lần cuối: 04/11/2026.
Người làm: Phương, sinh viên năm cuối Điện tử – Viễn thông, thực tập tại công ty LiDAR.
Luôn trả lời bằng tiếng Việt. Thuật ngữ chuyên ngành giữ tiếng Anh kèm giải thích. Không khen xã giao. Có chính kiến; khi có nhiều phương án thì nêu rõ nên chọn cái nào.

Ký hiệu trong file: **[ĐÃ CHỐT]** quyết định đã xác nhận · **[ĐO THẬT]** số liệu đo trên phần cứng · **[GIẢ ĐỊNH]** chưa kiểm chứng · **[ĐỀ XUẤT]** chưa chốt.

---

## 1. Mục tiêu đề tài
- Biến cảm biến đo khoảng cách 1D **Benewake TF03-180** thành **LiDAR quét 2D 360°** bằng **gương 45° quay**.
- Ứng dụng: **giám sát vật cản tại đường ngang đường sắt** (xe chết máy, người đi vào khi tàu sắp tới).
- Sản phẩm cuối cần: phần cứng chạy thật, firmware, phần mềm PC (bản đồ 2D + vùng cấm + báo động), số liệu đánh giá sai số, báo cáo tốt nghiệp.
- Mở rộng theo yêu cầu: **ghép 2–4 LiDAR đồng bộ** thành bản đồ chung. Cụ thể yêu cầu đồng bộ gì vẫn chờ xác nhận (xem mục 7).

## 2. Phần cứng

| Khối | Linh kiện | Trạng thái / thông số chính |
|---|---|---|
| Cảm biến | **Benewake TF03-180** ×2 (S/N U6600386020170, firmware 1.1.24) | 5–24 V, ~150 mA @5 V; UART LVTTL 3,3 V; CAN (không dùng). Dây: đỏ VCC, đen GND, **nâu TX**, **xanh dương RX**, trắng/xanh lá CAN. Cả hai con đã lưu cấu hình **115200 baud, 1000 Hz**, 0 lỗi checksum **[ĐO THẬT]** |
| Gương 45° | Cụm cốc gương tháo từ máy quét SICK | Đĩa encoder **180 vạch, có 1 vạch rộng** (người dùng đếm tay) |
| Động cơ | **Leadshine 42CME06** (NEMA17, 0,6 N·m, 2,5 A, có encoder A/B, không có kênh index) + driver **CL42C** | **[ĐÃ CHỐT]** (leader chọn). Driver: jumper tín hiệu đặt **5 V**; DIP 3200 xung/vòng (SW1 off, SW2 off, SW3 on, SW4 on); SW6 off = vòng kín; SW7 off = PUL/DIR. Lỗi nháy đỏ 7 lần = lỗi bám vị trí. Chạy vòng kín ổn định **[ĐO THẬT]** |
| Mốc 0° | **Lá chắn thiếc trên phần quay + module cảm biến khe LM393** (3 chân VCC/GND/DO) | **[ĐÃ CHỐT]** thay cho đọc vạch rộng trên đĩa 180 vạch (kém tin cậy do rung cạnh, mất tín hiệu). OPB tháo từ SICK chỉ còn dự phòng |
| Bộ điều khiển (tạm) | Arduino UNO | Phát xung bằng Timer1, đếm bước, bắt mốc 0° trên D2 |
| Bộ điều khiển (chính) | **STM32 Nucleo-F411RE** | **[ĐÃ CHỐT] nhưng CHƯA CÓ HÀNG** |
| USB–UART | **FT232RL** (TF03 → PC, COM12) | Thay CP2102 (CP2102 phát lệnh chập chờn) |
| Nguồn | 24 V (driver cần 24–50 V); buck 24→5 V trên PCB | **[ĐỀ XUẤT]** thay nguồn tổ ong bằng adapter 24 V 2–3 A |
| Khung | Nhôm 2020 + chi tiết in 3D (tháp động cơ, giá TF03, giá cảm biến khe) | Người dùng **không có máy in 3D**: hoặc thuê in ngoài (PETG, 4 tường, infill 30 %, file STL đã xoay nằm nghiêng), hoặc dựng bằng bộ **Thorlabs** (chờ danh sách đồ) |
| Thiết bị khác | Máy đo Leica (khoảng cách chuẩn), thẻ Thorlabs VRC4 (quan sát tia 905 nm), một LiDAR 2D thương mại kết nối LAN (chưa rõ model) | |

## 3. Kiến trúc hệ thống
```
TF03 ─UART→ STM32 (USART1+DMA) ─USB (ST-Link VCP)→ PC: bản đồ 2D, vùng cấm, báo động
Module khe ─→ STM32 (TIM2 input capture, mốc 0°)
STM32 (TIM4, PB6) ─PUL/DIR/ENA qua MOSFET→ CL42C → 42CME06 → gương SICK
```
- **Góc = (số bước kể từ lần qua mốc 0° gần nhất) × 360/3200** **[ĐÃ CHỐT]**. Không tính từ lúc khởi động, vì rotor trễ sau lệnh một lượng phụ thuộc tốc độ (mục 6).
- Hiện chưa có STM32 nên chạy **bản thử**: TF03 → FT232 → PC (COM12), UNO → PC (COM10). PC tự ghép góc theo thời gian.

### Phân chân Nucleo-F411RE (PCB v0.1) **[GIẢ ĐỊNH, cần đối chiếu UM1724]**
| Arduino | STM32 | Chức năng |
|---|---|---|
| D10 | PB6 | PUL (TIM4_CH1) |
| D7 | PA8 | DIR |
| D6 | PB10 | ENA |
| D4 | PB5 | ALM (vào) |
| D3 | PB3 | INDEX mốc 0° (TIM2_CH2) |
| D8 / D2 | PA9 / PA10 | USART1 TX→TF03 RX / RX←TF03 TX |
| A0, A1 | PA0, PA1 | encoder (không lắp). **[ĐỀ XUẤT]** đổi PA0/J6 thành cổng **SYNC** cho nhiều máy |
| D12 | PA6 | nút START |
| D5 / D15 | PB4 / PB8 | LED1 / LED2 |
| D0, D1 | PA3, PA2 | không dùng (USART2 = ST-Link VCP lên PC) |

Khi chạy trên breadboard với STM32: driver nối kiểu anode chung (PUL+/DIR+ → 5 V, PUL−/DIR− → chân STM32 open-drain, phải là chân FT, cần kiểm tra datasheet). Module khe cấp **3,3 V** khi dùng STM32.

## 4. Định dạng dữ liệu
**Không dùng point cloud chuẩn (.pcd/.las), không dùng ROS/rosbag.** Dữ liệu là quét 2D dạng (góc, khoảng cách), lưu bằng CSV.

- **Khung TF03 (UART, 9 byte):** `59 59 DistL DistH StrL StrH 00 00 CHK`. Dist tính bằng cm, byte thấp trước. CHK = byte thấp của tổng 8 byte đầu.
  - Lệnh: đặt tần số `5A 06 03 LL HH CHK` (1000 Hz = `5A 06 03 E8 03 4E`); đặt baud `5A 08 06 <4B LE> CHK` (115200 = `5A 08 06 00 C2 01 00 2B`); lưu `5A 04 11 6F` (trả về `5A 05 11 00 70`); khôi phục gốc `5A 04 10 6E`; phiên bản `5A 04 01 5F`. Đổi baud chỉ có hiệu lực sau khi lưu và khởi động lại.
  - Quy tắc băng thông: **baud ≥ 112 × tần số**. Vượt mức này thì TF03 quá tải và bỏ lệnh. Đây là nguyên nhân sự cố treo ở 460800 baud.
- **Dòng mốc từ UNO** (`index_flag_test.ino`): `MOC 0 t=<micros> #<n> | buoc/vong 3200 | ...`. `t=` là thời điểm mốc tính bằng micros trong ngắt.
- **Log** (thư mục `log_YYYYMMDD_HHMMSS/`):
  - `tf03_raw.csv`: `n,dist_cm,strength,t_pc_s` (n = số thứ tự khung, t_pc = thời điểm PC nhận gói)
  - `index.csv`: `t_uno_s,t_pc_s`
  - Định dạng cũ: `t_s,dist_cm,strength` và `t_s`. Script phát lại vẫn đọc được.
- **Một vòng quét xuất ra** (`vong_N.csv`): `angle_deg,r_m,strength,x_m,y_m`.
- **Gói STM32 → PC** **[ĐỀ XUẤT]**, 15 byte: `AA 55 | id(1) | t_us(4) | step(2, 0–3199) | dist_cm(2) | str(2) | flags(1) | chk(1)`, baud 460800 hoặc 921600.

## 5. Ngôn ngữ, công cụ, thư viện
- **Python 3**: `pyserial`, `numpy`, `matplotlib` (script cũ), **`PySide6` + `pyqtgraph`** (app chính). Không thêm Open3D, PCL hay ROS khi chưa thống nhất.
- **Arduino C++** (UNO). **STM32CubeIDE + HAL** cho firmware chính (chưa bắt đầu).
- **KiCad 7**, sinh tự động bằng Python: `kiutils` (sơ đồ nguyên lý), `pcbnew` (PCB). ERC phải chạy trong giao diện KiCad.
- **CadQuery** sinh STEP/STL; người dùng lắp ráp trên **Fusion 360**.
- Môi trường của người dùng: Windows, Arduino IDE, PowerShell. Lỗi treo GUI Benewake ở 1000 Hz là do GUI, không phải do cảm biến.

## 6. Kết quả đo đã có **[ĐO THẬT]**
- **Mốc 0° bằng lá chắn + LM393** (UNO, Timer1 đếm bước): ở 1–4 vòng/s, số bước mỗi vòng luôn **3200**, độ lặp lại **0 bước** (tốt hơn 0,11°). Mốc dịch theo tốc độ (so với 1 vòng/s): **9 / 17 / 24 bước** ở 2 / 3 / 4 vòng/s, khoảng 8,3 bước/(vòng/s), tương đương trễ khoảng 2,6 ms. Nguyên nhân chưa tách được: sai số bám của driver hoặc trễ phát hiện mốc. Số vòng đo mỗi tốc độ: **[CẦN BỔ SUNG]**.
- **Đọc đĩa 180 vạch bằng OPB:** mỗi vạch bị đếm khoảng 4 lần do rung cạnh (chùm rung ≤ khoảng 160 µs). Lọc khóa 300 µs thì đếm đúng 179–180 và 540 cạnh/s. Có lúc mất tín hiệu một đoạn mỗi vòng do gá lỏng hoặc mối hàn chập chờn.
- **Bản đồ 2D đầu tiên** (03/11, PC + UNO): tường thẳng, khoảng 990–1025 điểm/vòng ở 1 vòng/s; TF03 thực tế khoảng 1000–1025 Hz. Đối chiếu sơ đồ phòng: tủ phải khoảng 2,7 m và tường dưới khoảng 3,5 m khớp. Bản đồ cần **Xoay ≈ −40°, không đảo chiều** **[GIẢ ĐỊNH, đọc từ ảnh]**.
- Phòng thử nghiệm ước lượng khoảng **3,47 × 5,37 m** (suy từ bản vẽ, **chưa đo bằng thước**). LiDAR đặt trên bàn sát tường trái (gần máy tính), cột quay về tường, mặt quét khoảng 0,91 m. Mô phỏng: thấy khoảng 90,9 % diện tích.

## 7. Quyết định kỹ thuật
**[ĐÃ CHỐT]**
- TF03 + gương 45° SICK + 42CME06/CL42C vòng kín + STM32 Nucleo-F411RE + PCB dạng shield.
- Góc tính từ số bước kể từ mốc 0° gần nhất; chỉ lấy dữ liệu khi đã quay đều. Tốc độ vận hành **1–3 vòng/s** ở 1000 Hz.
- Mốc 0° bằng lá chắn + module LM393.
- Hiệu chuẩn không gian giữa nhiều LiDAR bằng **3 cột mốc chung** (phương án 2.b, `calib/calib_2lidar.py`). Cột Ø160–200 mm, không thẳng hàng.
- Không dùng Raspberry Pi hay FPGA thay STM32. Pi chỉ có thể thay PC làm máy chủ.
- Không ghi giá tiền trong báo cáo.

**[ĐỀ XUẤT, CHỜ CHỐT]**
- Đồng bộ thời gian nhiều máy: mỗi máy một STM32 + dây SYNC (PA0/J6, RS-485 nếu dây dài); tạm thời đồng bộ bằng phần mềm trên PC.
- Thu PCB về cỡ shield Arduino (~69 × 53 mm, buck MP2459/LMR16006, cần tra datasheet); đặt PCB trên đế sát chân cột; adapter 24 V.
- Giảm vùng mù do cột: thanh tròn mảnh đặt xa hơn (góc mù ≈ 2·atan((rộng cột + rộng chùm)/2 / khoảng cách); hiện tính ra khoảng 24,7° với chùm 15 mm **[GIẢ ĐỊNH]**); nhiều máy che vùng mù cho nhau.
- Chừa sẵn đầu nối SPI cho module Ethernet W5500.

**Câu hỏi đang chờ người dùng:** yêu cầu cụ thể của "kết nối 2–4 con đồng bộ"; khoảng cách giữa các máy; máy chủ (PC hay Pi); có gương cho máy thứ hai không; model của LiDAR thương mại dùng LAN; danh sách đồ Thorlabs; hạn nộp đồ án.

## 8. Cấu trúc thư mục (`tf03/`)
```
tf03_tool.py, tf03_baud_scan.py, tf03_cmd_probe.py, tf03_blast.py, tf03_recover.py   # công cụ TF03 (bản tiếng Anh trong en/)
motor_test/  motor_test_enc/  motor_pot/  motor_pot_enc/   # sketch UNO thử động cơ, biến trở, đếm encoder
encoder_only/            # chỉ đọc cảm biến khe (không chạy động cơ)
index_flag_test/         # UNO: Timer1 phát và đếm bước + mốc 0° bằng lá chắn, in "MOC 0 t=..."  ← sketch đang dùng
map2d/lidar_map_pc.py    # bản đồ 2D thử nghiệm (matplotlib), có khớp đồng hồ theo biên dưới
lidar_app/               # APP CHÍNH: app.py (giao diện), core.py (tách khung, ClockSync, Mapper, nguồn Live/Replay),
                         #            detect.py (học nền, vùng cấm, gom cụm, báo động), gen_fake_log.py (log giả lập)
calib/calib_2lidar.py    # hiệu chuẩn 2 LiDAR bằng cột mốc (Kabsch 2D), có --demo
cad/                     # CadQuery: nema17_tower_mount, lidar_frame (giá TF03/giá khe), build_assembly; in_ngoai/ (STL đã xoay)
pcb/                     # KiCad: design.py (nguồn dữ liệu duy nhất), gen_sch.py, gen_pcb.py, check_net.py, README (hướng dẫn đi dây)
wiring/                  # script vẽ sơ đồ nối dây (SVG → PNG bằng playwright)
```
Artifact liên quan: báo cáo tiến độ 17–31/10 (Docs), slide tiến độ, trang "Mô phỏng đặt LiDAR".
Ghi chép dự án (Claude Project "datn"): `claude/ket_qua_do_moc_0do.md`, `claude/dong_bo_nhieu_lidar.md`.

## 9. Việc đang làm dở
1. Chạy bản đồng bộ đồng hồ (`index_flag_test.ino` mới + `lidar_map_pc.py` hoặc `lidar_app` chế độ Trực tiếp). Kiểm tra bản đồ đứng yên ở 1 và 3 vòng/s. **Chế độ Trực tiếp của app chưa chạy với phần cứng.**
2. Đo phòng bằng thước hoặc Leica; chỉnh r0, Xoay, Đảo chiều; lập bảng sai số khoảng cách tới từng bức tường.
3. Thí nghiệm A: sai số và nhiễu TF03 theo khoảng cách (0,5–10 m, 1000 mẫu, so với Leica); ảnh hưởng của gương; ảnh hưởng của bề mặt.
4. Đo góc mù thật do cột (từ CSV một vòng quét) và thêm bộ lọc vùng mù vào app.
5. App đợt 3: đo bằng chuột, khớp đường thẳng cho tường, điều khiển TF03 trong app.
6. Phần cứng: mua Nucleo-F411RE, 2 module LM393, cột mốc; làm khung cứng (Thorlabs hoặc thuê in); chốt PCB (SYNC, đối chiếu UM1724, ERC, đi dây, đặt mạch).
7. Firmware STM32 (sau khi có Nucleo): TIM4 phát và đếm bước, TIM2 input capture mốc 0° (homing, kiểm tra 3200 ± vài bước mỗi vòng), USART1 + DMA đọc TF03, gói nhị phân lên PC.

## 10. Quy ước
- **Trung thực:** không tự bịa thông số, số đo, trích dẫn. Phân biệt rõ đã xác nhận, giả định, đề xuất. Không nói code đã chạy trên phần cứng nếu chưa chạy. Đầu file code thử nghiệm ghi `CODE THU NGHIEM` kèm trạng thái kiểm thử (mô phỏng, cú pháp, phần cứng).
- **Arduino:** mỗi sketch nằm trong **thư mục riêng cùng tên** (nhiều file `.ino` chung thư mục sẽ báo lỗi `redefinition`). Serial 115200, Newline. `tone()` không dùng trên 20 vòng/s; dùng Timer1 khi cần đếm bước. Khi Python mở cổng COM thì phải đóng Serial Monitor.
- **Đơn vị và quy ước:** SI (m, s); TF03 trả về cm. Góc tính bằng độ, 0° theo trục +x, tăng ngược chiều kim đồng hồ khi nhìn từ trên xuống. Công thức x = r·cos θ, y = r·sin θ; `dir` (±1) và `offset` (độ) để hiệu chỉnh. Một bước động cơ = 0,1125°.
- **Python:** tách phần xử lý (`core.py`, `detect.py`, không phụ thuộc giao diện) khỏi giao diện (`app.py`). Mọi nguồn dữ liệu đều có chế độ **ghi log và phát lại**. Kiểm tra logic bằng dữ liệu giả lập trước khi đưa người dùng chạy thật.
- **Comment trong code** viết tiếng Việt không dấu (an toàn cho Arduino IDE và terminal). **Chữ hiển thị trên giao diện** viết tiếng Việt có dấu.
- **Báo cáo:** văn phong kỹ thuật, không ghi giá tiền. Thiếu số liệu thì đánh dấu `[CẦN BỔ SUNG]`, không tự điền. Khi người dùng đã gửi ảnh chứa thông tin thì tự điền từ ảnh, không bắt người dùng điền lại.
- **An toàn phần cứng:** không cấp 24 V vào jack DC hay VIN của UNO. Module khe dùng 3,3 V khi đi với STM32. Đặt jumper FT232 ở 3,3 V khi nối TF03.
