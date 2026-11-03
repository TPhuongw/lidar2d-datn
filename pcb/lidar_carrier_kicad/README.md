# Mạch mang LiDAR 2D cho NUCLEO-F411RE (bản nháp v0.1)

Shield cắm lên header Arduino của NUCLEO-F411RE, kích thước 100 × 80 mm, 2 lớp.
Đã có: sơ đồ nguyên lý, bố trí linh kiện, viền mạch, 4 lỗ M3, vùng GND mặt dưới (chưa đổ).
**Chưa đi dây**: phần này em tự làm theo hướng dẫn ở cuối file.

## File trong dự án

| File | Nội dung |
|---|---|
| `lidar_carrier.kicad_pro / .kicad_sch / .kicad_pcb` | Dự án KiCad 7 (mở được bằng KiCad 7 và 8) |
| `lidar_carrier_schematic.pdf` | Sơ đồ nguyên lý dạng PDF |
| `BOM_lidar_carrier.csv` | Danh sách linh kiện |
| `lidar_carrier.step` | Khối mạch 3D để đưa vào Fusion 360 (chưa có mô hình 3D linh kiện) |
| `design.py`, `gen_sch.py`, `gen_pcb.py` | Script sinh sơ đồ/PCB từ một nguồn dữ liệu duy nhất |

## Bảng phân chân Nucleo-F411RE

| Arduino | Chân STM32 | Chức năng | Ghi chú |
|---|---|---|---|
| D10 | PB6 | PUL (TIM4_CH1, phát xung) | Qua R2 → cổng Q1 |
| D7 | PA8 | DIR | Qua R3 → cổng Q2 |
| D6 | PB10 | ENA | Qua R4 → cổng Q3. Mức cao = tắt driver |
| D4 | PB5 | ALM (vào, EXTI5) | Kéo lên 3,3 V, lọc RC |
| D3 | PB3 | INDEX cảm biến khe (vào, EXTI3) | PB3 cũng là chân SWO, không dùng SWO |
| D8 | PA9 | USART1_TX → TF03 RX | Qua R11 100 Ω |
| D2 | PA10 | USART1_RX ← TF03 TX | Qua R10 100 Ω |
| A0 / A1 | PA0 / PA1 | ENC_A / ENC_B (TIM2 chế độ encoder) | Tùy chọn, chưa lắp |
| D12 | PA6 | Nút START (EXTI6) | |
| D5 / D15 | PB4 / PB8 | LED1 / LED2 | |
| D0 / D1 | PA3 / PA2 | **Không dùng** | ST-Link VCP gửi dữ liệu lên PC |
| D13 | PA5 | **Không dùng** | LED LD2 trên Nucleo |

Ánh xạ Arduino ↔ STM32 lấy theo user manual UM1724. **Cần đối chiếu lại bảng chân của bo NUCLEO-F411RE trong UM1724 trước khi đặt mạch.**

## Lựa chọn thiết kế chính

- **Nguồn:** 24 V → cầu chì tự phục hồi 1,5 A → diode SS34 chống ngược cực → TVS SMBJ28CA → LM2596S-5.0.
  Giá trị buck theo datasheet TI LM2596 (SNVS124): Cin 470 µF/50 V, Cout 220 µF/25 V, L 68 µH, diode Schottky 40 V.
- **Tải 5 V ước tính khoảng 0,3–0,4 A:** TF03 khoảng 150 mA, LED quang của driver tối đa khoảng 60 mA, phần còn lại nhỏ.
- **Nucleo cấp nguồn qua USB** (ST-Link), vì đằng nào cũng cắm USB để gửi dữ liệu.
  Muốn chạy không cần PC: nối J2 (E5V_OUT) tới CN7 chân 6 (E5V), chuyển jumper JP5 sang vị trí 2-3 (theo UM1724).
- **PUL/DIR/ENA dùng MOSFET AO3400A kéo xuống GND**, driver đặt jumper 5 V. Logic không đảo: chân MCU mức cao thì LED quang trong driver sáng.
  Không phụ thuộc chân chịu 5 V, và không bắt chân MCU hút dòng 7–20 mA.
- **TF03:** UART LVTTL 3,3 V, nối thẳng vào STM32 qua điện trở 100 Ω.

## Cần kiểm tra trước khi đặt mạch

1. Đối chiếu bảng chân với UM1724.
2. **Hướng cắm:** cạnh trái của footprint Arduino là phía cổng USB của Uno, tương ứng phía ST-Link/USB của Nucleo.
   Đặt Nucleo thật lên bản in 1:1 của PCB để kiểm tra không có linh kiện nào vướng cổng USB và các header morpho.
3. **Độ cao:** header morpho (cắm đực) trên Nucleo nhô lên phía dưới shield. Đo khe hở thực tế.
   Nếu không đủ, dùng header Arduino loại cao (stackable), hoặc khoét rãnh trên PCB tại vị trí 2 hàng morpho.
4. Chọn L1 có dòng bão hòa Isat ≥ 2 A. C6 (10 µF, 0805) chọn loại ≥ 10 V.
5. Chạy **ERC** trong KiCad (Inspect → Electrical Rules Checker). Bản này đã kiểm tra netlist khớp 100% với thiết kế,
   nhưng chưa chạy ERC vì KiCad 7 không có ERC qua dòng lệnh.

## Hướng dẫn đi dây

1. **Thiết lập:** File → Board Setup → Net Classes đã có sẵn: Power 1,0 mm (+24V, SW), Supply_5V 0,6 mm (+5V, GND, +3V3), Default 0,3 mm.
   Kiểm tra quy tắc của nhà sản xuất mạch (độ rộng, khoảng cách tối thiểu) ở mục Constraints.
2. **Đi khối buck trước, và quan trọng nhất:**
   - Vòng C1 → U1 chân 1 (VIN) → U1 chân 3 (GND) → C1 phải **ngắn nhất có thể**.
   - Nút SW (U1 chân 2, D3, L1) ngắn và rộng. Đây là nút nhiễu mạnh nhất, không cho đường tín hiệu nào chạy dưới nó.
   - Chân FB (U1 chân 4) lấy từ phía C3, đi xa L1.
   - GND của D3, C1, C3 nối thẳng xuống vùng GND mặt dưới bằng nhiều via.
3. **Nguồn 5 V và 3,3 V:** đi dạng hình sao từ C3/C4 tới J3 (các chân +), J4, J2. 3V3 lấy từ chân 3V3 của header Arduino.
4. **Tín hiệu driver:** từ header Arduino qua R2–R4 tới Q1–Q3, rồi D cực tới J3. Đường PUL ngắn, tránh đi song song dài với nút SW.
5. **Tín hiệu còn lại** (UART, INDEX, ALM, LED, nút): đi mặt trên. Hạn chế cắt vùng GND mặt dưới.
6. **Đổ đồng:** nhấn **B** để đổ vùng GND mặt dưới. Có thể thêm vùng GND mặt trên rồi khâu via giữa hai lớp.
7. **Kiểm tra:** Inspect → Design Rules Checker cho tới khi hết lỗi và hết đường chưa nối.
8. **Xuất file sản xuất:** File → Fabrication Outputs → Gerbers + Drill files.
