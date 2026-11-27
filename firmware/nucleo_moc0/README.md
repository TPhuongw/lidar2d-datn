# nucleo_moc0: Nucleo-F411RE thay Arduino UNO

**Trạng thái: CODE THỬ NGHIỆM.** Đã biên dịch sạch bằng PlatformIO (framework stm32cube 1.28.3, gcc 7.2.1, bật `-Wall -Wextra` cho code của dự án), **chưa chạy trên phần cứng.**

Firmware làm đúng việc của sketch `index_flag_test.ino` trên UNO:
- Phát xung bước cho driver CL42C và đếm từng bước.
- Chốt thời điểm và số bước mỗi lần lá chắn đi qua module khe (mốc 0°).

Giao thức nối tiếp giữ nguyên như UNO (lệnh `r`, `d`, `z`, `?` và dòng `MOC 0 t=...`), nên **`lidar_app` chạy được mà không cần sửa**: ở ô cổng UNO, chọn cổng COM của ST-Link. Đây là bước 1–2 trong kế hoạch firmware. Bước 3 (TF03 nối vào STM32, góc tính theo số bước) chỉ làm sau khi bước này chạy đúng trên phần cứng.

| Thành phần | Chân |
|---|---|
| PUL | D10 = PB6, GPIO đảo mức trong ngắt TIM4 (2 MHz) |
| DIR | D7 = PA8 |
| Mốc 0° | D3 = PB3, TIM2_CH2 input capture (AF1), kéo lên nội, 1 µs |
| Cổng nối tiếp lên PC | USART2 (PA2/PA3) = ST-Link VCP, 115200 |
| Báo mốc | LD2 = PA5, đảo trạng thái mỗi lần qua mốc |
| Dừng | B1 = PC13, nhấn để giảm tốc về 0 |

Thông số lấy từ CubeMX DB:
- PB6 = TIM4_CH1 (AF2); PB3 = TIM2_CH2 (AF1); PA9/PA10 = USART1 (AF7, dành cho TF03 ở bước 3).
- Cấu hình mặc định của board: SYSCLK 84 MHz, APB1 42 MHz, xung timer APB1 84 MHz.
- PB3 mặc định là SWO của ST-Link: **không bật SWV/SWO trace** khi debug.

## Nạp và chạy (VS Code + PlatformIO)

Project dùng **PlatformIO**, framework `stm32cube` (thư viện HAL của ST), nên không cần CubeIDE hay CubeMX. Các file:
- `platformio.ini`: board `nucleo_f411re`, nạp qua ST-Link, Serial Monitor ở COM14, 115200.
- `src/main.c`: xung nhịp 84 MHz (HSI + PLL), USART2, LD2, B1.
- `src/lidar_fw.c` và `include/lidar_fw.h`: phát xung bước và bắt mốc 0°.

Các bước:
1. Trong VS Code: **File → Open Folder**, chọn thư mục `firmware/nucleo_moc0`. Mở đúng thư mục này, không mở thư mục gốc của repo.
2. Cắm cáp USB vào cổng ST-Link của Nucleo.
3. Dùng ba nút trên thanh trạng thái dưới đáy:
   - ✓ **Build**: biên dịch.
   - → **Upload**: nạp chương trình.
   - 🔌 **Serial Monitor**: mở COM14.
4. Trong Serial Monitor, bấm nút **RESET** đen trên board, phải thấy dòng chào:
   `nucleo_moc0 | lenh: r <vong/s>, r 0, d, z, ? | TIM clk 84000000 Hz | qua NPN`
5. Gõ lệnh (ví dụ `?`) rồi Enter để gửi.

Lệnh tương đương ở terminal: `pio run`, `pio run -t upload`, `pio device monitor`.

**Mỗi lần chỉ một chương trình được mở COM14.** Trước khi chạy `lidar_app` phải đóng Serial Monitor (biểu tượng thùng rác trên khung terminal).

## Nối dây

**Không nối thẳng chân STM32 vào PUL−/DIR− khi chưa kiểm tra.** Driver đang đặt jumper 5 V, trong khi STM32 ra 3,3 V. Tôi chưa tra được datasheet DS10314 (trang ST không tải được) để xác nhận PB6/PA8 chịu được 5 V (cột "I/O structure" = FT). Firmware mặc định dùng cách sau, không phụ thuộc điểm này:

```
5V (chân 5V của Nucleo) ── PUL+          5V ── DIR+
                           PUL− ── C            DIR− ── C
                                  NPN (S8050 / 2N2222 / BC547)
PB6 (D10) ── 1 kΩ ── B           PA8 (D7) ── 1 kΩ ── B
                     E ── GND                         E ── GND
GND Nucleo ── GND chung với cực âm nguồn tín hiệu
```
- Chân STM32 ở mức cao thì transistor dẫn, opto trong driver sáng. Firmware đã tính theo chiều này (`OUT_OPEN_DRAIN = 0`).
- ENA để hở như khi dùng UNO.
- Chỉ khi đã xác nhận PB6/PA8 là FT mới được nối kiểu anode chung (PUL+ → 5 V, PUL− → chân STM32) và đổi `OUT_OPEN_DRAIN` thành `1`.

**Module khe LM393:** VCC → **3V3** của Nucleo (không dùng 5 V), GND → GND, DO → **D3 (PB3)**.
- Nếu vẫn dùng cảm biến khe trần như hướng dẫn trong sketch UNO (LED cấp 5 V qua điện trở, transistor quang kéo xuống), chân ra chỉ cần nối vào PB3: PB3 đã có điện trở kéo lên nội 3,3 V.
- Firmware coi **cạnh lên** là mốc (`IDX_RISING = 1`), giống UNO. Nếu module LM393 cho cạnh ngược lại, đổi `IDX_RISING` thành `0`.

**Không cấp 24 V vào bất kỳ chân nào của Nucleo.** Nucleo lấy nguồn từ cáp USB.

## Kiểm tra

Thực hiện theo thứ tự, ghi lại kết quả.

1. **Chưa nối driver:**
   - Gõ `r 1`, đợi 2 s, gõ `?` vài lần: `buoc tuyet doi` phải tăng khoảng 3200 mỗi giây.
   - Gõ `r 0`.
2. **Nối driver và động cơ, chưa gắn module khe:**
   - Chạy `r 1`, `r 2`, `r 3`: động cơ phải tăng tốc êm, không kêu giật.
   - Driver không được nháy đỏ 7 lần (lỗi bám vị trí).
   - Thử `d` khi đã dừng: động cơ phải đổi chiều.
3. **Gắn module khe:** ở mỗi tốc độ 1, 2, 3, 4 vòng/s, gõ `z` rồi để chạy ít nhất 100 vòng.
   - Mỗi dòng phải có `buoc/vong 3200`.
   - `min..max` phải là 0..0.
   - Ghi lại độ lệch mốc ở từng tốc độ để so với số đo trên UNO: 9 / 17 / 24 bước ở 2 / 3 / 4 vòng/s so với 1 vòng/s.
4. **Với app:** mở `lidar_app`, chế độ Trực tiếp, cổng UNO = cổng COM ST-Link, cổng TF03 vẫn là FT232.
   - Bản đồ phải hiện như khi dùng UNO.
   - Gửi log phiên này để so với `log_20261110_101719`.

Gặp lỗi gì thì ghi lại nguyên văn: lỗi build (chép cả khung Problems), dòng in ra trên cổng COM, hoặc đèn trên driver.
