# nucleo_lidar: TF03 nối vào STM32, góc theo số bước (bước 3)

**Trạng thái: CODE THỬ NGHIỆM.**
- Đã biên dịch sạch bằng PlatformIO (framework stm32cube 1.28.3, gcc 7.2.1, `-Wall -Wextra`).
- Phía PC đã thử bằng Nucleo giả lập: luồng sạch, mất gói, byte hỏng, gói bị cắt giữa hai lần đọc, chạy trong app rồi phát lại.
- **Đã chạy trên phần cứng (27/11):** app chế độ Nucleo, TF03 1000 Hz, 3 vòng/s, không bỏ sót mốc 0°, khoảng 1 khung lỗi mỗi 30 s. Lệnh `p`, `B` và lệnh `f` 5000 Hz mới thêm sau đó, chưa thử trên phần cứng.

Khác với `nucleo_moc0` (thay UNO, giữ nguyên giao thức chữ):
- TF03 nối thẳng vào Nucleo, không qua FT232 nữa.
- Mỗi khung đo được STM32 gắn **số bước động cơ kể từ mốc 0° gần nhất**, chốt ngay khi nhận byte đầu khung.
- PC tính **góc = số bước × 360 / 3200**, không còn ghép góc theo thời điểm PC nhận dữ liệu.
- Cả hệ chỉ còn một cổng COM (ST-Link, COM14), chạy ở **460800 baud**.

Muốn quay lại cách cũ (UNO/FT232) thì nạp lại `nucleo_moc0` và nối TF03 vào FT232 như trước.

## Nối dây

Giữ nguyên driver và module khe như với `nucleo_moc0`. **Rút TF03 khỏi FT232**, rồi nối:

| TF03 | Nucleo |
|---|---|
| Đỏ (VCC) | 5V |
| Đen (GND) | GND |
| **Nâu (TX)** | **D2** (PA10, USART1_RX) |
| Xanh dương (RX) | D8 (PA9, USART1_TX) |
| Trắng / xanh lá (CAN) | để hở |

- TF03 dùng UART 3,3 V nên nối thẳng vào STM32 được.
- TF03 ăn khoảng 150 mA ở 5 V (CLAUDE.md). Nguồn 5V từ cổng USB của Nucleo đủ cấp.
- Firmware đọc TF03 ở **460800 baud**, đúng cấu hình con TF03 đang dùng (460800 / 2000 Hz, kiểm tra trên máy ngày 27/11). Nếu TF03 đặt baud khác, gõ lệnh `u <baud>`. Sai baud thì lệnh `?` cho thấy `byte nhan` tăng, `loi khung UART` tăng, còn `byte 0x59` đứng ở 0.

## Nạp và kiểm tra

1. VS Code: **File → Open Folder** → `firmware/nucleo_lidar`. Bấm **✓** (Build), rồi **→** (Upload).
2. Bấm **🔌** (Serial Monitor), lần này ở **460800** (đã đặt sẵn trong `platformio.ini`). Bấm RESET trên board, phải thấy:
   `nucleo_lidar | lenh: ... | VCP 460800 | TF03 460800 | TIM clk 84000000 Hz`
3. **Kiểm tra TF03:** gõ `?` vài lần cách nhau 1 s.
   - `TF03 khung` phải tăng khoảng 2000 mỗi giây ở 2000 Hz (khoảng 1000 nếu TF03 đặt 1000 Hz).
   - `loi checksum` và `tran` phải gần như đứng yên.
   - Nếu `TF03 khung` đứng ở 0: kiểm tra dây nâu đã cắm vào D2 chưa, và TF03 có nguồn chưa.
4. **Kiểm tra mốc:** gõ `r 1`. Mỗi vòng có một dòng `MOC 0 ... | buoc/vong 3200 | 1000.x ms | ~2000 khung | ...`.
   - Số khung mỗi vòng xấp xỉ tần số TF03 chia tốc độ quay: ở 2000 Hz là 2000 khung ở 1 vòng/s, 800 khung ở 2,5 vòng/s.
5. Gõ `r 0`, rồi **đóng Serial Monitor** để nhả COM14.
6. Mở `lidar_app` và chọn **"Trực tiếp (Nucleo, TF03 nối vào STM32)"**. Ô **COM UNO / Nucleo** chọn COM14; ô COM TF03 bỏ qua. Bấm Bắt đầu.
   - App tự gửi lệnh `b 1` để bật luồng nhị phân, rồi đặt tốc độ quay.
   - Log ghi vào `stm_raw.csv` và `stm_idx.csv`. Phát lại thư mục log như bình thường, app tự nhận ra log của Nucleo.

## Lệnh

| Lệnh | Tác dụng |
|---|---|
| `r <vòng/s>`, `r 0` | Đặt tốc độ quay, có tăng/giảm tốc 1 vòng/s² |
| `d` | Đảo chiều quay (chỉ khi đã dừng) |
| `z` | Xóa thống kê độ lặp lại của mốc |
| `?` | Trạng thái: tốc độ, số bước, số khung TF03, lỗi, số gói bị bỏ |
| `b 1` / `b 0` | Bật / tắt luồng gói nhị phân. App tự gửi lệnh này, không cần gõ tay |
| `u <baud>` | Baud USART1 tới TF03 (mặc định 460800) |
| `t` | Tự kiểm tra USART1: nối tạm D8 sang D2, firmware phát 100 khung kiểu TF03 và đếm số khung nhận lại |
| `f <Hz>` | Đặt và lưu tần số TF03 qua D8 (100, 200, 500, 1000, 2000, 5000), chờ TF03 xác nhận lưu, rồi đo lại số khung/giây. Từ chối nếu vi phạm quy tắc baud ≥ 112 × Hz. Chỉ dùng khi app không chạy, vì firmware dừng khoảng 1,5 s |
| `p` | Dò baud TF03 (thử baud hiện tại trước, rồi 115200/230400/460800/921600), đặt USART1 theo baud tìm được, in `TF03 DO baud <b> khung/s <n>`. App tự gửi lệnh này mỗi lần bấm Bắt đầu |
| `B <baud>` | Đổi và lưu baud TF03 (115200, 230400, 460800, 921600). Có hiệu lực sau khi rút nguồn TF03 (dây đỏ hoặc USB Nucleo) rồi cắm lại. Từ chối nếu baud mới không đủ cho tần số TF03 đang chạy |

Trong app, ở chế độ **Trực tiếp (Nucleo)**, các nút Dò TF03 / Đặt tần số / Đặt baud gửi các lệnh `p`, `f`, `B` qua COM của Nucleo (bấm Dừng trước). Chưa thử trên phần cứng.

## Gói nhị phân (16 byte, số nguyên little-endian, CRC-8 đa thức 0x07 trên byte 0..14)

| Loại | Byte |
|---|---|
| Khung TF03 | `AA 55` · seq u8 · t_us u32 · step i16 · dist_cm u16 · strength u16 · rev u8 · flags u8 · crc |
| Mốc 0° | `AA 56` · seq u8 · t_us u32 · rev_steps i16 · n_khung u16 · tf_loi u16 · rev u8 · flags u8 · crc |

Ý nghĩa các trường:
- `step`: số bước kể từ mốc 0° gần nhất, chốt lúc nhận byte 0x59 đầu khung. Chốt ở đầu khung thay vì cuối khung để bớt thời gian truyền 9 byte (0,2 ms ở 460800, 0,7 ms ở 115200).
- `rev`: số thứ tự mốc, lấy mod 256.
- `flags`: bit0 = đã có mốc; bit1 = đang quay đều; bit2 = chiều 1; bit3 = số bước bị kẹp do quá lâu không có mốc.
- `rev_steps`: số bước của vòng vừa xong, phải bằng 3200. `n_khung`: số khung TF03 trong vòng đó.
- `tf_loi`: cộng dồn số khung sai checksum, số lần tràn bộ đệm UART và số khung bị bỏ vì hàng đợi đầy.
- `seq`: tăng 1 qua mọi gói, để PC đếm gói bị mất.

App (`core.StepMapper`) gom các khung theo `rev` thành từng vòng. Vòng bị bỏ khi: thiếu gói mốc ở đầu hoặc cuối vòng, `rev_steps` lệch 3200 quá 2 bước, hoặc mất quá 2 % gói trong vòng. Lý do bỏ được ghi vào `session.json`, mục `mapper_bo_vong`.

## Giới hạn đã biết

- **Trễ đo bên trong TF03** (từ lúc đo tới lúc xuất khung) chưa biết. Độ trễ này cố định nên làm lệch góc một lượng tỉ lệ với tốc độ quay; tạm bù bằng ô Xoay. Mốc 0° cũng dịch theo tốc độ: đã đo 9/17/24 bước ở 2/3/4 vòng/s trên UNO, cần đo lại trên Nucleo.
- ST-Link VCP chạy được ở 460800 baud (Serial Monitor đọc rõ chữ, 27/11). Ở 2000 Hz, luồng gói chiếm khoảng 70 % băng thông (16 byte × 2000 khung/s = 320 kbit/s). **Không được hạ VCP xuống 230400 khi TF03 chạy 2000 Hz**, vì không đủ băng thông.
- Mọi ngắt chỉ ghi biến và hàng đợi, không chặn. Phần phát lên PC dùng ngắt TXE với hàng đợi 4 KB; hàng đợi đầy thì bỏ nguyên gói và đếm vào `bo goi: hang phat`.
