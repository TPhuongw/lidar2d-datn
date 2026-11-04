# Ứng dụng giám sát LiDAR 2D (bản thử trên PC)

## Cài đặt
```
pip install PySide6 pyqtgraph pyserial numpy
python app.py
```

## Các file
| File | Nội dung |
|---|---|
| `app.py` | Giao diện (PySide6 + pyqtgraph) |
| `core.py` | Tách khung TF03, khớp đồng hồ, ghép góc theo mốc 0°, nguồn Trực tiếp / Phát lại |
| `detect.py` | Học nền, vùng cấm, gom cụm, chống báo động giả |
| `gen_fake_log.py` | Tạo thư mục `log_gia_lap/` để thử app khi không có phần cứng |
| `settings.json` | Tự tạo khi đóng app: cổng COM, r0, xoay, vùng cấm... |

## Thử không cần phần cứng
1. `python gen_fake_log.py` → tạo `log_gia_lap/` (phòng 8 × 6 m, một người đi vào vùng giữa phòng ở giây 6–24).
2. `python app.py` → chọn **Phát lại thư mục log** → chọn `log_gia_lap` → **Bắt đầu**.
3. Đặt **Xoay** = 3,2° cho tường thẳng (độ xoay do trễ cố định trong log giả lập).
4. **Vẽ vùng cấm** quanh góc dưới bên phải, bấm **Học nền** (4 vòng) ngay từ đầu, trước khi người đi vào.

## Chạy với phần cứng
1. UNO nạp **bản mới** `index_flag_test.ino` (dòng mốc có `t=`), đóng Arduino IDE.
2. Chọn **Trực tiếp**, chọn cổng TF03 (FT232) và UNO → **Bắt đầu**. Động cơ tự quay theo ô *Tốc độ quay*.
3. Dữ liệu thô được ghi vào `log_<ngày giờ>/` cạnh `app.py` → phát lại được bất kỳ lúc nào.

## Đổi baud và tần số TF03
Bấm **Dừng** trước, rồi dùng mục **TF03**. **Dò TF03** tìm baud hiện tại và đo tần số thật. Điều kiện: baud ≥ 112 × Hz.
- **Tăng tần số:** đổi baud trước nếu baud hiện tại chưa đủ. **Đặt baud** → rút nguồn TF03, cắm lại → **Dò TF03** → **Đặt tần số**.
- **Giảm baud:** giảm tần số trước, rồi mới **Đặt baud**.

App từ chối các tổ hợp sai điều kiện trên. Cấu hình được lưu vào TF03. Đã chạy thật 2000 Hz trong 370 s (đo được 1999,2 Hz); 5000 Hz chưa được kiểm chứng.

## Quy trình giám sát
1. Chỉnh **r0**, **Xoay**, **Đảo chiều** cho bản đồ khớp phòng thật. **Làm bước này trước khi vẽ vùng cấm**, vì vùng cấm vẽ theo tọa độ x, y.
2. **Vẽ vùng cấm**: bấm nút, bấm chuột trái lên bản đồ để thêm từng đỉnh, bấm nút lần nữa để kết thúc.
3. **Học nền** khi vùng cấm đang trống và bản đồ đã đứng yên (mặc định 20 vòng). Nền lưu theo góc thô nên không phụ thuộc r0 / Xoay. Nền không được lưu khi đóng app. Chưa học nền thì app hiện CHƯA HỌC NỀN và không báo động, trừ khi tắt dùng nền.
4. Khi có vật trong vùng cấm liên tiếp *N* vòng (mặc định 3): ô trạng thái chuyển đỏ, có tiếng beep, ghi nhật ký và lưu ảnh vào `su_kien/`.

## Tham số phát hiện
| Tham số | Ý nghĩa |
|---|---|
| Gần hơn nền (m) | Điểm phải gần hơn nền ít nhất bấy nhiêu mới tính là vật mới |
| Vật nhỏ nhất (m) | Bỏ cụm điểm nhỏ hơn (nhiễu, côn trùng) |
| Báo khi liên tiếp (vòng) | Chống báo động giả; tăng lên nếu hay báo nhầm |

## Giới hạn hiện tại
- Góc tính theo thời gian (PC + UNO), chưa phải STM32: dùng để **trình diễn**, không dùng làm số liệu đánh giá độ chính xác.
- Chưa có: đo bằng chuột, theo dõi vật thể, ghép nhiều LiDAR.
- Từng nút của mục **TF03** mới được kiểm tra đủ với TF03 giả lập, chưa kiểm tra đủ với cảm biến thật.
- Đã chạy chế độ Trực tiếp với phần cứng thật (UNO + TF03), xem `data/log_20261104_120106`. Bản đồ có thể lệch vài độ trong khoảng 30 s đầu; bản khớp mốc mới mới được phát lại trên log này, chưa chạy lại trên phần cứng.
