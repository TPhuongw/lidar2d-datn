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
| `settings.json` | Tự lưu khoảng 0,5 s sau mỗi lần đổi và khi đóng app: cổng COM, r0, xoay, vùng cấm... |
| `nen.npz` | Nền học gần nhất ở chế độ Trực tiếp |

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
1. Chỉnh **r0**, **Xoay**, **Đảo chiều** cho bản đồ khớp phòng thật, rồi tích **Khóa hiệu chỉnh** để tránh đổi nhầm. Lăn chuột trên ô số chỉ cuộn bảng, không đổi giá trị. Nếu đổi Xoay hay Đảo chiều sau khi đã vẽ vùng cấm, vùng tự xoay theo và vẫn ở đúng vị trí thật so với LiDAR.
2. **Vẽ vùng cấm**: bấm nút, bấm chuột trái lên bản đồ để thêm từng đỉnh, bấm nút lần nữa để kết thúc.
3. **Học nền** khi vùng cấm đang trống và bản đồ đã đứng yên (mặc định 20 vòng). Nền lưu theo góc thô nên không phụ thuộc Xoay. Học xong ở chế độ Trực tiếp, nền tự lưu vào `nen.npz`; phát lại không ghi đè file này. Mở app lần sau, bấm **Nạp nền đã lưu** nếu LiDAR chưa bị dời. App hỏi lại khi tốc độ quay lệch quá 5 % so với lúc học, và tự bù nếu r0 đã đổi. Chưa học nền thì app hiện CHƯA HỌC NỀN và không báo động, trừ khi tắt dùng nền.
4. Khi có vật trong vùng cấm liên tiếp *N* vòng (mặc định 3): ô trạng thái chuyển đỏ, có tiếng beep, ghi nhật ký và lưu ảnh vào `su_kien/`.

## Log mốc thời gian báo động
Mỗi lần chạy Trực tiếp, app ghi `alarm.csv` vào thư mục log; phát lại thì ghi `alarm_replay.csv`. Mỗi lần báo động là một dòng `BAO_DONG`, mỗi lần hết báo động là một dòng `HET`. Các mốc `t_*` dùng chung đồng hồ với `t_pc` trong `tf03_raw.csv`.

| Cột | Ý nghĩa |
|---|---|
| `t_first_hit` | Tia chạm vật ở vòng đầu của chuỗi vòng liên tiếp (ước lượng từ góc) |
| `t_scan_end` | Hết vòng gây báo động |
| `t_ready` | Mapper trả về vòng đó |
| `t_alarm` | Detector bật báo động |
| `t_snap` | Xong beep và lưu ảnh |
| `t_shown` | Đã vẽ lại ô trạng thái màu đỏ |
| `tre_hit_den_shown_s` | `t_shown − t_first_hit` (s) |
| `t_wall` | Giờ hệ thống, để đối chiếu với video |

- Log không đo được lúc vật thực sự vào vùng. Khoảng này cần đo bằng camera hoặc cảm biến ngoài.
- Khi phát lại, đồng hồ chỉ tiến theo từng nhịp giao diện nên `t_alarm`, `t_snap`, `t_shown` trùng nhau. Chỉ số của chế độ Trực tiếp mới dùng được để đánh giá các khâu này.

## Theo dõi đối tượng
Sau khi học nền, app nối các cụm khác nền giữa các vòng thành đối tượng, trên toàn phòng chứ không chỉ trong vùng cấm. Mỗi cụm dùng thời điểm tia quét tới nó, không dùng thời điểm đầu vòng. Đối tượng được xác nhận sau 3 lần thấy và bị bỏ sau 1,5 s không thấy.

- **Trên bản đồ:** quỹ đạo 30 s gần nhất, màu đỏ khi trong vùng cấm, xanh khi ngoài vùng, kèm số thứ tự và tốc độ hoặc thời gian đứng yên.
- **Log:** `tracks.csv` khi chạy Trực tiếp, `tracks_replay.csv` khi phát lại. Mỗi lần đo một dòng, cột `t` cùng đồng hồ với `t_pc`.
- Theo dõi không ảnh hưởng đến báo động. Đổi Xoay hoặc Đảo chiều thì đối tượng được đánh số lại.

Giới hạn:
- Tâm cụm là tâm mặt nhìn thấy, nên lệch về phía LiDAR so với tâm vật (khoảng 0,1 m với trụ bán kính 0,2 m trong log giả lập).
- Tốc độ lấy từ cửa sổ khoảng 2 s nên phản ứng chậm khi vật đổi tốc độ.
- Hai vật đi sát nhau dưới 0,35 m được tính là một.
- Tốc độ mới được kiểm tra với quỹ đạo giả lập, chưa đối chiếu với phép đo thật trên phần cứng. Góc vẫn tính theo thời gian, nên chưa dùng làm số liệu đánh giá.

## Tham số phát hiện
| Tham số | Ý nghĩa |
|---|---|
| Gần hơn nền (m) | Điểm phải gần hơn nền ít nhất bấy nhiêu mới tính là vật mới |
| Vật nhỏ nhất (m) | Bỏ cụm điểm nhỏ hơn (nhiễu, côn trùng) |
| Báo khi liên tiếp (vòng) | Chống báo động giả; tăng lên nếu hay báo nhầm |

## Giới hạn hiện tại
- Góc tính theo thời gian (PC + UNO), chưa phải STM32: dùng để **trình diễn**, không dùng làm số liệu đánh giá độ chính xác.
- Chưa có: đo bằng chuột, ghép nhiều LiDAR.
- Từng nút của mục **TF03** mới được kiểm tra đủ với TF03 giả lập, chưa kiểm tra đủ với cảm biến thật.
- Đã chạy chế độ Trực tiếp với phần cứng thật (UNO + TF03), xem `data/log_20261104_120106`. Bản đồ có thể lệch vài độ trong khoảng 30 s đầu; bản khớp mốc mới mới được phát lại trên log này, chưa chạy lại trên phần cứng.
