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
3. **Học nền** khi vùng cấm đang trống và bản đồ đã đứng yên (mặc định 20 vòng). Nền lưu theo góc thô nên không phụ thuộc Xoay. Học xong ở chế độ Trực tiếp, nền tự lưu vào `nen.npz`; phát lại không ghi đè file này. Mở app lần sau, bấm **Nạp nền đã lưu** nếu LiDAR chưa bị dời. App hỏi lại khi tốc độ quay lệch quá 5 % so với lúc học, và tự bù nếu r0 đã đổi.
   - **Tự nạp và căn nền khi bắt đầu** (ô cạnh nút, mặc định bật, chỉ ở chế độ Trực tiếp): bấm Bắt đầu mà chưa có nền thì app tự nạp `nen.npz`; nếu nền vẫn còn trong bộ nhớ từ lần chạy trước thì kiểm tra lại nền đó. Dù nạp tự động hay bấm nút, app chờ đủ 20 mốc 0° (vì lúc đầu đồng hồ mốc chưa ổn định, góc lệch thêm 1–2°), rồi đo 10 vòng ổn định xem nền lệch bao nhiêu độ. Trong lúc này app chưa phát hiện vật.
     - Lệch dưới 10°, sai số trung vị không quá 2,5 cm, và các vòng cho độ lệch khác nhau không quá 1,5°: nền được coi là đúng chỗ. Nếu lệch từ 1° trở lên thì app xoay nền theo rồi lưu lại. Dưới 1° thì để nguyên, để không tích lũy sai số của bộ ước lượng qua các lần nạp.
     - Ngoài các điều kiện trên: app bỏ nền, hiện CHƯA HỌC NỀN và báo cần học lại.
     - Kết quả ghi vào `session.json`, sự kiện `can_nen`.
     - Chỉ bù được lệch góc. LiDAR bị dời chỗ thì phải học lại.
     - **Khi đang chạy:** nếu 10 vòng ổn định liên tiếp đều lệch quá 5° và lệch đều nhau (phân tán không quá 1,5°), app coi là giá LiDAR bị xoay và áp dụng cùng quy tắc trên: lệch dưới 10° thì xoay nền theo, lệch nhiều hơn thì bỏ nền và báo học lại (sự kiện `can_nen`, `ly_do = lech_deu_khi_chay`). Lệch thoáng qua do dữ liệu về trễ thì không đều nên không kích hoạt.
     - Thử trên phần cứng (06/11): phiên 16:50 tự nạp nền và khớp (lệch 0,5°, sai số 1 cm). Phiên 16:53, khi xoay giá: phát lại bằng code mới thì đầu phiên đo được 8,25° và xoay nền theo; lần xoay giá thứ hai lệch thêm −12,5° nên app báo học lại.
     - Ngưỡng 2,5 cm lấy từ log 14:48: nền đúng chỗ cho sai số 1,0 cm và lệch −0,25 đến −0,5° (phân tán 0,15–0,45°) suốt 30 phút; nền và vòng quét của hai log 11:44 và 14:48 cho 4–5,5 cm.
     - Đã thử bằng cổng COM giả: xoay 6° thì khớp và xoay nền theo, lần sau đo lại ra 0,0°; xoay 25° và dời 0,3 m đều bị từ chối. Chưa chạy trên phần cứng. Chưa học nền thì app hiện CHƯA HỌC NỀN và không báo động, trừ khi tắt dùng nền.
4. Khi có vật trong vùng cấm liên tiếp *N* vòng (mặc định 3): ô trạng thái chuyển đỏ, có tiếng beep, ghi nhật ký và lưu ảnh vào `su_kien/`.

## Thông số phiên đo và an toàn dữ liệu
Mỗi lần bấm **Bắt đầu**, app ghi `session.json` vào thư mục log (phát lại: `session_replay.json`, ghi đè mỗi lần phát):
- `thong_so`: toàn bộ thông số lúc bắt đầu, gồm r0, Xoay, vùng cấm, các ngưỡng, baud, tốc độ quay.
- `su_kien`: mọi thay đổi trong lúc chạy (`doi`, kèm giá trị cũ và mới, thời gian trễ tối đa khoảng 0,5 s), và các lần `hoc_nen`, `nap_nen`.
- `nen_luc_bat_dau`: nền đang có khi bấm Bắt đầu, học lúc nào, ở tốc độ nào. Nền học hoặc nạp từ trước vẫn được dùng tiếp qua các lần Bắt đầu.
- `bat_dau`, `ket_thuc`, `nguon`, `phien_ban` (phiên bản code theo git). `ket_thuc` rỗng nghĩa là app không được đóng bình thường.

`tf03_raw.csv` được đẩy xuống ổ đĩa khoảng mỗi giây, `index.csv` sau mỗi mốc. App chết hoặc bị tắt ngang chỉ mất khoảng 1 s dữ liệu thô cuối. Mất điện đột ngột có thể mất nhiều hơn vì hệ điều hành còn giữ đệm.

## Đo diện tích
Bấm **Đo diện tích** khi đang chạy hoặc sau khi dừng. App lấy tối đa 20 vòng gần nhất đã ổn định tốc độ (cần ít nhất 10), không cần học nền, rồi:
1. Lấy trung vị theo vòng để bỏ người đi qua, trung vị theo góc (5 ô, 2,5°) để bỏ tia lẻ.
2. Bỏ hướng xa hơn *Đo: bỏ hướng xa hơn* (mặc định 8 m), nội suy từ hai bên. Nên đặt ngưỡng lớn hơn đường chéo phòng một chút.
3. Tính diện tích = ½ Σ r²·Δθ và chu vi đường bao, vẽ đường bao nét đứt lên bản đồ.

Kết quả ghi vào `dien_tich.csv`, `duong_bao_<giờ>.csv` trong thư mục log (phát lại: thêm `_replay`) và `session.json`.

Đây là **diện tích nhìn thấy được từ vị trí LiDAR, ở độ cao mặt quét**, không phải diện tích sàn: đồ đạc làm số nhỏ đi, phần khuất không đo được, người đứng yên hơn nửa số vòng được tính như vật cản. Hướng bị bỏ được nội suy nên chỗ cửa mở hoặc kính lớn có thể sai; app cảnh báo khi nội suy từ 10° trở lên. Chưa đối chiếu với phép đo thật.

## Diện tích vùng cấm
Dưới ô báo động, app hiện diện tích vùng cấm, tính từ các đỉnh đã vẽ (không phải phép đo bằng LiDAR). Khi đã có nền hoặc đã bấm **Đo diện tích**, app hiện thêm phần **giám sát được**: phần vùng nằm trước đường bao LiDAR nhìn thấy, tính bằng lưới 2 cm, ưu tiên lần đo diện tích gần nhất rồi đến nền. Dưới 90 % thì có cảnh báo, vì vật ở phần còn lại không phát hiện được. Vùng có cạnh tự cắt thì app yêu cầu vẽ lại. Hai số này ghi vào `session.json` (`vung_cam_luc_bat_dau` và các sự kiện `vung_cam`).

Phần giám sát được chỉ tính ở độ cao mặt quét, và phụ thuộc vào lúc lấy đường bao: người hoặc đồ đạc lúc đó sẽ làm số thay đổi. Vật thấp hơn mặt quét vẫn không thấy dù tỉ lệ là 100 %.

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
- **Trên góc bản đồ và mục Trạng thái:** thời gian và số đối tượng đang có, trong đó bao nhiêu trong vùng cấm. Khi phát lại, thời gian hiện dạng `đã phát / tổng`; khi chạy Trực tiếp là thời gian đã chạy.
- **Gộp đối tượng:** hai đối tượng cách nhau dưới *Gộp khi cách dưới* (mặc định 0,6 m) và lệch vận tốc dưới *Gộp khi lệch tốc độ dưới* (mặc định 0,25 m/s) liên tục 1,5 s thì tính là một, giữ số thứ tự cũ hơn và hiện `(gộp 2)`. Sau đó nếu một phần đi xa hơn 1,5 lần ngưỡng khoảng cách thì tách thành đối tượng mới. Đặt ngưỡng khoảng cách bằng 0 để tắt.
- **Chờ tốc độ ổn định:** app chỉ phát hiện, theo dõi và học nền khi 3 vòng liên tiếp có chu kỳ lệch nhau dưới 0,3 %, và chu kỳ lệch lúc học nền dưới 0,3 %. Ngoài điều kiện này, bản đồ vẫn hiện nhưng có cảnh báo và báo động đang bật không tự tắt theo vòng đó. Đổi tốc độ quay thì phải học lại nền. Máy tính quá tải cũng kích hoạt điều kiện này, vì góc được ghép theo thời điểm PC nhận dữ liệu: log `log_20261104_171459` có mốc UNO vẫn đều nhưng PC nhận trễ tới khoảng 2 s, chu kỳ tính ra lệch tới 2,3 %, sinh hàng loạt đối tượng giả. Khi đo không nên chạy tác vụ nặng trên cùng máy. Nền lưu từ bản cũ không có tốc độ thì bỏ qua điều kiện so với lúc học.
- **Mốc 0° về trễ, dồn cục:** log `log_20261106_114442` từ phút thứ 16 có mốc trên UNO vẫn đều 0,400 s, nhưng PC nhận theo cục 4 dòng mỗi khoảng 1,7 s, trễ tới 4 s. Dữ liệu TF03 cũng trễ theo đợt. Khớp đồng hồ theo thời điểm nhận làm bản đồ xoay hàng trăm độ. Cách app xử lý:
  - **Thời điểm từng khung TF03** lấy theo thời điểm PC nhận gói chứa khung đó; mỗi khung đứng sau nó trong gói thì lùi thêm một chu kỳ khung. Log `log_20261106_144837` (USB cắm thẳng) cho thấy PC nhận gói rất đều (3,5 ms, p99 4,7 ms), còn chính TF03 xuất khung không đều. Vì vậy cách cũ, coi thời điểm khung là đường thẳng theo số thứ tự khung, làm bản đồ thỉnh thoảng xoay 10–50° trong 1–2 vòng rồi trở lại. Nếu một gói về trễ hơn mô hình đường thẳng quá 15 ms (PC đang khựng) thì dùng mô hình. Mô hình khớp theo **biên dưới** (cạnh bao lồi dưới, cách ước lượng lệch đồng hồ của Moon và cộng sự, 1999), cửa sổ 400 gói đọc; không dùng cửa sổ dài hơn vì đồng hồ TF03 tự trôi không đều.
  - Tỷ lệ vòng lệch không quá 2° khi phát lại (mô phỏng, chưa chạy trên phần cứng): log 14:48 từ 98,2–98,7 % lên 99,9–100 % (lệch lớn nhất 10°, chỉ ở lúc khởi động); log 17:14 từ 97,5 % lên 99,1 %; log 11:44 ở đoạn bình thường từ 94 % lên 99,1 %, ở đoạn dồn cục từ 3 % lên 65 %.
  - Đồng hồ mốc 0°: tốc độ trôi chỉ cập nhật khi mốc đang về tốt, hồi quy trên các mốc đúng giờ (trễ dưới 40 ms) trong 1500 mốc gần nhất. Độ lệch lấy biên dưới của 300 mốc gần nhất. Nếu 10 s không có mốc đúng giờ thì giữ độ lệch cũ.
  - **Chặn phát hiện** khi quá 30 s không có mốc nào đúng giờ (ghi sự kiện `dong_bo_moc_kem` / `dong_bo_moc_lai` vào `session.json`), hoặc khi vòng quét bị xoay quá 5° so với nền đã học. Độ xoay ước lượng bằng trung vị |r − nền| nhỏ nhất khi xoay vòng quét ±60°. Số vòng bị bỏ vì xoay ghi vào `so_vong_bo_do_lech_goc` trong `session.json`. Độ xoay chỉ dùng để chặn, không dùng để sửa góc.
  - Bộ kiểm tra xoay (đo trên log 11:44, bản trước khi đổi cách tính thời điểm khung): chặn được 190/194 vòng lệch quá 5°; 4 vòng lọt đều lệch khoảng 6°. Ở đoạn bình thường chỉ 1/1758 vòng bị chặn nhầm.
  - **Nguyên nhân thật của các lần dồn cục ở phút 15–16** (log 11:44 và 16:09) là bảng lịch sử. Bản cũ dựng lại toàn bộ bảng sau mỗi vòng quét, với cột tự giãn theo nội dung (`ResizeToContents`). Khi có 121 lượt vào, mỗi lần dựng lại mất 3,5 s, app chiếm 93 % CPU, luồng đọc cổng COM bị bỏ đói, và mốc 0° trễ tới 43 s. Bản mới chỉ đổi những ô thay đổi, cập nhật tối đa 2 lần/s, và chỉ tính lại độ rộng cột khi số dòng đổi: với 229 lượt, mỗi lần cập nhật mất 14–17 ms, app chiếm 6–8 % CPU (đo bằng phát lại log 16:09).
  - Gặp cảnh báo này thì cắm UNO và FT232 thẳng vào cổng USB trên máy, không qua hub, rồi đóng các chương trình nặng. Chỉ khi chuyển sang STM32 (góc gắn ngay trên vi điều khiển) thì lỗi này mới hết hẳn.
- **Cụm bị tách:** khi một đối tượng biến mất mà ngay cạnh đó (dưới 0,6 m) có đối tượng đang thấy và một trong hai vừa sinh trong vòng 3 s, hai đối tượng được nối thành một, giữ số thứ tự cũ, bỏ lượt dư trong bảng. Hai đối tượng đã tồn tại lâu cạnh nhau thì vẫn tách riêng, để vật cúi xuống cạnh người khác vẫn được giữ.
- **Giữ vật biến mất:** LiDAR chỉ quét một mặt phẳng, nên vật thấp xuống dưới mặt quét (cúi, ngã) là không thấy nữa. Đối tượng tồn tại từ 3 s trở lên mà biến mất giữa vùng cấm (cách mép vùng từ 0,5 m, không đang đi ra, nằm trước nền từ 0,5 m) được giữ tối đa *Giữ vật biến mất* giây (mặc định 60 s, 0 = tắt). Trong lúc giữ, app báo `BÁO ĐỘNG: VẬT BIẾN MẤT TRONG VÙNG`, không báo vùng trống. Có vật xuất hiện lại trong bán kính 0,6 m thì nối lại, giữ số thứ tự và thời gian dừng. Bấm **Xác nhận vùng trống** để bỏ các vật đang giữ. `alarm.csv` có thêm dòng `MAT_TIN_HIEU`, `NOI_LAI`, `HET_GIU`.
- **Tiếng báo:** **tiếng kéo dài** khi đang thấy đối tượng, **tiếng bíp bíp ngắt quãng** khi không còn thấy ai nhưng còn vật biến mất đang được giữ trong vùng. Nếu vừa có vật đang thấy vừa có vật đang giữ thì phát tiếng kéo dài. Ô **Bíp liên tục** chọn điều kiện của tiếng kéo dài: *Có vật trong vùng* (mặc định; chưa có vùng cấm thì tính toàn phòng), *Có vật đang đi* (trên toàn phòng, bật từ 0,2 m/s, tắt dưới 0,1 m/s hoặc khi đứng yên), hoặc *Tắt*.
- **Điểm cuối:** khi đối tượng mất tín hiệu, bản đồ đánh dấu ✕ tại vị trí cuối cùng kèm tọa độ. Bấm một dòng trong bảng lịch sử để đánh dấu lại điểm cuối của lượt đó.
- **Bảng lịch sử (dưới bản đồ):** mỗi lượt vào vùng một dòng, mới nhất ở trên, hiển thị tối đa 200 dòng: số thứ tự, lúc xuất hiện, vào vùng, ra hoặc mất (giờ hệ thống khi chạy Trực tiếp, thời gian từ đầu khi phát lại), thời gian trong vùng, tốc độ lúc vào, trung bình và lớn nhất, độ rộng lớn nhất, điểm cuối, số lần mất tín hiệu, trạng thái. Ra rồi vào lại tính thêm một lượt. Chưa có vùng cấm thì ghi mọi đối tượng trong phòng. Mỗi lượt được ghi vào `luot_vao.csv` (phát lại: `luot_vao_replay.csv`) khi kết thúc. Bảng xóa khi bấm Bắt đầu hoặc **Xóa bảng**.
- Theo dõi không ảnh hưởng đến cách bật báo động; chỉ có giữ vật làm báo động kết thúc muộn hơn. Đổi Xoay hoặc Đảo chiều thì đối tượng được đánh số lại và các vật đang giữ bị bỏ.

Giới hạn:
- Tâm cụm là tâm mặt nhìn thấy, nên lệch về phía LiDAR so với tâm vật (khoảng 0,1 m với trụ bán kính 0,2 m trong log giả lập).
- Tốc độ lấy từ cửa sổ khoảng 2 s nên phản ứng chậm khi vật đổi tốc độ.
- Hai vật đi sát nhau dưới 0,35 m được tính là một ngay từ khâu gom cụm. Số đối tượng vì vậy không phải số người.
- Khi đối tượng đang ở trạng thái gộp, một vật lạ đi qua trong phạm vi 1,5 lần ngưỡng khoảng cách sẽ bị gom tạm vào.
- Số lượt vào phụ thuộc vào tracking: mất đối tượng rồi tạo lại sẽ đếm thêm lượt, hai người sát nhau chỉ đếm một lượt. Số này chưa được đối chiếu với đếm tay.
- Giữ vật chỉ là suy luận, LiDAR không thấy được vật khi đang giữ. Người rời đi mà tín hiệu mất giữa vùng (ví dụ qua cửa nằm trong vùng) cũng bị giữ, nên báo động kéo dài. Một người khác xuất hiện gần chỗ cũ sẽ bị nối nhầm vào đối tượng cũ. Các thời gian 60 s, 3 s và khoảng cách 0,5 m là giá trị tự chọn, chưa kiểm chứng.
- Gộp đối tượng mới được kiểm tra với quỹ đạo tự tạo. Hai log phần cứng hiện có không có lần gộp nào.
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
