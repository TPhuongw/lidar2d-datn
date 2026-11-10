# data/ - log do quan trong

Thu muc `log_*` o noi khac KHONG len kho. Log nao can dung o may khac (phat lai trong app,
phan tich, bao cao) thi copy vao day roi commit + push.

Moi log nen kem 1 dong ghi chu o bang duoi: ngay, vi tri dat LiDAR, toc do quay, muc dich.
Kich thuoc tham khao: ~1,5 MB moi phut do o 1000 Hz. Chi giu log can thiet.

| Thu muc | Ngay | Noi dung |
|---|---|---|
| `log_20261104_120106` | 04/11/2026 | 2000 Hz, 1,5 roi 3 vong/s, 370 s, co nguoi di lai. Dung de do lech goc tung vong. Vi tri dat LiDAR: [CAN BO SUNG] |
| `log_20261104_141825` | 04/11/2026 | 1000 Hz, 3 vong/s, 616 s (~1845 vong), co dot PC nghen serial ~2 s o t=490..494 s. Dung de kiem chung chong lech goc ClockSync/Mapper. |
