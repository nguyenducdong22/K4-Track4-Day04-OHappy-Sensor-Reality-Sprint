# K4-Track4-Day04-OHappy-Sensor-Reality-Sprint

**Đề T6: Drone rangefinder fault detector.** Nhóm OHappy, thành viên trong [TEAMMATES.md](TEAMMATES.md).

Drone hạ cánh từ 10 m và dùng rangefinder (ToF/LiDAR một tia, 20 Hz) để biết độ cao so với mặt đất.
Nhóm tạo dữ liệu tổng hợp có 6 loại lỗi, mỗi loại 3 mức. Trên cùng một chuỗi dữ liệu, nhóm so sánh 5 cách xử lý: raw, median filter, MAD/Hampel z-score, Kalman + gate (ý tưởng từ PX4 EKF2), và Kalman + luật fallback S1/S2/S3.
Các metric gồm timeout rate, range variance, precision/recall phát hiện lỗi, RMSE độ cao và **landing risk score** (tính trong vòng kín, tức controller hạ cánh dùng chính độ cao đã ước lượng).

## Chạy lại

```bash
pip install -r requirements.txt
python src/rangefinder_benchmark.py      # ~30-60 s, deterministic (seed cố định)
```

Môi trường đã chạy: Python 3.13, numpy 2.5.3, pandas 3.0.5, matplotlib 3.11.2.

## Cấu trúc

| Đường dẫn | Nội dung |
|---|---|
| `src/rangefinder_benchmark.py` | Mô phỏng cảm biến, các bộ lọc/detector, controller hạ cánh, metric, plot |
| `out/benchmark_log.txt` | Bảng kết quả đầy đủ (log console) |
| `out/results.csv` | Kết quả gộp theo (lỗi, mức, mode) |
| `out/results_raw.csv` | Kết quả từng lần chạy (40 seed × cấu hình) |
| `out/raw_vs_filtered.png` | Range thô so với range đã lọc, cho từng loại lỗi |
| `out/recall_vs_level.png` | Recall phát hiện lỗi theo mức lỗi |
| `out/landing_risk.png` | Landing risk ở mức lỗi nặng |
| `out/failure_case_stuck.png` | Failure case chính: cảm biến bị đóng băng (stuck) |
| `out/limitation_terrain.png` | Giới hạn: bậc địa hình thật bị coi nhầm là lỗi |
| `reports/` | Báo cáo cá nhân (.md + .pdf) |

## Thiết kế benchmark

- **Baseline:** không cố ý tạo lỗi. Nhiễu nền σ = 2 cm + 0.5 %·h, dải đo 0.1–12 m.
- **Lỗi (bắt đầu ngẫu nhiên trong khoảng t = 3–7 s):**
  - noise σ 0.1 / 0.3 / 0.6 m
  - timeout 10 / 30 / 60 % (theo burst)
  - invalid (0 hoặc > max) 5 / 15 / 30 %
  - spike ±1–6 m với tần suất 2 / 5 / 10 %
  - stuck 1 s / 3 s / đến hết
  - bias +0.3 / +0.8 / +1.5 m
  - Thêm `terrain`: mặt đất **thật sự** nhô lên 0.5 / 1 / 2 m. Trường hợp này không phải lỗi, dùng để đo báo động nhầm.
- **Controller hạ cánh:** hạ 1.5 m/s; khi độ cao ước lượng < 2.5 m thì hạ 0.3 m/s.
- **Landing risk** = clip((v_chạm_đất − 0.4) / 0.8, 0, 1). Hard landing khi v ≥ 0.9 m/s.
- **Fallback S1/S2/S3** (nhóm tự định nghĩa, xét trong cửa sổ 1 s):
  - S2 khi ≥ 10 % mẫu bị gạt → hạ tối đa 0.5 m/s.
  - S3 khi ≥ 40 % mẫu bị gạt, hoặc phát hiện stuck, hoặc mất/invalid liên tục ≥ 0.5 s → bỏ rangefinder, dùng baro + IMU, hạ 0.3 m/s.

## Nguồn đã đọc

- PX4 docs: Using the ECL EKF, phần Range Finder (kinematic consistency check, `EKF2_RNG_K_GATE`, conditional range aiding): https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf.html (bản `main`, truy cập 2026-10-05)
- PX4 docs: Distance Sensors (Rangefinders): https://docs.px4.io/main/en/sensor/rangefinders.html
- PX4-Autopilot PR #19340 "Range finder kinematic consistency check": https://github.com/PX4/PX4-Autopilot/pull/19340

> Mọi con số trong `out/` là **benchmark tự chạy của nhóm trên dữ liệu tổng hợp**, không phải số liệu do PX4 công bố.
