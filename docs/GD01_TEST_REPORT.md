# GD01 – BÁO CÁO KIỂM THỬ VÀ BENCHMARK (TEST & BENCHMARK REPORT)

## 1. Môi trường kiểm thử và biên dịch

- **Trình biên dịch C++ host:** `Clang 22.1.8` (qua `ziglang c++`, flags: `-std=c++17 -O2 -Wall -Wextra -Werror`).
- **Biên dịch firmware mục tiêu ESP32 (`arduino-cli` / ESP32 core):** `NOT RUN` (máy hiện tại không cài `arduino-cli` hoặc Arduino ESP32 board package).
- **Kiểm tra cú pháp & liên kết host cho `esp32_robot.ino`:** `PASSED` (`0 errors, 0 warnings` với `-std=c++17 -O2 -Wall -Wextra -Werror` sử dụng mock header Arduino/ESP32; đã phát hiện và sửa 1 lỗi tràn buffer `snprintf` từ `320` lên `640` byte trong `handlePlan()`).
- **Kiểm thử phần cứng trên robot thật:** `BLOCKED / NOT RUN` (chưa có thiết bị kết nối và tồn tại blocker phần cứng quá tải IC L293D cần đo dòng trước khi vận hành tải thật).

---

## 2. Kết quả Unit Test (Groups A – E)

| Bộ test | Nhóm | Số test | Passed | Failed | Trạng thái |
|---|---|---:|---:|---:|---|
| `tests/test_coverage_planner.cpp` | A (Geometry), B (Dir/Corner), C (Overlap) | 25 | 25 | 0 | **PASS** |
| `tests/test_plan_validation.cpp` | D (Path Integrity & Validation) | 7 | 7 | 0 | **PASS** |
| `tests/test_lane_tracker.cpp` | E1 (Lane Status & Blocked Handling) | 7 | 7 | 0 | **PASS** |
| `tests/test_progress.cpp` | E2 (Progress & CleaningResult) | 5 | 5 | 0 | **PASS** |
| **Tổng Unit Tests tự động** | **Groups A – E** | **44** | **44** | **0** | **100% PASS** |

### Trạng thái Group F – Regression Tests
| Hạng mục | Trạng thái | Bằng chứng / Lý do |
|---|---|---|
| Bảo toàn `arduino_robot.ino` | **PASS** | SHA256 trùng khớp 100%, `git diff` trống |
| Bảo toàn cú pháp & liên kết `esp32_robot.ino` (LiDAR, US, Pose, UART, WiFi, OTA, WebServer, Settings) | **PASS (Host static/compile)** | Biên dịch sạch `0 errors, 0 warnings` bằng Clang C++17 |
| Biên dịch target Xtensa ESP32 (`arduino-cli`) | **NOT RUN** | Thiếu toolchain `arduino-cli` + `esp32:esp32` core trên máy |
| Kiểm thử phần cứng (LiDAR thật, siêu âm thật, động cơ, UART, WiFi/OTA thực tế) | **BLOCKED / NOT RUN** | Chưa kết nối robot; tồn tại `HW-BLOCKER-01` (quá tải L293D) |

---

## 3. Kết quả Algorithm Benchmark (Step 17)

- **Cấu hình chuẩn:** `MopWidth = 180 mm`, `Edge = 210 mm`, `V_nom = 194.1 mm/s`, `Ω_nom = 86.8 °/s`, `LaneChangePenalty = 1.5 s`.
- **Vị trí xuất phát mô phỏng:** Robot đứng gần góc `TOP_RIGHT` (trường hợp không trùng góc cố định `BOTTOM_LEFT` của thuật toán gốc V1).
- **Bộ nhớ đo thực tế:** `sizeof(CoveragePlan) = 4,696 bytes` | `sizeof(CleaningLane) = 32 bytes` | `sizeof(Step) = 16 bytes`.

### Bảng so sánh chi tiết (V1 cố định `Y + BOTTOM_LEFT` vs. V2 Adaptive `AUTO` cùng tỷ lệ overlap)

| Phòng | Ov% | Thuật toán | Hướng | Góc | Số làn | Bước (mm) | Lau (m) | Chuyển (m) | Tiếp cận (m) | Tổng (m) | Lần xoay | Tổng góc (°) | Chi phí (s) | Thời gian (µs) |
|---|---:|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Room A (2000×3000) | 0% | V1 | Y | BL | 10 | 175.6 | 25.80 | 1.58 | 3.88 | 31.26 | 21 | 1980 | 197.4 | 0.4 |
| Room A (2000×3000) | 0% | V2 | Y | TR | 10 | 175.6 | 25.80 | 1.58 | 0.28 | 27.66 | 21 | 1980 | **178.8** | 5.2 |
| Room B (3000×5000) | 0% | V1 | Y | BL | 16 | 172.0 | 73.28 | 2.58 | 6.68 | 82.54 | 33 | 3060 | 483.0 | 0.4 |
| Room B (3000×5000) | 0% | V2 | Y | TR | 16 | 172.0 | 73.28 | 2.58 | 0.48 | 76.34 | 33 | 3060 | **451.0** | 6.6 |
| Room C (5000×3000) | 0% | V1 | Y | BL | 27 | 176.2 | 69.66 | 4.58 | 6.78 | 81.02 | 54 | 4950 | 513.4 | 0.5 |
| Room C (5000×3000) | 0% | V2 | X | TR | 16 | 172.0 | 73.28 | 2.58 | 0.38 | 76.24 | 33 | 3060 | **450.5** | 6.2 |
| Room D (4000×4000) | 0% | V1 | Y | BL | 21 | 179.0 | 75.18 | 3.58 | 6.78 | 85.54 | 43 | 3960 | 516.3 | 0.4 |
| Room D (4000×4000) | 0% | V2 | X | TR | 21 | 179.0 | 75.18 | 3.58 | 0.38 | 79.14 | 43 | 3870 | **482.3** | 6.0 |
| Room E (2000×6000) | 0% | V1 | Y | BL | 10 | 175.6 | 55.80 | 1.58 | 6.78 | 64.16 | 21 | 1980 | 366.8 | 0.2 |
| Room E (2000×6000) | 0% | V2 | Y | TR | 10 | 175.6 | 55.80 | 1.58 | 0.38 | 57.76 | 21 | 1980 | **333.9** | 6.1 |
| Room F (6000×2000) | 0% | V1 | Y | BL | 32 | 180.0 | 50.56 | 5.58 | 6.88 | 63.02 | 64 | 5850 | 438.6 | 0.6 |
| Room F (6000×2000) | 0% | V2 | X | TR | 10 | 175.6 | 55.80 | 1.58 | 0.28 | 57.66 | 21 | 1980 | **333.4** | 6.0 |
| Room A (2000×3000) | 10% | V1 | Y | BL | 11 | 158.0 | 28.38 | 1.58 | 3.88 | 33.84 | 23 | 2160 | 214.2 | 0.3 |
| Room A (2000×3000) | 10% | V2 | Y | TR | 11 | 158.0 | 28.38 | 1.58 | 0.28 | 30.24 | 23 | 2160 | **195.7** | 4.9 |
| Room B (3000×5000) | 10% | V1 | Y | BL | 17 | 161.2 | 77.86 | 2.58 | 6.68 | 87.12 | 35 | 3240 | 510.1 | 0.4 |
| Room B (3000×5000) | 10% | V2 | Y | TR | 17 | 161.2 | 77.86 | 2.58 | 0.48 | 80.92 | 35 | 3240 | **478.2** | 6.8 |
| Room C (5000×3000) | 10% | V1 | Y | BL | 30 | 157.9 | 77.40 | 4.58 | 6.78 | 88.76 | 60 | 5490 | 564.0 | 0.5 |
| Room C (5000×3000) | 10% | V2 | X | TR | 17 | 161.2 | 77.86 | 2.58 | 0.38 | 80.82 | 35 | 3240 | **477.7** | 6.6 |
| Room D (4000×4000) | 10% | V1 | Y | BL | 24 | 155.7 | 85.92 | 3.58 | 6.78 | 96.28 | 49 | 4500 | 582.4 | 0.5 |
| Room D (4000×4000) | 10% | V2 | X | TR | 24 | 155.7 | 85.92 | 3.58 | 0.38 | 89.88 | 49 | 4410 | **548.3** | 6.6 |
| Room E (2000×6000) | 10% | V1 | Y | BL | 11 | 158.0 | 61.38 | 1.58 | 6.78 | 69.74 | 23 | 2160 | 399.2 | 0.2 |
| Room E (2000×6000) | 10% | V2 | Y | TR | 11 | 158.0 | 61.38 | 1.58 | 0.38 | 63.34 | 23 | 2160 | **366.2** | 6.7 |
| Room F (6000×2000) | 10% | V1 | Y | BL | 36 | 159.4 | 56.88 | 5.58 | 6.88 | 69.34 | 72 | 6570 | 485.4 | 0.7 |
| Room F (6000×2000) | 10% | V2 | X | TR | 11 | 158.0 | 61.38 | 1.58 | 0.28 | 63.24 | 23 | 2160 | **365.7** | 6.6 |
| Room A (2000×3000) | 20% | V1 | Y | BL | 12 | 143.6 | 30.96 | 1.58 | 3.88 | 36.42 | 25 | 2340 | 231.1 | 0.2 |
| Room A (2000×3000) | 20% | V2 | Y | TR | 12 | 143.6 | 30.96 | 1.58 | 0.28 | 32.82 | 25 | 2340 | **212.5** | 5.3 |
| Room B (3000×5000) | 20% | V1 | Y | BL | 19 | 143.3 | 87.02 | 2.58 | 6.68 | 96.28 | 39 | 3600 | 564.5 | 0.4 |
| Room B (3000×5000) | 20% | V2 | Y | TR | 19 | 143.3 | 87.02 | 2.58 | 0.48 | 90.08 | 39 | 3600 | **532.5** | 7.3 |
| Room C (5000×3000) | 20% | V1 | Y | BL | 33 | 143.1 | 85.14 | 4.58 | 6.78 | 96.50 | 66 | 6030 | 614.6 | 0.6 |
| Room C (5000×3000) | 20% | V2 | X | TR | 19 | 143.3 | 87.02 | 2.58 | 0.38 | 89.98 | 39 | 3600 | **532.0** | 6.9 |
| Room D (4000×4000) | 20% | V1 | Y | BL | 26 | 143.2 | 93.08 | 3.58 | 6.78 | 103.44 | 53 | 4860 | 626.4 | 0.4 |
| Room D (4000×4000) | 20% | V2 | X | TR | 26 | 143.2 | 93.08 | 3.58 | 0.38 | 97.04 | 53 | 4770 | **592.4** | 6.7 |
| Room E (2000×6000) | 20% | V1 | Y | BL | 12 | 143.6 | 66.96 | 1.58 | 6.78 | 75.32 | 25 | 2340 | 431.5 | 0.3 |
| Room E (2000×6000) | 20% | V2 | Y | TR | 12 | 143.6 | 66.96 | 1.58 | 0.38 | 68.92 | 25 | 2340 | **398.5** | 7.2 |
| Room F (6000×2000) | 20% | V1 | Y | BL | 40 | 143.1 | 63.20 | 5.58 | 6.88 | 75.66 | 80 | 7290 | 532.3 | 0.7 |
| Room F (6000×2000) | 20% | V2 | X | TR | 12 | 143.6 | 66.96 | 1.58 | 0.28 | 68.82 | 25 | 2340 | **398.0** | 7.4 |

### Nhận xét từ số liệu benchmark thực tế
- **Giảm chi phí tổng hợp trong 18/18 kịch bản:** Mức giảm từ **5.7% – 25.2%**; mạnh nhất ở phòng ngang dài (`Room F 6000×2000` ở 20% overlap: giảm từ `80` xuống `25` lần xoay — giảm **68.8% số lần xoay** và **25.2% chi phí**).
- **Minh bạch về quãng đường lau riêng (`Clean(m)`):** Ở `Room C` và `Room F`, quãng đường lau riêng của V2 cao hơn nhẹ so với V1 (do làm tròn `ceil()` theo biên độ khác nhau của cạnh ngắn và cạnh dài), nhưng tổng quãng đường (`Total(m)`) và tổng chi phí (`Cost(s)`) của V2 đều thấp hơn rõ rệt nhờ cắt mạnh quãng đường chuyển làn, quãng đường tiếp cận và số lần xoay.
- **Hiệu năng tính toán:** Đánh giá đầy đủ 8 candidate × 2 thứ tự tiếp cận chỉ tốn **4.9 – 7.4 µs** trên PC (dự kiến `< 0.5 ms` trên ESP32 240 MHz).
