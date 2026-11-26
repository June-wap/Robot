# GD01.2 – BÁO CÁO KIỂM THỬ HỒI QUY (REGRESSION REPORT)

## 1. Môi trường kiểm thử và phạm vi xác minh

| Hạng mục | Công cụ / Trạng thái | Ghi chú |
|---|---|---|
| Trình biên dịch C++ Host | **Clang 22.1.8** (`zig c++`, `-std=c++17 -O2 -Wall -Wextra -Werror`) | Có sẵn trên máy |
| Unit Tests (Groups A – E) | **44 / 44 PASS** | So sánh trực tiếp trước & sau refactor |
| Algorithm Benchmark | **18 / 18 kịch bản V2 (+ 18 dòng V1) TRÙNG KHỚP 100%** | Không lấy `Time(us)` làm điều kiện |
| Multi-TU Compile & Link Check | **22 `.cpp` TUs + `esp32_robot.ino` PASS (`0 errors, 0 warnings`)** | Dùng mock Arduino/ESP32 headers |
| Standalone Header Isolation Check | **27 / 27 file `.h` PASS (`0 errors, 0 warnings`)** | Chứng minh không missing/circular include |
| Target Xtensa ESP32 Build (`arduino-cli` / ESP32 core) | **NOT RUN (Toolchain không có trên máy)** | Máy không cài `arduino-cli` hoặc `Arduino15` ESP32 core |
| Kiểm thử phần cứng trên robot thật | **NOT RUN / BLOCKED** | Chưa kết nối phần cứng; tồn tại `HW-BLOCKER-01..03` |

---

## 2. Kết quả Unit Tests (Baseline vs. Post-Refactor)

| Bộ test | Nhóm | Baseline (`f2b6db1`) | Post-Refactor (`refactor/gd01-modular-firmware`) | So sánh |
|---|---|---:|---:|---|
| `tests/test_coverage_planner.cpp` | A, B, C | 25 / 25 PASS | 25 / 25 PASS | **IDENTICAL** |
| `tests/test_plan_validation.cpp` | D | 7 / 7 PASS | 7 / 7 PASS | **IDENTICAL** |
| `tests/test_lane_tracker.cpp` | E1 | 7 / 7 PASS | 7 / 7 PASS | **IDENTICAL** |
| `tests/test_progress.cpp` | E2 | 5 / 5 PASS | 5 / 5 PASS | **IDENTICAL** |
| **Tổng cộng** | **Groups A – E** | **44 / 44 PASS** | **44 / 44 PASS** | **100% PASS** |

---

## 3. So sánh Algorithm Benchmark (18 kịch bản V2 & 18 dòng V1)

So sánh tự động toàn bộ 36 dòng bảng benchmark (Room A–F × Overlap 0% / 10% / 20% × V1 / V2), loại trừ cột `Time(us)`:

| Tiêu chí so sánh | Baseline | Post-Refactor | Sai lệch |
|---|---|---|---|
| Hướng lau (`Dir`) | 18/18 | 18/18 | **0** |
| Góc xuất phát (`Corner`) | 18/18 | 18/18 | **0** |
| Số làn (`Lanes`) | 18/18 | 18/18 | **0** |
| Khoảng cách làn (`Space(mm)`) | 18/18 | 18/18 | **0.0 mm** |
| Quãng đường lau / chuyển / tiếp cận / tổng | 18/18 | 18/18 | **0.00 m** |
| Số lần xoay (`Turns`) & Tổng góc (`TurnDeg`) | 18/18 | 18/18 | **0** |
| Chi phí kế hoạch (`Cost(s)`) | 18/18 | 18/18 | **0.0 s** |

---

## 4. Xác minh tính toàn vẹn mã nguồn & bảo toàn hành vi

| Kiểm tra | Kết quả | Bằng chứng |
|---|---|---|
| Arduino Uno SHA256 (`arduino_robot/arduino_robot.ino`) | **PASS** | Trước & sau: `FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13`; `git diff f2b6db1 -- arduino_robot/` rỗng |
| Planning GD01 files (`src/planning/*`) | **PASS** | 5/5 file (`LaneTypes.h`, `CoveragePlanner.*`, `LaneTracker.*`) trùng khớp 100% SHA256 (`git mv` 100% rename) |
| Web UI `INDEX_HTML` (`src/web/WebPage.cpp`) | **PASS** | Trùng khớp 100% SHA256 byte-for-byte: `71108fbec40866cf556d77c50209d23090ac69306a76c16fc3f6c33b9e723559` (8,295 bytes) |
| Duplicate symbols / Undefined references | **PASS** | Link đầy đủ 22 C++ TUs + `esp32_robot.ino` với `-Wall -Wextra -Werror` đạt 0 errors, 0 warnings |
| Missing includes / Circular includes | **PASS** | 27/27 header compile standalone 0 errors, 0 warnings |
| ISR linkage & `volatile` | **PASS** | `isrUs0/1/2` giữ nguyên `IRAM_ATTR`; `usRiseUs`, `usWidthUs`, `usHigh`, `usDone`, `newScan` giữ nguyên `volatile` |
| Runtime parity checks (host) | **PASS** | Đã chạy thực thi kiểm tra `setup()`, `loop()`, `MathUtils`, `rampAxis` (soft-start + 50 ms reverse dead-time), `Settings` strict parsers, `Lidar::calcCrc`, và cả 8 `ApiHandlers` |
| API Contract & HTTP status codes | **PASS (Static + Host call)** | Đủ 8 route (`/`, `/api/status`, `/api/scan`, `/api/scan.bin`, `/api/cmd`, `/api/settings`, `/api/plan`, `/api/unomode`), giữ nguyên JSON schema và mã `200/400/409` |
| Thứ tự `setup()` & `loop()` | **PASS (Code audit)** | `RobotApp.cpp` giữ chính xác từng bước theo đúng thứ tự của `esp32_robot.ino` gốc |

---

## 5. So sánh bộ nhớ (Struct / Static Memory)

Do chưa có `arduino-cli` / ESP32 toolchain trên máy để đo Flash/DRAM ELF thực tế cho chip Xtensa ESP32, bảng dưới ghi nhận chính xác kích thước cấu trúc đo trên host và đối chiếu cấp phát tĩnh:

| Cấu trúc / Thành phần | Baseline (GD01) | Post-Refactor (GD01.2) | Chênh lệch |
|---|---:|---:|---:|
| `sizeof(CoveragePlan)` | `4,696 bytes` | `4,696 bytes` | `0 bytes` |
| `sizeof(CleaningLane)` | `32 bytes` | `32 bytes` | `0 bytes` |
| `sizeof(Step)` | `16 bytes` | `16 bytes` | `0 bytes` |
| `sizeof(LaneTracker)` | `88 bytes` | `88 bytes` | `0 bytes` |
| `strlen(INDEX_HTML)` (`PROGMEM`) | `8,295 bytes` | `8,295 bytes` | `0 bytes` |
| `sizeof(RobotContext)` (host x86_64) | *(phân tán thành globals)* | `5,656 bytes` | Không có bản sao trùng lặp |
| Heap động mới thêm trong `planning/` hoặc `loop()` | `0` | `0` | `0` |
| **Target ESP32 Flash / DRAM (Xtensa ELF)** | *NOT MEASURED* | *NOT MEASURED* | Cần đo bằng `arduino-cli compile` trên máy có ESP32 core |

---

## 6. Danh sách kiểm tra chưa thực hiện (Not Run)

1. **Biên dịch target Xtensa ESP32 thực tế (`arduino-cli compile --fqbn esp32:esp32:esp32 esp32_robot`):** Chưa chạy do máy không cài `arduino-cli` và `esp32:esp32` board core.
2. **Đo chênh lệch Flash/RAM trên binary ESP32 thực tế:** Thực hiện ngay khi verify trên Arduino IDE / Arduino CLI.
3. **Kiểm thử phần cứng (Hardware-in-the-loop):** Chưa thực hiện (LiDAR LD14 thật, 3 siêu âm thật, UART1 với Arduino Uno thật, WiFi/mDNS/OTA thật, Web UI trên trình duyệt thực tế, lái motor trên đế nâng bánh).
