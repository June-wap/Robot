# GD01.3 – BÁO CÁO BIÊN DỊCH MỤC TIÊU ESP32 (ESP32 TARGET BUILD REPORT)

## 1. Cấu hình Toolchain & Board tham chiếu

- **Công cụ build:** `arduino-cli v1.6.0` (nguồn chính thức `github.com/arduino/arduino-cli/releases`, lưu ngoài repository trong thư mục scratch, không commit vào Git).
- **Board Package:** `esp32:esp32@4.0.0` (Espressif Systems official package index).
- **Trình biên dịch mục tiêu:** `xtensa-esp32-elf-g++` (`crosstool-NG esp-15.2.0_20251204`, GCC `15.2.0`).
- **FQBN tham chiếu:** `esp32:esp32:esp32` (**ESP32 Dev Module** — dùng làm cấu hình tham chiếu do chưa xác định phiên bản bo mạch vật lý cụ thể).
- **Lệnh biên dịch:** `arduino-cli compile --fqbn esp32:esp32:esp32 --warnings all esp32_robot`
- **Hành động phần cứng:** **KHÔNG** upload firmware, **KHÔNG** kích hoạt động cơ.

---

## 2. So sánh kết quả biên dịch ESP32 (Baseline `f584b67` vs. Post-GD01.3)

| Hạng mục | Baseline (`f584b67`) | Post-GD01.3 | Chênh lệch |
|---|---:|---:|---:|
| **Build Status** | **PASS** (exit 0) | **PASS** (exit 0) | Không đổi |
| **Flash Usage (Program Storage)** | `1,100,028 bytes` (`83%` / `1,310,720 B`) | `1,104,908 bytes` (`84%` / `1,310,720 B`) | `+4,880 bytes` (`+0.37%`) |
| **Static RAM Usage (Global DRAM)** | `62,244 bytes` (`18%` / `327,680 B`) | `62,252 bytes` (`18%` / `327,680 B`) | `+8 bytes` |
| **Remaining Local RAM** | `265,436 bytes` | `265,428 bytes` | `-8 bytes` |
| **Compiler Warnings (modules sửa trong GD01.3)** | `0` | `0` | `0` |
| **Compiler Warnings (pre-existing ngoài phạm vi)** | `3` (`-Wattributes` tại `Ultrasonic.h/.cpp`) | `3` (`-Wattributes` tại `Ultrasonic.h/.cpp`) | `0` mới |

---

## 3. Phân tích Stack trên Xtensa ESP32 (`xtensa-esp32-elf-g++ -Os -fstack-usage`)

Arduino-ESP32 cấp mặc định **`8,192 bytes`** stack cho `loopTask` (`CONFIG_ARDUINO_LOOP_STACK_SIZE`). Đo trực tiếp từng hàm bằng `-fstack-usage`:

| Hàm (Xtensa ESP32 ELF) | Baseline (`f584b67`) | Post-GD01.3 | Đánh giá an toàn |
|---|---:|---:|---|
| `CoveragePlanner::generateBestPlan` | **`9,440 bytes`** | **`128 bytes`** | **SỬA LỖI TRÀN STACK** (giảm `9,312 B` / `-98.6%`) |
| `CoveragePlanner::evaluateCandidateMetrics` | *(không tồn tại)* | `160 bytes` | Tĩnh, không mảng lớn |
| `CoveragePlanner::buildCandidate` | `96 bytes` | `96 bytes` | An toàn |
| `CoveragePlanner::verifyPlanIntegrity` | `32 bytes` | `80 bytes` | An toàn |
| `CoveragePlanner::computeMetricsFromSteps` | `80 bytes` | `80 bytes` | An toàn |
| `LaneTracker::onSessionFinished` (cao nhất trong `LaneTracker`) | *(<= 64 B)* | `64 bytes` | An toàn |
| **Peak call-chain `generateBestPlan()`** | **`~9,616 bytes` (> 8 KB!)** | **`~336 bytes`** | **Dư > 7.8 KB trên `loopTask` stack** |

- **Tính Reentrant:** `CoveragePlanner` không sử dụng biến `static` mutable và không cấp phát heap động → **hoàn toàn reentrant**.
- **Stack các hàm Web API:** `handleStatus` dùng buffer stack `768 B`, `handlePlan` dùng `640 B + 128 B = 768 B` (khi gọi trong `loop()` không chồng lên `generateBestPlan()`, hoàn toàn nằm trong giới hạn `8,192 B` của `loopTask`).

---

## 4. Ghi nhận Compiler Warnings & Giới hạn cấu hình

1. **3 cảnh báo pre-existing (ngoài phạm vi GD01.3):**
   - `esp32_robot/src/sensors/Ultrasonic.cpp:30..32`: `warning: ignoring attribute 'section (".iram1.4")' because it conflicts with previous 'section (".iram1.0")' [-Wattributes]`.
   - Nguyên nhân: `IRAM_ATTR` trong ESP-IDF dùng macro `__COUNTER__`, nên đặt `IRAM_ATTR` ở cả khai báo trong `Ultrasonic.h` lẫn định nghĩa trong `Ultrasonic.cpp` sinh ra 2 tên section khác nhau. Theo ràng buộc không chỉnh sửa `Ultrasonic` trong GD01.3, cảnh báo này được giữ nguyên và khuyến nghị dọn (chỉ đặt `IRAM_ATTR` ở phần định nghĩa hoặc thống nhất 1 nơi) ở giai đoạn Safety/Hardening.
2. **Giới hạn board tham chiếu:** Build thực hiện trên `esp32:esp32:esp32` (`ESP32 Dev Module`, Default 4MB with spiffs `1.2MB APP`). Nếu bo mạch thực tế dùng phân vùng Flash khác (hoặc ESP32-S3/C3), cần biên dịch lại đúng FQBN của board thật trước khi nạp.
