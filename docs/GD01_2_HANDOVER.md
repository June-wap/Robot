# GD01.2 – TÀI LIỆU BÀN GIAO (HANDOVER)

**Nhánh Git:** `refactor/gd01-modular-firmware` (tách từ `feature/gd01-adaptive-zigzag` @ `f2b6db1`)  
**Trạng thái phần mềm:** Hoàn tất tái cấu trúc modular cho firmware ESP32, sẵn sàng kiểm thử target build & triển khai Mobile App (GD02).  
**Trạng thái phần cứng:** Chưa kiểm thử trên robot thật (`HARDWARE VERIFIED: NO`).

---

## 1. Hướng dẫn Build và Kiểm thử

### 1.1. Build firmware ESP32 bằng Arduino IDE
1. Mở Arduino IDE (1.8.19+ hoặc 2.x) đã cài board **esp32 by Espressif Systems**.
2. Chọn **File → Open** và mở `SmartCleaningRobot/esp32_robot/esp32_robot.ino`.
   - Arduino IDE sẽ tự động quét và biên dịch toàn bộ các file `.cpp` trong `esp32_robot/src/**`.
   - Mọi `#include` trong `src/` dùng đường dẫn tương đối chuẩn nên không cần cấu hình thêm include path.
3. Chọn Board **ESP32 Dev Module** (giữ nguyên Partition Scheme mặc định).
4. Nhấn **Verify** để xác nhận kết quả biên dịch target Xtensa ESP32 và ghi nhận dung lượng Flash/DRAM.
5. (Tùy chọn) Ghi đè thông tin mạng cục bộ bằng cách sao chép `esp32_robot/src/config/Secrets.h.example` thành `esp32_robot/src/config/Secrets.h` (đã nằm trong `.gitignore`).

### 1.2. Build firmware ESP32 bằng Arduino CLI
```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 esp32_robot
```

### 1.3. Chạy Unit Tests & Benchmark trên PC
Các module `esp32_robot/src/planning/*` biên dịch độc lập với Arduino/ESP32:

```bash
# Chạy từ thư mục gốc SmartCleaningRobot (g++ / clang++ / zig c++)
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_coverage_planner.cpp esp32_robot/src/planning/CoveragePlanner.cpp -o test_coverage_planner
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_plan_validation.cpp  esp32_robot/src/planning/CoveragePlanner.cpp -o test_plan_validation
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_lane_tracker.cpp     esp32_robot/src/planning/CoveragePlanner.cpp esp32_robot/src/planning/LaneTracker.cpp -o test_lane_tracker
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_progress.cpp         esp32_robot/src/planning/CoveragePlanner.cpp esp32_robot/src/planning/LaneTracker.cpp -o test_progress
```

---

## 2. Chính sách Arduino Uno & Kế hoạch Modular hóa tương lai

- Trong GD01.2, `arduino_robot/arduino_robot.ino` **không bị chỉnh sửa** (SHA256 giữ nguyên `FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13`).
- Nếu cần modular hóa Arduino Uno ở giai đoạn sau (khi nâng cấp driver motor thay thế L293D), đề xuất tách nhẹ 3 file trong `arduino_robot/src/`: `MotorDriver` (HW-130/driver mới + watchdog), `BluetoothBridge` (ring buffer TX HC-05), `EspLinkProtocol` (parser `M,/W,/T,` và forward `C,`). Không triển khai khi chưa có yêu cầu trực tiếp.

---

## 3. Sẵn sàng cho GD02 (Mobile App & Safety Hardening)

Kiến trúc modular GD01.2 mở đường trực tiếp cho GD02:
1. **Phát triển Mobile App (Android Studio):**
   - Toàn bộ hợp đồng HTTP REST API đã cô lập gọn trong `esp32_robot/src/web/ApiHandlers.cpp` và `RobotWebServer.cpp`, tách biệt khỏi thuật toán điều hướng và cảm biến.
   - Các endpoint `/api/status`, `/api/plan`, `/api/scan.bin` (`720 B`), `/api/settings`, `/api/cmd`, `/api/unomode` sẵn sàng tiêu thụ không thay đổi schema.
2. **Sửa lỗi an toàn (Safety Hardening - giai đoạn riêng):**
   - Sửa spam UART khi lái tay gặp vật cản tại `src/core/RobotApp.cpp`.
   - Chuyển `noInterrupts()` sang `portMUX_TYPE` tại `src/sensors/Ultrasonic.cpp`.
   - Nâng cấp cơ chế chống khóa vị trí `POS_JUMP_MAX_MM` tại `src/localization/PoseEstimator.cpp`.
   - Bổ sung trạng thái `ST_ESTOP` / `ST_OTA`, timeout `PH_DRIVE`, kiểm tra cảm biến rơi/đứt dây sau khi hoàn tất nâng cấp phần cứng (`HW-BLOCKER-01..03`).
3. **Bảo mật trước khi push lên Git remote công khai:**
   - Thực hiện commit cấu hình riêng để gỡ mật khẩu WiFi/AP/OTA mặc định khỏi `NetworkConfig.h` (chỉ giữ cơ chế nạp từ `Secrets.h`), xoay vòng mật khẩu thực tế và rà soát lịch sử commit cũ (`a3ce321..HEAD`) nếu dự định mở công khai repository.
