# GD01.2 – KẾ HOẠCH REFACTOR KIẾN TRÚC MODULAR (REFACTOR PLAN)

**Dự án:** SmartCleaningRobot  
**Baseline:** Nhánh `feature/gd01-adaptive-zigzag`, commit `f2b6db1`  
**Nhánh thực hiện:** `refactor/gd01-modular-firmware`  
**Phạm vi:** Tái cấu trúc tổ chức mã nguồn firmware ESP32 từ monolithic sang modular. **Không thay đổi hành vi, không nâng cấp thuật toán, không sửa firmware Arduino Uno.**

---

## 1. Mục tiêu và Ràng buộc

### Mục tiêu
- Giảm file entry point `esp32_robot/esp32_robot.ino` từ `1,327` dòng xuống entry point tối giản (`robotSetup()` / `robotLoop()`).
- Phân chia mã nguồn thành 9 phân hệ chức năng trong `esp32_robot/src/` (`config`, `core`, `sensors`, `localization`, `planning`, `navigation`, `control`, `communication`, `web`) tương thích hoàn toàn Arduino IDE & Arduino CLI.
- Tập trung cấu hình phần cứng/robot/mạng và quản lý trạng thái dùng chung rõ ràng trong một thể hiện duy nhất `RobotContext gRobot`, đưa toàn bộ biến nội bộ về phạm vi `static` của module sở hữu.
- Tách HTML/CSS/JS (`INDEX_HTML` `PROGMEM`) và HTTP API handlers khỏi lõi điều hướng để thuận tiện cho Mobile App (GD02).

### Ràng buộc bảo toàn nghiêm ngặt
- Giữ nguyên 100%: sơ đồ chân GPIO, baud rate (`230400` / `38400`), giao thức UART (`M,`, `W,`, `T,`, `C,`), thuật toán LiDAR + CRC, thuật toán Siêu âm + ISR `IRAM_ATTR` + `volatile`, PCA WallDetection, PoseEstimator, Calibration, CoveragePlanner V2, LaneTracker, Navigator, MotionController (PWM/Ramp/dead-time), HTTP API contract & status codes, Web UI `INDEX_HTML`, WiFi STA/AP, mDNS, OTA, NVS keys, thứ tự thực thi từng câu lệnh trong `setup()` và `loop()`.
- Giữ nguyên tuyệt đối `arduino_robot/arduino_robot.ino` (xác minh bằng SHA256 `FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13`).
- Không tự động nạp firmware hoặc kích hoạt động cơ.

---

## 2. Quy trình Refactor tăng dần (Incremental Steps)

Mỗi bước đều qua kiểm tra biên dịch multi-TU (`-std=c++17 -O2 -Wall -Wextra -Werror`), kiểm tra SHA256 Arduino Uno, chạy unit tests liên quan và tạo commit riêng trên nhánh `refactor/gd01-modular-firmware`:

| Bước | Nhóm module | Thay đổi chính | Commit |
|---:|---|---|---|
| 0 | Baseline | Ghi nhận `git status`, SHA256 Arduino, chạy 44 unit tests + 18 benchmark + host compile check, tạo nhánh `refactor/gd01-modular-firmware` | `f2b6db1` (base) |
| 1 | Config & Core leaf | Tách `HardwareConfig.h`, `RobotConfig.h`, `NetworkConfig.h`, `Secrets.h.example`, `RobotTypes.h`, `MathUtils.h/.cpp` | `072237c` |
| 2 | WebPage | Tách `INDEX_HTML` (`PROGMEM`) sang `src/web/WebPage.h/.cpp` | `32ae505` |
| 3 | Planning | `git mv` 5 file GD01 (`LaneTypes.h`, `CoveragePlanner.*`, `LaneTracker.*`) vào `src/planning/`; cập nhật include trong `tests/*.cpp` | `20a5361` |
| 4 | Shared State & Settings | Khởi tạo `src/core/RobotContext.h/.cpp` (1 thể hiện duy nhất `gRobot`) và tách `src/config/Settings.h/.cpp` (NVS `Preferences` + strict parsers) | `ac91490` |
| 5 | LiDAR | Tách `src/sensors/Lidar.h/.cpp` (CRC table, LD14 parser, scan rollover) | `f3ac1b6` |
| 6 | Ultrasonic & Obstacles | Tách `src/sensors/Ultrasonic.h/.cpp` (ISR `IRAM_ATTR`, `volatile` state, `median3`) và `src/sensors/ObstacleDetector.h/.cpp` | `2b32ad6` |
| 7 | Localization | Tách `WallDetection.h/.cpp`, `PoseEstimator.h/.cpp`, `Calibration.h/.cpp` | `7d6a242` |
| 8 | Control | Tách `MotionController.h/.cpp` và `CommandHandler.h/.cpp` | `87afa8d` |
| 9 | Navigation | Tách `CleaningSession.h/.cpp` và `Navigator.h/.cpp`; bỏ các `extern` tạm | `1460f67` |
| 10 | Communication | Tách `ArduinoLink.h/.cpp`, `WiFiManager.h/.cpp`, `OtaManager.h/.cpp` | `1bde0ee` |
| 11 | Web Server & API | Tách `RobotWebServer.h/.cpp` và `ApiHandlers.h/.cpp` | `acf55c1` |
| 12 | App Orchestrator | Tách `src/core/RobotApp.h/.cpp`; đơn giản hóa `esp32_robot.ino` còn 27 dòng | `179545e` |
| 13 | Verification & Docs | Chạy hồi quy 44 unit tests + 18 benchmarks + 27 standalone header checks + runtime parity + tài liệu GD01.2 & GitHub readiness | Final docs commit |

---

## 3. Thay đổi tối thiểu bắt buộc để biên dịch C++ đa đơn vị (.cpp)

Do Arduino IDE chỉ tự sinh function prototype cho file `.ino` chứ **không** tự sinh cho các file `.cpp` trong `src/`, các thay đổi kỹ thuật tối thiểu bắt buộc gồm:
1. Khai báo đầy đủ prototype trong từng file `.h` kèm include guard `#ifndef ... #define ... #endif`.
2. Bổ sung `#include <Arduino.h>`, `<math.h>`, `<stdint.h>`, `<stdlib.h>`, `<stdio.h>`, `<string.h>` trực tiếp tại các `.h`/`.cpp` sử dụng API tương ứng.
3. Chuyển `const int US_TRIG[3]` / `US_ECHO[3]` thành `constexpr int US_TRIG[3]` / `US_ECHO[3]` trong `HardwareConfig.h` để tránh duplicate symbol giữa các đơn vị biên dịch và đảm bảo truy cập an toàn trong ISR `IRAM_ATTR`.
4. Thêm các hàm biên giới khởi tạo/đồng bộ nhỏ (`lidarBeginSerial`, `lidarInit`, `ultrasonicInit`, `poseInitTimer`, `resetCalibration`, `resetNavigationState`, `arduinoLinkInit`, `arduinoLinkSendImmediateStop`, `arduinoLinkSendOtaStop`, `saveSettings`, `webServerInit`, `webServerHandleClient`, `otaInit`, `otaHandle`) mà **không thay đổi bất kỳ câu lệnh hay thứ tự thực thi phần cứng nào**.
5. Hỗ trợ tùy chọn `#if __has_include("Secrets.h")` trong `NetworkConfig.h` kèm `Secrets.h.example` (giữ nguyên giá trị mặc định hiện hành của robot để không phá vỡ hành vi build/chạy hiện tại).

---

## 4. Các lỗi gốc phát hiện thêm nhưng KHÔNG sửa trong GD01.2

Kế thừa danh sách lỗi ngoài phạm vi trong `docs/GD01_SOURCE_AUDIT.md` (mục 5) và bổ sung các điểm ghi nhận thêm khi audit refactor (đều **giữ nguyên hành vi** trong GD01.2 theo ràng buộc):
1. **Spam UART khi điều khiển tay gặp vật cản trước (`RobotApp.cpp`, nhánh `ST_IDLE`/`ST_DONE`/`ST_ERR`):** Khi giữ nút tiến (`l > 0 && r > 0`) và `us[0] < US_BLOCK_MM`, `emergencyStopMotors()` được gọi mỗi vòng `loop()`, bắn liên tục `"M,0,0\n"` ra `Serial1` trong suốt thời gian `manualUntil` còn hiệu lực.
2. **Nguy cơ khóa vĩnh viễn bộ sửa vị trí (`PoseEstimator.cpp`):** Nếu trượt bánh làm vị trí ước lượng lệch khỏi LiDAR quá `POS_JUMP_MAX_MM` (`300 mm`), điều kiện `fabsf(mxv - gRobot.px_mm) < POS_JUMP_MAX_MM` từ chối mọi bản cập nhật vị trí về sau.
3. **Đồng bộ ISR dùng `noInterrupts()`/`interrupts()` thay vì `portMUX_TYPE` (`Ultrasonic.cpp`):** Nên chuyển sang `portENTER_CRITICAL`/`portEXIT_CRITICAL` chuẩn FreeRTOS ESP32 ở giai đoạn Safety/Hardening riêng.
4. **OTA mDNS discovery (`OtaManager.cpp`):** `ArduinoOTA.setMdnsEnabled(false)` tránh xung đột với `MDNS.begin("robot")` nhưng không đăng ký dịch vụ `_arduino._tcp` qua `MDNS.enableArduino()`, nên Arduino IDE có thể cần nhập IP thủ công khi nạp OTA.
5. **Hardcoded WiFi/AP/OTA credentials mặc định (`NetworkConfig.h`):** Cần tách secrets khỏi lịch sử Git và xoay vòng mật khẩu trước khi publish repository công khai.
6. **Toàn bộ các lỗi an toàn & phần cứng đã liệt kê tại `docs/GD01_SOURCE_AUDIT.md` mục 5:** `HW-BLOCKER-01` (quá tải L293D), `HW-BLOCKER-02` (thiếu cliff sensor & E-Stop cứng), `HW-BLOCKER-03` (kiểm tra cầu phân áp), `blockCount` dùng chung 2 cảm biến, lệnh `S` không thoát `ST_RUN`, v.v.
