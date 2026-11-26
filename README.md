# SmartCleaningRobot

Firmware hệ thống robot lau nhà tự hành kiến trúc Dual-MCU (**ESP32** Master + **Arduino Uno** Slave), tích hợp **Adaptive Zigzag Coverage Planner V2** và kiến trúc **Modular Firmware (GD01.2)** phục vụ bảo trì dài hạn và phát triển Mobile App (GD02+).

## 1. Cấu trúc thư mục

```text
SmartCleaningRobot/
├── esp32_robot/                     # Sketch ESP32 tương thích Arduino IDE / Arduino CLI
│   ├── esp32_robot.ino              # Entry point tối giản (gọi robotSetup / robotLoop)
│   └── src/
│       ├── config/                  # Cấu hình chân, thông số robot, mạng/OTA, NVS Settings
│       ├── core/                    # RobotApp, RobotTypes, RobotContext, MathUtils
│       ├── sensors/                 # LiDAR LD14, Ultrasonic (ISR), ObstacleDetector
│       ├── localization/            # WallDetection (PCA/TLS), PoseEstimator, Calibration
│       ├── planning/                # LaneTypes, CoveragePlanner V2, LaneTracker (host-testable)
│       ├── navigation/              # Navigator, CleaningSession
│       ├── control/                 # MotionController (PWM/Ramp), CommandHandler
│       ├── communication/           # ArduinoLink (UART1/High-Z), WiFiManager, OtaManager
│       └── web/                     # RobotWebServer, ApiHandlers, WebPage (PROGMEM)
├── arduino_robot/
│   └── arduino_robot.ino            # Firmware Arduino Uno (HW-130/L293D + HC-05) - giữ nguyên
├── tests/                           # 44 Unit tests (Groups A-E) + 18 Benchmark scenarios
├── docs/                            # Tài liệu kiểm toán, thuật toán GD01 & refactor GD01.2
├── README.md
├── CHANGELOG.md
└── .gitignore
```

## 2. Sơ đồ phần cứng (Hardware Mapping)

### ESP32 (Master)
- **LiDAR LD14:** `TX -> GPIO16` (`UART2`, `230400 bps`, RX buffer `2048 B`).
- **Liên kết Arduino Uno (`UART1`, `38400 bps`):**
  - `GPIO19 (TX)` → Arduino `D0 (RX)`
  - `GPIO18 (RX)` ← Arduino `D1 (TX)` (qua cầu phân áp 5V → 3.3V, 1k/2k)
  - Hỗ trợ chế độ `High-Z` (`/api/unomode?en=1`) để nạp Arduino Uno qua USB không cần tháo dây.
- **3 cảm biến siêu âm (ECHO qua cầu phân áp 1k/2k):**
  - Trước: `TRIG 25 / ECHO 26`
  - Trái: `TRIG 27 / ECHO 14`
  - Phải: `TRIG 32 / ECHO 33`

### Arduino Uno (Slave — giữ nguyên trong GD01.2)
- **Shield HW-130 (L293D / `AFMotor.h`):** `M1` = 3 bánh trái, `M2` = 3 bánh phải, `M3/M4` = 2 chổi lau.
- **HC-05 Bluetooth (`SoftwareSerial` `9600 bps`):** `TX -> A0`, `RX <- A1` (qua cầu phân áp).

## 3. Cung cấp thông tin mạng / OTA (Credentials)

Không đưa mật khẩu thực tế vào tài liệu hoặc lịch sử build log. Để ghi đè thông tin WiFi / AP / OTA cục bộ mà không sửa mã nguồn chia sẻ:

1. Sao chép `esp32_robot/src/config/Secrets.h.example` thành `esp32_robot/src/config/Secrets.h`.
2. Điền thông tin `WIFI_SSID`, `WIFI_PASS`, `AP_SSID`, `AP_PASS`, `OTA_HOSTNAME`, `OTA_PASSWORD` cục bộ.
3. File `Secrets.h` được liệt kê trong `.gitignore` và sẽ tự động được nạp bởi `NetworkConfig.h` qua `#if __has_include("Secrets.h")`.
4. Trước khi publish lên remote công khai, cần xoay vòng mật khẩu và gỡ giá trị mặc định khỏi `NetworkConfig.h` trong một commit cấu hình riêng.

## 4. HTTP REST API Contract

| Endpoint | Mô tả | Status Codes |
|---|---|---|
| `GET /` | Giao diện Web UI nhúng (`PROGMEM`) | `200` |
| `GET /api/status` | JSON trạng thái robot, vị trí, siêu âm, PWM, tiến độ làn, `cleaning_result` | `200` |
| `GET /api/scan` | JSON mảng 360 giá trị khoảng cách LiDAR (mm) | `200` |
| `GET /api/scan.bin` | Binary `720 byte` (`360 x uint16_t` little-endian) cho Mobile App | `200` |
| `GET /api/plan` | JSON kế hoạch V2 (`direction`, `start_corner`, `metrics`, danh sách `lanes[]`) | `200` |
| `GET /api/settings` | Đọc / lưu cấu hình NVS (`mop`, `cruise`, `edge`, `overlap`, `direction`) | `200`, `400`, `409` |
| `GET /api/cmd` | Lệnh điều khiển (`c=start\|stop\|F\|B\|L\|R\|S\|mopon\|mopoff`, tùy chọn `v`) | `200` |
| `GET /api/unomode` | Bật/tắt chế độ High-Z GPIO18/19 (`en=1\|0`) | `200`, `409` |

## 5. Hướng dẫn Build & Kiểm thử

### 5.1. Build ESP32 bằng Arduino IDE (1.8.x / 2.x)
1. Cài đặt board package **esp32 by Espressif Systems** (2.x hoặc 3.x).
2. Mở file `esp32_robot/esp32_robot.ino` (Arduino IDE tự động biên dịch đệ quy toàn bộ file trong `esp32_robot/src/`).
3. Chọn Board: **ESP32 Dev Module**.
4. Nhấn **Verify** để biên dịch. Không tự ý Upload hoặc chạy động cơ khi chưa có người giám sát an toàn phần cứng.

### 5.2. Build ESP32 bằng Arduino CLI
```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 esp32_robot
```

### 5.3. Chạy Unit Tests & Benchmark trên PC (Host C++17)
Các module trong `esp32_robot/src/planning/` hoàn toàn độc lập phần cứng, có thể biên dịch trực tiếp bằng `g++` hoặc `clang++` (hoặc `zig c++`):

```bash
# Ví dụ với g++ / clang++ (chạy từ thư mục gốc SmartCleaningRobot)
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_coverage_planner.cpp esp32_robot/src/planning/CoveragePlanner.cpp -o build/test_coverage_planner && ./build/test_coverage_planner
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_plan_validation.cpp  esp32_robot/src/planning/CoveragePlanner.cpp -o build/test_plan_validation  && ./build/test_plan_validation
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_lane_tracker.cpp     esp32_robot/src/planning/CoveragePlanner.cpp esp32_robot/src/planning/LaneTracker.cpp -o build/test_lane_tracker && ./build/test_lane_tracker
g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_progress.cpp         esp32_robot/src/planning/CoveragePlanner.cpp esp32_robot/src/planning/LaneTracker.cpp -o build/test_progress     && ./build/test_progress
```

## 6. Tài liệu kỹ thuật

- GD01 (Thuật toán Planner V2): `docs/GD01_SOURCE_AUDIT.md`, `docs/GD01_ALGORITHM_DESIGN.md`, `docs/GD01_INTEGRATION_REPORT.md`, `docs/GD01_TEST_REPORT.md`, `docs/GD01_HANDOVER.md`.
- GD01.2 (Modular Refactoring): `docs/GD01_2_REFACTOR_PLAN.md`, `docs/GD01_2_MODULE_ARCHITECTURE.md`, `docs/GD01_2_FUNCTION_MAPPING.md`, `docs/GD01_2_DEPENDENCY_MAP.md`, `docs/GD01_2_REGRESSION_REPORT.md`, `docs/GD01_2_HANDOVER.md`.
