# Changelog

Tất cả các thay đổi quan trọng của dự án **SmartCleaningRobot** được ghi nhận tại đây.

## [GD01.2] - Modular Firmware Refactoring (Branch: `refactor/gd01-modular-firmware`)

### Changed
- Tái cấu trúc firmware ESP32 từ kiến trúc monolithic (`esp32_robot.ino` 1,327 dòng) sang kiến trúc modular gồm 9 phân hệ trong `esp32_robot/src/` (`config`, `core`, `sensors`, `localization`, `planning`, `navigation`, `control`, `communication`, `web`).
- Tối giản file entry point `esp32_robot/esp32_robot.ino` xuống còn 27 dòng, ủy quyền điều phối vòng đời sang `src/core/RobotApp.cpp` với thứ tự `setup()` và `loop()` bảo toàn 100%.
- Tập trung trạng thái dùng chung trong một thể hiện duy nhất `RobotContext gRobot` (`src/core/RobotContext.*`), đưa các biến nội bộ về phạm vi `static` của từng module sở hữu.
- Di chuyển nguyên vẹn 5 file thuật toán GD01 (`LaneTypes.h`, `CoveragePlanner.*`, `LaneTracker.*`) vào `esp32_robot/src/planning/` và cập nhật đường dẫn include của 4 bộ unit tests trong `tests/`.
- Tách `INDEX_HTML` (`PROGMEM`) sang `src/web/WebPage.*` (khớp SHA256 100% byte-for-byte với baseline).
- Bổ sung cơ chế tùy chọn nạp `src/config/Secrets.h` (bị ignore bởi `.gitignore`) kèm mẫu `Secrets.h.example` trong `NetworkConfig.h`.

### Added
- Tài liệu kiến trúc & hồi quy GD01.2: `docs/GD01_2_REFACTOR_PLAN.md`, `docs/GD01_2_MODULE_ARCHITECTURE.md`, `docs/GD01_2_FUNCTION_MAPPING.md`, `docs/GD01_2_DEPENDENCY_MAP.md`, `docs/GD01_2_REGRESSION_REPORT.md`, `docs/GD01_2_HANDOVER.md`.
- Tài liệu chuẩn bị GitHub: `README.md`, `CHANGELOG.md`, `.gitignore`.

### Unchanged
- `arduino_robot/arduino_robot.ino` không thay đổi (SHA256 giữ nguyên `FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13`).
- Bảo toàn 100% thuật toán Planner V2, LaneTracker, LiDAR, Ultrasonic ISR (`IRAM_ATTR`), WallDetection, PoseEstimator, Calibration, Navigator, MotionController (PWM/Ramp), UART protocol, NVS keys, HTTP API contract và Web UI.

## [GD01] - Adaptive Zigzag Coverage Path Planning V2 (Branch: `feature/gd01-adaptive-zigzag`)

### Added
- Triển khai `CoveragePlanner` V2 (8 candidates × 2 thứ tự tiếp cận, hàm chi phí nhất quán theo thời gian, overlap 0–30%).
- Triển khai `LaneTracker` (trạng thái làn `PENDING/RUNNING/COMPLETED/BLOCKED/PARTIAL/FAILED`, `CleaningProgress`, `CleaningResult`).
- Mở rộng REST API (`/api/status`, `/api/plan`, `/api/settings`) và 44 unit tests + 18 benchmark scenarios.
- Tài liệu GD01 (`GD01_SOURCE_AUDIT.md`, `GD01_ALGORITHM_DESIGN.md`, `GD01_INTEGRATION_REPORT.md`, `GD01_TEST_REPORT.md`, `GD01_HANDOVER.md`).

## [Baseline] - Initial Firmware Import (Branch: `main`, commit `a3ce321`)

### Added
- Nhập mã nguồn gốc `esp32_robot/esp32_robot.ino` và `arduino_robot/arduino_robot.ino`.
