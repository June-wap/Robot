# GD01.2 – KIẾN TRÚC MODULE FIRMWARE ESP32 (MODULE ARCHITECTURE)

## 1. Tổng quan cây thư mục cuối cùng

```text
SmartCleaningRobot/
├── esp32_robot/
│   ├── esp32_robot.ino              # 27 dòng - Arduino IDE/CLI entry point
│   └── src/
│       ├── config/
│       │   ├── HardwareConfig.h     # Chân GPIO, baud rate, ngưỡng lọc LiDAR
│       │   ├── RobotConfig.h        # Kích thước robot, PWM/Ramp, hệ số P, ngưỡng vật cản
│       │   ├── NetworkConfig.h      # Macro WiFi/AP/mDNS/OTA (+ optional Secrets.h)
│       │   ├── Secrets.h.example    # Mẫu file cấu hình credentials cục bộ (không chứa pass thật)
│       │   ├── Settings.h           # Giao diện NVS Preferences & strict argument parsers
│       │   └── Settings.cpp         # loadSettings(), saveSettings(), parseStrict*
│       ├── core/
│       │   ├── RobotTypes.h         # struct Wall, enum State, enum Phase
│       │   ├── RobotContext.h       # Struct trạng thái dùng chung duy nhất (extern gRobot)
│       │   ├── RobotContext.cpp     # Khởi tạo giá trị mặc định khớp 100% baseline
│       │   ├── MathUtils.h          # Khai báo wrap180, wrap90, clampi, clampf, median3
│       │   ├── MathUtils.cpp        # Cài đặt hàm toán học & lọc trung vị
│       │   ├── RobotApp.h           # Entry orchestrator robotSetup() / robotLoop()
│       │   └── RobotApp.cpp         # Điều phối toàn bộ vòng đời đúng thứ tự baseline
│       ├── sensors/
│       │   ├── Lidar.h              # Giao diện driver LiDAR LD14
│       │   ├── Lidar.cpp            # Bảng CRC-8, parser gói 47 byte, rollover 360°
│       │   ├── Ultrasonic.h         # Giao diện siêu âm 3 kênh + khai báo ISR IRAM_ATTR
│       │   ├── Ultrasonic.cpp       # ISR đo xung ECHO, volatile flags, lọc median3
│       │   ├── ObstacleDetector.h   # Giao diện phát hiện vật cản trước & khoảng hở tường hông
│       │   └── ObstacleDetector.cpp # frontBlockedRaw(), expectedSideClearance()
│       ├── localization/
│       │   ├── WallDetection.h      # Giao diện khớp tường PCA/TLS
│       │   ├── WallDetection.cpp    # fitWall() (cửa sổ ±25°, RMS <= 25 mm)
│       │   ├── PoseEstimator.h      # Giao diện định vị dead-reckoning & sửa vị trí
│       │   ├── PoseEstimator.cpp    # poseInitTimer(), predictPose(), correctPose()
│       │   ├── Calibration.h        # Giao diện hiệu chỉnh 4 tường đầu phiên
│       │   └── Calibration.cpp      # resetCalibration(), doCalibrate()
│       ├── planning/                # ĐỘC LẬP PHẦN CỨNG (Host-testable C++17)
│       │   ├── LaneTypes.h          # Kiểu dữ liệu làn, bước, tiến độ, kết quả, giới hạn tĩnh
│       │   ├── CoveragePlanner.h    # Khai báo Adaptive Zigzag Coverage Planner V2
│       │   ├── CoveragePlanner.cpp  # Đánh giá 8 candidates x 2 approach orders
│       │   ├── LaneTracker.h        # Khai báo theo dõi trạng thái làn & tiến độ
│       │   └── LaneTracker.cpp      # Máy trạng thái làn & CleaningResult
│       ├── navigation/
│       │   ├── CleaningSession.h    # Quản lý vòng đời phiên lau
│       │   ├── CleaningSession.cpp  # buildPlan(), startCleaning(), finishCleaning(), stopAll(), setError()
│       │   ├── Navigator.h          # Điều hướng từng bước
│       │   └── Navigator.cpp        # resetNavigationState(), runNav() (PH_TURN/PH_DRIVE/PH_BACKOFF)
│       ├── control/
│       │   ├── MotionController.h   # Giao diện điều khiển PWM & Soft-Start Ramp
│       │   ├── MotionController.cpp # drive(), stopMotors(), emergencyStopMotors(), rampAxis()
│       │   ├── CommandHandler.h     # Giao diện xử lý lệnh chung cho Web API & HC-05
│       │   └── CommandHandler.cpp   # applyCmd()
│       ├── communication/
│       │   ├── ArduinoLink.h        # Giao diện UART1 <-> Arduino Uno + High-Z flash mode
│       │   ├── ArduinoLink.cpp      # arduinoLinkInit(), pollLink(), sendLink(), setUnoFlashMode()
│       │   ├── WiFiManager.h        # Giao diện kết nối WiFi STA / fallback AP / mDNS
│       │   ├── WiFiManager.cpp      # startWiFi()
│       │   ├── OtaManager.h         # Giao diện ArduinoOTA
│       │   └── OtaManager.cpp       # otaInit(), otaHandle()
│       └── web/
│           ├── RobotWebServer.h     # Sở hữu WebServer(80) và đăng ký route
│           ├── RobotWebServer.cpp   # getWebServer(), webServerInit(), webServerHandleClient()
│           ├── ApiHandlers.h        # Khai báo 8 HTTP route handlers
│           ├── ApiHandlers.cpp      # / , /api/status, /api/scan, /api/scan.bin, /api/cmd, /api/settings, /api/plan, /api/unomode
│           ├── WebPage.h            # Khai báo extern INDEX_HTML PROGMEM
│           └── WebPage.cpp          # Nội dung HTML/CSS/JS nhúng nguyên văn từ baseline
├── arduino_robot/
│   └── arduino_robot.ino            # KHÔNG THAY ĐỔI (SHA256 giữ nguyên 100%)
└── tests/                           # 4 bộ unit tests + benchmark (44 tests + 18 scenarios)
```

---

## 2. Trách nhiệm từng phân hệ (Module Responsibilities)

| Phân hệ | Vai trò | Biên giới sở hữu |
|---|---|---|
| `config/` | Chứa toàn bộ hằng số phần cứng, tham số robot, cấu hình mạng và đồng bộ cấu hình runtime vào Flash NVS | Sở hữu `Preferences prefs` (static trong `Settings.cpp`); không chứa logic điều hướng hay HTTP |
| `core/` | Định nghĩa kiểu cốt lõi (`State`, `Phase`, `Wall`), trạng thái dùng chung duy nhất (`RobotContext gRobot`), hàm toán học thuần và bộ điều phối `RobotApp` | `RobotApp` giữ chính xác trình tự `setup()` và `loop()`, không chứa thuật toán chi tiết |
| `sensors/` | Thu thập và lọc sơ bộ dữ liệu LiDAR LD14, siêu âm 3 kênh (ngắt ngoài `IRAM_ATTR`) và phát hiện vật cản | Đóng gói toàn bộ biến ISR `volatile` (`usRiseUs`, `usWidthUs`, `usHigh`, `usDone`) và buffer tích lũy LiDAR (`acc`, `crcTable`) |
| `localization/` | Ước lượng tư thế robot `(x, y, psi)` và đo kích thước phòng chữ nhật bằng PCA/TLS | Sở hữu timer `lastPoseMs` và bộ đếm `calibOk/calibFail/calibD/calibSin/calibCos` |
| `planning/` | Thuật toán chia làn Adaptive Zigzag V2 và quản lý tiến độ/kết quả làn | **100% độc lập Arduino/ESP32**, không dùng heap động, biên dịch trực tiếp trên host C++ cho unit test & benchmark |
| `navigation/` | Thực thi từng bước trong `CoveragePlan` (`PH_TURN`/`PH_DRIVE`/`PH_BACKOFF`) và quản lý vòng đời phiên lau | Sở hữu biến nội bộ pha chạy (`stepStarted`, `phase`, `tgtHeading`, `stepStartMs`, `backoffUntilMs`, `stepRetries`, `blockCount`) |
| `control/` | Chuyển lệnh hướng/tốc độ thành mục tiêu PWM (`cmdL/cmdR`), soft-start ramp, dừng khẩn cấp và giải mã lệnh điều khiển | Tách biệt hoàn toàn khỏi thuật toán chia làn và giao thức HTTP |
| `communication/` | Truyền thông UART1 (`38400 bps`) với Arduino Uno (bao gồm chế độ High-Z nạp Uno), kết nối WiFi STA/AP + mDNS và cập nhật OTA | Sở hữu lịch gửi `lastSendMs`, `lastMopMs`, `lastTelMs`, `lastSentMop` |
| `web/` | Khởi tạo HTTP Server, định tuyến 8 API endpoints và lưu trữ Web UI (`PROGMEM`) | Sở hữu đối tượng `WebServer server(80)` và chuỗi `INDEX_HTML`; tách sạch khỏi điều hướng để dễ mở rộng cho Mobile App GD02 |

---

## 3. Phân loại quản lý trạng thái (Shared vs. Module-Internal State)

### 3.1. Trạng thái dùng chung duy nhất (`RobotContext gRobot`)
- **Trạng thái hệ thống:** `state`, `errMsg`, `myIP`.
- **Kế hoạch & tiến độ (không sao chép lớn):** `activePlan` (`4,696 bytes`), `laneTracker` (`88 bytes`), `stepIdx`.
- **Định vị & phòng:** `px_mm`, `py_mm`, `psi`, `roomW`, `roomL`, `haveRoom`.
- **Đầu ra cảm biến:** `scan[360]` (`720 bytes`), `volatile bool newScan`, `lastScanMs`, `us[3]`, `newUsFront`.
- **Điều khiển động cơ & lái tay:** `cmdL`, `cmdR`, `outL`, `outR`, `mopPwm`, `manualUntil`, `manualL`, `manualR`.
- **Liên kết Uno:** `unoFlashMode`.
- **Cài đặt runtime:** `cfgMopWidth`, `cfgCruise`, `cfgEdge`, `cfgOverlap`, `cfgDirection`.

### 3.2. Trạng thái đóng gói nội bộ (`static` trong `.cpp`)
- `Settings.cpp`: `prefs`.
- `Lidar.cpp`: `acc[360]`, `lastStartAngle`, `crcTable[256]`, `buf[47]`/`idx` (trong `pollLidar`).
- `Ultrasonic.cpp`: `volatile usRiseUs[3]`, `volatile usWidthUs[3]`, `volatile usHigh[3]`, `volatile usDone[3]`, `usHist[3][3]`, `usHistIdx[3]`, `usTurn`, `usBusy`, `lastUsMs`, `usTrigMs`.
- `PoseEstimator.cpp`: `lastPoseMs`.
- `Calibration.cpp`: `calibOk`, `calibFail`, `calibD[4]`, `calibSin`, `calibCos`.
- `Navigator.cpp`: `stepStarted`, `phase`, `tgtHeading`, `stepStartMs`, `backoffUntilMs`, `stepRetries`, `blockCount`.
- `ArduinoLink.cpp`: `lastSentMop`, `lastSendMs`, `lastMopMs`, `lastTelMs`, `line[24]`/`n` (trong `pollLink`).
- `RobotWebServer.cpp`: `server(80)`.
