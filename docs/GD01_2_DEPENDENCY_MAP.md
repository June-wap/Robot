# GD01.2 – BẢN ĐỒ PHỤ THUỘC MODULE (DEPENDENCY MAP)

## 1. Phân tầng kiến trúc & Đảm bảo không phụ thuộc vòng tròn (Acyclic Headers)

Toàn bộ 27 header trong `esp32_robot/src/` đã được kiểm chứng biên dịch độc lập (`-std=c++17 -Wall -Wextra -Werror`) với **0 lỗi, 0 cảnh báo, 0 circular include**.

```text
Layer 0 (Leaf / Hardware-Independent):
  config/HardwareConfig.h   config/RobotConfig.h   config/NetworkConfig.h
  core/RobotTypes.h         core/MathUtils.*
  planning/LaneTypes.h      planning/CoveragePlanner.*   planning/LaneTracker.*
  web/WebPage.*

Layer 1 (Shared State):
  core/RobotContext.*  ──► RobotTypes.h, RobotConfig.h, LaneTypes.h, LaneTracker.h

Layer 2 (Drivers, Storage & Estimation):
  config/Settings.*             ──► RobotContext, RobotConfig, MathUtils, Preferences
  sensors/Lidar.*               ──► HardwareConfig, RobotContext
  sensors/Ultrasonic.*          ──► HardwareConfig, RobotConfig, MathUtils, RobotContext
  sensors/ObstacleDetector.*    ──► RobotConfig, RobotContext
  localization/WallDetection.*  ──► RobotTypes, RobotConfig, RobotContext
  localization/PoseEstimator.*  ──► WallDetection, RobotConfig, MathUtils, RobotContext
  communication/WiFiManager.*   ──► NetworkConfig, RobotContext, WiFi, ESPmDNS

Layer 3 (Control, Navigation, Link & OTA):
  control/MotionController.*    ──► RobotConfig, MathUtils, RobotContext, ArduinoLink (.cpp)
  communication/ArduinoLink.*   ──► HardwareConfig, RobotContext, MotionController (.cpp), CommandHandler (.cpp)
  communication/OtaManager.*    ──► NetworkConfig, RobotContext, ArduinoLink, ArduinoOTA
  navigation/CleaningSession.*  ──► RobotConfig, RobotContext, CoveragePlanner, MotionController, Calibration (.cpp), Navigator (.cpp)
  localization/Calibration.*    ──► WallDetection, RobotConfig, MathUtils, RobotContext, CleaningSession (.cpp), Navigator (.cpp)
  navigation/Navigator.*        ──► RobotConfig, MathUtils, RobotContext, ObstacleDetector, MotionController, CleaningSession (.cpp)
  control/CommandHandler.*      ──► RobotConfig, MathUtils, RobotContext, MotionController, CleaningSession (.cpp)

Layer 4 (Web Interface & Orchestrator):
  web/ApiHandlers.*             ──► RobotWebServer, WebPage, Settings, CommandHandler, ArduinoLink, RobotContext
  web/RobotWebServer.*          ──► WebServer, ApiHandlers (.cpp)
  core/RobotApp.*               ──► Settings, Lidar, Ultrasonic, PoseEstimator, Calibration, MotionController, CleaningSession, Navigator, ArduinoLink, WiFiManager, OtaManager, RobotWebServer, RobotContext
  esp32_robot.ino               ──► core/RobotApp.h
```

---

## 2. Bảng chi tiết Dependency giữa các module

| Module (`.cpp`) | Header nội bộ được include | Thư viện Arduino/ESP32 |
|---|---|---|
| `core/MathUtils.cpp` | `MathUtils.h` | `<math.h>` |
| `core/RobotContext.cpp` | `RobotContext.h` | `<Arduino.h>`, `<string.h>` |
| `config/Settings.cpp` | `Settings.h`, `RobotConfig.h`, `MathUtils.h`, `RobotContext.h` | `<Preferences.h>`, `<math.h>`, `<stdlib.h>` |
| `planning/CoveragePlanner.cpp` | `CoveragePlanner.h` (`LaneTypes.h`) | **Không phụ thuộc Arduino** (`<math.h>`, `<string.h>`) |
| `planning/LaneTracker.cpp` | `LaneTracker.h` (`LaneTypes.h`) | **Không phụ thuộc Arduino** (`<math.h>`, `<string.h>`) |
| `sensors/Lidar.cpp` | `Lidar.h`, `HardwareConfig.h`, `RobotContext.h` | `<Arduino.h>`, `<string.h>` |
| `sensors/Ultrasonic.cpp` | `Ultrasonic.h`, `HardwareConfig.h`, `RobotConfig.h`, `MathUtils.h`, `RobotContext.h` | `<Arduino.h>` |
| `sensors/ObstacleDetector.cpp` | `ObstacleDetector.h`, `RobotConfig.h`, `RobotContext.h` | `<math.h>`, `<stdint.h>` |
| `localization/WallDetection.cpp` | `WallDetection.h`, `RobotConfig.h`, `RobotContext.h` | `<Arduino.h>`, `<math.h>`, `<stdint.h>` |
| `localization/PoseEstimator.cpp` | `PoseEstimator.h`, `WallDetection.h`, `RobotConfig.h`, `MathUtils.h`, `RobotContext.h`, `RobotTypes.h` | `<Arduino.h>`, `<math.h>` |
| `localization/Calibration.cpp` | `Calibration.h`, `WallDetection.h`, `RobotConfig.h`, `MathUtils.h`, `RobotContext.h`, `RobotTypes.h`, `CleaningSession.h`, `Navigator.h` | `<Arduino.h>`, `<math.h>` |
| `control/MotionController.cpp` | `MotionController.h`, `ArduinoLink.h`, `RobotConfig.h`, `MathUtils.h`, `RobotContext.h` | `<stdlib.h>` |
| `control/CommandHandler.cpp` | `CommandHandler.h`, `MotionController.h`, `RobotConfig.h`, `MathUtils.h`, `RobotContext.h`, `RobotTypes.h`, `CleaningSession.h` | `<Arduino.h>` |
| `navigation/CleaningSession.cpp` | `CleaningSession.h`, `Navigator.h`, `RobotConfig.h`, `MotionController.h`, `RobotContext.h`, `RobotTypes.h`, `Calibration.h`, `CoveragePlanner.h`, `LaneTypes.h` | `<Arduino.h>`, `<string.h>` |
| `navigation/Navigator.cpp` | `Navigator.h`, `CleaningSession.h`, `RobotConfig.h`, `MotionController.h`, `MathUtils.h`, `RobotContext.h`, `RobotTypes.h`, `LaneTypes.h`, `ObstacleDetector.h` | `<Arduino.h>`, `<math.h>` |
| `communication/ArduinoLink.cpp` | `ArduinoLink.h`, `HardwareConfig.h`, `CommandHandler.h`, `MotionController.h`, `RobotContext.h` | `<Arduino.h>` |
| `communication/WiFiManager.cpp` | `WiFiManager.h`, `NetworkConfig.h`, `RobotContext.h` | `<Arduino.h>`, `<ESPmDNS.h>`, `<WiFi.h>` |
| `communication/OtaManager.cpp` | `OtaManager.h`, `ArduinoLink.h`, `NetworkConfig.h`, `RobotContext.h` | `<Arduino.h>`, `<ArduinoOTA.h>` |
| `web/WebPage.cpp` | `WebPage.h` | `<Arduino.h>` (`PROGMEM`) |
| `web/RobotWebServer.cpp` | `RobotWebServer.h`, `ApiHandlers.h` | `<WebServer.h>` |
| `web/ApiHandlers.cpp` | `ApiHandlers.h`, `RobotWebServer.h`, `WebPage.h`, `ArduinoLink.h`, `RobotConfig.h`, `Settings.h`, `CommandHandler.h`, `RobotContext.h`, `RobotTypes.h`, `LaneTypes.h` | `<Arduino.h>`, `<stdio.h>` |
| `core/RobotApp.cpp` | `RobotApp.h`, `RobotContext.h`, `RobotTypes.h`, và các header công khai của 8 phân hệ | `<Arduino.h>` |

---

## 3. Ghi chú quan hệ tương hỗ ở mức implementation (.cpp)

Một số cặp module tương tác hai chiều ở mức lời gọi hàm trong `.cpp` nhưng **hoàn toàn không có phụ thuộc vòng trong `.h`**:
- `MotionController.cpp` (`emergencyStopMotors`) → `ArduinoLink.h` (`arduinoLinkSendImmediateStop`), và `ArduinoLink.cpp` (`sendLink`) → `MotionController.h` (`rampAxis`).
- `CleaningSession.cpp` (`startCleaning`) → `Calibration.h` (`resetCalibration`) & `Navigator.h` (`resetNavigationState`), trong khi `Calibration.cpp` và `Navigator.cpp` gọi ngược `CleaningSession.h` (`buildPlan`, `finishCleaning`, `setError`).
- `RobotWebServer.cpp` đăng ký callback từ `ApiHandlers.h`, còn `ApiHandlers.cpp` lấy `WebServer&` qua `RobotWebServer.h` (`getWebServer()`).

Tất cả các header chỉ khai báo chữ ký hàm tối giản nên đảm bảo thời gian biên dịch nhanh và không phát sinh chu trình include.
