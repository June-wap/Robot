# GD01.2 – BẢNG ÁNH XẠ HÀM VÀ BIẾN (FUNCTION & VARIABLE MAPPING)

Tài liệu đối chiếu 1–1 toàn bộ hàm, hằng số và biến từ file monolithic baseline `esp32_robot/esp32_robot.ino` (commit `f2b6db1`, 1,327 dòng) sang cấu trúc modular mới (`esp32_robot/src/`).

---

## 1. Ánh xạ hàm (Function Migration Mapping)

| Hàm gốc (`esp32_robot.ino` baseline) | File mới (Module đích) | Ghi chú bảo toàn |
|---|---|---|
| `wrap180`, `wrap90`, `clampi`, `clampf`, `median3` | `src/core/MathUtils.h/.cpp` | Công thức và kiểu dữ liệu giữ nguyên 100% |
| `drive`, `stopMotors`, `emergencyStopMotors`, `rampAxis` | `src/control/MotionController.h/.cpp` | `emergencyStopMotors` gọi `arduinoLinkSendImmediateStop()` để giữ đúng `Serial1.print("M,0,0\n")` + `lastSendMs = millis()` |
| `applyCmd` | `src/control/CommandHandler.h/.cpp` | Giữ nguyên ngưỡng clamp `80..MAX_SAFE_PWM`, timeout lái tay `400 ms` |
| `setUnoFlashMode`, `pollLink`, `sendLink` | `src/communication/ArduinoLink.h/.cpp` | Bổ sung `arduinoLinkInit()`, `arduinoLinkSendImmediateStop()`, `arduinoLinkSendOtaStop()` giữ nguyên timing 50/300/2000 ms |
| `initCrc`, `calcCrc`, `handlePacket`, `pollLidar` | `src/sensors/Lidar.h/.cpp` | Bổ sung `lidarBeginSerial()`, `lidarInit()` bảo toàn thứ tự trong `setup()` |
| `handleUsIsr`, `isrUs0`, `isrUs1`, `isrUs2`, `pushUsSample`, `pollUltrasonic` | `src/sensors/Ultrasonic.h/.cpp` | Giữ nguyên `IRAM_ATTR`, `volatile`, `noInterrupts()/interrupts()`, `median3`, `US_PERIOD_MS`, `US_TIMEOUT_MS`; thêm `ultrasonicInit()` |
| `frontBlockedRaw`, `expectedSideClearance` | `src/sensors/ObstacleDetector.h/.cpp` | Giữ nguyên góc quét `±15°`, `US_BLOCK_MM`, `LIDAR_BLOCK_MM` |
| `fitWall` | `src/localization/WallDetection.h/.cpp` | Giữ nguyên PCA/TLS `±25°`, `n >= 10`, `rms <= 25.0f` |
| `predictPose`, `correctPose` | `src/localization/PoseEstimator.h/.cpp` | Bổ sung `poseInitTimer()`; giữ nguyên `K_YAW`, `K_POS`, `POS_JUMP_MAX_MM` |
| `doCalibrate` | `src/localization/Calibration.h/.cpp` | Bổ sung `resetCalibration()`; giữ nguyên 5 khung đạt / tối đa 60 khung lỗi |
| `setError`, `buildPlan`, `startCleaning`, `finishCleaning`, `stopAll` | `src/navigation/CleaningSession.h/.cpp` | Giữ nguyên tương tác với `CoveragePlanner` và `LaneTracker` |
| `runNav` | `src/navigation/Navigator.h/.cpp` | Bổ sung `resetNavigationState()`; giữ nguyên mọi nhánh `CLEAN_LANE`, `CHANGE_LANE`, `APPROACH_START`, `PH_TURN`, `PH_DRIVE`, `PH_BACKOFF` |
| `parseStrictFloat`, `parseStrictInt`, `parseDirectionArg`, `loadSettings` (+ đoạn lưu `prefs.put*`) | `src/config/Settings.h/.cpp` | Đóng gói thao tác ghi NVS thành `saveSettings()`; giữ nguyên namespace `"robot"` và các khóa `"mop"`, `"cruise"`, `"edge"`, `"overlap"`, `"dir"` |
| `startWiFi` | `src/communication/WiFiManager.h/.cpp` | Giữ nguyên STA 15 s -> fallback AP + mDNS `"robot"` |
| Khối cấu hình `ArduinoOTA` trong `setup()` & `ArduinoOTA.handle()` | `src/communication/OtaManager.h/.cpp` | Tách thành `otaInit()` và `otaHandle()`; giữ nguyên callback `onStart` |
| `WebServer server(80)`, đăng ký 8 route, `server.begin()`, `server.handleClient()` | `src/web/RobotWebServer.h/.cpp` | Đóng gói qua `getWebServer()`, `webServerInit()`, `webServerHandleClient()` |
| `handleRoot`, `handleStatus`, `handleScan`, `handleScanBin`, `handleCmd`, `handleUnoMode`, `handleSettings`, `handlePlan` | `src/web/ApiHandlers.h/.cpp` | Giữ nguyên 100% đường dẫn, query args, JSON schema, HTTP status codes và headers |
| `INDEX_HTML` (`PROGMEM`) | `src/web/WebPage.h/.cpp` | Trùng khớp 100% SHA256 byte-for-byte |
| `setup()`, `loop()` | `src/core/RobotApp.h/.cpp` (`robotSetup()`, `robotLoop()`) + `esp32_robot.ino` | `esp32_robot.ino` gọi thẳng `robotSetup()` và `robotLoop()`, thứ tự từng dòng giữ nguyên tuyệt đối |

---

## 2. Ánh xạ file Planning GD01 (Di chuyển nguyên vẹn)

| File cũ (`f2b6db1`) | File mới (`refactor/gd01-modular-firmware`) | Thay đổi nội dung |
|---|---|---|
| `esp32_robot/LaneTypes.h` | `esp32_robot/src/planning/LaneTypes.h` | 0 byte thay đổi (100% rename) |
| `esp32_robot/CoveragePlanner.h` | `esp32_robot/src/planning/CoveragePlanner.h` | 0 byte thay đổi (100% rename) |
| `esp32_robot/CoveragePlanner.cpp` | `esp32_robot/src/planning/CoveragePlanner.cpp` | 0 byte thay đổi (100% rename) |
| `esp32_robot/LaneTracker.h` | `esp32_robot/src/planning/LaneTracker.h` | 0 byte thay đổi (100% rename) |
| `esp32_robot/LaneTracker.cpp` | `esp32_robot/src/planning/LaneTracker.cpp` | 0 byte thay đổi (100% rename) |

---

## 3. Ánh xạ hằng số và biến toàn cục (Constants & Global Variables)

| Nhóm cũ (`esp32_robot.ino`) | Vị trí mới | Phạm vi sở hữu |
|---|---|---|
| `LIDAR_RX_PIN`, `LINK_TX_PIN`, `LINK_RX_PIN`, `US_TRIG[3]`, `US_ECHO[3]`, `LIDAR_BAUD`, `LINK_BAUD`, `LIDAR_*` | `src/config/HardwareConfig.h` | Macro / `constexpr` toàn cục |
| `ROBOT_RADIUS_MM`, `DEF_EDGE_MM`, `DEF_MOP_WIDTH_MM`, `V_MAX_MM_S`, `SPIN_MAX_DEG_S`, `*_PWM`, `PWM_*_STEP`, `KT`, `KH`, `TURN_TOL_DEG`, `K_YAW`, `K_POS`, `POS_JUMP_MAX_MM`, `US_*`, `LIDAR_BLOCK_MM`, `SIDE_NEAR_MM`, `WALL_HALF_DEG`, `BLOCK_CONFIRM_CNT`, `DEF_OVERLAP_PCT` | `src/config/RobotConfig.h` | Macro toàn cục |
| `WIFI_SSID`, `WIFI_PASS`, `AP_SSID`, `AP_PASS`, `MDNS_HOSTNAME`, `OTA_HOSTNAME`, `OTA_PASSWORD` | `src/config/NetworkConfig.h` (+ tùy chọn `Secrets.h`) | Macro toàn cục |
| `Wall`, `State`, `Phase` | `src/core/RobotTypes.h` | Kiểu dữ liệu dùng chung |
| `state`, `errMsg`, `myIP`, `activePlan`, `laneTracker`, `stepIdx`, `px_mm`, `py_mm`, `psi`, `roomW`, `roomL`, `haveRoom`, `scan[360]`, `volatile newScan`, `lastScanMs`, `us[3]`, `newUsFront`, `cmdL`, `cmdR`, `outL`, `outR`, `mopPwm`, `manualUntil`, `manualL`, `manualR`, `unoFlashMode`, `cfgMopWidth`, `cfgCruise`, `cfgEdge`, `cfgOverlap`, `cfgDirection` | `src/core/RobotContext.h/.cpp` (`gRobot.*`) | Một thể hiện chia sẻ duy nhất |
| `prefs` | `src/config/Settings.cpp` | `static` nội bộ |
| `acc[360]`, `lastStartAngle`, `crcTable[256]` | `src/sensors/Lidar.cpp` | `static` nội bộ |
| `volatile usRiseUs[3]`, `volatile usWidthUs[3]`, `volatile usHigh[3]`, `volatile usDone[3]`, `usHist[3][3]`, `usHistIdx[3]`, `usTurn`, `usBusy`, `lastUsMs`, `usTrigMs` | `src/sensors/Ultrasonic.cpp` | `static` nội bộ (giữ nguyên `volatile`) |
| `lastPoseMs` | `src/localization/PoseEstimator.cpp` | `static` nội bộ |
| `calibOk`, `calibFail`, `calibD[4]`, `calibSin`, `calibCos` | `src/localization/Calibration.cpp` | `static` nội bộ |
| `stepStarted`, `phase`, `tgtHeading`, `stepStartMs`, `backoffUntilMs`, `stepRetries`, `blockCount` | `src/navigation/Navigator.cpp` | `static` nội bộ |
| `lastSentMop`, `lastSendMs`, `lastMopMs`, `lastTelMs` | `src/communication/ArduinoLink.cpp` | `static` nội bộ |
| `server(80)` | `src/web/RobotWebServer.cpp` | `static` nội bộ (truy cập qua `getWebServer()`) |
