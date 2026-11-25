# GD01 – BÁO CÁO TÍCH HỢP (INTEGRATION REPORT)

## 1. Tóm tắt thay đổi mã nguồn

### 1.1. File mới được tạo (Module độc lập phần cứng)
- `esp32_robot/LaneTypes.h`: Định nghĩa kiểu dữ liệu `CleaningDirection`, `StartCorner`, `OverlapMode`, `LaneStatus`, `StepType`, `StepPurpose`, `Step`, `CleaningResult`, `PlanError`, `PlanMetrics`, `CleaningLane`, `CleaningProgress`, `PlanRequest`, `CoveragePlan` và các hàm chuyển chuỗi.
- `esp32_robot/CoveragePlanner.h` & `esp32_robot/CoveragePlanner.cpp`: Thuật toán Adaptive Zigzag Coverage Planner V2 (Level 1), không phụ thuộc `Arduino.h`, không cấp phát heap động.
- `esp32_robot/LaneTracker.h` & `esp32_robot/LaneTracker.cpp`: Bộ quản lý trạng thái làn, tính tiến độ và kết quả phiên lau.

### 1.2. File được chỉnh sửa tối thiểu
- `esp32_robot/esp32_robot.ino`:
  - Include `LaneTypes.h`, `CoveragePlanner.h`, `LaneTracker.h`; thay `std::vector<Step> steps` bằng `CoveragePlan activePlan` và `LaneTracker laneTracker`.
  - Bổ sung cấu hình `cfgOverlap` (`10.0f`) và `cfgDirection` (`DIRECTION_AUTO`) lưu NVS Flash (`"overlap"`, `"dir"`).
  - `buildPlan()`: gọi `CoveragePlanner::generateBestPlan(req, activePlan)` và `laneTracker.beginSession(&activePlan)`.
  - `doCalibrate()`: kiểm tra kết quả `if (!buildPlan()) return;` thay cho kiểm tra cứng `roomW < 2*edge + 200`.
  - `runNav()`: sử dụng `s.purpose` (`APPROACH_START / CLEAN_LANE / CHANGE_LANE`) và `s.laneId`; cập nhật `LaneTracker` ở mọi điểm bắt đầu/tiến trình/hoàn thành/bị chặn/timeout; khi `CHANGE_LANE` hoặc `APPROACH_START` thất bại, dừng phiên (`RESULT_FAILED`) thay vì bỏ qua bước để lau sai vị trí làn.
  - `finishCleaning()`, `startCleaning()`, `stopAll()`, `setError()`: đồng bộ trạng thái phiên với `LaneTracker`.
  - `handleStatus()`, `handlePlan()`, `handleSettings()`, `loadSettings()`: mở rộng API tương thích ngược theo Step 13, thêm kiểm tra chặt chuỗi số trả `HTTP 400` (nâng buffer header `/api/plan` lên `640` byte tránh truncation).
  - `INDEX_HTML`: cập nhật phần hiển thị cài đặt (`overlap`, `direction`) và vẽ màu từng làn theo trạng thái từ `/api/plan` V2.

### 1.3. File bắt buộc giữ nguyên (KHÔNG chỉnh sửa)
- `arduino_robot/arduino_robot.ino`: **0 thay đổi** — xác nhận `git diff main -- arduino_robot/arduino_robot.ino` rỗng và SHA256 không đổi (`FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13`).

---

## 2. Xác nhận bảo toàn thành phần gốc trong `esp32_robot.ino`

| Thành phần gốc | Trạng thái | Ghi chú kiểm tra |
|---|---|---|
| Sơ đồ chân ESP32 (`16, 18, 19, 25, 26, 27, 32, 33`) | Giữ nguyên 100% | Không sửa bất kỳ `#define` hay mảng pin nào |
| Giao thức UART ESP32↔Uno (`38400 bps`: `M, W, T, C`) | Giữ nguyên 100% | Chỉ thay `steps.size()` bằng `activePlan.stepCount` ở dòng telemetry `T,...` |
| LiDAR LD14 parser (`230400 bps`, CRC-8, `acc/scan`) | Giữ nguyên 100% | Không chỉnh sửa |
| Siêu âm Non-blocking ISR + Median-of-3 | Giữ nguyên 100% | Không chỉnh sửa |
| Khớp tường `fitWall()` & Định vị `predictPose()` / `correctPose()` | Giữ nguyên 100% | Không chỉnh sửa |
| Motor control, Soft-Start `rampAxis()`, `emergencyStopMotors()` | Giữ nguyên 100% | Không chỉnh sửa |
| WiFi STA/AP, mDNS, ArduinoOTA, High-Z `setUnoFlashMode()` | Giữ nguyên 100% | Không chỉnh sửa |
| Các endpoint `/`, `/api/scan`, `/api/scan.bin`, `/api/cmd`, `/api/unomode` | Giữ nguyên 100% | Tương thích ngược hoàn toàn |

---

## 3. Rủi ro tích hợp và phạm vi không thực hiện

- **Loại bỏ chèn động `ROLE_RECOVER` và vòng viền `ROLE_PERIM` khỏi baseline:** Theo yêu cầu nghiêm ngặt của Phase 01 Level 1 (Step 10 & Step 04), không tự thêm hành vi vòng tránh hoặc chèn bước ngoài kế hoạch; sử dụng đúng cơ chế *Skip and Record* trên 3 mục đích bước (`APPROACH_START`, `CLEAN_LANE`, `CHANGE_LANE`).
- **Tác động bộ nhớ tĩnh:** `sizeof(CoveragePlan) = 4,696 bytes` (đặt trong `.bss`), cộng `LaneTracker` (`~88 bytes`), tổng tăng RAM tĩnh `~4.8 KB` (~1.5% SRAM 320 KB của ESP32), đồng thời loại bỏ hoàn toàn cấp phát heap động từ `std::vector<Step>` và `std::vector<float>`.
- **Các lỗi ngoài phạm vi (xem chi tiết trong `GD01_SOURCE_AUDIT.md`):** Không tự ý sửa lỗi an toàn phần cứng (quá tải L293D — **BLOCKER** kiểm thử vật lý), lỗi an toàn firmware gốc (OTA lock, lệnh `S`, bộ đếm `blockCount` chung, watchdog Uno).
