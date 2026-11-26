# GD01.3 – BÁO CÁO KHẮC PHỤC LỖI INDEPENDENT CODE REVIEW (BUG-01 .. BUG-09)

**Baseline:** `refactor/gd01-modular-firmware` @ `f584b67`  
**Branch sửa lỗi:** `fix/gd01-adaptive-zigzag-correctness`  
**Phạm vi:** Chỉ khắc phục lỗi thuộc `planning/` (`CoveragePlanner`, `LaneTracker`), `navigation/` (`Navigator`, `CleaningSession`), `web/ApiHandlers.cpp` (`snprintf` checks), và unit/integration tests. Không sửa `arduino_robot.ino`, không sửa sơ đồ chân, không sửa LiDAR/Ultrasonic/Localization/PWM/UART.

---

## BUG-01 – Chuyển làn sau khi `CLEAN_LANE` bị chặn vật cản hoặc timeout xoay

- **Trạng thái:** `Confirmed`
- **Root cause:** Khi `CLEAN_LANE` gặp vật cản giữa làn (`blocked`) hoặc timeout xoay (`PH_TURN > 15000 ms`), `Navigator.cpp` tăng `gRobot.stepIdx++` vô điều kiện, giả định ngầm robot đã ở điểm cuối làn cũ và tiếp tục chạy bước `CHANGE_LANE` sang làn tiếp theo từ vị trí sai trục dọc.
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/navigation/Navigator.cpp` (L75–L79, L143–L150).
- **Fix implemented:**
  - Trên `PH_TURN` timeout của `CLEAN_LANE`: ghi `onLaneFailed(...)`, dừng động cơ khẩn cấp và gọi `setError("Timeout xoay dau lan")` (không sang `CHANGE_LANE`, làn sau giữ `LANE_PENDING`).
  - Trên `blocked` của `CLEAN_LANE`: ghi `onLaneBlocked(...)` (`LANE_BLOCKED` hoặc `LANE_PARTIAL`). Nếu còn bước tiếp theo mà `!laneTracker.isPositionAtLaneEnd(s.laneId, px_mm, py_mm)` (robot không ở điểm cuối làn), dừng phiên có kiểm soát (`finishCleaning()` nếu đã có `completedLanes > 0 || partialLanes > 0` → `RESULT_PARTIAL`; ngược lại `setError(...)` → `RESULT_FAILED`), không chạy bước chuyển làn, không tự tìm đường vòng và không bổ sung hành vi lùi/xoay mới.
  - Bổ sung kiểm tra cả `X` và `Y` trước khi bắt đầu mọi `CLEAN_LANE` bằng `isPositionAtLaneStart()`.
- **Test evidence:** `tests/test_gd01_3_integration.cpp` — `T03` (vật cản giữa làn), `T04` (timeout xoay đầu làn), `T05` (`CHANGE_LANE` đúng 1 trục nhưng sai trục kia) đều **PASS**.
- **Remaining limitation:** Do chưa có thuật toán path-replanning/A* (ngoài phạm vi GD01.3), phiên sẽ dừng có kiểm soát khi gặp vật cản giữa làn thay vì tự tìm đường vòng.

---

## BUG-02 – Đánh dấu hoàn thành làn sai (False Lane Completion) ở bước ngắn `< 30 mm`

- **Trạng thái:** `Confirmed`
- **Root cause:** Trong `Navigator.cpp`, nhánh `!stepStarted` áp dụng chung điều kiện `fabsf(s.val - cur) < 30.0f` cho mọi `StepPurpose` và lập tức gọi `laneTracker.onLaneCompleted(s.laneId)` khi `s.purpose == CLEAN_LANE` — chỉ cần robot đứng gần tọa độ đích (nhưng chưa hề đi từ điểm xuất phát làn hoặc sai trục ngang) vẫn bị đánh dấu `LANE_COMPLETED`.
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/navigation/Navigator.cpp` (L35–L47).
- **Fix implemented:**
  - Tách biệt hoàn toàn nhánh khởi tạo `!stepStarted`: điều kiện bỏ qua bước ngắn `< 30.0f` chỉ áp dụng cho `APPROACH_START` và `CHANGE_LANE`.
  - Với `CLEAN_LANE`: bắt buộc xác minh vị trí bắt đầu trên cả `X` và `Y` (`isPositionAtLaneStart`: sai số dọc `<= LANE_START_ALONG_TOL_MM = 100 mm`, sai số ngang `<= LANE_CROSS_TRACK_MAX_MM = 80 mm`); nếu không đạt → `emergencyStopMotors()` + `setError("Vi tri bat dau lan khong hop le")`, không gọi `onLaneStarted` hay `onLaneCompleted`.
  - Trong `LaneTracker::onLaneCompleted(laneId)`: chỉ cho phép chuyển sang `LANE_COMPLETED` nếu làn đang ở trạng thái `LANE_RUNNING`.
- **Test evidence:** `tests/test_gd01_3_integration.cpp` — `T01`, `T02`, `T05` **PASS**.
- **Remaining limitation:** Phụ thuộc vào độ chính xác ước lượng pose `(px_mm, py_mm)` từ LiDAR + dead-reckoning của hệ thống gốc.

---

## BUG-03 – Thiếu kiểm tra sai lệch ngang (Cross-Track Validation)

- **Trạng thái:** `Confirmed`
- **Root cause:** `LaneTracker::updateLaneCovered()` chỉ chiếu tọa độ dọc theo trục làn, bỏ qua hoàn toàn sai lệch ngang so với tâm làn; `Navigator::runNav()` khi `reached` gọi thẳng `onLaneCompleted()` dù robot bị trôi lệch ngang khỏi làn.
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/planning/LaneTracker.cpp` (L70–L97), `esp32_robot/src/navigation/Navigator.cpp` (L91–L116).
- **Fix implemented:**
  - Khai báo ngưỡng cấu hình trong `esp32_robot/src/planning/LaneTracker.h`: `LANE_CROSS_TRACK_MAX_MM = 80.0f` (< nửa bề rộng chổi tiêu chuẩn `90 mm`), `LANE_START_ALONG_TOL_MM = 100.0f`, `LANE_END_ALONG_TOL_MM = 80.0f`.
  - `LaneTracker::updateLaneCovered()` tính `computeCrossTrackError()`; nếu vượt `LANE_CROSS_TRACK_MAX_MM`, bật cờ `currentLaneOffTrack_ = true` và **không cộng thêm `estimatedCoveredLengthMm`**.
  - Bổ sung `LaneTracker::completeLaneWithPose(laneId, curX, curY)`: chỉ đánh dấu `LANE_COMPLETED` khi không lệch ngang quá ngưỡng và đã phủ đủ chiều dài đến cuối làn; ngược lại đánh dấu `LANE_PARTIAL` hoặc `LANE_FAILED` (không bao giờ `COMPLETED`).
  - `Navigator.cpp` gọi `completeLaneWithPose()` khi `reached` trên `CLEAN_LANE`, và dừng phiên có kiểm soát nếu vị trí cuối bị lệch khi còn bước tiếp theo.
- **Test evidence:** `tests/test_gd01_3_integration.cpp` — `T06` (trôi ngang > 80 mm không bao giờ `COMPLETED`), `T07` & `T15` (đi đúng làn trên cả trục Y & X → `COMPLETED`) **PASS**.
- **Remaining limitation:** Chỉ xác minh theo pose ước lượng trong phần mềm (không khẳng định diện tích lau vật lý thực tế vì không có cảm biến độ ẩm/chổi lau).

---

## BUG-04 – Nguy cơ tràn Stack ESP32 trong `CoveragePlanner::generateBestPlan()`

- **Trạng thái:** `Confirmed` (**Nghiêm trọng — đã xác minh trực tiếp bằng `xtensa-esp32-elf-g++ -fstack-usage`**)
- **Root cause:** `CoveragePlanner::generateBestPlan()` cấp phát 2 biến cục bộ `CoveragePlan bestPlan;` và `CoveragePlan tmpCand;` ngay trên stack → đo thực tế trên Xtensa ESP32 đạt **`9,440 bytes`** cho riêng frame `generateBestPlan` (vượt `8,192 bytes` mặc định của `loopTask` trên Arduino-ESP32!).
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/planning/CoveragePlanner.cpp` (L392–L394).
- **Fix implemented:**
  - Viết hàm `evaluateCandidateMetrics()` đánh giá trực tiếp 8 candidate × 2 thứ tự tiếp cận chỉ bằng metadata gọn `CandidateEval` / `PlanMetrics` trên stack mà không cấp phát mảng `lanes[64]` hay `steps[160]`.
  - Sau khi chọn cấu hình tối ưu, gọi `buildCandidate()` duy nhất 1 lần trực tiếp vào `outPlan`; nếu build/integrity thất bại, `clearPlan(outPlan, err)` xóa sạch atomic.
  - Không dùng heap động, không thêm biến `static` mutable → planner **hoàn toàn REENTRANT**.
- **Test evidence:**
  - Đo thực tế bằng `xtensa-esp32-elf-g++ 15.2.0 -Os -fstack-usage`: `generateBestPlan` giảm từ **`9,440 bytes` → `128 bytes`** (call-chain tối đa `336 bytes`, giảm `96.5%`).
  - 44/44 unit tests + 18/18 benchmark scenarios khớp 100% với baseline; `T14` xác minh atomic invalidation khi thất bại.
- **Remaining limitation:** `buildCandidate()` vẫn dựng trực tiếp vào tham chiếu `outPlan` do người gọi cung cấp (trên firmware là `gRobot.activePlan` tĩnh trong DRAM).

---

## BUG-05 – Tràn số khi tính số làn (`rawRatio` → `uint32_t`) & vòng lặp `wrapAngle180`

- **Trạng thái:** `Confirmed`
- **Root cause:** `CoveragePlanner::buildCandidate()` ép `(uint32_t)ceilf(rawRatio) + 1u` trước khi kiểm tra `rawRatio` có vượt `MAX_LANES - 1` hay `UINT32_MAX` không (gây Undefined Behavior/wrap-around với `roomW = 1e12f`); đồng thời `wrapAngle180()` dùng vòng `while (deg > 180.0f) deg -= 360.0f;` có thể treo vô hạn nếu `|deg| >= 1e10f`.
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/planning/CoveragePlanner.cpp` (L13–L17, L204–L210).
- **Fix implemented:**
  - Đổi `wrapAngle180()` sang dùng `fmodf(deg, 360.0f)` chạy trong thời gian `O(1)` kèm check `std::isfinite()`.
  - Tách `computeLaneGrid()` kiểm tra `!isFiniteFloat(rawRatio)` và `rawRatio > (float)(MAX_LANES - 1u)` **trước** khi gọi `ceilf()` và ép kiểu `uint32_t`.
  - Bổ sung kiểm tra `std::isfinite` cho `2.0f * edgeMm`, `workW`, `workL`, `effSpacing`, `actualSpacing`, mọi quãng đường và `estimatedCost`.
- **Test evidence:** `tests/test_gd01_3_integration.cpp` — `T12`, `T13` (`1e12f`, `1e38f`, `1e-20f`, `psi = 1e12f`, `NaN`, `+Inf`) **PASS**.
- **Remaining limitation:** Độ chính xác số học vẫn theo chuẩn IEEE-754 `float` 32-bit.

---

## BUG-06 – `verifyPlanIntegrity()` chưa kiểm tra đủ tính hợp lệ của kế hoạch

- **Trạng thái:** `Confirmed`
- **Root cause:** `verifyPlanIntegrity()` bản gốc chỉ kiểm tra sơ bộ biên phòng và đếm `cleanSteps`/`shiftSteps`, chưa kiểm tra tọa độ hữu hạn (`NaN`/`Inf`), phạm vi enum, trục `CLEAN_LANE` khớp `chosenDirection`, target step khớp cuối làn, thứ tự xen kẽ `APPROACH_START → CLEAN_LANE ↔ CHANGE_LANE`, `CHANGE_LANE` khớp tọa độ làn sau, `actualSpacing <= effectiveSpacing`, và tính liên tục đường đi.
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/planning/CoveragePlanner.cpp` (L433–L502).
- **Fix implemented:** Nâng cấp `verifyPlanIntegrity()` kiểm tra đầy đủ toàn bộ 9 nhóm điều kiện (enum range, finite floats, spacing bounds, corner match, lane continuity, strict step ordering, step axis, target-to-lane matching, và mô phỏng liên tục `(modelX, modelY)` qua từng bước).
- **Test evidence:** `tests/test_plan_validation.cpp` (`D01..D07`) + `tests/test_gd01_3_integration.cpp` (`T11`, `T16`) **PASS**.
- **Remaining limitation:** Kiểm tra tính liên tục giả định mô hình hình học lý tưởng của phòng chữ nhật (không kiểm tra vật cản động trong phòng).

---

## BUG-07 – Hủy phiên lau trong `ST_CALIB` giữ `CleaningResult = RESULT_NONE`

- **Trạng thái:** `Confirmed`
- **Root cause:** `startCleaning()` gọi `laneTracker.reset()` (đặt `result_ = RESULT_NONE`) rồi vào `ST_CALIB`. Khi user gọi `stopAll()` trong `ST_CALIB`, `LaneTracker::onUserAborted()` kiểm tra `if (result_ == RESULT_RUNNING)` bị `false`, nên `result_` không chuyển sang `RESULT_ABORTED`.
- **File & dòng code liên quan (baseline `f584b67`):** `esp32_robot/src/navigation/CleaningSession.cpp` (L54, L75–L77), `esp32_robot/src/planning/LaneTracker.cpp` (L132–L147).
- **Fix implemented:**
  - Bổ sung `LaneTracker::onCalibrationStarted()` (reset dữ liệu cũ và đặt `result_ = RESULT_RUNNING`), gọi trong `startCleaning()`.
  - `LaneTracker::onUserAborted()` xử lý sạch cả khi `plan_ == nullptr` (trong `ST_CALIB`) lẫn khi đang ở `ST_RUN`, luôn chuyển sang `RESULT_ABORTED`.
  - Khi `stopAll()` gọi lúc `ST_IDLE`/`ST_DONE`/`ST_ERR`, không gọi `onUserAborted()` nên kết quả phiên vừa kết thúc được bảo toàn.
  - `onSessionFinished()` chỉ trả `RESULT_COMPLETED` khi `completedLanes == totalLanes`, trả `RESULT_PARTIAL` khi `completedLanes > 0 || partialLanes > 0`, và `RESULT_FAILED` khi `0` tiến độ.
- **Test evidence:** `tests/test_gd01_3_integration.cpp` — `T08` (hủy trong `CALIB`), `T09` (hủy trong `RUN`), `T10` (chưa hoàn thành), `T14` (reset sau lỗi) + `tests/test_progress.cpp` (`P01..P05`) **PASS**.
- **Remaining limitation:** Không có.

---

## BUG-08 – `handleCmd()` luôn trả HTTP `200 OK "ok"` ngay cả khi lệnh bị bỏ qua

- **Trạng thái:** `Confirmed (Hành vi firmware gốc — KHÔNG thay đổi contract trong GD01.3)`
- **Root cause:** Trong `ApiHandlers.cpp`, `handleCmd()` gọi `applyCmd()` (hàm `void`) và luôn trả `server.send(200, "text/plain", "ok")` kể cả khi `unoFlashMode == true`, lệnh không hợp lệ, hoặc gửi lệnh lái tay trong `ST_RUN`/`ST_CALIB`.
- **File & dòng code liên quan:** `esp32_robot/src/web/ApiHandlers.cpp` (L75–L80), `esp32_robot/src/control/CommandHandler.cpp` (L11–L24).
- **Fix implemented:** **Giữ nguyên 100% API contract gốc trong GD01.3** theo ràng buộc phạm vi để không phá vỡ Web UI hiện hành; đã tài liệu hóa rõ giới hạn và đề xuất định dạng JSON `accepted/rejected` cho GD02 trong `docs/GD02_MOBILE_API_CONTRACT.md`.
- **Test evidence:** `tests/test_gd01_3_integration.cpp` — `T17` xác minh `handleCmd()` giữ nguyên `200 "ok"`.
- **Remaining limitation:** Client hiện tại chưa phân biệt được lệnh được thực thi hay bị từ chối chỉ từ HTTP response của `/api/cmd` (sẽ chuẩn hóa ở GD02).

---

## BUG-09 – Thiếu namespace `std::isfinite` trong unit test & thiếu kiểm tra `snprintf` trong `ApiHandlers.cpp`

- **Trạng thái:** `Confirmed`
- **Root cause:**
  1. `tests/test_coverage_planner.cpp` (dòng 257) gọi `isfinite(...)` không có `std::` sau khi include `<cmath>` (có thể lỗi biên dịch trên một số chuẩn C++17 strict libstdc++/MSVC).
  2. `ApiHandlers.cpp` (`handleStatus`, `handleSettings`, `handlePlan`) không kiểm tra giá trị trả về của `snprintf` (`n < 0 || n >= sizeof(buf)`), có nguy cơ âm thầm trả JSON bị cắt cụt nếu buffer quá nhỏ.
- **File & dòng code liên quan (baseline `f584b67`):** `tests/test_coverage_planner.cpp` (L257), `esp32_robot/src/web/ApiHandlers.cpp` (L30–L49, L153–L210).
- **Fix implemented:**
  - Đổi `isfinite` → `std::isfinite` tại `tests/test_coverage_planner.cpp` (L257) và trong `CoveragePlanner.cpp` / `LaneTracker.cpp`.
  - Tăng buffer `handleStatus()` từ `680` → `768` bytes, tăng `reserve()` trong `handlePlan()` lên `640 + laneCount * 104 + 8`, và kiểm tra đầy đủ return value của `snprintf` ở cả `handleStatus`, `handleSettings`, `handlePlan` (header và từng lane item), trả HTTP `500` JSON lỗi nếu serialization thất bại thay vì trả JSON gãy.
- **Test evidence:** `tests/test_coverage_planner.cpp` biên dịch sạch với `-std=c++17 -Wall -Wextra -Werror`; `tests/test_gd01_3_integration.cpp` — `T11` (64 làn) và `T17` (0 và 1 làn) **PASS**.
- **Remaining limitation:** Trên ESP32 thực tế, chuỗi `String j` cho 64 làn dùng ~6.6 KB heap tạm thời trong thời gian xử lý HTTP request (rất nhỏ so với DRAM khả dụng ~265 KB, nhưng nên xem xét streaming chunked response ở GD02 nếu bổ sung thêm nhiều trường metadata).
