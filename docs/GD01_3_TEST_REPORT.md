# GD01.3 – BÁO CÁO KIỂM THỬ & BENCHMARK (TEST & REGRESSION REPORT)

**Baseline so sánh:** `refactor/gd01-modular-firmware` @ `f584b67`  
**Branch kiểm thử:** `fix/gd01-adaptive-zigzag-correctness`  
**Trình biên dịch Host:** Clang `22.1.8` (`zig c++`, `-std=c++17 -O2 -Wall -Wextra -Werror`)

---

## 1. Tổng hợp kết quả Unit Tests & Integration Tests

| Bộ kiểm thử | Phạm vi | Số test | Passed | Failed | Trạng thái |
|---|---|---:|---:|---:|---|
| `tests/test_coverage_planner.cpp` | Groups A (Geometry), B (Dir/Corner), C (Overlap) | 25 | 25 | 0 | **PASS** |
| `tests/test_plan_validation.cpp` | Group D (Path Integrity & Validation) | 7 | 7 | 0 | **PASS** |
| `tests/test_lane_tracker.cpp` | Group E1 (Lane Status & Blocked Handling) | 7 | 7 | 0 | **PASS** |
| `tests/test_progress.cpp` | Group E2 (Progress & CleaningResult) | 5 | 5 | 0 | **PASS** |
| **Tổng Baseline Unit Tests** | **Groups A – E** | **44** | **44** | **0** | **100% PASS (0 Regression)** |
| `tests/test_gd01_3_integration.cpp` | Navigator + LaneTracker + CleaningSession + Planner + API (`T01..T17`) | 17 | 17 | 0 | **100% PASS** |
| **TỔNG CỘNG TỰ ĐỘNG** | **All Suites** | **61** | **61** | **0** | **100% PASS** |

---

## 2. Chi tiết 17 kịch bản Integration & Correctness mới (`tests/test_gd01_3_integration.cpp`)

Bộ test chạy thực thi trực tiếp các file `.cpp` thật của firmware (`Navigator.cpp`, `CleaningSession.cpp`, `CoveragePlanner.cpp`, `LaneTracker.cpp`, `MotionController.cpp`, `CommandHandler.cpp`, `ObstacleDetector.cpp`, `ApiHandlers.cpp`, `MathUtils.cpp`, `RobotContext.cpp`) với mock thời gian `millis()`, động học, cảm biến và `WebServer`:

| ID | Kịch bản kiểm thử | Kết quả |
|---|---|---|
| `T01` | `CLEAN_LANE` bắt đầu sai tọa độ ngang (`+120 mm > 80 mm`) → dừng phiên `ST_ERR` / `RESULT_FAILED`, làn 0 giữ `PENDING` | **PASS** |
| `T02` | Robot đặt ngay gần đích làn (`10 mm < 30 mm`) nhưng chưa ở điểm đầu → không `COMPLETED`, dừng phiên `ST_ERR` / `RESULT_FAILED` | **PASS** |
| `T03` | Robot gặp vật cản giữa làn (`us[0]=40 mm` × 3 ticks) → làn 0 = `PARTIAL`, dừng phiên có kiểm soát `ST_DONE` / `RESULT_PARTIAL`, làn 1 giữ `PENDING` | **PASS** |
| `T04` | Robot timeout khi xoay đầu làn (`+15.5 s`) → làn 0 = `FAILED`, dừng phiên `ST_ERR` / `RESULT_FAILED`, làn 1 giữ `PENDING` | **PASS** |
| `T05` | `CHANGE_LANE` đúng trục `X` nhưng sai trục `Y` (`-350 mm`) → bị chặn khi vào `CLEAN_LANE(1)`, `ST_ERR` / `RESULT_FAILED`, làn 1 giữ `PENDING` | **PASS** |
| `T06` | Robot đi lệch làn ngang (`+3 mm/tick` vượt `80 mm`) → không bao giờ đánh dấu `LANE_COMPLETED`, dừng phiên | **PASS** |
| `T07` | Robot bám đúng tâm làn tới hết các làn (hướng `Y`) → toàn bộ `LANE_COMPLETED`, `ST_DONE`, `RESULT_COMPLETED`, `100%` | **PASS** |
| `T08` | Người dùng dừng (`stopAll`) trong `ST_CALIB` → `ST_IDLE`, `RESULT_ABORTED` | **PASS** |
| `T09` | Người dùng dừng (`stopAll`) trong `ST_RUN` sau làn 0 + giữa làn 1 → `ST_IDLE`, `RESULT_ABORTED`, làn 0 `COMPLETED`, làn 1 `PARTIAL` | **PASS** |
| `T10` | Phiên 1 làn bị vật cản giữa làn cuối → `ST_DONE`, `RESULT_PARTIAL` (không `COMPLETED`) | **PASS** |
| `T11` | Kế hoạch đúng `64` làn (`MAX_LANES`) → `verifyPlanIntegrity` PASS & `handlePlan()` trả HTTP `200` JSON đầy đủ `id: 0..63` | **PASS** |
| `T12` | Kế hoạch yêu cầu `> 64` làn → `PLAN_ERR_EXCEEDS_MAX_LANES`, `valid = false` | **PASS** |
| `T13` | Đầu vào số học cực lớn/nhỏ/NaN/Inf (`1e12f`, `1e38f`, `1e-20f`, `psi = 1e12f`, `NaN`, `+Inf`) → không tràn/treo, từ chối an toàn | **PASS** |
| `T14` | `buildPlan()` thất bại sau phiên thành công → xóa sạch kế hoạch cũ (`valid=false`, `laneCount=0`, `stepCount=0`), `ST_ERR` / `RESULT_FAILED` | **PASS** |
| `T15` | Điều hướng đầy đủ hướng `DIRECTION_X` qua `Navigator` + `LaneTracker` → hoàn thành toàn bộ làn, `RESULT_COMPLETED` | **PASS** |
| `T16` | `verifyPlanIntegrity()` từ chối kế hoạch hỏng (enum sai, `NaN`, trục sai, target sai, `CHANGE_LANE` lệch, vượt `effectiveSpacing`) | **PASS** |
| `T17` | `handlePlan()` trả JSON hợp lệ cho `0` làn và `1` làn; `handleCmd()` bảo toàn contract gốc HTTP `200 "ok"` | **PASS** |

---

## 3. So sánh Benchmark Regression (18 kịch bản V2 + 18 dòng V1)

So sánh tự động toàn bộ 36 dòng bảng benchmark (`Room A..F` × `Overlap 0% / 10% / 20%` × `V1 / V2`) giữa baseline `f584b67` và bản sau sửa `GD01.3` (không dùng thời gian thực thi PC làm điều kiện xác minh thời gian vật lý):

| Chỉ số hình học & chi phí | Baseline (`f584b67`) | Post-GD01.3 | Sai lệch |
|---|---|---|---|
| Hướng lau (`Dir`) | 18/18 | 18/18 | **0** |
| Góc xuất phát (`Corner`) | 18/18 | 18/18 | **0** |
| Số làn (`Lanes`) | 18/18 | 18/18 | **0** |
| Khoảng cách làn (`Space(mm)`) | 18/18 | 18/18 | **0.0 mm** |
| Quãng đường lau / chuyển / tiếp cận / tổng | 18/18 | 18/18 | **0.00 m** |
| Số lần xoay (`Turns`) & Tổng góc (`TurnDeg`) | 18/18 | 18/18 | **0** |
| Chi phí kế hoạch (`Cost(s)`) | 18/18 | 18/18 | **0.0 s** |

> Lưu ý: Benchmark chỉ đánh giá hiệu năng thuật toán lập đường trên máy tính; không tuyên bố cải thiện thời gian lau vật lý thực tế khi chưa kiểm thử trên robot thật.

---

## 4. Xác minh tính bất biến ngoài phạm vi

- **`arduino_robot/arduino_robot.ino` SHA256:** `FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13` (trùng khớp 100% baseline).
- **Kiểm thử phần cứng thực tế:** `NOT RUN` (không nạp firmware, không kích hoạt động cơ trong GD01.3).
