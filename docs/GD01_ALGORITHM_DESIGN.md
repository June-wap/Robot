# GD01 – TÀI LIỆU THIẾT KẾ THUẬT TOÁN ADAPTIVE ZIGZAG COVERAGE PLANNER V2 (LEVEL 1)

## 1. Mô tả thuật toán gốc (V1)
- Luôn cố định hướng lau theo trục `Y` (`GO_Y`) và phân bố làn theo trục `X` (`GO_X`).
- Luôn xuất phát cố định từ góc sau-trái `(edge, edge)` (`CORNER_BOTTOM_LEFT`), bất kể vị trí ban đầu `(px_mm, py_mm, psi)` của robot.
- Không đánh giá chi phí góc xoay hay quãng đường tiếp cận, không quản lý trạng thái từng làn (`CleaningLane`), không tính tiến độ theo làn (`CleaningProgress`), và luôn đánh dấu `ST_DONE` dù có làn bị chặn hoặc timeout.

## 2. Mô tả thuật toán nâng cấp (Adaptive Zigzag V2 – Level 1)
Thuật toán V2 được tách hoàn toàn thành module C++ độc lập phần cứng (`LaneTypes.h`, `CoveragePlanner.h/.cpp`, `LaneTracker.h/.cpp`), không dùng cấp phát heap động:
1. Kiểm tra chặt toàn bộ đầu vào (`NaN`, `Inf`, kích thước phòng, `mopWidth`, `edge >= robotRadius`, `overlap ∈ [0, 30]`, vị trí robot trong phòng, thông số động học).
2. Sinh tập phương án ứng viên (tối đa **8 candidate × 2 thứ tự tiếp cận = 16 phương án** với `DIRECTION_AUTO`, hoặc **4 candidate × 2 = 8 phương án** với `DIRECTION_X` / `DIRECTION_Y`).
3. Mô phỏng đường đi thực tế từ danh sách `Step` đã sinh để tính `PlanMetrics` và `EstimatedCost`.
4. Chọn phương án có chi phí thấp nhất với chuỗi tiêu chí hòa (tie-break) xác định tuyệt đối (**deterministic**).
5. Kiểm định toàn vẹn hình học (`verifyPlanIntegrity`) trước khi ghi đè atomic vào `activePlan`.
6. Theo dõi trạng thái từng làn (`LaneTracker`) theo sự kiện thực tế từ `runNav()` và xuất kết quả phiên lau (`CleaningResult`) + tiến độ (`CleaningProgress`).

## 3. Công thức chia làn và độ chồng lấn

### 3.1. Chế độ chồng lấn hình học
| Chế độ | Tỷ lệ chồng lấn | Ghi chú |
|---|---|---|
| `OVERLAP_QUICK` | `0%` | Bước hiệu dụng bằng `100% MopWidth` |
| `OVERLAP_STANDARD` | `10%` | Mặc định (`DEF_OVERLAP_PCT = 10.0f`) |
| `OVERLAP_DEEP` | `20%` | Giảm khe hở giữa hai làn liên tiếp |
| `OVERLAP_CUSTOM` | `0% – 30%` | Chỉnh từ `/api/settings?overlap=...` |

### 3.2. Công thức tính làn
Với chiều dài phòng trục lau $L_{\text{long}}$ và chiều rộng trục chia làn $L_{\text{cross}}$:
$$\text{LongSpan} = L_{\text{long}} - 2 \cdot E, \qquad \text{CrossSpan} = L_{\text{cross}} - 2 \cdot E$$
$$\text{EffectiveSpacing} = \text{MopWidth} \cdot \left(1 - \frac{\text{OverlapPercent}}{100}\right)$$

- Nếu $\text{LongSpan} < 50\text{ mm}$ hoặc $\text{CrossSpan} < 0$: từ chối lập kế hoạch (`PLAN_ERR_ROOM_TOO_SMALL`), **không** tự giảm $E$.
- Nếu $\text{CrossSpan} \le 10^{-3}\text{ mm}$ (hành lang hẹp đúng bằng $2E$): sinh đúng **1 làn hợp lệ** với `LaneCount = 1`, `ActualSpacing = 0` (tránh chia cho 0).
- Nếu $\text{CrossSpan} > 10^{-3}\text{ mm}$:
  $$\text{LaneCount} = \left\lceil \frac{\text{CrossSpan}}{\text{EffectiveSpacing}} \right\rceil + 1$$
  $$\text{ActualSpacing} = \frac{\text{CrossSpan}}{\text{LaneCount} - 1} \le \text{EffectiveSpacing}$$
- Giới hạn bộ nhớ tĩnh: `MAX_LANES = 64` (hỗ trợ phòng rộng tới `~8.35 m` với `MopWidth = 180 mm` ở overlap `30%`), `MAX_PLAN_STEPS = 160` (`2 + 64 + 63 = 129 ≤ 160`).

## 4. Phương pháp sinh 8 Candidate và chọn góc/hướng

### 4.1. Danh sách 8 Candidate cố định (Deterministic)
| Candidate | Hướng lau (`dir`) | Góc xuất phát (`corner`) | Tọa độ góc |
|---|---|---|---|
| C01 | `DIRECTION_X` | `CORNER_BOTTOM_LEFT` | `(E, E)` |
| C02 | `DIRECTION_X` | `CORNER_BOTTOM_RIGHT` | `(W - E, E)` |
| C03 | `DIRECTION_X` | `CORNER_TOP_LEFT` | `(E, L - E)` |
| C04 | `DIRECTION_X` | `CORNER_TOP_RIGHT` | `(W - E, L - E)` |
| C05 | `DIRECTION_Y` | `CORNER_BOTTOM_LEFT` | `(E, E)` |
| C06 | `DIRECTION_Y` | `CORNER_BOTTOM_RIGHT` | `(W - E, E)` |
| C07 | `DIRECTION_Y` | `CORNER_TOP_LEFT` | `(E, L - E)` |
| C08 | `DIRECTION_Y` | `CORNER_TOP_RIGHT` | `(W - E, L - E)` |

Mỗi candidate được đánh giá với **cả 2 thứ tự tiếp cận** từ `(robotX, robotY, robotPsiDeg)`:
- Thứ tự `X → Y` (`approachYFirst = false`)
- Thứ tự `Y → X` (`approachYFirst = true`)
Các bước tiếp cận có độ dài `< 1.0 mm` tự động bị loại bỏ (không sinh bước 0-length thừa).

### 4.2. Hàm chi phí (Đơn vị nhất quán: giây)
$$\text{EstimatedCost} = \frac{\text{TotalDistanceMm}}{V_{\text{nom}}} + \frac{\text{TotalTurnDegrees}}{\Omega_{\text{nom}}} + P_{\text{lane}} \cdot (\text{LaneCount} - 1)$$

Trong đó tái sử dụng thông số động học ước lượng có sẵn của firmware *(chỉ dùng cho xếp hạng phương án, không phải vận tốc đo thực tế)*:
- $V_{\text{nom}} = \frac{\text{cfgCruise}}{255} \cdot V_{\max} \approx 194.1\text{ mm/s}$ (ở `cfgCruise = 165`, `V_MAX = 300 mm/s`).
- $\Omega_{\text{nom}} = \frac{\text{MIN\_TURN\_PWM} + \text{MAX\_TURN\_PWM}}{510} \cdot \Omega_{\max} \approx 86.8\text{ °/s}$.
- $P_{\text{lane}} = 1.5\text{ s}$ (phạt chuyển làn bao gồm giảm tốc, dừng ổn định và tăng tốc mềm).

### 4.3. Quy trình chọn phương án tối ưu
1. Loại bỏ candidate không hợp lệ.
2. Chọn candidate có `estimatedCost` nhỏ nhất (`|Δcost| > 1e-3 s`).
3. Hòa chi phí → chọn `totalTurnDegrees` nhỏ hơn (`|Δangle| > 0.5°`).
4. Hòa góc xoay → chọn `approachDistanceMm` nhỏ hơn (`|Δapp| > 1.0 mm`).
5. Hòa tất cả → giữ candidate đứng trước theo thứ tự C01..C08 (100% deterministic).

## 5. Cấu trúc dữ liệu cốt lõi (`LaneTypes.h`)

- **`CleaningLane` (32 bytes):** `id`, `startX`, `startY`, `endX`, `endY`, `plannedLengthMm`, `estimatedCoveredLengthMm`, `status`.
- **`Step` (16 bytes):** `type` (`GO_X | GO_Y`), `val` (mm), `purpose` (`APPROACH_START | CLEAN_LANE | CHANGE_LANE`), `laneId` (`-1` nếu không phải `CLEAN_LANE`).
- **`CoveragePlan` (4,696 bytes tĩnh):** Lưu phương án tốt nhất, `PlanMetrics`, mảng `lanes[64]`, mảng `steps[160]`.

## 6. Sơ đồ hoạt động và chuyển trạng thái làn

```text
[doCalibrate() xong]
        │
        ▼
[CoveragePlanner::generateBestPlan(req, activePlan)]
   ├── Thất bại ──► activePlan.valid=false -> setError() -> ST_ERR (RESULT_FAILED)
   └── Thành công ─► laneTracker.beginSession(&activePlan) -> ST_RUN (RESULT_RUNNING)
                          │
                          ▼
               [runNav() duyệt từng Step]
                 ├── APPROACH_START: thất bại/vật cản -> RESULT_FAILED -> ST_ERR
                 ├── CHANGE_LANE:    retry 1 lần (PH_BACKOFF có sẵn);
                 │                   vẫn thất bại -> onChangeLaneFailed() -> ST_ERR
                 └── CLEAN_LANE:
                       PENDING ──(bắt đầu)──► RUNNING
                          ├── đạt đích (reached) ──────────► COMPLETED
                          ├── vật cản (đã đi >=15% & >=100mm) ► PARTIAL  (Skip & Record)
                          ├── vật cản (mới đầu làn) ───────► BLOCKED  (Skip & Record)
                          └── timeout xoay (>15s) ─────────► FAILED   (Skip & Record)
                          │
                          ▼ (hết activePlan.stepCount)
               [finishCleaning() -> onSessionFinished()]
                 ├── completedLanes == totalLanes ──► ST_DONE (RESULT_COMPLETED)
                 └── còn làn chưa hoàn thành ───────► ST_DONE (RESULT_PARTIAL)
```

## 7. Công thức tính tiến độ (`LaneTracker::getProgress`)
- `ProcessedLanes = completedLanes + blockedLanes + partialLanes + failedLanes`
- `ProcessedPercent = (totalLanes > 0) ? 100 * ProcessedLanes / totalLanes : 0`
- `CompletedPercent = (totalLanes > 0) ? 100 * completedLanes / totalLanes : 0`
- `EstimatedDistancePercent = (totalPlannedCleanMm > 0) ? 100 * sum(coveredMm) / totalPlannedCleanMm : 0`
- Tất cả phần trăm được kẹp trong `[0, 100]`, không chia cho 0, làn đang `RUNNING` không được tính vào `completed` hay `processed`.

## 8. API mở rộng (Tương thích ngược)
- **`GET /api/status`**: Giữ toàn bộ 16 trường cũ, bổ sung 13 trường mới (`cleaning_result`, `cleaning_direction`, `start_corner`, `total_lanes`, `current_lane_id`, `completed_lanes`, `blocked_lanes`, `partial_lanes`, `failed_lanes`, `pending_lanes`, `processed_percent`, `completed_percent`, `estimated_distance_percent`).
- **`GET /api/plan`**: Trả `planner_version: "2.0"`, `direction`, `axis`, `start_corner`, `room_width_mm`, `room_length_mm`, `mop_width_mm`, `edge_mm`, `edge`, `overlap_percent`, `lane_spacing_mm`, `lane_count`, `metrics`, và mảng `lanes: [{id, start_x, start_y, end_x, end_y, status}]`.
- **`GET /api/settings`**: Bổ sung `overlap` (`0..30`) và `direction` (`AUTO|X|Y`), kiểm tra chặt định dạng số trả `HTTP 400` nếu sai, `HTTP 409` nếu robot đang chạy, lưu vào Flash NVS (`"overlap"`, `"dir"`).

## 9. Giới hạn thuật toán (Level 1)
- Chỉ áp dụng cho phòng hình chữ nhật (dựa trên `roomW × roomL` từ `doCalibrate()`).
- Không có SLAM, Occupancy Grid, A\* vòng tránh vật cản hoặc tự tạo đường thay thế khi bước `CHANGE_LANE` bị chặn.
- `completedPercent` và `estimatedDistancePercent` dựa trên định vị ước lượng của firmware, không phải đo xác nhận vật lý độ sạch của sàn.

## 10. Hướng dẫn Build và Kiểm thử
- **Biên dịch firmware ESP32 bằng Arduino IDE:** Mở `esp32_robot/esp32_robot.ino` (Arduino IDE tự nạp các file `LaneTypes.h`, `CoveragePlanner.h/.cpp`, `LaneTracker.h/.cpp` nằm cùng thư mục), chọn board `ESP32 Dev Module`, bấm Verify/Upload.
- **Chạy Unit Test & Benchmark trên PC (Clang/GCC C++17):**
  ```bash
  g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_coverage_planner.cpp esp32_robot/CoveragePlanner.cpp esp32_robot/LaneTracker.cpp -o test_coverage_planner && ./test_coverage_planner
  g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_plan_validation.cpp esp32_robot/CoveragePlanner.cpp esp32_robot/LaneTracker.cpp -o test_plan_validation && ./test_plan_validation
  g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_lane_tracker.cpp esp32_robot/CoveragePlanner.cpp esp32_robot/LaneTracker.cpp -o test_lane_tracker && ./test_lane_tracker
  g++ -std=c++17 -O2 -Wall -Wextra -Werror tests/test_progress.cpp esp32_robot/CoveragePlanner.cpp esp32_robot/LaneTracker.cpp -o test_progress && ./test_progress
  ```
