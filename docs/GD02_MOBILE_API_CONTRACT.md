# GD02 – ĐẶC TẢ HỢP ĐỒNG API CHO MOBILE APP (MOBILE API CONTRACT)

Tài liệu này đóng băng hợp đồng HTTP REST API hiện hành sau **GD01.3** để đội phát triển Mobile App (Android Studio / GD02) tích hợp trực tiếp, đồng thời ghi nhận giới hạn của `/api/cmd` hiện tại và đề xuất phiên bản nâng cấp cho GD02.

- **Base URL:** `http://robot.local` (mDNS STA) hoặc `http://192.168.4.1` (fallback AP `RobotLauNha`), cổng `80`.
- **Header chung trên các endpoint động:** `Cache-Control: no-store`.

---

## 1. Danh sách Endpoint hiện hành (Frozen Contract – GD01.3)

### 1.1. `GET /api/status`
- **Tần suất khuyến nghị:** `500 ms`
- **HTTP Status:** `200 OK` (`application/json`), hoặc `500 Internal Server Error` (`{"error":"status_serialization_failed"}`) nếu lỗi tuần tự hóa.
- **Schema JSON:**
  ```json
  {
    "state": "IDLE|CALIB|RUN|DONE|ERR",
    "x": 210,
    "y": 210,
    "psi": 0,
    "W": 3000,
    "L": 5000,
    "step": 0,
    "total": 33,
    "us0": 9999,
    "us1": 9999,
    "us2": 9999,
    "mop": 200,
    "oL": 0,
    "oR": 0,
    "uno": 0,
    "err": "",
    "cleaning_result": "NONE|RUNNING|COMPLETED|PARTIAL|ABORTED|FAILED",
    "cleaning_direction": "AUTO|X|Y",
    "start_corner": "NONE|BOTTOM_LEFT|BOTTOM_RIGHT|TOP_LEFT|TOP_RIGHT",
    "total_lanes": 17,
    "current_lane_id": 0,
    "completed_lanes": 0,
    "blocked_lanes": 0,
    "partial_lanes": 0,
    "failed_lanes": 0,
    "pending_lanes": 17,
    "processed_percent": 0.00,
    "completed_percent": 0.00,
    "estimated_distance_percent": 0.00
  }
  ```
- **Lưu ý ngữ nghĩa sau GD01.3:**
  - Hủy trong `CALIB` hoặc `RUN` đều trả `cleaning_result: "ABORTED"`.
  - Làn đi lệch ngang vượt `LANE_CROSS_TRACK_MAX_MM` (`80 mm`) không bao giờ được tính vào `completed_lanes`.
  - `estimated_distance_percent` là tỷ lệ ước lượng theo pose phần mềm, không phải diện tích lau vật lý đo thực tế.

### 1.2. `GET /api/plan`
- **Tần suất khuyến nghị:** `1000–1500 ms` khi đang chạy hoặc khi `current_lane_id` thay đổi.
- **HTTP Status:** `200 OK` (`application/json`), hoặc `500` nếu lỗi serialization. Hỗ trợ đầy đủ từ `0`, `1` đến `64` (`MAX_LANES`) làn.
- **Schema JSON:**
  ```json
  {
    "planner_version": "2.0",
    "valid": true,
    "direction": "Y",
    "axis": "Y",
    "start_corner": "BOTTOM_LEFT",
    "room_width_mm": 3000,
    "room_length_mm": 5000,
    "mop_width_mm": 180,
    "edge_mm": 210,
    "edge": 210,
    "overlap_percent": 10.0,
    "lane_spacing_mm": 161.25,
    "lane_count": 17,
    "metrics": {
      "cleaning_distance_mm": 77860.0,
      "transition_distance_mm": 2580.0,
      "approach_distance_mm": 0.0,
      "total_distance_mm": 80440.0,
      "turn_count": 33,
      "total_turn_degrees": 2970.0,
      "estimated_cost": 460.12
    },
    "lanes": [
      { "id": 0, "start_x": 210, "start_y": 210, "end_x": 210, "end_y": 4790, "status": "PENDING" }
    ]
  }
  ```
- **Trạng thái `lanes[].status`:** `PENDING`, `RUNNING`, `COMPLETED`, `BLOCKED`, `PARTIAL`, `FAILED`.

### 1.3. `GET /api/scan.bin` & `GET /api/scan`
- **`GET /api/scan.bin` (khuyến nghị cho Mobile App):** Trả `200 OK` (`application/octet-stream`), cố định đúng **`720 bytes`** (`360 × uint16_t` little-endian, đơn vị `mm`, góc `0..359°` thuận chiều kim đồng hồ).
- **`GET /api/scan`:** Trả `200 OK` (`application/json`), mảng JSON 360 số nguyên `mm`.

### 1.4. `GET /api/settings`
- **Đọc:** `GET /api/settings` → `200 OK` JSON `{"mop":180,"cruise":165,"edge":210,"overlap":10.0,"direction":"AUTO"}`.
- **Ghi (lưu NVS `"robot"`):** `GET /api/settings?mop=180&cruise=165&edge=210&overlap=10&direction=AUTO`
  - Phạm vi hợp lệ: `mop` `[100..400]`, `cruise` `[100..210]`, `edge` `[180..500]`, `overlap` `[0..30]`, `direction` `AUTO|X|Y` (hoặc `0|1|2`).
  - **Status Codes:** `200 OK` (đã lưu + trả JSON hiện hành), `400 Bad Request` (sai kiểu/phạm vi), `409 Conflict` (`busy` nếu đang ở `ST_RUN` hoặc `ST_CALIB`).

### 1.5. `GET /api/unomode`
- `GET /api/unomode?en=1|0` → `200 OK` (`"1"` hoặc `"0"`), hoặc `409 Conflict` (`"busy"`) khi đang ở `ST_RUN` / `ST_CALIB`.

### 1.6. `GET /api/cmd`
- **Cú pháp hiện hành:** `GET /api/cmd?c=<start|stop|F|B|L|R|S|mopon|mopoff>[&v=80..210]`
- **Hành vi hiện hành (giữ nguyên từ firmware gốc):** Luôn trả `200 OK` (`text/plain`, body `"ok"`).

---

## 2. Giới hạn đã ghi nhận của `/api/cmd` (BUG-08) & Đề xuất cho GD02

### Giới hạn hiện tại
- `handleCmd()` gọi `applyCmd()` (không trả về trạng thái chấp nhận/từ chối) và luôn trả `HTTP 200 "ok"` ngay cả khi:
  - Đang bật `unoFlashMode == true` (mọi lệnh bị bỏ qua).
  - Gửi lệnh lái tay `F/B/L/R` hoặc `mopon/mopoff` khi robot đang ở `ST_RUN` / `ST_CALIB`.
  - Tham số `c` trống hoặc không nằm trong danh sách lệnh hỗ trợ.
- **Cách xử lý tạm thời phía Mobile App trên GD01.3:** Sau khi gọi `/api/cmd`, đọc lại `state`, `mop`, `uno` từ `/api/status` để đồng bộ giao diện thực tế.

### Đề xuất nâng cấp tương thích ngược cho GD02 (không triển khai trong GD01.3)
1. Bổ sung tham số tùy chọn `&fmt=json` hoặc phiên bản endpoint `/api/v2/cmd` (giữ `/api/cmd` cũ cho Web UI):
   - **Khi chấp nhận (`200 OK`):**
     ```json
     { "accepted": true, "cmd": "start", "state": "CALIB", "reason": "" }
     ```
   - **Khi từ chối do xung đột trạng thái (`409 Conflict`):**
     ```json
     { "accepted": false, "cmd": "F", "state": "RUN", "reason": "manual_disabled_while_running" }
     ```
   - **Khi lệnh không hợp lệ (`400 Bad Request`):**
     ```json
     { "accepted": false, "cmd": "XYZ", "state": "IDLE", "reason": "unknown_command" }
     ```
2. Bổ sung xác thực cục bộ (API token / session header) cho các lệnh điều khiển và cài đặt khi triển khai GD02.
