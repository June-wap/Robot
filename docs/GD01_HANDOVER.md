# GD01 – TÀI LIỆU BÀN GIAO (HANDOVER)

## 1. Cấu trúc bàn giao dự án

Thư mục dự án: `SmartCleaningRobot/` (nhánh Git: `feature/gd01-adaptive-zigzag`)

```text
SmartCleaningRobot/
├── esp32_robot/
│   ├── esp32_robot.ino        # Firmware ESP32 tích hợp Planner V2 + LaneTracker + API V2
│   ├── LaneTypes.h            # Kiểu dữ liệu & giới hạn tĩnh (MAX_LANES=64, MAX_PLAN_STEPS=160)
│   ├── CoveragePlanner.h      # Khai báo Adaptive Zigzag Coverage Planner V2
│   ├── CoveragePlanner.cpp    # Cài đặt Planner V2 (C++ độc lập phần cứng)
│   ├── LaneTracker.h          # Khai báo quản lý trạng thái làn & tiến độ
│   └── LaneTracker.cpp        # Cài đặt LaneTracker & CleaningResult
├── arduino_robot/
│   └── arduino_robot.ino      # Giữ nguyên 100% không thay đổi
├── tests/
│   ├── test_coverage_planner.cpp  # Groups A, B, C + Step 17 Benchmark
│   ├── test_plan_validation.cpp   # Group D (Path Integrity)
│   ├── test_lane_tracker.cpp      # Group E1 (Lane Status & Blocked Handling)
│   └── test_progress.cpp          # Group E2 (Progress & CleaningResult)
└── docs/
    ├── GD01_SOURCE_AUDIT.md
    ├── GD01_ALGORITHM_DESIGN.md
    ├── GD01_INTEGRATION_REPORT.md
    ├── GD01_TEST_REPORT.md
    └── GD01_HANDOVER.md
```

## 2. Hướng dẫn biên dịch và nạp firmware (Arduino IDE)

1. Mở trực tiếp file `SmartCleaningRobot/esp32_robot/esp32_robot.ino` bằng Arduino IDE (Arduino IDE tự động nạp 5 file `.h/.cpp` cùng thư mục sketch).
2. Chọn Board: **ESP32 Dev Module** (esp32 core 2.x hoặc 3.x).
3. Kiểm tra không thay đổi gì ở `arduino_robot/arduino_robot.ino` — không cần nạp lại Arduino Uno.
4. Nhấn **Verify** để kiểm tra dung lượng Flash/RAM cho chip ESP32, sau đó chỉ Upload (qua USB hoặc OTA `robot` / `robot1234`) khi đã có người giám sát trực tiếp và đảm bảo điều kiện an toàn phần cứng bên dưới.

## 3. Lưu ý an toàn bắt buộc trước khi kiểm thử trên robot thật

> [!WARNING]
> **Blocker phần cứng tồn tại từ hệ thống gốc (ngoài phạm vi Phase 01):**
> 1. **HW-130 / L293D quá tải dòng:** Mỗi kênh M1/M2 kéo 3 motor giảm tốc song song (dòng kéo khi quay có thể vượt 0.6 A định mức). Đo dòng thực tế và đặt robot trên đế nâng bánh không tải để thử nghiệm luồng API/kế hoạch trước khi đặt xuống sàn.
> 2. **Không có cảm biến chống rơi & E-Stop phần cứng:** Chỉ chạy thử trên mặt sàn phẳng có người giám sát và sẵn sàng ngắt nguồn.
> 3. Các lỗi an toàn firmware gốc (xem mục 5 của `GD01_SOURCE_AUDIT.md`) chưa được sửa trong Phase 01 do ràng buộc phạm vi — cần triển khai ở giai đoạn Safety riêng.

## 4. Hướng dẫn tích hợp Mobile App (Android Studio) trong tương lai

Firmware ESP32 đã sẵn sàng toàn bộ endpoint cho Mobile App kết nối qua WiFi (mDNS `robot.local` hoặc AP `192.168.4.1`):

1. **Polling trạng thái & tiến độ (`500 ms`):** `GET /api/status`
   - Đọc `state`, `x`, `y`, `psi`, `W`, `L`, `cleaning_result`, `cleaning_direction`, `start_corner`, `total_lanes`, `current_lane_id`, `completed_lanes`, `blocked_lanes`, `partial_lanes`, `failed_lanes`, `pending_lanes`, `processed_percent`, `completed_percent`, `estimated_distance_percent`.
2. **Vẽ bản đồ làn có màu theo trạng thái (`1–2 s` hoặc khi đổi `current_lane_id`):** `GET /api/plan`
   - Đọc `lanes[]` gồm `{id, start_x, start_y, end_x, end_y, status}` và `metrics`.
3. **Vẽ Radar LiDAR tốc độ cao:** `GET /api/scan.bin`
   - Nhận đúng `720 byte` nhị phân (`360 × uint16_t` little-endian, mm) thay cho `/api/scan` JSON để tối ưu độ mượt 60 FPS trên Android Canvas.
4. **Cấu hình chia làn:** `GET /api/settings?mop=180&cruise=165&edge=210&overlap=10&direction=AUTO`
   - Trả `200 OK` + JSON cấu hình hiện hành, `400 Bad Request` nếu giá trị sai định dạng/phạm vi, `409 Conflict` nếu robot đang chạy.
