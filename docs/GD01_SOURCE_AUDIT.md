# GD01 – BÁO CÁO KIỂM TOÁN MÃ NGUỒN (SOURCE CODE AUDIT)

**Dự án:** Smart Cleaning Robot – Phase 01: Adaptive Zigzag Coverage Path Planning V2 (Level 1)  
**Phiên bản baseline được kiểm toán:**
- `esp32_robot/esp32_robot.ino` (1187 dòng, commit baseline `a3ce321`)
- `arduino_robot/arduino_robot.ino` (192 dòng, commit baseline `a3ce321`, SHA256: `FEFAD2E529C71808A9F89EE74400DDD4EC524481BA2ECC7BC921BC17D46F5A13`)

---

## 1. Kiến trúc và chức năng đang tồn tại

### 1.1. Phân vai Dual-MCU (Giữ nguyên 100%)
- **ESP32 (Master):** Đọc LiDAR LD14 (UART2 `230400 bps`, GPIO16), đọc 3 cảm biến siêu âm HC-SR04 non-blocking bằng ngắt ngoài + lọc trung vị 3 mẫu (TRIG `25/27/32`, ECHO `26/14/33`), khớp 4 tường phẳng bằng PCA/TLS (`fitWall()`), định vị Dead-Reckoning RK2 + lọc bổ sung (`predictPose()`, `correctPose()`), đo phòng (`doCalibrate()`), điều hướng (`runNav()`), ramp PWM bảo vệ cầu H (`rampAxis()`), WebServer + mDNS (`robot.local`) + OTA + NVS `Preferences`, chế độ High-Z nạp Uno (`setUnoFlashMode()`), liên kết UART1 `38400 bps` (RX18/TX19) tới Arduino.
- **Arduino Uno (Slave — Bắt buộc giữ nguyên không sửa đổi):** Nhận lệnh `M,<L>,<R>\n`, `W,<mop>\n`, `T,<text>\n` qua Hardware Serial `38400 bps`, điều khiển 4 kênh `AF_DCMotor` (HW-130 / L293D), cầu nối Bluetooth HC-05 qua `SoftwareSerial(A0, A1)` với ring-buffer TX 64 byte, Watchdog `400 ms`.

### 1.2. Quy ước hệ tọa độ và hướng quay (đã xác minh từ mã nguồn)
- Gốc tọa độ `(0, 0)` nằm tại góc **sau-trái (`BOTTOM_LEFT`)** của phòng.
- Trục **`+X`**: sang phải, phạm vi `[0, roomW]` (`roomW = left + right`).
- Trục **`+Y`**: hướng trước mặt robot lúc bắt đầu hiệu chỉnh, phạm vi `[0, roomL]` (`roomL = front + back`).
- Góc hướng **`psi`** (độ, chuẩn hóa `(-180, 180]`):
  - `0°` = `+Y` (tiến lên)
  - `90°` = `+X` (sang phải)
  - `180°` = `-Y` (đi xuống)
  - `-90°` / `270°` = `-X` (sang trái)
  - Thuận chiều kim đồng hồ.
- Tọa độ 4 góc vùng làm việc an toàn (đã xác nhận khớp hoàn toàn quy ước trong đặc tả):
  - `CORNER_BOTTOM_LEFT  = (edge, edge)`
  - `CORNER_BOTTOM_RIGHT = (roomW - edge, edge)`
  - `CORNER_TOP_LEFT     = (edge, roomL - edge)`
  - `CORNER_TOP_RIGHT    = (roomW - edge, roomL - edge)`

### 1.3. Thuật toán chia làn hiện tại (`buildPlan()`, L585–L642)
- Đã có nâng cấp sơ bộ (chọn `planAxisY = (roomL >= roomW)`, chồng lấn cố định `LANE_OVERLAP_RATIO = 0.15f`, chọn 1 góc gần nhất bằng so sánh với trung điểm, sinh `steps` gồm `ROLE_HOME`, `ROLE_SHIFT`, `ROLE_LANE`, `ROLE_PERIM`, và chèn động `ROLE_RECOVER`).
- **Chuyển đổi sang `GO_X` / `GO_Y` và thực thi trong `runNav()` (L745–L855):**
  - Khi `!stepStarted`: bỏ qua bước nếu `|s.val - cur| < 30.0f`; tính `tgtHeading` (`90/270` cho `GO_X`, `0/180` cho `GO_Y`); đặt `phase = PH_TURN`, `stepStartMs = millis()`, `blockCount = 0`.
  - `PH_TURN`: P-controller `KT = 3.0`, kết thúc khi `|err| < 3.0°`, timeout `15000 ms`.
  - `PH_DRIVE`: P-controller bám hướng `KH = 2.5`, giảm tốc `SLOW_PWM` khi còn `< 300 mm`, kết thúc khi `reached` hoặc `blocked` (`blockCount >= 3`).
  - `PH_BACKOFF`: lùi `450 ms` khi bị chặn ở `ROLE_SHIFT` lần đầu (`stepRetries == 0`).
- **Tất cả các nhánh tăng `stepIdx` trong baseline:**
  1. `L751`: Khoảng cách đích `< 30.0f` mm khi khởi tạo bước.
  2. `L778`: Timeout pha quay `PH_TURN > 15000 ms`.
  3. `L802`: Đạt tọa độ đích (`reached == true`).
  4. `L835`: Bị chặn (`blocked == true`) sau khi không retry.
- **Lưu trữ cấu hình hiện tại (`loadSettings()` / `handleSettings()`, L986–L1024):**
  - Namespace NVS `"robot"`: `"mop"` (`float`), `"cruise"` (`int`), `"edge"` (`int`), `"perim"` (`bool`).
- **API hiện tại:**
  - `/api/status`: trả `state, x, y, psi, W, L, step, total, us0, us1, us2, mop, oL, oR, uno, err`.
  - `/api/plan`: trả `axis, lanes (mảng số toạ độ 1D), edge, perim`.
  - `/api/settings`: đọc/lưu `mop, cruise, edge, perim`.
  - `/api/scan`, `/api/scan.bin`, `/api/cmd`, `/api/unomode`.

---

## 2. Hạn chế của thuật toán chia làn hiện tại

1. **Không đánh giá đa phương án bằng hàm chi phí định lượng:** Chỉ dùng heuristic cứng `roomL >= roomW` và 1 góc gần trung điểm; chưa tính đến góc hướng ban đầu `psi`, tổng góc xoay, chi phí tiếp cận theo 2 thứ tự (`X→Y` vs `Y→X`), và phạt chuyển làn.
2. **Không hỗ trợ cấu hình chế độ hướng lau (`AUTO / X / Y`) và độ chồng lấn động (`0–30%`):** Chồng lấn đang hardcode `15%`, không có chế độ `QUICK / STANDARD / DEEP / CUSTOM`.
3. **Thiếu mô hình dữ liệu làn độc lập (`CleaningLane`) và theo dõi tiến độ (`LaneTracker`):**
   - Tiến độ trên UI hiện tính bằng `stepIdx / steps.size()` — gộp lẫn bước homing, chuyển làn và bước bị bỏ qua thành "hoàn thành".
   - Không phân biệt trạng thái từng làn (`PENDING / RUNNING / COMPLETED / BLOCKED / PARTIAL / FAILED`).
   - Không tính `processedPercent`, `completedPercent`, `estimatedDistancePercent`.
4. **Thiếu kết quả phiên lau (`CleaningResult`):** `finishCleaning()` luôn chuyển sang `ST_DONE` và in `"Hoan thanh!"` kể cả khi nhiều làn bị chặn hoặc timeout.
5. **Xử lý khi chuyển làn thất bại chưa an toàn cho kế hoạch:** Khi bước chuyển làn (`CHANGE_LANE`) timeout hoặc bị chặn sau retry, code cũ vẫn tăng `stepIdx++` và tiếp tục chạy bước `CLEAN_LANE` kế tiếp ở sai vị trí làn!
6. **Chưa kiểm tra đầy đủ đầu vào bất thường (`NaN`, `Inf`, phòng hẹp 1 làn suy biến, vượt giới hạn bộ nhớ, xây dựng kế hoạch không atomic).**
7. **Thuật toán gắn chặt trong file `.ino`:** Không tách được thành module C++ độc lập với phần cứng nên không thể chạy unit test và benchmark tự động trên máy tính.

---

## 3. Thành phần cần nâng cấp vs. Thành phần phải giữ nguyên

### 3.1. Thành phần nâng cấp (trong phạm vi Phase 01)
- Tách module C++ độc lập phần cứng đặt chung thư mục sketch `esp32_robot/`:
  - `LaneTypes.h`: Định nghĩa kiểu dữ liệu chuẩn (`CleaningDirection`, `StartCorner`, `LaneStatus`, `StepPurpose`, `CleaningResult`, `PlanMetrics`, `CleaningLane`, `Step`, `CleaningProgress`, `PlanConfig`, `PlanValidationError`, `CoveragePlan`).
  - `CoveragePlanner.h` / `CoveragePlanner.cpp`: Sinh tối thiểu 8 candidate (2 hướng × 4 góc × đánh giá thứ tự tiếp cận X→Y / Y→X), hàm chi phí theo đơn vị thời gian nhất quán, tính làn theo overlap `0–30%`, kiểm tra đầu vào toàn diện (`NaN`, `Inf`, suy biến 1 làn, vượt giới hạn bộ nhớ), xây dựng atomic vào buffer tạm.
  - `LaneTracker.h` / `LaneTracker.cpp`: Quản lý máy trạng thái từng làn (`PENDING → RUNNING → COMPLETED / BLOCKED / PARTIAL / FAILED`), tính `CleaningProgress` không chia 0, xác định `CleaningResult` (`NONE / RUNNING / COMPLETED / PARTIAL / ABORTED / FAILED`).
- Điểm tích hợp tối thiểu trong `esp32_robot/esp32_robot.ino`:
  - `buildPlan()` ủy quyền sang `CoveragePlanner`.
  - `runNav()` cập nhật `LaneTracker` theo kết quả thực tế của từng bước (`APPROACH_START`, `CLEAN_LANE`, `CHANGE_LANE`); khi `CHANGE_LANE` thất bại (blocked/timeout), dừng phiên an toàn (`RESULT_FAILED` / `RESULT_PARTIAL`) thay vì chạy tiếp ở sai làn; bỏ nhánh chèn động `ROLE_RECOVER` ngoài phạm vi Level 1 để tuân thủ chặt chẽ cơ chế Skip-and-Record ở Step 10.
  - Mở rộng `/api/status`, `/api/plan`, `/api/settings` (bổ sung `overlap`, `direction` với validate chặt) mà vẫn giữ tương thích ngược với Web UI và trường JSON cũ.

### 3.2. Thành phần bắt buộc giữ nguyên (KHÔNG chỉnh sửa)
- `arduino_robot/arduino_robot.ino`: Giữ nguyên 100% (cùng SHA256).
- Sơ đồ chân ESP32 (`16, 18, 19, 25, 26, 27, 32, 33`) và Arduino Uno (`0, 1, A0, A1, M1..M4, 13`).
- Giao thức UART (`M,l,r`, `W,p`, `T,...`, `C,k` ở `38400 bps`) và baud rate LiDAR (`230400 bps`).
- Toàn bộ thuật toán LiDAR (`initCrc`, `calcCrc`, `handlePacket`, `pollLidar`), Siêu âm (`isrUs*`, `pollUltrasonic`), Khớp tường (`fitWall`), Định vị (`predictPose`, `correctPose`), Điều khiển PWM/Soft-Start (`drive`, `rampAxis`), WiFi/mDNS/OTA/High-Z Uno mode.

---

## 4. Phân tích rủi ro tích hợp

1. **Giả định `GO_X` là bước chuyển làn:** Đã kiểm tra toàn bộ `esp32_robot.ino` — không còn chỗ nào dùng `s.type == GO_X` để nhận diện chuyển làn; chuyển toàn bộ sang `s.purpose == CHANGE_LANE` và `s.purpose == CLEAN_LANE`.
2. **Tương thích ngược `/api/plan` với Web UI:** Web UI cũ đọc `lanes` dạng mảng số `[210, 353, ...]`. Bản nâng cấp trả `lanes` dạng mảng object `{id, start_x, start_y, end_x, end_y, status}` đúng đặc tả Step 13, đồng thời bổ sung trường `lane_coords` (tương thích ngược) và cập nhật `drawMap()` trên Web UI để vẽ đúng từng làn kèm màu theo trạng thái (`PENDING / RUNNING / COMPLETED / BLOCKED / PARTIAL / FAILED`).
3. **Tương thích NVS Flash cũ:** Các khóa cũ (`mop`, `cruise`, `edge`) được giữ nguyên tên và kiểu dữ liệu; khóa mới (`overlap`, `dir`) dùng giá trị mặc định an toàn (`10.0f`, `DIRECTION_AUTO`) nếu chưa tồn tại trong Flash.
4. **Giới hạn RAM trên ESP32:** Không giữ đồng thời 8 danh sách bước; mỗi candidate chỉ xây dựng vào struct tạm hoặc tính trực tiếp trên 1 buffer `CoveragePlan` ứng viên rồi chỉ giữ lại bản tốt nhất (`bestPlan`). Với `MAX_LANES = 64` và `MAX_PLAN_STEPS = 160` (đủ cho phòng rộng tới `~10 m` với chổi `180 mm` ở overlap `20%`), tổng RAM tĩnh của 1 `CoveragePlan` chỉ `~3.8 KB` (rất nhỏ so với 320 KB SRAM của ESP32).

---

## 5. Các lỗi ngoài phạm vi Phase 01 (Chỉ ghi nhận, KHÔNG tự ý sửa)

### 5.1. Blocker đối với kiểm thử phần cứng thực tế
- **[HW-BLOCKER-01] Quá tải IC cầu H L293D (HW-130):** Mỗi kênh M1/M2 kéo song song 3 motor giảm tốc cho hệ skid-steer 6 bánh, dòng kéo khi quay tại chỗ có thể đạt 2–4 A, vượt xa định mức liên tục 0.6 A (đỉnh 1.2 A) của L293D. **Bắt buộc đo dòng tải và nâng cấp driver công suất trước khi chạy tải thật kéo dài.**
- **[HW-BLOCKER-02] Không có cảm biến chống rơi cầu thang (Cliff sensor) và nút E-Stop cắt nguồn cứng:** Chỉ thử nghiệm trên mặt sàn phẳng có giám sát trực tiếp.
- **[HW-BLOCKER-03] Cầu phân áp 5V → 3.3V:** Cần kiểm tra vật lý trên mạch đã hàn cho 3 chân ECHO và Arduino TX → GPIO18.

### 5.2. Lỗi an toàn / thuật toán gốc ngoài phạm vi Phase 01
- Bộ đếm `blockCount` dùng chung giữa LiDAR và siêu âm trước (`fresh || usTick`).
- Lệnh `S` không thoát trạng thái `ST_RUN`; không có trạng thái khóa `ST_ESTOP` và `ST_OTA`.
- Điều khiển thủ công vẫn hoạt động trong `ST_ERR` và chỉ kiểm tra siêu âm trước khi tiến.
- Vùng quét vật cản phía trước `±15°` hẹp hơn bề rộng robot `340 mm` và khoảng hở dừng `25 mm` nhỏ.
- Pha `PH_BACKOFF` không kiểm tra vật cản phía sau; `PH_TURN` không kiểm tra va chạm quanh thân; `PH_DRIVE` chưa có timeout riêng; quay lại `PH_TURN` không reset `stepStartMs`; `ST_CALIB` chưa có deadline khi mất hẳn LiDAR.
- Siêu âm timeout và đứt dây đều trả về `9999`.
- Watchdog trên Arduino Uno dùng chung cho motor và chổi; UART chưa có CRC/ACK; `SoftwareSerial` có rủi ro gây UART overrun; credentials WiFi/OTA đang hardcode plaintext.
- Định vị phụ thuộc phòng chữ nhật 4 tường phẳng, không có Encoder/IMU, có nguy cơ khóa nhầm trục `90°` của `wrap90`.
