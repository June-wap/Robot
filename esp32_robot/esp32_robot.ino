/*
 * ROBOT LAU NHÀ TỰ ĐỘNG - ESP32 (BẢN TỐI ƯU: WiFi + HC-05 + OTA + Non-blocking US + Soft-Start)
 * ---------------------------------------------------------------------------------------------
 * Mở trình duyệt (điện thoại hoặc máy tính cùng WiFi) vào  http://<IP của ESP32>
 * (hoặc http://robot.local). IP được in ra Serial Monitor 115200 khi khởi động.
 * Nếu không vào được WiFi nhà, ESP32 tự phát WiFi riêng:
 *      Tên: RobotLauNha   Mật khẩu: 88888888   ->  http://192.168.4.1
 *
 * Trang web có: nút Bắt đầu / Dừng lau tự động, bật/tắt chổi, phím điều khiển tay,
 * thanh chỉnh tốc độ, radar LiDAR, bản đồ phòng có vệt di chuyển và chế độ nhả chân D0/D1
 * để nạp lại Arduino Uno qua USB mà không cần tháo mối hàn.
 *
 * API (có thể gọi từ app khác / Android Studio):
 *   GET /api/status                 -> JSON trạng thái
 *   GET /api/scan                   -> JSON 360 giá trị khoảng cách LiDAR (mm)
 *   GET /api/scan.bin               -> Binary 720 byte (360 x uint16_t LE) cho Mobile App
 *   GET /api/plan                   -> JSON trục làn, danh sách làn, edge, perim
 *   GET /api/settings[?mop=&cruise=&edge=&perim=] -> đọc / lưu cài đặt vào Flash
 *   GET /api/cmd?c=start|stop|F|B|L|R|S|mopon|mopoff[&v=80..210]
 *   GET /api/unomode?en=1|0         -> bật/tắt chế độ High-Z GPIO18/19 để nạp Uno qua USB
 *
 * Đấu dây ESP32 (giữ nguyên 100% phần cứng đã hàn):
 *   LD14 TX -> GPIO16 | VCC 5V | GND
 *   ESP32 GPIO19 (TX) -> Arduino RX (chân 0)
 *   ESP32 GPIO18 (RX) <- Arduino TX (chân 1) QUA CẦU PHÂN ÁP 5V->3.3V (1k/2k)
 *   GND chung. Arduino chuyển tiếp lệnh HC-05 sang ESP32 và gửi trạng thái lên điện thoại.
 *   HC-05 (nối với Arduino): A = lau tự động, X = dừng, F/B/L/R = điều khiển tay, W/w = chổi
 *   Siêu âm (ECHO qua cầu phân áp 1k/2k):
 *       Trước TRIG 25/ECHO 26 | Trái TRIG 27/ECHO 14 | Phải TRIG 32/ECHO 33
 *
 * Quy ước góc LiDAR: 0 = trước, tăng theo chiều kim đồng hồ.
 * Hệ toạ độ phòng: gốc ở góc sau-trái, +Y = hướng robot lúc bắt đầu, +X = bên phải.
 * psi: 0 = +Y, 90 = +X, thuận chiều kim đồng hồ.
 */

#include <Arduino.h>
#include "src/config/RobotConfig.h"
#include "src/config/Settings.h"
#include "src/core/RobotTypes.h"
#include "src/core/RobotContext.h"
#include "src/sensors/Lidar.h"
#include "src/sensors/Ultrasonic.h"
#include "src/localization/PoseEstimator.h"
#include "src/localization/Calibration.h"
#include "src/control/MotionController.h"
#include "src/navigation/CleaningSession.h"
#include "src/navigation/Navigator.h"
#include "src/communication/ArduinoLink.h"
#include "src/communication/WiFiManager.h"
#include "src/communication/OtaManager.h"
#include "src/web/RobotWebServer.h"

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  loadSettings();

  arduinoLinkInit();
  lidarBeginSerial();
  ultrasonicInit();
  lidarInit();

  startWiFi();
  webServerInit();

  otaInit();

  poseInitTimer();
  gRobot.lastScanMs = millis();
}

void loop() {
  webServerHandleClient();
  otaHandle();
  pollLink();
  pollLidar();
  pollUltrasonic();
  predictPose();

  bool fresh = gRobot.newScan;
  if (fresh) gRobot.newScan = false;
  bool usTick = gRobot.newUsFront;
  if (usTick) gRobot.newUsFront = false;

  switch (gRobot.state) {
    case ST_IDLE: case ST_DONE: case ST_ERR:
      if (!gRobot.unoFlashMode && millis() < gRobot.manualUntil) {
        int l = gRobot.manualL, r = gRobot.manualR;
        if (l > 0 && r > 0 && gRobot.us[0] < US_BLOCK_MM) {
          emergencyStopMotors();
        } else {
          drive(l, r);
        }
      } else {
        stopMotors();
      }
      break;

    case ST_CALIB:
      stopMotors();
      if (fresh) doCalibrate();
      break;

    case ST_RUN:
      if (millis() - gRobot.lastScanMs > 1000) {
        emergencyStopMotors();
        if (millis() - gRobot.lastScanMs > 5000) setError("Mat LiDAR");
        break;
      }
      if (fresh) correctPose();
      runNav(fresh || usTick);
      break;
  }

  sendLink();
}
