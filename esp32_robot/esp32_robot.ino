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
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <math.h>
#include <stdlib.h>
#include "src/planning/LaneTypes.h"
#include "src/planning/CoveragePlanner.h"
#include "src/planning/LaneTracker.h"
#include "src/config/HardwareConfig.h"
#include "src/config/RobotConfig.h"
#include "src/config/NetworkConfig.h"
#include "src/config/Settings.h"
#include "src/core/RobotTypes.h"
#include "src/core/RobotContext.h"
#include "src/core/MathUtils.h"
#include "src/sensors/Lidar.h"
#include "src/sensors/Ultrasonic.h"
#include "src/sensors/ObstacleDetector.h"
#include "src/localization/WallDetection.h"
#include "src/localization/PoseEstimator.h"
#include "src/localization/Calibration.h"
#include "src/control/MotionController.h"
#include "src/control/CommandHandler.h"
#include "src/navigation/CleaningSession.h"
#include "src/navigation/Navigator.h"
#include "src/web/WebPage.h"

// ======================= BIẾN TOÀN CỤC =======================
WebServer server(80);

// Tham chiếu tạm tới RobotContext trong quá trình tách dần từng module (không tạo bản sao)
static State& state = gRobot.state;
static CoveragePlan& activePlan = gRobot.activePlan;
static LaneTracker& laneTracker = gRobot.laneTracker;
static int& stepIdx = gRobot.stepIdx;
static float& px_mm = gRobot.px_mm;
static float& py_mm = gRobot.py_mm;
static float& psi = gRobot.psi;
static float& roomW = gRobot.roomW;
static float& roomL = gRobot.roomL;
static bool& haveRoom = gRobot.haveRoom;
static uint16_t (&scan)[360] = gRobot.scan;
static volatile bool& newScan = gRobot.newScan;
static unsigned long& lastScanMs = gRobot.lastScanMs;
static uint16_t (&us)[3] = gRobot.us;
static bool& newUsFront = gRobot.newUsFront;
static int& cmdL = gRobot.cmdL;
static int& cmdR = gRobot.cmdR;
static int& outL = gRobot.outL;
static int& outR = gRobot.outR;
static int& mopPwm = gRobot.mopPwm;
static unsigned long& manualUntil = gRobot.manualUntil;
static int& manualL = gRobot.manualL;
static int& manualR = gRobot.manualR;
static bool& unoFlashMode = gRobot.unoFlashMode;
static String& errMsg = gRobot.errMsg;
static float& cfgMopWidth = gRobot.cfgMopWidth;
static int& cfgCruise = gRobot.cfgCruise;
static int& cfgEdge = gRobot.cfgEdge;
static float& cfgOverlap = gRobot.cfgOverlap;
static CleaningDirection& cfgDirection = gRobot.cfgDirection;
static String& myIP = gRobot.myIP;

// Liên kết UART
int lastSentMop = -1;
unsigned long lastSendMs = 0, lastMopMs = 0, lastTelMs = 0;

// ======================= CHẾ ĐỘ NHẢ CHÂN NẠP UNO =======================
void arduinoLinkSendImmediateStop() {
  if (!unoFlashMode) {
    Serial1.print("M,0,0\n");
    lastSendMs = millis();
  }
}
void setUnoFlashMode(bool en) {
  if (en == unoFlashMode) return;
  if (en) {
    cmdL = cmdR = outL = outR = 0;
    mopPwm = 0;
    manualUntil = 0;
    Serial1.print("M,0,0\nW,0\n");
    Serial1.flush();
    delay(20);
    Serial1.end();
    pinMode(LINK_TX_PIN, INPUT);
    pinMode(LINK_RX_PIN, INPUT);
    unoFlashMode = true;
    Serial.println("UNO FLASH MODE: ON (GPIO18/19 = High-Z)");
  } else {
    Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    unoFlashMode = false;
    lastSentMop = -1;
    Serial.println("UNO FLASH MODE: OFF (UART1 resumed)");
  }
}

// ======================= WEB SERVER =======================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  const char* names[] = {"IDLE", "CALIB", "RUN", "DONE", "ERR"};
  String safeErr = errMsg;
  safeErr.replace("\"", "'");
  CleaningProgress pr = laneTracker.getProgress();
  const char* dirStr = activePlan.valid ? directionToStr(activePlan.chosenDirection)
                                        : directionToStr(cfgDirection);
  const char* corStr = activePlan.valid ? cornerToStr(activePlan.startCorner) : "NONE";

  char b[680];
  snprintf(b, sizeof(b),
           "{\"state\":\"%s\",\"x\":%d,\"y\":%d,\"psi\":%d,\"W\":%d,\"L\":%d,"
           "\"step\":%d,\"total\":%d,\"us0\":%d,\"us1\":%d,\"us2\":%d,"
           "\"mop\":%d,\"oL\":%d,\"oR\":%d,\"uno\":%d,\"err\":\"%s\","
           "\"cleaning_result\":\"%s\",\"cleaning_direction\":\"%s\",\"start_corner\":\"%s\","
           "\"total_lanes\":%u,\"current_lane_id\":%d,\"completed_lanes\":%u,"
           "\"blocked_lanes\":%u,\"partial_lanes\":%u,\"failed_lanes\":%u,\"pending_lanes\":%u,"
           "\"processed_percent\":%.2f,\"completed_percent\":%.2f,\"estimated_distance_percent\":%.2f}",
           names[state], (int)px_mm, (int)py_mm, (int)psi,
           haveRoom ? (int)roomW : 0, haveRoom ? (int)roomL : 0,
           stepIdx, (int)activePlan.stepCount, us[0], us[1], us[2],
           mopPwm, outL, outR, unoFlashMode ? 1 : 0, safeErr.c_str(),
           cleaningResultToStr(laneTracker.getResult()), dirStr, corStr,
           (unsigned)pr.totalLanes, (int)pr.currentLaneId, (unsigned)pr.completedLanes,
           (unsigned)pr.blockedLanes, (unsigned)pr.partialLanes, (unsigned)pr.failedLanes,
           (unsigned)pr.pendingLanes,
           pr.processedPercent, pr.completedPercent, pr.estimatedDistancePercent);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

void handleScan() {
  String s;
  s.reserve(2200);
  s += '[';
  for (int i = 0; i < 360; i++) {
    if (i) s += ',';
    s += scan[i];
  }
  s += ']';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// Trả 720 byte nhị phân (360 x uint16_t little-endian) cho Mobile App (Android Studio)
void handleScanBin() {
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(sizeof(scan));
  server.send(200, "application/octet-stream", "");
  server.client().write((const uint8_t*)scan, sizeof(scan));
}

void handleCmd() {
  int v = server.hasArg("v") ? server.arg("v").toInt() : cfgCruise;
  applyCmd(server.arg("c"), v);
  server.send(200, "text/plain", "ok");
}

void handleUnoMode() {
  if (server.hasArg("en")) {
    bool en = (server.arg("en").toInt() != 0);
    if (en && (state == ST_RUN || state == ST_CALIB)) {
      server.send(409, "text/plain", "busy");
      return;
    }
    setUnoFlashMode(en);
  }
  server.send(200, "text/plain", unoFlashMode ? "1" : "0");
}

// Lệnh từ HC-05 (Arduino chuyển tiếp dạng "C,<ký tự>\n")
void pollLink() {
  if (unoFlashMode) return;
  static char line[24];
  static int n = 0;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      line[n] = 0;
      if (n >= 3 && line[0] == 'C' && line[1] == ',') {
        char k = line[2];
        if      (k == 'A') applyCmd("start", cfgCruise);
        else if (k == 'X') applyCmd("stop", cfgCruise);
        else if (k == 'F') applyCmd("F", cfgCruise);
        else if (k == 'B') applyCmd("B", cfgCruise);
        else if (k == 'L') applyCmd("L", cfgCruise);
        else if (k == 'R') applyCmd("R", cfgCruise);
        else if (k == 'S') applyCmd("S", cfgCruise);
        else if (k == 'W') applyCmd("mopon", cfgCruise);
        else if (k == 'w') applyCmd("mopoff", cfgCruise);
      }
      n = 0;
    } else if (c != '\r' && n < (int)sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

// ---- Cài đặt lưu trong Flash (đã tách sang Settings) ----
void handleSettings() {
  bool hasAny = server.hasArg("mop") || server.hasArg("cruise") ||
                server.hasArg("edge") || server.hasArg("overlap") ||
                server.hasArg("direction");
  if (hasAny) {
    if (state == ST_RUN || state == ST_CALIB) {
      server.send(409, "text/plain", "busy");
      return;
    }

    float newMop = cfgMopWidth;
    int   newCruise = cfgCruise;
    int   newEdge = cfgEdge;
    float newOverlap = cfgOverlap;
    CleaningDirection newDir = cfgDirection;

    if (server.hasArg("mop")) {
      if (!parseStrictFloat(server.arg("mop"), newMop) || newMop < 100.0f || newMop > 400.0f) {
        server.send(400, "text/plain", "invalid mop (100..400)");
        return;
      }
    }
    if (server.hasArg("cruise")) {
      if (!parseStrictInt(server.arg("cruise"), newCruise) || newCruise < 100 || newCruise > MAX_SAFE_PWM) {
        server.send(400, "text/plain", "invalid cruise");
        return;
      }
    }
    if (server.hasArg("edge")) {
      if (!parseStrictInt(server.arg("edge"), newEdge) || newEdge < ROBOT_RADIUS_MM + 10 || newEdge > 500) {
        server.send(400, "text/plain", "invalid edge");
        return;
      }
    }
    if (server.hasArg("overlap")) {
      if (!parseStrictFloat(server.arg("overlap"), newOverlap) ||
          newOverlap < MIN_OVERLAP_PERCENT || newOverlap > MAX_OVERLAP_PERCENT) {
        server.send(400, "text/plain", "invalid overlap (0..30)");
        return;
      }
    }
    if (server.hasArg("direction")) {
      if (!parseDirectionArg(server.arg("direction"), newDir)) {
        server.send(400, "text/plain", "invalid direction (AUTO|X|Y)");
        return;
      }
    }

    cfgMopWidth  = newMop;
    cfgCruise    = newCruise;
    cfgEdge      = newEdge;
    cfgOverlap   = newOverlap;
    cfgDirection = newDir;

    saveSettings();
  }

  char b[160];
  snprintf(b, sizeof(b),
           "{\"mop\":%d,\"cruise\":%d,\"edge\":%d,\"overlap\":%.1f,\"direction\":\"%s\"}",
           (int)cfgMopWidth, cfgCruise, cfgEdge, cfgOverlap, directionToStr(cfgDirection));
  server.send(200, "application/json", b);
}

void handlePlan() {
  String j;
  j.reserve(640 + activePlan.laneCount * 96);

  char hdr[640];
  snprintf(hdr, sizeof(hdr),
           "{\"planner_version\":\"2.0\",\"valid\":%s,\"direction\":\"%s\",\"axis\":\"%s\","
           "\"start_corner\":\"%s\",\"room_width_mm\":%.0f,\"room_length_mm\":%.0f,"
           "\"mop_width_mm\":%.0f,\"edge_mm\":%.0f,\"edge\":%.0f,\"overlap_percent\":%.1f,"
           "\"lane_spacing_mm\":%.2f,\"lane_count\":%u,"
           "\"metrics\":{\"cleaning_distance_mm\":%.1f,\"transition_distance_mm\":%.1f,"
           "\"approach_distance_mm\":%.1f,\"total_distance_mm\":%.1f,"
           "\"turn_count\":%u,\"total_turn_degrees\":%.1f,\"estimated_cost\":%.2f},\"lanes\":[",
           activePlan.valid ? "true" : "false",
           activePlan.valid ? directionToStr(activePlan.chosenDirection) : directionToStr(cfgDirection),
           activePlan.valid ? directionToStr(activePlan.chosenDirection) : directionToStr(cfgDirection),
           activePlan.valid ? cornerToStr(activePlan.startCorner) : "NONE",
           activePlan.valid ? activePlan.roomW : 0.0f,
           activePlan.valid ? activePlan.roomL : 0.0f,
           activePlan.valid ? activePlan.mopWidthMm : cfgMopWidth,
           activePlan.valid ? activePlan.edgeMm : (float)cfgEdge,
           activePlan.valid ? activePlan.edgeMm : (float)cfgEdge,
           activePlan.valid ? activePlan.overlapPercent : cfgOverlap,
           activePlan.valid ? activePlan.actualSpacingMm : 0.0f,
           activePlan.valid ? (unsigned)activePlan.laneCount : 0u,
           activePlan.metrics.cleaningDistanceMm,
           activePlan.metrics.transitionDistanceMm,
           activePlan.metrics.approachDistanceMm,
           activePlan.metrics.totalDistanceMm,
           (unsigned)activePlan.metrics.turnCount,
           activePlan.metrics.totalTurnDegrees,
           activePlan.metrics.estimatedCost);
  j += hdr;

  if (activePlan.valid) {
    char item[128];
    for (uint16_t i = 0; i < activePlan.laneCount; i++) {
      const CleaningLane& ln = activePlan.lanes[i];
      if (i > 0) j += ',';
      snprintf(item, sizeof(item),
               "{\"id\":%u,\"start_x\":%.0f,\"start_y\":%.0f,\"end_x\":%.0f,\"end_y\":%.0f,\"status\":\"%s\"}",
               (unsigned)ln.id, ln.startX, ln.startY, ln.endX, ln.endY, laneStatusToStr(ln.status));
      j += item;
    }
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

void startWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Dang ket noi WiFi");
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(250); Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    myIP = WiFi.localIP().toString();
    Serial.println("Da ket noi WiFi nha.");
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    myIP = WiFi.softAPIP().toString();
    Serial.printf("Khong vao duoc WiFi nha -> phat WiFi rieng '%s' (mat khau %s)\n", AP_SSID, AP_PASS);
  }
  Serial.println("==============================================");
  Serial.println("Mo trinh duyet:  http://" + myIP);
  Serial.println("hoac:            http://robot.local");
  Serial.println("==============================================");
  if (MDNS.begin("robot")) MDNS.addService("http", "tcp", 80);
}

void sendLink() {
  if (unoFlashMode) return;
  unsigned long now = millis();

  // Cập nhật PWM thực tế qua bộ ramp mềm và gửi ở 20 Hz (50 ms)
  if (now - lastSendMs >= 50) {
    lastSendMs = now;
    outL = rampAxis(outL, cmdL);
    outR = rampAxis(outR, cmdR);
    Serial1.printf("M,%d,%d\n", outL, outR);
  }

  // Gửi lệnh chổi ngay khi thay đổi trạng thái hoặc mỗi 300 ms
  if (mopPwm != lastSentMop || now - lastMopMs >= 300) {
    lastMopMs = now;
    lastSentMop = mopPwm;
    Serial1.printf("W,%d\n", mopPwm);
  }

  // Telemetry gọn (< 32 byte, gửi mỗi 2s lệch pha với gói M để tránh nghẽn SoftwareSerial của Uno)
  if (now - lastTelMs >= 2000 && now - lastSendMs >= 20) {
    lastTelMs = now;
    const char* nm[] = {"IDLE", "CAL", "RUN", "DONE", "ERR"};
    Serial1.printf("T,%s %d,%d %da %d/%d\n",
                   nm[state], (int)px_mm, (int)py_mm, (int)psi, stepIdx, (int)activePlan.stepCount);
  }
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  loadSettings();

  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  lidarBeginSerial();
  ultrasonicInit();
  lidarInit();

  startWiFi();
  server.on("/", handleRoot);
  server.on("/api/status", handleStatus);
  server.on("/api/scan", handleScan);
  server.on("/api/scan.bin", handleScanBin);
  server.on("/api/cmd", handleCmd);
  server.on("/api/settings", handleSettings);
  server.on("/api/plan", handlePlan);
  server.on("/api/unomode", handleUnoMode);
  server.begin();

  // Nạp code qua WiFi (Arduino IDE -> Tools -> Port -> robot)
  ArduinoOTA.setHostname("robot");
  ArduinoOTA.setPassword("robot1234");
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.onStart([]() {
    cmdL = cmdR = outL = outR = 0;
    mopPwm = 0;
    if (!unoFlashMode) {
      Serial1.print("M,0,0\nW,0\n");
      Serial1.flush();
    }
  });
  ArduinoOTA.begin();

  poseInitTimer();
  lastScanMs = millis();
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  pollLink();
  pollLidar();
  pollUltrasonic();
  predictPose();

  bool fresh = newScan;
  if (fresh) newScan = false;
  bool usTick = newUsFront;
  if (usTick) newUsFront = false;

  switch (state) {
    case ST_IDLE: case ST_DONE: case ST_ERR:
      if (!unoFlashMode && millis() < manualUntil) {
        int l = manualL, r = manualR;
        if (l > 0 && r > 0 && us[0] < US_BLOCK_MM) {
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
      if (millis() - lastScanMs > 1000) {
        emergencyStopMotors();
        if (millis() - lastScanMs > 5000) setError("Mat LiDAR");
        break;
      }
      if (fresh) correctPose();
      runNav(fresh || usTick);
      break;
  }

  sendLink();
}
