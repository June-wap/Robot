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
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include "LaneTypes.h"
#include "CoveragePlanner.h"
#include "LaneTracker.h"
#include "src/config/HardwareConfig.h"
#include "src/config/RobotConfig.h"
#include "src/config/NetworkConfig.h"
#include "src/core/RobotTypes.h"
#include "src/core/MathUtils.h"
#include "src/web/WebPage.h"

// ======================= BIẾN TOÀN CỤC =======================
WebServer server(80);
State state = ST_IDLE;

CoveragePlan activePlan;
LaneTracker  laneTracker;
int  stepIdx = 0;
bool stepStarted = false;
Phase phase = PH_TURN;
float tgtHeading = 0;
unsigned long stepStartMs = 0;
unsigned long backoffUntilMs = 0;
uint8_t stepRetries = 0;

float px_mm = 0, py_mm = 0, psi = 0;
float roomW = 0, roomL = 0;
bool  haveRoom = false;
unsigned long lastPoseMs = 0;

// LiDAR
uint16_t acc[360];
uint16_t scan[360];
volatile bool newScan = false;
float lastStartAngle = 0;
unsigned long lastScanMs = 0;
uint8_t crcTable[256];

// Siêu âm Non-blocking (ngắt ngoài + lọc trung vị 3 mẫu)
volatile uint32_t usRiseUs[3]  = {0, 0, 0};
volatile uint32_t usWidthUs[3] = {0, 0, 0};
volatile bool     usHigh[3]    = {false, false, false};
volatile bool     usDone[3]    = {false, false, false};

uint16_t us[3] = {9999, 9999, 9999};
uint16_t usHist[3][3] = {{9999, 9999, 9999}, {9999, 9999, 9999}, {9999, 9999, 9999}};
uint8_t  usHistIdx[3] = {0, 0, 0};
int      usTurn = 0;
bool     usBusy = false;
unsigned long lastUsMs = 0;
unsigned long usTrigMs = 0;
bool     newUsFront = false;

// Điều khiển động cơ (target vs output thực tế qua soft-start)
int cmdL = 0, cmdR = 0;
int outL = 0, outR = 0;
int mopPwm = 0, lastSentMop = -1;
unsigned long lastSendMs = 0, lastMopMs = 0, lastTelMs = 0;
int blockCount = 0;

// Hiệu chỉnh phòng
int calibOk = 0, calibFail = 0;
float calibD[4] = {0, 0, 0, 0};
float calibSin = 0, calibCos = 0;

// Điều khiển tay
unsigned long manualUntil = 0;
int manualL = 0, manualR = 0;

// Chế độ nhả chân GPIO18/19 (High-Z) để nạp code Arduino Uno qua USB
bool unoFlashMode = false;

String errMsg = "";

// Cài đặt lưu trong Flash (NVS)
Preferences prefs;
float cfgMopWidth = DEF_MOP_WIDTH_MM;
int   cfgCruise   = DEF_CRUISE_PWM;
int   cfgEdge     = DEF_EDGE_MM;
float cfgOverlap  = DEF_OVERLAP_PCT;
CleaningDirection cfgDirection = DIRECTION_AUTO;
String myIP = "";

// ======================= ĐIỀU KHIỂN ĐỘNG CƠ =======================

void drive(int l, int r) {
  cmdL = clampi(l, -MAX_SAFE_PWM, MAX_SAFE_PWM);
  cmdR = clampi(r, -MAX_SAFE_PWM, MAX_SAFE_PWM);
}

void stopMotors() { cmdL = cmdR = 0; }

// Dừng khẩn cấp: đưa output về 0 ngay lập tức (dùng khi va chạm, stopAll, OTA)
void emergencyStopMotors() {
  cmdL = cmdR = 0;
  outL = outR = 0;
  if (!unoFlashMode) {
    Serial1.print("M,0,0\n");
    lastSendMs = millis();
  }
}

// Tăng/giảm tốc mềm & dead-time đảo chiều 50 ms để giảm dòng đỉnh trên L293D
int rampAxis(int cur, int tgt) {
  if (tgt == 0) {
    if (abs(cur) <= PWM_STOP_STEP) return 0;
    return cur + (cur > 0 ? -PWM_STOP_STEP : PWM_STOP_STEP);
  }
  if ((cur > 0 && tgt < 0) || (cur < 0 && tgt > 0)) {
    return 0;  // Ép qua 0 tối thiểu 1 chu kỳ (50 ms) trước khi đảo chiều
  }
  int diff = tgt - cur;
  if (diff > PWM_RAMP_STEP)  return cur + PWM_RAMP_STEP;
  if (diff < -PWM_RAMP_STEP) return cur - PWM_RAMP_STEP;
  return tgt;
}

// ======================= CHẾ ĐỘ NHẢ CHÂN NẠP UNO =======================
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

// ======================= LiDAR LD14 =======================
void initCrc() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
    crcTable[i] = c;
  }
}

uint8_t calcCrc(const uint8_t* p, int len) {
  uint8_t c = 0;
  for (int i = 0; i < len; i++) c = crcTable[(c ^ p[i]) & 0xFF];
  return c;
}

void handlePacket(const uint8_t* b) {
  float startA = (b[4] | (b[5] << 8)) / 100.0f;
  float endA   = (b[42] | (b[43] << 8)) / 100.0f;

  if (startA < lastStartAngle - 180.0f) {
    memcpy(scan, acc, sizeof(acc));
    memset(acc, 0, sizeof(acc));
    newScan = true;
    lastScanMs = millis();
  }
  lastStartAngle = startA;

  if (endA < startA) endA += 360.0f;
  float step = (endA - startA) / 11.0f;

  for (int i = 0; i < 12; i++) {
    uint16_t d = b[6 + 3 * i] | (b[7 + 3 * i] << 8);
    uint8_t inten = b[8 + 3 * i];
    if (d < LIDAR_MIN_MM || d > LIDAR_MAX_MM || inten < LIDAR_MIN_INTENS) continue;
    float ang = startA + step * i;
    int deg = ((int)(ang + 0.5f) + LIDAR_OFFSET_DEG) % 360;
    if (deg < 0) deg += 360;
    if (!LIDAR_CLOCKWISE) deg = (360 - deg) % 360;
    acc[deg] = d;
  }
}

void pollLidar() {
  static uint8_t buf[47];
  static int idx = 0;
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if (idx == 0) { if (b != 0x54) continue; }
    else if (idx == 1) { if (b != 0x2C) { idx = 0; continue; } }
    buf[idx++] = b;
    if (idx == 47) {
      idx = 0;
      if (calcCrc(buf, 46) == buf[46]) handlePacket(buf);
    }
  }
}

// ======================= KHỚP TƯỜNG (PCA / TLS) =======================
Wall fitWall(int center) {
  Wall w = {false, 0, 0};
  float X[64], Y[64];
  int n = 0;
  for (int a = -WALL_HALF_DEG; a <= WALL_HALF_DEG && n < 64; a++) {
    uint16_t d = scan[((center + a) % 360 + 360) % 360];
    if (!d) continue;
    float r = radians((float)(center + a));
    X[n] = d * sinf(r);
    Y[n] = d * cosf(r);
    n++;
  }
  if (n < 10) return w;
  float mx = 0, my = 0;
  for (int i = 0; i < n; i++) { mx += X[i]; my += Y[i]; }
  mx /= n; my /= n;
  float sxx = 0, syy = 0, sxy = 0;
  for (int i = 0; i < n; i++) {
    float dx = X[i] - mx, dy = Y[i] - my;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  float phi = 0.5f * atan2f(2.0f * sxy, sxx - syy);
  float nx = -sinf(phi), ny = cosf(phi);
  float res = 0;
  for (int i = 0; i < n; i++) {
    float e = (X[i] - mx) * nx + (Y[i] - my) * ny;
    res += e * e;
  }
  float rms = sqrtf(res / n);
  if (rms > 25.0f) return w;
  w.ok = true;
  w.dist = fabsf(mx * nx + my * ny);
  w.phi = degrees(phi);
  return w;
}

// ======================= ĐỊNH VỊ =======================
void predictPose() {
  unsigned long now = millis();
  float dt = (now - lastPoseMs) / 1000.0f;
  lastPoseMs = now;
  if (dt <= 0 || dt > 0.5f) return;
  // Dùng outL/outR (PWM thực tế sau ramp) để ước lượng động học sát hơn
  float v = (outL + outR) / 510.0f * V_MAX_MM_S;
  float w = (outL - outR) / 510.0f * SPIN_MAX_DEG_S;
  float mid = psi + 0.5f * w * dt;
  px_mm += v * dt * sinf(radians(mid));
  py_mm += v * dt * cosf(radians(mid));
  psi = wrap180(psi + w * dt);
}

void correctPose() {
  const int al[4] = {0, 90, 180, 270};
  Wall w[4];
  float sx = 0, sy = 0;
  int n = 0;
  for (int k = 0; k < 4; k++) {
    w[k] = fitWall(al[k]);
    if (w[k].ok) {
      float e = wrap90(w[k].phi - 90.0f);
      sx += sinf(radians(4.0f * e));
      sy += cosf(radians(4.0f * e));
      n++;
    }
  }
  if (n == 0) return;
  float eps = degrees(atan2f(sx, sy)) / 4.0f;
  float meas = psi + wrap90(eps - psi);
  psi = wrap180(psi + K_YAW * (meas - psi));

  if (!haveRoom) return;
  float psq = roundf(psi / 90.0f) * 90.0f;
  if (fabsf(psi - psq) > 12.0f) return;

  float xs = 0, ys = 0;
  int nx = 0, ny = 0;
  for (int k = 0; k < 4; k++) {
    if (!w[k].ok) continue;
    int q = ((al[k] + (int)psq) % 360 + 360) % 360;
    float d = w[k].dist;
    if      (q == 0)   { ys += roomL - d; ny++; }
    else if (q == 90)  { xs += roomW - d; nx++; }
    else if (q == 180) { ys += d;         ny++; }
    else               { xs += d;         nx++; }
  }
  if (nx) {
    float mxv = xs / nx;
    if (fabsf(mxv - px_mm) < POS_JUMP_MAX_MM) px_mm += K_POS * (mxv - px_mm);
  }
  if (ny) {
    float myv = ys / ny;
    if (fabsf(myv - py_mm) < POS_JUMP_MAX_MM) py_mm += K_POS * (myv - py_mm);
  }
}

// ======================= HIỆU CHỈNH BAN ĐẦU & LẬP ĐƯỜNG =======================
bool buildPlan();

void setError(const char* m) {
  errMsg = m;
  if (state == ST_RUN || state == ST_CALIB) {
    laneTracker.onSessionFailed(m);
  }
  state = ST_ERR;
  mopPwm = 0;
  emergencyStopMotors();
}

void doCalibrate() {
  const int al[4] = {0, 90, 180, 270};
  Wall w[4];
  bool all = true;
  for (int k = 0; k < 4; k++) { w[k] = fitWall(al[k]); if (!w[k].ok) all = false; }
  if (!all) {
    if (++calibFail > 60) setError("Khong thay du 4 buc tuong");
    return;
  }
  float sx = 0, sy = 0;
  for (int k = 0; k < 4; k++) {
    float e = wrap90(w[k].phi - 90.0f);
    sx += sinf(radians(4.0f * e)); sy += cosf(radians(4.0f * e));
    calibD[k] += w[k].dist;
  }
  calibSin += sx; calibCos += sy;
  if (++calibOk < 5) return;

  float eps = degrees(atan2f(calibSin, calibCos)) / 4.0f;
  float front = calibD[0] / 5.0f, right = calibD[1] / 5.0f, back = calibD[2] / 5.0f, left = calibD[3] / 5.0f;
  psi = eps;
  roomW = left + right;
  roomL = front + back;
  px_mm = left;
  py_mm = back;
  haveRoom = true;

  if (!buildPlan()) return;
  stepIdx = 0; stepStarted = false; stepRetries = 0;
  mopPwm = MOP_PWM;
  state = ST_RUN;
  Serial.printf("Phong: %.0f x %.0f mm | Huong %s | Goc %s | %u lan | %u buoc\n",
                roomW, roomL, directionToStr(activePlan.chosenDirection),
                cornerToStr(activePlan.startCorner), activePlan.laneCount, activePlan.stepCount);
}

bool buildPlan() {
  PlanRequest req;
  req.roomW                   = roomW;
  req.roomL                   = roomL;
  req.edgeMm                  = (float)cfgEdge;
  req.mopWidthMm              = cfgMopWidth;
  req.overlapPercent          = cfgOverlap;
  req.robotRadiusMm           = (float)ROBOT_RADIUS_MM;
  req.robotX                  = px_mm;
  req.robotY                  = py_mm;
  req.robotPsiDeg             = psi;
  req.requestedDirection      = cfgDirection;
  // Tái sử dụng thông số động học ước lượng có sẵn của firmware (không phải vận tốc đo thực tế)
  req.nominalLinearSpeedMmS   = ((float)cfgCruise / 255.0f) * V_MAX_MM_S;
  req.nominalAngularSpeedDegS = ((float)(MIN_TURN_PWM + MAX_TURN_PWM) * 0.5f / 255.0f) * SPIN_MAX_DEG_S;
  req.laneChangePenaltySec    = PLANNER_DEF_LANE_PENALTY_S;

  PlanError err = CoveragePlanner::generateBestPlan(req, activePlan);
  if (err != PLAN_OK) {
    laneTracker.onSessionFailed(planErrorToStr(err));
    setError(planErrorToStr(err));
    return false;
  }

  laneTracker.beginSession(&activePlan);
  return true;
}

// ======================= CẢM BIẾN SIÊU ÂM NON-BLOCKING =======================
static inline void IRAM_ATTR handleUsIsr(int idx) {
  if (digitalRead(US_ECHO[idx]) == HIGH) {
    usRiseUs[idx] = micros();
    usHigh[idx] = true;
  } else if (usHigh[idx]) {
    usWidthUs[idx] = micros() - usRiseUs[idx];
    usHigh[idx] = false;
    usDone[idx] = true;
  }
}
void IRAM_ATTR isrUs0() { handleUsIsr(0); }
void IRAM_ATTR isrUs1() { handleUsIsr(1); }
void IRAM_ATTR isrUs2() { handleUsIsr(2); }

void pushUsSample(int idx, uint16_t mm) {
  uint8_t p = usHistIdx[idx];
  usHist[idx][p] = mm;
  usHistIdx[idx] = (p + 1) % 3;
  us[idx] = median3(usHist[idx][0], usHist[idx][1], usHist[idx][2]);
  if (idx == 0) newUsFront = true;
}

void pollUltrasonic() {
  unsigned long now = millis();
  if (!usBusy) {
    if (now - lastUsMs < US_PERIOD_MS) return;
    lastUsMs = now;
    noInterrupts();
    usDone[usTurn] = false;
    usHigh[usTurn] = false;
    usWidthUs[usTurn] = 0;
    interrupts();

    digitalWrite(US_TRIG[usTurn], LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG[usTurn], HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG[usTurn], LOW);
    usTrigMs = now;
    usBusy = true;
    return;
  }

  if (usDone[usTurn]) {
    noInterrupts();
    uint32_t w = usWidthUs[usTurn];
    usDone[usTurn] = false;
    interrupts();
    uint16_t mm = (w >= 115 && w <= 12000) ? (uint16_t)(w * 0.1715f) : 9999;
    pushUsSample(usTurn, mm);
    usTurn = (usTurn + 1) % 3;
    usBusy = false;
  } else if (now - usTrigMs >= US_TIMEOUT_MS) {
    noInterrupts();
    usHigh[usTurn] = false;
    usDone[usTurn] = false;
    interrupts();
    pushUsSample(usTurn, 9999);
    usTurn = (usTurn + 1) % 3;
    usBusy = false;
  }
}

bool frontBlockedRaw() {
  if (us[0] < US_BLOCK_MM) return true;
  uint16_t m = 9999;
  for (int a = -15; a <= 15; a++) {
    uint16_t d = scan[(a + 360) % 360];
    if (d && d < m) m = d;
  }
  return m < LIDAR_BLOCK_MM;
}

// Khoảng hở dự kiến từ mép robot tới tường bên trái / bên phải theo hướng hiện tại
void expectedSideClearance(float heading, float &leftGap, float &rightGap) {
  if (!haveRoom) { leftGap = rightGap = 9999.0f; return; }
  int h = ((int)roundf(heading / 90.0f) * 90 % 360 + 360) % 360;
  if (h == 0) {           // +Y
    leftGap  = px_mm - ROBOT_RADIUS_MM;
    rightGap = (roomW - px_mm) - ROBOT_RADIUS_MM;
  } else if (h == 180) {  // -Y
    leftGap  = (roomW - px_mm) - ROBOT_RADIUS_MM;
    rightGap = px_mm - ROBOT_RADIUS_MM;
  } else if (h == 90) {   // +X
    leftGap  = (roomL - py_mm) - ROBOT_RADIUS_MM;
    rightGap = py_mm - ROBOT_RADIUS_MM;
  } else {                // 270 (-X)
    leftGap  = py_mm - ROBOT_RADIUS_MM;
    rightGap = (roomL - py_mm) - ROBOT_RADIUS_MM;
  }
}

// ======================= ĐIỀU HƯỚNG =======================
void finishCleaning() {
  mopPwm = 0;
  emergencyStopMotors();
  laneTracker.onSessionFinished();
  state = ST_DONE;
  Serial.printf("Ket thuc phien: %s\n", cleaningResultToStr(laneTracker.getResult()));
}

void runNav(bool sensorTick) {
  if (!activePlan.valid || stepIdx >= (int)activePlan.stepCount) {
    finishCleaning();
    return;
  }
  const Step &s = activePlan.steps[stepIdx];

  if (!stepStarted) {
    if (s.purpose == CLEAN_LANE) {
      laneTracker.onLaneStarted(s.laneId);
    }
    float cur = (s.type == GO_X) ? px_mm : py_mm;
    if (fabsf(s.val - cur) < 30.0f) {
      if (s.purpose == CLEAN_LANE) {
        laneTracker.onLaneCompleted(s.laneId);
      }
      stepIdx++;
      stepRetries = 0;
      return;
    }
    if (s.type == GO_X) tgtHeading = (s.val > px_mm) ? 90.0f : 270.0f;
    else                tgtHeading = (s.val > py_mm) ? 0.0f  : 180.0f;
    stepStarted = true;
    phase = PH_TURN;
    stepStartMs = millis();
    blockCount = 0;
  }

  // Pha lùi ngắn sẵn có khi chuyển làn bị vướng mép vật cản
  if (phase == PH_BACKOFF) {
    if (millis() < backoffUntilMs) {
      drive(-SLOW_PWM, -SLOW_PWM);
      return;
    }
    emergencyStopMotors();
    stepStarted = false;  // Thử định hướng và chạy lại bước CHANGE_LANE
    return;
  }

  float err = wrap180(tgtHeading - psi);

  if (phase == PH_TURN) {
    if (fabsf(err) < TURN_TOL_DEG) { phase = PH_DRIVE; stopMotors(); blockCount = 0; return; }
    if (millis() - stepStartMs > 15000) {
      emergencyStopMotors();
      stepStarted = false;
      stepRetries = 0;
      if (s.purpose == CLEAN_LANE) {
        laneTracker.onLaneFailed(s.laneId, px_mm, py_mm, "Timeout xoay");
        stepIdx++;
      } else {
        // Không chuyển làn / tiếp cận thành công -> dừng phiên để tránh chạy sai làn
        laneTracker.onChangeLaneFailed("Timeout xoay chuyen lan");
        setError("Timeout xoay chuyen lan");
      }
      return;
    }
    int t = (int)fabsf(err * KT);
    t = clampi(t, MIN_TURN_PWM, MAX_TURN_PWM);
    if (err > 0) drive(t, -t); else drive(-t, t);
    return;
  }

  // PH_DRIVE
  if (s.purpose == CLEAN_LANE) {
    laneTracker.updateLaneCovered(s.laneId, px_mm, py_mm);
  }

  float pos = (s.type == GO_X) ? px_mm : py_mm;
  bool positive = (tgtHeading == 0.0f || tgtHeading == 90.0f);
  bool reached = positive ? (pos >= s.val) : (pos <= s.val);

  // Chỉ đếm vật cản khi có mẫu cảm biến mới (không đếm theo tốc độ loop)
  if (sensorTick) {
    if (frontBlockedRaw()) blockCount++;
    else blockCount = 0;
  }
  bool blocked = (blockCount >= BLOCK_CONFIRM_CNT);

  if (reached) {
    if (s.purpose == CLEAN_LANE) {
      laneTracker.onLaneCompleted(s.laneId);
    }
    stopMotors();
    stepIdx++;
    stepStarted = false;
    stepRetries = 0;
    return;
  }

  if (blocked) {
    emergencyStopMotors();
    blockCount = 0;

    // Bước chuyển làn (CHANGE_LANE): dùng cơ chế retry sẵn có 1 lần;
    // nếu vẫn thất bại -> dừng phiên an toàn thay vì lau sai vị trí làn.
    if (s.purpose == CHANGE_LANE) {
      if (stepRetries == 0) {
        stepRetries = 1;
        phase = PH_BACKOFF;
        backoffUntilMs = millis() + 450;
        Serial.println("Vat can khi chuyen lan -> lui nhe thu lai");
        return;
      }
      laneTracker.onChangeLaneFailed("Vat can khi chuyen lan");
      setError("Khong chuyen duoc sang lan ke tiep");
      return;
    }

    if (s.purpose == APPROACH_START) {
      laneTracker.onSessionFailed("Vat can khi tiep can diem bat dau");
      setError("Khong tiep can duoc diem bat dau");
      return;
    }

    // Bước lau làn (CLEAN_LANE): Skip and Record (ghi nhận BLOCKED hoặc PARTIAL)
    laneTracker.onLaneBlocked(s.laneId, px_mm, py_mm, "Gap vat can");
    stepIdx++;
    stepStarted = false;
    stepRetries = 0;
    Serial.printf("Lan %d bi can -> %s\n", (int)s.laneId,
                  laneStatusToStr(activePlan.lanes[s.laneId].status));
    return;
  }

  float remain = fabsf(s.val - pos);
  int base = (remain < 300.0f) ? SLOW_PWM : cfgCruise;
  int corr = (int)(KH * err);
  corr = clampi(corr, -60, 60);

  // Chỉ né vật cản hông nếu đó không phải bức tường biên theo kế hoạch
  float leftWallGap, rightWallGap;
  expectedSideClearance(tgtHeading, leftWallGap, rightWallGap);
  if (us[1] < SIDE_NEAR_MM && leftWallGap  > SIDE_NEAR_MM + 40.0f) corr += 25;
  if (us[2] < SIDE_NEAR_MM && rightWallGap > SIDE_NEAR_MM + 40.0f) corr -= 25;

  drive(base + corr, base - corr);

  if (fabsf(err) > 25.0f) phase = PH_TURN;
}

// ======================= LỆNH ĐIỀU KHIỂN =======================
void startCleaning() {
  if (unoFlashMode) return;
  memset(&activePlan, 0, sizeof(activePlan));
  laneTracker.reset();
  calibOk = calibFail = 0;
  calibSin = calibCos = 0;
  for (int k = 0; k < 4; k++) calibD[k] = 0;
  haveRoom = false; errMsg = "";
  manualUntil = 0; mopPwm = 0;
  stepIdx = 0; stepStarted = false; stepRetries = 0; blockCount = 0;
  emergencyStopMotors();
  state = ST_CALIB;
  Serial.println("Bat dau hieu chinh...");
}

void stopAll() {
  if (state == ST_RUN || state == ST_CALIB) {
    laneTracker.onUserAborted();
  }
  mopPwm = 0;
  manualUntil = 0;
  state = ST_IDLE;
  emergencyStopMotors();
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

void applyCmd(const String& c, int v) {
  if (unoFlashMode) return;
  v = clampi(v, 80, MAX_SAFE_PWM);
  if (c == "start")       startCleaning();
  else if (c == "stop")   stopAll();
  else if (c == "mopon")  { if (state != ST_RUN) mopPwm = MOP_PWM; }
  else if (c == "mopoff") { if (state != ST_RUN) mopPwm = 0; }
  else if (c == "S")      { manualUntil = 0; stopMotors(); }
  else if (state != ST_RUN && state != ST_CALIB) {
    if      (c == "F") { manualL =  v; manualR =  v; manualUntil = millis() + 400; }
    else if (c == "B") { manualL = -v; manualR = -v; manualUntil = millis() + 400; }
    else if (c == "L") { manualL = -v; manualR =  v; manualUntil = millis() + 400; }
    else if (c == "R") { manualL =  v; manualR = -v; manualUntil = millis() + 400; }
  }
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

// ---- Cài đặt lưu trong Flash ----
bool parseStrictFloat(const String& str, float& out) {
  if (str.length() == 0) return false;
  char* endp = nullptr;
  float v = strtof(str.c_str(), &endp);
  if (endp == str.c_str() || *endp != '\0' || !isfinite(v)) return false;
  out = v;
  return true;
}

bool parseStrictInt(const String& str, int& out) {
  if (str.length() == 0) return false;
  char* endp = nullptr;
  long v = strtol(str.c_str(), &endp, 10);
  if (endp == str.c_str() || *endp != '\0') return false;
  if (v < -32768L || v > 32767L) return false;
  out = (int)v;
  return true;
}

bool parseDirectionArg(const String& str, CleaningDirection& out) {
  if (str == "AUTO" || str == "auto" || str == "0") { out = DIRECTION_AUTO; return true; }
  if (str == "X"    || str == "x"    || str == "1") { out = DIRECTION_X;    return true; }
  if (str == "Y"    || str == "y"    || str == "2") { out = DIRECTION_Y;    return true; }
  return false;
}

void loadSettings() {
  prefs.begin("robot", true);
  cfgMopWidth = clampf(prefs.getFloat("mop", DEF_MOP_WIDTH_MM), 100.0f, 400.0f);
  cfgCruise   = clampi(prefs.getInt("cruise", DEF_CRUISE_PWM), 100, MAX_SAFE_PWM);
  cfgEdge     = clampi(prefs.getInt("edge", DEF_EDGE_MM), ROBOT_RADIUS_MM + 10, 500);
  cfgOverlap  = clampf(prefs.getFloat("overlap", DEF_OVERLAP_PCT), MIN_OVERLAP_PERCENT, MAX_OVERLAP_PERCENT);
  int dirInt  = prefs.getInt("dir", (int)DIRECTION_AUTO);
  if (dirInt >= (int)DIRECTION_AUTO && dirInt <= (int)DIRECTION_Y) {
    cfgDirection = (CleaningDirection)dirInt;
  } else {
    cfgDirection = DIRECTION_AUTO;
  }
  prefs.end();
}

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

    prefs.begin("robot", false);
    prefs.putFloat("mop", cfgMopWidth);
    prefs.putInt("cruise", cfgCruise);
    prefs.putInt("edge", cfgEdge);
    prefs.putFloat("overlap", cfgOverlap);
    prefs.putInt("dir", (int)cfgDirection);
    prefs.end();
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
  Serial2.setRxBufferSize(2048);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);

  for (int i = 0; i < 3; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(US_ECHO[0]), isrUs0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO[1]), isrUs1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO[2]), isrUs2, CHANGE);

  initCrc();
  memset(acc, 0, sizeof(acc));
  memset(scan, 0, sizeof(scan));

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

  lastPoseMs = millis();
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
