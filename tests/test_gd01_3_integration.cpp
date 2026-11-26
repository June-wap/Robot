#include <cstdio>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include <Arduino.h>
#include <WebServer.h>

#include "../esp32_robot/src/core/RobotContext.h"
#include "../esp32_robot/src/core/RobotTypes.h"
#include "../esp32_robot/src/core/MathUtils.h"
#include "../esp32_robot/src/planning/LaneTypes.h"
#include "../esp32_robot/src/planning/CoveragePlanner.h"
#include "../esp32_robot/src/planning/LaneTracker.h"
#include "../esp32_robot/src/navigation/CleaningSession.h"
#include "../esp32_robot/src/navigation/Navigator.h"
#include "../esp32_robot/src/control/MotionController.h"
#include "../esp32_robot/src/control/CommandHandler.h"
#include "../esp32_robot/src/communication/ArduinoLink.h"
#include "../esp32_robot/src/localization/Calibration.h"
#include "../esp32_robot/src/config/Settings.h"
#include "../esp32_robot/src/web/RobotWebServer.h"
#include "../esp32_robot/src/web/ApiHandlers.h"
#include "../esp32_robot/src/web/WebPage.h"

// ======================= MOCK HARDWARE & STUBS =======================
unsigned long gMockMillis = 1000;
HardwareSerial Serial, Serial1, Serial2;

static int gImmediateStopCount = 0;
void arduinoLinkInit() {}
void setUnoFlashMode(bool en) { gRobot.unoFlashMode = en; }
void pollLink() {}
void sendLink() {}
void arduinoLinkSendImmediateStop() { gImmediateStopCount++; }
void arduinoLinkSendOtaStop() {}

void resetCalibration() {}
void doCalibrate() {}

void loadSettings() {}
void saveSettings() {}
bool parseStrictFloat(const String& str, float& out) {
  if (str.length() == 0) return false;
  char* endp = nullptr;
  float v = strtof(str.c_str(), &endp);
  if (endp == str.c_str() || *endp != '\0' || !std::isfinite(v)) return false;
  out = v;
  return true;
}
bool parseStrictInt(const String& str, int& out) {
  if (str.length() == 0) return false;
  char* endp = nullptr;
  long v = strtol(str.c_str(), &endp, 10);
  if (endp == str.c_str() || *endp != '\0' || v < -32768L || v > 32767L) return false;
  out = (int)v;
  return true;
}
bool parseDirectionArg(const String& str, CleaningDirection& out) {
  if (str == "AUTO" || str == "auto" || str == "0") { out = DIRECTION_AUTO; return true; }
  if (str == "X"    || str == "x"    || str == "1") { out = DIRECTION_X;    return true; }
  if (str == "Y"    || str == "y"    || str == "2") { out = DIRECTION_Y;    return true; }
  return false;
}

static WebServer gMockServer(80);
WebServer& getWebServer() { return gMockServer; }
void webServerInit() {}
void webServerHandleClient() {}

const char INDEX_HTML[] PROGMEM = "<html>mock</html>";

// ======================= TEST FRAMEWORK =======================
static int gPassed = 0;
static int gFailed = 0;

#define EXPECT_TRUE(cond, name) do { \
  if (cond) { gPassed++; printf("[PASS] %s\n", name); } \
  else { gFailed++; printf("[FAIL] %s (line %d)\n", name, __LINE__); } \
} while (0)

static void resetSimEnvironment(float roomW, float roomL, float rx, float ry, float psi,
                                CleaningDirection dir = DIRECTION_Y, float overlap = 10.0f,
                                float mop = 180.0f, int edge = 210) {
  gMockMillis = 1000;
  gImmediateStopCount = 0;
  gRobot = RobotContext();
  gRobot.roomW = roomW;
  gRobot.roomL = roomL;
  gRobot.haveRoom = true;
  gRobot.px_mm = rx;
  gRobot.py_mm = ry;
  gRobot.psi = psi;
  gRobot.cfgDirection = dir;
  gRobot.cfgOverlap = overlap;
  gRobot.cfgMopWidth = mop;
  gRobot.cfgEdge = edge;
  gRobot.us[0] = 9999;
  gRobot.us[1] = 9999;
  gRobot.us[2] = 9999;
  std::memset(gRobot.scan, 0, sizeof(gRobot.scan));
  resetNavigationState();
}

// Mô phỏng động học đơn giản theo cmdL/cmdR để Navigator thực thi thực tế
static void stepKinematics(float dtSec, float crossDriftMmPerStep = 0.0f, bool alongY = true) {
  gMockMillis += (unsigned long)(dtSec * 1000.0f);
  const float v = (gRobot.cmdL + gRobot.cmdR) / 510.0f * V_MAX_MM_S;
  const float w = (gRobot.cmdL - gRobot.cmdR) / 510.0f * SPIN_MAX_DEG_S;
  const float mid = gRobot.psi + 0.5f * w * dtSec;
  gRobot.px_mm += v * dtSec * sinf(radians(mid));
  gRobot.py_mm += v * dtSec * cosf(radians(mid));
  if (fabsf(v) > 1.0f && fabsf(crossDriftMmPerStep) > 0.0f) {
    if (alongY) gRobot.px_mm += crossDriftMmPerStep;
    else        gRobot.py_mm += crossDriftMmPerStep;
  }
  gRobot.psi = wrap180(gRobot.psi + w * dtSec);
}

static void setupActiveSession(float roomW, float roomL, float rx, float ry, float psi,
                               CleaningDirection dir, float overlap = 10.0f,
                               float mop = 180.0f, int edge = 210) {
  resetSimEnvironment(roomW, roomL, rx, ry, psi, dir, overlap, mop, edge);
  gRobot.state = ST_CALIB;
  bool ok = buildPlan();
  if (ok) {
    resetNavigationState();
    gRobot.mopPwm = MOP_PWM;
    gRobot.state = ST_RUN;
  }
}

int main() {
  printf("\n=== GD01.3 INTEGRATION & CORRECTNESS TEST SUITE ===\n");

  // 1. CLEAN_LANE bắt đầu sai tọa độ ngang -> từ chối chạy & dừng phiên, không COMPLETED
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    // Đẩy robot lệch ngang 120 mm (> LANE_CROSS_TRACK_MAX_MM=80 mm) trước khi bắt đầu làn 0
    gRobot.px_mm = gRobot.activePlan.lanes[0].startX + 120.0f;
    gRobot.py_mm = gRobot.activePlan.lanes[0].startY;
    runNav(true);
    EXPECT_TRUE(gRobot.state == ST_ERR &&
                gRobot.laneTracker.getResult() == RESULT_FAILED &&
                gRobot.activePlan.lanes[0].status == LANE_PENDING &&
                gRobot.laneTracker.getProgress().completedLanes == 0,
                "T01: CLEAN_LANE starting off cross-track aborts session safely (no false start/complete)");
  }

  // 2. Robot bắt đầu ngay gần đích (< 30 mm) nhưng chưa lau làn -> KHÔNG đánh dấu COMPLETED
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    // Đặt robot cách điểm đích làn 0 chỉ 10 mm (< 30 mm cũ) nhưng cách điểm đầu ~1570 mm
    gRobot.px_mm = gRobot.activePlan.lanes[0].endX;
    gRobot.py_mm = gRobot.activePlan.lanes[0].endY - 10.0f;
    runNav(true);
    EXPECT_TRUE(gRobot.state == ST_ERR &&
                gRobot.laneTracker.getResult() == RESULT_FAILED &&
                gRobot.activePlan.lanes[0].status != LANE_COMPLETED &&
                gRobot.laneTracker.getProgress().completedLanes == 0,
                "T02: Robot near lane end without starting at lane start is NOT marked COMPLETED");
  }

  // 3. Robot gặp vật cản giữa làn -> ghi PARTIAL, dừng phiên có kiểm soát, không chạy làn sau sai vị trí
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    // Chạy vào giữa làn 0 (~700 mm)
    for (int iter = 0; iter < 150 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      stepKinematics(0.05f);
      if (gRobot.py_mm >= 900.0f) break;
    }
    // Mô phỏng vật cản trước liên tiếp 3 chu kỳ cảm biến
    gRobot.us[0] = 40;
    for (int k = 0; k < 5 && gRobot.state == ST_RUN; k++) {
      runNav(true);
      gMockMillis += 50;
    }
    CleaningProgress pr = gRobot.laneTracker.getProgress();
    EXPECT_TRUE(gRobot.state == ST_DONE &&
                gRobot.laneTracker.getResult() == RESULT_PARTIAL &&
                gRobot.activePlan.lanes[0].status == LANE_PARTIAL &&
                gRobot.activePlan.lanes[1].status == LANE_PENDING &&
                pr.completedLanes == 0 && pr.partialLanes == 1,
                "T03: Mid-lane obstacle marks PARTIAL and halts session without invalid lane transition");
  }

  // 4. Robot timeout khi xoay đầu làn -> ghi FAILED, dừng phiên, không chuyển làn
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 90.0f, DIRECTION_Y);
    // Ở đúng đầu làn 0 nhưng góc hướng sai 90 độ và không xoay được
    runNav(true);  // vào PH_TURN
    gMockMillis += 15500;
    runNav(true);  // vượt timeout 15000 ms
    EXPECT_TRUE(gRobot.state == ST_ERR &&
                gRobot.laneTracker.getResult() == RESULT_FAILED &&
                gRobot.activePlan.lanes[0].status == LANE_FAILED &&
                gRobot.activePlan.lanes[1].status == LANE_PENDING &&
                gRobot.laneTracker.getProgress().completedLanes == 0,
                "T04: Turn timeout at lane start marks LANE_FAILED and stops session");
  }

  // 5. Bước chuyển làn thành công trên 1 trục nhưng sai trục còn lại -> bị chặn khi vào làn kế tiếp
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    // Chạy hoàn thành làn 0
    for (int iter = 0; iter < 500 && gRobot.state == ST_RUN && gRobot.stepIdx == 0; iter++) {
      runNav(true);
      stepKinematics(0.05f);
    }
    bool lane0Done = (gRobot.activePlan.lanes[0].status == LANE_COMPLETED &&
                      gRobot.activePlan.steps[gRobot.stepIdx].purpose == CHANGE_LANE);
    // Hoàn thành CHANGE_LANE (đúng trục X của làn 1) nhưng cố tình làm sai trục Y đi 350 mm
    const float lane1StartX = gRobot.activePlan.lanes[1].startX;
    const float lane1StartY = gRobot.activePlan.lanes[1].startY;
    gRobot.px_mm = lane1StartX;
    gRobot.py_mm = lane1StartY - 350.0f;
    gRobot.psi = 90.0f;
    runNav(true);  // CHANGE_LANE bỏ qua bước ngắn (<30 mm theo X) -> sang CLEAN_LANE(1)
    runNav(true);  // CLEAN_LANE(1) kiểm tra CẢ X VÀ Y -> phát hiện Y lệch 350 mm!
    EXPECT_TRUE(lane0Done &&
                gRobot.state == ST_ERR &&
                gRobot.laneTracker.getResult() == RESULT_FAILED &&
                gRobot.activePlan.lanes[1].status == LANE_PENDING &&
                gRobot.laneTracker.getProgress().completedLanes == 1,
                "T05: CHANGE_LANE right on X but wrong on Y fails validation before starting next lane");
  }

  // 6. Robot đi lệch làn ngang (> 80 mm) trong khi lái -> KHÔNG được đánh dấu COMPLETED
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    for (int iter = 0; iter < 600 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      // Gây trôi ngang +3 mm/bước -> nhanh chóng lệch > 80 mm khỏi tâm làn 0
      stepKinematics(0.05f, 3.0f, true);
    }
    EXPECT_TRUE(gRobot.activePlan.lanes[0].status != LANE_COMPLETED &&
                gRobot.laneTracker.getProgress().completedLanes == 0 &&
                gRobot.laneTracker.getResult() != RESULT_COMPLETED &&
                gRobot.state != ST_RUN,
                "T06: Off-track robot (>80mm cross-track) is never marked LANE_COMPLETED");
  }

  // 7. Robot tới đúng điểm cuối trên toàn bộ làn -> RESULT_COMPLETED & 100%
  {
    setupActiveSession(1000.0f, 1600.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    for (int iter = 0; iter < 5000 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      stepKinematics(0.05f);
    }
    CleaningProgress pr = gRobot.laneTracker.getProgress();
    EXPECT_TRUE(gRobot.state == ST_DONE &&
                gRobot.laneTracker.getResult() == RESULT_COMPLETED &&
                pr.completedLanes == gRobot.activePlan.laneCount &&
                fabsf(pr.completedPercent - 100.0f) < 1e-2f,
                "T07: On-track execution to lane endpoints completes all lanes (RESULT_COMPLETED)");
  }

  // 8. Người dùng dừng trong ST_CALIB -> RESULT_ABORTED (không còn RESULT_NONE)
  {
    resetSimEnvironment(3000.0f, 4000.0f, 500.0f, 500.0f, 0.0f);
    startCleaning();
    bool inCalib = (gRobot.state == ST_CALIB && gRobot.laneTracker.getResult() == RESULT_RUNNING);
    stopAll();
    EXPECT_TRUE(inCalib &&
                gRobot.state == ST_IDLE &&
                gRobot.laneTracker.getResult() == RESULT_ABORTED,
                "T08: User stop during ST_CALIB transitions CleaningResult to RESULT_ABORTED");
  }

  // 9. Người dùng dừng trong ST_RUN -> RESULT_ABORTED và bảo toàn kết quả đã lau
  {
    setupActiveSession(1200.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    // Chạy hết làn 0 và nửa làn 1
    for (int iter = 0; iter < 1200 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      stepKinematics(0.05f);
      if (gRobot.activePlan.lanes[0].status == LANE_COMPLETED &&
          gRobot.activePlan.lanes[1].status == LANE_RUNNING &&
          gRobot.activePlan.lanes[1].estimatedCoveredLengthMm >= 500.0f) {
        break;
      }
    }
    stopAll();
    CleaningProgress pr = gRobot.laneTracker.getProgress();
    EXPECT_TRUE(gRobot.state == ST_IDLE &&
                gRobot.laneTracker.getResult() == RESULT_ABORTED &&
                pr.completedLanes == 1 && pr.partialLanes == 1,
                "T09: User stop during ST_RUN sets RESULT_ABORTED and preserves completed/partial lanes");
  }

  // 10. Phiên có làn chưa hoàn thành (làn cuối bị cản giữa làn) -> RESULT_PARTIAL (không COMPLETED)
  {
    setupActiveSession(420.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y); // đúng 1 làn
    for (int iter = 0; iter < 150 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      stepKinematics(0.05f);
      if (gRobot.py_mm >= 1000.0f) break;
    }
    gRobot.us[0] = 30;
    for (int k = 0; k < 6 && gRobot.state == ST_RUN; k++) {
      runNav(true);
      gMockMillis += 50;
    }
    CleaningProgress pr = gRobot.laneTracker.getProgress();
    EXPECT_TRUE(gRobot.state == ST_DONE &&
                gRobot.laneTracker.getResult() == RESULT_PARTIAL &&
                gRobot.activePlan.lanes[0].status == LANE_PARTIAL &&
                pr.completedLanes == 0 && pr.partialLanes == 1,
                "T10: Session with incomplete final lane yields RESULT_PARTIAL (never COMPLETED)");
  }

  // 11. Kế hoạch có đúng 64 làn (MAX_LANES) + xác minh handlePlan() JSON đầy đủ
  {
    // effSpacing = 180 * 0.9 = 162 mm. 63 intervals * 162 = 10206 -> roomW = 10206 + 420 = 10626 mm -> đúng 64 làn
    setupActiveSession(10626.0f, 3000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y, 10.0f, 180.0f, 210);
    bool plan64Ok = (gRobot.activePlan.valid &&
                     gRobot.activePlan.laneCount == MAX_LANES &&
                     CoveragePlanner::verifyPlanIntegrity(gRobot.activePlan));
    handlePlan();
    const std::string& body = gMockServer.lastBody;
    bool jsonOk = (gMockServer.lastStatus == 200 &&
                   body.find("\"lane_count\":64") != std::string::npos &&
                   body.find("\"id\":0,") != std::string::npos &&
                   body.find("\"id\":63,") != std::string::npos &&
                   body.size() > 4500 &&
                   body.substr(body.size() - 2) == "]}");
    EXPECT_TRUE(plan64Ok && jsonOk,
                "T11: Exact 64-lane plan passes integrity and serializes complete JSON in handlePlan()");
  }

  // 12. Kế hoạch yêu cầu vượt 64 làn -> từ chối PLAN_ERR_EXCEEDS_MAX_LANES
  {
    PlanRequest r;
    r.roomW = 10800.0f; // > 10626 mm -> 65 lanes
    r.roomL = 3000.0f;
    r.edgeMm = 210.0f;
    r.mopWidthMm = 180.0f;
    r.overlapPercent = 10.0f;
    r.robotRadiusMm = 170.0f;
    r.robotX = 500.0f;
    r.robotY = 500.0f;
    r.robotPsiDeg = 0.0f;
    r.requestedDirection = DIRECTION_Y;
    r.nominalLinearSpeedMmS = 194.1f;
    r.nominalAngularSpeedDegS = 86.8f;
    r.laneChangePenaltySec = 1.5f;
    CoveragePlan p;
    PlanError err = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(err == PLAN_ERR_EXCEEDS_MAX_LANES && !p.valid,
                "T12: Plan requiring > 64 lanes rejected with PLAN_ERR_EXCEEDS_MAX_LANES");
  }

  // 13. Đầu vào số học cực lớn, cực nhỏ, NaN, Infinity -> không overflow/treo, từ chối an toàn
  {
    PlanRequest base;
    base.roomW = 3000.0f; base.roomL = 4000.0f; base.edgeMm = 210.0f;
    base.mopWidthMm = 180.0f; base.overlapPercent = 10.0f; base.robotRadiusMm = 170.0f;
    base.robotX = 500.0f; base.robotY = 500.0f; base.robotPsiDeg = 0.0f;
    base.requestedDirection = DIRECTION_AUTO;
    base.nominalLinearSpeedMmS = 194.1f; base.nominalAngularSpeedDegS = 86.8f;
    base.laneChangePenaltySec = 1.5f;

    CoveragePlan p;
    PlanRequest huge1 = base; huge1.roomW = 1e12f; huge1.roomL = 1e12f; // rawRatio ~ 6e9 (> UINT32_MAX!)
    PlanError e1 = CoveragePlanner::generateBestPlan(huge1, p);

    PlanRequest huge2 = base; huge2.roomW = 1e38f; huge2.roomL = 1e38f;
    PlanError e2 = CoveragePlanner::generateBestPlan(huge2, p);

    PlanRequest tinyMop = base; tinyMop.mopWidthMm = 1e-20f;
    PlanError e3 = CoveragePlanner::generateBestPlan(tinyMop, p);

    PlanRequest hugeAngle = base; hugeAngle.robotPsiDeg = 1e12f; // không được treo while()!
    PlanError e4 = CoveragePlanner::generateBestPlan(hugeAngle, p);
    bool p4Valid = p.valid;

    PlanRequest nanPsi = base; nanPsi.robotPsiDeg = std::numeric_limits<float>::quiet_NaN();
    PlanError e5 = CoveragePlanner::generateBestPlan(nanPsi, p);
    bool p5Invalid = !p.valid;

    PlanRequest infSpeed = base; infSpeed.nominalLinearSpeedMmS = std::numeric_limits<float>::infinity();
    PlanError e6 = CoveragePlanner::generateBestPlan(infSpeed, p);
    bool p6Invalid = !p.valid;

    EXPECT_TRUE(e1 == PLAN_ERR_EXCEEDS_MAX_LANES &&
                e2 == PLAN_ERR_EXCEEDS_MAX_LANES &&
                e3 == PLAN_ERR_EXCEEDS_MAX_LANES &&
                e4 == PLAN_OK && p4Valid &&
                e5 == PLAN_ERR_INVALID_POSE && p5Invalid &&
                e6 == PLAN_ERR_INVALID_KINEMATICS && p6Invalid,
                "T13: Extreme numeric inputs (1e12, 1e38, 1e-20, huge psi, NaN, Inf) handled safely");
  }

  // 14. Plan generation thất bại sau một phiên thành công -> vô hiệu hóa kế hoạch cũ sạch sẽ
  {
    setupActiveSession(1000.0f, 1600.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    for (int iter = 0; iter < 5000 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      stepKinematics(0.05f);
    }
    bool prevDone = (gRobot.state == ST_DONE && gRobot.laneTracker.getResult() == RESULT_COMPLETED);
    // Bắt đầu phiên mới với phòng quá nhỏ -> buildPlan() phải thất bại và xóa sạch activePlan
    startCleaning();
    gRobot.roomW = 300.0f;
    gRobot.roomL = 300.0f;
    gRobot.px_mm = 150.0f;
    gRobot.py_mm = 150.0f;
    bool built = buildPlan();
    EXPECT_TRUE(prevDone && !built &&
                !gRobot.activePlan.valid &&
                gRobot.activePlan.laneCount == 0 &&
                gRobot.activePlan.stepCount == 0 &&
                gRobot.state == ST_ERR &&
                gRobot.laneTracker.getResult() == RESULT_FAILED,
                "T14: Failed plan generation after successful session invalidates stale plan cleanly");
  }

  // 15. Kiểm thử đầy đủ hướng DIRECTION_X qua Navigator + LaneTracker
  {
    setupActiveSession(1800.0f, 1000.0f, 210.0f, 210.0f, 90.0f, DIRECTION_X);
    bool isDirX = (gRobot.activePlan.valid && gRobot.activePlan.chosenDirection == DIRECTION_X);
    for (int iter = 0; iter < 6000 && gRobot.state == ST_RUN; iter++) {
      runNav(true);
      stepKinematics(0.05f);
    }
    CleaningProgress pr = gRobot.laneTracker.getProgress();
    EXPECT_TRUE(isDirX &&
                gRobot.state == ST_DONE &&
                gRobot.laneTracker.getResult() == RESULT_COMPLETED &&
                pr.completedLanes == gRobot.activePlan.laneCount &&
                pr.completedLanes > 1,
                "T15: Full DIRECTION_X navigation executes all lanes and transitions to RESULT_COMPLETED");
  }

  // 16. verifyPlanIntegrity từ chối kế hoạch hỏng (sai enum, NaN, sai trục, sai target, sai thứ tự, vượt spacing)
  {
    PlanRequest r;
    r.roomW = 1200.0f; r.roomL = 2000.0f; r.edgeMm = 210.0f; r.mopWidthMm = 180.0f;
    r.overlapPercent = 10.0f; r.robotRadiusMm = 170.0f;
    r.robotX = 500.0f; r.robotY = 500.0f; r.robotPsiDeg = 0.0f;
    r.requestedDirection = DIRECTION_Y;
    r.nominalLinearSpeedMmS = 194.1f; r.nominalAngularSpeedDegS = 86.8f; r.laneChangePenaltySec = 1.5f;

    CoveragePlan good;
    CoveragePlanner::generateBestPlan(r, good);

    CoveragePlan bad1 = good; bad1.chosenDirection = (CleaningDirection)99;
    CoveragePlan bad2 = good; bad2.lanes[0].endY = std::numeric_limits<float>::quiet_NaN();
    CoveragePlan bad3 = good;
    for (uint16_t i = 0; i < bad3.stepCount; i++) {
      if (bad3.steps[i].purpose == CLEAN_LANE) { bad3.steps[i].type = GO_X; break; }
    }
    CoveragePlan bad4 = good;
    for (uint16_t i = 0; i < bad4.stepCount; i++) {
      if (bad4.steps[i].purpose == CLEAN_LANE) { bad4.steps[i].val -= 50.0f; break; }
    }
    CoveragePlan bad5 = good;
    for (uint16_t i = 0; i < bad5.stepCount; i++) {
      if (bad5.steps[i].purpose == CHANGE_LANE) { bad5.steps[i].val += 40.0f; break; }
    }
    CoveragePlan bad6 = good; bad6.effectiveSpacingMm = 50.0f; // actualSpacing > effectiveSpacing

    EXPECT_TRUE(CoveragePlanner::verifyPlanIntegrity(good) &&
                !CoveragePlanner::verifyPlanIntegrity(bad1) &&
                !CoveragePlanner::verifyPlanIntegrity(bad2) &&
                !CoveragePlanner::verifyPlanIntegrity(bad3) &&
                !CoveragePlanner::verifyPlanIntegrity(bad4) &&
                !CoveragePlanner::verifyPlanIntegrity(bad5) &&
                !CoveragePlanner::verifyPlanIntegrity(bad6),
                "T16: verifyPlanIntegrity rejects invalid enum, NaN, wrong axis, bad targets, and spacing violation");
  }

  // 17. API handlePlan() cho 0 làn và 1 làn + handleCmd() xác nhận contract gốc 200 OK
  {
    resetSimEnvironment(1000.0f, 1600.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    // 0 lanes (chưa có plan)
    handlePlan();
    bool zeroOk = (gMockServer.lastStatus == 200 &&
                   gMockServer.lastBody.find("\"valid\":false") != std::string::npos &&
                   gMockServer.lastBody.find("\"lanes\":[]}") != std::string::npos);

    // 1 lane
    setupActiveSession(420.0f, 2000.0f, 210.0f, 210.0f, 0.0f, DIRECTION_Y);
    handlePlan();
    bool oneOk = (gMockServer.lastStatus == 200 &&
                  gMockServer.lastBody.find("\"lane_count\":1") != std::string::npos &&
                  gMockServer.lastBody.find("\"id\":0,") != std::string::npos);

    // handleCmd contract gốc
    gMockServer.args.clear();
    gMockServer.args["c"] = "stop";
    handleCmd();
    bool cmdOk = (gMockServer.lastStatus == 200 && gMockServer.lastBody == "ok");

    EXPECT_TRUE(zeroOk && oneOk && cmdOk,
                "T17: handlePlan() valid JSON for 0 & 1 lanes and handleCmd() preserves 200 OK contract");
  }

  printf("\nSUMMARY [test_gd01_3_integration]: Passed=%d Failed=%d\n", gPassed, gFailed);
  return (gFailed == 0) ? 0 : 1;
}
