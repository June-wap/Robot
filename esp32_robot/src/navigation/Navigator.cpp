#include "Navigator.h"
#include "CleaningSession.h"
#include "../config/RobotConfig.h"
#include "../control/MotionController.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"
#include "../core/RobotTypes.h"
#include "../planning/LaneTypes.h"
#include "../sensors/ObstacleDetector.h"
#include <Arduino.h>
#include <math.h>

static bool          stepStarted = false;
static Phase         phase = PH_TURN;
static float         tgtHeading = 0.0f;
static unsigned long stepStartMs = 0;
static unsigned long backoffUntilMs = 0;
static uint8_t       stepRetries = 0;
static int           blockCount = 0;

void resetNavigationState() {
  gRobot.stepIdx = 0;
  stepStarted = false;
  stepRetries = 0;
  blockCount = 0;
}

static void stopSessionControlledOnIncompleteLane(const char* failReason) {
  const CleaningProgress pr = gRobot.laneTracker.getProgress();
  if (pr.completedLanes > 0 || pr.partialLanes > 0) {
    finishCleaning();
  } else {
    setError(failReason);
  }
}

void runNav(bool sensorTick) {
  if (!gRobot.activePlan.valid || gRobot.stepIdx >= (int)gRobot.activePlan.stepCount) {
    finishCleaning();
    return;
  }
  const Step& s = gRobot.activePlan.steps[gRobot.stepIdx];

  if (!stepStarted) {
    if (s.purpose == CLEAN_LANE) {
      // Xác minh vị trí bắt đầu hợp lệ trên CẢ X VÀ Y trước khi khởi chạy làn mới.
      // Tuyệt đối không bỏ qua hoặc đánh dấu hoàn thành chỉ vì robot ở gần tọa độ đích.
      if (!gRobot.laneTracker.isPositionAtLaneStart(s.laneId, gRobot.px_mm, gRobot.py_mm)) {
        emergencyStopMotors();
        stepStarted = false;
        stepRetries = 0;
        setError("Vi tri bat dau lan khong hop le");
        return;
      }
      gRobot.laneTracker.onLaneStarted(s.laneId);
    } else {
      // Chỉ cho phép bỏ qua bước ngắn (< 30 mm) đối với APPROACH_START và CHANGE_LANE
      float cur = (s.type == GO_X) ? gRobot.px_mm : gRobot.py_mm;
      if (fabsf(s.val - cur) < 30.0f) {
        gRobot.stepIdx++;
        stepRetries = 0;
        return;
      }
    }

    if (s.type == GO_X) tgtHeading = (s.val > gRobot.px_mm) ? 90.0f : 270.0f;
    else                tgtHeading = (s.val > gRobot.py_mm) ? 0.0f  : 180.0f;
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

  float err = wrap180(tgtHeading - gRobot.psi);

  if (phase == PH_TURN) {
    if (fabsf(err) < TURN_TOL_DEG) { phase = PH_DRIVE; stopMotors(); blockCount = 0; return; }
    if (millis() - stepStartMs > 15000) {
      emergencyStopMotors();
      stepStarted = false;
      stepRetries = 0;
      if (s.purpose == CLEAN_LANE) {
        gRobot.laneTracker.onLaneFailed(s.laneId, gRobot.px_mm, gRobot.py_mm, "Timeout xoay");
        // Không mặc định robot đã tới cuối làn và không chuyển sang làn sau từ đầu làn sai
        setError("Timeout xoay dau lan");
      } else {
        // Không chuyển làn / tiếp cận thành công -> dừng phiên để tránh chạy sai làn
        gRobot.laneTracker.onChangeLaneFailed("Timeout xoay chuyen lan");
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
    gRobot.laneTracker.updateLaneCovered(s.laneId, gRobot.px_mm, gRobot.py_mm);
  }

  float pos = (s.type == GO_X) ? gRobot.px_mm : gRobot.py_mm;
  bool positive = (tgtHeading == 0.0f || tgtHeading == 90.0f);
  bool reached = positive ? (pos >= s.val) : (pos <= s.val);

  // Chỉ đếm vật cản khi có mẫu cảm biến mới (không đếm theo tốc độ loop)
  if (sensorTick) {
    if (frontBlockedRaw()) blockCount++;
    else blockCount = 0;
  }
  bool blocked = (blockCount >= BLOCK_CONFIRM_CNT);

  if (reached) {
    stopMotors();
    stepStarted = false;
    stepRetries = 0;

    if (s.purpose == CLEAN_LANE) {
      const bool completedOk = gRobot.laneTracker.completeLaneWithPose(s.laneId, gRobot.px_mm, gRobot.py_mm);
      if (completedOk) {
        gRobot.stepIdx++;
        return;
      }

      // Làn không đạt xác minh hoàn thành (ví dụ lệch ngang vượt ngưỡng) -> không đánh dấu COMPLETED
      const bool hasMoreSteps = (gRobot.stepIdx + 1 < (int)gRobot.activePlan.stepCount);
      if (!hasMoreSteps) {
        gRobot.stepIdx++;
        return;
      }
      if (!gRobot.laneTracker.isPositionAtLaneEnd(s.laneId, gRobot.px_mm, gRobot.py_mm)) {
        // Vị trí cuối bị lệch -> dừng phiên có kiểm soát thay vì chuyển làn từ vị trí sai
        setError("Lech lan vuot nguong cho phep");
        return;
      }
      gRobot.stepIdx++;
      return;
    }

    gRobot.stepIdx++;
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
      gRobot.laneTracker.onChangeLaneFailed("Vat can khi chuyen lan");
      setError("Khong chuyen duoc sang lan ke tiep");
      return;
    }

    if (s.purpose == APPROACH_START) {
      gRobot.laneTracker.onSessionFailed("Vat can khi tiep can diem bat dau");
      setError("Khong tiep can duoc diem bat dau");
      return;
    }

    // Bước lau làn (CLEAN_LANE): ghi nhận BLOCKED hoặc PARTIAL
    gRobot.laneTracker.onLaneBlocked(s.laneId, gRobot.px_mm, gRobot.py_mm, "Gap vat can");
    stepStarted = false;
    stepRetries = 0;
    Serial.printf("Lan %d bi can -> %s\n", (int)s.laneId,
                  laneStatusToStr(gRobot.activePlan.lanes[s.laneId].status));

    const bool hasMoreSteps = (gRobot.stepIdx + 1 < (int)gRobot.activePlan.stepCount);
    if (!hasMoreSteps) {
      gRobot.stepIdx++;
      return;
    }

    // Chỉ chuyển bước kế tiếp nếu vị trí thực tế đã ở gần điểm cuối làn hợp lệ;
    // nếu bị chặn giữa làn -> dừng phiên có kiểm soát, không tự tìm đường vòng hay chạy sai làn sau.
    if (gRobot.laneTracker.isPositionAtLaneEnd(s.laneId, gRobot.px_mm, gRobot.py_mm)) {
      gRobot.stepIdx++;
      return;
    }

    stopSessionControlledOnIncompleteLane("Vat can chan lan, dung phien");
    return;
  }

  float remain = fabsf(s.val - pos);
  int base = (remain < 300.0f) ? SLOW_PWM : gRobot.cfgCruise;
  int corr = (int)(KH * err);
  corr = clampi(corr, -60, 60);

  // Chỉ né vật cản hông nếu đó không phải bức tường biên theo kế hoạch
  float leftWallGap, rightWallGap;
  expectedSideClearance(tgtHeading, leftWallGap, rightWallGap);
  if (gRobot.us[1] < SIDE_NEAR_MM && leftWallGap  > SIDE_NEAR_MM + 40.0f) corr += 25;
  if (gRobot.us[2] < SIDE_NEAR_MM && rightWallGap > SIDE_NEAR_MM + 40.0f) corr -= 25;

  drive(base + corr, base - corr);

  if (fabsf(err) > 25.0f) phase = PH_TURN;
}
