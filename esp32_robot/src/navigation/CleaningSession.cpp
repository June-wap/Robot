#include "CleaningSession.h"
#include "Navigator.h"
#include "../config/RobotConfig.h"
#include "../control/MotionController.h"
#include "../core/RobotContext.h"
#include "../core/RobotTypes.h"
#include "../localization/Calibration.h"
#include "../planning/CoveragePlanner.h"
#include "../planning/LaneTypes.h"
#include <Arduino.h>
#include <string.h>

void setError(const char* m) {
  gRobot.errMsg = m;
  if (gRobot.state == ST_RUN || gRobot.state == ST_CALIB) {
    gRobot.laneTracker.onSessionFailed(m);
  }
  gRobot.state = ST_ERR;
  gRobot.mopPwm = 0;
  emergencyStopMotors();
}

bool buildPlan() {
  PlanRequest req;
  req.roomW                   = gRobot.roomW;
  req.roomL                   = gRobot.roomL;
  req.edgeMm                  = (float)gRobot.cfgEdge;
  req.mopWidthMm              = gRobot.cfgMopWidth;
  req.overlapPercent          = gRobot.cfgOverlap;
  req.robotRadiusMm           = (float)ROBOT_RADIUS_MM;
  req.robotX                  = gRobot.px_mm;
  req.robotY                  = gRobot.py_mm;
  req.robotPsiDeg             = gRobot.psi;
  req.requestedDirection      = gRobot.cfgDirection;
  // Tái sử dụng thông số động học ước lượng có sẵn của firmware (không phải vận tốc đo thực tế)
  req.nominalLinearSpeedMmS   = ((float)gRobot.cfgCruise / 255.0f) * V_MAX_MM_S;
  req.nominalAngularSpeedDegS = ((float)(MIN_TURN_PWM + MAX_TURN_PWM) * 0.5f / 255.0f) * SPIN_MAX_DEG_S;
  req.laneChangePenaltySec    = PLANNER_DEF_LANE_PENALTY_S;

  PlanError err = CoveragePlanner::generateBestPlan(req, gRobot.activePlan);
  if (err != PLAN_OK) {
    gRobot.laneTracker.onSessionFailed(planErrorToStr(err));
    setError(planErrorToStr(err));
    return false;
  }

  gRobot.laneTracker.beginSession(&gRobot.activePlan);
  return true;
}

void startCleaning() {
  if (gRobot.unoFlashMode) return;
  memset(&gRobot.activePlan, 0, sizeof(gRobot.activePlan));
  gRobot.laneTracker.onCalibrationStarted();
  resetCalibration();
  gRobot.haveRoom = false;
  gRobot.errMsg = "";
  gRobot.manualUntil = 0;
  gRobot.mopPwm = 0;
  resetNavigationState();
  emergencyStopMotors();
  gRobot.state = ST_CALIB;
  Serial.println("Bat dau hieu chinh...");
}

void finishCleaning() {
  gRobot.mopPwm = 0;
  emergencyStopMotors();
  gRobot.laneTracker.onSessionFinished();
  gRobot.state = ST_DONE;
  Serial.printf("Ket thuc phien: %s\n", cleaningResultToStr(gRobot.laneTracker.getResult()));
}

void stopAll() {
  if (gRobot.state == ST_RUN || gRobot.state == ST_CALIB) {
    gRobot.laneTracker.onUserAborted();
  }
  gRobot.mopPwm = 0;
  gRobot.manualUntil = 0;
  gRobot.state = ST_IDLE;
  emergencyStopMotors();
}
