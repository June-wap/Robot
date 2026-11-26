#include "Calibration.h"
#include "WallDetection.h"
#include "../config/RobotConfig.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"
#include "../core/RobotTypes.h"
#include <Arduino.h>
#include <math.h>

extern bool buildPlan();
extern void setError(const char* m);
extern void resetNavigationState();

static int   calibOk = 0;
static int   calibFail = 0;
static float calibD[4] = {0, 0, 0, 0};
static float calibSin = 0;
static float calibCos = 0;

void resetCalibration() {
  calibOk = 0;
  calibFail = 0;
  calibSin = 0.0f;
  calibCos = 0.0f;
  for (int k = 0; k < 4; k++) calibD[k] = 0.0f;
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
  gRobot.psi = eps;
  gRobot.roomW = left + right;
  gRobot.roomL = front + back;
  gRobot.px_mm = left;
  gRobot.py_mm = back;
  gRobot.haveRoom = true;

  if (!buildPlan()) return;
  resetNavigationState();
  gRobot.mopPwm = MOP_PWM;
  gRobot.state = ST_RUN;
  Serial.printf("Phong: %.0f x %.0f mm | Huong %s | Goc %s | %u lan | %u buoc\n",
                gRobot.roomW, gRobot.roomL, directionToStr(gRobot.activePlan.chosenDirection),
                cornerToStr(gRobot.activePlan.startCorner),
                gRobot.activePlan.laneCount, gRobot.activePlan.stepCount);
}
