#include "PoseEstimator.h"
#include "WallDetection.h"
#include "../config/RobotConfig.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"
#include "../core/RobotTypes.h"
#include <Arduino.h>
#include <math.h>

static unsigned long lastPoseMs = 0;

void poseInitTimer() {
  lastPoseMs = millis();
}

void predictPose() {
  unsigned long now = millis();
  float dt = (now - lastPoseMs) / 1000.0f;
  lastPoseMs = now;
  if (dt <= 0 || dt > 0.5f) return;
  // Dùng outL/outR (PWM thực tế sau ramp) để ước lượng động học sát hơn
  float v = (gRobot.outL + gRobot.outR) / 510.0f * V_MAX_MM_S;
  float w = (gRobot.outL - gRobot.outR) / 510.0f * SPIN_MAX_DEG_S;
  float mid = gRobot.psi + 0.5f * w * dt;
  gRobot.px_mm += v * dt * sinf(radians(mid));
  gRobot.py_mm += v * dt * cosf(radians(mid));
  gRobot.psi = wrap180(gRobot.psi + w * dt);
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
  float meas = gRobot.psi + wrap90(eps - gRobot.psi);
  gRobot.psi = wrap180(gRobot.psi + K_YAW * (meas - gRobot.psi));

  if (!gRobot.haveRoom) return;
  float psq = roundf(gRobot.psi / 90.0f) * 90.0f;
  if (fabsf(gRobot.psi - psq) > 12.0f) return;

  float xs = 0, ys = 0;
  int nx = 0, ny = 0;
  for (int k = 0; k < 4; k++) {
    if (!w[k].ok) continue;
    int q = ((al[k] + (int)psq) % 360 + 360) % 360;
    float d = w[k].dist;
    if      (q == 0)   { ys += gRobot.roomL - d; ny++; }
    else if (q == 90)  { xs += gRobot.roomW - d; nx++; }
    else if (q == 180) { ys += d;                ny++; }
    else               { xs += d;                nx++; }
  }
  if (nx) {
    float mxv = xs / nx;
    if (fabsf(mxv - gRobot.px_mm) < POS_JUMP_MAX_MM) gRobot.px_mm += K_POS * (mxv - gRobot.px_mm);
  }
  if (ny) {
    float myv = ys / ny;
    if (fabsf(myv - gRobot.py_mm) < POS_JUMP_MAX_MM) gRobot.py_mm += K_POS * (myv - gRobot.py_mm);
  }
}
