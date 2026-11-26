#include "MotionController.h"
#include "../config/RobotConfig.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"
#include <stdlib.h>

extern void arduinoLinkSendImmediateStop();

void drive(int l, int r) {
  gRobot.cmdL = clampi(l, -MAX_SAFE_PWM, MAX_SAFE_PWM);
  gRobot.cmdR = clampi(r, -MAX_SAFE_PWM, MAX_SAFE_PWM);
}

void stopMotors() {
  gRobot.cmdL = 0;
  gRobot.cmdR = 0;
}

// Dừng khẩn cấp: đưa output về 0 ngay lập tức (dùng khi va chạm, stopAll, OTA)
void emergencyStopMotors() {
  gRobot.cmdL = 0;
  gRobot.cmdR = 0;
  gRobot.outL = 0;
  gRobot.outR = 0;
  arduinoLinkSendImmediateStop();
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
