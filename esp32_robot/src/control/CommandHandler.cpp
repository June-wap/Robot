#include "CommandHandler.h"
#include "MotionController.h"
#include "../config/RobotConfig.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"
#include "../core/RobotTypes.h"

extern void startCleaning();
extern void stopAll();

void applyCmd(const String& c, int v) {
  if (gRobot.unoFlashMode) return;
  v = clampi(v, 80, MAX_SAFE_PWM);
  if (c == "start")       startCleaning();
  else if (c == "stop")   stopAll();
  else if (c == "mopon")  { if (gRobot.state != ST_RUN) gRobot.mopPwm = MOP_PWM; }
  else if (c == "mopoff") { if (gRobot.state != ST_RUN) gRobot.mopPwm = 0; }
  else if (c == "S")      { gRobot.manualUntil = 0; stopMotors(); }
  else if (gRobot.state != ST_RUN && gRobot.state != ST_CALIB) {
    if      (c == "F") { gRobot.manualL =  v; gRobot.manualR =  v; gRobot.manualUntil = millis() + 400; }
    else if (c == "B") { gRobot.manualL = -v; gRobot.manualR = -v; gRobot.manualUntil = millis() + 400; }
    else if (c == "L") { gRobot.manualL = -v; gRobot.manualR =  v; gRobot.manualUntil = millis() + 400; }
    else if (c == "R") { gRobot.manualL =  v; gRobot.manualR = -v; gRobot.manualUntil = millis() + 400; }
  }
}
