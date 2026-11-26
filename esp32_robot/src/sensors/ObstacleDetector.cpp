#include "ObstacleDetector.h"
#include "../config/RobotConfig.h"
#include "../core/RobotContext.h"
#include <math.h>
#include <stdint.h>

bool frontBlockedRaw() {
  if (gRobot.us[0] < US_BLOCK_MM) return true;
  uint16_t m = 9999;
  for (int a = -15; a <= 15; a++) {
    uint16_t d = gRobot.scan[(a + 360) % 360];
    if (d && d < m) m = d;
  }
  return m < LIDAR_BLOCK_MM;
}

// Khoảng hở dự kiến từ mép robot tới tường bên trái / bên phải theo hướng hiện tại
void expectedSideClearance(float heading, float& leftGap, float& rightGap) {
  if (!gRobot.haveRoom) { leftGap = rightGap = 9999.0f; return; }
  int h = ((int)roundf(heading / 90.0f) * 90 % 360 + 360) % 360;
  if (h == 0) {           // +Y
    leftGap  = gRobot.px_mm - ROBOT_RADIUS_MM;
    rightGap = (gRobot.roomW - gRobot.px_mm) - ROBOT_RADIUS_MM;
  } else if (h == 180) {  // -Y
    leftGap  = (gRobot.roomW - gRobot.px_mm) - ROBOT_RADIUS_MM;
    rightGap = gRobot.px_mm - ROBOT_RADIUS_MM;
  } else if (h == 90) {   // +X
    leftGap  = (gRobot.roomL - gRobot.py_mm) - ROBOT_RADIUS_MM;
    rightGap = gRobot.py_mm - ROBOT_RADIUS_MM;
  } else {                // 270 (-X)
    leftGap  = gRobot.py_mm - ROBOT_RADIUS_MM;
    rightGap = (gRobot.roomL - gRobot.py_mm) - ROBOT_RADIUS_MM;
  }
}
