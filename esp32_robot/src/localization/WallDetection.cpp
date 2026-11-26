#include "WallDetection.h"
#include "../config/RobotConfig.h"
#include "../core/RobotContext.h"
#include <Arduino.h>
#include <math.h>
#include <stdint.h>

Wall fitWall(int center) {
  Wall w = {false, 0, 0};
  float X[64], Y[64];
  int n = 0;
  for (int a = -WALL_HALF_DEG; a <= WALL_HALF_DEG && n < 64; a++) {
    uint16_t d = gRobot.scan[((center + a) % 360 + 360) % 360];
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
