#include "MathUtils.h"
#include <math.h>

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

float wrap90(float a) {
  a = fmodf(a, 90.0f);
  if (a > 45.0f) a -= 90.0f;
  if (a < -45.0f) a += 90.0f;
  return a;
}

int clampi(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}
