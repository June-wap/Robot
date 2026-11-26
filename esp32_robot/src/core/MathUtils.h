#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>

float    wrap180(float a);
float    wrap90(float a);
int      clampi(int v, int lo, int hi);
float    clampf(float v, float lo, float hi);
uint16_t median3(uint16_t a, uint16_t b, uint16_t c);

#endif // MATH_UTILS_H
