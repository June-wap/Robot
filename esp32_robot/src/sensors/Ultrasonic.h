#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include <stdint.h>

void ultrasonicInit();
void IRAM_ATTR isrUs0();
void IRAM_ATTR isrUs1();
void IRAM_ATTR isrUs2();
void pushUsSample(int idx, uint16_t mm);
void pollUltrasonic();

#endif // ULTRASONIC_H
