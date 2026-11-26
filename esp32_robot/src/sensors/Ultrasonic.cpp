#include "Ultrasonic.h"
#include "../config/HardwareConfig.h"
#include "../config/RobotConfig.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"

static volatile uint32_t usRiseUs[3]  = {0, 0, 0};
static volatile uint32_t usWidthUs[3] = {0, 0, 0};
static volatile bool     usHigh[3]    = {false, false, false};
static volatile bool     usDone[3]    = {false, false, false};

static uint16_t usHist[3][3] = {{9999, 9999, 9999}, {9999, 9999, 9999}, {9999, 9999, 9999}};
static uint8_t  usHistIdx[3] = {0, 0, 0};
static int      usTurn = 0;
static bool     usBusy = false;
static unsigned long lastUsMs = 0;
static unsigned long usTrigMs = 0;

static inline void IRAM_ATTR handleUsIsr(int idx) {
  if (digitalRead(US_ECHO[idx]) == HIGH) {
    usRiseUs[idx] = micros();
    usHigh[idx] = true;
  } else if (usHigh[idx]) {
    usWidthUs[idx] = micros() - usRiseUs[idx];
    usHigh[idx] = false;
    usDone[idx] = true;
  }
}

void IRAM_ATTR isrUs0() { handleUsIsr(0); }
void IRAM_ATTR isrUs1() { handleUsIsr(1); }
void IRAM_ATTR isrUs2() { handleUsIsr(2); }

void ultrasonicInit() {
  for (int i = 0; i < 3; i++) {
    pinMode(US_TRIG[i], OUTPUT);
    digitalWrite(US_TRIG[i], LOW);
    pinMode(US_ECHO[i], INPUT);
  }
  attachInterrupt(digitalPinToInterrupt(US_ECHO[0]), isrUs0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO[1]), isrUs1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(US_ECHO[2]), isrUs2, CHANGE);
}

void pushUsSample(int idx, uint16_t mm) {
  uint8_t p = usHistIdx[idx];
  usHist[idx][p] = mm;
  usHistIdx[idx] = (p + 1) % 3;
  gRobot.us[idx] = median3(usHist[idx][0], usHist[idx][1], usHist[idx][2]);
  if (idx == 0) gRobot.newUsFront = true;
}

void pollUltrasonic() {
  unsigned long now = millis();
  if (!usBusy) {
    if (now - lastUsMs < US_PERIOD_MS) return;
    lastUsMs = now;
    noInterrupts();
    usDone[usTurn] = false;
    usHigh[usTurn] = false;
    usWidthUs[usTurn] = 0;
    interrupts();

    digitalWrite(US_TRIG[usTurn], LOW);
    delayMicroseconds(2);
    digitalWrite(US_TRIG[usTurn], HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG[usTurn], LOW);
    usTrigMs = now;
    usBusy = true;
    return;
  }

  if (usDone[usTurn]) {
    noInterrupts();
    uint32_t w = usWidthUs[usTurn];
    usDone[usTurn] = false;
    interrupts();
    uint16_t mm = (w >= 115 && w <= 12000) ? (uint16_t)(w * 0.1715f) : 9999;
    pushUsSample(usTurn, mm);
    usTurn = (usTurn + 1) % 3;
    usBusy = false;
  } else if (now - usTrigMs >= US_TIMEOUT_MS) {
    noInterrupts();
    usHigh[usTurn] = false;
    usDone[usTurn] = false;
    interrupts();
    pushUsSample(usTurn, 9999);
    usTurn = (usTurn + 1) % 3;
    usBusy = false;
  }
}
