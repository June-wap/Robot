#include "Lidar.h"
#include "../config/HardwareConfig.h"
#include "../core/RobotContext.h"
#include <Arduino.h>
#include <string.h>

static uint16_t acc[360];
static float    lastStartAngle = 0.0f;
static uint8_t  crcTable[256];

void lidarBeginSerial() {
  Serial2.setRxBufferSize(LIDAR_RX_BUF_SIZE);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, -1);
}

void lidarInit() {
  initCrc();
  memset(acc, 0, sizeof(acc));
  memset(gRobot.scan, 0, sizeof(gRobot.scan));
}

void initCrc() {
  for (int i = 0; i < 256; i++) {
    uint8_t c = (uint8_t)i;
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x4D) : (uint8_t)(c << 1);
    crcTable[i] = c;
  }
}

uint8_t calcCrc(const uint8_t* p, int len) {
  uint8_t c = 0;
  for (int i = 0; i < len; i++) c = crcTable[(c ^ p[i]) & 0xFF];
  return c;
}

void handlePacket(const uint8_t* b) {
  float startA = (b[4] | (b[5] << 8)) / 100.0f;
  float endA   = (b[42] | (b[43] << 8)) / 100.0f;

  if (startA < lastStartAngle - 180.0f) {
    memcpy(gRobot.scan, acc, sizeof(acc));
    memset(acc, 0, sizeof(acc));
    gRobot.newScan = true;
    gRobot.lastScanMs = millis();
  }
  lastStartAngle = startA;

  if (endA < startA) endA += 360.0f;
  float step = (endA - startA) / 11.0f;

  for (int i = 0; i < 12; i++) {
    uint16_t d = b[6 + 3 * i] | (b[7 + 3 * i] << 8);
    uint8_t inten = b[8 + 3 * i];
    if (d < LIDAR_MIN_MM || d > LIDAR_MAX_MM || inten < LIDAR_MIN_INTENS) continue;
    float ang = startA + step * i;
    int deg = ((int)(ang + 0.5f) + LIDAR_OFFSET_DEG) % 360;
    if (deg < 0) deg += 360;
    if (!LIDAR_CLOCKWISE) deg = (360 - deg) % 360;
    acc[deg] = d;
  }
}

void pollLidar() {
  static uint8_t buf[47];
  static int idx = 0;
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    if (idx == 0) { if (b != 0x54) continue; }
    else if (idx == 1) { if (b != 0x2C) { idx = 0; continue; } }
    buf[idx++] = b;
    if (idx == 47) {
      idx = 0;
      if (calcCrc(buf, 46) == buf[46]) handlePacket(buf);
    }
  }
}
