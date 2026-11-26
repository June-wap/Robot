#include "ArduinoLink.h"
#include "../config/HardwareConfig.h"
#include "../control/CommandHandler.h"
#include "../control/MotionController.h"
#include "../core/RobotContext.h"
#include <Arduino.h>

static int           lastSentMop = -1;
static unsigned long lastSendMs  = 0;
static unsigned long lastMopMs   = 0;
static unsigned long lastTelMs   = 0;

void arduinoLinkInit() {
  Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
}

void arduinoLinkSendImmediateStop() {
  if (!gRobot.unoFlashMode) {
    Serial1.print("M,0,0\n");
    lastSendMs = millis();
  }
}

void arduinoLinkSendOtaStop() {
  if (!gRobot.unoFlashMode) {
    Serial1.print("M,0,0\nW,0\n");
    Serial1.flush();
  }
}

void setUnoFlashMode(bool en) {
  if (en == gRobot.unoFlashMode) return;
  if (en) {
    gRobot.cmdL = gRobot.cmdR = gRobot.outL = gRobot.outR = 0;
    gRobot.mopPwm = 0;
    gRobot.manualUntil = 0;
    Serial1.print("M,0,0\nW,0\n");
    Serial1.flush();
    delay(20);
    Serial1.end();
    pinMode(LINK_TX_PIN, INPUT);
    pinMode(LINK_RX_PIN, INPUT);
    gRobot.unoFlashMode = true;
    Serial.println("UNO FLASH MODE: ON (GPIO18/19 = High-Z)");
  } else {
    Serial1.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
    gRobot.unoFlashMode = false;
    lastSentMop = -1;
    Serial.println("UNO FLASH MODE: OFF (UART1 resumed)");
  }
}

// Lệnh từ HC-05 (Arduino chuyển tiếp dạng "C,<ký tự>\n")
void pollLink() {
  if (gRobot.unoFlashMode) return;
  static char line[24];
  static int n = 0;
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      line[n] = 0;
      if (n >= 3 && line[0] == 'C' && line[1] == ',') {
        char k = line[2];
        if      (k == 'A') applyCmd("start", gRobot.cfgCruise);
        else if (k == 'X') applyCmd("stop", gRobot.cfgCruise);
        else if (k == 'F') applyCmd("F", gRobot.cfgCruise);
        else if (k == 'B') applyCmd("B", gRobot.cfgCruise);
        else if (k == 'L') applyCmd("L", gRobot.cfgCruise);
        else if (k == 'R') applyCmd("R", gRobot.cfgCruise);
        else if (k == 'S') applyCmd("S", gRobot.cfgCruise);
        else if (k == 'W') applyCmd("mopon", gRobot.cfgCruise);
        else if (k == 'w') applyCmd("mopoff", gRobot.cfgCruise);
      }
      n = 0;
    } else if (c != '\r' && n < (int)sizeof(line) - 1) {
      line[n++] = c;
    }
  }
}

void sendLink() {
  if (gRobot.unoFlashMode) return;
  unsigned long now = millis();

  // Cập nhật PWM thực tế qua bộ ramp mềm và gửi ở 20 Hz (50 ms)
  if (now - lastSendMs >= 50) {
    lastSendMs = now;
    gRobot.outL = rampAxis(gRobot.outL, gRobot.cmdL);
    gRobot.outR = rampAxis(gRobot.outR, gRobot.cmdR);
    Serial1.printf("M,%d,%d\n", gRobot.outL, gRobot.outR);
  }

  // Gửi lệnh chổi ngay khi thay đổi trạng thái hoặc mỗi 300 ms
  if (gRobot.mopPwm != lastSentMop || now - lastMopMs >= 300) {
    lastMopMs = now;
    lastSentMop = gRobot.mopPwm;
    Serial1.printf("W,%d\n", gRobot.mopPwm);
  }

  // Telemetry gọn (< 32 byte, gửi mỗi 2s lệch pha với gói M để tránh nghẽn SoftwareSerial của Uno)
  if (now - lastTelMs >= 2000 && now - lastSendMs >= 20) {
    lastTelMs = now;
    const char* nm[] = {"IDLE", "CAL", "RUN", "DONE", "ERR"};
    Serial1.printf("T,%s %d,%d %da %d/%d\n",
                   nm[gRobot.state], (int)gRobot.px_mm, (int)gRobot.py_mm, (int)gRobot.psi,
                   gRobot.stepIdx, (int)gRobot.activePlan.stepCount);
  }
}
