#include "OtaManager.h"
#include "ArduinoLink.h"
#include "../config/NetworkConfig.h"
#include "../core/RobotContext.h"
#include <Arduino.h>
#include <ArduinoOTA.h>

void otaInit() {
  // Nạp code qua WiFi (Arduino IDE -> Tools -> Port -> robot)
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.setMdnsEnabled(false);
  ArduinoOTA.onStart([]() {
    gRobot.cmdL = gRobot.cmdR = gRobot.outL = gRobot.outR = 0;
    gRobot.mopPwm = 0;
    arduinoLinkSendOtaStop();
  });
  ArduinoOTA.begin();
}

void otaHandle() {
  ArduinoOTA.handle();
}
