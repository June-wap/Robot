#include "RobotApp.h"
#include "RobotContext.h"
#include "RobotTypes.h"
#include "../communication/ArduinoLink.h"
#include "../communication/OtaManager.h"
#include "../communication/WiFiManager.h"
#include "../config/RobotConfig.h"
#include "../config/Settings.h"
#include "../control/MotionController.h"
#include "../localization/Calibration.h"
#include "../localization/PoseEstimator.h"
#include "../navigation/CleaningSession.h"
#include "../navigation/Navigator.h"
#include "../sensors/Lidar.h"
#include "../sensors/Ultrasonic.h"
#include "../web/RobotWebServer.h"
#include <Arduino.h>

void robotSetup() {
  Serial.begin(115200);
  loadSettings();

  arduinoLinkInit();
  lidarBeginSerial();
  ultrasonicInit();
  lidarInit();

  startWiFi();
  webServerInit();

  otaInit();

  poseInitTimer();
  gRobot.lastScanMs = millis();
}

void robotLoop() {
  webServerHandleClient();
  otaHandle();
  pollLink();
  pollLidar();
  pollUltrasonic();
  predictPose();

  bool fresh = gRobot.newScan;
  if (fresh) gRobot.newScan = false;
  bool usTick = gRobot.newUsFront;
  if (usTick) gRobot.newUsFront = false;

  switch (gRobot.state) {
    case ST_IDLE:
    case ST_DONE:
    case ST_ERR:
      if (!gRobot.unoFlashMode && millis() < gRobot.manualUntil) {
        int l = gRobot.manualL, r = gRobot.manualR;
        if (l > 0 && r > 0 && gRobot.us[0] < US_BLOCK_MM) {
          emergencyStopMotors();
        } else {
          drive(l, r);
        }
      } else {
        stopMotors();
      }
      break;

    case ST_CALIB:
      stopMotors();
      if (fresh) doCalibrate();
      break;

    case ST_RUN:
      if (millis() - gRobot.lastScanMs > 1000) {
        emergencyStopMotors();
        if (millis() - gRobot.lastScanMs > 5000) setError("Mat LiDAR");
        break;
      }
      if (fresh) correctPose();
      runNav(fresh || usTick);
      break;
  }

  sendLink();
}
