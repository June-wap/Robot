/*
 * ROBOT LAU NHÀ TỰ ĐỘNG - ESP32 (KIẾN TRÚC MODULAR GD01.2)
 * -----------------------------------------------------------------------------
 * Entry point tối giản tương thích Arduino IDE / Arduino CLI.
 * Toàn bộ logic được tổ chức trong esp32_robot/src/ theo từng module độc lập:
 *   config/         : HardwareConfig, RobotConfig, NetworkConfig, Settings (NVS)
 *   core/           : RobotApp, RobotTypes, RobotContext, MathUtils
 *   sensors/        : Lidar (LD14), Ultrasonic (ISR), ObstacleDetector
 *   localization/   : WallDetection (PCA/TLS), PoseEstimator, Calibration
 *   planning/       : LaneTypes, CoveragePlanner V2, LaneTracker
 *   navigation/     : Navigator, CleaningSession
 *   control/        : MotionController (PWM/Ramp), CommandHandler
 *   communication/  : ArduinoLink (UART1/High-Z), WiFiManager (STA/AP/mDNS), OtaManager
 *   web/            : RobotWebServer, ApiHandlers, WebPage (PROGMEM)
 *
 * Xem chi tiết sơ đồ chân, API contract và hướng dẫn build trong README.md và docs/.
 */

#include "src/core/RobotApp.h"

void setup() {
  robotSetup();
}

void loop() {
  robotLoop();
}
