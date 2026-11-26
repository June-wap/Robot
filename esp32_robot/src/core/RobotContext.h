#ifndef ROBOT_CONTEXT_H
#define ROBOT_CONTEXT_H

#include <Arduino.h>
#include <stdint.h>
#include "RobotTypes.h"
#include "../config/RobotConfig.h"
#include "../planning/LaneTypes.h"
#include "../planning/LaneTracker.h"

// Trạng thái dùng chung giữa các module của firmware ESP32.
// Các biến chỉ dùng nội bộ một module được giữ static trong file .cpp sở hữu.
struct RobotContext {
  // Robot state
  State  state;
  String errMsg;
  String myIP;

  // Planning & Lane Tracking (một thể hiện duy nhất, không sao chép)
  CoveragePlan activePlan;
  LaneTracker  laneTracker;
  int          stepIdx;

  // Localization state
  float px_mm;
  float py_mm;
  float psi;
  float roomW;
  float roomL;
  bool  haveRoom;

  // LiDAR shared scan state
  uint16_t      scan[360];
  volatile bool newScan;
  unsigned long lastScanMs;

  // Ultrasonic shared output state
  uint16_t us[3];
  bool     newUsFront;

  // Motion & Manual control state
  int           cmdL;
  int           cmdR;
  int           outL;
  int           outR;
  int           mopPwm;
  unsigned long manualUntil;
  int           manualL;
  int           manualR;

  // Arduino link mode
  bool unoFlashMode;

  // Runtime settings (đồng bộ với NVS Preferences)
  float             cfgMopWidth;
  int               cfgCruise;
  int               cfgEdge;
  float             cfgOverlap;
  CleaningDirection cfgDirection;

  RobotContext();
};

extern RobotContext gRobot;

#endif // ROBOT_CONTEXT_H
