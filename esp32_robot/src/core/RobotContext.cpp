#include "RobotContext.h"
#include <string.h>

RobotContext::RobotContext()
  : state(ST_IDLE),
    errMsg(""),
    myIP(""),
    activePlan(),
    laneTracker(),
    stepIdx(0),
    px_mm(0.0f),
    py_mm(0.0f),
    psi(0.0f),
    roomW(0.0f),
    roomL(0.0f),
    haveRoom(false),
    newScan(false),
    lastScanMs(0),
    newUsFront(false),
    cmdL(0),
    cmdR(0),
    outL(0),
    outR(0),
    mopPwm(0),
    manualUntil(0),
    manualL(0),
    manualR(0),
    unoFlashMode(false),
    cfgMopWidth(DEF_MOP_WIDTH_MM),
    cfgCruise(DEF_CRUISE_PWM),
    cfgEdge(DEF_EDGE_MM),
    cfgOverlap(DEF_OVERLAP_PCT),
    cfgDirection(DIRECTION_AUTO) {
  memset(&activePlan, 0, sizeof(activePlan));
  memset(scan, 0, sizeof(scan));
  us[0] = 9999;
  us[1] = 9999;
  us[2] = 9999;
}

RobotContext gRobot;
