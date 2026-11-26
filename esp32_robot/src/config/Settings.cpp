#include "Settings.h"
#include "RobotConfig.h"
#include "../core/MathUtils.h"
#include "../core/RobotContext.h"
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>

static Preferences prefs;

bool parseStrictFloat(const String& str, float& out) {
  if (str.length() == 0) return false;
  char* endp = nullptr;
  float v = strtof(str.c_str(), &endp);
  if (endp == str.c_str() || *endp != '\0' || !isfinite(v)) return false;
  out = v;
  return true;
}

bool parseStrictInt(const String& str, int& out) {
  if (str.length() == 0) return false;
  char* endp = nullptr;
  long v = strtol(str.c_str(), &endp, 10);
  if (endp == str.c_str() || *endp != '\0') return false;
  if (v < -32768L || v > 32767L) return false;
  out = (int)v;
  return true;
}

bool parseDirectionArg(const String& str, CleaningDirection& out) {
  if (str == "AUTO" || str == "auto" || str == "0") { out = DIRECTION_AUTO; return true; }
  if (str == "X"    || str == "x"    || str == "1") { out = DIRECTION_X;    return true; }
  if (str == "Y"    || str == "y"    || str == "2") { out = DIRECTION_Y;    return true; }
  return false;
}

void loadSettings() {
  prefs.begin("robot", true);
  gRobot.cfgMopWidth = clampf(prefs.getFloat("mop", DEF_MOP_WIDTH_MM), 100.0f, 400.0f);
  gRobot.cfgCruise   = clampi(prefs.getInt("cruise", DEF_CRUISE_PWM), 100, MAX_SAFE_PWM);
  gRobot.cfgEdge     = clampi(prefs.getInt("edge", DEF_EDGE_MM), ROBOT_RADIUS_MM + 10, 500);
  gRobot.cfgOverlap  = clampf(prefs.getFloat("overlap", DEF_OVERLAP_PCT), MIN_OVERLAP_PERCENT, MAX_OVERLAP_PERCENT);
  int dirInt         = prefs.getInt("dir", (int)DIRECTION_AUTO);
  if (dirInt >= (int)DIRECTION_AUTO && dirInt <= (int)DIRECTION_Y) {
    gRobot.cfgDirection = (CleaningDirection)dirInt;
  } else {
    gRobot.cfgDirection = DIRECTION_AUTO;
  }
  prefs.end();
}

void saveSettings() {
  prefs.begin("robot", false);
  prefs.putFloat("mop", gRobot.cfgMopWidth);
  prefs.putInt("cruise", gRobot.cfgCruise);
  prefs.putInt("edge", gRobot.cfgEdge);
  prefs.putFloat("overlap", gRobot.cfgOverlap);
  prefs.putInt("dir", (int)gRobot.cfgDirection);
  prefs.end();
}
