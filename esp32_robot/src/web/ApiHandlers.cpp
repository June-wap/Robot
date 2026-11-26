#include "ApiHandlers.h"
#include "RobotWebServer.h"
#include "WebPage.h"
#include "../communication/ArduinoLink.h"
#include "../config/RobotConfig.h"
#include "../config/Settings.h"
#include "../control/CommandHandler.h"
#include "../core/RobotContext.h"
#include "../core/RobotTypes.h"
#include "../planning/LaneTypes.h"
#include <Arduino.h>
#include <stdio.h>

void handleRoot() {
  WebServer& server = getWebServer();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleStatus() {
  WebServer& server = getWebServer();
  const char* names[] = {"IDLE", "CALIB", "RUN", "DONE", "ERR"};
  String safeErr = gRobot.errMsg;
  safeErr.replace("\"", "'");
  CleaningProgress pr = gRobot.laneTracker.getProgress();
  const char* dirStr = gRobot.activePlan.valid ? directionToStr(gRobot.activePlan.chosenDirection)
                                               : directionToStr(gRobot.cfgDirection);
  const char* corStr = gRobot.activePlan.valid ? cornerToStr(gRobot.activePlan.startCorner) : "NONE";

  char b[768];
  int n = snprintf(b, sizeof(b),
                   "{\"state\":\"%s\",\"x\":%d,\"y\":%d,\"psi\":%d,\"W\":%d,\"L\":%d,"
                   "\"step\":%d,\"total\":%d,\"us0\":%d,\"us1\":%d,\"us2\":%d,"
                   "\"mop\":%d,\"oL\":%d,\"oR\":%d,\"uno\":%d,\"err\":\"%s\","
                   "\"cleaning_result\":\"%s\",\"cleaning_direction\":\"%s\",\"start_corner\":\"%s\","
                   "\"total_lanes\":%u,\"current_lane_id\":%d,\"completed_lanes\":%u,"
                   "\"blocked_lanes\":%u,\"partial_lanes\":%u,\"failed_lanes\":%u,\"pending_lanes\":%u,"
                   "\"processed_percent\":%.2f,\"completed_percent\":%.2f,\"estimated_distance_percent\":%.2f}",
                   names[gRobot.state], (int)gRobot.px_mm, (int)gRobot.py_mm, (int)gRobot.psi,
                   gRobot.haveRoom ? (int)gRobot.roomW : 0, gRobot.haveRoom ? (int)gRobot.roomL : 0,
                   gRobot.stepIdx, (int)gRobot.activePlan.stepCount, gRobot.us[0], gRobot.us[1], gRobot.us[2],
                   gRobot.mopPwm, gRobot.outL, gRobot.outR, gRobot.unoFlashMode ? 1 : 0, safeErr.c_str(),
                   cleaningResultToStr(gRobot.laneTracker.getResult()), dirStr, corStr,
                   (unsigned)pr.totalLanes, (int)pr.currentLaneId, (unsigned)pr.completedLanes,
                   (unsigned)pr.blockedLanes, (unsigned)pr.partialLanes, (unsigned)pr.failedLanes,
                   (unsigned)pr.pendingLanes,
                   pr.processedPercent, pr.completedPercent, pr.estimatedDistancePercent);
  server.sendHeader("Cache-Control", "no-store");
  if (n < 0 || n >= (int)sizeof(b)) {
    server.send(500, "application/json", "{\"error\":\"status_serialization_failed\"}");
    return;
  }
  server.send(200, "application/json", b);
}

void handleScan() {
  WebServer& server = getWebServer();
  String s;
  s.reserve(2200);
  s += '[';
  for (int i = 0; i < 360; i++) {
    if (i) s += ',';
    s += gRobot.scan[i];
  }
  s += ']';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// Trả 720 byte nhị phân (360 x uint16_t little-endian) cho Mobile App (Android Studio)
void handleScanBin() {
  WebServer& server = getWebServer();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(sizeof(gRobot.scan));
  server.send(200, "application/octet-stream", "");
  server.client().write((const uint8_t*)gRobot.scan, sizeof(gRobot.scan));
}

void handleCmd() {
  WebServer& server = getWebServer();
  int v = server.hasArg("v") ? server.arg("v").toInt() : gRobot.cfgCruise;
  applyCmd(server.arg("c"), v);
  server.send(200, "text/plain", "ok");
}

void handleUnoMode() {
  WebServer& server = getWebServer();
  if (server.hasArg("en")) {
    bool en = (server.arg("en").toInt() != 0);
    if (en && (gRobot.state == ST_RUN || gRobot.state == ST_CALIB)) {
      server.send(409, "text/plain", "busy");
      return;
    }
    setUnoFlashMode(en);
  }
  server.send(200, "text/plain", gRobot.unoFlashMode ? "1" : "0");
}

void handleSettings() {
  WebServer& server = getWebServer();
  bool hasAny = server.hasArg("mop") || server.hasArg("cruise") ||
                server.hasArg("edge") || server.hasArg("overlap") ||
                server.hasArg("direction");
  if (hasAny) {
    if (gRobot.state == ST_RUN || gRobot.state == ST_CALIB) {
      server.send(409, "text/plain", "busy");
      return;
    }

    float newMop = gRobot.cfgMopWidth;
    int   newCruise = gRobot.cfgCruise;
    int   newEdge = gRobot.cfgEdge;
    float newOverlap = gRobot.cfgOverlap;
    CleaningDirection newDir = gRobot.cfgDirection;

    if (server.hasArg("mop")) {
      if (!parseStrictFloat(server.arg("mop"), newMop) || newMop < 100.0f || newMop > 400.0f) {
        server.send(400, "text/plain", "invalid mop (100..400)");
        return;
      }
    }
    if (server.hasArg("cruise")) {
      if (!parseStrictInt(server.arg("cruise"), newCruise) || newCruise < 100 || newCruise > MAX_SAFE_PWM) {
        server.send(400, "text/plain", "invalid cruise");
        return;
      }
    }
    if (server.hasArg("edge")) {
      if (!parseStrictInt(server.arg("edge"), newEdge) || newEdge < ROBOT_RADIUS_MM + 10 || newEdge > 500) {
        server.send(400, "text/plain", "invalid edge");
        return;
      }
    }
    if (server.hasArg("overlap")) {
      if (!parseStrictFloat(server.arg("overlap"), newOverlap) ||
          newOverlap < MIN_OVERLAP_PERCENT || newOverlap > MAX_OVERLAP_PERCENT) {
        server.send(400, "text/plain", "invalid overlap (0..30)");
        return;
      }
    }
    if (server.hasArg("direction")) {
      if (!parseDirectionArg(server.arg("direction"), newDir)) {
        server.send(400, "text/plain", "invalid direction (AUTO|X|Y)");
        return;
      }
    }

    gRobot.cfgMopWidth  = newMop;
    gRobot.cfgCruise    = newCruise;
    gRobot.cfgEdge      = newEdge;
    gRobot.cfgOverlap   = newOverlap;
    gRobot.cfgDirection = newDir;

    saveSettings();
  }

  char b[160];
  int n = snprintf(b, sizeof(b),
                   "{\"mop\":%d,\"cruise\":%d,\"edge\":%d,\"overlap\":%.1f,\"direction\":\"%s\"}",
                   (int)gRobot.cfgMopWidth, gRobot.cfgCruise, gRobot.cfgEdge,
                   gRobot.cfgOverlap, directionToStr(gRobot.cfgDirection));
  if (n < 0 || n >= (int)sizeof(b)) {
    server.send(500, "application/json", "{\"error\":\"settings_serialization_failed\"}");
    return;
  }
  server.send(200, "application/json", b);
}

void handlePlan() {
  WebServer& server = getWebServer();
  String j;
  j.reserve(640 + (size_t)gRobot.activePlan.laneCount * 104u + 8u);

  char hdr[640];
  int hdrLen = snprintf(hdr, sizeof(hdr),
                        "{\"planner_version\":\"2.0\",\"valid\":%s,\"direction\":\"%s\",\"axis\":\"%s\","
                        "\"start_corner\":\"%s\",\"room_width_mm\":%.0f,\"room_length_mm\":%.0f,"
                        "\"mop_width_mm\":%.0f,\"edge_mm\":%.0f,\"edge\":%.0f,\"overlap_percent\":%.1f,"
                        "\"lane_spacing_mm\":%.2f,\"lane_count\":%u,"
                        "\"metrics\":{\"cleaning_distance_mm\":%.1f,\"transition_distance_mm\":%.1f,"
                        "\"approach_distance_mm\":%.1f,\"total_distance_mm\":%.1f,"
                        "\"turn_count\":%u,\"total_turn_degrees\":%.1f,\"estimated_cost\":%.2f},\"lanes\":[",
                        gRobot.activePlan.valid ? "true" : "false",
                        gRobot.activePlan.valid ? directionToStr(gRobot.activePlan.chosenDirection) : directionToStr(gRobot.cfgDirection),
                        gRobot.activePlan.valid ? directionToStr(gRobot.activePlan.chosenDirection) : directionToStr(gRobot.cfgDirection),
                        gRobot.activePlan.valid ? cornerToStr(gRobot.activePlan.startCorner) : "NONE",
                        gRobot.activePlan.valid ? gRobot.activePlan.roomW : 0.0f,
                        gRobot.activePlan.valid ? gRobot.activePlan.roomL : 0.0f,
                        gRobot.activePlan.valid ? gRobot.activePlan.mopWidthMm : gRobot.cfgMopWidth,
                        gRobot.activePlan.valid ? gRobot.activePlan.edgeMm : (float)gRobot.cfgEdge,
                        gRobot.activePlan.valid ? gRobot.activePlan.edgeMm : (float)gRobot.cfgEdge,
                        gRobot.activePlan.valid ? gRobot.activePlan.overlapPercent : gRobot.cfgOverlap,
                        gRobot.activePlan.valid ? gRobot.activePlan.actualSpacingMm : 0.0f,
                        gRobot.activePlan.valid ? (unsigned)gRobot.activePlan.laneCount : 0u,
                        gRobot.activePlan.metrics.cleaningDistanceMm,
                        gRobot.activePlan.metrics.transitionDistanceMm,
                        gRobot.activePlan.metrics.approachDistanceMm,
                        gRobot.activePlan.metrics.totalDistanceMm,
                        (unsigned)gRobot.activePlan.metrics.turnCount,
                        gRobot.activePlan.metrics.totalTurnDegrees,
                        gRobot.activePlan.metrics.estimatedCost);
  if (hdrLen < 0 || hdrLen >= (int)sizeof(hdr)) {
    server.sendHeader("Cache-Control", "no-store");
    server.send(500, "application/json", "{\"error\":\"plan_header_serialization_failed\"}");
    return;
  }
  j += hdr;

  if (gRobot.activePlan.valid) {
    char item[128];
    for (uint16_t i = 0; i < gRobot.activePlan.laneCount; i++) {
      const CleaningLane& ln = gRobot.activePlan.lanes[i];
      if (i > 0) j += ',';
      int itemLen = snprintf(item, sizeof(item),
                             "{\"id\":%u,\"start_x\":%.0f,\"start_y\":%.0f,\"end_x\":%.0f,\"end_y\":%.0f,\"status\":\"%s\"}",
                             (unsigned)ln.id, ln.startX, ln.startY, ln.endX, ln.endY, laneStatusToStr(ln.status));
      if (itemLen < 0 || itemLen >= (int)sizeof(item)) {
        server.sendHeader("Cache-Control", "no-store");
        server.send(500, "application/json", "{\"error\":\"plan_lane_serialization_failed\"}");
        return;
      }
      j += item;
    }
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}
