#include "CoveragePlanner.h"
#include <cmath>
#include <cstring>

static inline bool isFiniteFloat(float v) {
  return std::isfinite(v);
}

static inline float clampFloat(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline bool isValidDirection(CleaningDirection d) {
  return (d == DIRECTION_X || d == DIRECTION_Y);
}

static inline bool isValidStartCorner(StartCorner c) {
  return (c == CORNER_BOTTOM_LEFT || c == CORNER_BOTTOM_RIGHT ||
          c == CORNER_TOP_LEFT || c == CORNER_TOP_RIGHT);
}

static inline bool isValidLaneStatus(LaneStatus s) {
  return (s == LANE_PENDING || s == LANE_RUNNING || s == LANE_COMPLETED ||
          s == LANE_BLOCKED || s == LANE_PARTIAL || s == LANE_FAILED);
}

static inline bool isValidStepType(StepType t) {
  return (t == GO_X || t == GO_Y);
}

static inline bool isValidStepPurpose(StepPurpose p) {
  return (p == APPROACH_START || p == CLEAN_LANE || p == CHANGE_LANE);
}

// Chuẩn hóa góc về (-180, 180] bằng fmodf (O(1), không treo khi |deg| cực lớn)
float CoveragePlanner::wrapAngle180(float deg) {
  if (!isFiniteFloat(deg)) return 0.0f;
  deg = fmodf(deg, 360.0f);
  if (deg > 180.0f)  deg -= 360.0f;
  if (deg <= -180.0f) deg += 360.0f;
  return deg;
}

void CoveragePlanner::clearPlan(CoveragePlan& plan, PlanError err) {
  std::memset(&plan, 0, sizeof(CoveragePlan));
  plan.valid = false;
  plan.error = err;
  plan.chosenDirection = DIRECTION_Y;
  plan.startCorner = CORNER_BOTTOM_LEFT;
}

void CoveragePlanner::getCornerCoords(float roomW, float roomL, float edgeMm,
                                      StartCorner corner, float& outX, float& outY) {
  const float xMin = edgeMm;
  const float xMax = roomW - edgeMm;
  const float yMin = edgeMm;
  const float yMax = roomL - edgeMm;

  switch (corner) {
    case CORNER_BOTTOM_LEFT:  outX = xMin; outY = yMin; break;
    case CORNER_BOTTOM_RIGHT: outX = xMax; outY = yMin; break;
    case CORNER_TOP_LEFT:     outX = xMin; outY = yMax; break;
    case CORNER_TOP_RIGHT:    outX = xMax; outY = yMax; break;
    default:                  outX = xMin; outY = yMin; break;
  }
}

float CoveragePlanner::computeApproachManhattan(const PlanRequest& req, StartCorner corner) {
  float sx = 0.0f, sy = 0.0f;
  getCornerCoords(req.roomW, req.roomL, req.edgeMm, corner, sx, sy);
  return fabsf(req.robotX - sx) + fabsf(req.robotY - sy);
}

PlanError CoveragePlanner::validateRequest(const PlanRequest& req) {
  if (!isFiniteFloat(req.roomW) || !isFiniteFloat(req.roomL) ||
      req.roomW <= 0.0f || req.roomL <= 0.0f) {
    return PLAN_ERR_INVALID_ROOM;
  }

  if (!isFiniteFloat(req.mopWidthMm) || req.mopWidthMm <= 0.0f) {
    return PLAN_ERR_INVALID_MOP_WIDTH;
  }

  if (!isFiniteFloat(req.robotRadiusMm) || req.robotRadiusMm <= 0.0f ||
      !isFiniteFloat(req.edgeMm) || req.edgeMm < req.robotRadiusMm) {
    return PLAN_ERR_INVALID_EDGE;
  }

  if (!isFiniteFloat(req.overlapPercent) ||
      req.overlapPercent < MIN_OVERLAP_PERCENT ||
      req.overlapPercent > MAX_OVERLAP_PERCENT) {
    return PLAN_ERR_INVALID_OVERLAP;
  }

  const float effSpacing = req.mopWidthMm * (1.0f - req.overlapPercent / 100.0f);
  if (!isFiniteFloat(effSpacing) || effSpacing <= 0.0f) {
    return PLAN_ERR_INVALID_MOP_WIDTH;
  }

  if (!isFiniteFloat(req.robotX) || !isFiniteFloat(req.robotY) ||
      !isFiniteFloat(req.robotPsiDeg) ||
      req.robotX < 0.0f || req.robotX > req.roomW ||
      req.robotY < 0.0f || req.robotY > req.roomL) {
    return PLAN_ERR_INVALID_POSE;
  }

  if (req.requestedDirection != DIRECTION_AUTO &&
      req.requestedDirection != DIRECTION_X &&
      req.requestedDirection != DIRECTION_Y) {
    return PLAN_ERR_INVALID_DIRECTION;
  }

  if (!isFiniteFloat(req.nominalLinearSpeedMmS) || req.nominalLinearSpeedMmS <= 0.0f ||
      !isFiniteFloat(req.nominalAngularSpeedDegS) || req.nominalAngularSpeedDegS <= 0.0f ||
      !isFiniteFloat(req.laneChangePenaltySec) || req.laneChangePenaltySec < 0.0f) {
    return PLAN_ERR_INVALID_KINEMATICS;
  }

  const float twoEdge = 2.0f * req.edgeMm;
  if (!isFiniteFloat(twoEdge)) {
    return PLAN_ERR_INVALID_EDGE;
  }

  const float workW = req.roomW - twoEdge;
  const float workL = req.roomL - twoEdge;
  if (!isFiniteFloat(workW) || !isFiniteFloat(workL)) {
    return PLAN_ERR_INVALID_ROOM;
  }

  // Vùng làm việc phải không âm ở cả 2 trục (không giảm edge để ép kế hoạch hợp lệ)
  if (workW < -1e-3f || workL < -1e-3f) {
    return PLAN_ERR_ROOM_TOO_SMALL;
  }

  // Phải có ít nhất một trục đủ chiều dài tối thiểu cho hướng lau được yêu cầu
  const bool canCleanX = (workW >= PLANNER_MIN_LANE_LENGTH_MM && workL >= -1e-3f);
  const bool canCleanY = (workL >= PLANNER_MIN_LANE_LENGTH_MM && workW >= -1e-3f);

  if (req.requestedDirection == DIRECTION_X && !canCleanX) {
    return PLAN_ERR_ROOM_TOO_SMALL;
  }
  if (req.requestedDirection == DIRECTION_Y && !canCleanY) {
    return PLAN_ERR_ROOM_TOO_SMALL;
  }
  if (req.requestedDirection == DIRECTION_AUTO && !canCleanX && !canCleanY) {
    return PLAN_ERR_ROOM_TOO_SMALL;
  }

  return PLAN_OK;
}

PlanError CoveragePlanner::computeLaneGrid(float crossSpan, float effSpacing,
                                           uint16_t& outLaneCount, float& outActualSpacing) {
  if (!isFiniteFloat(crossSpan) || !isFiniteFloat(effSpacing) || effSpacing <= 0.0f) {
    return PLAN_ERR_INVALID_ROOM;
  }

  if (crossSpan > 1e-3f) {
    const float rawRatio = crossSpan / effSpacing;
    if (!isFiniteFloat(rawRatio)) {
      // crossSpan dương / effSpacing dương bị tràn +Inf -> chắc chắn vượt MAX_LANES
      return PLAN_ERR_EXCEEDS_MAX_LANES;
    }
    if (rawRatio < 0.0f) {
      return PLAN_ERR_INVALID_ROOM;
    }
    // Kiểm tra chặn tràn TRƯỚC khi gọi ceilf() và ép kiểu sang số nguyên
    const float maxIntervals = (float)(MAX_LANES - 1u);
    if (rawRatio > maxIntervals) {
      return PLAN_ERR_EXCEEDS_MAX_LANES;
    }
    const float ceilVal = ceilf(rawRatio);
    if (!isFiniteFloat(ceilVal) || ceilVal < 0.0f || ceilVal > maxIntervals) {
      return PLAN_ERR_EXCEEDS_MAX_LANES;
    }
    const uint32_t laneCount32 = (uint32_t)ceilVal + 1u;
    if (laneCount32 == 0u || laneCount32 > MAX_LANES) {
      return PLAN_ERR_EXCEEDS_MAX_LANES;
    }
    const float actualSpacing = crossSpan / (float)(laneCount32 - 1u);
    if (!isFiniteFloat(actualSpacing) || actualSpacing <= 0.0f) {
      return PLAN_ERR_INVALID_ROOM;
    }
    outLaneCount = (uint16_t)laneCount32;
    outActualSpacing = actualSpacing;
  } else {
    // Phòng hẹp suy biến thành đúng 1 làn hợp lệ (không chia cho 0)
    outLaneCount = 1u;
    outActualSpacing = 0.0f;
  }

  return PLAN_OK;
}

PlanError CoveragePlanner::computeMetricsFromSteps(const PlanRequest& req, CoveragePlan& plan) {
  PlanMetrics m;
  std::memset(&m, 0, sizeof(PlanMetrics));
  m.laneCount = plan.laneCount;

  float cx = req.robotX;
  float cy = req.robotY;
  float cPsi = wrapAngle180(req.robotPsiDeg);

  for (uint16_t i = 0; i < plan.stepCount; i++) {
    const Step& st = plan.steps[i];
    float dist = 0.0f;
    float tgtHeading = cPsi;

    if (st.type == GO_X) {
      dist = fabsf(st.val - cx);
      if (!isFiniteFloat(dist)) return PLAN_ERR_INVALID_ROOM;
      if (dist < 1e-3f) continue;
      tgtHeading = (st.val > cx) ? 90.0f : -90.0f;
      cx = st.val;
    } else {
      dist = fabsf(st.val - cy);
      if (!isFiniteFloat(dist)) return PLAN_ERR_INVALID_ROOM;
      if (dist < 1e-3f) continue;
      tgtHeading = (st.val > cy) ? 0.0f : 180.0f;
      cy = st.val;
    }

    const float deltaAngle = fabsf(wrapAngle180(tgtHeading - cPsi));
    if (deltaAngle > PLANNER_TURN_TOL_DEG) {
      m.turnCount++;
      m.totalTurnDegrees += deltaAngle;
      cPsi = tgtHeading;
    }

    switch (st.purpose) {
      case APPROACH_START: m.approachDistanceMm   += dist; break;
      case CLEAN_LANE:     m.cleaningDistanceMm   += dist; break;
      case CHANGE_LANE:    m.transitionDistanceMm += dist; break;
    }
  }

  m.totalDistanceMm = m.approachDistanceMm + m.cleaningDistanceMm + m.transitionDistanceMm;
  if (!isFiniteFloat(m.cleaningDistanceMm) || !isFiniteFloat(m.transitionDistanceMm) ||
      !isFiniteFloat(m.approachDistanceMm) || !isFiniteFloat(m.totalDistanceMm) ||
      !isFiniteFloat(m.totalTurnDegrees)) {
    return PLAN_ERR_INVALID_ROOM;
  }

  const uint16_t laneChanges = (m.laneCount > 1) ? (uint16_t)(m.laneCount - 1) : 0;
  m.estimatedCost = (m.totalDistanceMm / req.nominalLinearSpeedMmS) +
                    (m.totalTurnDegrees / req.nominalAngularSpeedDegS) +
                    (req.laneChangePenaltySec * (float)laneChanges);

  if (!isFiniteFloat(m.estimatedCost) || m.estimatedCost < 0.0f) {
    return PLAN_ERR_INVALID_KINEMATICS;
  }

  plan.metrics = m;
  return PLAN_OK;
}

// Đánh giá 1 candidate hoàn toàn không cấp phát mảng lanes[] / steps[] trên stack
PlanError CoveragePlanner::evaluateCandidateMetrics(const PlanRequest& req,
                                                    CleaningDirection dir,
                                                    StartCorner corner,
                                                    bool approachYFirst,
                                                    PlanMetrics& outMetrics) {
  std::memset(&outMetrics, 0, sizeof(PlanMetrics));

  if (!isValidDirection(dir)) return PLAN_ERR_INVALID_DIRECTION;
  if (!isValidStartCorner(corner)) return PLAN_ERR_INVALID_POSE;

  const float xMin = req.edgeMm;
  const float xMax = req.roomW - req.edgeMm;
  const float yMin = req.edgeMm;
  const float yMax = req.roomL - req.edgeMm;

  const bool alongY = (dir == DIRECTION_Y);
  const float longMin  = alongY ? yMin : xMin;
  const float longMax  = alongY ? yMax : xMax;
  const float crossMin = alongY ? xMin : yMin;
  const float crossMax = alongY ? xMax : yMax;

  const float longSpan  = longMax - longMin;
  const float crossSpan = crossMax - crossMin;

  if (!isFiniteFloat(longSpan) || !isFiniteFloat(crossSpan)) {
    return PLAN_ERR_INVALID_ROOM;
  }
  if (longSpan < PLANNER_MIN_LANE_LENGTH_MM || crossSpan < -1e-3f) {
    return PLAN_ERR_ROOM_TOO_SMALL;
  }

  const float effSpacing = req.mopWidthMm * (1.0f - req.overlapPercent / 100.0f);
  if (!isFiniteFloat(effSpacing) || effSpacing <= 0.0f) {
    return PLAN_ERR_INVALID_MOP_WIDTH;
  }

  uint16_t laneCount = 0;
  float actualSpacing = 0.0f;
  const PlanError gridErr = computeLaneGrid(crossSpan, effSpacing, laneCount, actualSpacing);
  if (gridErr != PLAN_OK) return gridErr;

  float startX = 0.0f, startY = 0.0f;
  getCornerCoords(req.roomW, req.roomL, req.edgeMm, corner, startX, startY);

  const float dxApp = fabsf(startX - req.robotX);
  const float dyApp = fabsf(startY - req.robotY);
  if (!isFiniteFloat(dxApp) || !isFiniteFloat(dyApp)) {
    return PLAN_ERR_INVALID_POSE;
  }

  uint32_t stepCount = 0;
  if (dxApp >= PLANNER_ZERO_STEP_EPS_MM) stepCount++;
  if (dyApp >= PLANNER_ZERO_STEP_EPS_MM) stepCount++;
  stepCount += (uint32_t)(2u * (uint32_t)laneCount - 1u);
  if (stepCount > MAX_PLAN_STEPS) {
    return PLAN_ERR_EXCEEDS_MAX_STEPS;
  }

  PlanMetrics m;
  std::memset(&m, 0, sizeof(PlanMetrics));
  m.laneCount = laneCount;

  float cx = req.robotX;
  float cy = req.robotY;
  float cPsi = wrapAngle180(req.robotPsiDeg);

  auto simStep = [&](StepType t, float val, StepPurpose p) -> bool {
    float dist = 0.0f;
    float tgtHeading = cPsi;
    if (t == GO_X) {
      dist = fabsf(val - cx);
      if (!isFiniteFloat(dist)) return false;
      if (dist < 1e-3f) return true;
      tgtHeading = (val > cx) ? 90.0f : -90.0f;
      cx = val;
    } else {
      dist = fabsf(val - cy);
      if (!isFiniteFloat(dist)) return false;
      if (dist < 1e-3f) return true;
      tgtHeading = (val > cy) ? 0.0f : 180.0f;
      cy = val;
    }

    const float deltaAngle = fabsf(wrapAngle180(tgtHeading - cPsi));
    if (deltaAngle > PLANNER_TURN_TOL_DEG) {
      m.turnCount++;
      m.totalTurnDegrees += deltaAngle;
      cPsi = tgtHeading;
    }

    switch (p) {
      case APPROACH_START: m.approachDistanceMm   += dist; break;
      case CLEAN_LANE:     m.cleaningDistanceMm   += dist; break;
      case CHANGE_LANE:    m.transitionDistanceMm += dist; break;
    }
    return true;
  };

  if (approachYFirst) {
    if (dyApp >= PLANNER_ZERO_STEP_EPS_MM && !simStep(GO_Y, startY, APPROACH_START)) {
      return PLAN_ERR_INVALID_ROOM;
    }
    if (dxApp >= PLANNER_ZERO_STEP_EPS_MM && !simStep(GO_X, startX, APPROACH_START)) {
      return PLAN_ERR_INVALID_ROOM;
    }
  } else {
    if (dxApp >= PLANNER_ZERO_STEP_EPS_MM && !simStep(GO_X, startX, APPROACH_START)) {
      return PLAN_ERR_INVALID_ROOM;
    }
    if (dyApp >= PLANNER_ZERO_STEP_EPS_MM && !simStep(GO_Y, startY, APPROACH_START)) {
      return PLAN_ERR_INVALID_ROOM;
    }
  }

  const bool startAtCrossMin = alongY
      ? (corner == CORNER_BOTTOM_LEFT || corner == CORNER_TOP_LEFT)
      : (corner == CORNER_BOTTOM_LEFT || corner == CORNER_BOTTOM_RIGHT);
  const bool startAtLongMin = alongY
      ? (corner == CORNER_BOTTOM_LEFT || corner == CORNER_BOTTOM_RIGHT)
      : (corner == CORNER_BOTTOM_LEFT || corner == CORNER_TOP_LEFT);

  const StepType longType  = alongY ? GO_Y : GO_X;
  const StepType crossType = alongY ? GO_X : GO_Y;

  for (uint16_t i = 0; i < laneCount; i++) {
    float crossCoord = crossMin;
    if (laneCount > 1) {
      if (i == laneCount - 1) {
        crossCoord = startAtCrossMin ? crossMax : crossMin;
      } else {
        crossCoord = startAtCrossMin
            ? (crossMin + (float)i * actualSpacing)
            : (crossMax - (float)i * actualSpacing);
      }
      crossCoord = clampFloat(crossCoord, crossMin, crossMax);
    }

    const bool goForwardLong = ((i % 2) == 0) ? startAtLongMin : !startAtLongMin;
    const float laneLongEnd = goForwardLong ? longMax : longMin;

    if (i > 0) {
      if (!simStep(crossType, crossCoord, CHANGE_LANE)) return PLAN_ERR_INVALID_ROOM;
    }
    if (!simStep(longType, laneLongEnd, CLEAN_LANE)) return PLAN_ERR_INVALID_ROOM;
  }

  m.totalDistanceMm = m.approachDistanceMm + m.cleaningDistanceMm + m.transitionDistanceMm;
  if (!isFiniteFloat(m.cleaningDistanceMm) || !isFiniteFloat(m.transitionDistanceMm) ||
      !isFiniteFloat(m.approachDistanceMm) || !isFiniteFloat(m.totalDistanceMm) ||
      !isFiniteFloat(m.totalTurnDegrees)) {
    return PLAN_ERR_INVALID_ROOM;
  }

  const uint16_t laneChanges = (m.laneCount > 1) ? (uint16_t)(m.laneCount - 1) : 0;
  m.estimatedCost = (m.totalDistanceMm / req.nominalLinearSpeedMmS) +
                    (m.totalTurnDegrees / req.nominalAngularSpeedDegS) +
                    (req.laneChangePenaltySec * (float)laneChanges);

  if (!isFiniteFloat(m.estimatedCost) || m.estimatedCost < 0.0f) {
    return PLAN_ERR_INVALID_KINEMATICS;
  }

  outMetrics = m;
  return PLAN_OK;
}

PlanError CoveragePlanner::buildCandidate(const PlanRequest& req,
                                          CleaningDirection dir,
                                          StartCorner corner,
                                          bool approachYFirst,
                                          CoveragePlan& outCand) {
  clearPlan(outCand, PLAN_OK);

  const PlanError valErr = validateRequest(req);
  if (valErr != PLAN_OK) {
    clearPlan(outCand, valErr);
    return valErr;
  }

  if (!isValidDirection(dir)) {
    clearPlan(outCand, PLAN_ERR_INVALID_DIRECTION);
    return PLAN_ERR_INVALID_DIRECTION;
  }
  if (!isValidStartCorner(corner)) {
    clearPlan(outCand, PLAN_ERR_INVALID_POSE);
    return PLAN_ERR_INVALID_POSE;
  }

  const float xMin = req.edgeMm;
  const float xMax = req.roomW - req.edgeMm;
  const float yMin = req.edgeMm;
  const float yMax = req.roomL - req.edgeMm;

  const bool alongY = (dir == DIRECTION_Y);
  const float longMin  = alongY ? yMin : xMin;
  const float longMax  = alongY ? yMax : xMax;
  const float crossMin = alongY ? xMin : yMin;
  const float crossMax = alongY ? xMax : yMax;

  const float longSpan  = longMax - longMin;
  const float crossSpan = crossMax - crossMin;

  if (!isFiniteFloat(longSpan) || !isFiniteFloat(crossSpan)) {
    clearPlan(outCand, PLAN_ERR_INVALID_ROOM);
    return PLAN_ERR_INVALID_ROOM;
  }
  if (longSpan < PLANNER_MIN_LANE_LENGTH_MM || crossSpan < -1e-3f) {
    clearPlan(outCand, PLAN_ERR_ROOM_TOO_SMALL);
    return PLAN_ERR_ROOM_TOO_SMALL;
  }

  const float effSpacing = req.mopWidthMm * (1.0f - req.overlapPercent / 100.0f);
  if (!isFiniteFloat(effSpacing) || effSpacing <= 0.0f) {
    clearPlan(outCand, PLAN_ERR_INVALID_MOP_WIDTH);
    return PLAN_ERR_INVALID_MOP_WIDTH;
  }

  uint16_t laneCount = 0;
  float actualSpacing = 0.0f;
  const PlanError gridErr = computeLaneGrid(crossSpan, effSpacing, laneCount, actualSpacing);
  if (gridErr != PLAN_OK) {
    clearPlan(outCand, gridErr);
    return gridErr;
  }

  outCand.chosenDirection    = dir;
  outCand.startCorner        = corner;
  outCand.approachYFirst     = approachYFirst;
  outCand.roomW              = req.roomW;
  outCand.roomL              = req.roomL;
  outCand.edgeMm             = req.edgeMm;
  outCand.mopWidthMm         = req.mopWidthMm;
  outCand.overlapPercent     = req.overlapPercent;
  outCand.effectiveSpacingMm = effSpacing;
  outCand.actualSpacingMm    = actualSpacing;
  outCand.laneCount          = laneCount;

  float startX = 0.0f, startY = 0.0f;
  getCornerCoords(req.roomW, req.roomL, req.edgeMm, corner, startX, startY);

  auto pushStep = [&](StepType t, float v, StepPurpose p, int16_t lid) -> bool {
    if (outCand.stepCount >= MAX_PLAN_STEPS) return false;
    outCand.steps[outCand.stepCount++] = { t, v, p, lid };
    return true;
  };

  const float dxApp = fabsf(startX - req.robotX);
  const float dyApp = fabsf(startY - req.robotY);
  if (!isFiniteFloat(dxApp) || !isFiniteFloat(dyApp)) {
    clearPlan(outCand, PLAN_ERR_INVALID_POSE);
    return PLAN_ERR_INVALID_POSE;
  }

  if (approachYFirst) {
    if (dyApp >= PLANNER_ZERO_STEP_EPS_MM) {
      if (!pushStep(GO_Y, startY, APPROACH_START, -1)) {
        clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_STEPS);
        return PLAN_ERR_EXCEEDS_MAX_STEPS;
      }
    }
    if (dxApp >= PLANNER_ZERO_STEP_EPS_MM) {
      if (!pushStep(GO_X, startX, APPROACH_START, -1)) {
        clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_STEPS);
        return PLAN_ERR_EXCEEDS_MAX_STEPS;
      }
    }
  } else {
    if (dxApp >= PLANNER_ZERO_STEP_EPS_MM) {
      if (!pushStep(GO_X, startX, APPROACH_START, -1)) {
        clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_STEPS);
        return PLAN_ERR_EXCEEDS_MAX_STEPS;
      }
    }
    if (dyApp >= PLANNER_ZERO_STEP_EPS_MM) {
      if (!pushStep(GO_Y, startY, APPROACH_START, -1)) {
        clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_STEPS);
        return PLAN_ERR_EXCEEDS_MAX_STEPS;
      }
    }
  }

  // Xác định hướng quét đầu tiên từ góc xuất phát
  const bool startAtCrossMin = alongY
      ? (corner == CORNER_BOTTOM_LEFT || corner == CORNER_TOP_LEFT)
      : (corner == CORNER_BOTTOM_LEFT || corner == CORNER_BOTTOM_RIGHT);
  const bool startAtLongMin = alongY
      ? (corner == CORNER_BOTTOM_LEFT || corner == CORNER_BOTTOM_RIGHT)
      : (corner == CORNER_BOTTOM_LEFT || corner == CORNER_TOP_LEFT);

  const StepType longType  = alongY ? GO_Y : GO_X;
  const StepType crossType = alongY ? GO_X : GO_Y;

  for (uint16_t i = 0; i < outCand.laneCount; i++) {
    float crossCoord = crossMin;
    if (outCand.laneCount > 1) {
      if (i == outCand.laneCount - 1) {
        crossCoord = startAtCrossMin ? crossMax : crossMin;
      } else {
        crossCoord = startAtCrossMin
            ? (crossMin + (float)i * actualSpacing)
            : (crossMax - (float)i * actualSpacing);
      }
      crossCoord = clampFloat(crossCoord, crossMin, crossMax);
    }

    const bool goForwardLong = ((i % 2) == 0) ? startAtLongMin : !startAtLongMin;
    const float laneLongStart = goForwardLong ? longMin : longMax;
    const float laneLongEnd   = goForwardLong ? longMax : longMin;

    CleaningLane& ln = outCand.lanes[i];
    ln.id = i;
    if (alongY) {
      ln.startX = crossCoord;
      ln.startY = laneLongStart;
      ln.endX   = crossCoord;
      ln.endY   = laneLongEnd;
    } else {
      ln.startX = laneLongStart;
      ln.startY = crossCoord;
      ln.endX   = laneLongEnd;
      ln.endY   = crossCoord;
    }
    ln.plannedLengthMm = fabsf(laneLongEnd - laneLongStart);
    ln.estimatedCoveredLengthMm = 0.0f;
    ln.status = LANE_PENDING;

    if (i > 0) {
      if (!pushStep(crossType, crossCoord, CHANGE_LANE, -1)) {
        clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_STEPS);
        return PLAN_ERR_EXCEEDS_MAX_STEPS;
      }
    }

    if (!pushStep(longType, laneLongEnd, CLEAN_LANE, (int16_t)i)) {
      clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_STEPS);
      return PLAN_ERR_EXCEEDS_MAX_STEPS;
    }
  }

  const PlanError metricErr = computeMetricsFromSteps(req, outCand);
  if (metricErr != PLAN_OK) {
    clearPlan(outCand, metricErr);
    return metricErr;
  }

  outCand.valid = true;
  outCand.error = PLAN_OK;

  if (!verifyPlanIntegrity(outCand)) {
    clearPlan(outCand, PLAN_ERR_INTEGRITY_FAILED);
    return PLAN_ERR_INTEGRITY_FAILED;
  }

  return PLAN_OK;
}

bool CoveragePlanner::isBetterMetrics(const PlanMetrics& a, const PlanMetrics& b) {
  // 1. Ưu tiên chi phí thấp nhất
  const float costDiff = a.estimatedCost - b.estimatedCost;
  if (fabsf(costDiff) > 1e-3f) {
    return costDiff < 0.0f;
  }

  // 2. Chi phí gần bằng nhau -> ưu tiên ít góc xoay hơn
  const float turnDiff = a.totalTurnDegrees - b.totalTurnDegrees;
  if (fabsf(turnDiff) > 0.5f) {
    return turnDiff < 0.0f;
  }

  // 3. Tiếp theo ưu tiên quãng đường tiếp cận ngắn hơn
  const float appDiff = a.approachDistanceMm - b.approachDistanceMm;
  if (fabsf(appDiff) > 1.0f) {
    return appDiff < 0.0f;
  }

  // 4. Tie-break cuối cùng: giữ nguyên candidate đứng trước (deterministic)
  return false;
}

PlanError CoveragePlanner::generateBestPlan(const PlanRequest& req, CoveragePlan& outPlan) {
  const PlanError valErr = validateRequest(req);
  if (valErr != PLAN_OK) {
    clearPlan(outPlan, valErr);
    return valErr;
  }

  // Danh sách cố định 8 candidate theo đúng bảng C01..C08 (Step 04)
  struct CandSpec {
    CleaningDirection dir;
    StartCorner       corner;
  };
  static const CandSpec kCandidates[8] = {
    { DIRECTION_X, CORNER_BOTTOM_LEFT  },  // C01
    { DIRECTION_X, CORNER_BOTTOM_RIGHT },  // C02
    { DIRECTION_X, CORNER_TOP_LEFT     },  // C03
    { DIRECTION_X, CORNER_TOP_RIGHT    },  // C04
    { DIRECTION_Y, CORNER_BOTTOM_LEFT  },  // C05
    { DIRECTION_Y, CORNER_BOTTOM_RIGHT },  // C06
    { DIRECTION_Y, CORNER_TOP_LEFT     },  // C07
    { DIRECTION_Y, CORNER_TOP_RIGHT    }   // C08
  };

  // Chỉ giữ metadata nhẹ trên stack (~64 byte), không tạo CoveragePlan cục bộ (tránh tràn stack 8 KB ESP32)
  CandidateEval best;
  std::memset(&best, 0, sizeof(best));
  best.valid = false;
  best.dir = DIRECTION_Y;
  best.corner = CORNER_BOTTOM_LEFT;
  best.approachYFirst = false;

  PlanError lastErr = PLAN_ERR_ROOM_TOO_SMALL;
  uint8_t evaluated = 0;

  for (int idx = 0; idx < 8; idx++) {
    if (req.requestedDirection != DIRECTION_AUTO &&
        kCandidates[idx].dir != req.requestedDirection) {
      continue;
    }

    evaluated++;

    // Đánh giá cả 2 thứ tự tiếp cận: X->Y (false) và Y->X (true)
    for (int order = 0; order < 2; order++) {
      const bool yFirst = (order == 1);
      PlanMetrics candMetrics;
      const PlanError err = evaluateCandidateMetrics(
          req, kCandidates[idx].dir, kCandidates[idx].corner, yFirst, candMetrics);
      if (err == PLAN_OK) {
        if (!best.valid || isBetterMetrics(candMetrics, best.metrics)) {
          best.valid = true;
          best.dir = kCandidates[idx].dir;
          best.corner = kCandidates[idx].corner;
          best.approachYFirst = yFirst;
          best.metrics = candMetrics;
        }
      } else {
        lastErr = err;
      }
    }
  }

  if (!best.valid) {
    clearPlan(outPlan, lastErr);
    return lastErr;
  }

  // Dựng đầy đủ duy nhất 1 lần phương án tốt nhất vào outPlan; nếu build/integrity lỗi -> clearPlan atomic
  const PlanError buildErr = buildCandidate(req, best.dir, best.corner, best.approachYFirst, outPlan);
  if (buildErr != PLAN_OK) {
    clearPlan(outPlan, buildErr);
    return buildErr;
  }

  outPlan.candidatesEvaluated = evaluated;
  return PLAN_OK;
}

bool CoveragePlanner::verifyPlanIntegrity(const CoveragePlan& plan) {
  // 1. Kiểm tra cờ, enum và phạm vi tổng thể
  if (!plan.valid || plan.error != PLAN_OK) return false;
  if (!isValidDirection(plan.chosenDirection)) return false;
  if (!isValidStartCorner(plan.startCorner)) return false;
  if (plan.laneCount == 0 || plan.laneCount > MAX_LANES) return false;
  if (plan.stepCount == 0 || plan.stepCount > MAX_PLAN_STEPS) return false;

  // 2. Kiểm tra mọi thông số kích thước & metric đều hữu hạn và hợp lệ
  if (!isFiniteFloat(plan.roomW) || !isFiniteFloat(plan.roomL) ||
      !isFiniteFloat(plan.edgeMm) || !isFiniteFloat(plan.mopWidthMm) ||
      !isFiniteFloat(plan.overlapPercent) || !isFiniteFloat(plan.effectiveSpacingMm) ||
      !isFiniteFloat(plan.actualSpacingMm)) {
    return false;
  }
  if (plan.roomW <= 0.0f || plan.roomL <= 0.0f || plan.edgeMm <= 0.0f ||
      plan.mopWidthMm <= 0.0f || plan.effectiveSpacingMm <= 0.0f) {
    return false;
  }
  if (plan.overlapPercent < MIN_OVERLAP_PERCENT || plan.overlapPercent > MAX_OVERLAP_PERCENT) {
    return false;
  }
  if (!isFiniteFloat(plan.metrics.cleaningDistanceMm) ||
      !isFiniteFloat(plan.metrics.transitionDistanceMm) ||
      !isFiniteFloat(plan.metrics.approachDistanceMm) ||
      !isFiniteFloat(plan.metrics.totalDistanceMm) ||
      !isFiniteFloat(plan.metrics.totalTurnDegrees) ||
      !isFiniteFloat(plan.metrics.estimatedCost) ||
      plan.metrics.cleaningDistanceMm < 0.0f ||
      plan.metrics.transitionDistanceMm < 0.0f ||
      plan.metrics.approachDistanceMm < 0.0f ||
      plan.metrics.totalDistanceMm < 0.0f ||
      plan.metrics.totalTurnDegrees < 0.0f ||
      plan.metrics.estimatedCost < 0.0f) {
    return false;
  }

  const float tol = 0.25f;
  const float xMin = plan.edgeMm - tol;
  const float xMax = plan.roomW - plan.edgeMm + tol;
  const float yMin = plan.edgeMm - tol;
  const float yMax = plan.roomL - plan.edgeMm + tol;
  if (!isFiniteFloat(xMin) || !isFiniteFloat(xMax) || !isFiniteFloat(yMin) || !isFiniteFloat(yMax)) {
    return false;
  }
  if (xMin > xMax || yMin > yMax) return false;

  if (plan.laneCount == 1) {
    if (fabsf(plan.actualSpacingMm) > tol) return false;
  } else {
    if (plan.actualSpacingMm <= 0.0f || plan.actualSpacingMm > plan.effectiveSpacingMm + tol) {
      return false;
    }
  }

  const bool alongY = (plan.chosenDirection == DIRECTION_Y);
  const StepType expectedLongType  = alongY ? GO_Y : GO_X;
  const StepType expectedCrossType = alongY ? GO_X : GO_Y;

  // 3. Kiểm tra góc xuất phát khớp đầu làn 0
  float cornerX = 0.0f, cornerY = 0.0f;
  getCornerCoords(plan.roomW, plan.roomL, plan.edgeMm, plan.startCorner, cornerX, cornerY);

  // 4. Kiểm tra tính hợp lệ, khoảng cách và tính liên tục của từng làn
  for (uint16_t i = 0; i < plan.laneCount; i++) {
    const CleaningLane& ln = plan.lanes[i];
    if (ln.id != i) return false;
    if (!isValidLaneStatus(ln.status)) return false;

    if (!isFiniteFloat(ln.startX) || !isFiniteFloat(ln.startY) ||
        !isFiniteFloat(ln.endX) || !isFiniteFloat(ln.endY) ||
        !isFiniteFloat(ln.plannedLengthMm) || !isFiniteFloat(ln.estimatedCoveredLengthMm)) {
      return false;
    }

    if (ln.startX < xMin || ln.startX > xMax || ln.endX < xMin || ln.endX > xMax) return false;
    if (ln.startY < yMin || ln.startY > yMax || ln.endY < yMin || ln.endY > yMax) return false;
    if (ln.plannedLengthMm < PLANNER_MIN_LANE_LENGTH_MM - tol) return false;

    if (alongY) {
      if (fabsf(ln.startX - ln.endX) > tol) return false;
      if (fabsf(fabsf(ln.endY - ln.startY) - ln.plannedLengthMm) > tol) return false;
    } else {
      if (fabsf(ln.startY - ln.endY) > tol) return false;
      if (fabsf(fabsf(ln.endX - ln.startX) - ln.plannedLengthMm) > tol) return false;
    }

    if (i == 0) {
      if (fabsf(ln.startX - cornerX) > tol || fabsf(ln.startY - cornerY) > tol) return false;
    } else {
      const CleaningLane& prev = plan.lanes[i - 1];
      if (alongY) {
        const float laneGap = fabsf(ln.startX - prev.startX);
        if (laneGap < 0.5f || laneGap > plan.effectiveSpacingMm + tol) return false;
        const float dirPrev = prev.endY - prev.startY;
        const float dirCur  = ln.endY - ln.startY;
        if (dirPrev * dirCur >= 0.0f) return false;
        // Điểm bắt đầu dọc làn i phải liên tục với điểm kết thúc dọc làn i-1
        if (fabsf(ln.startY - prev.endY) > tol) return false;
      } else {
        const float laneGap = fabsf(ln.startY - prev.startY);
        if (laneGap < 0.5f || laneGap > plan.effectiveSpacingMm + tol) return false;
        const float dirPrev = prev.endX - prev.startX;
        const float dirCur  = ln.endX - ln.startX;
        if (dirPrev * dirCur >= 0.0f) return false;
        // Điểm bắt đầu dọc làn i phải liên tục với điểm kết thúc dọc làn i-1
        if (fabsf(ln.startX - prev.endX) > tol) return false;
      }
    }
  }

  // 5. Kiểm tra thứ tự bước, trục, ánh xạ laneId, target và tính liên tục đường đi
  uint16_t cleanSteps = 0;
  uint16_t shiftSteps = 0;
  uint8_t  approachSteps = 0;
  bool     approachUsedX = false;
  bool     approachUsedY = false;
  bool     expectClean = true;
  float    modelX = plan.lanes[0].startX;
  float    modelY = plan.lanes[0].startY;

  for (uint16_t i = 0; i < plan.stepCount; i++) {
    const Step& st = plan.steps[i];
    if (!isValidStepType(st.type) || !isValidStepPurpose(st.purpose)) return false;
    if (!isFiniteFloat(st.val)) return false;

    if (st.type == GO_X) {
      if (st.val < xMin || st.val > xMax) return false;
    } else {
      if (st.val < yMin || st.val > yMax) return false;
    }

    if (st.purpose == APPROACH_START) {
      // Mọi bước APPROACH_START phải đứng trước CLEAN_LANE đầu tiên
      if (cleanSteps > 0 || shiftSteps > 0) return false;
      if (st.laneId != -1) return false;
      if (++approachSteps > 2) return false;
      if (st.type == GO_X) {
        if (approachUsedX) return false;
        approachUsedX = true;
        if (fabsf(st.val - plan.lanes[0].startX) > tol) return false;
      } else {
        if (approachUsedY) return false;
        approachUsedY = true;
        if (fabsf(st.val - plan.lanes[0].startY) > tol) return false;
      }
    } else if (st.purpose == CLEAN_LANE) {
      if (!expectClean) return false;
      if (st.type != expectedLongType) return false;
      if (st.laneId < 0 || st.laneId >= (int16_t)plan.laneCount) return false;
      if (st.laneId != (int16_t)cleanSteps) return false;

      const CleaningLane& ln = plan.lanes[st.laneId];
      // Mô hình đường đi phải đang đứng đúng điểm bắt đầu làn này
      if (fabsf(modelX - ln.startX) > tol || fabsf(modelY - ln.startY) > tol) return false;

      const float expectedTarget = alongY ? ln.endY : ln.endX;
      if (fabsf(st.val - expectedTarget) > tol) return false;

      if (alongY) modelY = st.val;
      else        modelX = st.val;

      if (fabsf(modelX - ln.endX) > tol || fabsf(modelY - ln.endY) > tol) return false;

      cleanSteps++;
      expectClean = false;
    } else {
      // CHANGE_LANE
      if (expectClean) return false;
      if (cleanSteps == 0 || cleanSteps >= plan.laneCount) return false;
      if (st.laneId != -1) return false;
      if (st.type != expectedCrossType) return false;

      const CleaningLane& nextLn = plan.lanes[cleanSteps];
      const float expectedCross = alongY ? nextLn.startX : nextLn.startY;
      if (fabsf(st.val - expectedCross) > tol) return false;

      if (alongY) modelX = st.val;
      else        modelY = st.val;

      // Sau CHANGE_LANE phải khớp chính xác điểm bắt đầu làn kế tiếp trên cả 2 trục
      if (fabsf(modelX - nextLn.startX) > tol || fabsf(modelY - nextLn.startY) > tol) return false;

      shiftSteps++;
      expectClean = true;
    }
  }

  if (cleanSteps != plan.laneCount) return false;
  if (shiftSteps != (uint16_t)(plan.laneCount > 1 ? plan.laneCount - 1 : 0)) return false;
  if (expectClean) return false;  // Không kết thúc lửng lơ sau CHANGE_LANE

  return true;
}
