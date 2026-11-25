#include "CoveragePlanner.h"
#include <math.h>
#include <string.h>

static inline bool isFiniteFloat(float v) {
  return isfinite(v) != 0;
}

static inline float clampFloat(float v, float lo, float hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

float CoveragePlanner::wrapAngle180(float deg) {
  while (deg > 180.0f)  deg -= 360.0f;
  while (deg <= -180.0f) deg += 360.0f;
  return deg;
}

void CoveragePlanner::clearPlan(CoveragePlan& plan, PlanError err) {
  memset(&plan, 0, sizeof(CoveragePlan));
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

  const float workW = req.roomW - 2.0f * req.edgeMm;
  const float workL = req.roomL - 2.0f * req.edgeMm;

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

void CoveragePlanner::computeMetricsFromSteps(const PlanRequest& req, CoveragePlan& plan) {
  PlanMetrics m;
  memset(&m, 0, sizeof(PlanMetrics));
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
      if (dist < 1e-3f) continue;
      tgtHeading = (st.val > cx) ? 90.0f : -90.0f;
      cx = st.val;
    } else {
      dist = fabsf(st.val - cy);
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

  const uint16_t laneChanges = (m.laneCount > 1) ? (uint16_t)(m.laneCount - 1) : 0;
  m.estimatedCost = (m.totalDistanceMm / req.nominalLinearSpeedMmS) +
                    (m.totalTurnDegrees / req.nominalAngularSpeedDegS) +
                    (req.laneChangePenaltySec * (float)laneChanges);

  plan.metrics = m;
}

PlanError CoveragePlanner::buildCandidate(const PlanRequest& req,
                                          CleaningDirection dir,
                                          StartCorner corner,
                                          bool approachYFirst,
                                          CoveragePlan& outCand) {
  clearPlan(outCand, PLAN_OK);

  if (dir != DIRECTION_X && dir != DIRECTION_Y) {
    clearPlan(outCand, PLAN_ERR_INVALID_DIRECTION);
    return PLAN_ERR_INVALID_DIRECTION;
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

  if (longSpan < PLANNER_MIN_LANE_LENGTH_MM || crossSpan < -1e-3f) {
    clearPlan(outCand, PLAN_ERR_ROOM_TOO_SMALL);
    return PLAN_ERR_ROOM_TOO_SMALL;
  }

  const float effSpacing = req.mopWidthMm * (1.0f - req.overlapPercent / 100.0f);
  if (!isFiniteFloat(effSpacing) || effSpacing <= 0.0f) {
    clearPlan(outCand, PLAN_ERR_INVALID_MOP_WIDTH);
    return PLAN_ERR_INVALID_MOP_WIDTH;
  }

  uint32_t laneCount32 = 1;
  float actualSpacing = 0.0f;
  if (crossSpan > 1e-3f) {
    const float rawRatio = crossSpan / effSpacing;
    if (!isFiniteFloat(rawRatio) || rawRatio < 0.0f) {
      clearPlan(outCand, PLAN_ERR_INVALID_ROOM);
      return PLAN_ERR_INVALID_ROOM;
    }
    laneCount32 = (uint32_t)ceilf(rawRatio) + 1u;
    if (laneCount32 > MAX_LANES) {
      clearPlan(outCand, PLAN_ERR_EXCEEDS_MAX_LANES);
      return PLAN_ERR_EXCEEDS_MAX_LANES;
    }
    actualSpacing = crossSpan / (float)(laneCount32 - 1u);
  } else {
    // Phòng hẹp suy biến thành đúng 1 làn hợp lệ (không chia cho 0)
    laneCount32 = 1u;
    actualSpacing = 0.0f;
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
  outCand.laneCount          = (uint16_t)laneCount32;

  float startX = 0.0f, startY = 0.0f;
  getCornerCoords(req.roomW, req.roomL, req.edgeMm, corner, startX, startY);

  // Thêm bước tiếp cận (bỏ qua bước có độ dài < PLANNER_ZERO_STEP_EPS_MM)
  auto pushStep = [&](StepType t, float v, StepPurpose p, int16_t lid) -> bool {
    if (outCand.stepCount >= MAX_PLAN_STEPS) return false;
    outCand.steps[outCand.stepCount++] = { t, v, p, lid };
    return true;
  };

  const float dxApp = fabsf(startX - req.robotX);
  const float dyApp = fabsf(startY - req.robotY);
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

  computeMetricsFromSteps(req, outCand);
  outCand.valid = true;
  outCand.error = PLAN_OK;

  if (!verifyPlanIntegrity(outCand)) {
    clearPlan(outCand, PLAN_ERR_INTEGRITY_FAILED);
    return PLAN_ERR_INTEGRITY_FAILED;
  }

  return PLAN_OK;
}

bool CoveragePlanner::isBetterCandidate(const CoveragePlan& a, const CoveragePlan& b) {
  if (!b.valid) return a.valid;
  if (!a.valid) return false;

  // 1. Ưu tiên chi phí thấp nhất
  const float costDiff = a.metrics.estimatedCost - b.metrics.estimatedCost;
  if (fabsf(costDiff) > 1e-3f) {
    return costDiff < 0.0f;
  }

  // 2. Chi phí gần bằng nhau -> ưu tiên ít góc xoay hơn
  const float turnDiff = a.metrics.totalTurnDegrees - b.metrics.totalTurnDegrees;
  if (fabsf(turnDiff) > 0.5f) {
    return turnDiff < 0.0f;
  }

  // 3. Tiếp theo ưu tiên quãng đường tiếp cận ngắn hơn
  const float appDiff = a.metrics.approachDistanceMm - b.metrics.approachDistanceMm;
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

  CoveragePlan bestPlan;
  clearPlan(bestPlan, PLAN_ERR_ROOM_TOO_SMALL);
  CoveragePlan tmpCand;

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
      const PlanError err = buildCandidate(req, kCandidates[idx].dir,
                                           kCandidates[idx].corner, yFirst, tmpCand);
      if (err == PLAN_OK) {
        if (!bestPlan.valid || isBetterCandidate(tmpCand, bestPlan)) {
          bestPlan = tmpCand;
        }
      } else {
        lastErr = err;
      }
    }
  }

  if (!bestPlan.valid) {
    clearPlan(outPlan, lastErr);
    return lastErr;
  }

  bestPlan.candidatesEvaluated = evaluated;
  // Chỉ ghi đè kế hoạch đích sau khi xây dựng & kiểm định thành công (atomic)
  outPlan = bestPlan;
  return PLAN_OK;
}

bool CoveragePlanner::verifyPlanIntegrity(const CoveragePlan& plan) {
  if (!plan.valid || plan.laneCount == 0 || plan.laneCount > MAX_LANES) return false;
  if (plan.stepCount == 0 || plan.stepCount > MAX_PLAN_STEPS) return false;

  const float tol = 0.25f;
  const float xMin = plan.edgeMm - tol;
  const float xMax = plan.roomW - plan.edgeMm + tol;
  const float yMin = plan.edgeMm - tol;
  const float yMax = plan.roomL - plan.edgeMm + tol;

  const bool alongY = (plan.chosenDirection == DIRECTION_Y);

  // 1. Kiểm tra tính hợp lệ của từng làn
  for (uint16_t i = 0; i < plan.laneCount; i++) {
    const CleaningLane& ln = plan.lanes[i];
    if (ln.id != i) return false;

    if (ln.startX < xMin || ln.startX > xMax || ln.endX < xMin || ln.endX > xMax) return false;
    if (ln.startY < yMin || ln.startY > yMax || ln.endY < yMin || ln.endY > yMax) return false;
    if (ln.plannedLengthMm < PLANNER_MIN_LANE_LENGTH_MM - tol) return false;

    if (alongY) {
      if (fabsf(ln.startX - ln.endX) > tol) return false;
    } else {
      if (fabsf(ln.startY - ln.endY) > tol) return false;
    }

    // Kiểm tra 2 làn liên tiếp không trùng lặp và đảo chiều ngược nhau
    if (i > 0) {
      const CleaningLane& prev = plan.lanes[i - 1];
      if (alongY) {
        if (fabsf(ln.startX - prev.startX) < 0.5f) return false;
        const float dirPrev = prev.endY - prev.startY;
        const float dirCur  = ln.endY - ln.startY;
        if (dirPrev * dirCur >= 0.0f) return false;
      } else {
        if (fabsf(ln.startY - prev.startY) < 0.5f) return false;
        const float dirPrev = prev.endX - prev.startX;
        const float dirCur  = ln.endX - ln.startX;
        if (dirPrev * dirCur >= 0.0f) return false;
      }
    }
  }

  // 2. Kiểm tra ánh xạ bước di chuyển với các làn
  uint16_t cleanSteps = 0;
  uint16_t shiftSteps = 0;
  for (uint16_t i = 0; i < plan.stepCount; i++) {
    const Step& st = plan.steps[i];
    if (st.type == GO_X) {
      if (st.val < xMin || st.val > xMax) return false;
    } else {
      if (st.val < yMin || st.val > yMax) return false;
    }

    if (st.purpose == CLEAN_LANE) {
      if (st.laneId < 0 || st.laneId >= (int16_t)plan.laneCount) return false;
      if (st.laneId != (int16_t)cleanSteps) return false;
      cleanSteps++;
    } else {
      if (st.laneId != -1) return false;
      if (st.purpose == CHANGE_LANE) shiftSteps++;
    }
  }

  if (cleanSteps != plan.laneCount) return false;
  if (shiftSteps != (uint16_t)(plan.laneCount > 1 ? plan.laneCount - 1 : 0)) return false;

  return true;
}
