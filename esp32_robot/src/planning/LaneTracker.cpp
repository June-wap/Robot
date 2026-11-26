#include "LaneTracker.h"
#include <cmath>
#include <cstring>

static inline float clampPct(float v) {
  if (v < 0.0f)   return 0.0f;
  if (v > 100.0f) return 100.0f;
  return v;
}

LaneTracker::LaneTracker()
    : plan_(nullptr),
      result_(RESULT_NONE),
      currentLaneId_(-1),
      totalPlannedCleanMm_(0.0f),
      currentLaneOffTrack_(false),
      currentLaneMaxCrossTrackMm_(0.0f) {
  lastReason_[0] = '\0';
}

void LaneTracker::reset() {
  plan_ = nullptr;
  result_ = RESULT_NONE;
  currentLaneId_ = -1;
  totalPlannedCleanMm_ = 0.0f;
  currentLaneOffTrack_ = false;
  currentLaneMaxCrossTrackMm_ = 0.0f;
  lastReason_[0] = '\0';
}

void LaneTracker::onCalibrationStarted() {
  reset();
  result_ = RESULT_RUNNING;
}

void LaneTracker::setReason(const char* reason) {
  if (!reason) {
    lastReason_[0] = '\0';
    return;
  }
  std::strncpy(lastReason_, reason, sizeof(lastReason_) - 1);
  lastReason_[sizeof(lastReason_) - 1] = '\0';
}

bool LaneTracker::isValidLaneId(int16_t laneId) const {
  return (plan_ != nullptr && plan_->valid &&
          laneId >= 0 && laneId < (int16_t)plan_->laneCount);
}

float LaneTracker::computeAlongProgress(const CleaningLane& ln, float curX, float curY) const {
  const float dx = ln.endX - ln.startX;
  const float dy = ln.endY - ln.startY;
  if (fabsf(dy) >= fabsf(dx)) {
    return (dy >= 0.0f) ? (curY - ln.startY) : (ln.startY - curY);
  }
  return (dx >= 0.0f) ? (curX - ln.startX) : (ln.startX - curX);
}

float LaneTracker::computeCrossTrackError(int16_t laneId, float curX, float curY) const {
  if (!isValidLaneId(laneId) || !std::isfinite(curX) || !std::isfinite(curY)) {
    return 1e9f;
  }
  const CleaningLane& ln = plan_->lanes[laneId];
  const float dx = ln.endX - ln.startX;
  const float dy = ln.endY - ln.startY;
  if (fabsf(dy) >= fabsf(dx)) {
    return fabsf(curX - ln.startX);
  }
  return fabsf(curY - ln.startY);
}

bool LaneTracker::isWithinCrossTrack(int16_t laneId, float curX, float curY) const {
  return computeCrossTrackError(laneId, curX, curY) <= LANE_CROSS_TRACK_MAX_MM;
}

bool LaneTracker::isPositionAtLaneStart(int16_t laneId, float curX, float curY) const {
  if (!isValidLaneId(laneId) || !std::isfinite(curX) || !std::isfinite(curY)) {
    return false;
  }
  const CleaningLane& ln = plan_->lanes[laneId];
  const float dx = ln.endX - ln.startX;
  const float dy = ln.endY - ln.startY;
  float alongErr = 0.0f;
  float crossErr = 0.0f;
  if (fabsf(dy) >= fabsf(dx)) {
    alongErr = fabsf(curY - ln.startY);
    crossErr = fabsf(curX - ln.startX);
  } else {
    alongErr = fabsf(curX - ln.startX);
    crossErr = fabsf(curY - ln.startY);
  }
  return (alongErr <= LANE_START_ALONG_TOL_MM) && (crossErr <= LANE_CROSS_TRACK_MAX_MM);
}

bool LaneTracker::isPositionAtLaneEnd(int16_t laneId, float curX, float curY) const {
  if (!isValidLaneId(laneId) || !std::isfinite(curX) || !std::isfinite(curY)) {
    return false;
  }
  const CleaningLane& ln = plan_->lanes[laneId];
  const float dx = ln.endX - ln.startX;
  const float dy = ln.endY - ln.startY;
  float alongErr = 0.0f;
  float crossErr = 0.0f;
  if (fabsf(dy) >= fabsf(dx)) {
    alongErr = fabsf(curY - ln.endY);
    crossErr = fabsf(curX - ln.endX);
  } else {
    alongErr = fabsf(curX - ln.endX);
    crossErr = fabsf(curY - ln.endY);
  }
  return (alongErr <= LANE_END_ALONG_TOL_MM) && (crossErr <= LANE_CROSS_TRACK_MAX_MM);
}

void LaneTracker::beginSession(CoveragePlan* plan) {
  reset();
  if (!plan || !plan->valid || plan->laneCount == 0) {
    result_ = RESULT_FAILED;
    setReason("Ke hoach khong hop le");
    return;
  }

  plan_ = plan;
  totalPlannedCleanMm_ = 0.0f;
  for (uint16_t i = 0; i < plan_->laneCount; i++) {
    plan_->lanes[i].status = LANE_PENDING;
    plan_->lanes[i].estimatedCoveredLengthMm = 0.0f;
    totalPlannedCleanMm_ += plan_->lanes[i].plannedLengthMm;
  }
  currentLaneId_ = -1;
  currentLaneOffTrack_ = false;
  currentLaneMaxCrossTrackMm_ = 0.0f;
  result_ = RESULT_RUNNING;
  lastReason_[0] = '\0';
}

void LaneTracker::onLaneStarted(int16_t laneId) {
  if (!isValidLaneId(laneId)) return;
  CleaningLane& ln = plan_->lanes[laneId];
  if (ln.status == LANE_PENDING) {
    ln.status = LANE_RUNNING;
    currentLaneId_ = laneId;
    currentLaneOffTrack_ = false;
    currentLaneMaxCrossTrackMm_ = 0.0f;
  }
}

void LaneTracker::updateLaneCovered(int16_t laneId, float curX, float curY) {
  if (!isValidLaneId(laneId) || !std::isfinite(curX) || !std::isfinite(curY)) return;
  CleaningLane& ln = plan_->lanes[laneId];
  if (ln.status != LANE_RUNNING) return;

  const float crossErr = computeCrossTrackError(laneId, curX, curY);
  if (crossErr > currentLaneMaxCrossTrackMm_) {
    currentLaneMaxCrossTrackMm_ = crossErr;
  }
  if (crossErr > LANE_CROSS_TRACK_MAX_MM) {
    // Đang lệch khỏi tâm làn vượt ngưỡng -> không ghi nhận thêm tiến độ lau
    currentLaneOffTrack_ = true;
    return;
  }

  float proj = computeAlongProgress(ln, curX, curY);
  if (proj < 0.0f) proj = 0.0f;
  if (proj > ln.plannedLengthMm) proj = ln.plannedLengthMm;
  if (proj > ln.estimatedCoveredLengthMm) {
    ln.estimatedCoveredLengthMm = proj;
  }
}

void LaneTracker::onLaneCompleted(int16_t laneId) {
  if (!isValidLaneId(laneId)) return;
  CleaningLane& ln = plan_->lanes[laneId];
  // Chỉ cho phép hoàn thành làn đang ở trạng thái RUNNING (tránh đánh dấu sai làn PENDING/FAILED/BLOCKED)
  if (ln.status != LANE_RUNNING) return;
  ln.status = LANE_COMPLETED;
  ln.estimatedCoveredLengthMm = ln.plannedLengthMm;
  if (currentLaneId_ == laneId) currentLaneId_ = -1;
}

bool LaneTracker::completeLaneWithPose(int16_t laneId, float curX, float curY) {
  if (!isValidLaneId(laneId)) return false;
  CleaningLane& ln = plan_->lanes[laneId];
  if (ln.status != LANE_RUNNING) return false;

  if (!std::isfinite(curX) || !std::isfinite(curY)) {
    ln.status = LANE_FAILED;
    if (currentLaneId_ == laneId) currentLaneId_ = -1;
    setReason("Toa do khong hop le khi ket thuc lan");
    return false;
  }

  updateLaneCovered(laneId, curX, curY);

  const float crossErr = computeCrossTrackError(laneId, curX, curY);
  const float alongProg = computeAlongProgress(ln, curX, curY);
  const bool offTrack = currentLaneOffTrack_ || (crossErr > LANE_CROSS_TRACK_MAX_MM);
  const bool reachedEnd = (alongProg >= ln.plannedLengthMm - LANE_END_ALONG_TOL_MM) &&
                          (ln.estimatedCoveredLengthMm >= ln.plannedLengthMm - LANE_END_ALONG_TOL_MM);

  if (offTrack || !reachedEnd) {
    const float covered = ln.estimatedCoveredLengthMm;
    const bool isPartial = (ln.plannedLengthMm > 0.0f) &&
                           (covered >= LANE_PARTIAL_MIN_MM) &&
                           (covered >= ln.plannedLengthMm * LANE_PARTIAL_MIN_RATIO);
    ln.status = isPartial ? LANE_PARTIAL : LANE_FAILED;
    if (currentLaneId_ == laneId) currentLaneId_ = -1;
    setReason(offTrack ? "Lech lan vuot nguong cho phep" : "Chua hoan thanh du chieu dai lan");
    return false;
  }

  onLaneCompleted(laneId);
  return true;
}

void LaneTracker::onLaneBlocked(int16_t laneId, float curX, float curY, const char* reason) {
  if (!isValidLaneId(laneId)) return;
  updateLaneCovered(laneId, curX, curY);
  CleaningLane& ln = plan_->lanes[laneId];

  const float covered = ln.estimatedCoveredLengthMm;
  const bool isPartial = (ln.plannedLengthMm > 0.0f) &&
                         (covered >= LANE_PARTIAL_MIN_MM) &&
                         (covered >= ln.plannedLengthMm * LANE_PARTIAL_MIN_RATIO);

  ln.status = isPartial ? LANE_PARTIAL : LANE_BLOCKED;
  if (currentLaneId_ == laneId) currentLaneId_ = -1;
  setReason(reason ? reason : "Gap vat can");
}

void LaneTracker::onLaneFailed(int16_t laneId, float curX, float curY, const char* reason) {
  if (!isValidLaneId(laneId)) return;
  updateLaneCovered(laneId, curX, curY);
  CleaningLane& ln = plan_->lanes[laneId];
  ln.status = LANE_FAILED;
  if (currentLaneId_ == laneId) currentLaneId_ = -1;
  setReason(reason ? reason : "Loi thuc thi lan");
}

void LaneTracker::onChangeLaneFailed(const char* reason) {
  if (isValidLaneId(currentLaneId_) && plan_->lanes[currentLaneId_].status == LANE_RUNNING) {
    plan_->lanes[currentLaneId_].status = LANE_FAILED;
  }
  currentLaneId_ = -1;
  result_ = RESULT_FAILED;
  setReason(reason ? reason : "Khong chuyen duoc sang lan ke tiep");
}

void LaneTracker::onUserAborted() {
  if (result_ == RESULT_RUNNING) {
    if (isValidLaneId(currentLaneId_) && plan_->lanes[currentLaneId_].status == LANE_RUNNING) {
      CleaningLane& ln = plan_->lanes[currentLaneId_];
      if (ln.estimatedCoveredLengthMm >= LANE_PARTIAL_MIN_MM &&
          ln.estimatedCoveredLengthMm >= ln.plannedLengthMm * LANE_PARTIAL_MIN_RATIO) {
        ln.status = LANE_PARTIAL;
      } else {
        ln.status = LANE_PENDING;
      }
    }
    currentLaneId_ = -1;
    result_ = RESULT_ABORTED;
    setReason(plan_ ? "Nguoi dung dung" : "Nguoi dung dung trong hieu chinh");
  }
}

void LaneTracker::onSessionFailed(const char* reason) {
  if (isValidLaneId(currentLaneId_) && plan_->lanes[currentLaneId_].status == LANE_RUNNING) {
    plan_->lanes[currentLaneId_].status = LANE_FAILED;
  }
  currentLaneId_ = -1;
  result_ = RESULT_FAILED;
  setReason(reason ? reason : "Phien lau loi");
}

void LaneTracker::onSessionFinished() {
  if (!plan_ || !plan_->valid || plan_->laneCount == 0) {
    result_ = RESULT_FAILED;
    return;
  }
  if (result_ == RESULT_ABORTED || result_ == RESULT_FAILED) {
    return;
  }

  currentLaneId_ = -1;
  const CleaningProgress p = getProgress();
  if (p.totalLanes > 0 && p.completedLanes == p.totalLanes) {
    result_ = RESULT_COMPLETED;
  } else if (p.completedLanes > 0 || p.partialLanes > 0) {
    result_ = RESULT_PARTIAL;
  } else {
    result_ = RESULT_FAILED;
  }
}

CleaningProgress LaneTracker::getProgress() const {
  CleaningProgress p;
  std::memset(&p, 0, sizeof(CleaningProgress));
  p.currentLaneId = currentLaneId_;

  if (!plan_ || !plan_->valid || plan_->laneCount == 0) {
    return p;
  }

  p.totalLanes = plan_->laneCount;
  float totalCoveredMm = 0.0f;

  for (uint16_t i = 0; i < plan_->laneCount; i++) {
    const CleaningLane& ln = plan_->lanes[i];
    totalCoveredMm += ln.estimatedCoveredLengthMm;

    switch (ln.status) {
      case LANE_COMPLETED: p.completedLanes++; break;
      case LANE_BLOCKED:   p.blockedLanes++;   break;
      case LANE_PARTIAL:   p.partialLanes++;   break;
      case LANE_FAILED:    p.failedLanes++;    break;
      case LANE_PENDING:   p.pendingLanes++;   break;
      case LANE_RUNNING:
        // Làn đang chạy không tính là completed hay processed
        break;
    }
  }

  const uint16_t processed = (uint16_t)(p.completedLanes + p.blockedLanes +
                                        p.partialLanes + p.failedLanes);

  p.processedPercent = clampPct(100.0f * (float)processed / (float)p.totalLanes);
  p.completedPercent = clampPct(100.0f * (float)p.completedLanes / (float)p.totalLanes);

  if (totalPlannedCleanMm_ > 1e-3f) {
    p.estimatedDistancePercent = clampPct(100.0f * totalCoveredMm / totalPlannedCleanMm_);
  } else {
    p.estimatedDistancePercent = 0.0f;
  }

  return p;
}
