#include <cstdio>
#include <cmath>
#include "../esp32_robot/LaneTypes.h"
#include "../esp32_robot/CoveragePlanner.h"
#include "../esp32_robot/LaneTracker.h"

static int gPassed = 0;
static int gFailed = 0;

#define EXPECT_TRUE(cond, name) do { \
  if (cond) { gPassed++; printf("[PASS] %s\n", name); } \
  else { gFailed++; printf("[FAIL] %s (line %d)\n", name, __LINE__); } \
} while (0)

static CoveragePlan makePlanWithLanes() {
  PlanRequest r;
  r.roomW = 1200.0f;  // span = 780, eff = 162 -> 6 lanes
  r.roomL = 2400.0f;
  r.edgeMm = 210.0f;
  r.mopWidthMm = 180.0f;
  r.overlapPercent = 10.0f;
  r.robotRadiusMm = 170.0f;
  r.robotX = 210.0f;
  r.robotY = 210.0f;
  r.robotPsiDeg = 0.0f;
  r.requestedDirection = DIRECTION_Y;
  r.nominalLinearSpeedMmS = 194.1f;
  r.nominalAngularSpeedDegS = 86.8f;
  r.laneChangePenaltySec = 1.5f;
  CoveragePlan p;
  CoveragePlanner::generateBestPlan(r, p);
  return p;
}

int main() {
  printf("\n=== GROUP E (PART 2): PROGRESS TRACKING & CLEANING RESULTS ===\n");

  // P01: Trước khi có phiên lau -> không chia cho 0, mọi tỷ lệ = 0%
  {
    LaneTracker tr;
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(pr.totalLanes == 0 && pr.processedPercent == 0.0f &&
                pr.completedPercent == 0.0f && pr.estimatedDistancePercent == 0.0f &&
                tr.getResult() == RESULT_NONE,
                "P01: Empty tracker avoids division by zero");
  }

  // P02: Hoàn thành toàn bộ các làn -> RESULT_COMPLETED và 100%
  {
    CoveragePlan p = makePlanWithLanes();
    LaneTracker tr;
    tr.beginSession(&p);
    for (uint16_t i = 0; i < p.laneCount; i++) {
      tr.onLaneStarted((int16_t)i);
      tr.onLaneCompleted((int16_t)i);
    }
    tr.onSessionFinished();
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(tr.getResult() == RESULT_COMPLETED &&
                pr.completedLanes == p.laneCount &&
                pr.pendingLanes == 0 &&
                fabsf(pr.processedPercent - 100.0f) < 1e-3f &&
                fabsf(pr.completedPercent - 100.0f) < 1e-3f &&
                fabsf(pr.estimatedDistancePercent - 100.0f) < 1e-3f,
                "P02: All lanes completed yields RESULT_COMPLETED and 100%");
  }

  // P03: Còn làn bị chặn/partial/failed -> KHÔNG được tạo RESULT_COMPLETED (phải là RESULT_PARTIAL)
  {
    CoveragePlan p = makePlanWithLanes(); // 6 lanes
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0); tr.onLaneCompleted(0);
    tr.onLaneStarted(1); tr.onLaneBlocked(1, p.lanes[1].startX, p.lanes[1].startY, "Can");
    tr.onLaneStarted(2); tr.onLaneBlocked(2, p.lanes[2].startX, p.lanes[2].startY + 990.0f, "Can giua");
    tr.onLaneStarted(3); tr.onLaneFailed(3, p.lanes[3].startX, p.lanes[3].startY, "Timeout");
    tr.onLaneStarted(4); tr.onLaneCompleted(4);
    tr.onLaneStarted(5); tr.onLaneCompleted(5);
    tr.onSessionFinished();
    CleaningProgress pr = tr.getProgress();

    uint16_t sumAll = pr.completedLanes + pr.blockedLanes + pr.partialLanes +
                      pr.failedLanes + pr.pendingLanes;
    EXPECT_TRUE(tr.getResult() == RESULT_PARTIAL &&
                sumAll == pr.totalLanes &&
                pr.completedLanes == 3 && pr.blockedLanes == 1 &&
                pr.partialLanes == 1 && pr.failedLanes == 1 &&
                fabsf(pr.processedPercent - 100.0f) < 1e-3f &&
                fabsf(pr.completedPercent - 50.0f) < 1e-2f &&
                pr.estimatedDistancePercent > 50.0f && pr.estimatedDistancePercent < 100.0f,
                "P03: Mixed statuses yield RESULT_PARTIAL, no double counting, accurate percentages");
  }

  // P04: Làn đang RUNNING không được tính là completed hay processed
  {
    CoveragePlan p = makePlanWithLanes(); // 6 lanes
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0);
    tr.updateLaneCovered(0, p.lanes[0].startX, p.lanes[0].startY + 500.0f);
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(pr.currentLaneId == 0 &&
                pr.completedLanes == 0 &&
                pr.processedPercent == 0.0f &&
                pr.completedPercent == 0.0f &&
                pr.pendingLanes == p.laneCount - 1 &&
                pr.estimatedDistancePercent > 0.0f && pr.estimatedDistancePercent <= 100.0f,
                "P04: RUNNING lane excluded from completed/processed, updates estimatedDistancePercent");
  }

  // P05: Bắt đầu phiên mới reset hoàn toàn bộ đếm của phiên cũ
  {
    CoveragePlan p = makePlanWithLanes();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0); tr.onLaneCompleted(0);
    tr.onLaneStarted(1); tr.onLaneBlocked(1, p.lanes[1].startX, p.lanes[1].startY);
    tr.onSessionFinished();

    tr.beginSession(&p);
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(tr.getResult() == RESULT_RUNNING &&
                pr.completedLanes == 0 && pr.blockedLanes == 0 &&
                pr.partialLanes == 0 && pr.failedLanes == 0 &&
                pr.pendingLanes == p.laneCount &&
                pr.processedPercent == 0.0f && pr.completedPercent == 0.0f &&
                pr.estimatedDistancePercent == 0.0f,
                "P05: New session cleanly resets all counters and results");
  }

  printf("\nSUMMARY [test_progress]: Passed=%d Failed=%d\n", gPassed, gFailed);
  return (gFailed == 0) ? 0 : 1;
}
