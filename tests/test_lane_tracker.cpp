#include <cstdio>
#include <cmath>
#include <cstring>
#include "../esp32_robot/src/planning/LaneTypes.h"
#include "../esp32_robot/src/planning/CoveragePlanner.h"
#include "../esp32_robot/src/planning/LaneTracker.h"

static int gPassed = 0;
static int gFailed = 0;

#define EXPECT_TRUE(cond, name) do { \
  if (cond) { gPassed++; printf("[PASS] %s\n", name); } \
  else { gFailed++; printf("[FAIL] %s (line %d)\n", name, __LINE__); } \
} while (0)

static CoveragePlan makeSmallPlan() {
  PlanRequest r;
  r.roomW = 1000.0f;
  r.roomL = 2000.0f;
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
  printf("\n=== GROUP E (PART 1): LANE TRACKER & BLOCKED LANE HANDLING ===\n");

  // E01: Bắt đầu phiên đặt tất cả làn về PENDING và RESULT_RUNNING
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    bool allPending = (p.laneCount > 0);
    for (uint16_t i = 0; i < p.laneCount; i++) {
      if (p.lanes[i].status != LANE_PENDING) allPending = false;
    }
    EXPECT_TRUE(allPending && tr.getResult() == RESULT_RUNNING && tr.getCurrentLaneId() == -1,
                "E01: beginSession resets all lanes to PENDING and RESULT_RUNNING");
  }

  // E02: Chuyển trạng thái PENDING -> RUNNING -> COMPLETED
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0);
    bool isRunning = (p.lanes[0].status == LANE_RUNNING && tr.getCurrentLaneId() == 0);
    tr.onLaneCompleted(0);
    bool isComp = (p.lanes[0].status == LANE_COMPLETED &&
                   fabsf(p.lanes[0].estimatedCoveredLengthMm - p.lanes[0].plannedLengthMm) < 1e-3f &&
                   tr.getCurrentLaneId() == -1);
    EXPECT_TRUE(isRunning && isComp, "E02: PENDING -> RUNNING -> COMPLETED transition");
  }

  // E03: Vật cản ở đầu làn -> LANE_BLOCKED (không tăng completedLanes)
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0);
    // Mới đi được 40 mm (< 100 mm & < 15%)
    tr.onLaneBlocked(0, p.lanes[0].startX, p.lanes[0].startY + 40.0f, "Vat can dau lan");
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(p.lanes[0].status == LANE_BLOCKED &&
                pr.blockedLanes == 1 && pr.completedLanes == 0,
                "E03: Early obstacle marks LANE_BLOCKED without incrementing completedLanes");
  }

  // E04: Vật cản giữa làn (đã đi 800 mm / 1580 mm) -> LANE_PARTIAL
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0);
    tr.onLaneBlocked(0, p.lanes[0].startX, p.lanes[0].startY + 800.0f, "Vat can giua lan");
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(p.lanes[0].status == LANE_PARTIAL &&
                pr.partialLanes == 1 && pr.completedLanes == 0 &&
                fabsf(p.lanes[0].estimatedCoveredLengthMm - 800.0f) < 1.0f,
                "E04: Mid-lane obstacle marks LANE_PARTIAL with covered length");
  }

  // E05: Timeout / lỗi điều hướng -> LANE_FAILED (không đánh dấu hoàn thành)
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(1);
    tr.onLaneFailed(1, p.lanes[1].startX, p.lanes[1].startY, "Timeout xoay");
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(p.lanes[1].status == LANE_FAILED &&
                pr.failedLanes == 1 && pr.completedLanes == 0,
                "E05: Timeout marks LANE_FAILED and never completed");
  }

  // E06: Chuyển làn thất bại -> phiên chuyển RESULT_FAILED, không tiếp tục lau sai làn
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0);
    tr.onLaneCompleted(0);
    tr.onChangeLaneFailed("Vat can khi chuyen lan");
    EXPECT_TRUE(tr.getResult() == RESULT_FAILED && strlen(tr.getLastReason()) > 0,
                "E06: Failed CHANGE_LANE transitions session to RESULT_FAILED");
  }

  // E07: Người dùng dừng giữa chừng -> RESULT_ABORTED, bảo toàn trạng thái đã lau
  {
    CoveragePlan p = makeSmallPlan();
    LaneTracker tr;
    tr.beginSession(&p);
    tr.onLaneStarted(0);
    tr.onLaneCompleted(0);
    tr.onLaneStarted(1);
    tr.updateLaneCovered(1, p.lanes[1].startX, p.lanes[1].startY - 600.0f);
    tr.onUserAborted();
    CleaningProgress pr = tr.getProgress();
    EXPECT_TRUE(tr.getResult() == RESULT_ABORTED &&
                pr.completedLanes == 1 && pr.partialLanes == 1,
                "E07: User abort records RESULT_ABORTED and preserves completed/partial lanes");
  }

  printf("\nSUMMARY [test_lane_tracker]: Passed=%d Failed=%d\n", gPassed, gFailed);
  return (gFailed == 0) ? 0 : 1;
}
