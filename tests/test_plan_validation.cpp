#include <cstdio>
#include <cmath>
#include "../esp32_robot/src/planning/LaneTypes.h"
#include "../esp32_robot/src/planning/CoveragePlanner.h"

static int gPassed = 0;
static int gFailed = 0;

#define EXPECT_TRUE(cond, name) do { \
  if (cond) { gPassed++; printf("[PASS] %s\n", name); } \
  else { gFailed++; printf("[FAIL] %s (line %d)\n", name, __LINE__); } \
} while (0)

static PlanRequest makeReq(float w, float l, float rx, float ry, float psi,
                           float ov, CleaningDirection dir) {
  PlanRequest r;
  r.roomW = w;
  r.roomL = l;
  r.edgeMm = 210.0f;
  r.mopWidthMm = 180.0f;
  r.overlapPercent = ov;
  r.robotRadiusMm = 170.0f;
  r.robotX = rx;
  r.robotY = ry;
  r.robotPsiDeg = psi;
  r.requestedDirection = dir;
  r.nominalLinearSpeedMmS = 194.1f;
  r.nominalAngularSpeedDegS = 86.8f;
  r.laneChangePenaltySec = 1.5f;
  return r;
}

int main() {
  printf("\n=== GROUP D: PATH INTEGRITY & VALIDATION ===\n");

  // D01: Kiểm tra toàn vẹn trên tất cả 8 tổ hợp (2 hướng x 4 góc)
  {
    PlanRequest r = makeReq(3200, 4800, 1500, 2200, 30.0f, 10.0f, DIRECTION_AUTO);
    bool allOk = true;
    const CleaningDirection dirs[2] = { DIRECTION_X, DIRECTION_Y };
    const StartCorner corners[4] = {
      CORNER_BOTTOM_LEFT, CORNER_BOTTOM_RIGHT, CORNER_TOP_LEFT, CORNER_TOP_RIGHT
    };
    for (int d = 0; d < 2; d++) {
      for (int c = 0; c < 4; c++) {
        for (int yf = 0; yf < 2; yf++) {
          CoveragePlan cand;
          PlanError e = CoveragePlanner::buildCandidate(r, dirs[d], corners[c], yf == 1, cand);
          if (e != PLAN_OK || !CoveragePlanner::verifyPlanIntegrity(cand)) allOk = false;

          // Kiểm tra không có bước 0-length thừa
          float cx = r.robotX, cy = r.robotY;
          for (uint16_t i = 0; i < cand.stepCount; i++) {
            float dist = (cand.steps[i].type == GO_X)
                ? fabsf(cand.steps[i].val - cx)
                : fabsf(cand.steps[i].val - cy);
            if (dist < 0.5f) allOk = false;
            if (cand.steps[i].type == GO_X) cx = cand.steps[i].val;
            else cy = cand.steps[i].val;
          }
        }
      }
    }
    EXPECT_TRUE(allOk, "D01: All 16 candidate variants pass integrity & non-zero step checks");
  }

  // D02: Robot đã đứng ngay tại góc xuất phát -> không sinh bước APPROACH_START thừa
  {
    PlanRequest r = makeReq(3000, 5000, 210.0f, 210.0f, 0.0f, 10.0f, DIRECTION_Y);
    CoveragePlan p;
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    bool noApproach = true;
    for (uint16_t i = 0; i < p.stepCount; i++) {
      if (p.steps[i].purpose == APPROACH_START) noApproach = false;
    }
    EXPECT_TRUE(e == PLAN_OK && noApproach && p.metrics.approachDistanceMm < 1e-3f,
                "D02: Robot at start corner generates zero redundant APPROACH_START steps");
  }

  // D03: Kiểm tra metric khớp chính xác với hình học đường đi
  {
    PlanRequest r = makeReq(3000, 5000, 210.0f, 210.0f, 0.0f, 10.0f, DIRECTION_Y);
    CoveragePlan p;
    CoveragePlanner::generateBestPlan(r, p);
    float expectedClean = (float)p.laneCount * (5000.0f - 420.0f);
    float expectedTrans = 3000.0f - 420.0f;
    EXPECT_TRUE(fabsf(p.metrics.cleaningDistanceMm - expectedClean) < 1.0f &&
                fabsf(p.metrics.transitionDistanceMm - expectedTrans) < 1.0f &&
                fabsf(p.metrics.totalDistanceMm - (expectedClean + expectedTrans)) < 1.0f,
                "D03: Computed metrics match exact path geometry");
  }

  // D04: Đường đi phù hợp góc xuất phát TOP_RIGHT + DIRECTION_X
  {
    PlanRequest r = makeReq(5000, 3000, 4790.0f, 2790.0f, -90.0f, 10.0f, DIRECTION_X);
    CoveragePlan p;
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(e == PLAN_OK &&
                p.chosenDirection == DIRECTION_X &&
                p.startCorner == CORNER_TOP_RIGHT &&
                fabsf(p.lanes[0].startX - 4790.0f) < 0.5f &&
                fabsf(p.lanes[0].startY - 2790.0f) < 0.5f &&
                fabsf(p.lanes[0].endX - 210.0f) < 0.5f &&
                fabsf(p.lanes[1].startX - 210.0f) < 0.5f &&
                fabsf(p.lanes[1].endX - 4790.0f) < 0.5f,
                "D04: Zigzag path alternates accurately from TOP_RIGHT along X");
  }

  // D05: Kiểm tra atomic: khi yêu cầu mới lỗi, kế hoạch cũ bị vô hiệu hóa sạch (không giữ kế hoạch cũ)
  {
    CoveragePlan p;
    CoveragePlanner::generateBestPlan(makeReq(3000, 5000, 500, 500, 0, 10, DIRECTION_AUTO), p);
    PlanRequest bad = makeReq(200, 200, 100, 100, 0, 10, DIRECTION_AUTO);
    PlanError e = CoveragePlanner::generateBestPlan(bad, p);
    EXPECT_TRUE(e == PLAN_ERR_ROOM_TOO_SMALL && !p.valid && p.laneCount == 0 && p.stepCount == 0,
                "D05: Failed plan invalidates output cleanly (no stale plan reused)");
  }

  // D06: Vị trí robot ngoài phòng hoặc edge < robotRadius bị từ chối
  {
    CoveragePlan p;
    PlanRequest badPose = makeReq(3000, 5000, 3500, 1000, 0, 10, DIRECTION_AUTO);
    PlanRequest badEdge = makeReq(3000, 5000, 1000, 1000, 0, 10, DIRECTION_AUTO);
    badEdge.edgeMm = 150.0f; // < robotRadius 170
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(badPose, p) == PLAN_ERR_INVALID_POSE &&
                CoveragePlanner::generateBestPlan(badEdge, p) == PLAN_ERR_INVALID_EDGE,
                "D06: Invalid robot pose and edge < robotRadius rejected");
  }

  // D07: Kiểm tra StepPurpose và laneId trên toàn bộ danh sách bước
  {
    CoveragePlan p;
    CoveragePlanner::generateBestPlan(makeReq(3000, 5000, 1000, 1000, 0, 10, DIRECTION_AUTO), p);
    bool ok = true;
    int cleanCnt = 0, shiftCnt = 0;
    for (uint16_t i = 0; i < p.stepCount; i++) {
      if (p.steps[i].purpose == CLEAN_LANE) {
        if (p.steps[i].laneId != cleanCnt) ok = false;
        cleanCnt++;
      } else if (p.steps[i].purpose == CHANGE_LANE) {
        if (p.steps[i].laneId != -1) ok = false;
        shiftCnt++;
      } else if (p.steps[i].purpose == APPROACH_START) {
        if (p.steps[i].laneId != -1) ok = false;
      }
    }
    EXPECT_TRUE(ok && cleanCnt == (int)p.laneCount && shiftCnt == (int)p.laneCount - 1,
                "D07: StepPurpose and laneId mapping strictly consistent");
  }

  printf("\nSUMMARY [test_plan_validation]: Passed=%d Failed=%d\n", gPassed, gFailed);
  return (gFailed == 0) ? 0 : 1;
}
