#include <cstdio>
#include <cmath>
#include <limits>
#include <chrono>
#include <cstring>
#include "../esp32_robot/LaneTypes.h"
#include "../esp32_robot/CoveragePlanner.h"

static int gPassed = 0;
static int gFailed = 0;

#define EXPECT_TRUE(cond, name) do { \
  if (cond) { gPassed++; printf("[PASS] %s\n", name); } \
  else { gFailed++; printf("[FAIL] %s (line %d)\n", name, __LINE__); } \
} while (0)

static PlanRequest makeBaseReq(float w, float l, float rx = -1.0f, float ry = -1.0f,
                               float psi = 0.0f, float overlap = 10.0f,
                               CleaningDirection dir = DIRECTION_AUTO) {
  PlanRequest r;
  r.roomW = w;
  r.roomL = l;
  r.edgeMm = 210.0f;
  r.mopWidthMm = 180.0f;
  r.overlapPercent = overlap;
  r.robotRadiusMm = 170.0f;
  r.robotX = (rx >= 0.0f) ? rx : w * 0.5f;
  r.robotY = (ry >= 0.0f) ? ry : l * 0.5f;
  r.robotPsiDeg = psi;
  r.requestedDirection = dir;
  // Tái sử dụng thông số động học từ firmware (DEF_CRUISE_PWM=165, V_MAX=300, SPIN=150)
  r.nominalLinearSpeedMmS = (165.0f / 255.0f) * 300.0f;
  r.nominalAngularSpeedDegS = (147.5f / 255.0f) * 150.0f;
  r.laneChangePenaltySec = PLANNER_DEF_LANE_PENALTY_S;
  return r;
}

// Mô phỏng thuật toán gốc V1 (cố định hướng Y, cố định góc BOTTOM_LEFT, bước = span / ceil(span / eff))
static PlanMetrics runLegacyAlgorithm(const PlanRequest& req, float overlapPct,
                                      uint16_t& outLanes, float& outSpacing) {
  CoveragePlan legacy;
  PlanRequest r = req;
  r.overlapPercent = overlapPct;
  r.requestedDirection = DIRECTION_Y;
  CoveragePlanner::buildCandidate(r, DIRECTION_Y, CORNER_BOTTOM_LEFT, false, legacy);
  outLanes = legacy.laneCount;
  outSpacing = legacy.actualSpacingMm;
  return legacy.metrics;
}

static void runGroupA_Geometry() {
  printf("\n=== GROUP A: GEOMETRY ===\n");
  CoveragePlan p;

  // A01: Phòng 3000 x 5000 mm -> tạo được cả X và Y
  {
    PlanRequest r = makeBaseReq(3000, 5000);
    CoveragePlan px, py;
    PlanError ex = CoveragePlanner::buildCandidate(r, DIRECTION_X, CORNER_BOTTOM_LEFT, false, px);
    PlanError ey = CoveragePlanner::buildCandidate(r, DIRECTION_Y, CORNER_BOTTOM_LEFT, false, py);
    EXPECT_TRUE(ex == PLAN_OK && ey == PLAN_OK && px.valid && py.valid, "A01: Room 3000x5000 supports X and Y");
  }
  // A02: Phòng 5000 x 3000 mm -> tạo đúng cả hai hướng
  {
    PlanRequest r = makeBaseReq(5000, 3000);
    CoveragePlan px, py;
    PlanError ex = CoveragePlanner::buildCandidate(r, DIRECTION_X, CORNER_TOP_RIGHT, true, px);
    PlanError ey = CoveragePlanner::buildCandidate(r, DIRECTION_Y, CORNER_TOP_RIGHT, true, py);
    EXPECT_TRUE(ex == PLAN_OK && ey == PLAN_OK && px.laneCount < py.laneCount, "A02: Room 5000x3000 both directions valid");
  }
  // A03: Phòng hình vuông -> lựa chọn deterministic
  {
    PlanRequest r = makeBaseReq(4000, 4000, 2000, 2000, 0.0f);
    CoveragePlan p1, p2;
    PlanError e1 = CoveragePlanner::generateBestPlan(r, p1);
    PlanError e2 = CoveragePlanner::generateBestPlan(r, p2);
    EXPECT_TRUE(e1 == PLAN_OK && e2 == PLAN_OK &&
                p1.chosenDirection == p2.chosenDirection &&
                p1.startCorner == p2.startCorner &&
                p1.laneCount == p2.laneCount,
                "A03: Square room deterministic selection");
  }
  // A04: Phòng hẹp (đủ 1 làn dọc Y: W=420=2*edge, L=2000)
  {
    PlanRequest r = makeBaseReq(420, 2000, 210, 500, 0.0f);
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(e == PLAN_OK && p.valid && p.laneCount == 1 && p.chosenDirection == DIRECTION_Y,
                "A04: Narrow corridor creates 1 valid lane");
  }
  // A05: Phòng quá nhỏ (W=400 < 2*210)
  {
    PlanRequest r = makeBaseReq(400, 400, 200, 200);
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(e == PLAN_ERR_ROOM_TOO_SMALL && !p.valid, "A05: Room too small rejected");
  }
  // A06: Bề rộng lau bằng 0
  {
    PlanRequest r = makeBaseReq(3000, 4000);
    r.mopWidthMm = 0.0f;
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_ERR_INVALID_MOP_WIDTH && !p.valid,
                "A06: Zero mop width rejected");
  }
  // A07: Bề rộng lau âm
  {
    PlanRequest r = makeBaseReq(3000, 4000);
    r.mopWidthMm = -120.0f;
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_ERR_INVALID_MOP_WIDTH && !p.valid,
                "A07: Negative mop width rejected");
  }
  // A08: Kích thước NaN hoặc Infinity
  {
    PlanRequest r1 = makeBaseReq(std::numeric_limits<float>::quiet_NaN(), 4000);
    PlanRequest r2 = makeBaseReq(3000, std::numeric_limits<float>::infinity());
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r1, p) == PLAN_ERR_INVALID_ROOM &&
                CoveragePlanner::generateBestPlan(r2, p) == PLAN_ERR_INVALID_ROOM,
                "A08: NaN and Infinity room dimensions rejected");
  }
  // A09: Overlap không hợp lệ (< 0 hoặc > 30)
  {
    PlanRequest r1 = makeBaseReq(3000, 4000, -1, -1, 0, -5.0f);
    PlanRequest r2 = makeBaseReq(3000, 4000, -1, -1, 0, 35.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r1, p) == PLAN_ERR_INVALID_OVERLAP &&
                CoveragePlanner::generateBestPlan(r2, p) == PLAN_ERR_INVALID_OVERLAP,
                "A09: Out-of-range overlap rejected");
  }
  // A10: Vượt giới hạn MAX_LANES (phòng 20000x20000 với mop=180 -> > 64 làn)
  {
    PlanRequest r = makeBaseReq(20000, 20000);
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(e == PLAN_ERR_EXCEEDS_MAX_LANES && !p.valid,
                "A10: Exceeding MAX_LANES safely rejected");
  }
}

static void runGroupB_DirectionAndCorner() {
  printf("\n=== GROUP B: DIRECTION AND CORNER ===\n");
  CoveragePlan p;

  // B01: Phòng dài theo Y -> chọn Y
  {
    PlanRequest r = makeBaseReq(3000, 5000);
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(e == PLAN_OK && p.chosenDirection == DIRECTION_Y && p.candidatesEvaluated == 8,
                "B01: Long-Y room selects DIRECTION_Y (8 candidates evaluated)");
  }
  // B02: Phòng dài theo X -> chọn X
  {
    PlanRequest r = makeBaseReq(5000, 3000);
    PlanError e = CoveragePlanner::generateBestPlan(r, p);
    EXPECT_TRUE(e == PLAN_OK && p.chosenDirection == DIRECTION_X && p.candidatesEvaluated == 8,
                "B02: Long-X room selects DIRECTION_X (8 candidates evaluated)");
  }
  // B03: Robot gần Bottom Left
  {
    PlanRequest r = makeBaseReq(3000, 5000, 300, 300, 0.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.startCorner == CORNER_BOTTOM_LEFT,
                "B03: Robot near BOTTOM_LEFT chooses BOTTOM_LEFT");
  }
  // B04: Robot gần Bottom Right
  {
    PlanRequest r = makeBaseReq(3000, 5000, 2700, 300, 0.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.startCorner == CORNER_BOTTOM_RIGHT,
                "B04: Robot near BOTTOM_RIGHT chooses BOTTOM_RIGHT");
  }
  // B05: Robot gần Top Left
  {
    PlanRequest r = makeBaseReq(3000, 5000, 300, 4700, 180.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.startCorner == CORNER_TOP_LEFT,
                "B05: Robot near TOP_LEFT chooses TOP_LEFT");
  }
  // B06: Robot gần Top Right
  {
    PlanRequest r = makeBaseReq(3000, 5000, 2700, 4700, 180.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.startCorner == CORNER_TOP_RIGHT,
                "B06: Robot near TOP_RIGHT chooses TOP_RIGHT");
  }
  // B07: Chọn X thủ công trên phòng dài Y
  {
    PlanRequest r = makeBaseReq(3000, 5000, -1, -1, 0, 10, DIRECTION_X);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.chosenDirection == DIRECTION_X && p.candidatesEvaluated == 4,
                "B07: Manual DIRECTION_X enforces X (4 candidates)");
  }
  // B08: Chọn Y thủ công trên phòng dài X
  {
    PlanRequest r = makeBaseReq(5000, 3000, -1, -1, 0, 10, DIRECTION_Y);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.chosenDirection == DIRECTION_Y && p.candidatesEvaluated == 4,
                "B08: Manual DIRECTION_Y enforces Y (4 candidates)");
  }
  // B09: Chi phí bằng nhau -> kết quả deterministic tuyệt đối
  {
    PlanRequest r = makeBaseReq(4000, 4000, 2000, 2000, 45.0f);
    CoveragePlan pa, pb;
    CoveragePlanner::generateBestPlan(r, pa);
    CoveragePlanner::generateBestPlan(r, pb);
    EXPECT_TRUE(pa.valid && pb.valid &&
                pa.chosenDirection == pb.chosenDirection &&
                pa.startCorner == pb.startCorner &&
                fabsf(pa.metrics.estimatedCost - pb.metrics.estimatedCost) < 1e-6f,
                "B09: Equal-cost tie-break is deterministic");
  }
}

static void runGroupC_Overlap() {
  printf("\n=== GROUP C: OVERLAP ===\n");
  CoveragePlan p;

  // C01: Overlap 0%
  {
    PlanRequest r = makeBaseReq(3000, 5000, -1, -1, 0, 0.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.actualSpacingMm <= r.mopWidthMm + 1e-3f,
                "C01: Overlap 0% spacing <= MopWidth");
  }
  // C02: Overlap 10%
  {
    PlanRequest r = makeBaseReq(3000, 5000, -1, -1, 0, 10.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.actualSpacingMm <= r.mopWidthMm * 0.90f + 1e-3f,
                "C02: Overlap 10% spacing <= 90% MopWidth");
  }
  // C03: Overlap 20%
  {
    PlanRequest r = makeBaseReq(3000, 5000, -1, -1, 0, 20.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.actualSpacingMm <= r.mopWidthMm * 0.80f + 1e-3f,
                "C03: Overlap 20% spacing <= 80% MopWidth");
  }
  // C04: Overlap 30%
  {
    PlanRequest r = makeBaseReq(3000, 5000, -1, -1, 0, 30.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.actualSpacingMm <= r.mopWidthMm * 0.70f + 1e-3f,
                "C04: Overlap 30% spacing <= 70% MopWidth");
  }
  // C05: Thay đổi overlap -> số làn tính lại tăng theo overlap
  {
    CoveragePlan p0, p10, p20, p30;
    CoveragePlanner::generateBestPlan(makeBaseReq(3000, 5000, -1, -1, 0, 0.0f), p0);
    CoveragePlanner::generateBestPlan(makeBaseReq(3000, 5000, -1, -1, 0, 10.0f), p10);
    CoveragePlanner::generateBestPlan(makeBaseReq(3000, 5000, -1, -1, 0, 20.0f), p20);
    CoveragePlanner::generateBestPlan(makeBaseReq(3000, 5000, -1, -1, 0, 30.0f), p30);
    EXPECT_TRUE(p0.laneCount < p10.laneCount && p10.laneCount < p20.laneCount &&
                p20.laneCount < p30.laneCount,
                "C05: Increasing overlap increases laneCount");
  }
  // C06: Chỉ có 1 làn hợp lệ -> không chia cho 0
  {
    PlanRequest r = makeBaseReq(420, 3000, 210, 1500, 0, 20.0f);
    EXPECT_TRUE(CoveragePlanner::generateBestPlan(r, p) == PLAN_OK &&
                p.laneCount == 1 && p.actualSpacingMm == 0.0f &&
                isfinite(p.metrics.estimatedCost),
                "C06: Single-lane room avoids division by zero");
  }
}

static void runBenchmark() {
  printf("\n=== STEP 17: ALGORITHM BENCHMARK (Legacy V1 vs Adaptive V2) ===\n");
  printf("sizeof(CoveragePlan) = %zu bytes | sizeof(CleaningLane) = %zu bytes | sizeof(Step) = %zu bytes\n",
         sizeof(CoveragePlan), sizeof(CleaningLane), sizeof(Step));

  struct RoomSpec { const char* name; float w; float l; float rx; float ry; float psi; };
  // Đặt robot gần góc TOP_RIGHT (mô phỏng trường hợp khởi động không ở BOTTOM_LEFT)
  const RoomSpec rooms[6] = {
    { "Room A (2000x3000)", 2000, 3000, 1700, 2600, 180.0f },
    { "Room B (3000x5000)", 3000, 5000, 2600, 4500, 180.0f },
    { "Room C (5000x3000)", 5000, 3000, 4600, 2600, 270.0f },
    { "Room D (4000x4000)", 4000, 4000, 3600, 3600, 180.0f },
    { "Room E (2000x6000)", 2000, 6000, 1700, 5500, 180.0f },
    { "Room F (6000x2000)", 6000, 2000, 5600, 1700, 270.0f }
  };
  const float overlaps[3] = { 0.0f, 10.0f, 20.0f };

  printf("\n| Room | Ov%% | Algo | Dir | Corner | Lanes | Space(mm) | Clean(m) | Trans(m) | Appr(m) | Total(m) | Turns | TurnDeg | Cost(s) | Time(us) |\n");
  printf("|------|-----|------|-----|--------|-------|-----------|----------|----------|---------|----------|-------|---------|---------|----------|\n");

  for (int ovIdx = 0; ovIdx < 3; ovIdx++) {
    float ov = overlaps[ovIdx];
    for (int i = 0; i < 6; i++) {
      PlanRequest req = makeBaseReq(rooms[i].w, rooms[i].l, rooms[i].rx, rooms[i].ry, rooms[i].psi, ov, DIRECTION_AUTO);

      uint16_t legLanes = 0;
      float legSpace = 0.0f;
      auto t0 = std::chrono::high_resolution_clock::now();
      PlanMetrics leg = runLegacyAlgorithm(req, ov, legLanes, legSpace);
      auto t1 = std::chrono::high_resolution_clock::now();
      double legUs = std::chrono::duration<double, std::micro>(t1 - t0).count();

      CoveragePlan v2;
      auto t2 = std::chrono::high_resolution_clock::now();
      CoveragePlanner::generateBestPlan(req, v2);
      auto t3 = std::chrono::high_resolution_clock::now();
      double v2Us = std::chrono::duration<double, std::micro>(t3 - t2).count();

      printf("| %s | %.0f%% | V1 | Y | BL | %u | %.1f | %.2f | %.2f | %.2f | %.2f | %u | %.0f | %.1f | %.1f |\n",
             rooms[i].name, ov, legLanes, legSpace,
             leg.cleaningDistanceMm / 1000.0f, leg.transitionDistanceMm / 1000.0f,
             leg.approachDistanceMm / 1000.0f, leg.totalDistanceMm / 1000.0f,
             leg.turnCount, leg.totalTurnDegrees, leg.estimatedCost, legUs);

      printf("| %s | %.0f%% | V2 | %s | %s | %u | %.1f | %.2f | %.2f | %.2f | %.2f | %u | %.0f | %.1f | %.1f |\n",
             rooms[i].name, ov, directionToStr(v2.chosenDirection), cornerToStr(v2.startCorner),
             v2.laneCount, v2.actualSpacingMm,
             v2.metrics.cleaningDistanceMm / 1000.0f, v2.metrics.transitionDistanceMm / 1000.0f,
             v2.metrics.approachDistanceMm / 1000.0f, v2.metrics.totalDistanceMm / 1000.0f,
             v2.metrics.turnCount, v2.metrics.totalTurnDegrees, v2.metrics.estimatedCost, v2Us);
    }
  }
}

int main() {
  runGroupA_Geometry();
  runGroupB_DirectionAndCorner();
  runGroupC_Overlap();
  runBenchmark();
  printf("\nSUMMARY [test_coverage_planner]: Passed=%d Failed=%d\n", gPassed, gFailed);
  return (gFailed == 0) ? 0 : 1;
}
