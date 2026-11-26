#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include "LaneTypes.h"

// ============================================================================
// Adaptive Zigzag Coverage Planner V2 - Level 1 (Hardware-independent C++)
// - Hỗ trợ hướng lau DIRECTION_AUTO, DIRECTION_X, DIRECTION_Y
// - Đánh giá tối đa 8 candidate (2 hướng x 4 góc xuất phát) x 2 thứ tự tiếp cận
// - Không phụ thuộc Arduino.h, không cấp phát heap động, có thể unit test trên PC
// ============================================================================
class CoveragePlanner {
public:
  // Kiểm tra toàn bộ đầu vào trước khi lập kế hoạch
  static PlanError validateRequest(const PlanRequest& req);

  // Xây dựng 1 phương án cụ thể cho (dir, corner, approachYFirst)
  static PlanError buildCandidate(const PlanRequest& req,
                                  CleaningDirection dir,
                                  StartCorner corner,
                                  bool approachYFirst,
                                  CoveragePlan& outCand);

  // Đánh giá tất cả candidate hợp lệ và chọn phương án chi phí thấp nhất (atomic)
  // Nếu thất bại, outPlan.valid = false và trả về mã lỗi PlanError.
  static PlanError generateBestPlan(const PlanRequest& req, CoveragePlan& outPlan);

  // Kiểm định toàn vẹn hình học & logic của kế hoạch đã sinh
  static bool verifyPlanIntegrity(const CoveragePlan& plan);

  // Khoảng cách Manhattan từ vị trí robot tới góc xuất phát
  static float computeApproachManhattan(const PlanRequest& req, StartCorner corner);

  // Lấy tọa độ góc xuất phát (đã xác minh theo quy ước hệ tọa độ firmware)
  static void getCornerCoords(float roomW, float roomL, float edgeMm,
                              StartCorner corner, float& outX, float& outY);

private:
  static float wrapAngle180(float deg);
  static bool  isBetterCandidate(const CoveragePlan& a, const CoveragePlan& b);
  static void  computeMetricsFromSteps(const PlanRequest& req, CoveragePlan& plan);
  static void  clearPlan(CoveragePlan& plan, PlanError err);
};

#endif // COVERAGE_PLANNER_H
