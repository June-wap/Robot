#ifndef COVERAGE_PLANNER_H
#define COVERAGE_PLANNER_H

#include "LaneTypes.h"

// ============================================================================
// Adaptive Zigzag Coverage Planner V2 - Level 1 (Hardware-independent C++)
// - Hỗ trợ hướng lau DIRECTION_AUTO, DIRECTION_X, DIRECTION_Y
// - Đánh giá tối đa 8 candidate (2 hướng x 4 góc xuất phát) x 2 thứ tự tiếp cận
// - Đánh giá metadata nhẹ trên stack (< 256 byte trên ESP32), không cấp phát heap động
// - Hoàn toàn REENTRANT & thread-safe (không sử dụng biến static mutable)
// - Atomic plan commit: chỉ ghi kế hoạch đích sau khi xây dựng & kiểm định thành công;
//   khi thất bại sẽ xóa sạch outPlan (outPlan.valid = false) để tránh dùng lại kế hoạch cũ.
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

  // Đánh giá tất cả candidate hợp lệ bằng metadata nhẹ và dựng phương án tốt nhất (atomic)
  // Nếu thất bại, outPlan.valid = false và trả về mã lỗi PlanError.
  static PlanError generateBestPlan(const PlanRequest& req, CoveragePlan& outPlan);

  // Kiểm định toàn vẹn hình học, số học, enum & tính liên tục đường đi của kế hoạch
  static bool verifyPlanIntegrity(const CoveragePlan& plan);

  // Khoảng cách Manhattan từ vị trí robot tới góc xuất phát
  static float computeApproachManhattan(const PlanRequest& req, StartCorner corner);

  // Lấy tọa độ góc xuất phát (đã xác minh theo quy ước hệ tọa độ firmware)
  static void getCornerCoords(float roomW, float roomL, float edgeMm,
                              StartCorner corner, float& outX, float& outY);

private:
  struct CandidateEval {
    bool              valid;
    CleaningDirection dir;
    StartCorner       corner;
    bool              approachYFirst;
    PlanMetrics       metrics;
  };

  static float     wrapAngle180(float deg);
  static bool      isBetterMetrics(const PlanMetrics& a, const PlanMetrics& b);
  static PlanError computeLaneGrid(float crossSpan, float effSpacing,
                                   uint16_t& outLaneCount, float& outActualSpacing);
  static PlanError evaluateCandidateMetrics(const PlanRequest& req,
                                            CleaningDirection dir,
                                            StartCorner corner,
                                            bool approachYFirst,
                                            PlanMetrics& outMetrics);
  static PlanError computeMetricsFromSteps(const PlanRequest& req, CoveragePlan& plan);
  static void      clearPlan(CoveragePlan& plan, PlanError err);
};

#endif // COVERAGE_PLANNER_H
