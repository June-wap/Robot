#ifndef LANE_TYPES_H
#define LANE_TYPES_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// GIỚI HẠN BỘ NHỚ TĨNH CHO ESP32 (TÀI LIỆU HÓA THEO STEP 15)
// - MAX_LANES = 64: với MopWidth = 180 mm ở overlap tối đa 30% (effSpacing = 126 mm),
//   64 làn hỗ trợ chiều rộng phòng tới ~8.35 m (vượt tầm đo ổn định 8 m của LD14).
// - MAX_PLAN_STEPS = 160: 2 bước approach + 64 CLEAN_LANE + 63 CHANGE_LANE = 129 <= 160.
// - Bộ nhớ tĩnh 1 CoveragePlan ~ 4.1 KB (< 1.3% SRAM 320 KB của ESP32), không dùng heap động.
// ============================================================================
#define MAX_LANES           64
#define MAX_PLAN_STEPS      160
#define MAX_OVERLAP_PERCENT 30.0f
#define MIN_OVERLAP_PERCENT 0.0f

// Các hằng số mặc định phục vụ hàm chi phí (tái sử dụng từ thông số động học firmware)
#define PLANNER_TURN_TOL_DEG       3.0f
#define PLANNER_ZERO_STEP_EPS_MM   1.0f
#define PLANNER_MIN_LANE_LENGTH_MM 50.0f
#define PLANNER_DEF_LANE_PENALTY_S 1.5f

enum CleaningDirection {
  DIRECTION_AUTO = 0,
  DIRECTION_X    = 1,
  DIRECTION_Y    = 2
};

enum StartCorner {
  CORNER_BOTTOM_LEFT  = 0,
  CORNER_BOTTOM_RIGHT = 1,
  CORNER_TOP_LEFT     = 2,
  CORNER_TOP_RIGHT    = 3
};

enum OverlapMode {
  OVERLAP_QUICK    = 0,  // 0%
  OVERLAP_STANDARD = 1,  // 10%
  OVERLAP_DEEP     = 2,  // 20%
  OVERLAP_CUSTOM   = 3   // 0..30%
};

enum LaneStatus {
  LANE_PENDING   = 0,
  LANE_RUNNING   = 1,
  LANE_COMPLETED = 2,
  LANE_BLOCKED   = 3,
  LANE_PARTIAL   = 4,
  LANE_FAILED    = 5
};

enum StepType {
  GO_X = 0,
  GO_Y = 1
};

enum StepPurpose {
  APPROACH_START = 0,
  CLEAN_LANE     = 1,
  CHANGE_LANE    = 2
};

struct Step {
  StepType    type;
  float       val;
  StepPurpose purpose;
  int16_t     laneId;
};

enum CleaningResult {
  RESULT_NONE      = 0,
  RESULT_RUNNING   = 1,
  RESULT_COMPLETED = 2,
  RESULT_PARTIAL   = 3,
  RESULT_ABORTED   = 4,
  RESULT_FAILED    = 5
};

enum PlanError {
  PLAN_OK = 0,
  PLAN_ERR_INVALID_ROOM,
  PLAN_ERR_ROOM_TOO_SMALL,
  PLAN_ERR_INVALID_MOP_WIDTH,
  PLAN_ERR_INVALID_EDGE,
  PLAN_ERR_INVALID_OVERLAP,
  PLAN_ERR_INVALID_POSE,
  PLAN_ERR_INVALID_DIRECTION,
  PLAN_ERR_INVALID_KINEMATICS,
  PLAN_ERR_EXCEEDS_MAX_LANES,
  PLAN_ERR_EXCEEDS_MAX_STEPS,
  PLAN_ERR_INTEGRITY_FAILED
};

struct PlanMetrics {
  float    cleaningDistanceMm;
  float    transitionDistanceMm;
  float    approachDistanceMm;
  float    totalDistanceMm;
  uint16_t laneCount;
  uint16_t turnCount;
  float    totalTurnDegrees;
  float    estimatedCost;
};

struct CleaningLane {
  uint16_t   id;
  float      startX;
  float      startY;
  float      endX;
  float      endY;
  float      plannedLengthMm;
  float      estimatedCoveredLengthMm;
  LaneStatus status;
};

struct CleaningProgress {
  uint16_t totalLanes;
  uint16_t completedLanes;
  uint16_t blockedLanes;
  uint16_t partialLanes;
  uint16_t failedLanes;
  uint16_t pendingLanes;
  int16_t  currentLaneId;
  float    processedPercent;
  float    completedPercent;
  float    estimatedDistancePercent;
};

struct PlanRequest {
  float             roomW;
  float             roomL;
  float             edgeMm;
  float             mopWidthMm;
  float             overlapPercent;
  float             robotRadiusMm;
  float             robotX;
  float             robotY;
  float             robotPsiDeg;
  CleaningDirection requestedDirection;
  float             nominalLinearSpeedMmS;
  float             nominalAngularSpeedDegS;
  float             laneChangePenaltySec;
};

struct CoveragePlan {
  bool              valid;
  PlanError         error;
  CleaningDirection chosenDirection;  // DIRECTION_X hoặc DIRECTION_Y
  StartCorner       startCorner;
  bool              approachYFirst;
  float             roomW;
  float             roomL;
  float             edgeMm;
  float             mopWidthMm;
  float             overlapPercent;
  float             effectiveSpacingMm;
  float             actualSpacingMm;
  uint8_t           candidatesEvaluated;
  PlanMetrics       metrics;
  uint16_t          laneCount;
  CleaningLane      lanes[MAX_LANES];
  uint16_t          stepCount;
  Step              steps[MAX_PLAN_STEPS];
};

// ======================= HÀM TIỆN ÍCH CHUYỂN CHUỖI =======================
inline const char* directionToStr(CleaningDirection d) {
  switch (d) {
    case DIRECTION_X:    return "X";
    case DIRECTION_Y:    return "Y";
    case DIRECTION_AUTO: return "AUTO";
    default:             return "UNKNOWN";
  }
}

inline const char* cornerToStr(StartCorner c) {
  switch (c) {
    case CORNER_BOTTOM_LEFT:  return "BOTTOM_LEFT";
    case CORNER_BOTTOM_RIGHT: return "BOTTOM_RIGHT";
    case CORNER_TOP_LEFT:     return "TOP_LEFT";
    case CORNER_TOP_RIGHT:    return "TOP_RIGHT";
    default:                  return "UNKNOWN";
  }
}

inline const char* laneStatusToStr(LaneStatus s) {
  switch (s) {
    case LANE_PENDING:   return "PENDING";
    case LANE_RUNNING:   return "RUNNING";
    case LANE_COMPLETED: return "COMPLETED";
    case LANE_BLOCKED:   return "BLOCKED";
    case LANE_PARTIAL:   return "PARTIAL";
    case LANE_FAILED:    return "FAILED";
    default:             return "UNKNOWN";
  }
}

inline const char* cleaningResultToStr(CleaningResult r) {
  switch (r) {
    case RESULT_NONE:      return "NONE";
    case RESULT_RUNNING:   return "RUNNING";
    case RESULT_COMPLETED: return "COMPLETED";
    case RESULT_PARTIAL:   return "PARTIAL";
    case RESULT_ABORTED:   return "ABORTED";
    case RESULT_FAILED:    return "FAILED";
    default:               return "UNKNOWN";
  }
}

inline const char* planErrorToStr(PlanError e) {
  switch (e) {
    case PLAN_OK:                     return "OK";
    case PLAN_ERR_INVALID_ROOM:       return "Kich thuoc phong khong hop le";
    case PLAN_ERR_ROOM_TOO_SMALL:     return "Phong qua nho";
    case PLAN_ERR_INVALID_MOP_WIDTH:  return "Be rong choi lau khong hop le";
    case PLAN_ERR_INVALID_EDGE:       return "Khoang cach mep tuong khong hop le";
    case PLAN_ERR_INVALID_OVERLAP:    return "Ty le chong lan ngoai pham vi 0-30%";
    case PLAN_ERR_INVALID_POSE:       return "Vi tri hoac goc robot khong hop le";
    case PLAN_ERR_INVALID_DIRECTION:  return "Huong lau khong hop le";
    case PLAN_ERR_INVALID_KINEMATICS: return "Thong so van toc khong hop le";
    case PLAN_ERR_EXCEEDS_MAX_LANES:  return "So lan vuot gioi han bo nho";
    case PLAN_ERR_EXCEEDS_MAX_STEPS:  return "So buoc vuot gioi han bo nho";
    case PLAN_ERR_INTEGRITY_FAILED:   return "Ke hoach khong dat kiem dinh hinh hoc";
    default:                          return "Loi lap ke hoach";
  }
}

inline float overlapModeToPercent(OverlapMode m, float customVal = 10.0f) {
  switch (m) {
    case OVERLAP_QUICK:    return 0.0f;
    case OVERLAP_STANDARD: return 10.0f;
    case OVERLAP_DEEP:     return 20.0f;
    case OVERLAP_CUSTOM:   return customVal;
    default:               return 10.0f;
  }
}

#endif // LANE_TYPES_H
