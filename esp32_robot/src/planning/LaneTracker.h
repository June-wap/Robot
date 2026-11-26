#ifndef LANE_TRACKER_H
#define LANE_TRACKER_H

#include "LaneTypes.h"

// Ngưỡng phân biệt LANE_PARTIAL vs LANE_BLOCKED:
// Nếu đã lau được >= 15% chiều dài làn và >= 100 mm thì tính là PARTIAL.
#define LANE_PARTIAL_MIN_RATIO 0.15f
#define LANE_PARTIAL_MIN_MM    100.0f

class LaneTracker {
public:
  LaneTracker();

  void reset();
  void beginSession(CoveragePlan* plan);

  void onLaneStarted(int16_t laneId);
  void updateLaneCovered(int16_t laneId, float curX, float curY);
  void onLaneCompleted(int16_t laneId);
  void onLaneBlocked(int16_t laneId, float curX, float curY, const char* reason = "Vat can");
  void onLaneFailed(int16_t laneId, float curX, float curY, const char* reason = "Loi dieu huong");

  void onChangeLaneFailed(const char* reason = "Khong chuyen duoc lan");
  void onUserAborted();
  void onSessionFailed(const char* reason);
  void onSessionFinished();

  CleaningProgress getProgress() const;
  CleaningResult   getResult() const { return result_; }
  const char*      getLastReason() const { return lastReason_; }
  int16_t          getCurrentLaneId() const { return currentLaneId_; }

private:
  bool isValidLaneId(int16_t laneId) const;
  void setReason(const char* reason);

  CoveragePlan*  plan_;
  CleaningResult result_;
  int16_t        currentLaneId_;
  float          totalPlannedCleanMm_;
  char           lastReason_[64];
};

#endif // LANE_TRACKER_H
