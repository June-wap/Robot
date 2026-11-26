#ifndef LANE_TRACKER_H
#define LANE_TRACKER_H

#include "LaneTypes.h"

// Ngưỡng phân biệt LANE_PARTIAL vs LANE_BLOCKED:
// Nếu đã lau hợp lệ >= 15% chiều dài làn và >= 100 mm thì tính là PARTIAL.
#define LANE_PARTIAL_MIN_RATIO   0.15f
#define LANE_PARTIAL_MIN_MM      100.0f

// Ngưỡng xác minh vị trí & sai lệch ngang (Cross-Track) theo ước lượng pose robot.
// Mục đích: xác minh tính hợp lệ của tiến độ/hoàn thành làn trong phần mềm điều hướng
// (không khẳng định đã đo hoặc xác nhận diện tích lau vật lý thực tế).
#define LANE_CROSS_TRACK_MAX_MM  80.0f   // Sai lệch ngang tối đa so với tâm làn (< nửa bề rộng chổi 90 mm)
#define LANE_START_ALONG_TOL_MM  100.0f  // Sai lệch dọc tối đa tại điểm xuất phát làn
#define LANE_END_ALONG_TOL_MM    80.0f   // Sai lệch dọc tối đa tại điểm kết thúc làn

class LaneTracker {
public:
  LaneTracker();

  void reset();
  void onCalibrationStarted();
  void beginSession(CoveragePlan* plan);

  void onLaneStarted(int16_t laneId);
  void updateLaneCovered(int16_t laneId, float curX, float curY);
  void onLaneCompleted(int16_t laneId);
  bool completeLaneWithPose(int16_t laneId, float curX, float curY);
  void onLaneBlocked(int16_t laneId, float curX, float curY, const char* reason = "Vat can");
  void onLaneFailed(int16_t laneId, float curX, float curY, const char* reason = "Loi dieu huong");

  void onChangeLaneFailed(const char* reason = "Khong chuyen duoc lan");
  void onUserAborted();
  void onSessionFailed(const char* reason);
  void onSessionFinished();

  float computeCrossTrackError(int16_t laneId, float curX, float curY) const;
  bool  isWithinCrossTrack(int16_t laneId, float curX, float curY) const;
  bool  isPositionAtLaneStart(int16_t laneId, float curX, float curY) const;
  bool  isPositionAtLaneEnd(int16_t laneId, float curX, float curY) const;

  CleaningProgress getProgress() const;
  CleaningResult   getResult() const { return result_; }
  const char*      getLastReason() const { return lastReason_; }
  int16_t          getCurrentLaneId() const { return currentLaneId_; }
  bool             isCurrentLaneOffTrack() const { return currentLaneOffTrack_; }
  float            getCurrentLaneMaxCrossTrackMm() const { return currentLaneMaxCrossTrackMm_; }

private:
  bool  isValidLaneId(int16_t laneId) const;
  float computeAlongProgress(const CleaningLane& ln, float curX, float curY) const;
  void  setReason(const char* reason);

  CoveragePlan*  plan_;
  CleaningResult result_;
  int16_t        currentLaneId_;
  float          totalPlannedCleanMm_;
  bool           currentLaneOffTrack_;
  float          currentLaneMaxCrossTrackMm_;
  char           lastReason_[64];
};

#endif // LANE_TRACKER_H
