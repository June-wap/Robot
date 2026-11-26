#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

bool frontBlockedRaw();
void expectedSideClearance(float heading, float& leftGap, float& rightGap);

#endif // OBSTACLE_DETECTOR_H
