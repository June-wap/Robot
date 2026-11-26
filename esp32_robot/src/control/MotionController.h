#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

void drive(int l, int r);
void stopMotors();
void emergencyStopMotors();
int  rampAxis(int cur, int tgt);

#endif // MOTION_CONTROLLER_H
