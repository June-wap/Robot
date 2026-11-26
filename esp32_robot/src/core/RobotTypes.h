#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

struct Wall {
  bool  ok;
  float dist;
  float phi;
};

enum State {
  ST_IDLE,
  ST_CALIB,
  ST_RUN,
  ST_DONE,
  ST_ERR
};

enum Phase {
  PH_TURN,
  PH_DRIVE,
  PH_BACKOFF
};

#endif // ROBOT_TYPES_H
