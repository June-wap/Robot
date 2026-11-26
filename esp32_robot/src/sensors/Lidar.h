#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>

void    lidarBeginSerial();
void    lidarInit();
void    initCrc();
uint8_t calcCrc(const uint8_t* p, int len);
void    handlePacket(const uint8_t* b);
void    pollLidar();

#endif // LIDAR_H
