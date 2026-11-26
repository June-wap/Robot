#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

// ======================= CẤU HÌNH PHẦN CỨNG =======================
#define LIDAR_RX_PIN      16
#define LINK_TX_PIN       19
#define LINK_RX_PIN       18

constexpr int US_TRIG[3] = {25, 27, 32};   // 0 trước, 1 trái, 2 phải
constexpr int US_ECHO[3] = {26, 14, 33};

#define LIDAR_BAUD        230400
#define LINK_BAUD         38400
#define LIDAR_RX_BUF_SIZE 2048

#define LIDAR_OFFSET_DEG  0
#define LIDAR_CLOCKWISE   true
#define LIDAR_MIN_MM      100
#define LIDAR_MAX_MM      8000
#define LIDAR_MIN_INTENS  10

#endif // HARDWARE_CONFIG_H
