#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ======================= THÔNG SỐ ROBOT & BẢO VỆ L293D =======================
#define ROBOT_RADIUS_MM    170
#define DEF_EDGE_MM        (ROBOT_RADIUS_MM + 40)
#define DEF_MOP_WIDTH_MM   180.0f
#define V_MAX_MM_S         300.0f
#define SPIN_MAX_DEG_S     150.0f

// Giới hạn trần PWM & bước tăng tốc mềm (mỗi 50 ms) để giảm dòng đỉnh bảo vệ L293D
#define MAX_SAFE_PWM       210
#define DEF_CRUISE_PWM     165
#define SLOW_PWM           110
#define MIN_TURN_PWM       120
#define MAX_TURN_PWM       175
#define MOP_PWM            200
#define PWM_RAMP_STEP      30    // Tăng/giảm tối đa 30 PWM / 50 ms (~0.3s từ 0 -> 170)
#define PWM_STOP_STEP      90    // Giảm tốc nhanh hơn khi về 0

#define KT                 3.0f
#define KH                 2.5f
#define TURN_TOL_DEG       3.0f
#define K_YAW              0.8f
#define K_POS              0.5f
#define POS_JUMP_MAX_MM    300.0f

#define US_BLOCK_MM        55
#define LIDAR_BLOCK_MM     (ROBOT_RADIUS_MM + 25)
#define SIDE_NEAR_MM       60
#define WALL_HALF_DEG      25
#define US_TIMEOUT_MS      14    // ~2.4 m, không block CPU
#define US_PERIOD_MS       30
#define BLOCK_CONFIRM_CNT  3     // 3 chu kỳ dữ liệu cảm biến thực liên tiếp
#define DEF_OVERLAP_PCT    10.0f // Chế độ STANDARD = 10% (phạm vi 0..30%)

#endif // ROBOT_CONFIG_H
