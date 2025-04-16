#ifndef _CONFIG_H
#define _CONFIG_H
#include <cstdint>

#define UART_ID uart1
#define BAUD_RATE 1000000
#define UART_TX_PIN 4
#define UART_RX_PIN 5

constexpr int ServoNum = 22;
constexpr int tickRate = 25;

struct ServoLimits {
  int16_t lowerAngle;
  int16_t upperAngle;
};

// Array of deviation values from the middle (each deviation can be -2048 to +2047)
const int deviations[ServoNum + 1][2] = {
    {0, 0},        // ID 0: NULL
    {-1536, 1536}, // ID 1: SHOULDER_MAIN_R
    {-1536, 1536}, // ID 2: SHOULDER_MAIN_L
    {-1024, 1024}, // ID 3: SHOULDER_TILT_R
    {-1024, 1024}, // ID 4: SHOULDER_TILT_L
    {-1024, 1024}, // ID 5: ELBOW_ROTATE_R
    {-1024, 1024}, // ID 6: ELBOW_ROTATE_L
    {-1536, 1024}, // ID 7: ELBOW_MAIN_R
    {-1024, 1536}, // ID 8: ELBOW_MAIN_L
    {-1024, 1024}, // ID 9: HIP_ROTATE_R
    {-1024, 1024}, // ID 10: HIP_ROTATE_L
    {-1024, 1024}, // ID 11: HIP_TILT_R
    {-1024, 1024}, // ID 12: HIP_TILT_L
    {-1024, 1024}, // ID 13: HIP_MAIN_R
    {-1024, 1024}, // ID 14: HIP_MAIN_L
    {0, 1536},     // ID 15: KNEE_R
    {-1536, 0},    // ID 16: KNEE_L
    {-1024, 1024}, // ID 17: FOOT_MAIN_R
    {-1024, 1024}, // ID 18: FOOT_MAIN_L
    {-1024, 1024}, // ID 19: FOOT_TILT_R
    {-1024, 1024}, // ID 20: FOOT_TILT_L
    {-1024, 1024}, // ID 21: HEAD_HORIZONTAL
    {-1024, 1024}  // ID 22: HEAD_VERTICAL
};

#endif