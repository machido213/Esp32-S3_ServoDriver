#ifndef IO_CONFIG_H
#define IO_CONFIG_H

// === 馬達控制腳位 (7 軸) ===
#define MOTOR_AXIS_COUNT 7
typedef struct { int pul_pin; int dir_pin; } motor_pin_t;

// 依序為 Axis 1 ~ 7
static const motor_pin_t MOTOR_PINS[MOTOR_AXIS_COUNT] = {
    {4, 5}, {6, 7}, {8, 9}, {10, 11}, {12, 13}, {14, 15}, {16, 17}
};

// === 系統 IO ===
#define SERVO_ENABLE_GPIO 42      // 伺服始能 (Output)
#define SYS_START_TRIGGER_GPIO 38 // 啟動繼電器 (Output)
#define SYS_POWER_CUTOFF_GPIO  39 // 斷電繼電器 (Output)
#define BTN_STOP_GPIO          40 // 停止按鈕 (Input)
#define SYS_STATE_FEEDBACK_GPIO 41 // 接觸器回饋 (Input)

// === 環境監控 ===
#define TEMP_SENSOR_GPIO 18
#define FAN_PWM_GPIO     21

#endif // IO_CONFIG_H