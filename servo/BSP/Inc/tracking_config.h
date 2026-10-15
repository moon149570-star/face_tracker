#ifndef TRACKING_CONFIG_H
#define TRACKING_CONFIG_H

/* 舵机标定：沿用项目的500～2500 us映射，不代表所有MG996R的实测范围。 */
#define PAN_PULSE_MIN_US       500U
#define PAN_PULSE_MAX_US       2500U
#define TILT_PULSE_MIN_US      500U
#define TILT_PULSE_MAX_US      2500U
#define PAN_HOME_DEG           90.0f
#define TILT_HOME_DEG          45.0f
#define PAN_MIN_DEG            10.0f
#define PAN_MAX_DEG            170.0f
#define TILT_MIN_DEG           10.0f
#define TILT_MAX_DEG           135.0f

/* 目标越追越偏时只反转对应轴符号，取值为+1或-1。 */
#define PAN_DIRECTION          1.0f
#define TILT_DIRECTION         1.0f

/* 时间参数：20 ms目标周期，1 s启动等待，200 ms失联保持。 */
#define TRACK_PERIOD_MS        20U
#define TRACK_HOME_SETTLE_MS   1000U
#define TRACK_TIMEOUT_MS       200U
#define TRACK_FILTER_TAU_S     0.040f
#define TRACK_DEAD_ENTER       0.015f  /* 小于此误差时进入居中区。 */
#define TRACK_DEAD_EXIT        0.030f  /* 大于此误差时重新跟踪。 */

/* 输入为归一化误差，PID输出为度/秒；参数与本次整理前保持一致。 */
#define PAN_KP                 100.0f
#define PAN_KI                 3.0f
#define PAN_KD                 1.5f
#define TILT_KP                80.0f
#define TILT_KI                2.0f
#define TILT_KD                1.2f
#define PID_INTEGRAL_LIMIT     12.0f
#define PID_DERIVATIVE_TAU_S   0.060f
#define PAN_MAX_SPEED_DPS      120.0f
#define TILT_MAX_SPEED_DPS     90.0f
#define PAN_ACCEL_DPS2         600.0f
#define TILT_ACCEL_DPS2        450.0f

/* 保留现有OLED开关；接收队列长度必须是2的幂，实际容量为长度减1。 */
#define TRACK_OLED_ENABLE      1
#define SERIAL_RX_RING_SIZE    256U
#define SERIAL_PARTIAL_MS      30U

#endif
