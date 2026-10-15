#ifndef FACE_SERVO_H
#define FACE_SERVO_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t min_pulse, max_pulse; /* 0°/180°对应的脉宽标定值，微秒。 */
} Servo_HandleTypeDef;

extern Servo_HandleTypeDef servo1; /* PA8：下方水平舵机。 */
extern Servo_HandleTypeDef servo2; /* PA9：上方垂直舵机。 */

/* 角度限于0～180°，追踪阶段更严格的机械限位由FaceTracker负责。 */
void Servo_SetAngle(Servo_HandleTypeDef *servo, float angle);
void Servo_SetPulse(Servo_HandleTypeDef *servo, uint16_t pulse_width);
void Servo_InitAll(void);

#endif
