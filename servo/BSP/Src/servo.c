#include "servo.h"
#include "tim.h"
#include "tracking_config.h"

Servo_HandleTypeDef servo1, servo2;

void Servo_SetPulse(Servo_HandleTypeDef *servo, uint16_t pulse_width)
{
    if (pulse_width < servo->min_pulse) {
        pulse_width = servo->min_pulse;
    }
    if (pulse_width > servo->max_pulse) {
        pulse_width = servo->max_pulse;
    }

    /* TIM1以1 MHz计数，比较值可直接使用微秒数。 */
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_width);
}

void Servo_SetAngle(Servo_HandleTypeDef *servo, float angle)
{
    uint16_t pulse;

    if (angle < 0.0f) {
        angle = 0.0f;
    }
    if (angle > 180.0f) {
        angle = 180.0f;
    }
    pulse = (uint16_t)(servo->min_pulse +
        (servo->max_pulse - servo->min_pulse) * (angle / 180.0f) + 0.5f);
    Servo_SetPulse(servo, pulse);
}

void Servo_InitAll(void)
{
    servo1.htim = &htim1;
    servo1.channel = TIM_CHANNEL_1;
    servo1.min_pulse = PAN_PULSE_MIN_US;
    servo1.max_pulse = PAN_PULSE_MAX_US;

    servo2.htim = &htim1;
    servo2.channel = TIM_CHANNEL_2;
    servo2.min_pulse = TILT_PULSE_MIN_US;
    servo2.max_pulse = TILT_PULSE_MAX_US;

    /* 两路CCR都准备好并产生更新事件后，再开启PWM，避免输出旧初始脉宽。 */
    Servo_SetAngle(&servo1, PAN_HOME_DEG);
    Servo_SetAngle(&servo2, TILT_HOME_DEG);
    if (HAL_TIM_GenerateEvent(&htim1, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}
