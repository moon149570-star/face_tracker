#ifndef FACE_PID_H
#define FACE_PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float integral;          /* 已乘Ki的积分贡献，单位与输出相同。 */
    float previous_error;
    float derivative;        /* 滤波后的误差变化率。 */
    float output_limit;
    float integral_limit;
    float derivative_tau;    /* 微分滤波时间常数，秒。 */
    uint8_t have_sample;     /* 首帧不计算微分，避免启动尖峰。 */
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit, float derivative_tau);
void PID_Reset(PID_TypeDef *pid);

/* dt为控制间隔，sample_dt为新测量间隔，均以秒计。 */
float PID_Update(PID_TypeDef *pid, float error, float dt,
                 uint8_t new_sample, float sample_dt);

#endif
