#include "pid.h"

/* 对称限幅：积分项和输出都以0为中心。 */
static float Clamp(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->derivative = 0.0f;
    pid->have_sample = 0U;
}

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd,
              float output_limit, float integral_limit, float derivative_tau)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->output_limit = output_limit;
    pid->integral_limit = integral_limit;
    pid->derivative_tau = derivative_tau;
    PID_Reset(pid);
}

float PID_Update(PID_TypeDef *pid, float error, float dt,
                 uint8_t new_sample, float sample_dt)
{
    float candidate_integral;
    float proportional_output;
    float derivative_output;
    float candidate_output;

    if (dt <= 0.0f) {
        return 0.0f;
    }

    /* 控制周期与视觉帧间隔不同：仅在新测量到达时更新微分。 */
    if (new_sample) {
        if (pid->have_sample && sample_dt > 0.0f) {
            float alpha = sample_dt / (pid->derivative_tau + sample_dt);
            float derivative = (error - pid->previous_error) / sample_dt;
            pid->derivative += alpha * (derivative - pid->derivative);
        }
        pid->previous_error = error;
        pid->have_sample = 1U;
    }

    proportional_output = pid->kp * error;
    derivative_output = pid->kd * pid->derivative;
    candidate_integral = Clamp(pid->integral + pid->ki * error * dt,
                               pid->integral_limit);
    candidate_output = proportional_output + candidate_integral + derivative_output;

    /* 未饱和，或误差有助于退出饱和时才积分，避免积分累积过多。 */
    if ((candidate_output <= pid->output_limit && candidate_output >= -pid->output_limit) ||
        (candidate_output > pid->output_limit && error < 0.0f) ||
        (candidate_output < -pid->output_limit && error > 0.0f)) {
        pid->integral = candidate_integral;
    }
    return Clamp(proportional_output + pid->integral + derivative_output,
                 pid->output_limit);
}
