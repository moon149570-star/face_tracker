#include "face_tracker.h"
#include "tracking_config.h"
#include <string.h>

static float Abs(float value)
{
    return value < 0.0f ? -value : value;
}

static float Clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static void StopAxis(TrackAxis *axis)
{
    PID_Reset(&axis->pid);
    axis->speed = 0.0f;
    axis->centered = 1U;
}

/* 丢失目标、重新捕获或更换分辨率时清除控制历史，角度保持不变。 */
static void ResetTracking(FaceTracker *tracker)
{
    tracker->filter_ready = 0U;
    tracker->new_sample = 0U;
    StopAxis(&tracker->pan);
    StopAxis(&tracker->tilt);
}

void FaceTracker_Init(FaceTracker *tracker, uint32_t now)
{
    memset(tracker, 0, sizeof(*tracker));
    tracker->boot_ms = now;
    tracker->last_step_ms = now;
    tracker->pan.angle = PAN_HOME_DEG;
    tracker->tilt.angle = TILT_HOME_DEG;
    tracker->pan.centered = 1U;
    tracker->tilt.centered = 1U;
    tracker->state = TRACK_HOMING;
    PID_Init(&tracker->pan.pid, PAN_KP, PAN_KI, PAN_KD,
             PAN_MAX_SPEED_DPS, PID_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S);
    PID_Init(&tracker->tilt.pid, TILT_KP, TILT_KI, TILT_KD,
             TILT_MAX_SPEED_DPS, PID_INTEGRAL_LIMIT, PID_DERIVATIVE_TAU_S);
}

void FaceTracker_OnFrame(FaceTracker *tracker, const FaceFrame *frame)
{
    float x, y, alpha, dt;

    /* 启动等待阶段的数据不参与跟踪，必须等到归位后的新测量。 */
    if ((uint32_t)(frame->received_ms - tracker->boot_ms) < TRACK_HOME_SETTLE_MS) {
        return;
    }
    if (tracker->have_face && frame->sequence == tracker->face.sequence) {
        return; /* 同序号重传不刷新超时，防止旧坐标维持运动。 */
    }

    dt = (float)(uint32_t)(frame->received_ms - tracker->previous_sample_ms) * 0.001f;
    if (!tracker->filter_ready || dt > (float)TRACK_TIMEOUT_MS * 0.001f ||
        frame->width != tracker->face.width || frame->height != tracker->face.height) {
        ResetTracking(tracker);
        dt = (float)TRACK_PERIOD_MS * 0.001f;
    }

    /* 同批数据可能具有相同时间戳，保留原控制器的5 ms除零保护下限。 */
    dt = Clamp(dt, 0.005f, (float)TRACK_TIMEOUT_MS * 0.001f);
    tracker->sample_dt = dt;
    tracker->previous_sample_ms = frame->received_ms;
    tracker->face = *frame;
    tracker->have_face = 1U;
    if (!frame->valid) {
        ResetTracking(tracker);
        return;
    }

    /* 中心为0，边缘约为±1；切换分辨率无需重新缩放PID增益。 */
    x = ((float)frame->x * 2.0f / (float)frame->width - 1.0f) * PAN_DIRECTION;
    y = ((float)frame->y * 2.0f / (float)frame->height - 1.0f) * TILT_DIRECTION;
    alpha = dt / (TRACK_FILTER_TAU_S + dt);
    if (!tracker->filter_ready) {
        tracker->pan.filtered_error = x;
        tracker->tilt.filtered_error = y;
    } else {
        tracker->pan.filtered_error += alpha * (x - tracker->pan.filtered_error);
        tracker->tilt.filtered_error += alpha * (y - tracker->tilt.filtered_error);
    }
    tracker->filter_ready = 1U;
    tracker->new_sample = 1U;
}

static void UpdateAxis(TrackAxis *axis, float dt, float sample_dt,
                       uint8_t new_sample, float acceleration, float low, float high)
{
    float error = axis->filtered_error;
    float desired_speed;
    float angle;

    /* 进入和离开居中区使用不同阈值，避免边界附近反复启停。 */
    if (axis->centered) {
        if (Abs(error) > TRACK_DEAD_EXIT) {
            axis->centered = 0U;
        }
    } else if (Abs(error) < TRACK_DEAD_ENTER) {
        axis->centered = 1U;
    }

    if (axis->centered) {
        PID_Reset(&axis->pid);
        desired_speed = 0.0f;
    } else {
        error += error > 0.0f ? -TRACK_DEAD_ENTER : TRACK_DEAD_ENTER;
        desired_speed = PID_Update(&axis->pid, error, dt, new_sample, sample_dt);
    }

    /* PID给出角速度；先限制速度变化量，再积分得到命令角度。 */
    axis->speed += Clamp(desired_speed - axis->speed, -acceleration * dt, acceleration * dt);
    angle = axis->angle + axis->speed * dt;
    axis->angle = Clamp(angle, low, high);
    if ((axis->angle <= low && axis->speed < 0.0f) ||
        (axis->angle >= high && axis->speed > 0.0f)) {
        axis->speed = 0.0f;
        PID_Reset(&axis->pid); /* 到达软件限位后不再积累积分。 */
    }
}

uint8_t FaceTracker_Update(FaceTracker *tracker, uint32_t now)
{
    uint32_t elapsed = (uint32_t)(now - tracker->last_step_ms);
    float dt;

    if (elapsed < TRACK_PERIOD_MS) {
        return 0U;
    }
    tracker->last_step_ms = now;
    if ((uint32_t)(now - tracker->boot_ms) < TRACK_HOME_SETTLE_MS) {
        tracker->state = TRACK_HOMING;
        return 1U;
    }

    /* 长时间未调度也停止运动，避免按过长dt一次跳过很大的角度。 */
    if (!tracker->have_face || !tracker->face.valid ||
        (uint32_t)(now - tracker->face.received_ms) > TRACK_TIMEOUT_MS ||
        elapsed > TRACK_PERIOD_MS * 3U) {
        tracker->state = TRACK_HOLD;
        ResetTracking(tracker);
        return 1U;
    }

    dt = (float)elapsed * 0.001f;
    tracker->state = TRACK_ACTIVE;
    UpdateAxis(&tracker->pan, dt, tracker->sample_dt, tracker->new_sample,
               PAN_ACCEL_DPS2, PAN_MIN_DEG, PAN_MAX_DEG);
    UpdateAxis(&tracker->tilt, dt, tracker->sample_dt, tracker->new_sample,
               TILT_ACCEL_DPS2, TILT_MIN_DEG, TILT_MAX_DEG);
    tracker->new_sample = 0U;
    return 1U;
}
