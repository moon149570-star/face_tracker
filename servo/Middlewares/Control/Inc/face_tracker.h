#ifndef FACE_TRACKER_H
#define FACE_TRACKER_H

#include "face_protocol.h"
#include "pid.h"

/* HOMING:启动等待；HOLD:保持位置；ACTIVE:有效目标跟踪。 */
typedef enum { TRACK_HOMING = 0, TRACK_HOLD, TRACK_ACTIVE } TrackState;

typedef struct {
    PID_TypeDef pid;
    float angle;              /* 命令角度，度，不是实测轴角度。 */
    float speed;              /* 当前命令角速度，度/秒。 */
    float filtered_error;     /* 滤波后的归一化图像误差。 */
    uint8_t centered;
} TrackAxis;

typedef struct {
    TrackAxis pan, tilt;
    FaceFrame face;            /* 最近接受的测量，也可能是无人脸状态。 */
    uint32_t boot_ms, last_step_ms, previous_sample_ms;
    float sample_dt;           /* 两次接受测量的时间间隔，秒。 */
    uint8_t have_face;         /* 已接受过一帧，与face.valid含义不同。 */
    uint8_t new_sample;
    uint8_t filter_ready;
    TrackState state;
} FaceTracker;

/* 本模块不访问硬件；调用者保证输入已通过协议校验。所有时间戳以ms计。 */
void FaceTracker_Init(FaceTracker *tracker, uint32_t now);
void FaceTracker_OnFrame(FaceTracker *tracker, const FaceFrame *frame);

/* 返回1表示执行了控制周期，调用者据此更新PWM。不会补跑积压的旧周期。 */
uint8_t FaceTracker_Update(FaceTracker *tracker, uint32_t now);

#endif
