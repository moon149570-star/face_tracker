#ifndef FACE_SERIAL_H
#define FACE_SERIAL_H

#include "face_protocol.h"

typedef struct {
    uint32_t good_frames;      /* 通过协议校验的帧数，包含无人脸帧。 */
    uint32_t bad_frames;       /* 完整候选帧校验失败次数，不是噪声字节数。 */
    uint32_t uart_errors;
    uint32_t ring_overflows;
} SerialStats;

extern volatile SerialStats g_serial_stats;

void Serial_Init(void);
void Serial_Poll(void);        /* 主循环调用：恢复错误状态并解析已缓存字节。 */

/* 只返回最近一帧，读取后清除就绪标志；调用方不需要关闭中断。 */
uint8_t Serial_GetLatest(FaceFrame *frame);

#endif
