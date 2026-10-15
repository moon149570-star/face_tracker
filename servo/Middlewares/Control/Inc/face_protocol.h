#ifndef FACE_PROTOCOL_H
#define FACE_PROTOCOL_H

#include <stdint.h>

#define FACE_FRAME_SIZE        16U
#define FACE_PROTOCOL_VERSION  1U
#define FACE_FLAG_VALID        1U
#define FACE_HEADER_1          0xA3U
#define FACE_HEADER_2          0xB3U

typedef struct {
    uint16_t x, y, width, height;
    uint8_t sequence;
    uint8_t valid;             /* 0=无人脸，1=有效人脸。 */
    uint32_t received_ms;      /* STM32收到帧尾字节的时间，不是采集时间。 */
} FaceFrame;

typedef struct {
    uint8_t data[FACE_FRAME_SIZE];
    uint8_t count;             /* 已缓存的字节数，函数返回时始终小于16。 */
    uint32_t last_byte_ms;
    uint32_t accepted, rejected;
} FaceParser;

void FaceParser_Init(FaceParser *parser);
uint16_t Face_CRC16(const uint8_t *data, uint16_t length);

/* 输入一个字节；仅返回1时写入result。版本、范围和CRC均在此验证。 */
uint8_t FaceParser_Push(FaceParser *parser, uint8_t byte, uint32_t now,
                        FaceFrame *result);

#endif
