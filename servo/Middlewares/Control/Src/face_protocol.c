#include "face_protocol.h"
#include "tracking_config.h"
#include <string.h>

/* 帧头占2字节，CRC覆盖version至height，共12字节；CRC自身位于末尾。 */
enum {
    PAYLOAD_OFFSET = 2,
    PAYLOAD_LENGTH = 12,
    CRC_OFFSET = 14,
    MIN_IMAGE_SIZE = 16,
    MAX_IMAGE_SIZE = 4096
};

static uint16_t ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

uint16_t Face_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;

    /* CRC-16/CCITT-FALSE：poly=0x1021，不反射，结果无异或。 */
    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            crc = (uint16_t)((crc & 0x8000U) != 0U ?
                            ((uint32_t)crc << 1) ^ 0x1021U : (uint32_t)crc << 1);
        }
    }
    return crc;
}

void FaceParser_Init(FaceParser *parser)
{
    memset(parser, 0, sizeof(*parser));
}

static uint8_t DecodeFrame(const uint8_t *data, uint32_t now, FaceFrame *result)
{
    uint16_t x = ReadU16(data + 6);
    uint16_t y = ReadU16(data + 8);
    uint16_t width = ReadU16(data + 10);
    uint16_t height = ReadU16(data + 12);
    uint8_t valid = data[4];

    if (data[2] != FACE_PROTOCOL_VERSION || valid > FACE_FLAG_VALID || data[5] != 0U) {
        return 0U;
    }
    if (width < MIN_IMAGE_SIZE || width > MAX_IMAGE_SIZE ||
        height < MIN_IMAGE_SIZE || height > MAX_IMAGE_SIZE) {
        return 0U;
    }
    if (valid ? (x >= width || y >= height) : (x != 0U || y != 0U)) {
        return 0U;
    }
    if (Face_CRC16(data + PAYLOAD_OFFSET, PAYLOAD_LENGTH) != ReadU16(data + CRC_OFFSET)) {
        return 0U;
    }

    /* 校验全部通过后才写结果，错误帧不会覆盖调用者保存的上一帧。 */
    result->x = x;
    result->y = y;
    result->width = width;
    result->height = height;
    result->sequence = data[3];
    result->valid = valid;
    result->received_ms = now;
    return 1U;
}

uint8_t FaceParser_Push(FaceParser *parser, uint8_t byte, uint32_t now,
                        FaceFrame *result)
{
    uint8_t *data = parser->data;
    uint8_t offset = 0U;

    if (parser->count != 0U &&
        (uint32_t)(now - parser->last_byte_ms) > SERIAL_PARTIAL_MS) {
        parser->count = 0U;
    }
    parser->last_byte_ms = now;
    data[parser->count++] = byte;

    /* 找到下一个可能的帧头后只搬运一次，保留最后一个待配对字节。 */
    while (parser->count - offset >= 2 &&
           (data[offset] != FACE_HEADER_1 || data[offset + 1U] != FACE_HEADER_2)) {
        ++offset;
    }
    if (offset != 0U) {
        parser->count -= offset;
        memmove(data, data + offset, parser->count);
    }
    if (parser->count < FACE_FRAME_SIZE) {
        return 0U;
    }

    if (DecodeFrame(data, now, result)) {
        parser->count = 0U;
        ++parser->accepted;
        return 1U;
    }

    /* 坏帧只丢首字节，保留其中可能出现的新帧头，下一次输入继续同步。 */
    ++parser->rejected;
    --parser->count;
    memmove(data, data + 1, parser->count);
    return 0U;
}
