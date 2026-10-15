#include "serial.h"
#include "usart.h"
#include "tracking_config.h"
#include <string.h>

#if SERIAL_RX_RING_SIZE < 2U || SERIAL_RX_RING_SIZE > 65536U || \
    (SERIAL_RX_RING_SIZE & (SERIAL_RX_RING_SIZE - 1U)) != 0
#error SERIAL_RX_RING_SIZE must be a power of two in [2, 65536]
#endif

#define RX_INDEX_MASK (SERIAL_RX_RING_SIZE - 1U)

volatile SerialStats g_serial_stats;
static uint8_t rx_byte;

/* 单生产者/单消费者：中断只写head，主循环只写tail，空出一格区分满和空。 */
static volatile uint8_t ring[SERIAL_RX_RING_SIZE];
static volatile uint32_t ring_time[SERIAL_RX_RING_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t discard, retry_rx;

/* 解析器、最新帧及ready只在主循环访问，不需要中断锁。 */
static FaceParser parser;
static FaceFrame latest;
static uint8_t ready;

static void ArmReceive(void)
{
    retry_rx = HAL_UART_Receive_IT(&huart2, &rx_byte, 1U) != HAL_OK;
}

void Serial_Init(void)
{
    head = 0U;
    tail = 0U;
    discard = 0U;
    ready = 0U;
    memset((void *)&g_serial_stats, 0, sizeof(g_serial_stats));
    FaceParser_Init(&parser);
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    ArmReceive();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART2) {
        return;
    }
    next = (uint16_t)((head + 1U) & RX_INDEX_MASK);
    if (!discard) {
        if (next == tail) {
            ++g_serial_stats.ring_overflows;
            discard = 1U; /* 溢出后暂停入队，由主循环丢弃不完整数据流。 */
        } else {
            ring[head] = rx_byte;
            ring_time[head] = HAL_GetTick();
            __DMB(); /* 先发布字节和时间，再发布新的head。 */
            head = next;
        }
    }
    ArmReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART2) {
        return;
    }
    ++g_serial_stats.uart_errors;
    discard = 1U;
    HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    ArmReceive();
}

void Serial_Poll(void)
{
    uint32_t saved_irq_mask;

    if (discard) {
        /* 只在清队列时短暂关中断，并恢复进入前的中断状态。 */
        saved_irq_mask = __get_PRIMASK();
        __disable_irq();
        tail = head;
        discard = 0U;
        __set_PRIMASK(saved_irq_mask);
        parser.count = 0U;
        ready = 0U;
    }
    if (retry_rx) {
        saved_irq_mask = __get_PRIMASK();
        __disable_irq();
        __HAL_UART_CLEAR_OREFLAG(&huart2);
        ArmReceive();
        __set_PRIMASK(saved_irq_mask);
    }

    while (tail != head && !discard) {
        uint8_t byte = ring[tail];
        uint32_t received_ms = ring_time[tail];

        __DMB();
        tail = (uint16_t)((tail + 1U) & RX_INDEX_MASK);
        /* 解析器仅在成功时写latest，省去原来的临时帧及整帧复制。 */
        if (FaceParser_Push(&parser, byte, received_ms, &latest)) {
            ready = 1U;
        }
    }
    if (discard) {
        ready = 0U; /* 解析期间发生接收错误时，不发布该批数据。 */
    }
    g_serial_stats.good_frames = parser.accepted;
    g_serial_stats.bad_frames = parser.rejected;
}

uint8_t Serial_GetLatest(FaceFrame *frame)
{
    if (!ready) {
        return 0U;
    }
    *frame = latest;
    ready = 0U;
    return 1U;
}
