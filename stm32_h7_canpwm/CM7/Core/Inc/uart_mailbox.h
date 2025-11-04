#ifndef UART_MAILBOX_H
#define UART_MAILBOX_H

#include <string.h>
#include "stm32h7xx_hal.h"   // adjust if your HAL header path differs

// ================================
// Configuration (override before include if desired)
// ================================
#ifndef UART_MB_MAX
#define UART_MB_MAX        4      // max mailboxes (UARTs) you’ll register
#endif
#ifndef RX_RB_SZ
#define RX_RB_SZ           512    // ring buffer size per mailbox
#endif
#ifndef RX_CHUNK_SZ
#define RX_CHUNK_SZ        64     // ReceiveToIdle staging buffer
#endif

// ================================
/* Public types & API */
// ================================
typedef struct {
    UART_HandleTypeDef *huart;                 // bound UART
    uint8_t            rxChunk[RX_CHUNK_SZ];   // HAL staging buffer
    volatile uint8_t   rb[RX_RB_SZ];           // RX ring buffer
    volatile uint16_t  head;
    volatile uint16_t  tail;
    volatile uint32_t  dropped;                // overflow counter
} uart_mb_t;

// Global registry access (read-only for you; managed internally)
extern uart_mb_t  g_mailboxes[UART_MB_MAX];
extern uint32_t   g_mb_count;

// Register a UART and start ReceiveToIdle IT. Returns 0 on success.
int  uart_mb_register(UART_HandleTypeDef *huart, uart_mb_t **out_mb);

// Find a mailbox by huart pointer (or NULL if not found).
uart_mb_t* uart_mb_find(UART_HandleTypeDef *huart);

// Non-blocking: get one byte from mailbox ring; returns 1 if a byte was read.
static inline int uart_mb_get(uart_mb_t *mb, uint8_t *b) {
    if (mb->head == mb->tail) return 0;
    *b = mb->rb[mb->tail];
    mb->tail = (uint16_t)((mb->tail + 1U) % RX_RB_SZ);
    return 1;
}

// Optional: approximate count of buffered bytes
static inline uint16_t uart_mb_count(const uart_mb_t *mb) {
    int16_t diff = (int16_t)mb->head - (int16_t)mb->tail;
    return (uint16_t)(diff < 0 ? diff + RX_RB_SZ : diff);
}

// ================================
// Implementation (define UART_MAILBOX_IMPL in ONE .c before including)
// ================================
#ifdef UART_MAILBOX_IMPL

uart_mb_t g_mailboxes[UART_MB_MAX];
uint32_t  g_mb_count = 0;

static inline int _mb_put(uart_mb_t *mb, uint8_t b) {
    uint16_t next = (uint16_t)((mb->head + 1U) % RX_RB_SZ);
    if (next == mb->tail) { mb->dropped++; return 0; }  // full
    mb->rb[mb->head] = b;
    mb->head = next;
    return 1;
}

static uart_mb_t* _mb_find(UART_HandleTypeDef *huart) {
    for (uint32_t i = 0; i < g_mb_count; ++i)
        if (g_mailboxes[i].huart == huart) return &g_mailboxes[i];
    return NULL;
}

uart_mb_t* uart_mb_find(UART_HandleTypeDef *huart) { return _mb_find(huart); }

int uart_mb_register(UART_HandleTypeDef *huart, uart_mb_t **out_mb) {
    uart_mb_t *existing = _mb_find(huart);
    if (existing) {
        if (out_mb) *out_mb = existing;
        HAL_UARTEx_ReceiveToIdle_IT(existing->huart, existing->rxChunk, sizeof(existing->rxChunk));
        return 0;
    }
    if (g_mb_count >= UART_MB_MAX) return -1;

    uart_mb_t *mb = &g_mailboxes[g_mb_count++];
    memset((void*)mb, 0, sizeof(*mb));
    mb->huart = huart;

    HAL_UARTEx_ReceiveToIdle_IT(mb->huart, mb->rxChunk, sizeof(mb->rxChunk));
    if (out_mb) *out_mb = mb;
    return 0;
}

// ---- Shared HAL callbacks ----
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    uart_mb_t *mb = _mb_find(huart);
    if (!mb || Size == 0) return;

    for (uint16_t i = 0; i < Size; ++i) (void)_mb_put(mb, mb->rxChunk[i]);

    HAL_UARTEx_ReceiveToIdle_IT(mb->huart, mb->rxChunk, sizeof(mb->rxChunk));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    uart_mb_t *mb = _mb_find(huart);
    if (!mb) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UARTEx_ReceiveToIdle_IT(mb->huart, mb->rxChunk, sizeof(mb->rxChunk));
}

#endif // UART_MAILBOX_IMPL
#endif // UART_MAILBOX_H
