#include "lan9252_board.h"
#include "stm32h7xx_hal.h"
#include "spi.h"
#include <string.h>

typedef struct {
    uint32_t last_cycles;
    uint32_t last_tick;
    uint32_t cycles_per_us;
    uint32_t fractional_cycles;
    uint32_t microseconds;
    uint8_t initialized;
} BoardClock;
static BoardClock clock_state;

static uint32_t board_time_us(void *context)
{
    BoardClock *clock = (BoardClock *)context;
    uint32_t tick = HAL_GetTick();
    uint32_t cycles = DWT->CYCCNT;
    uint64_t delta = (uint32_t)(cycles - clock->last_cycles);
    uint64_t coarse = (uint64_t)(uint32_t)(tick - clock->last_tick)
        * clock->cycles_per_us * 1000U;
    /* Tick quantization is <1 ms. Choose the nearest whole cycle wrap to
     * support gaps longer than the 8.95 s raw counter period at 480 MHz.
     * Sampling gaps must be < one HAL tick wrap (49.7 days). */
    if (coarse > delta) {
        delta += ((coarse - delta + UINT64_C(0x80000000)) >> 32) << 32;
    }
    clock->last_cycles = cycles;
    clock->last_tick = tick;
    delta += clock->fractional_cycles;
    clock->microseconds += (uint32_t)(delta / clock->cycles_per_us);
    clock->fractional_cycles = (uint32_t)(delta % clock->cycles_per_us);
    return clock->microseconds;
}

static void board_select(void *context)
{
    (void)context;
    HAL_GPIO_WritePin(LAN9252_CS_GPIO_Port, LAN9252_CS_Pin, GPIO_PIN_RESET);
}

static void board_deselect(void *context)
{
    (void)context;
    HAL_GPIO_WritePin(LAN9252_CS_GPIO_Port, LAN9252_CS_Pin, GPIO_PIN_SET);
}

static LAN9252Status board_transfer(void *context, const uint8_t *tx,
    uint8_t *rx, size_t length, uint32_t timeout_us)
{
    uint8_t tx_chunk[32];
    uint8_t rx_chunk[32];
    uint32_t start = board_time_us(context);
    while (length != 0U) {
        uint32_t elapsed = board_time_us(context) - start;
        uint32_t timeout_ms;
        uint16_t count;
        HAL_StatusTypeDef status;
        if (elapsed >= timeout_us) {
            return LAN9252_ERR_DIRECT_TIMEOUT;
        }
        /* HAL V1.12.1 checks elapsed milliseconds >= Timeout. Floor the
         * remaining budget; only the sub-ms case needs a 1 ms exception. */
        timeout_ms = (timeout_us - elapsed) / 1000U;
        if (timeout_ms == 0U) timeout_ms = 1U;
        count = (uint16_t)(length > sizeof(tx_chunk) ? sizeof(tx_chunk) : length);
        if (tx != NULL) memcpy(tx_chunk, tx, count);
        else memset(tx_chunk, 0, count);
        status = HAL_SPI_TransmitReceive(&hspi1, tx_chunk, rx_chunk, count, timeout_ms);
        if (status == HAL_TIMEOUT) return LAN9252_ERR_DIRECT_TIMEOUT;
        if (status != HAL_OK) return LAN9252_ERR_IO;
        if (board_time_us(context) - start >= timeout_us) {
            return LAN9252_ERR_DIRECT_TIMEOUT;
        }
        if (rx != NULL) { memcpy(rx, rx_chunk, count); rx += count; }
        if (tx != NULL) tx += count;
        length -= count;
    }
    return LAN9252_OK;
}

const LAN9252Transport *LAN9252_BoardTransport(void)
{
    static const LAN9252Transport transport = {
        board_transfer, board_select, board_deselect, board_time_us, &clock_state
    };
    if (!clock_state.initialized) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->LAR = UINT32_C(0xc5acce55); /* Unlock the Cortex-M7 DWT. */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        /* Never reset CYCCNT: other users of the core counter may exist. */
        clock_state.cycles_per_us = SystemCoreClock / 1000000U;
        clock_state.last_tick = HAL_GetTick();
        clock_state.last_cycles = DWT->CYCCNT;
        clock_state.initialized = 1U;
    }
    return &transport;
}
