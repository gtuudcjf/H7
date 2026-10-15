#include "lan9252_board.h"
#include "stm32h7xx_hal.h"
#include "spi.h"
#include "lan9252_regs.h"
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
volatile LAN9252SpiDebug g_lan9252_spi_debug;
volatile LAN9252HalFault g_lan9252_hal_fault;

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

static void CaptureHalFault(uint32_t reason, uint32_t hal_status,
    uint32_t elapsed_us, uint32_t timeout_us, uint32_t timeout_ms, uint32_t count)
{
    /* A driver-only deadline must not acquire unrelated HAL evidence later. */
    if (g_lan9252_hal_fault.valid || g_lan9252_first_fault.valid) return;
    g_lan9252_hal_fault.reason = reason;
    g_lan9252_hal_fault.hal_status = hal_status;
    g_lan9252_hal_fault.hal_error = hal_status == UINT32_C(0xffffffff) ?
        UINT32_C(0xffffffff) : hspi1.ErrorCode;
    g_lan9252_hal_fault.hal_state = (uint32_t)hspi1.State;
    g_lan9252_hal_fault.elapsed_us = elapsed_us;
    g_lan9252_hal_fault.timeout_us = timeout_us;
    g_lan9252_hal_fault.timeout_ms = timeout_ms;
    g_lan9252_hal_fault.chunk_length = count;
    g_lan9252_hal_fault.spi_sr = hspi1.Instance->SR;
    g_lan9252_hal_fault.spi_cr1 = hspi1.Instance->CR1;
    g_lan9252_hal_fault.spi_cr2 = hspi1.Instance->CR2;
    g_lan9252_hal_fault.spi_cfg1 = hspi1.Instance->CFG1;
    g_lan9252_hal_fault.spi_cfg2 = hspi1.Instance->CFG2;
    g_lan9252_hal_fault.tx_remaining = hspi1.TxXferCount;
    g_lan9252_hal_fault.rx_remaining = hspi1.RxXferCount;
    g_lan9252_hal_fault.valid = 1;
}

static LAN9252Status board_transfer(void *context, const uint8_t *tx,
    uint8_t *rx, size_t length, uint32_t timeout_us)
{
    uint8_t tx_chunk[32];
    uint8_t rx_chunk[32];
    uint32_t start = board_time_us(context);
    while (length != 0U) {
        uint32_t elapsed = board_time_us(context) - start;
        uint32_t remaining_us;
        uint32_t timeout_ms;
        uint16_t count;
        HAL_StatusTypeDef status;
        if (elapsed >= timeout_us) {
            CaptureHalFault(1, UINT32_C(0xffffffff), elapsed, timeout_us, 0, 0);
            return LAN9252_ERR_DIRECT_TIMEOUT;
        }
        /* HAL checks integer ticks with >=: 1 ms can expire a few us after
         * entry. Round up and add one tick so this guard cannot pre-empt
         * the DWT budget. Pre/post DWT checks still reject late completion.
         * Quotient/remainder avoids overflow in remaining_us + 999. */
        remaining_us = timeout_us - elapsed;
        timeout_ms = remaining_us / 1000U + 1U;
        if (remaining_us % 1000U != 0U) ++timeout_ms;
        count = (uint16_t)(length > sizeof(tx_chunk) ? sizeof(tx_chunk) : length);
        if (tx != NULL) memcpy(tx_chunk, tx, count);
        else memset(tx_chunk, 0, count);
        status = HAL_SPI_TransmitReceive(&hspi1, tx_chunk, rx_chunk, count, timeout_ms);
        if (status == HAL_TIMEOUT) {
            CaptureHalFault(2, (uint32_t)status, board_time_us(context) - start,
                            timeout_us, timeout_ms, count);
            return LAN9252_ERR_DIRECT_TIMEOUT;
        }
        /* In H7 HAL V1.12.1 blocking TxRx, FLAG is set only when the EOT
         * wait times out, then the API returns HAL_ERROR. Its tick timeout
         * can precede the DWT budget, so elapsed time alone cannot detect it.
         * Read the error code only for HAL_ERROR: HAL_BUSY can retain stale
         * error flags from an earlier call. */
        if (status == HAL_ERROR &&
            (HAL_SPI_GetError(&hspi1) & HAL_SPI_ERROR_FLAG) != 0U) {
            CaptureHalFault(3, (uint32_t)status, board_time_us(context) - start,
                            timeout_us, timeout_ms, count);
            return LAN9252_ERR_DIRECT_TIMEOUT;
        }
        elapsed = board_time_us(context) - start;
        if (elapsed >= timeout_us) {
            CaptureHalFault(4, (uint32_t)status, elapsed, timeout_us, timeout_ms, count);
            return LAN9252_ERR_DIRECT_TIMEOUT;
        }
        if (status != HAL_OK) {
            CaptureHalFault(5, (uint32_t)status, elapsed, timeout_us, timeout_ms, count);
            return LAN9252_ERR_IO;
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

/* Keep the lab comparison separate from board_transfer: the production
 * timeout/fault policy and its original probe result remain unchanged.
 * Each HAL call is bounded to 5 ms; no retries and no LAN9252 register writes. */
static void ReadByteTestDebug(LAN9252SpiReadDebug *sample, uint8_t fast,
                             uint8_t split)
{
    uint16_t header_size = fast ? 4U : 3U;
    uint16_t frame_size = header_size + 4U;
    HAL_StatusTypeDef result;
    memset(sample, 0, sizeof(*sample));
    memset(sample->rx, 0xcc, sizeof(sample->rx));
    sample->hal_status[0] = UINT32_C(0xffffffff);
    sample->hal_status[1] = UINT32_C(0xffffffff);
    sample->tx[0] = fast ? LAN9252_SPI_FAST_READ : 0x03U;
    sample->tx[1] = 0;
    sample->tx[2] = (uint8_t)LAN9252_REG_BYTE_TEST;

    board_select(NULL);
    result = HAL_SPI_TransmitReceive(&hspi1, sample->tx, sample->rx,
                                     split ? header_size : frame_size, 5U);
    sample->hal_calls = 1;
    sample->hal_status[0] = (uint32_t)result;
    sample->hal_error[0] = HAL_SPI_GetError(&hspi1);
    if (result == HAL_OK && split) {
        /* CS stays low across both calls; HAL closes/reopens SPI between them. */
        result = HAL_SPI_TransmitReceive(&hspi1, sample->tx + header_size,
                                         sample->rx + header_size, 4U, 5U);
        sample->hal_calls = 2;
        sample->hal_status[1] = (uint32_t)result;
        sample->hal_error[1] = HAL_SPI_GetError(&hspi1);
    }
    board_deselect(NULL); /* Also release CS after ERROR/BUSY/TIMEOUT. */
    if (result == HAL_OK) {
        const uint8_t *data = sample->rx + header_size;
        sample->value = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                      | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
        sample->value_valid = 1;
    }
}

void LAN9252_BoardRunSpiDiagnostics(void)
{
    LAN9252SpiReadDebug sample;
    g_lan9252_spi_debug.done = 0;
    g_lan9252_spi_debug.passed_mask = 0;
    board_deselect(NULL); /* Ensure an idle-high CS before the first frame. */
    ReadByteTestDebug(&sample, 1, 1);
    g_lan9252_spi_debug.fast_split = sample;
    if (sample.value_valid && sample.value == LAN9252_BYTE_TEST_VALUE)
        g_lan9252_spi_debug.passed_mask |= 1U;
    ReadByteTestDebug(&sample, 1, 0);
    g_lan9252_spi_debug.fast_single = sample;
    if (sample.value_valid && sample.value == LAN9252_BYTE_TEST_VALUE)
        g_lan9252_spi_debug.passed_mask |= 2U;
    ReadByteTestDebug(&sample, 0, 0);
    g_lan9252_spi_debug.normal_single = sample;
    if (sample.value_valid && sample.value == LAN9252_BYTE_TEST_VALUE)
        g_lan9252_spi_debug.passed_mask |= 4U;
    g_lan9252_spi_debug.done = 1; /* Watch only after all three snapshots exist. */
}
