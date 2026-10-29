#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include "biss_encoder.h"

static uint32_t primask, aborts;
static uint8_t *receive_buffer;
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spi, uint8_t *tx,
                                             uint8_t *rx, uint16_t length)
{
    assert(spi->Instance == SPI4 && length == 6U && tx != NULL);
    receive_buffer = rx;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *spi)
{
    assert(spi->Instance == SPI4);
    ++aborts;
    return HAL_OK;
}
static void CompleteFrame(uint32_t position)
{
    uint32_t index;
    const uint8_t crc = BissFrame_CalculateCrc6((position << 2U) | 3U, 19U);
    const uint64_t bits = ((uint64_t)1U << 34U) | ((uint64_t)position << 16U) |
                          ((uint64_t)3U << 14U) | ((uint64_t)crc << 8U) | 0xFFU;
    for (index = 0U; index < 6U; ++index)
        receive_buffer[index] = (uint8_t)(bits >> ((5U - index) * 8U));
    BissEncoder_OnTransferComplete();
}
int main(void)
{
    SPI_HandleTypeDef spi = {SPI4, 0U};
    BissEncoderSnapshot snapshot;
    uint32_t index;
    assert(BissEncoder_Init(&spi) == HAL_OK);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.valid_age_ticks == UINT32_MAX);
    for (index = 0U; index < BISS_ENCODER_READY_FRAME_COUNT; ++index)
    {
        assert(BissEncoder_StartRead() == HAL_OK);
        assert(BissEncoder_StartRead() == HAL_BUSY);
        CompleteFrame(index);
    }
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.ready && snapshot.valid_age_ticks == 0U);
    assert(snapshot.position_raw == 31U && primask == 0U);
    for (index = 0U; index < 21U; ++index) BissEncoder_ControlTick();
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.valid_age_ticks == MOTOR_ENCODER_STALE_LIMIT_TICKS);
    BissEncoder_ControlTick();
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.valid_age_ticks > MOTOR_ENCODER_STALE_LIMIT_TICKS);
    assert(BissEncoder_StartRead() == HAL_OK);
    for (index = 0U; index < 5U; ++index) BissEncoder_ControlTick();
    assert(aborts == 0U);
    BissEncoder_ControlTick();
    assert(aborts == 1U);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(!snapshot.ready && snapshot.timeout_count == 1U);
    BissEncoder_ControlTick();
    assert(aborts == 1U);
    BissEncoder_OnAbortComplete();
    assert(BissEncoder_StartRead() == HAL_OK);
    CompleteFrame(100U);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(!snapshot.ready && snapshot.valid_age_ticks == 0U);
    assert(BissEncoder_Init(&spi) == HAL_OK);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.sequence == 0U && snapshot.valid_age_ticks == UINT32_MAX);
    puts("BiSS transport timeout/freshness tests passed");
    return 0;
}
