/**
 * @file biss_encoder.c
 * @brief 使用 SPI4 DMA 产生 48 个 BiSS-C 时钟并发布校验后的角度快照。
 *
 * 本模块不执行电机控制。DMA 回调只解析固定六字节帧，控制中断通过
 * BissEncoder_GetSnapshot() 获取一次不可变副本，避免一轮 FOC 使用两个角度。
 */
#include "biss_encoder.h"

#include <limits.h>
#include <string.h>

static SPI_HandleTypeDef *encoder_spi;

/*
 * 当前工程未启用 D-Cache；32 字节对齐同时为以后启用 Cache 后执行显式
 * clean/invalidate 留出正确的缓存行边界。两个缓冲区必须位于 DMA 可访问 SRAM。
 */
static uint8_t encoder_tx[BISS_FRAME_RAW_BYTES] __attribute__((aligned(32)));
static uint8_t encoder_rx[BISS_FRAME_RAW_BYTES] __attribute__((aligned(32)));

static volatile BissEncoderSnapshot encoder_snapshot;
static volatile uint32_t consecutive_valid_count;
static volatile uint32_t dma_busy_ticks;
static volatile bool dma_busy;
static volatile bool abort_requested;

static uint32_t BissEncoder_EnterCritical(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void BissEncoder_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static uint32_t BissEncoder_IncrementSaturated(uint32_t value)
{
    return (value < UINT32_MAX) ? (value + 1U) : UINT32_MAX;
}

HAL_StatusTypeDef BissEncoder_Init(SPI_HandleTypeDef *spi)
{
    uint32_t primask;

    if ((spi == 0) || (spi->Instance != SPI4))
    {
        return HAL_ERROR;
    }

    primask = BissEncoder_EnterCritical();
    encoder_spi = spi;
    memset(encoder_tx, 0, sizeof(encoder_tx));
    memset(encoder_rx, 0xFF, sizeof(encoder_rx));
    memset((void *)&encoder_snapshot, 0, sizeof(encoder_snapshot));
    encoder_snapshot.valid_age_ticks = UINT32_MAX;
    encoder_snapshot.frame_status = BISS_FRAME_SYNC_ERROR;
    consecutive_valid_count = 0U;
    dma_busy_ticks = 0U;
    dma_busy = false;
    abort_requested = false;
    BissEncoder_ExitCritical(primask);
    return HAL_OK;
}

HAL_StatusTypeDef BissEncoder_StartRead(void)
{
    HAL_StatusTypeDef status;

    if (encoder_spi == 0)
    {
        return HAL_ERROR;
    }
    if (dma_busy)
    {
        return HAL_BUSY;
    }

    dma_busy = true;
    dma_busy_ticks = 0U;
    abort_requested = false;
    encoder_snapshot.busy = true;
    status = HAL_SPI_TransmitReceive_DMA(
        encoder_spi,
        encoder_tx,
        encoder_rx,
        BISS_FRAME_RAW_BYTES);
    if (status != HAL_OK)
    {
        dma_busy = false;
        encoder_snapshot.busy = false;
        if (status != HAL_BUSY)
        {
            encoder_snapshot.spi_error_count = BissEncoder_IncrementSaturated(
                encoder_snapshot.spi_error_count);
            encoder_snapshot.last_spi_error = encoder_spi->ErrorCode;
            consecutive_valid_count = 0U;
            encoder_snapshot.ready = false;
        }
    }
    return status;
}

void BissEncoder_ControlTick(void)
{
    if (encoder_snapshot.sequence != 0U)
    {
        encoder_snapshot.valid_age_ticks = BissEncoder_IncrementSaturated(
            encoder_snapshot.valid_age_ticks);
    }

    if (!dma_busy)
    {
        return;
    }

    dma_busy_ticks = BissEncoder_IncrementSaturated(dma_busy_ticks);
    if ((dma_busy_ticks > BISS_ENCODER_DMA_TIMEOUT_TICKS) && !abort_requested)
    {
        abort_requested = true;
        encoder_snapshot.timeout_count = BissEncoder_IncrementSaturated(
            encoder_snapshot.timeout_count);
        consecutive_valid_count = 0U;
        encoder_snapshot.ready = false;

        /* HAL_SPI_Abort_IT 只发起中止，不在 10 kHz 中断内等待外设。 */
        if (HAL_SPI_Abort_IT(encoder_spi) != HAL_OK)
        {
            dma_busy = false;
            encoder_snapshot.busy = false;
            encoder_snapshot.spi_error_count = BissEncoder_IncrementSaturated(
                encoder_snapshot.spi_error_count);
            encoder_snapshot.last_spi_error = encoder_spi->ErrorCode;
        }
    }
}

void BissEncoder_OnTransferComplete(void)
{
    BissFrame17 frame;

    dma_busy = false;
    dma_busy_ticks = 0U;
    abort_requested = false;
    encoder_snapshot.busy = false;

    if (!BissFrame_Parse17(encoder_rx, &frame))
    {
        memcpy((void *)encoder_snapshot.raw, frame.raw, BISS_FRAME_RAW_BYTES);
        encoder_snapshot.received_crc = frame.received_crc;
        encoder_snapshot.calculated_crc = frame.calculated_crc;
        encoder_snapshot.frame_status = frame.status;
        consecutive_valid_count = 0U;
        encoder_snapshot.ready = false;
        if (frame.status == BISS_FRAME_CRC_ERROR)
        {
            encoder_snapshot.crc_error_count = BissEncoder_IncrementSaturated(
                encoder_snapshot.crc_error_count);
        }
        else
        {
            encoder_snapshot.frame_error_count = BissEncoder_IncrementSaturated(
                encoder_snapshot.frame_error_count);
        }
        return;
    }

    memcpy((void *)encoder_snapshot.raw, frame.raw, BISS_FRAME_RAW_BYTES);
    encoder_snapshot.position_raw = frame.position_raw;
    encoder_snapshot.received_crc = frame.received_crc;
    encoder_snapshot.calculated_crc = frame.calculated_crc;
    encoder_snapshot.frame_status = BISS_FRAME_OK;
    encoder_snapshot.warning = !frame.warning_ok;
    encoder_snapshot.sequence = BissEncoder_IncrementSaturated(encoder_snapshot.sequence);
    encoder_snapshot.valid_count = BissEncoder_IncrementSaturated(
        encoder_snapshot.valid_count);
    encoder_snapshot.valid_age_ticks = 0U;
    consecutive_valid_count = BissEncoder_IncrementSaturated(consecutive_valid_count);
    encoder_snapshot.ready = consecutive_valid_count >= BISS_ENCODER_READY_FRAME_COUNT;
}

void BissEncoder_OnTransferError(uint32_t error_code)
{
    dma_busy = false;
    dma_busy_ticks = 0U;
    abort_requested = false;
    encoder_snapshot.busy = false;
    encoder_snapshot.spi_error_count = BissEncoder_IncrementSaturated(
        encoder_snapshot.spi_error_count);
    encoder_snapshot.last_spi_error = error_code;
    consecutive_valid_count = 0U;
    encoder_snapshot.ready = false;
}

void BissEncoder_OnAbortComplete(void)
{
    dma_busy = false;
    dma_busy_ticks = 0U;
    abort_requested = false;
    encoder_snapshot.busy = false;
}

bool BissEncoder_GetSnapshot(BissEncoderSnapshot *snapshot)
{
    uint32_t primask;

    if ((snapshot == 0) || (encoder_spi == 0))
    {
        return false;
    }

    primask = BissEncoder_EnterCritical();
    memcpy(snapshot, (const void *)&encoder_snapshot, sizeof(*snapshot));
    BissEncoder_ExitCritical(primask);
    return true;
}
