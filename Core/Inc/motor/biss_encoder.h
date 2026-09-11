/**
 * @file biss_encoder.h
 * @brief SPI4+DMA 的非阻塞 BiSS-C 编码器采集适配层。
 */
#ifndef BISS_ENCODER_H
#define BISS_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "biss_frame.h"

#define BISS_ENCODER_READY_FRAME_COUNT   (32U)
#define BISS_ENCODER_DMA_TIMEOUT_TICKS   (2U)

typedef struct
{
    uint32_t position_raw;
    uint32_t sequence;
    uint32_t valid_age_ticks;
    uint32_t valid_count;
    uint32_t crc_error_count;
    uint32_t frame_error_count;
    uint32_t spi_error_count;
    uint32_t timeout_count;
    uint32_t last_spi_error;
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    uint8_t received_crc;
    uint8_t calculated_crc;
    BissFrameStatus frame_status;
    bool warning;
    bool ready;
    bool busy;
} BissEncoderSnapshot;

HAL_StatusTypeDef BissEncoder_Init(SPI_HandleTypeDef *spi);
HAL_StatusTypeDef BissEncoder_StartRead(void);
void BissEncoder_ControlTick(void);
void BissEncoder_OnTransferComplete(void);
void BissEncoder_OnTransferError(uint32_t error_code);
void BissEncoder_OnAbortComplete(void);
bool BissEncoder_GetSnapshot(BissEncoderSnapshot *snapshot);

#endif /* BISS_ENCODER_H */
