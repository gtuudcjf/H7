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
#include "motor_params.h"

#define BISS_ENCODER_READY_FRAME_COUNT   (32U)
#define BISS_ENCODER_DMA_TIMEOUT_TICKS   MOTOR_ENCODER_DMA_TIMEOUT_TICKS

typedef struct
{
    /* 最近一帧通过同步、状态位和CRC校验的17位单圈位置。 */
    uint32_t position_raw;
    /* 每发布一帧有效位置加1；控制层用它判断是否得到新样本。 */
    uint32_t sequence;
    /* 距离最近有效帧经过的20 kHz控制周期数。 */
    uint32_t valid_age_ticks;
    /* 以下计数器只增不减，便于在Keil中判断通信长期稳定性。 */
    uint32_t valid_count;
    uint32_t crc_error_count;
    uint32_t frame_error_count;
    uint32_t spi_error_count;
    uint32_t timeout_count;
    uint32_t dma_guard_error_count;
    uint32_t last_spi_error;
    /* 原始48位采样窗，按SPI MSB first顺序保存，主要用于协议调试。 */
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    uint8_t received_crc;
    uint8_t calculated_crc;
    BissFrameStatus frame_status;
    /* warning来自编码器状态位；ready表示已连续收到足够多的有效帧。 */
    bool warning;
    bool ready;
    /* DMA 或帧解析/发布仍占有缓冲区；不表示编码器故障。 */
    bool busy;
} BissEncoderSnapshot;

/** 绑定SPI4并清空采集状态；本函数不会产生时钟。 */
HAL_StatusTypeDef BissEncoder_Init(SPI_HandleTypeDef *spi);

/**
 * 启动一次6字节全双工DMA传输。TX字节只用于让SPI主机产生48个时钟，
 * 真正的数据从SPI4_MISO进入RX缓冲区；传输或解析/发布进行中返回HAL_BUSY。
 */
HAL_StatusTypeDef BissEncoder_StartRead(void);

/** 每个20 kHz控制周期更新有效帧年龄，并处理DMA超时。 */
void BissEncoder_ControlTick(void);

/** SPI4完成回调：持有RX所有权解析帧，以短临界区发布快照后才允许复用。 */
void BissEncoder_OnTransferComplete(void);
void BissEncoder_OnTransferError(uint32_t error_code);
void BissEncoder_OnAbortComplete(void);

/**
 * 在短临界区内复制完整快照，避免FOC同时读到新旧两帧拼接的数据。
 * 返回true只代表成功取得快照，不代表snapshot->ready为true。
 */
bool BissEncoder_GetSnapshot(BissEncoderSnapshot *snapshot);

#endif /* BISS_ENCODER_H */
