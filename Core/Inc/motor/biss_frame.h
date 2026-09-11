/**
 * @file biss_frame.h
 * @brief 与 STM32 HAL 无关的 17 位 BiSS-C 单圈位置帧解析器。
 *
 * SPI/DMA 只负责提供 48 个按 MSB 优先排列的采样位；本模块负责寻找
 * 13 位 ACK、检查状态位和 CRC，并且只在整帧有效时发布位置值。
 */
#ifndef BISS_FRAME_H
#define BISS_FRAME_H

#include <stdbool.h>
#include <stdint.h>

#define BISS_FRAME_RAW_BYTES        (6U)
#define BISS_FRAME_RAW_BITS         (48U)
#define BISS_POSITION_BITS          (17U)
#define BISS_POSITION_MAX           (131071UL)
#define BISS_ACK_BITS               (13U)
#define BISS_CRC_BITS               (6U)

typedef enum
{
    BISS_FRAME_OK = 0,
    BISS_FRAME_BAD_ARGUMENT,
    BISS_FRAME_SYNC_ERROR,
    BISS_FRAME_ENCODER_ERROR,
    BISS_FRAME_CRC_ERROR
} BissFrameStatus;

typedef struct
{
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    uint32_t position_raw;
    uint8_t received_crc;
    uint8_t calculated_crc;
    uint8_t payload_bit_index;
    bool error_ok;
    bool warning_ok;
    BissFrameStatus status;
} BissFrame17;

/**
 * @brief 计算 BiSS-C 发送端使用的六位反相 CRC。
 * @param payload 按 MSB 优先、低位右对齐的数据。
 * @param bit_count 参与计算的位数，范围为 1～32。
 * @return 反相后的六位 CRC；参数无效时返回 0。
 */
uint8_t BissFrame_CalculateCrc6(uint32_t payload, uint8_t bit_count);

/**
 * @brief 从一次 48 时钟采样中解析 17 位编码器位置。
 * @return 仅当同步、Error 状态和 CRC 全部有效时返回 true。
 * @note Warning 有效地反映到 frame 中，但不会丢弃位置样本。
 */
bool BissFrame_Parse17(const uint8_t raw[BISS_FRAME_RAW_BYTES], BissFrame17 *frame);

#endif /* BISS_FRAME_H */
