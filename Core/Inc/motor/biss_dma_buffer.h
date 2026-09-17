/**
 * @file biss_dma_buffer.h
 * @brief STM32H7 DMA帧缓冲区及越界保护区。
 */
#ifndef BISS_DMA_BUFFER_H
#define BISS_DMA_BUFFER_H

#include <stdbool.h>
#include <stdint.h>

#define BISS_DMA_FRAME_BYTES     (6U)
#define BISS_DMA_STORAGE_BYTES   (32U)
#define BISS_DMA_GUARD_VALUE     (0xA5U)

/**
 * @brief 一条BiSS帧独占一个完整的H7数据缓存行。
 *
 * storage[0..5] 是DMA帧；storage[6..31] 是保护区。即使未来启用
 * D-Cache并按32字节执行维护，也不会覆盖相邻的电机控制状态。
 */
typedef struct
{
    uint8_t storage[BISS_DMA_STORAGE_BYTES];
} BissDmaBuffer;

void BissDmaBuffer_Init(BissDmaBuffer *buffer, uint8_t frame_fill);
bool BissDmaBuffer_GuardIntact(const BissDmaBuffer *buffer);

#endif /* BISS_DMA_BUFFER_H */
