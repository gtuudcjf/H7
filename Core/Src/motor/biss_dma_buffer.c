/**
 * @file biss_dma_buffer.c
 * @brief BiSS DMA缓冲区保护实现。
 */
#include "biss_dma_buffer.h"

void BissDmaBuffer_Init(BissDmaBuffer *buffer, uint8_t frame_fill)
{
    uint32_t index;

    if (buffer == 0)
    {
        return;
    }

    for (index = 0U; index < BISS_DMA_FRAME_BYTES; ++index)
    {
        buffer->storage[index] = frame_fill;
    }
    for (; index < BISS_DMA_STORAGE_BYTES; ++index)
    {
        buffer->storage[index] = BISS_DMA_GUARD_VALUE;
    }
}

bool BissDmaBuffer_GuardIntact(const BissDmaBuffer *buffer)
{
    uint32_t index;

    if (buffer == 0)
    {
        return false;
    }
    for (index = BISS_DMA_FRAME_BYTES; index < BISS_DMA_STORAGE_BYTES; ++index)
    {
        if (buffer->storage[index] != BISS_DMA_GUARD_VALUE)
        {
            return false;
        }
    }
    return true;
}
