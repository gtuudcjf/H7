/**
 * @file test_biss_dma_buffer.c
 * @brief BiSS DMA帧区与越界保护区的行为测试。
 */
#include <assert.h>
#include <stdio.h>

#include "biss_dma_buffer.h"

static void Test_WritingOnlyTheSixByteFrameKeepsGuardIntact(void)
{
    BissDmaBuffer buffer;
    uint32_t index;

    BissDmaBuffer_Init(&buffer, 0xFFU);
    for (index = 0U; index < BISS_DMA_FRAME_BYTES; ++index)
    {
        buffer.storage[index] = (uint8_t)index;
    }
    assert(BissDmaBuffer_GuardIntact(&buffer));
}

static void Test_WritingTheFirstBytePastTheFrameBreaksTheGuard(void)
{
    BissDmaBuffer buffer;

    BissDmaBuffer_Init(&buffer, 0xFFU);
    buffer.storage[BISS_DMA_FRAME_BYTES] = 0x00U;
    assert(!BissDmaBuffer_GuardIntact(&buffer));
}

static void Test_BufferReservesOneCompleteH7CacheLine(void)
{
    assert(sizeof(BissDmaBuffer) == BISS_DMA_STORAGE_BYTES);
    assert(BISS_DMA_STORAGE_BYTES == 32U);
}

int main(void)
{
    Test_WritingOnlyTheSixByteFrameKeepsGuardIntact();
    Test_WritingTheFirstBytePastTheFrameBreaksTheGuard();
    Test_BufferReservesOneCompleteH7CacheLine();

    puts("BiSS DMA buffer tests passed");
    return 0;
}
