/**
 * @file test_biss_sequence.c
 * @brief BiSS有效帧新鲜度序号回绕行为的主机侧单元测试。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "biss_sequence.h"

int main(void)
{
    assert(BissSequence_Next(0U) == 1U);
    assert(BissSequence_Next(1U) == 2U);
    assert(BissSequence_Next(UINT32_MAX - 1U) == UINT32_MAX);
    assert(BissSequence_Next(UINT32_MAX) == 1U);

    puts("BiSS sequence tests passed");
    return 0;
}
