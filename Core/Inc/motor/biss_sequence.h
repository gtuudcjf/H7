/**
 * @file biss_sequence.h
 * @brief BiSS有效帧新鲜度序号的无零值循环递增。
 */
#ifndef BISS_SEQUENCE_H
#define BISS_SEQUENCE_H

#include <stdint.h>

/** 0保留为“尚无有效帧”；达到UINT32_MAX后回到1。 */
static inline uint32_t BissSequence_Next(uint32_t sequence)
{
    return (sequence == UINT32_MAX) ? 1U : (sequence + 1U);
}

#endif /* BISS_SEQUENCE_H */
