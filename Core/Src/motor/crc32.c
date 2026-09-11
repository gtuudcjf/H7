/**
 * @file crc32.c
 * @brief IEEE 802.3 反射式 CRC-32，校准记录掉电完整性检查使用。
 */
#include "crc32.h"

uint32_t Crc32_Calculate(const void *data, size_t size)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;
    size_t byte_index;

    if ((data == 0) && (size != 0U))
    {
        return 0U;
    }

    for (byte_index = 0U; byte_index < size; ++byte_index)
    {
        uint8_t bit_index;

        crc ^= bytes[byte_index];
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            const uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}
