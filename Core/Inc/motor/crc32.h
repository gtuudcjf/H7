/**
 * @file crc32.h
 * @brief 与平台无关的 IEEE CRC-32 校验。
 */
#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t Crc32_Calculate(const void *data, size_t size);

#endif /* CRC32_H */
