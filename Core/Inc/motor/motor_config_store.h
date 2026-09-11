/**
 * @file motor_config_store.h
 * @brief 编码器电角度零点记录及 H743 最后一个 Flash 扇区存储接口。
 */
#ifndef MOTOR_CONFIG_STORE_H
#define MOTOR_CONFIG_STORE_H

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_CONFIG_HOST_TEST
#include "stm32h7xx_hal.h"
#endif

#define MOTOR_CONFIG_MAGIC       (0x4D434647UL) /* "MCFG" */
#define MOTOR_CONFIG_VERSION     (1U)
#define MOTOR_CONFIG_FLASH_ADDR  (0x081E0000UL)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t electrical_zero_raw;
    int8_t encoder_direction;
    uint8_t pole_pairs;
    uint8_t calibrated;
    uint8_t reserved0;
    uint32_t crc32;
    uint32_t reserved[3];
} MotorCalibrationConfig;

bool MotorCalibrationConfig_Build(MotorCalibrationConfig *config,
                                  uint32_t electrical_zero_raw,
                                  int8_t encoder_direction,
                                  uint8_t pole_pairs);
bool MotorCalibrationConfig_IsValid(const MotorCalibrationConfig *config);

#ifndef MOTOR_CONFIG_HOST_TEST
HAL_StatusTypeDef MotorConfigStore_Load(MotorCalibrationConfig *config);
HAL_StatusTypeDef MotorConfigStore_Save(const MotorCalibrationConfig *config,
                                        bool safe_to_write);
#endif

#endif /* MOTOR_CONFIG_STORE_H */
