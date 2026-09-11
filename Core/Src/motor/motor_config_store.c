/**
 * @file motor_config_store.c
 * @brief 校准记录校验，以及受安全条件保护的 H743 Bank2 Sector7 写入。
 */
#include "motor_config_store.h"

#include <stddef.h>
#include <string.h>

#include "biss_frame.h"
#include "crc32.h"
#include "motor_params.h"

typedef char MotorConfigRecordMustBe32Bytes[
    (sizeof(MotorCalibrationConfig) == 32U) ? 1 : -1];

static uint32_t MotorCalibrationConfig_CalculateCrc(
    const MotorCalibrationConfig *config)
{
    MotorCalibrationConfig copy;

    if (config == 0)
    {
        return 0U;
    }

    copy = *config;
    copy.crc32 = 0U;
    return Crc32_Calculate(&copy, sizeof(copy));
}

bool MotorCalibrationConfig_Build(MotorCalibrationConfig *config,
                                  uint32_t electrical_zero_raw,
                                  int8_t encoder_direction,
                                  uint8_t pole_pairs)
{
    if ((config == 0) ||
        (electrical_zero_raw > BISS_POSITION_MAX) ||
        ((encoder_direction != 1) && (encoder_direction != -1)) ||
        (pole_pairs != MOTOR_POLE_PAIRS))
    {
        return false;
    }

    memset(config, 0, sizeof(*config));
    config->magic = MOTOR_CONFIG_MAGIC;
    config->version = MOTOR_CONFIG_VERSION;
    config->size = (uint16_t)sizeof(*config);
    config->electrical_zero_raw = electrical_zero_raw;
    config->encoder_direction = encoder_direction;
    config->pole_pairs = pole_pairs;
    config->calibrated = 1U;
    config->crc32 = MotorCalibrationConfig_CalculateCrc(config);
    return true;
}

bool MotorCalibrationConfig_IsValid(const MotorCalibrationConfig *config)
{
    if ((config == 0) ||
        (config->magic != MOTOR_CONFIG_MAGIC) ||
        (config->version != MOTOR_CONFIG_VERSION) ||
        (config->size != sizeof(*config)) ||
        (config->electrical_zero_raw > BISS_POSITION_MAX) ||
        ((config->encoder_direction != 1) && (config->encoder_direction != -1)) ||
        (config->pole_pairs != MOTOR_POLE_PAIRS) ||
        (config->calibrated != 1U))
    {
        return false;
    }

    return config->crc32 == MotorCalibrationConfig_CalculateCrc(config);
}

#ifndef MOTOR_CONFIG_HOST_TEST

HAL_StatusTypeDef MotorConfigStore_Load(MotorCalibrationConfig *config)
{
    if (config == 0)
    {
        return HAL_ERROR;
    }

    memcpy(config, (const void *)MOTOR_CONFIG_FLASH_ADDR, sizeof(*config));
    return MotorCalibrationConfig_IsValid(config) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MotorConfigStore_Save(const MotorCalibrationConfig *config,
                                        bool safe_to_write)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    HAL_StatusTypeDef status;
    MotorCalibrationConfig readback;
    static uint32_t flash_word[8] __attribute__((aligned(32)));

    if (!safe_to_write)
    {
        return HAL_BUSY;
    }
    if (!MotorCalibrationConfig_IsValid(config))
    {
        return HAL_ERROR;
    }

    memcpy(flash_word, config, sizeof(flash_word));
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK)
    {
        return status;
    }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_2;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    status = HAL_FLASHEx_Erase(&erase, &sector_error);
    if (status == HAL_OK)
    {
        status = HAL_FLASH_Program(
            FLASH_TYPEPROGRAM_FLASHWORD,
            MOTOR_CONFIG_FLASH_ADDR,
            (uint32_t)flash_word);
    }

    (void)HAL_FLASH_Lock();
    if (status != HAL_OK)
    {
        return status;
    }

    status = MotorConfigStore_Load(&readback);
    if ((status != HAL_OK) ||
        (memcmp(&readback, config, sizeof(readback)) != 0))
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

#endif /* MOTOR_CONFIG_HOST_TEST */
