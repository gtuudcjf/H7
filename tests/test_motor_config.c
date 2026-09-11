/**
 * @file test_motor_config.c
 * @brief 编码器校准记录、CRC32 和参数合法性的主机侧单元测试。
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "crc32.h"
#include "motor_config_store.h"
#include "motor_params.h"

typedef char MotorConfigMustBeOneFlashWord[
    (sizeof(MotorCalibrationConfig) == 32U) ? 1 : -1];

static void Test_Crc32MatchesStandardCheckValue(void)
{
    static const uint8_t text[] = "123456789";

    assert(Crc32_Calculate(text, 9U) == 0xCBF43926UL);
    assert(Crc32_Calculate(NULL, 1U) == 0U);
}

static void Test_BuildsAndValidatesBoundaryZeroValues(void)
{
    MotorCalibrationConfig config;

    assert(MotorCalibrationConfig_Build(&config, 0U, 1, MOTOR_POLE_PAIRS));
    assert(config.magic == MOTOR_CONFIG_MAGIC);
    assert(config.version == MOTOR_CONFIG_VERSION);
    assert(config.size == sizeof(config));
    assert(config.calibrated == 1U);
    assert(MotorCalibrationConfig_IsValid(&config));

    assert(MotorCalibrationConfig_Build(
        &config, 131071U, -1, MOTOR_POLE_PAIRS));
    assert(MotorCalibrationConfig_IsValid(&config));
}

static void Test_RejectsInvalidBuildArguments(void)
{
    MotorCalibrationConfig config;

    assert(!MotorCalibrationConfig_Build(NULL, 0U, 1, MOTOR_POLE_PAIRS));
    assert(!MotorCalibrationConfig_Build(&config, 131072U, 1, MOTOR_POLE_PAIRS));
    assert(!MotorCalibrationConfig_Build(&config, 0U, 0, MOTOR_POLE_PAIRS));
    assert(!MotorCalibrationConfig_Build(&config, 0U, 2, MOTOR_POLE_PAIRS));
    assert(!MotorCalibrationConfig_Build(&config, 0U, 1, 0U));
}

static void Test_RejectsCorruptedRecordFieldsAndCrc(void)
{
    MotorCalibrationConfig valid;
    MotorCalibrationConfig damaged;

    assert(MotorCalibrationConfig_Build(&valid, 12345U, 1, MOTOR_POLE_PAIRS));

#define EXPECT_FIELD_REJECTED(statement)       \
    do                                          \
    {                                           \
        damaged = valid;                        \
        statement;                              \
        assert(!MotorCalibrationConfig_IsValid(&damaged)); \
    } while (0)

    EXPECT_FIELD_REJECTED(damaged.magic ^= 1U);
    EXPECT_FIELD_REJECTED(damaged.version += 1U);
    EXPECT_FIELD_REJECTED(damaged.size -= 1U);
    EXPECT_FIELD_REJECTED(damaged.electrical_zero_raw = 131072U);
    EXPECT_FIELD_REJECTED(damaged.encoder_direction = 0);
    EXPECT_FIELD_REJECTED(damaged.pole_pairs = 9U);
    EXPECT_FIELD_REJECTED(damaged.calibrated = 0U);
    EXPECT_FIELD_REJECTED(damaged.reserved[2] ^= 1U);
    EXPECT_FIELD_REJECTED(damaged.crc32 ^= 1U);

#undef EXPECT_FIELD_REJECTED

    assert(!MotorCalibrationConfig_IsValid(NULL));
}

int main(void)
{
    Test_Crc32MatchesStandardCheckValue();
    Test_BuildsAndValidatesBoundaryZeroValues();
    Test_RejectsInvalidBuildArguments();
    Test_RejectsCorruptedRecordFieldsAndCrc();

    puts("motor-config tests passed");
    return 0;
}
