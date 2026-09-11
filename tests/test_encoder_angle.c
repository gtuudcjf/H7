/**
 * @file test_encoder_angle.c
 * @brief 17 位机械位置到归一化机械角/电角度的主机侧单元测试。
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "encoder_angle.h"
#include "motor_params.h"

#define TEST_EPSILON (0.0001f)

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_EPSILON);
}

static void Test_ZeroAndHalfTurn(void)
{
    EncoderAngleConfig config;
    EncoderAngleSample sample;

    assert(EncoderAngle_Init(&config, 0U, 1, MOTOR_POLE_PAIRS));
    assert(EncoderAngle_Update(&config, 0U, &sample));
    AssertNear(sample.mechanical_angle_pu, 0.0f);
    AssertNear(sample.electrical_angle_pu, 0.0f);

    assert(EncoderAngle_Update(&config, 65536U, &sample));
    AssertNear(sample.mechanical_angle_pu, 0.5f);
    AssertNear(sample.electrical_angle_pu, 0.0f);
}

static void Test_SeventeenBitWrap(void)
{
    EncoderAngleConfig config;
    EncoderAngleSample sample;

    assert(EncoderAngle_Init(&config, 0U, 1, 1U));
    assert(EncoderAngle_Update(&config, 131071U, &sample));
    AssertNear(sample.mechanical_angle_pu, 131071.0f / 131072.0f);
    AssertNear(sample.electrical_angle_pu, 131071.0f / 131072.0f);

    assert(EncoderAngle_Update(&config, 0U, &sample));
    AssertNear(sample.mechanical_angle_pu, 0.0f);
}

static void Test_ZeroOffsetAndDirectionAreAppliedBeforePolePairs(void)
{
    EncoderAngleConfig config;
    EncoderAngleSample sample;

    assert(EncoderAngle_Init(&config, 4096U, -1, MOTOR_POLE_PAIRS));
    assert(EncoderAngle_Update(&config, 4096U, &sample));
    AssertNear(sample.mechanical_angle_pu, 0.0f);
    AssertNear(sample.electrical_angle_pu, 0.0f);

    assert(EncoderAngle_Update(&config, 8192U, &sample));
    AssertNear(sample.mechanical_angle_pu, 0.96875f);
    AssertNear(sample.electrical_angle_pu, 0.6875f);
}

static void Test_TenPolePairsWrapElectricalAngle(void)
{
    EncoderAngleConfig config;
    EncoderAngleSample sample;

    assert(EncoderAngle_Init(&config, 0U, 1, MOTOR_POLE_PAIRS));
    assert(EncoderAngle_Update(&config, 32768U, &sample));
    AssertNear(sample.mechanical_angle_pu, 0.25f);
    AssertNear(sample.electrical_angle_pu, 0.5f);
}

static void Test_InvalidArgumentsAreRejected(void)
{
    EncoderAngleConfig config;
    EncoderAngleSample sample;

    assert(!EncoderAngle_Init(NULL, 0U, 1, MOTOR_POLE_PAIRS));
    assert(!EncoderAngle_Init(&config, 131072U, 1, MOTOR_POLE_PAIRS));
    assert(!EncoderAngle_Init(&config, 0U, 0, MOTOR_POLE_PAIRS));
    assert(!EncoderAngle_Init(&config, 0U, 2, MOTOR_POLE_PAIRS));
    assert(!EncoderAngle_Init(&config, 0U, 1, 0U));

    assert(EncoderAngle_Init(&config, 0U, 1, MOTOR_POLE_PAIRS));
    assert(!EncoderAngle_Update(NULL, 0U, &sample));
    assert(!EncoderAngle_Update(&config, 0U, NULL));
    assert(!EncoderAngle_Update(&config, 131072U, &sample));

    config.direction = 0;
    assert(!EncoderAngle_Update(&config, 0U, &sample));
}

int main(void)
{
    Test_ZeroAndHalfTurn();
    Test_SeventeenBitWrap();
    Test_ZeroOffsetAndDirectionAreAppliedBeforePolePairs();
    Test_TenPolePairsWrapElectricalAngle();
    Test_InvalidArgumentsAreRejected();

    puts("encoder-angle tests passed");
    return 0;
}
