/**
 * @file test_speed_estimator.c
 * @brief 电机轴编码器差分测速与低通滤波的主机侧单元测试。
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "speed_estimator.h"

#define TEST_COUNTS_PER_TURN (131072U)
#define TEST_PERIOD_S        (0.0005f)
#define TEST_EPSILON_RAD_S     (0.001f)

static void AssertNear(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static SpeedEstimatorConfig DefaultConfig(void)
{
    SpeedEstimatorConfig config;

    config.counts_per_turn = TEST_COUNTS_PER_TURN;
    config.direction = 1;
    config.filter_cutoff_hz = 20.0f;
    return config;
}

static void PrimeAt(SpeedEstimator *estimator,
                    uint32_t position_raw,
                    uint32_t sequence)
{
    assert(SpeedEstimator_Update(estimator,
                                 position_raw,
                                 sequence,
                                 TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_PRIMED);
}

static void Test_FirstSamplePrimesAndSecondSamplePublishesRadS(void)
{
    SpeedEstimator estimator;
    const SpeedEstimatorConfig config = DefaultConfig();
    const float one_count_per_tick_rad_s = 0.095873799f;

    assert(SpeedEstimator_Init(&estimator, &config));
    PrimeAt(&estimator, 1000U, 10U);
    assert(!estimator.ready);
    assert(SpeedEstimator_Update(&estimator, 1001U, 11U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    assert(estimator.ready);
    AssertNear(estimator.raw_rad_s, one_count_per_tick_rad_s, TEST_EPSILON_RAD_S);
    assert(estimator.filtered_rad_s > 0.0f);
    assert(estimator.filtered_rad_s < estimator.raw_rad_s);
}

static void Test_RepeatedSequenceDoesNotPublishFakeZeroSpeed(void)
{
    SpeedEstimator estimator;
    const SpeedEstimatorConfig config = DefaultConfig();
    float previous_raw_rad_s;

    assert(SpeedEstimator_Init(&estimator, &config));
    PrimeAt(&estimator, 1000U, 10U);
    assert(SpeedEstimator_Update(&estimator, 1001U, 11U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    previous_raw_rad_s = estimator.raw_rad_s;

    assert(SpeedEstimator_Update(&estimator, 1001U, 11U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_NO_NEW_SAMPLE);
    AssertNear(estimator.raw_rad_s, previous_raw_rad_s, 0.0001f);

    assert(SpeedEstimator_Update(&estimator, 1003U, 12U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    AssertNear(estimator.raw_rad_s, 0.095873799f, TEST_EPSILON_RAD_S);
}

static void Test_SeventeenBitWrapUsesShortestDelta(void)
{
    SpeedEstimator estimator;
    const SpeedEstimatorConfig config = DefaultConfig();

    assert(SpeedEstimator_Init(&estimator, &config));
    PrimeAt(&estimator, 131071U, 20U);
    assert(SpeedEstimator_Update(&estimator, 0U, 21U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    AssertNear(estimator.raw_rad_s, 0.095873799f, TEST_EPSILON_RAD_S);

    SpeedEstimator_Reset(&estimator);
    PrimeAt(&estimator, 0U, 30U);
    assert(SpeedEstimator_Update(&estimator, 131071U, 31U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    AssertNear(estimator.raw_rad_s, -0.095873799f, TEST_EPSILON_RAD_S);
}

static void Test_CalibratedDirectionControlsSpeedSign(void)
{
    SpeedEstimator estimator;
    SpeedEstimatorConfig config = DefaultConfig();

    config.direction = -1;
    assert(SpeedEstimator_Init(&estimator, &config));
    PrimeAt(&estimator, 100U, 1U);
    assert(SpeedEstimator_Update(&estimator, 110U, 2U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    assert(estimator.raw_rad_s < 0.0f);
}

static void Test_FilterConvergesWithoutOvershoot(void)
{
    SpeedEstimator estimator;
    const SpeedEstimatorConfig config = DefaultConfig();
    uint32_t position = 1000U;
    uint32_t sequence = 1U;
    float previous_filtered = 0.0f;
    unsigned int index;

    assert(SpeedEstimator_Init(&estimator, &config));
    PrimeAt(&estimator, position, sequence);
    for (index = 0U; index < 200U; ++index)
    {
        position += 100U;
        ++sequence;
        assert(SpeedEstimator_Update(&estimator,
                                     position,
                                     sequence,
                                     TEST_PERIOD_S) ==
               SPEED_ESTIMATOR_UPDATE_READY);
        assert(estimator.filtered_rad_s >= previous_filtered);
        assert(estimator.filtered_rad_s <= estimator.raw_rad_s);
        previous_filtered = estimator.filtered_rad_s;
    }
    AssertNear(estimator.filtered_rad_s, estimator.raw_rad_s, 0.01f);
}

static void Test_ResetPreservesConfigurationAndClearsDynamicState(void)
{
    SpeedEstimator estimator;
    const SpeedEstimatorConfig config = DefaultConfig();

    assert(SpeedEstimator_Init(&estimator, &config));
    PrimeAt(&estimator, 10U, 1U);
    assert(SpeedEstimator_Update(&estimator, 20U, 2U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_READY);
    SpeedEstimator_Reset(&estimator);

    assert(estimator.config.counts_per_turn == TEST_COUNTS_PER_TURN);
    assert(estimator.config.direction == 1);
    AssertNear(estimator.config.filter_cutoff_hz, 20.0f, 0.0001f);
    assert(!estimator.has_position);
    assert(!estimator.ready);
    AssertNear(estimator.raw_rad_s, 0.0f, 0.0001f);
    AssertNear(estimator.filtered_rad_s, 0.0f, 0.0001f);
}

static void Test_InvalidConfigurationAndSamplesAreRejected(void)
{
    SpeedEstimator estimator;
    SpeedEstimatorConfig config = DefaultConfig();

    assert(!SpeedEstimator_Init(NULL, &config));
    assert(!SpeedEstimator_Init(&estimator, NULL));

    config.counts_per_turn = 1U;
    assert(!SpeedEstimator_Init(&estimator, &config));
    config = DefaultConfig();
    config.direction = 0;
    assert(!SpeedEstimator_Init(&estimator, &config));
    config.direction = 2;
    assert(!SpeedEstimator_Init(&estimator, &config));
    config = DefaultConfig();
    config.filter_cutoff_hz = 0.0f;
    assert(!SpeedEstimator_Init(&estimator, &config));
    config.filter_cutoff_hz = NAN;
    assert(!SpeedEstimator_Init(&estimator, &config));

    config = DefaultConfig();
    assert(SpeedEstimator_Init(&estimator, &config));
    assert(SpeedEstimator_Update(NULL, 0U, 1U, TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_INVALID);
    assert(SpeedEstimator_Update(&estimator,
                                 TEST_COUNTS_PER_TURN,
                                 1U,
                                 TEST_PERIOD_S) ==
           SPEED_ESTIMATOR_UPDATE_INVALID);
    assert(SpeedEstimator_Update(&estimator, 0U, 1U, 0.0f) ==
           SPEED_ESTIMATOR_UPDATE_INVALID);
    assert(SpeedEstimator_Update(&estimator, 0U, 1U, NAN) ==
           SPEED_ESTIMATOR_UPDATE_INVALID);
}

int main(void)
{
    Test_FirstSamplePrimesAndSecondSamplePublishesRadS();
    Test_RepeatedSequenceDoesNotPublishFakeZeroSpeed();
    Test_SeventeenBitWrapUsesShortestDelta();
    Test_CalibratedDirectionControlsSpeedSign();
    Test_FilterConvergesWithoutOvershoot();
    Test_ResetPreservesConfigurationAndClearsDynamicState();
    Test_InvalidConfigurationAndSamplesAreRejected();

    puts("speed estimator tests passed");
    return 0;
}
