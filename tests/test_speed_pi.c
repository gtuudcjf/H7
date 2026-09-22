/**
 * @file test_speed_pi.c
 * @brief 速度PI、安培限幅和抗积分饱和的主机侧单元测试。
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "speed_pi.h"

#define TEST_EPSILON (0.0002f)

static void AssertNear(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static SpeedPiConfig DefaultConfig(void)
{
    SpeedPiConfig config;

    config.kp_a_per_rpm = 0.005f;
    config.ki_a_per_rpm_s = 0.10f;
    config.kaw_per_s = 10.0f;
    config.output_limit_a = 0.6f;
    return config;
}

static void Test_ProportionalAndIntegralTermsUsePhysicalUnits(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    const SpeedPiConfig config = DefaultConfig();

    assert(SpeedPi_Init(&controller, &config));
    assert(SpeedPi_Step(&controller, 10.0f, 0.0f, 0.001f, &result));
    AssertNear(result.error_rpm, 10.0f, TEST_EPSILON);
    AssertNear(result.proportional_a, 0.05f, TEST_EPSILON);
    AssertNear(controller.integrator_a, 0.001f, TEST_EPSILON);
    AssertNear(result.integrator_a, 0.001f, TEST_EPSILON);
}

static void Test_PositiveAndNegativeLimitsAreSymmetric(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    const SpeedPiConfig config = DefaultConfig();

    assert(SpeedPi_Init(&controller, &config));
    assert(SpeedPi_Step(&controller, 1000.0f, 0.0f, 0.001f, &result));
    AssertNear(result.iq_command_a, 0.6f, TEST_EPSILON);
    assert(result.saturated);

    SpeedPi_Reset(&controller);
    assert(SpeedPi_Step(&controller, -1000.0f, 0.0f, 0.001f, &result));
    AssertNear(result.iq_command_a, -0.6f, TEST_EPSILON);
    assert(result.saturated);
}

static void Test_AntiWindupKeepsIntegratorFiniteAndBounded(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    const SpeedPiConfig config = DefaultConfig();
    unsigned int index;

    assert(SpeedPi_Init(&controller, &config));
    for (index = 0U; index < 10000U; ++index)
    {
        assert(SpeedPi_Step(&controller,
                            1000.0f,
                            0.0f,
                            0.001f,
                            &result));
    }
    assert(isfinite(controller.integrator_a));
    assert(fabsf(controller.integrator_a) <= config.output_limit_a);
}

static void Test_PreloadProducesBumplessFirstOutput(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    const SpeedPiConfig config = DefaultConfig();

    assert(SpeedPi_Init(&controller, &config));
    assert(SpeedPi_PreloadOutput(&controller, 20.0f, 10.0f, 0.30f));
    assert(SpeedPi_Step(&controller, 20.0f, 10.0f, 0.001f, &result));
    AssertNear(result.iq_command_a, 0.30f, 0.001f);
}

static void Test_ResetAndLimitUpdateKeepStateSafe(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    const SpeedPiConfig config = DefaultConfig();

    assert(SpeedPi_Init(&controller, &config));
    assert(SpeedPi_Step(&controller, 1000.0f, 0.0f, 0.001f, &result));
    assert(SpeedPi_SetOutputLimit(&controller, 0.2f));
    assert(fabsf(controller.integrator_a) <= 0.2f);

    SpeedPi_Reset(&controller);
    AssertNear(controller.integrator_a, 0.0f, TEST_EPSILON);
    AssertNear(controller.config.output_limit_a, 0.2f, TEST_EPSILON);
}

static void Test_GainsCanBeUpdatedWithoutResettingIntegrator(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    const SpeedPiConfig config = DefaultConfig();
    float previous_integrator;

    assert(SpeedPi_Init(&controller, &config));
    assert(SpeedPi_Step(&controller, 10.0f, 0.0f, 0.001f, &result));
    previous_integrator = controller.integrator_a;
    assert(SpeedPi_SetGains(&controller, 0.004f, 0.08f, 8.0f));
    AssertNear(controller.integrator_a, previous_integrator, TEST_EPSILON);
    AssertNear(controller.config.kp_a_per_rpm, 0.004f, TEST_EPSILON);
}

static void Test_InvalidArgumentsDoNotCorruptController(void)
{
    SpeedPiController controller;
    SpeedPiResult result;
    SpeedPiConfig config = DefaultConfig();
    SpeedPiController before;

    assert(!SpeedPi_Init(NULL, &config));
    assert(!SpeedPi_Init(&controller, NULL));
    config.kp_a_per_rpm = -1.0f;
    assert(!SpeedPi_Init(&controller, &config));
    config = DefaultConfig();
    config.ki_a_per_rpm_s = NAN;
    assert(!SpeedPi_Init(&controller, &config));
    config = DefaultConfig();
    config.output_limit_a = 0.0f;
    assert(!SpeedPi_Init(&controller, &config));

    config = DefaultConfig();
    assert(SpeedPi_Init(&controller, &config));
    before = controller;
    assert(!SpeedPi_Step(NULL, 0.0f, 0.0f, 0.001f, &result));
    assert(!SpeedPi_Step(&controller, NAN, 0.0f, 0.001f, &result));
    assert(!SpeedPi_Step(&controller, 0.0f, 0.0f, 0.0f, &result));
    assert(!SpeedPi_Step(&controller, 0.0f, 0.0f, 0.001f, NULL));
    assert(!SpeedPi_PreloadOutput(&controller, 0.0f, 0.0f, NAN));
    assert(!SpeedPi_SetGains(&controller, -1.0f, 0.0f, 0.0f));
    assert(!SpeedPi_SetOutputLimit(&controller, 0.0f));
    AssertNear(controller.integrator_a, before.integrator_a, TEST_EPSILON);
    AssertNear(controller.config.output_limit_a,
               before.config.output_limit_a,
               TEST_EPSILON);
}

int main(void)
{
    Test_ProportionalAndIntegralTermsUsePhysicalUnits();
    Test_PositiveAndNegativeLimitsAreSymmetric();
    Test_AntiWindupKeepsIntegratorFiniteAndBounded();
    Test_PreloadProducesBumplessFirstOutput();
    Test_ResetAndLimitUpdateKeepStateSafe();
    Test_GainsCanBeUpdatedWithoutResettingIntegrator();
    Test_InvalidArgumentsDoNotCorruptController();

    puts("speed PI tests passed");
    return 0;
}
