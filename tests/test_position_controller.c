#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "position_controller.h"

static bool Near(float actual, float expected)
{
    return fabsf(actual - expected) < 0.001f;
}

static void TestShortestPathAndLimit(void)
{
    PositionController controller;
    PositionControllerResult result;
    const PositionControllerConfig config = {0.4f, 20.0f, 0.5f};

    assert(PositionController_Init(&controller, &config));

    assert(PositionController_Step(&controller, 1.0f, 359.0f / 360.0f, &result));
    assert(Near(result.error_deg, 2.0f));
    assert(Near(result.speed_target_rpm, 0.8f));
    assert(!result.within_tolerance);

    assert(PositionController_Step(&controller, 359.0f, 1.0f / 360.0f, &result));
    assert(Near(result.error_deg, -2.0f));
    assert(Near(result.speed_target_rpm, -0.8f));

    assert(PositionController_Step(&controller, 180.0f, 0.0f, &result));
    assert(Near(result.error_deg, 180.0f));
    assert(Near(result.speed_target_rpm, 20.0f));

    assert(PositionController_Step(&controller, 0.0f, 0.5f, &result));
    assert(Near(result.error_deg, 180.0f));
    assert(Near(result.speed_target_rpm, 20.0f));

    assert(PositionController_Step(&controller, 100.0f, 0.0f, &result));
    assert(Near(result.speed_target_rpm, 20.0f));

    assert(PositionController_Step(&controller, 0.25f, 0.0f, &result));
    assert(Near(result.error_deg, 0.25f));
    assert(Near(result.speed_target_rpm, 0.0f));
    assert(result.within_tolerance);
}

static void TestInvalidInputsDoNotChangeResult(void)
{
    PositionController controller;
    PositionControllerResult result = {123.0f, 456.0f, true};
    PositionControllerConfig config = {0.4f, 20.0f, 0.5f};

    assert(!PositionController_Init(0, &config));
    assert(!PositionController_Init(&controller, 0));
    config.kp_rpm_per_deg = 0.0f;
    assert(!PositionController_Init(&controller, &config));
    config.kp_rpm_per_deg = NAN;
    assert(!PositionController_Init(&controller, &config));
    config.kp_rpm_per_deg = INFINITY;
    assert(!PositionController_Init(&controller, &config));
    config.kp_rpm_per_deg = 0.4f;
    config.max_speed_rpm = 0.0f;
    assert(!PositionController_Init(&controller, &config));
    config.max_speed_rpm = 20.0f;
    config.tolerance_deg = NAN;
    assert(!PositionController_Init(&controller, &config));
    config.tolerance_deg = 0.5f;
    assert(PositionController_Init(&controller, &config));

    assert(!PositionController_Step(&controller, NAN, 0.0f, &result));
    assert(!PositionController_Step(&controller, INFINITY, 0.0f, &result));
    assert(!PositionController_Step(&controller, -1.0f, 0.0f, &result));
    assert(!PositionController_Step(&controller, 360.0f, 0.0f, &result));
    assert(!PositionController_Step(&controller, 0.0f, NAN, &result));
    assert(!PositionController_Step(&controller, 0.0f, INFINITY, &result));
    assert(!PositionController_Step(&controller, 0.0f, -0.01f, &result));
    assert(!PositionController_Step(&controller, 0.0f, 1.0f, &result));
    assert(!PositionController_Step(0, 0.0f, 0.0f, &result));
    assert(!PositionController_Step(&controller, 0.0f, 0.0f, 0));
    assert(Near(result.error_deg, 123.0f));
    assert(Near(result.speed_target_rpm, 456.0f));
    assert(result.within_tolerance);
}

int main(void)
{
    TestShortestPathAndLimit();
    TestInvalidInputsDoNotChangeResult();
    puts("position controller tests passed");
    return 0;
}
