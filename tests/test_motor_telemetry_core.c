#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motor_telemetry_core.h"

static void TestJustFloatFrame(void)
{
    const float values[8] = {1.0f, -2.0f, 0.5f, 0.0f,
                             20.0f, 19.0f, 60.0f, 59.0f};
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES] = {0};
    const uint8_t one[] = {0x00U, 0x00U, 0x80U, 0x3FU};
    const uint8_t minus_two[] = {0x00U, 0x00U, 0x00U, 0xC0U};
    const uint8_t expected_channels[32] = {
        0x00U, 0x00U, 0x80U, 0x3FU, /* 1.0 */
        0x00U, 0x00U, 0x00U, 0xC0U, /* -2.0 */
        0x00U, 0x00U, 0x00U, 0x3FU, /* 0.5 */
        0x00U, 0x00U, 0x00U, 0x00U, /* 0.0 */
        0x00U, 0x00U, 0xA0U, 0x41U, /* 20.0 */
        0x00U, 0x00U, 0x98U, 0x41U, /* 19.0 */
        0x00U, 0x00U, 0x70U, 0x42U, /* 60.0 */
        0x00U, 0x00U, 0x6CU, 0x42U  /* 59.0 */
    };
    const uint8_t tail[] = {0x00U, 0x00U, 0x80U, 0x7FU};

    assert(MotorTelemetryCore_Encode(values, frame) == 0U);
    assert(memcmp(frame, one, sizeof(one)) == 0);
    assert(memcmp(frame + 4U, minus_two, sizeof(minus_two)) == 0);
    assert(memcmp(frame, expected_channels, sizeof(expected_channels)) == 0);
    assert(memcmp(frame + 32U, tail, sizeof(tail)) == 0);
}

static void TestNonFiniteValuesBecomeZero(void)
{
    const float values[8] = {NAN, INFINITY, -INFINITY, 4.0f,
                             5.0f, 6.0f, 7.0f, 8.0f};
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES];
    uint32_t index;

    assert(MotorTelemetryCore_Encode(values, frame) == 3U);
    for (index = 0U; index < 12U; ++index)
    {
        assert(frame[index] == 0U);
    }
}

int main(void)
{
    TestJustFloatFrame();
    TestNonFiniteValuesBecomeZero();
    puts("motor telemetry frame tests passed");
    return 0;
}
