#include "motor_telemetry_core.h"

#include <math.h>
#include <string.h>

typedef char MotorTelemetryFloat32Required[
    (sizeof(float) == 4U) && (sizeof(uint32_t) == 4U) ? 1 : -1];

uint32_t MotorTelemetryCore_Encode(
    const float values[MOTOR_TELEMETRY_CHANNEL_COUNT],
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES])
{
    uint32_t invalid_count = 0U;
    uint32_t channel;
    uint32_t byte;

    for (channel = 0U; channel < MOTOR_TELEMETRY_CHANNEL_COUNT; ++channel)
    {
        uint32_t bits = 0U;

        if (isfinite(values[channel]))
        {
            memcpy(&bits, &values[channel], sizeof(bits));
        }
        else
        {
            ++invalid_count;
        }
        for (byte = 0U; byte < 4U; ++byte)
        {
            frame[(4U * channel) + byte] = (uint8_t)(bits >> (8U * byte));
        }
    }

    frame[32] = 0x00U;
    frame[33] = 0x00U;
    frame[34] = 0x80U;
    frame[35] = 0x7FU;
    return invalid_count;
}
