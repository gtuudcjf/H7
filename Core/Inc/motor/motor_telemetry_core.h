#ifndef MOTOR_TELEMETRY_CORE_H
#define MOTOR_TELEMETRY_CORE_H

#include <stdint.h>

#define MOTOR_TELEMETRY_CHANNEL_COUNT (8U)
#define MOTOR_TELEMETRY_FRAME_BYTES   (36U)
#define MOTOR_TELEMETRY_PERIOD_MS     (10U)

uint32_t MotorTelemetryCore_Encode(
    const float values[MOTOR_TELEMETRY_CHANNEL_COUNT],
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES]);

#endif /* MOTOR_TELEMETRY_CORE_H */
