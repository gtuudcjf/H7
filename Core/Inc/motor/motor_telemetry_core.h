#ifndef MOTOR_TELEMETRY_CORE_H
#define MOTOR_TELEMETRY_CORE_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_TELEMETRY_CHANNEL_COUNT (8U)
#define MOTOR_TELEMETRY_FRAME_BYTES   (36U)
#define MOTOR_TELEMETRY_PERIOD_MS     (10U)

typedef enum
{
    MOTOR_TELEMETRY_UART = 0,
    MOTOR_TELEMETRY_USB = 1,
    MOTOR_TELEMETRY_PORT_COUNT = 2
} MotorTelemetryPortId;

typedef enum
{
    MOTOR_TELEMETRY_SEND_ACCEPTED = 0,
    MOTOR_TELEMETRY_SEND_SKIP,
    MOTOR_TELEMETRY_SEND_ERROR
} MotorTelemetrySendResult;

typedef MotorTelemetrySendResult (*MotorTelemetrySendFn)(
    void *context, uint8_t *frame, uint16_t length);

typedef struct
{
    MotorTelemetrySendFn send;
    void *context;
} MotorTelemetrySink;

typedef struct
{
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES];
    bool enabled;
    volatile bool busy;
    volatile uint32_t completed;
    volatile uint32_t dropped;
    volatile uint32_t errors;
} MotorTelemetryPort;

typedef struct
{
    MotorTelemetryPort port[MOTOR_TELEMETRY_PORT_COUNT];
    uint32_t last_ms;
    uint32_t invalid_values;
} MotorTelemetryCore;

uint32_t MotorTelemetryCore_Encode(
    const float values[MOTOR_TELEMETRY_CHANNEL_COUNT],
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES]);

void MotorTelemetryCore_Init(MotorTelemetryCore *core, uint32_t now_ms,
                             bool uart_enabled, bool usb_enabled);
void MotorTelemetryCore_SetEnabled(MotorTelemetryCore *core,
                                    MotorTelemetryPortId id, bool enabled);
void MotorTelemetryCore_Service(
    MotorTelemetryCore *core, uint32_t now_ms,
    const float values[MOTOR_TELEMETRY_CHANNEL_COUNT],
    const MotorTelemetrySink sinks[MOTOR_TELEMETRY_PORT_COUNT]);
void MotorTelemetryCore_OnComplete(MotorTelemetryCore *core,
                                    MotorTelemetryPortId id);
void MotorTelemetryCore_OnError(MotorTelemetryCore *core,
                                 MotorTelemetryPortId id);

#endif /* MOTOR_TELEMETRY_CORE_H */
