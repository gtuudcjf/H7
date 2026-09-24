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

void MotorTelemetryCore_Init(MotorTelemetryCore *core, uint32_t now_ms,
                             bool uart_enabled, bool usb_enabled)
{
    if (core == 0)
    {
        return;
    }

    memset(core, 0, sizeof(*core));
    core->last_ms = now_ms;
    core->port[MOTOR_TELEMETRY_UART].enabled = uart_enabled;
    core->port[MOTOR_TELEMETRY_USB].enabled = usb_enabled;
}

void MotorTelemetryCore_SetEnabled(MotorTelemetryCore *core,
                                    MotorTelemetryPortId id, bool enabled)
{
    if ((core == 0) || (id >= MOTOR_TELEMETRY_PORT_COUNT))
    {
        return;
    }
    core->port[id].enabled = enabled;
}

void MotorTelemetryCore_Service(
    MotorTelemetryCore *core, uint32_t now_ms,
    const float values[MOTOR_TELEMETRY_CHANNEL_COUNT],
    const MotorTelemetrySink sinks[MOTOR_TELEMETRY_PORT_COUNT])
{
    uint8_t encoded_frame[MOTOR_TELEMETRY_FRAME_BYTES];
    uint32_t index;

    if ((core == 0) || (values == 0) || (sinks == 0) ||
        ((uint32_t)(now_ms - core->last_ms) < MOTOR_TELEMETRY_PERIOD_MS))
    {
        return;
    }
    core->last_ms = now_ms;
    if (!core->port[MOTOR_TELEMETRY_UART].enabled &&
        !core->port[MOTOR_TELEMETRY_USB].enabled)
    {
        return;
    }

    core->invalid_values += MotorTelemetryCore_Encode(values, encoded_frame);
    for (index = 0U; index < MOTOR_TELEMETRY_PORT_COUNT; ++index)
    {
        MotorTelemetryPort *port = &core->port[index];
        MotorTelemetrySendResult result;

        if (!port->enabled)
        {
            continue;
        }
        if (port->busy)
        {
            ++port->dropped;
            continue;
        }
        if (sinks[index].send == 0)
        {
            ++port->errors;
            continue;
        }

        memcpy(port->frame, encoded_frame, sizeof(port->frame));
        port->busy = true;
        result = sinks[index].send(
            sinks[index].context, port->frame, MOTOR_TELEMETRY_FRAME_BYTES);
        if (result != MOTOR_TELEMETRY_SEND_ACCEPTED)
        {
            port->busy = false;
            if (result == MOTOR_TELEMETRY_SEND_SKIP)
            {
                ++port->dropped;
            }
            else
            {
                ++port->errors;
            }
        }
    }
}

void MotorTelemetryCore_OnComplete(MotorTelemetryCore *core,
                                    MotorTelemetryPortId id)
{
    if ((core == 0) || (id >= MOTOR_TELEMETRY_PORT_COUNT) ||
        !core->port[id].busy)
    {
        return;
    }
    ++core->port[id].completed;
    core->port[id].busy = false;
}

void MotorTelemetryCore_OnError(MotorTelemetryCore *core,
                                 MotorTelemetryPortId id)
{
    if ((core == 0) || (id >= MOTOR_TELEMETRY_PORT_COUNT) ||
        !core->port[id].busy)
    {
        return;
    }
    ++core->port[id].errors;
    core->port[id].busy = false;
}
