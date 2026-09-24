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

typedef struct
{
    uint32_t calls;
    MotorTelemetrySendResult result;
    const uint8_t *last_pointer;
    uint8_t last_frame[MOTOR_TELEMETRY_FRAME_BYTES];
} FakeSink;

static MotorTelemetrySendResult FakeSend(void *context, uint8_t *frame,
                                         uint16_t length)
{
    FakeSink *sink = (FakeSink *)context;

    assert(length == MOTOR_TELEMETRY_FRAME_BYTES);
    ++sink->calls;
    sink->last_pointer = frame;
    memcpy(sink->last_frame, frame, length);
    return sink->result;
}

static void TestIndependentPortsAndPersistentBuffers(void)
{
    MotorTelemetryCore core;
    FakeSink uart = {0U, MOTOR_TELEMETRY_SEND_ACCEPTED, 0, {0}};
    FakeSink usb = {0U, MOTOR_TELEMETRY_SEND_ACCEPTED, 0, {0}};
    const MotorTelemetrySink sinks[2] = {
        {FakeSend, &uart}, {FakeSend, &usb}
    };
    const float first[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const float second[8] = {9, 10, 11, 12, 13, 14, 15, 16};
    uint8_t previous_uart_frame[MOTOR_TELEMETRY_FRAME_BYTES];

    MotorTelemetryCore_Init(&core, 0U, true, true);
    MotorTelemetryCore_Service(&core, 9U, first, sinks);
    assert(uart.calls == 0U && usb.calls == 0U);
    MotorTelemetryCore_Service(&core, 10U, first, sinks);
    assert(uart.calls == 1U && usb.calls == 1U);
    assert(core.port[MOTOR_TELEMETRY_UART].busy);
    assert(core.port[MOTOR_TELEMETRY_USB].busy);
    assert(uart.last_pointer == core.port[MOTOR_TELEMETRY_UART].frame);
    assert(usb.last_pointer == core.port[MOTOR_TELEMETRY_USB].frame);
    memcpy(previous_uart_frame, uart.last_frame, sizeof(previous_uart_frame));

    MotorTelemetryCore_OnComplete(&core, MOTOR_TELEMETRY_USB);
    MotorTelemetryCore_Service(&core, 20U, second, sinks);
    assert(uart.calls == 1U && usb.calls == 2U);
    assert(core.port[MOTOR_TELEMETRY_UART].dropped == 1U);
    assert(core.port[MOTOR_TELEMETRY_USB].completed == 1U);
    assert(memcmp(previous_uart_frame,
                  core.port[MOTOR_TELEMETRY_UART].frame,
                  sizeof(previous_uart_frame)) == 0);
    assert(memcmp(usb.last_frame, previous_uart_frame,
                  sizeof(previous_uart_frame)) != 0);

    MotorTelemetryCore_OnComplete(&core, MOTOR_TELEMETRY_UART);
    MotorTelemetryCore_Service(&core, 30U, second, sinks);
    assert(uart.calls == 2U && usb.calls == 2U);
    assert(core.port[MOTOR_TELEMETRY_USB].dropped == 1U);
}

static void TestIndependentEnableAndSendFailures(void)
{
    MotorTelemetryCore core;
    FakeSink uart = {0U, MOTOR_TELEMETRY_SEND_SKIP, 0, {0}};
    FakeSink usb = {0U, MOTOR_TELEMETRY_SEND_ACCEPTED, 0, {0}};
    const MotorTelemetrySink sinks[2] = {
        {FakeSend, &uart}, {FakeSend, &usb}
    };
    const float values[8] = {0};

    MotorTelemetryCore_Init(&core, 0U, true, true);
    MotorTelemetryCore_Service(&core, 10U, values, sinks);
    assert(uart.calls == 1U && usb.calls == 1U);
    assert(!core.port[MOTOR_TELEMETRY_UART].busy);
    assert(core.port[MOTOR_TELEMETRY_UART].dropped == 1U);
    MotorTelemetryCore_OnComplete(&core, MOTOR_TELEMETRY_USB);

    MotorTelemetryCore_SetEnabled(&core, MOTOR_TELEMETRY_USB, false);
    uart.result = MOTOR_TELEMETRY_SEND_ERROR;
    MotorTelemetryCore_Service(&core, 20U, values, sinks);
    assert(uart.calls == 2U && usb.calls == 1U);
    assert(core.port[MOTOR_TELEMETRY_UART].errors == 1U);
    assert(!core.port[MOTOR_TELEMETRY_UART].busy);

    MotorTelemetryCore_SetEnabled(&core, MOTOR_TELEMETRY_UART, false);
    MotorTelemetryCore_SetEnabled(&core, MOTOR_TELEMETRY_USB, true);
    MotorTelemetryCore_Service(&core, 30U, values, sinks);
    assert(uart.calls == 2U && usb.calls == 2U);
    MotorTelemetryCore_OnError(&core, MOTOR_TELEMETRY_USB);
    assert(!core.port[MOTOR_TELEMETRY_USB].busy);
    assert(core.port[MOTOR_TELEMETRY_USB].errors == 1U);
}

static void TestTimeWrapAndNoBacklog(void)
{
    MotorTelemetryCore core;
    FakeSink uart = {0U, MOTOR_TELEMETRY_SEND_ACCEPTED, 0, {0}};
    const MotorTelemetrySink sinks[2] = {
        {FakeSend, &uart}, {0, 0}
    };
    const float values[8] = {0};

    MotorTelemetryCore_Init(&core, UINT32_MAX - 5U, true, false);
    MotorTelemetryCore_Service(&core, 3U, values, sinks);
    assert(uart.calls == 0U);
    MotorTelemetryCore_Service(&core, 4U, values, sinks);
    assert(uart.calls == 1U);
    MotorTelemetryCore_OnComplete(&core, MOTOR_TELEMETRY_UART);
    MotorTelemetryCore_Service(&core, 1000U, values, sinks);
    assert(uart.calls == 2U);
    MotorTelemetryCore_OnComplete(&core, MOTOR_TELEMETRY_UART);
    MotorTelemetryCore_Service(&core, 1001U, values, sinks);
    assert(uart.calls == 2U);
}

static void TestInvalidArgumentsAreIgnored(void)
{
    MotorTelemetryCore core;
    const float values[8] = {0};
    const MotorTelemetrySink sinks[2] = {{0, 0}, {0, 0}};

    MotorTelemetryCore_Init(&core, 0U, true, false);
    MotorTelemetryCore_Init(0, 0U, true, true);
    MotorTelemetryCore_SetEnabled(0, MOTOR_TELEMETRY_UART, false);
    MotorTelemetryCore_SetEnabled(&core, MOTOR_TELEMETRY_PORT_COUNT, false);
    MotorTelemetryCore_OnComplete(0, MOTOR_TELEMETRY_UART);
    MotorTelemetryCore_OnError(&core, MOTOR_TELEMETRY_PORT_COUNT);
    MotorTelemetryCore_Service(0, 10U, values, sinks);
    MotorTelemetryCore_Service(&core, 10U, 0, sinks);
    MotorTelemetryCore_Service(&core, 10U, values, 0);
    assert(core.port[MOTOR_TELEMETRY_UART].enabled);
    assert(core.port[MOTOR_TELEMETRY_UART].completed == 0U);
    assert(core.port[MOTOR_TELEMETRY_UART].errors == 0U);
    assert(core.last_ms == 0U);
}

int main(void)
{
    TestJustFloatFrame();
    TestNonFiniteValuesBecomeZero();
    TestIndependentPortsAndPersistentBuffers();
    TestIndependentEnableAndSendFailures();
    TestTimeWrapAndNoBacklog();
    TestInvalidArgumentsAreIgnored();
    puts("motor telemetry core tests passed");
    return 0;
}
