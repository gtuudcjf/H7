#include "motor_telemetry.h"
#include "motor_control.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
static MotorTelemetryCore telemetry;
static UART_HandleTypeDef *telemetry_uart;
volatile MotorTelemetryDebug g_motor_telemetry_debug;

static void PublishPort(volatile MotorTelemetryPortDebug *debug,
                        const MotorTelemetryPort *port)
{
    debug->enabled = port->enabled;
    debug->busy = port->busy;
    debug->completed = port->completed;
    debug->dropped = port->dropped;
    debug->errors = port->errors;
}

static void PublishDebug(void)
{
    PublishPort(&g_motor_telemetry_debug.uart, &telemetry.port[MOTOR_TELEMETRY_UART]);
    PublishPort(&g_motor_telemetry_debug.usb, &telemetry.port[MOTOR_TELEMETRY_USB]);
    g_motor_telemetry_debug.invalid_values = telemetry.invalid_values;
}

static MotorTelemetrySendResult SendUart(void *context, uint8_t *frame, uint16_t length)
{
    HAL_StatusTypeDef status;
    UART_HandleTypeDef *uart = (UART_HandleTypeDef *)context;
    if (uart == NULL) return MOTOR_TELEMETRY_SEND_ERROR;
    status = HAL_UART_Transmit_IT(uart, frame, length);
    if (status == HAL_OK) return MOTOR_TELEMETRY_SEND_ACCEPTED;
    return (status == HAL_BUSY) ? MOTOR_TELEMETRY_SEND_SKIP : MOTOR_TELEMETRY_SEND_ERROR;
}

static MotorTelemetrySendResult SendUsb(void *context, uint8_t *frame, uint16_t length)
{
    uint8_t status = USBD_BUSY;
    uint32_t enabled = NVIC_GetEnableIRQ(OTG_FS_IRQn);
    (void)context;
    /* Serialize submission against CDC completion/deinit, not motor interrupts. */
    NVIC_DisableIRQ(OTG_FS_IRQn);
    __DSB();
    __ISB();
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
        hUsbDeviceFS.pClassData != NULL)
    {
        /* A deinit interrupt before the mask may have released the old claim. */
        telemetry.port[MOTOR_TELEMETRY_USB].busy = true;
        status = CDC_Transmit_FS(frame, length);
    }
    if (enabled != 0U) NVIC_EnableIRQ(OTG_FS_IRQn);
    if (status == USBD_OK) return MOTOR_TELEMETRY_SEND_ACCEPTED;
    return (status == USBD_BUSY) ? MOTOR_TELEMETRY_SEND_SKIP : MOTOR_TELEMETRY_SEND_ERROR;
}

void MotorTelemetry_Init(UART_HandleTypeDef *uart, bool uart_enabled, bool usb_enabled)
{
    telemetry_uart = uart;
    MotorTelemetryCore_Init(&telemetry, HAL_GetTick(), uart_enabled, usb_enabled);
    PublishDebug();
}

void MotorTelemetry_Service(void)
{
    float values[MOTOR_TELEMETRY_CHANNEL_COUNT];
    uint32_t now = HAL_GetTick();
    uint32_t primask;
    const MotorTelemetrySink sinks[MOTOR_TELEMETRY_PORT_COUNT] = {
        {SendUart, telemetry_uart}, {SendUsb, NULL}
    };
    if ((uint32_t)(now - telemetry.last_ms) < MOTOR_TELEMETRY_PERIOD_MS) return;
    if (telemetry.port[MOTOR_TELEMETRY_UART].enabled ||
        telemetry.port[MOTOR_TELEMETRY_USB].enabled)
    {
        primask = __get_PRIMASK();
        __disable_irq();
        values[0] = g_motor_control_debug.id_ref_a;
        values[1] = g_motor_control_debug.id_a;
        values[2] = g_motor_control_debug.iq_ref_a;
        values[3] = g_motor_control_debug.iq_a;
        values[4] = g_motor_control_debug.speed_active_target_rpm;
        values[5] = g_motor_control_debug.speed_filtered_rpm;
        values[6] = g_motor_control_debug.position_target_deg;
        values[7] = g_motor_control_debug.position_feedback_deg;
        __set_PRIMASK(primask);
    }
    MotorTelemetryCore_Service(&telemetry, now, values, sinks);
    PublishDebug();
}

void MotorTelemetry_SetUartEnabled(bool enabled)
{
    MotorTelemetryCore_SetEnabled(&telemetry, MOTOR_TELEMETRY_UART, enabled);
    PublishDebug();
}

void MotorTelemetry_SetUsbEnabled(bool enabled)
{
    MotorTelemetryCore_SetEnabled(&telemetry, MOTOR_TELEMETRY_USB, enabled);
    PublishDebug();
}

void MotorTelemetry_OnUartComplete(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart != telemetry_uart) return;
    MotorTelemetryCore_OnComplete(&telemetry, MOTOR_TELEMETRY_UART);
}

void MotorTelemetry_OnUartError(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart != telemetry_uart) return;
    /* RX errors do not terminate an interrupt-driven TX in the H7 HAL. */
    if (uart->gState != HAL_UART_STATE_BUSY_TX)
        MotorTelemetryCore_OnError(&telemetry, MOTOR_TELEMETRY_UART);
}

void MotorTelemetry_OnUsbComplete(void)
{
    MotorTelemetryCore_OnComplete(&telemetry, MOTOR_TELEMETRY_USB);
}

void MotorTelemetry_OnUsbDisconnected(void)
{
    MotorTelemetryCore_OnError(&telemetry, MOTOR_TELEMETRY_USB);
}
