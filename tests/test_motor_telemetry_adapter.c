#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_telemetry.h"
#include "motor_control.h"
#include "usbd_cdc_if.h"

volatile MotorControlDebug g_motor_control_debug;
USBD_HandleTypeDef hUsbDeviceFS;
static uint32_t tick, primask, mask_calls, usb_irq = 1U;
static unsigned uart_calls, usb_calls;
static uint8_t *uart_frame, *usb_frame;
static HAL_StatusTypeDef uart_result = HAL_OK;
static uint8_t usb_result = USBD_OK;
static int disconnect_before_mask;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; ++mask_calls; }
void __set_PRIMASK(uint32_t value) { primask = value; }
uint32_t NVIC_GetEnableIRQ(int irq) { (void)irq; return usb_irq; }
void NVIC_DisableIRQ(int irq)
{
    (void)irq;
    if (disconnect_before_mask) {
        disconnect_before_mask = 0;
        MotorTelemetry_OnUsbDisconnected();
    }
    usb_irq = 0U;
}
void NVIC_EnableIRQ(int irq) { (void)irq; usb_irq = 1U; }
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t len)
{
    assert(len == 36U);
    ++uart_calls;
    uart_frame = data;
    if (uart_result == HAL_OK) uart->gState = HAL_UART_STATE_BUSY_TX;
    return uart_result;
}
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t len)
{
    assert(len == 36U && usb_irq == 0U);
    ++usb_calls;
    usb_frame = data;
    return usb_result;
}
static void Advance(void) { tick += 10U; MotorTelemetry_Service(); }
int main(void)
{
    UART_HandleTypeDef uart = {HAL_UART_STATE_READY};
    UART_HandleTypeDef other = {HAL_UART_STATE_READY};
    uint8_t expected[36], saved[36];
    const float values[8] = {1,2,3,4,5,6,7,8};
    g_motor_control_debug.id_ref_a = 1;
    g_motor_control_debug.id_a = 2;
    g_motor_control_debug.iq_ref_a = 3;
    g_motor_control_debug.iq_a = 4;
    g_motor_control_debug.speed_active_target_rpm = 5;
    g_motor_control_debug.speed_filtered_rpm = 6;
    g_motor_control_debug.position_target_deg = 7;
    g_motor_control_debug.position_feedback_deg = 8;
    MotorTelemetryCore_Encode(values, expected);
    MotorTelemetry_Init(&uart, true, true);
    MotorTelemetry_Service();
    assert(mask_calls == 0U);
    Advance();
    assert(uart_calls == 1U && usb_calls == 0U);
    assert(memcmp(uart_frame, expected, 36) == 0 && primask == 0U);
    assert(g_motor_telemetry_debug.usb.dropped == 1U);
    memcpy(saved, uart_frame, 36);
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = &hUsbDeviceFS;
    primask = 1U;
    Advance();
    assert(primask == 1U && uart_calls == 1U && usb_calls == 1U);
    assert(uart_frame != usb_frame && memcmp(uart_frame, saved, 36) == 0);
    MotorTelemetry_OnUartError(&uart); /* RX error while TX is active */
    MotorTelemetry_OnUartComplete(&other);
    Advance();
    assert(uart_calls == 1U && usb_calls == 1U);
    uart.gState = HAL_UART_STATE_READY;
    MotorTelemetry_OnUartComplete(&uart);
    MotorTelemetry_OnUsbComplete();
    MotorTelemetry_SetUartEnabled(false);
    disconnect_before_mask = 1;
    Advance();
    assert(g_motor_telemetry_debug.usb.busy); /* reconnect before submission */
    assert(usb_calls == 2U);
    Advance();
    assert(usb_calls == 2U);
    MotorTelemetry_OnUsbDisconnected();
    MotorTelemetry_SetUsbEnabled(false);
    {
        uint32_t previous_masks = mask_calls;
        Advance();
        assert(mask_calls == previous_masks);
    }
    MotorTelemetry_SetUartEnabled(true);
    uart_result = HAL_BUSY;
    Advance();
    assert(!g_motor_telemetry_debug.uart.busy);
    uart_result = HAL_ERROR;
    Advance();
    assert(g_motor_telemetry_debug.uart.errors == 1U);
    MotorTelemetry_SetUsbEnabled(true);
    usb_result = USBD_FAIL;
    usb_irq = 0U;
    Advance();
    assert(!g_motor_telemetry_debug.usb.busy && usb_irq == 0U);
    puts("motor telemetry adapter tests passed");
    return 0;
}
