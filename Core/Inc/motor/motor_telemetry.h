#ifndef MOTOR_TELEMETRY_H
#define MOTOR_TELEMETRY_H

#include "stm32h7xx_hal.h"
#include "motor_telemetry_core.h"

/* Foreground-only configuration; never reinitialize an active transfer. */
#define MOTOR_TELEMETRY_UART_ENABLED false
#define MOTOR_TELEMETRY_USB_ENABLED true

void MotorTelemetry_Init(UART_HandleTypeDef *uart, bool uart_enabled, bool usb_enabled);
void MotorTelemetry_Service(void);
void MotorTelemetry_SetUartEnabled(bool enabled);
void MotorTelemetry_SetUsbEnabled(bool enabled);
void MotorTelemetry_OnUartComplete(UART_HandleTypeDef *uart);
void MotorTelemetry_OnUartError(UART_HandleTypeDef *uart);
void MotorTelemetry_OnUsbComplete(void);
void MotorTelemetry_OnUsbDisconnected(void);

#endif
