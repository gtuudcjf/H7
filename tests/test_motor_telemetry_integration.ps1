$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
function Read-Source($path) { Get-Content -Raw -Encoding UTF8 (Join-Path $root $path) }
function Require($text, $pattern, $message) {
    if ($text -notmatch $pattern) { throw $message }
}
$main = Read-Source 'Core/Src/main.c'
$irq = Read-Source 'Core/Src/stm32h7xx_it.c'
$uart = Read-Source 'Core/Src/usart.c'
$cdc = Read-Source 'USB_DEVICE/App/usbd_cdc_if.c'
$project = Read-Source 'MDK-ARM/emptytest.uvprojx'
Require $main 'MotorControl_Start\(\)[\s\S]*?MotorTelemetry_Init\(&huart1' 'Telemetry initialization missing.'
Require $main 'MotorControl_Service\(\);\s*MotorTelemetry_Service\(\);' 'Foreground telemetry missing.'
Require $irq 'USART1_IRQHandler\(void\)[\s\S]*?HAL_UART_IRQHandler\(&huart1\)' 'USART1 IRQ missing.'
if ($irq -match 'USER CODE BEGIN 1[\s\S]*?USART1_IRQHandler[\s\S]*?USER CODE END 1') {
    throw 'USART1 handler in preserved USER section duplicates CubeMX generation.'
}
if ([regex]::Matches($irq, 'void\s+USART1_IRQHandler\s*\(').Count -ne 1) {
    throw 'USART1 handler must have exactly one definition.'
}
Require $irq 'USART1_IRQHandler\(void\)[\s\S]*?USER CODE BEGIN USART1_IRQn 0[\s\S]*?HAL_UART_IRQHandler\(&huart1\)[\s\S]*?USER CODE BEGIN USART1_IRQn 1' 'USART1 must use standard generated handler layout.'
Require $uart 'HAL_NVIC_SetPriority\(USART1_IRQn, 5, 0\)' 'UART priority must be 5.'
Require $uart 'HAL_NVIC_EnableIRQ\(USART1_IRQn\)' 'UART IRQ not enabled.'
Require $main 'HAL_UART_TxCpltCallback[\s\S]*?MotorTelemetry_OnUartComplete\(huart\)' 'UART completion missing.'
Require $main 'HAL_UART_ErrorCallback[\s\S]*?MotorTelemetry_OnUartError\(huart\)' 'UART error missing.'
Require $cdc 'USER CODE BEGIN 4[\s\S]*?MotorTelemetry_OnUsbDisconnected\(\)' 'CDC disconnect missing.'
Require $cdc 'USER CODE BEGIN 13[\s\S]*?MotorTelemetry_OnUsbComplete\(\)' 'CDC completion missing.'
Require $cdc 'dev_state != USBD_STATE_CONFIGURED[\s\S]*?pClassData == NULL[\s\S]*?return USBD_BUSY;[\s\S]*?hcdc->TxState' 'CDC null/configuration guard missing.'
foreach ($name in @('motor_telemetry_core.c', 'motor_telemetry.c')) {
    Require $project ([regex]::Escape($name)) "Keil missing $name"
}
Require (Read-Source 'emptytest.ioc') 'NVIC.USART1_IRQn=true\\:5\\:0' 'CubeMX UART IRQ missing.'
Write-Output 'motor telemetry integration tests passed'
