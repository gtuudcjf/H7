$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$header = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\motor\motor_control.h') -Raw
$control = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\motor\motor_control.c') -Raw
$main = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\main.c') -Raw
$params = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\motor\motor_params.h') -Raw
$keilProject = Get-Content -LiteralPath (Join-Path $projectRoot 'MDK-ARM\emptytest.uvprojx') -Raw

function Assert-Contains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

Assert-Contains $header 'MOTOR_CONTROL_OPEN_VOLTAGE' 'Voltage-open-loop mode must remain public.'
Assert-Contains $header 'MOTOR_CONTROL_OPEN_ANGLE_CURRENT' 'Open-angle current mode must be public.'
Assert-Contains $header 'MotorControl_RequestMode' 'A boundary-applied runtime mode request API is required.'
Assert-Contains $header 'MotorControl_SetCurrentPiGains' 'Current PI gains must be tunable through a validated API.'
Assert-Contains $header 'g_motor_control_debug' 'Keil/Ozone debug observability must be exported.'

Assert-Contains $control 'CURRENT_CALIBRATION_DISCARD_COUNT\s+\(16U\)' 'Calibration must discard 16 samples.'
Assert-Contains $control 'CURRENT_CALIBRATION_SAMPLE_COUNT\s+\(256U\)' 'Calibration must average 256 samples.'
Assert-Contains $control 'CURRENT_OVERCURRENT_CONFIRM_COUNT\s+\(2U\)' 'Overcurrent must require two consecutive samples.'
Assert-Contains $control 'CurrentPi_PreloadOutput' 'Open-loop to current-loop switching must preload the PI.'
Assert-Contains $control 'open_voltage_blend_active\s*=\s*true' 'Current-loop to open-loop switching must blend voltage.'

Assert-Contains $params 'MOTOR_CURRENT_PI_KP_V_PER_A\s+\(0\.05f\)' 'Conservative current Kp default is required.'
Assert-Contains $params 'MOTOR_CURRENT_PI_KI_V_PER_A_S\s+\(20\.0f\)' 'Conservative current Ki default is required.'
Assert-Contains $params 'MOTOR_CURRENT_START_IQ_A\s+\(0\.3f\)' 'Initial current command must remain 0.3 A.'
Assert-Contains $params 'MOTOR_CURRENT_COMMAND_LIMIT_A\s+\(2\.0f\)' 'Current command must be limited to 2 A.'
Assert-Contains $params 'MOTOR_CURRENT_TRIP_A\s+\(10\.0f\)' 'Software overcurrent threshold must be 10 A.'

Assert-Contains $main '\.mode\s*=\s*MOTOR_CONTROL_OPEN_VOLTAGE' 'Default startup must preserve voltage open loop.'
Assert-Contains $main 'HAL_ADCEx_InjectedConvCpltCallback' 'ADC injected completion callback must be integrated.'
Assert-Contains $main 'ADC_INJECTED_RANK_1[\s\S]*ADC_INJECTED_RANK_2' 'ADC callback must read I_A before I_B.'

Assert-Contains $keilProject 'current_sense\.c' 'Keil project must compile the current-sense module.'
Assert-Contains $keilProject 'foc_transform\.c' 'Keil project must compile the FOC transform module.'
Assert-Contains $keilProject 'current_pi\.c' 'Keil project must compile the current PI module.'

Write-Output 'motor-control integration checks passed'
