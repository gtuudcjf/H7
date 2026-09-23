$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$header = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\motor\motor_control.h') -Raw
$policyHeader = Get-Content -LiteralPath `
    (Join-Path $projectRoot 'Core\Inc\motor\motor_runtime_policy.h') -Raw
$publicControlHeaders = $header + "`n" + $policyHeader
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

Assert-Contains $publicControlHeaders 'MOTOR_CONTROL_OPEN_VOLTAGE' 'Voltage-open-loop mode must remain public.'
Assert-Contains $publicControlHeaders 'MOTOR_CONTROL_OPEN_ANGLE_CURRENT' 'Open-angle current mode must be public.'
Assert-Contains $header 'MotorControl_RequestMode' 'A boundary-applied runtime mode request API is required.'
Assert-Contains $header 'MotorControl_SwitchToOpenVoltage' 'A one-call voltage-mode switch API is required.'
Assert-Contains $header 'MotorControl_SwitchToOpenAngleCurrent' 'A one-call open-angle current-mode switch API is required.'
Assert-Contains $header 'MotorControl_SwitchToEncoderAngleCurrent' 'A one-call encoder-angle current-mode switch API is required.'
Assert-Contains $header 'MotorControl_SetCurrentPiGains' 'Current PI gains must be tunable through a validated API.'
Assert-Contains $header 'g_motor_control_debug' 'Keil/Ozone debug observability must be exported.'

Assert-Contains $control 'CURRENT_CALIBRATION_DISCARD_COUNT\s+\(16U\)' 'Calibration must discard 16 samples.'
Assert-Contains $control 'CURRENT_CALIBRATION_SAMPLE_COUNT\s+\(256U\)' 'Calibration must average 256 samples.'
Assert-Contains $control 'CURRENT_OVERCURRENT_CONFIRM_COUNT\s+\(2U\)' 'Overcurrent must require two consecutive samples.'
Assert-Contains $control 'CurrentPi_PreloadOutput' 'Open-loop to current-loop switching must preload the PI.'
Assert-Contains $control 'open_voltage_blend_active\s*=\s*true' 'Current-loop to open-loop switching must blend voltage.'
Assert-Contains $control 'open_voltage_magnitude\s*>\s*MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU' `
    'High-modulation voltage open loop must not trust an invalid fixed low-side sample.'

Assert-Contains $params 'MOTOR_CURRENT_PI_KP_V_PER_A\s+\(0\.05f\)' 'Conservative current Kp default is required.'
Assert-Contains $params 'MOTOR_CURRENT_PI_KI_V_PER_A_S\s+\(20\.0f\)' 'Conservative current Ki default is required.'
Assert-Contains $params 'MOTOR_CURRENT_START_IQ_A\s+\(0\.3f\)' 'Initial current command must remain 0.3 A.'
Assert-Contains $params 'MOTOR_CURRENT_COMMAND_LIMIT_A\s+\(2\.0f\)' 'Current command must be limited to 2 A.'
Assert-Contains $params 'MOTOR_CURRENT_TRIP_A\s+\(10\.0f\)' 'Software overcurrent threshold must be 10 A.'

Assert-Contains $main '\.mode\s*=\s*MOTOR_CONTROL_(OPEN_VOLTAGE|OPEN_ANGLE_CURRENT|ENCODER_ANGLE_CURRENT|ENCODER_SPEED_CURRENT|ENCODER_POSITION_CURRENT)' `
    'The selected startup mode must be one of the implemented modes.'
Assert-Contains $main 'MotorControl_SetOpenLoopCommand\(0\.0f,\s*0\.08f,\s*1\.0f\)' `
    'The voltage-open-loop startup profile must be initialized independently of mode selection.'
Assert-Contains $main 'MotorControl_SetCurrentCommand\(0\.0f,\s*0\.8f,\s*1\.0f\)' `
    'The open-angle current startup profile must be initialized independently of mode selection.'
Assert-Contains $main 'MotorControl_SetEncoderCurrentCommand\(0\.0f,\s*0\.6f\)' `
    'The encoder-angle current startup profile must be initialized independently of mode selection.'
Assert-Contains $main 'HAL_ADCEx_InjectedConvCpltCallback' 'ADC injected completion callback must be integrated.'
Assert-Contains $main 'ADC_INJECTED_RANK_1[\s\S]*ADC_INJECTED_RANK_2' 'ADC callback must read I_A before I_B.'

Assert-Contains $keilProject 'current_sense\.c' 'Keil project must compile the current-sense module.'
Assert-Contains $keilProject 'foc_transform\.c' 'Keil project must compile the FOC transform module.'
Assert-Contains $keilProject 'current_pi\.c' 'Keil project must compile the current PI module.'
Assert-Contains $keilProject 'motor_runtime_policy\.c' `
    'Keil project must compile the motor runtime policy module.'

Write-Output 'motor-control integration checks passed'
