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

function Assert-NotContains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
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

foreach ($field in @(
    'mode', 'requested_mode', 'run_state', 'fault',
    'phase_a_raw', 'phase_b_raw', 'phase_a_offset', 'phase_b_offset',
    'id_ref_a', 'id_a', 'iq_ref_a', 'iq_a', 'electrical_angle_pu',
    'speed_target_rpm', 'speed_active_target_rpm', 'speed_filtered_rpm',
    'speed_iq_command_a', 'speed_pi_saturated',
    'position_target_deg', 'position_feedback_deg', 'position_error_deg',
    'adc_age_ticks', 'overcurrent_count', 'voltage_saturated',
    'encoder_ready', 'encoder_calibrated', 'encoder_position_raw',
    'encoder_age_ticks', 'encoder_frame_status', 'encoder_crc_error_count',
    'encoder_frame_error_count', 'encoder_spi_error_count',
    'encoder_timeout_count', 'encoder_dma_guard_error_count'
)) {
    Assert-Contains $header ("\b" + [regex]::Escape($field) + "\b") `
        "Essential motor debug field is missing: $field"
}

foreach ($removed in @(
    'speed_pi_proportional_a', 'speed_pi_integrator_a',
    'ud_integrator_v', 'uq_integrator_v', 'i_alpha_a', 'i_beta_a',
    'encoder_raw', 'encoder_received_crc', 'encoder_calculated_crc',
    'startup_trace_count', 'startup_trace_stage', 'foreground_service_count',
    'speed_control_tick_count', 'position_speed_target_rpm',
    'current_voltage_limit_pu'
)) {
    Assert-NotContains ($header + "`n" + $control) `
        ("\b" + [regex]::Escape($removed) + "\b") `
        "Redundant motor diagnostic state is still present: $removed"
}
Assert-NotContains ($header + "`n" + $control) 'motor_startup_trace\.h|MotorStartupTrace' `
    'Startup-only trace code must be removed from motor control.'

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
Assert-Contains $main 'MotorControl_SetEncoderCurrentCommand\(0\.0f,\s*0\.0f\)' `
    'The encoder-angle current startup profile must remain at zero current for safe diagnostics.'
Assert-Contains $main 'HAL_ADCEx_InjectedConvCpltCallback' 'ADC injected completion callback must be integrated.'
Assert-Contains $main 'ADC_INJECTED_RANK_1[\s\S]*ADC_INJECTED_RANK_2' 'ADC callback must read I_A before I_B.'

Assert-Contains $keilProject 'current_sense\.c' 'Keil project must compile the current-sense module.'
Assert-Contains $keilProject 'foc_transform\.c' 'Keil project must compile the FOC transform module.'
Assert-Contains $keilProject 'current_pi\.c' 'Keil project must compile the current PI module.'
Assert-Contains $keilProject 'motor_runtime_policy\.c' `
    'Keil project must compile the motor runtime policy module.'

Write-Output 'motor-control integration checks passed'
