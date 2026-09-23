$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$header = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\motor\motor_control.h') -Raw
$policyHeader = Get-Content -LiteralPath `
    (Join-Path $projectRoot 'Core\Inc\motor\motor_runtime_policy.h') -Raw
$publicControlHeaders = $header + "`n" + $policyHeader
$control = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\motor\motor_control.c') -Raw
$main = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\main.c') -Raw
$project = Get-Content -LiteralPath (Join-Path $projectRoot 'MDK-ARM\emptytest.uvprojx') -Raw

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

Assert-Contains $publicControlHeaders 'MOTOR_CONTROL_ENCODER_ANGLE_CURRENT' `
    'The encoder-angle current mode must have an explicit name.'
Assert-Contains $header '#define\s+MOTOR_CONTROL_ENCODER_CURRENT\s+MOTOR_CONTROL_ENCODER_ANGLE_CURRENT' `
    'The previous reserved encoder mode name must remain as an alias.'
Assert-Contains $header 'MotorControl_SetEncoderCurrentCommand\s*\(float id_a,\s*float iq_a\)' `
    'Encoder mode requires a frequency-independent Id/Iq command.'
Assert-Contains $header 'MotorControl_SwitchToEncoderAngleCurrent\s*\(float id_a,[\s\S]*float iq_a\)' `
    'Encoder mode requires a one-call command-and-switch API.'
Assert-Contains $header 'MotorControl_RequestEncoderCalibration\s*\(void\)' `
    'Encoder calibration must be explicitly requested.'
Assert-Contains $header 'MotorControl_Service\s*\(void\)' `
    'Flash saving must run from a foreground service.'

Assert-Contains $control 'static bool MotorControl_RunCurrentLoop\(float dt_s,\s*float electrical_angle_pu\)' `
    'The current loop must receive one explicit immutable angle per ADC callback.'
Assert-Contains $control 'MOTOR_CONTROL_OPEN_ANGLE_CURRENT[\s\S]*open_loop_state\.electrical_angle_pu' `
    'Open-angle current mode must retain its virtual electrical angle.'
Assert-Contains $control 'MOTOR_CONTROL_ENCODER_ANGLE_CURRENT[\s\S]*encoder_angle_sample\.electrical_angle_pu' `
    'Encoder current mode must use the validated encoder angle.'
Assert-Contains $control 'run_state\s*==\s*MOTOR_RUN_STATE_READY[\s\S]*MotorControl_UpdateEncoderAngle\(true\)[\s\S]*MotorControl_StartSelectedMode\(\)' `
    'Direct mode-3 startup must wait safely for the first valid encoder frame.'
Assert-Contains $control 'BissEncoder_ControlTick\(\)' `
    'The 10 kHz tick must maintain encoder age and DMA timeout state.'
Assert-Contains $control 'BissEncoder_StartRead\(\)' `
    'The 10 kHz tick must schedule nonblocking encoder frames.'
Assert-Contains $control 'MotorConfigStore_Save[\s\S]*safe_to_write' `
    'Calibration saving must be explicitly guarded by stopped-power authorization.'
Assert-NotContains $control 'HAL_Delay\s*\(' 'Motor control must not introduce blocking delay calls.'

Assert-Contains $main 'while\s*\(1\)[\s\S]*MotorControl_Service\(\)' `
    'The main loop must service calibration completion and Flash writes.'
Assert-Contains $main '\.mode\s*=\s*MOTOR_CONTROL_(OPEN_VOLTAGE|OPEN_ANGLE_CURRENT|ENCODER_ANGLE_CURRENT|ENCODER_SPEED_CURRENT|ENCODER_POSITION_CURRENT)' `
    'Startup mode selection must accept each implemented mode.'
Assert-Contains $main 'MotorControl_SetEncoderCurrentCommand\(0\.0f,\s*0\.6f\)' `
    'The validated encoder-current command must be preloaded before startup.'
Assert-NotContains $main 'HAL_GetTick\(\)\s*>=' `
    'Production main must not contain time-triggered calibration or motion tests.'

Assert-Contains $project '<FileName>encoder_calibration\.c</FileName>' `
    'Keil project must compile the encoder calibration state machine.'
Assert-Contains $project '<FileName>biss_dma_buffer\.c</FileName>' `
    'Keil project must compile the isolated BiSS DMA buffer module.'

Write-Output 'encoder-angle current-mode integration checks passed'
