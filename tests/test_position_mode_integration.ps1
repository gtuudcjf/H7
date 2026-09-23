$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$header = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'Core/Inc/motor/motor_control.h')
$source = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'Core/Src/motor/motor_control.c')
$params = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'Core/Inc/motor/motor_params.h')
$main = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'Core/Src/main.c')
$project = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'MDK-ARM/emptytest.uvprojx')

function Assert-Contains([string]$Text, [string]$Pattern, [string]$Message)
{
    if ($Text -notmatch $Pattern) { throw $Message }
}

Assert-Contains $header 'MotorControl_SetPositionCommand\s*\(' `
    'The public position command API is missing.'
Assert-Contains $header 'MotorControl_SwitchToEncoderPositionCurrent\s*\(' `
    'The position mode switch API is missing.'
foreach ($field in @('position_target_deg', 'position_feedback_deg',
                    'position_error_deg', 'position_speed_target_rpm',
                    'position_control_ready'))
{
    Assert-Contains $header ("\b" + $field + "\b") "Debug field $field is missing."
}
foreach ($name in @('MOTOR_POSITION_KP_RPM_PER_DEG',
                  'MOTOR_POSITION_SPEED_LIMIT_RPM',
                  'MOTOR_POSITION_TOLERANCE_DEG'))
{
    Assert-Contains $params $name "Position parameter $name is missing."
}
Assert-Contains $source '#include\s+"position_controller\.h"' `
    'Motor control does not include the pure position controller.'
Assert-Contains $source 'PositionController_Init\s*\(' `
    'MotorControl_Init does not validate and initialize the position controller.'
Assert-Contains $source 'MotorControl_CapturePositionTarget\s*\(' `
    'Position mode does not capture the current shaft angle on entry.'
Assert-Contains $source 'encoder_angle_sample\.mechanical_angle_pu\s*\*\s*360\.0f' `
    'Position feedback must use the calibrated mechanical angle.'
Assert-Contains $source 'mode\s*==\s*MOTOR_CONTROL_ENCODER_POSITION_CURRENT' `
    'The new position mode is not recognized by control predicates.'
Assert-Contains $source 'run_state\s*!=\s*MOTOR_RUN_STATE_RUNNING' `
    'Position commands must be rejected outside RUNNING.'
Assert-Contains $source 'target_deg\s*>=\s*360\.0f' `
    'Position commands must reject the out-of-range 360-degree boundary.'
Assert-Contains $source '!encoder_calibration_valid' `
    'Position mode must continue to require encoder calibration.'
Assert-Contains $source 'MOTOR_FAULT_ENCODER_STALE' `
    'Position mode must retain stale-encoder protection.'

$task = [regex]::Match($source,
    'static bool MotorControl_RunSpeedTask\s*\(void\)[\s\S]*?\n\}')
if (-not $task.Success) { throw 'Could not locate the 1 kHz speed task.' }
Assert-Contains $task.Value 'PositionController_Step\s*\(' `
    'The 1 kHz task does not execute the position outer loop.'
Assert-Contains $task.Value 'requested_speed_rpm\s*=\s*speed_target_rpm' `
    'Mode 4 must continue to use its independent speed command.'
Assert-Contains $task.Value 'SpeedPi_Step\s*\(' `
    'Position mode must reuse the existing inner speed PI.'
Assert-Contains $task.Value 'speed_mode_waiting_for_estimator[\s\S]*?MOTOR_CONTROL_ENCODER_POSITION_CURRENT[\s\S]*?speed_active_target_rpm\s*=\s*0\.0f' `
    'Position mode must not inherit measured speed while waiting for the estimator.'
if ($task.Value.IndexOf('PositionController_Step') -gt
    $task.Value.IndexOf('SpeedPi_Step'))
{
    throw 'The position outer loop must execute before the speed PI.'
}

Assert-Contains $source 'motor_mode\s*==\s*MOTOR_CONTROL_ENCODER_ANGLE_CURRENT' `
    'Mode 3 current reference selection is missing.'
$handoff = [regex]::Match($source,
    'static void MotorControl_ApplyModeRequest\s*\(void\)[\s\S]*?\n\}')
if (-not $handoff.Success) { throw 'Could not locate mode handoff.' }
Assert-Contains $handoff.Value 'speed_estimator\.ready[\s\S]*?MOTOR_CONTROL_ENCODER_POSITION_CURRENT[\s\S]*?speed_active_target_rpm\s*=\s*0\.0f' `
    'Mode 4 to position handoff must start from zero speed reference.'
Assert-Contains $main '\.mode\s*=\s*MOTOR_CONTROL_ENCODER_SPEED_CURRENT' `
    'The validated speed mode must remain the default.'
Assert-Contains $main 'MotorControl_SetSpeedCommand\s*\(50\.0f\)' `
    'The validated 50 rpm speed command must remain unchanged.'
Assert-Contains $project '<FilePath>\.\./Core/Src/motor/position_controller\.c</FilePath>' `
    'The Keil project does not compile position_controller.c.'

Write-Host 'position-mode integration checks passed'
