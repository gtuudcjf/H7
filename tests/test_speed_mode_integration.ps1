$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$policyHeader = Get-Content -Raw (Join-Path $root 'Core/Inc/motor/motor_runtime_policy.h')
$motorHeader = Get-Content -Raw (Join-Path $root 'Core/Inc/motor/motor_control.h')
$motorSource = Get-Content -Raw (Join-Path $root 'Core/Src/motor/motor_control.c')
$mainSource = Get-Content -Raw (Join-Path $root 'Core/Src/main.c')
$encoderSource = Get-Content -Raw (Join-Path $root 'Core/Src/motor/biss_encoder.c')

function Assert-Contains([string]$Text, [string]$Pattern, [string]$Message)
{
    if ($Text -notmatch $Pattern)
    {
        throw $Message
    }
}

function Assert-NotContains([string]$Text, [string]$Pattern, [string]$Message)
{
    if ($Text -match $Pattern)
    {
        throw $Message
    }
}

Assert-Contains $policyHeader 'MOTOR_CONTROL_ENCODER_SPEED_CURRENT' `
    'The fourth encoder speed/current mode is missing.'
Assert-Contains $motorHeader 'MotorControl_SetSpeedCommand\s*\(' `
    'The public speed command API is missing.'
Assert-Contains $motorHeader 'MotorControl_SwitchToEncoderSpeedCurrent\s*\(' `
    'The safe runtime switch API for speed mode is missing.'
Assert-Contains $motorHeader 'MotorControl_SetSpeedPiGains\s*\(' `
    'The speed PI tuning API is missing.'
Assert-Contains $motorHeader 'MotorControl_SetSpeedIqLimit\s*\(' `
    'The speed PI current-limit API is missing.'
Assert-Contains $motorSource 'MOTOR_SPEED_CONTROL_DIVIDER' `
    'The 10 kHz to 1 kHz speed divider is missing.'
Assert-Contains $motorSource 'SpeedEstimator_Update\s*\(' `
    'Motor control does not consume the speed estimator.'
Assert-Contains $motorSource 'SpeedPi_Step\s*\(' `
    'Motor control does not execute the speed PI.'
Assert-Contains $motorSource 'SpeedPi_PreloadOutput\s*\(' `
    'Speed mode entry does not preload PI output for a smooth transition.'
Assert-Contains $motorSource 'speed_control_tick_count' `
    'Speed-loop execution is not observable in the debugger.'
Assert-Contains $mainSource 'MotorControl_SetSpeedCommand\s*\(' `
    'main.c does not preconfigure the speed command.'

$callbackMatch = [regex]::Match(
    $motorSource,
    'void\s+MotorControl_CurrentSampleComplete\s*\([\s\S]*?\n\}')
if (-not $callbackMatch.Success)
{
    throw 'Could not locate MotorControl_CurrentSampleComplete.'
}
Assert-Contains $callbackMatch.Value 'MotorControl_RunCurrentLoop\s*\(' `
    'The ADC callback no longer executes the current inner loop.'
Assert-NotContains $callbackMatch.Value 'SpeedPi_Step\s*\(' `
    'Speed PI must run in the divided fast task, not the 10 kHz ADC callback.'

Assert-Contains $motorSource 'previous_current_reference\s*=\s*current_reference_active' `
    'Mode transitions must snapshot the active current reference before changing it.'
Assert-Contains $motorSource 'MotorRuntimePolicy_ClampSpeedIq\s*\(' `
    'Speed-mode entry and runtime limit changes must clamp Iq through the tested policy.'
Assert-Contains $motorSource 'speed_current_reference_target\.d\s*=\s*0\.0f' `
    'Speed mode must force its d-axis current target to zero.'
Assert-Contains $motorSource 'current_reference_active\.q\s*=\s*MotorRuntimePolicy_ClampSpeedIq' `
    'Reducing the speed Iq limit must also bound the active speed-mode current reference.'
Assert-Contains $encoderSource '#include\s+"biss_sequence\.h"' `
    'BiSS acquisition must use a freshness sequence that wraps safely.'
Assert-Contains $encoderSource 'BissSequence_Next\s*\(encoder_snapshot\.sequence\)' `
    'Every valid BiSS frame must advance the freshness sequence, including at UINT32_MAX.'
Assert-NotContains $encoderSource 'sequence\s*=\s*BissEncoder_IncrementSaturated' `
    'The BiSS freshness sequence must not saturate and freeze speed updates.'

Write-Host 'speed-mode integration checks passed'
