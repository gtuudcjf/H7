$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$paramsHeader = Get-Content -Raw (Join-Path $root 'Core/Inc/motor/motor_params.h')
$mainSource = Get-Content -Raw (Join-Path $root 'Core/Src/main.c')
$motorSource = Get-Content -Raw (Join-Path $root 'Core/Src/motor/motor_control.c')

if ($paramsHeader -notmatch '#define\s+MOTOR_SPEED_IQ_LIMIT_A\s+\(0\.7f\)')
{
    throw 'The current speed-mode configuration must use a 0.7 A Iq limit.'
}

if ($paramsHeader -notmatch '#define\s+MOTOR_SPEED_PI_KP_A_PER_RPM\s+\(0\.03f\)' -or
    $paramsHeader -notmatch '#define\s+MOTOR_SPEED_PI_KI_A_PER_RPM_S\s+\(0\.015f\)' -or
    $paramsHeader -notmatch '#define\s+MOTOR_SPEED_PI_KAW_PER_S\s+\(10\.0f\)')
{
    throw 'The current 50 rpm configuration must use the 0.03/0.015/10 speed PI gains.'
}

if ($mainSource -notmatch 'MotorControl_SetSpeedPiGains\s*\(\s*MOTOR_SPEED_PI_KP_A_PER_RPM\s*,\s*MOTOR_SPEED_PI_KI_A_PER_RPM_S\s*,\s*MOTOR_SPEED_PI_KAW_PER_S\s*\)')
{
    throw 'main.c must apply the speed PI gains from motor_params.h.'
}

if ($mainSource -notmatch '\.mode\s*=\s*MOTOR_CONTROL_ENCODER_SPEED_CURRENT')
{
    throw 'The 0.7 A trial must start in encoder speed/current mode.'
}

if ($mainSource -notmatch 'MotorControl_SetSpeedCommand\s*\(\s*50\.0f\s*\)')
{
    throw 'The current speed-mode configuration must use a 50 rpm target.'
}

if ($paramsHeader -notmatch '#define\s+MOTOR_SPEED_CURRENT_COMMAND_SLEW_A_PER_S\s+\(10\.0f\)')
{
    throw 'Speed mode must define a dedicated 10 A/s current-command slew rate.'
}

if ($motorSource -notmatch 'MotorControl_ModeUsesSpeedLoop\s*\(\s*motor_mode\s*\)[\s\S]{0,160}MOTOR_SPEED_CURRENT_COMMAND_SLEW_A_PER_S')
{
    throw 'All modes using the speed loop must select its dedicated current-command slew rate.'
}

if ($motorSource -notmatch 'MOTOR_CURRENT_COMMAND_SLEW_A_PER_S')
{
    throw 'Non-speed current modes must retain the original current-command slew rate.'
}

Write-Host '50 rpm speed-loop configuration checks passed'
