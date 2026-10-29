$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent $PSScriptRoot
$paramsHeader = Get-Content -Raw (Join-Path $root 'Core/Inc/motor/motor_params.h')
$mainSource = Get-Content -Raw (Join-Path $root 'Core/Src/main.c')
$motorSource = Get-Content -Raw (Join-Path $root 'Core/Src/motor/motor_control.c')

if ($paramsHeader -notmatch '#define\s+MOTOR_SPEED_IQ_LIMIT_A\s+\(0\.7f\)')
{
    throw 'The current speed-mode configuration must use a 0.7 A Iq limit.'
}

if ($paramsHeader -notmatch '#define\s+MOTOR_SPEED_PI_KP_A_PER_RAD_S\s+\(0\.2864789f\)' -or
    $paramsHeader -notmatch '#define\s+MOTOR_SPEED_PI_KI_A_PER_RAD\s+\(0\.14323945f\)' -or
    $paramsHeader -notmatch '#define\s+MOTOR_SPEED_PI_KAW_PER_S\s+\(10\.0f\)')
{
    throw 'The current 5.2359878 rad/s configuration must use the 0.2864789/0.14323945/10 speed PI gains.'
}

if ($mainSource -notmatch 'MotorControl_SetSpeedPiGains\s*\(\s*MOTOR_SPEED_PI_KP_A_PER_RAD_S\s*,\s*MOTOR_SPEED_PI_KI_A_PER_RAD\s*,\s*MOTOR_SPEED_PI_KAW_PER_S\s*\)')
{
    throw 'main.c must apply the speed PI gains from motor_params.h.'
}

if ($mainSource -notmatch '\.mode\s*=\s*MOTOR_CONTROL_ENCODER_ANGLE_CURRENT' -or
    $mainSource -notmatch 'MotorControl_SetEncoderCurrentCommand\(0\.0f,\s*0\.0f\)')
{
    throw 'Default startup must remain encoder current control at zero Iq; speed tests use an explicit mode switch.'
}

if ($mainSource -notmatch 'MotorControl_SetSpeedCommand\s*\(\s*5\.2359878f\s*\)')
{
    throw 'The current speed-mode configuration must use a 5.2359878 rad/s target.'
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

Write-Host '5.2359878 rad/s speed-loop configuration checks passed'
