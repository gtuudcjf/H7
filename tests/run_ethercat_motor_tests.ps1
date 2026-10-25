$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$cc=(Get-Command tcc.exe -ErrorAction Stop).Source
$out=Join-Path $root 'tmp/ethercat_motor_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
Push-Location $root
try {
    $motor=Get-Content -LiteralPath 'Core/Src/motor/motor_control.c' -Raw -Encoding UTF8
    $start=$motor.IndexOf('HAL_StatusTypeDef MotorControl_Start(void)')
    $end=$motor.IndexOf('void MotorControl_GetFeedback(', $start)
    if($start -lt 0 -or $end -le $start){throw 'cannot extract production cleanup functions'}
    [IO.File]::WriteAllText((Join-Path $out 'motor_stop_under_test.inc'),$motor.Substring($start,$end-$start))
    & $cc -Wall -Werror -I Core/Inc/motor -I $out tests/test_motor_stop_cleanup.c Core/Src/motor/motor_runtime_policy.c -o (Join-Path $out 'stop.exe')
    if($LASTEXITCODE -ne 0){throw 'actual motor cleanup compilation failed'}
    & (Join-Path $out 'stop.exe')
    if($LASTEXITCODE -ne 0){throw 'actual motor cleanup behavior failed'}
    & $cc -Wall -Werror -I Core/Inc/motor tests/test_motor_runtime_policy.c Core/Src/motor/motor_runtime_policy.c -o (Join-Path $out 'policy.exe')
    if($LASTEXITCODE -ne 0){throw 'runtime policy compilation failed'}
    & (Join-Path $out 'policy.exe')
    if($LASTEXITCODE -ne 0){throw 'runtime policy behavior failed'}
    & $cc -Wall -Werror -I Core/Inc/ethercat tests/test_ethercat_motor_core.c Core/Src/ethercat/ethercat_motor_core.c -o (Join-Path $out 'core.exe')
    if($LASTEXITCODE -ne 0){throw 'motor core compilation failed'}
    & (Join-Path $out 'core.exe')
    if($LASTEXITCODE -ne 0){throw 'motor core behavior failed'}
    & $cc -Wall -Werror -I Core/Inc/motor tests/test_motor_command_guard.c Core/Src/motor/motor_command_guard.c -o (Join-Path $out 'guard.exe')
    if($LASTEXITCODE -ne 0){throw 'guard compilation failed'}
    & (Join-Path $out 'guard.exe')
    if($LASTEXITCODE -ne 0){throw 'guard behavior failed'}
    foreach($power in 0,1) {
        & $cc -Wall -Werror -DETHERCAT_BRINGUP_ONLY=0 -DETHERCAT_MOTOR_CONTROL=1 "-DETHERCAT_MOTOR_POWER_ENABLED=$power" -I tests/stubs/motor_guard -I Core/Inc/motor -I Core/Inc/ethercat tests/test_motor_safety.c Core/Src/motor/motor_safety.c Core/Src/motor/motor_command_guard.c -o (Join-Path $out "safety$power.exe")
        if($LASTEXITCODE -ne 0){throw 'safety compilation failed'}
        & (Join-Path $out "safety$power.exe")
        if($LASTEXITCODE -ne 0){throw 'safety behavior failed'}
        & $cc -Wall -Werror -DETHERCAT_BRINGUP_ONLY=0 -DETHERCAT_MOTOR_CONTROL=1 "-DETHERCAT_MOTOR_POWER_ENABLED=$power" -I tests/stubs/motor_guard -I Core/Inc/motor -I Core/Inc/ethercat tests/test_ethercat_motor_bridge.c Core/Src/ethercat/ethercat_motor_bridge.c Core/Src/ethercat/ethercat_motor_core.c Core/Src/motor/motor_safety.c Core/Src/motor/motor_command_guard.c -o (Join-Path $out "bridge$power.exe")
        if($LASTEXITCODE -ne 0){throw 'bridge compilation failed'}
        & (Join-Path $out "bridge$power.exe")
        if($LASTEXITCODE -ne 0){throw 'bridge behavior failed'}
    }
    & $cc -Wall -Werror -DETHERCAT_BRINGUP_ONLY=0 -DETHERCAT_MOTOR_CONTROL=1 -I Core/Inc/ethercat -I Middlewares/Third_Party/EtherCAT_SSC/Inc -I Middlewares/Third_Party/EtherCAT_SSC/Src tests/test_ethercat_motor_dictionary.c Core/Src/ethercat/ethercat_app.c Core/Src/ethercat/ethercat_app_core.c -o (Join-Path $out 'dictionary.exe')
    if($LASTEXITCODE -ne 0){throw 'motor dictionary compilation failed'}
    & (Join-Path $out 'dictionary.exe')
    if($LASTEXITCODE -ne 0){throw 'motor dictionary behavior failed'}
} finally { Pop-Location }
