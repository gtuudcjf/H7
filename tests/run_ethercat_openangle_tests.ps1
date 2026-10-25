param([ValidateSet('All','Core','Policy')][string]$Part='All')
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$cc=Join-Path $root 'tmp/commit_verification_20261015/tool/tcc/tcc.exe'
if(-not(Test-Path -LiteralPath $cc)){$cc=(Get-Command tcc.exe -ErrorAction Stop).Source}
$out=Join-Path $root 'tmp/openangle_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
Push-Location $root
try {
    if($Part -in 'All','Core') {
        & $cc -Wall -Werror -I Core/Inc/ethercat tests/test_ethercat_openangle_core.c Core/Src/ethercat/ethercat_motor_core.c -o (Join-Path $out 'core.exe')
        if($LASTEXITCODE -ne 0){throw 'openangle core compile failed'}
        & (Join-Path $out 'core.exe'); if($LASTEXITCODE -ne 0){throw 'openangle core failed'}
    }
    if($Part -in 'All','Policy') {
        & $cc -Wall -Werror -I Core/Inc/motor tests/test_open_angle_experiment.c Core/Src/motor/open_angle_experiment.c -o (Join-Path $out 'policy.exe')
        if($LASTEXITCODE -ne 0){throw 'openangle experiment compile failed'}
        & (Join-Path $out 'policy.exe'); if($LASTEXITCODE -ne 0){throw 'openangle experiment failed'}
    }
    if($Part -eq 'All') {
        foreach($guardProfile in 0,1) {
            & $cc -Wall -Werror -DETHERCAT_BRINGUP_ONLY=0 -DETHERCAT_MOTOR_CONTROL=1 "-DETHERCAT_MOTOR_OPEN_ANGLE=$guardProfile" -I Core/Inc/motor tests/test_openangle_guard.c Core/Src/motor/motor_command_guard.c -o (Join-Path $out "guard$guardProfile.exe")
            if($LASTEXITCODE -ne 0){throw 'session guard compile failed'}
            & (Join-Path $out "guard$guardProfile.exe"); if($LASTEXITCODE -ne 0){throw 'session guard failed'}
        }
        foreach($power in 0,1) {
            & $cc -Wall -Werror -DETHERCAT_BRINGUP_ONLY=0 -DETHERCAT_MOTOR_CONTROL=1 -DETHERCAT_MOTOR_OPEN_ANGLE=1 "-DETHERCAT_MOTOR_POWER_ENABLED=$power" -I tests/stubs/motor_guard -I Core/Inc/motor -I Core/Inc/ethercat tests/test_ethercat_openangle_bridge.c Core/Src/ethercat/ethercat_motor_bridge.c Core/Src/ethercat/ethercat_motor_core.c Core/Src/motor/motor_safety.c Core/Src/motor/motor_command_guard.c Core/Src/motor/open_angle_experiment.c -o (Join-Path $out "bridge$power.exe")
            if($LASTEXITCODE -ne 0){throw 'openangle bridge compile failed'}
            & (Join-Path $out "bridge$power.exe");if($LASTEXITCODE -ne 0){throw 'openangle bridge behavior failed'}
        }
        & "$PSScriptRoot/test_openangle_motor_integration.ps1"
        foreach($openAngle in 0,1) {
            & $cc -Wall -Werror -DETHERCAT_BRINGUP_ONLY=0 -DETHERCAT_MOTOR_CONTROL=1 "-DETHERCAT_MOTOR_OPEN_ANGLE=$openAngle" -I tests/stubs/drv8323 -I Core/Inc/motor -I Core/Inc/ethercat tests/test_drv8323_startup_diagnostics.c Core/Src/motor/drv8323_board.c Core/Src/motor/drv8323.c -o (Join-Path $out "drv_startup$openAngle.exe")
            if($LASTEXITCODE -ne 0){throw 'DRV8323 startup diagnostic compile failed'}
            & (Join-Path $out "drv_startup$openAngle.exe");if($LASTEXITCODE -ne 0){throw 'DRV8323 startup diagnostic behavior failed'}
        }
    }
} finally {Pop-Location}
