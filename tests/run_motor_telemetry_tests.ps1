$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot "native_compiler.ps1")
$compiler = Get-NativeCompiler

$testExecutable = Join-Path $env:TEMP 'test_motor_telemetry_core.exe'
Push-Location $projectRoot
try {
    & $compiler -std=c99 -Wall -Werror -I Core/Inc/motor `
        tests/test_motor_telemetry_core.c `
        Core/Src/motor/motor_telemetry_core.c -o $testExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'Telemetry core compile failed.'
    }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'Telemetry core test failed.'
    }
    $adapterExecutable = Join-Path $env:TEMP 'test_motor_telemetry_adapter.exe'
    & $compiler -std=c99 -Wall -Werror -I tests/stubs/telemetry -I Core/Inc/motor `
        tests/test_motor_telemetry_adapter.c Core/Src/motor/motor_telemetry.c `
        Core/Src/motor/motor_telemetry_core.c -o $adapterExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry adapter compile failed.' }
    & $adapterExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry adapter test failed.' }
} finally {
    Pop-Location
}
