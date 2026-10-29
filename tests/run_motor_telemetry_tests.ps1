$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compiler = $env:H7_HOST_CC
if (-not $compiler) {
    $localCompiler = Join-Path $projectRoot '.tools\host-cc\tcc\tcc.exe'
    if (Test-Path -LiteralPath $localCompiler -PathType Leaf) { $compiler = $localCompiler }
}
if (-not $compiler) { $compiler = (Get-Command tcc.exe -ErrorAction SilentlyContinue).Source }
if (-not $compiler -or -not (Get-Command $compiler -ErrorAction SilentlyContinue)) {
    throw "Host compiler not found: '$compiler'. Set H7_HOST_CC to an existing compiler, or install TinyCC at .tools\host-cc\tcc\tcc.exe."
}

$testExecutable = Join-Path $env:TEMP 'test_motor_telemetry_core.exe'
Push-Location $projectRoot
try {
    & $compiler -Wall -Werror -include tests/host_math_compat.h -I Core/Inc/motor `
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
    & $compiler -Wall -Werror -include tests/host_math_compat.h -I tests/stubs/telemetry -I Core/Inc/motor `
        tests/test_motor_telemetry_adapter.c Core/Src/motor/motor_telemetry.c `
        Core/Src/motor/motor_telemetry_core.c -o $adapterExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry adapter compile failed.' }
    & $adapterExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry adapter test failed.' }
} finally {
    Pop-Location
}
