$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$compiler = (Get-Command tcc.exe -ErrorAction SilentlyContinue).Source

if (-not $compiler) {
    $archive = Join-Path $env:TEMP 'codex_position_tcc_20260922\tcc32.zip'
    $toolDirectory = Join-Path $env:TEMP 'h7_vofa_tcc'
    if (-not (Test-Path -LiteralPath $archive)) {
        throw 'TinyCC archive unavailable.'
    }
    Expand-Archive -LiteralPath $archive -DestinationPath $toolDirectory -Force
    $compiler = (Get-ChildItem -LiteralPath $toolDirectory -Filter tcc.exe -Recurse |
        Select-Object -First 1).FullName
}
if (-not $compiler) {
    throw 'TinyCC executable unavailable.'
}

$testExecutable = Join-Path $env:TEMP 'test_motor_telemetry_core.exe'
Push-Location $projectRoot
try {
    & $compiler -Wall -Werror -I Core/Inc/motor `
        tests/test_motor_telemetry_core.c `
        Core/Src/motor/motor_telemetry_core.c -o $testExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'Telemetry core compile failed.'
    }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) {
        throw 'Telemetry core test failed.'
    }
} finally {
    Pop-Location
}
