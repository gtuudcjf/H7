$ErrorActionPreference = 'Stop'
foreach ($check in @(
    'run_motor_telemetry_tests',
    'test_motor_telemetry_integration',
    'test_h7_current_config',
    'test_motor_control_integration',
    'test_biss_h7_config',
    'test_encoder_mode_integration',
    'test_speed_mode_integration',
    'test_position_mode_integration'
)) {
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "$check.ps1")
    if ($LASTEXITCODE -ne 0) { throw "$check failed" }
}
Write-Output 'motor telemetry suite: 8 checks passed (core + adapter + wiring + 6 regressions)'
