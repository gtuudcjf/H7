$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$failures = @()
$pending = @()

foreach ($name in 'test_ethercat_h7_config','test_ethercat_identity','test_ethercat_ssc_integration','test_lan9252_board') {
    $script = Join-Path $PSScriptRoot "$name.ps1"
    try {
        & powershell.exe -NoProfile -ExecutionPolicy Bypass -File $script
        if ($LASTEXITCODE -ne 0) { throw "exit code $LASTEXITCODE" }
        Write-Output "$name`: PASS"
    } catch {
        $failures += $name
        Write-Output "$name`: FAIL ($_)"
    }
}

$nativeTests = @(
    @{ Name='test_lan9252_protocol'; Source='Core/Src/ethercat/lan9252_spi.c' },
    @{ Name='test_ethercat_app'; Source='Core/Src/ethercat/ethercat_app_core.c' }
)
$compiler = $null
foreach ($test in $nativeTests) {
    $name = $test.Name
    $testSource = Join-Path $PSScriptRoot "$name.c"
    if (-not (Test-Path -LiteralPath $testSource -PathType Leaf)) {
        $pending += $name
        Write-Output "$name`: PENDING (test source absent)"
        continue
    }
    try {
        if (-not $compiler) {
            $found = Get-Command tcc.exe -ErrorAction SilentlyContinue
            if ($found) { $compiler = $found.Source }
            if (-not $compiler) {
                $archive = Join-Path $env:TEMP 'codex_position_tcc_20260922/tcc32.zip'
                $toolDirectory = Join-Path $env:TEMP 'h7_vofa_tcc'
                if (-not (Test-Path -LiteralPath $archive -PathType Leaf)) { throw 'TinyCC archive unavailable.' }
                Expand-Archive -LiteralPath $archive -DestinationPath $toolDirectory -Force
                $compiler = (Get-ChildItem -LiteralPath $toolDirectory -Filter tcc.exe -Recurse |
                    Select-Object -First 1).FullName
                if (-not $compiler) { throw 'TinyCC executable unavailable.' }
            }
        }
        # Let the compiler diagnose missing production headers/sources as well
        # as C errors, so the native branch supports a real test-first run.
        $exe = Join-Path $env:TEMP "$name.exe"
        Push-Location $projectRoot
        try {
            & $compiler -Wall -Werror -I Core/Inc/ethercat "tests/$name.c" $test.Source -o $exe
            if ($LASTEXITCODE -ne 0) { throw "compile exit code $LASTEXITCODE" }
            & $exe
            if ($LASTEXITCODE -ne 0) { throw "test exit code $LASTEXITCODE" }
        } finally {
            Pop-Location
        }
        Write-Output "$name`: PASS"
    } catch {
        $failures += $name
        Write-Output "$name`: FAIL ($_)"
    }
}

Write-Output "EtherCAT suite: $($failures.Count) failed, $($pending.Count) pending."
if ($failures.Count -gt 0) { exit 1 }
exit 0
