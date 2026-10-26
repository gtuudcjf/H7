$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'native_compiler.ps1')
$compiler = Get-NativeCompiler
Push-Location $root
try {
    python tests/check_hbi_project.py
    if ($LASTEXITCODE -ne 0) { throw 'Project/ESI contract failed' }
    $output = Join-Path $root 'MDK-ARM/emptytest'
    [void](New-Item -ItemType Directory -Force -Path $output)
    # Compile the actual MPU implementation, alongside actual GPIO/FMC/board
    # source. Only the hardware registers and HAL boundary are simulated.
    $source = Get-Content Core/Src/main.c -Raw
    $match = [regex]::Match($source, '(?m)^void MPU_Config\(void\)\s*\{')
    if (-not $match.Success) { throw 'Missing MPU implementation' }
    $end = $match.Index + $match.Length; $depth = 1
    while ($depth -gt 0) {
        if ($source[$end] -eq '{') { $depth++ }
        elseif ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    [IO.File]::WriteAllText((Join-Path $output 'mpu_body.h'), $source.Substring($match.Index,$end-$match.Index))
    $tests = @(
        @{Name='test_lan9252_hbi'; Src=@('Core/Src/ethercat/lan9252_hbi.c'); Inc=@()},
        @{Name='test_hbi_board'; Src=@('Core/Src/gpio.c','Core/Src/fmc.c','Core/Src/ethercat/lan9252_board.c'); Inc=@('tests/stubs/hbi',$output)},
        @{Name='test_lan9252_port'; Src=@('Core/Src/ethercat/lan9252_port.c','Core/Src/ethercat/ethercat_service.c','Core/Src/ethercat/ethercat_app_core.c'); Inc=@('Middlewares/Third_Party/EtherCAT_SSC/Inc')},
        @{Name='test_ethercat_service'; Src=@('Core/Src/ethercat/ethercat_service.c','Core/Src/ethercat/ethercat_app_core.c'); Inc=@('tests/stubs/ethercat')}
    )
    foreach ($test in $tests) {
        $includeArgs = @()
        foreach ($path in $test.Inc) { $includeArgs += @('-I',$path) }
        $exe = Join-Path $env:TEMP ($test.Name+'.exe')
        & $compiler -std=c99 -Wall -Werror @includeArgs -I Core/Inc/ethercat "tests/$($test.Name).c" @($test.Src) -o $exe
        if ($LASTEXITCODE -ne 0) { throw "$($test.Name) compile failed" }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw "$($test.Name) failed" }
        if ($test.Name -eq 'test_hbi_board') {
            & $exe --stalled
            if ($LASTEXITCODE -ne 0) { throw 'Stopped timebase must fail initialization before HBI access' }
        }
    }
    & powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests/test_ethercat_ssc_integration.ps1
    if ($LASTEXITCODE -ne 0) { throw 'Real SSC integration failed' }
    Write-Output 'EtherCAT HBI suite: PASS'
} finally { Pop-Location }
