$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$found = Get-Command tcc.exe -ErrorAction SilentlyContinue
$compiler = if ($found) { $found.Source } else {
    Join-Path $env:TEMP 'h7_vofa_tcc/tcc/tcc.exe'
}
if (-not (Test-Path -LiteralPath $compiler)) { throw 'TinyCC executable unavailable.' }
$exe = Join-Path $env:TEMP 'test_lan9252_board.exe'
Push-Location $root
try {
    & $compiler -Wall -Werror -I tests/stubs/lan9252 -I Core/Inc/ethercat tests/test_lan9252_board.c Core/Src/ethercat/lan9252_board.c Core/Src/ethercat/lan9252_spi.c -o $exe
    if ($LASTEXITCODE -ne 0) { throw "compile exit code $LASTEXITCODE" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "test exit code $LASTEXITCODE" }
} finally { Pop-Location }
