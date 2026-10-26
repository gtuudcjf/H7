# Shared host C compiler discovery. Set NATIVE_CC to override local installation.
function Get-NativeCompiler {
    $candidates = @($env:NATIVE_CC)
    foreach ($name in 'tcc.exe','gcc.exe','clang.exe') {
        $found = Get-Command $name -ErrorAction SilentlyContinue
        if ($found) { $candidates += $found.Source }
    }
    $candidates += @((Join-Path $env:TEMP 'h7_vofa_tcc/tcc/tcc.exe'),
                     'D:/CodexTools/w64devkit-2.10.0/w64devkit/bin/gcc.exe')
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Leaf)) { return $candidate }
    }
    throw 'No host C compiler found. Set NATIVE_CC to gcc.exe, clang.exe or tcc.exe.'
}
