param([string]$UV4='D:/Keil_v5/UV4/UV4.exe', [switch]$SkipMotorCheck)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$mdk = Join-Path $root 'MDK-ARM'
$project = Join-Path $mdk 'emptytest.uvprojx'
$options = Join-Path $mdk 'emptytest.uvoptx'
$before = [IO.File]::ReadAllBytes($options)
$rte = Join-Path $mdk 'RTE/_emptytest/RTE_Components.h'
$rteBefore = [IO.File]::ReadAllBytes($rte)
$temporary = Join-Path $mdk 'hbi_motor_check.uvprojx'
if (Test-Path -LiteralPath $temporary) { throw "Temporary project already exists: $temporary" }
try {
    # Read and build the project without changing the user's target or options.
    python (Join-Path $PSScriptRoot 'test_hbi_startup.py')
    if ($LASTEXITCODE -ne 0) { throw 'ARM startup isolation check failed' }
    $builds = @()
    if (-not $SkipMotorCheck) {
        $text = [IO.File]::ReadAllText($project)
        $text = $text.Replace('ETHERCAT_BRINGUP_ONLY=1','ETHERCAT_BRINGUP_ONLY=0')
        $text = $text.Replace('<OutputDirectory>emptytest\</OutputDirectory>','<OutputDirectory>emptytest\motor-check\</OutputDirectory>')
        $text = $text.Replace('<OutputName>emptytest</OutputName>','<OutputName>motor-check</OutputName>')
        [IO.File]::WriteAllText($temporary,$text)
        $builds += @{Project=$temporary; Log=(Join-Path $mdk 'hbi-motor-build.log')}
    }
    # Finish with communication-only firmware so the default output is HBI.
    $builds += @{Project=$project; Log=(Join-Path $mdk 'hbi-build.log')}
    foreach ($build in $builds) {
        $arguments = '-r "'+$build.Project+'" -t emptytest -o "'+$build.Log+'"'
        $process = Start-Process -FilePath $UV4 -ArgumentList $arguments -WindowStyle Hidden -PassThru -Wait
        $log = Get-Content -LiteralPath $build.Log -Raw
        Get-Content -LiteralPath $build.Log -Tail 5
        if ($process.ExitCode -ne 0 -or $log -notmatch '0 Error\(s\), 0 Warning\(s\)') {
            throw "Keil build failed: $($build.Log) (exit $($process.ExitCode))"
        }
    }
    $map = Get-Content (Join-Path $mdk 'emptytest/emptytest.map') -Raw
    $symbols = $map.Substring($map.IndexOf('Image Symbol Table'))
    if ($symbols -match '(?m)^\s+MotorControl_(?:Start|Init)\s+0x') { throw 'Motor start/init linked into HBI firmware' }
    foreach ($name in 'EtherCAT_Init','MainLoop','HW_EscRead','board_read16','board_write16') {
        if ($symbols -notmatch "(?m)^\s+$name\s+0x") { throw "HBI map missing $name" }
    }
    $fromelf = Join-Path (Split-Path -Parent $UV4) '../ARM/ARMCC/bin/fromelf.exe'
    $disassembly = Join-Path $mdk 'emptytest/hbi-disassembly.txt'
    & $fromelf --text -c "--output=$disassembly" (Join-Path $mdk 'emptytest/emptytest.axf')
    if ($LASTEXITCODE -ne 0) { throw 'fromelf disassembly failed' }
    $assembly = Get-Content -LiteralPath $disassembly -Raw
    foreach ($entry in @(@{Name='board_read16'; Op='LDRH'},@{Name='board_write16'; Op='STRH'})) {
        $body = [regex]::Match($assembly, '(?ms)^\s{4}'+$entry.Name+'\r?\n(.*?)(?=^\s{4}[^ \t\r\n]|\z)').Groups[1].Value
        if ($body -notmatch $entry.Op -or $body -notmatch '\bDSB\b') { throw "Missing halfword/barrier instructions: $($entry.Name)" }
    }
    Get-FileHash -Algorithm SHA256 (Join-Path $mdk 'emptytest/emptytest.hex')
    Write-Output 'ARMCC full build and HBI map validation: PASS'
} finally {
    if ([Convert]::ToBase64String([IO.File]::ReadAllBytes($options)) -ne [Convert]::ToBase64String($before)) {
        [IO.File]::WriteAllBytes($options,$before)
    }
    # uVision changes the generated project-name comment for the temporary
    # motor-check project. Preserve the existing shared RTE header as well.
    if ([Convert]::ToBase64String([IO.File]::ReadAllBytes($rte)) -ne [Convert]::ToBase64String($rteBefore)) {
        [IO.File]::WriteAllBytes($rte,$rteBefore)
    }
    # Only remove explicitly named files made for this build, inside MDK-ARM.
    foreach ($name in 'hbi_motor_check.uvprojx','hbi_motor_check.uvoptx','hbi_motor_check.uvguix.tx') {
        $path = Join-Path $mdk $name
        if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path }
    }
}
