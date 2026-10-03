$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$ssc = Join-Path $root 'Middlewares/Third_Party/EtherCAT_SSC'
if (-not (Test-Path -LiteralPath $ssc -PathType Container)) { throw 'Missing SSC directory: Middlewares/Third_Party/EtherCAT_SSC' }
$config = Join-Path $ssc 'Inc/ecat_def.h'
if (-not (Test-Path -LiteralPath $config -PathType Leaf)) { throw 'Missing SSC configuration header: Inc/ecat_def.h' }
$files = @(Get-ChildItem -LiteralPath $ssc -File -Recurse | Where-Object { $_.Extension -in '.c','.h' })
$text = ($files | ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw }) -join "`n"
$sources = @('ecatappl','ecatcoe','ecatslv','mailbox','objdef','sdoserv','coeappl')
$headers = @('applInterface','coeappl','ecat_def','ecatappl','ecatcoe','ecatslv','esc','mailbox','objdef','sdoserv','el9800hw')
foreach ($name in $sources) {
    if (-not (Test-Path -LiteralPath (Join-Path $ssc "Src/$name.c") -PathType Leaf)) { throw "Missing SSC core source: $name.c" }
}
foreach ($name in $headers) {
    if (-not (Test-Path -LiteralPath (Join-Path $ssc "Inc/$name.h") -PathType Leaf)) { throw "Missing SSC core header: $name.h" }
}
foreach ($file in (Get-ChildItem -LiteralPath $ssc -File -Recurse)) {
    if ($file.Extension -eq '.c' -and $file.BaseName -notin $sources) { throw "SSC source outside minimal whitelist: $($file.Name)" }
    if ($file.Extension -eq '.h' -and $file.BaseName -notin $headers) { throw "SSC header outside minimal whitelist: $($file.Name)" }
    if ($file.Name -match '(?i)TcHelper|testappl|sampleappl|cia402appl|foeappl|eoeappl|aoeappl|diag\.|~RF|\.TMP$|\.(dll|lib)$') { throw "Excluded SSC import: $($file.Name)" }
}
$required = @{ COE_SUPPORTED=1; MAILBOX_SUPPORTED=1; MAILBOX_QUEUE=0; SEGMENTED_SDO_SUPPORTED=0; COMPLETE_ACCESS_SUPPORTED=0; SDO_RES_INTERFACE=0; DIAGNOSIS_SUPPORTED=0; EMERGENCY_SUPPORTED=0; VOE_SUPPORTED=0; DC_SUPPORTED=0; AL_EVENT_ENABLED=0; ECAT_TIMER_INT=0; INTERRUPTS_SUPPORTED=0; USE_DEFAULT_MAIN=0; FOE_SUPPORTED=0; EOE_SUPPORTED=0; SOE_SUPPORTED=0; AOE_SUPPORTED=0; BOOTSTRAPMODE_SUPPORTED=0; ESC_EEPROM_ACCESS_SUPPORT=0; CONTROLLER_16BIT=0; CONTROLLER_32BIT=1; BIG_ENDIAN_FORMAT=0; BIG_ENDIAN_16BIT=0; MAX_MBX_SIZE='0x0080'; MAX_PD_OUTPUT_SIZE=2; MAX_PD_INPUT_SIZE=6 }

function Get-PossibleMacroDirectives([string]$path) {
    # Comments cannot define a macro. For unknown #if conditions, inspect both
    # branches: either branch could be selected by the firmware build.
    $source = Get-Content -LiteralPath $path -Raw
    $source = $source -replace '(?s)/\*.*?\*/', ' ' -replace '//[^\r\n]*', ''
    $frames = [System.Collections.ArrayList]::new()
    $possible = $true
    foreach ($line in ($source -split '\r?\n')) {
        if ($line -match '^\s*#\s*(?:if|ifdef|ifndef)\b') {
            $condition = if ($line -match '^\s*#\s*if\s+0\s*$') { 'false' }
                elseif ($line -match '^\s*#\s*if\s+1\s*$') { 'true' }
                else { 'unknown' }
            [void]$frames.Add(@{ Parent=$possible; TakenDefinite=($condition -eq 'true') })
            $possible = $possible -and ($condition -ne 'false')
            continue
        }
        if ($line -match '^\s*#\s*elif\b') {
            if ($frames.Count -eq 0) { throw "Unmatched #elif in $path" }
            $frame = $frames[$frames.Count - 1]
            $condition = if ($line -match '^\s*#\s*elif\s+0\s*$') { 'false' }
                elseif ($line -match '^\s*#\s*elif\s+1\s*$') { 'true' }
                else { 'unknown' }
            $possible = $frame.Parent -and -not $frame.TakenDefinite -and ($condition -ne 'false')
            if ($condition -eq 'true') { $frame.TakenDefinite = $true }
            continue
        }
        if ($line -match '^\s*#\s*else\b') {
            if ($frames.Count -eq 0) { throw "Unmatched #else in $path" }
            $frame = $frames[$frames.Count - 1]
            $possible = $frame.Parent -and -not $frame.TakenDefinite
            $frame.TakenDefinite = $true
            continue
        }
        if ($line -match '^\s*#\s*endif\b') {
            if ($frames.Count -eq 0) { throw "Unmatched #endif in $path" }
            $possible = $frames[$frames.Count - 1].Parent
            $frames.RemoveAt($frames.Count - 1)
            continue
        }
        if (-not $possible) { continue }
        if ($line -match '^\s*#\s*(define|undef)\s+([A-Za-z_][A-Za-z_0-9]*)\b\s*([^\s]*)') {
            [pscustomobject]@{ File=$path; Kind=$Matches[1]; Macro=$Matches[2]; Value=$Matches[3] }
        }
    }
    if ($frames.Count -ne 0) { throw "Unclosed preprocessor condition in $path" }
}

$directives = @($files | ForEach-Object { Get-PossibleMacroDirectives $_.FullName })
foreach ($macro in $required.Keys) {
    $value = $required[$macro]
    $definitions = @($directives | Where-Object { $_.Macro -eq $macro -and $_.Kind -eq 'define' })
    if (@($definitions | Where-Object { $_.File -eq $config -and $_.Value -eq "$value" }).Count -eq 0) {
        throw "SSC macro $macro must be defined as $value in active or possible Inc/ecat_def.h configuration."
    }
    if (@($definitions | Where-Object { $_.Value -ne "$value" }).Count -gt 0) {
        throw "SSC macro $macro has a possible contradictory definition."
    }
    if (@($directives | Where-Object { $_.Macro -eq $macro -and $_.Kind -eq 'undef' }).Count -gt 0) {
        throw "SSC macro $macro has a possible #undef."
    }
}
foreach ($item in 'stm32f4xx','SPI1\s*->\s*DR','GPIO_DeInit\s*\(\s*GPIOA\s*\)','oled\.h','beep\.h','TIM[28]\s*->','EXTI\d*_IRQn','\b(?:malloc|calloc|realloc|free)\s*\(') {
    if ($text -match $item) { throw "Forbidden F407 reference in SSC integration: $item" }
}
$identity = @{ VENDOR_ID='ETHERCAT_VENDOR_ID'; PRODUCT_CODE='ETHERCAT_PRODUCT_CODE'; REVISION_NUMBER='ETHERCAT_REVISION'; SERIAL_NUMBER='ETHERCAT_SERIAL_NUMBER'; DEVICE_NAME='ETHERCAT_DEVICE_NAME' }
foreach ($macro in $identity.Keys) {
    $definitions = @($directives | Where-Object { $_.Macro -eq $macro -and $_.Kind -eq 'define' })
    if ($definitions.Count -ne 1 -or $definitions[0].Value -ne $identity[$macro]) { throw "SSC identity must consume ethercat_identity.h: $macro" }
}
$hw = Get-Content -LiteralPath (Join-Path $ssc 'Inc/el9800hw.h') -Raw
foreach ($signature in 'UINT32\s+HW_GetTimer\s*\(\s*void\s*\)', 'BOOL\s+HW_AccessFault\s*\(\s*void\s*\)', 'void\s+HW_RecordSyncManTimeout\s*\(\s*UINT8\s+channel\s*\)') {
    if ($hw -notmatch $signature) { throw "Missing polling HW contract: $signature" }
}
if ($text -notmatch '#\s*include\s+"ethercat_identity\.h"') { throw 'SSC configuration must include the shared identity definitions.' }
if (-not (Test-Path -LiteralPath (Join-Path $ssc 'README.md'))) { throw 'SSC import needs source/patch attribution in README.md.' }
if ($hw -match 'lan9252_port\.h') { throw 'SSC hardware declarations must be usable before the Task 7 port header exists.' }
$appl = Get-Content -LiteralPath (Join-Path $ssc 'Src/ecatappl.c') -Raw

# Limit Task 6 to the reviewed application bindings and ARM compile fixes.
foreach ($unit in 'ecatappl', 'ecatslv', 'coeappl', 'ecatcoe') {
    $path = "Middlewares/Third_Party/EtherCAT_SSC/Src/$unit.c"
    $baseline = (& git -C $root show "2dbba1fd6b944284dfc1953be1491dbcdb034188:$path") -join "`n"
    if ($LASTEXITCODE -ne 0) { throw "Cannot read reviewed SSC baseline: $unit" }
    # Task 9 mechanically trims imported trailing whitespace/blank EOF lines.
    # Compare the same normalized text so substantive vendor deltas stay exact.
    $baseline = ($baseline -replace '(?m)[ \t]+$', '').TrimEnd("`n")
    $current = ((Get-Content -LiteralPath (Join-Path $root $path) -Raw).Replace("`r`n", "`n") -replace '(?m)[ \t]+$', '').TrimEnd("`n")
    if ($unit -eq 'ecatcoe') {
        $expected = $baseline.Replace('#include "ecat_def.h"', "#include `"ecat_def.h`"`n#include `"el9800hw.h`"`n#include `"ecatslv.h`"")
        # Final review length fix: validate before interpreting the CoE header.
        $expected = $expected.Replace("    UINT8 result = 0;`n", @"
    UINT8 result = 0;

    /* H743 patch: do not interpret an absent/partial CoE header. */
    if (SWAPWORD(pCoeMbx->MbxHeader.Length) < COE_HEADER_SIZE)
        return MBXERR_SIZETOOSHORT;

"@)
    } else {
        $expected = $baseline.Replace('#include "el9800appl.h"', '#include "ethercat_app.h"')
        if ([regex]::Matches($current, '#include "ethercat_app.h"').Count -ne 1) { throw "SSC $unit must bind the real application exactly once." }
        if ($unit -eq 'coeappl') {
            $expected = $expected -replace '    if\(ApplicationObjDic != NULL\)\n    \{\n        result = AddObjectsToObjDictionary\(\(TOBJECT OBJMEM \*\) ApplicationObjDic\);\n    \}', '    result = AddObjectsToObjDictionary((TOBJECT OBJMEM *) ApplicationObjDic);'
        }
    }
    if ($current -cne $expected) {
        throw "Task 6 SSC change outside the reviewed bindings/compile fixes: $unit"
    }
}

# Execute the real polling bodies without linking the application/port that
# later tasks own. The ESC and clock are the only simulated boundaries.
function Get-CFunction([string]$source, [string]$name) {
    $match = [regex]::Match($source, "(?m)^(?:void|UINT16)\s+$name\([^)]*\)\s*\{")
    if (-not $match.Success) { throw "Missing C function $name" }
    $depth = 1
    $end = $match.Index + $match.Length
    while ($depth -gt 0 -and $end -lt $source.Length) {
        if ($source[$end] -eq '{') { $depth++ }
        elseif ($source[$end] -eq '}') { $depth-- }
        $end++
    }
    if ($depth -ne 0) { throw "Unbalanced C function $name" }
    return $source.Substring($match.Index, $end - $match.Index)
}
$slave = Get-Content -LiteralPath (Join-Path $ssc 'Src/ecatslv.c') -Raw
$mainLoop = Get-CFunction $appl 'MainLoop'
$timerBlock = [regex]::Match($mainLoop, '(?s)#if !ECAT_TIMER_INT\s*(.*?)#endif')
if (-not $timerBlock.Success) { throw 'Missing non-interrupt polling timer block.' }
$found = Get-Command tcc.exe -ErrorAction SilentlyContinue
$compiler = if ($found) { $found.Source } else { Join-Path $env:TEMP 'h7_vofa_tcc/tcc/tcc.exe' }
if (-not (Test-Path -LiteralPath $compiler)) { throw 'TinyCC executable unavailable for SSC polling tests.' }
$testDirectory = Join-Path $env:TEMP ('ssc_polling_' + [guid]::NewGuid().ToString('N'))
[void](New-Item -ItemType Directory -Path $testDirectory)
$extracted = (Get-CFunction $slave 'DisableSyncManChannel') + "`n" + (Get-CFunction $slave 'EnableSyncManChannel') + "`nvoid PollTimer(void) {`n" + $timerBlock.Groups[1].Value + "`n}`n"
[IO.File]::WriteAllText((Join-Path $testDirectory 'ssc_polling_bodies.h'), $extracted)
$exe = Join-Path $testDirectory 'ssc_polling.exe'
& $compiler -Wall -Werror -I $testDirectory -I (Join-Path $ssc 'Inc') -I (Join-Path $root 'Core/Inc/ethercat') (Join-Path $PSScriptRoot 'test_ethercat_ssc_polling.c') -o $exe
if ($LASTEXITCODE -ne 0) { throw "SSC polling test compile exit code $LASTEXITCODE" }
& $exe
if ($LASTEXITCODE -ne 0) { throw "SSC polling test exit code $LASTEXITCODE" }
if ($appl -match 'HW_ClearTimer\s*\(') { throw 'Polling SSC timer must retain a monotonic microsecond counter.' }
if ($appl -notmatch 'sscTimerBaseline\s*=\s*HW_GetTimer\s*\(\s*\)') { throw 'MainInit must establish the polling timer baseline.' }
$sdoExe = Join-Path $testDirectory 'ssc_sdo.exe'
$coe = Get-Content -LiteralPath (Join-Path $ssc 'Src/coeappl.c') -Raw
$selectionStart = $slave.IndexOf('    SyncType0x1C32 = sSyncManOutPar.u16SyncType;')
$selectionEnd = $slave.IndexOf('    /* Update Cycle time entries if DC Sync Mode enabled */', $selectionStart)
if ($selectionStart -lt 0 -or $selectionEnd -lt 0) { throw 'Missing SSC synchronization selection block.' }
$selection = $slave.Substring($selectionStart, $selectionEnd - $selectionStart)
$syncBody = (Get-CFunction $coe 'COE_ObjInit') + @"

UINT16 SelectSync(UINT16 dcControl) {
    UINT16 SyncType0x1C32 = 0, SyncType0x1C33 = 0;
    UINT16 u16MinSuppSyncType = sSyncManOutPar.u16SyncTypesSupported & sSyncManInPar.u16SyncTypesSupported;
    UINT32 cycleTimeSync0 = 1000000, cycleTimeSync1 = 1000000;
    BOOL bSubordinatedCycles = FALSE;
$selection
    sSyncManOutPar.u16SyncType = SyncType0x1C32;
    sSyncManInPar.u16SyncType = SyncType0x1C33;
    return 0;
}
"@
[IO.File]::WriteAllText((Join-Path $testDirectory 'ssc_freerun_bodies.h'), $syncBody)
& $compiler -Wall -Werror -I $testDirectory -I (Join-Path $ssc 'Inc') -I (Join-Path $ssc 'Src') -I (Join-Path $root 'Core/Inc/ethercat') (Join-Path $PSScriptRoot 'test_ethercat_ssc_sdo.c') -o $sdoExe
if ($LASTEXITCODE -ne 0) { throw "SSC SDO test compile exit code $LASTEXITCODE" }
& $sdoExe
if ($LASTEXITCODE -ne 0) { throw "SSC SDO test exit code $LASTEXITCODE" }
foreach ($native in 'mailbox', 'objects') {
    $nativeExe = Join-Path $testDirectory "ssc_$native.exe"
    & $compiler -Wall -Werror -I (Join-Path $ssc 'Inc') -I (Join-Path $ssc 'Src') -I (Join-Path $root 'Core/Inc/ethercat') (Join-Path $PSScriptRoot "test_ethercat_ssc_$native.c") -o $nativeExe
    if ($LASTEXITCODE -ne 0) { throw "SSC $native test compile exit code $LASTEXITCODE" }
    & $nativeExe
    if ($LASTEXITCODE -ne 0) { throw "SSC $native test exit code $LASTEXITCODE" }
}
$lengthExe = Join-Path $testDirectory 'ssc_lengths.exe'
& $compiler -Wall -Werror -I (Join-Path $ssc 'Inc') -I (Join-Path $ssc 'Src') -I (Join-Path $root 'Core/Inc/ethercat') (Join-Path $PSScriptRoot 'test_ethercat_ssc_lengths.c') (Join-Path $root 'Core/Src/ethercat/ethercat_app.c') (Join-Path $root 'Core/Src/ethercat/ethercat_app_core.c') -o $lengthExe
if ($LASTEXITCODE -ne 0) { throw "SSC length-chain test compile exit code $LASTEXITCODE" }
& $lengthExe
if ($LASTEXITCODE -ne 0) { throw "SSC length-chain test exit code $LASTEXITCODE" }
$appExe = Join-Path $testDirectory 'ssc_application.exe'
& $compiler -Wall -Werror -DETHERCAT_APP_SSC_TEST -I (Join-Path $ssc 'Inc') -I (Join-Path $ssc 'Src') -I (Join-Path $root 'Core/Inc/ethercat') (Join-Path $PSScriptRoot 'test_ethercat_app.c') (Join-Path $root 'Core/Src/ethercat/ethercat_app_core.c') (Join-Path $root 'Core/Src/ethercat/ethercat_app.c') -o $appExe
if ($LASTEXITCODE -ne 0) { throw "SSC application test compile exit code $LASTEXITCODE" }
& $appExe
if ($LASTEXITCODE -ne 0) { throw "SSC application test exit code $LASTEXITCODE" }
Write-Output 'EtherCAT SSC integration contract passed'
