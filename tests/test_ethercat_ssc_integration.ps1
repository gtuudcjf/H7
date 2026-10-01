$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$ssc = Join-Path $root 'Middlewares/Third_Party/EtherCAT_SSC'
if (-not (Test-Path -LiteralPath $ssc -PathType Container)) { throw 'Missing SSC directory: Middlewares/Third_Party/EtherCAT_SSC' }
$config = Join-Path $ssc 'Inc/ecat_def.h'
if (-not (Test-Path -LiteralPath $config -PathType Leaf)) { throw 'Missing SSC configuration header: Inc/ecat_def.h' }
$files = @(Get-ChildItem -LiteralPath $ssc -File -Recurse | Where-Object { $_.Extension -in '.c','.h' })
$text = ($files | ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw }) -join "`n"
$required = @{ COE_SUPPORTED=1; MAILBOX_SUPPORTED=1; DC_SUPPORTED=0; AL_EVENT_ENABLED=0; ECAT_TIMER_INT=0; FOE_SUPPORTED=0; EOE_SUPPORTED=0; SOE_SUPPORTED=0; AOE_SUPPORTED=0; BOOTSTRAPMODE_SUPPORTED=0; ESC_EEPROM_ACCESS_SUPPORT=0 }

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
foreach ($item in 'stm32f4xx','SPI1\s*->\s*DR','GPIO_DeInit\s*\(\s*GPIOA\s*\)','oled\.h','beep\.h') {
    if ($text -match $item) { throw "Forbidden F407 reference in SSC integration: $item" }
}
Write-Output 'EtherCAT SSC integration contract passed'
