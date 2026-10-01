$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$ssc = Join-Path $root 'Middlewares/Third_Party/EtherCAT_SSC'
if (-not (Test-Path -LiteralPath $ssc -PathType Container)) { throw 'Missing SSC directory: Middlewares/Third_Party/EtherCAT_SSC' }
$config = Join-Path $ssc 'Inc/ecat_def.h'
if (-not (Test-Path -LiteralPath $config -PathType Leaf)) { throw 'Missing SSC configuration header: Inc/ecat_def.h' }
$files = @(Get-ChildItem -LiteralPath $ssc -File -Recurse | Where-Object { $_.Extension -in '.c','.h' })
$text = ($files | ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw }) -join "`n"
$required = @{ COE_SUPPORTED=1; MAILBOX_SUPPORTED=1; DC_SUPPORTED=0; AL_EVENT_ENABLED=0; ECAT_TIMER_INT=0; FOE_SUPPORTED=0; EOE_SUPPORTED=0; SOE_SUPPORTED=0; AOE_SUPPORTED=0; BOOTSTRAPMODE_SUPPORTED=0; ESC_EEPROM_ACCESS_SUPPORT=0 }
foreach ($macro in $required.Keys) {
    $value = $required[$macro]
    $definitions = @($files | ForEach-Object {
        $file = $_
        [regex]::Matches((Get-Content -LiteralPath $file.FullName -Raw), "(?m)^\s*#\s*define\s+$macro\s+([^\s/]+)") |
            ForEach-Object { [pscustomobject]@{ File=$file.FullName; Value=$_.Groups[1].Value } }
    })
    if ($definitions.Count -ne 1 -or $definitions[0].File -ne $config -or $definitions[0].Value -ne "$value") {
        throw "SSC macro $macro must have exactly one definition of $value in Inc/ecat_def.h."
    }
    if ($text -match "(?m)^\s*#\s*undef\s+$macro\b") { throw "SSC macro $macro must not be undefined." }
}
foreach ($item in 'stm32f4xx','SPI1\s*->\s*DR','GPIO_DeInit\s*\(\s*GPIOA\s*\)','oled\.h','beep\.h') {
    if ($text -match $item) { throw "Forbidden F407 reference in SSC integration: $item" }
}
Write-Output 'EtherCAT SSC integration contract passed'
