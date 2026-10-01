$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$ssc = Join-Path $root 'Middlewares/Third_Party/EtherCAT_SSC'
if (-not (Test-Path -LiteralPath $ssc -PathType Container)) { throw 'Missing SSC directory: Middlewares/Third_Party/EtherCAT_SSC' }
$files = @(Get-ChildItem -LiteralPath $ssc -File -Recurse | Where-Object { $_.Extension -in '.c','.h' })
if ($files.Count -eq 0) { throw 'No SSC source files found.' }
$text = ($files | ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw }) -join "`n"
$required = @{ COE_SUPPORTED=1; MAILBOX_SUPPORTED=1; DC_SUPPORTED=0; AL_EVENT_ENABLED=0; ECAT_TIMER_INT=0; FOE_SUPPORTED=0; EOE_SUPPORTED=0; SOE_SUPPORTED=0; AOE_SUPPORTED=0; BOOTSTRAPMODE_SUPPORTED=0; ESC_EEPROM_ACCESS_SUPPORT=0 }
foreach ($macro in $required.Keys) {
    $value = $required[$macro]
    if ($text -notmatch "(?m)^\s*#\s*define\s+$macro\s+$value\b") { throw "SSC macro $macro must equal $value." }
}
foreach ($item in 'stm32f4xx','SPI1\s*->\s*DR','GPIO_DeInit\s*\(\s*GPIOA\s*\)','oled\.h','beep\.h') {
    if ($text -match $item) { throw "Forbidden F407 reference in SSC integration: $item" }
}
Write-Output 'EtherCAT SSC integration contract passed'
