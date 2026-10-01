$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function Read-Required([string]$relative) {
    $path = Join-Path $root $relative
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing required file: $relative" }
    return Get-Content -LiteralPath $path -Raw
}
function Require([string]$text, [string]$pattern, [string]$message) {
    if ($text -notmatch $pattern) { throw $message }
}
function Forbid([string]$text, [string]$pattern, [string]$message) {
    if ($text -match $pattern) { throw $message }
}

$spi = Read-Required 'Core/Src/spi.c'
$header = Read-Required 'Core/Inc/spi.h'
$gpio = Read-Required 'Core/Src/gpio.c'
$main = Read-Required 'Core/Src/main.c'
$ioc = Read-Required 'emptytest.ioc'
$project = Read-Required 'MDK-ARM/emptytest.uvprojx'

Require $header 'extern\s+SPI_HandleTypeDef\s+hspi1\s*;' 'SPI1 handle is not exported.'
Require $header 'void\s+MX_SPI1_Init\s*\(void\)' 'MX_SPI1_Init prototype is missing.'
Require $spi 'SPI_HandleTypeDef\s+hspi1\s*;' 'SPI1 handle is missing.'
Require $spi 'void\s+MX_SPI1_Init\s*\(void\)' 'MX_SPI1_Init is missing.'
Require $spi 'hspi1\.Instance\s*=\s*SPI1\s*;' 'SPI1 instance is incorrect.'
Require $spi 'hspi1\.Init\.BaudRatePrescaler\s*=\s*SPI_BAUDRATEPRESCALER_16\s*;' 'SPI1 prescaler must be 16.'
Require $ioc '(?m)^SPI1\.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16\s*$' 'CubeMX SPI1 prescaler must be 16.'
Require $main 'MX_SPI1_Init\s*\(\s*\)' 'main.c must initialize SPI1.'

Require $gpio 'HAL_GPIO_WritePin\s*\(\s*GPIOA\s*,\s*GPIO_PIN_4\s*,\s*GPIO_PIN_SET\s*\)' 'PA4 CS must start high.'
Require $gpio 'GPIO_PIN_4[\s\S]{0,240}GPIO_MODE_OUTPUT_PP[\s\S]{0,240}HAL_GPIO_Init\s*\(\s*GPIOA' 'PA4 must be push-pull output.'
Require $ioc '(?m)^PA4\.PinState=GPIO_PIN_SET\s*$' 'CubeMX PA4 default must be high.'
Require $ioc '(?m)^PA4\.Signal=GPIO_Output\s*$' 'CubeMX PA4 must be GPIO output.'
$spi1Pins = @{ PA5='SPI1_SCK'; PA6='SPI1_MISO'; PA7='SPI1_MOSI' }
foreach ($pin in $spi1Pins.Keys) {
    Require $ioc "(?m)^$pin\.Signal=$($spi1Pins[$pin])\s*`$" "$pin SPI1 assignment is incorrect."
}
Require $spi 'GPIO_PIN_5\|GPIO_PIN_6\|GPIO_PIN_7[\s\S]{0,250}GPIO_AF5_SPI1' 'PA5/6/7 must use SPI1 AF5.'
foreach ($pin in 0,1,3) {
    Require $ioc "(?m)^PE$pin\.Signal=GPIO_Input\s*`$" "PE$pin must be GPIO input."
    Forbid $ioc "(?m)^PE$pin\.Signal=GPIO_EXTI" "PE$pin must not use EXTI."
}
Require $project 'ETHERCAT_BRINGUP_ONLY=1' 'Keil target must define ETHERCAT_BRINGUP_ONLY=1.'
Require $project 'Core[\\/]Inc[\\/]ethercat' 'Keil target needs the EtherCAT include path.'
Require $project 'lan9252_spi\.c' 'Keil target is missing LAN9252 SPI source.'
Require $project 'ethercat_service\.c' 'Keil target is missing EtherCAT service source.'

$existing = @{ 2 = 32; 3 = 64; 4 = 128 }
foreach ($number in 2,3,4) {
    $rate = $existing[$number]
    Require $spi "hspi$number\.Instance\s*=\s*SPI$number\s*;" "SPI$number instance changed."
    Require $spi "hspi$number\.Init\.BaudRatePrescaler\s*=\s*SPI_BAUDRATEPRESCALER_$rate\s*;" "SPI$number prescaler changed."
    Require $ioc "(?m)^SPI$number\.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_$rate\s*`$" "CubeMX SPI$number prescaler changed."
}
$legacyPins = @{ PI1='SPI2_SCK'; PC2_C='SPI2_MISO'; PC3_C='SPI2_MOSI'; PC10='SPI3_SCK'; PC11='SPI3_MISO'; PC12='SPI3_MOSI'; PE2='SPI4_SCK'; PE5='SPI4_MISO'; PE6='GPIO_Output' }
foreach ($pin in $legacyPins.Keys) {
    $signal = $legacyPins[$pin]
    Require $ioc "(?m)^$pin\.Signal=$signal\s*`$" "$pin legacy assignment changed."
}
Write-Output 'EtherCAT H7 configuration contract passed'
