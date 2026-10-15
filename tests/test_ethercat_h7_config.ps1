param([switch]$HardwareOnly)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

function Read-Required([string]$relative) {
    $path = Join-Path $root $relative
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Missing required file: $relative" }
    return Get-Content -LiteralPath $path -Raw -Encoding UTF8
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
$halConfig = Read-Required 'Core/Inc/stm32h7xx_hal_conf.h'
$ioc = Read-Required 'emptytest.ioc'
$project = Read-Required 'MDK-ARM/emptytest.uvprojx'
$mainHeader = Read-Required 'Core/Inc/main.h'

# These checks catch a motor call leaking into the selected bring-up branch,
# omitted startup steps, callbacks bypassing the gate, and missing Keil units.
function Function-Body([string]$text, [string]$name) {
    $match = [regex]::Match($text, "\b$name\s*\([^;{}]*\)\s*\{")
    if (-not $match.Success) { throw "Missing function definition: $name" }
    $start = $match.Index + $match.Length
    $depth = 1
    for ($position = $start; $position -lt $text.Length; $position++) {
        if ($text[$position] -eq '{') { $depth++ }
        if ($text[$position] -eq '}') { $depth-- }
        if ($depth -eq 0) { return $text.Substring($start, $position - $start) }
    }
    throw "Unbalanced function: $name"
}
function Preprocess-Main([int]$mode) {
    $compiler = 'D:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe'
    if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) { throw 'ARM preprocessor unavailable; safety branch check cannot be skipped.' }
    $arguments = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m7', '-E', '-P', "-DETHERCAT_BRINGUP_ONLY=$mode")
    [xml]$xml = $project
    $controls = $xml.Project.Targets.Target.TargetOption.TargetArmAds.Cads.VariousControls
    foreach ($define in ([string]$controls.Define -split ',')) {
        if ($define -and $define -notmatch '^ETHERCAT_BRINGUP_ONLY') { $arguments += "-D$define" }
    }
    foreach ($include in ([string]$controls.IncludePath -split ';')) {
        $path = if ([IO.Path]::IsPathRooted($include)) { $include } else { Join-Path (Join-Path $root 'MDK-ARM') $include }
        $arguments += @('-I', [IO.Path]::GetFullPath($path))
    }
    $output = & $compiler @arguments (Join-Path $root 'Core/Src/main.c')
    if ($LASTEXITCODE -ne 0) { throw "Main preprocessing failed for mode $mode" }
    return $output -join "`n"
}

if (-not $HardwareOnly) {
    Require $mainHeader '#ifndef\s+ETHERCAT_BRINGUP_ONLY\s*\r?\n#define\s+ETHERCAT_BRINGUP_ONLY\s+1\b' 'Bring-up must be the header default.'
    Require $main 'USER CODE BEGIN SysInit[\s\S]*?#if\s+ETHERCAT_BRINGUP_ONLY[\s\S]*?#else[\s\S]*?USER CODE END SysInit' 'Generated peripheral initialization must remain inside the USER CODE safety gate.'
    Require $main 'USER CODE BEGIN 2[\s\S]*?#endif[\s\S]*?USER CODE END 2' 'Startup gate must close inside USER CODE 2.'
    $active = Preprocess-Main 1
    $entry = Function-Body $active 'main'
    Require $entry 'MX_GPIO_Init\s*\(\s*\)\s*;[\s\S]*?HAL_GPIO_WritePin\s*\([^;]*0x0010[^;]*GPIO_PIN_RESET\s*\)[\s\S]*?HAL_GPIO_WritePin\s*\([^;]*0x0010[^;]*GPIO_PIN_SET\s*\)[\s\S]*?MX_SPI1_Init\s*\(\s*\)[\s\S]*?EtherCAT_Init\s*\(\s*\)' 'Active bring-up order must be GPIO, DRV low, CS high, SPI1, EtherCAT.'
    Require $main 'HAL_GPIO_WritePin\s*\(\s*DRV_ENA_GPIO_Port\s*,\s*DRV_ENA_Pin\s*,\s*GPIO_PIN_RESET\s*\)' 'Bring-up must explicitly keep DRV enable low.'
    Require $main 'HAL_GPIO_WritePin\s*\(\s*LAN9252_CS_GPIO_Port\s*,\s*LAN9252_CS_Pin\s*,\s*GPIO_PIN_SET\s*\)' 'Bring-up must explicitly keep LAN9252 CS high before SPI1.'
    Require $entry 'while\s*\(\s*1\s*\)[\s\S]*?EtherCAT_Service\s*\(\s*\)' 'Active bring-up loop must service EtherCAT.'
    Forbid $active '\b(?:MotorControl_|MotorTelemetry_|BissEncoder_)\w*\s*\(' 'Active main or IRQ callbacks can reach a motor/encoder/telemetry path.'
    Forbid $entry '\bMX_(?:ADC\d|TIM\d|SPI[234]|USART\d_UART|USB_DEVICE)_Init\s*\(|\bHAL_(?:ADC|TIM|UART|SPI)_\w*Start\w*\s*\(' 'Bring-up starts unnecessary or motor peripherals.'
    Forbid $entry 'Error_Handler\s*\(' 'LAN9252 failure must remain diagnosable rather than enter Error_Handler.'
    $allowedCalls = @('MPU_Config','HAL_Init','SystemClock_Config','MX_GPIO_Init','HAL_GPIO_WritePin','MX_SPI1_Init','EtherCAT_Init','EtherCAT_GetDiagnostics','LAN9252_BoardRunSpiDiagnostics','EtherCAT_Service','while')
    foreach ($call in [regex]::Matches($entry, '\b([A-Za-z_]\w*)\s*\(')) {
        if ($call.Groups[1].Value -notin $allowedCalls) { throw "Unexpected call in active bring-up main: $($call.Groups[1].Value)" }
    }
    Require $active 'volatile\s+EtherCATStatus\s+ethercat_init_status' 'Debugger init-status symbol is missing.'
    Require $active 'EtherCATDiagnostics\s*\*\s*volatile\s+ethercat_diagnostics' 'Debugger live-diagnostics pointer is missing.'
    $legacy = Function-Body (Preprocess-Main 0) 'main'
    Require $entry 'EtherCAT_Init\s*\(\s*\)[\s\S]*?EtherCAT_GetDiagnostics\s*\(\s*\)[\s\S]*?LAN9252_BoardRunSpiDiagnostics\s*\(\s*\)[\s\S]*?while' 'SPI comparison must run once after the original probe, outside the service loop.'
    Forbid $legacy 'LAN9252_BoardRunSpiDiagnostics\s*\(' 'Motor mode must not run the lab SPI comparison.'
    Require $legacy 'MX_GPIO_Init\s*\([\s\S]*?MX_ADC1_Init\s*\([\s\S]*?MX_SPI2_Init\s*\([\s\S]*?MX_SPI3_Init\s*\([\s\S]*?MX_SPI4_Init\s*\([\s\S]*?MX_TIM6_Init\s*\([\s\S]*?MX_TIM8_Init\s*\([\s\S]*?MX_USART1_UART_Init\s*\([\s\S]*?MX_USB_DEVICE_Init\s*\([\s\S]*?BissEncoder_Init\s*\([\s\S]*?MotorControl_Init\s*\([\s\S]*?MotorControl_SetOpenLoopCommand\s*\([\s\S]*?MotorControl_SetCurrentCommand\s*\([\s\S]*?MotorControl_SetEncoderCurrentCommand\s*\([\s\S]*?MotorControl_SetSpeedCommand\s*\([\s\S]*?MotorControl_SetPositionCommand\s*\([\s\S]*?MotorControl_SetSpeedPiGains\s*\([\s\S]*?MotorControl_Start\s*\([\s\S]*?MotorTelemetry_Init\s*\([\s\S]*?MotorControl_Service\s*\([\s\S]*?MotorTelemetry_Service\s*\(' 'Original motor startup/service order must stay available only in mode 0.'
    Forbid $legacy 'EtherCAT_(?:Init|Service)\s*\(' 'Legacy motor branch must not acquire EtherCAT startup/service.'
    foreach ($file in (Get-ChildItem (Join-Path $root 'Core/Src/ethercat') -Filter '*.c')) {
        Forbid (Get-Content $file.FullName -Raw -Encoding UTF8) 'MotorControl_|MotorTelemetry_|TIM8\s*->\s*CCR|DRV_ENA|motor_.*target' "EtherCAT source reaches motor output: $($file.Name)"
    }
    $usbReceive = Function-Body (Read-Required 'USB_DEVICE/App/usbd_cdc_if.c') 'CDC_Receive_FS'
    Forbid $usbReceive 'Motor|Command|Parse' 'USB RX must not dispatch motor commands.'
    Require $gpio 'HAL_GPIO_WritePin\s*\(\s*GPIOC\s*,\s*GPIO_PIN_1\|GPIO_PIN_4\|GPIO_PIN_5\s*,\s*GPIO_PIN_RESET\s*\)[\s\S]*?HAL_GPIO_Init\s*\(\s*GPIOC' 'DRV PC4 must be latched low before output setup.'
    Require $mainHeader '#define\s+DRV_ENA_Pin\s+GPIO_PIN_4\s*\r?\n#define\s+DRV_ENA_GPIO_Port\s+GPIOC' 'DRV enable safety pin must remain PC4.'
    Require $ioc '(?m)^PC4\.Signal=GPIO_Output\s*$' 'CubeMX must retain DRV PC4 as an output.'
    Require $ioc '(?m)^ProjectManager\.KeepUserCode=true\s*$' 'CubeMX must preserve the USER CODE safety gate.'
    [xml]$keil = $project
    $groups = $keil.Project.Targets.Target.Groups.Group
    $expectedGroups = @{
        'Application/User/EtherCAT' = @('lan9252_spi.c','lan9252_board.c','lan9252_port.c','ethercat_service.c','ethercat_app_core.c','ethercat_app.c')
        'Middlewares/EtherCAT_SSC' = @('coeappl.c','ecatappl.c','ecatcoe.c','ecatslv.c','mailbox.c','objdef.c','sdoserv.c')
    }
    foreach ($name in $expectedGroups.Keys) {
        $group = @($groups | Where-Object { $_.GroupName -eq $name })
        if ($group.Count -ne 1) { throw "Keil must contain exactly one $name group." }
        foreach ($unit in $expectedGroups[$name]) {
            $files = @($group[0].Files.File | Where-Object { $_.FileName -eq $unit })
            if ($files.Count -ne 1) { throw "Keil must contain exactly one $unit in $name." }
            $expectedDirectory = if ($name -eq 'Application/User/EtherCAT') { '../Core/Src/ethercat/' } else { '../Middlewares/Third_Party/EtherCAT_SSC/Src/' }
            if (([string]$files[0].FilePath).Replace('\','/') -ne "$expectedDirectory$unit") { throw "Wrong Keil source path: $unit" }
        }
    }
    Require $project 'Middlewares[\\/]Third_Party[\\/]EtherCAT_SSC[\\/]Inc' 'Keil needs SSC headers.'
    Forbid $project '407 LAN9252|STM32F407|Ethercat[\\/]Inc' 'Reference F407 source/include path must not enter Keil.'
}

function Read-ClockNumber([string]$text, [string]$pattern, [string]$name) {
    $match = [regex]::Match($text, $pattern)
    if (-not $match.Success) { throw "Missing generated clock setting: $name." }
    return [int64]$match.Groups[1].Value
}

Require $header 'extern\s+SPI_HandleTypeDef\s+hspi1\s*;' 'SPI1 handle is not exported.'
Require $header 'void\s+MX_SPI1_Init\s*\(void\)' 'MX_SPI1_Init prototype is missing.'
Require $spi 'SPI_HandleTypeDef\s+hspi1\s*;' 'SPI1 handle is missing.'
Require $spi 'void\s+MX_SPI1_Init\s*\(void\)' 'MX_SPI1_Init is missing.'
Require $spi 'hspi1\.Instance\s*=\s*SPI1\s*;' 'SPI1 instance is incorrect.'
Require $spi 'hspi1\.Init\.BaudRatePrescaler\s*=\s*SPI_BAUDRATEPRESCALER_16\s*;' 'SPI1 prescaler must be 16.'
Require $spi 'hspi1\.Init\.CLKPolarity\s*=\s*SPI_POLARITY_LOW\s*;' 'SPI1 must use CPOL low.'
Require $spi 'hspi1\.Init\.CLKPhase\s*=\s*SPI_PHASE_1EDGE\s*;' 'SPI1 must use first-edge sampling.'
$spi1Init = Function-Body $spi 'MX_SPI1_Init'
Require $spi1Init 'hspi1\.Init\.MasterKeepIOState\s*=\s*SPI_MASTER_KEEP_IO_STATE_ENABLE\s*;[\s\S]*?HAL_SPI_Init\s*\(\s*&hspi1\s*\)' 'SPI1 must retain IO state before HAL init so split LAN9252 frames survive HAL close/reopen.'
Forbid $spi1Init 'hspi1\.Init\.MasterKeepIOState\s*=\s*SPI_MASTER_KEEP_IO_STATE_DISABLE\b' 'SPI1 must not release IO between split LAN9252 transfers.'
Require $spi 'spiHandle->Instance\s*==\s*SPI1[\s\S]{0,700}Spi123ClockSelection\s*=\s*RCC_SPI123CLKSOURCE_PLL\s*;' 'SPI1 must select the 64 MHz SPI123 PLL kernel.'
Require $ioc '(?m)^SPI1\.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16\s*$' 'CubeMX SPI1 prescaler must be 16.'
Require $ioc '(?m)^SPI1\.CLKPolarity=SPI_POLARITY_LOW\s*$' 'CubeMX SPI1 must use CPOL low.'
Require $ioc '(?m)^SPI1\.CLKPhase=SPI_PHASE_1EDGE\s*$' 'CubeMX SPI1 must use first-edge sampling.'
Require $ioc '(?m)^SPI1\.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE\s*$' 'CubeMX must preserve SPI1 IO retention when code is regenerated.'
Require $ioc '(?m)^SPI1\.IPParameters=(?:[^,\r\n]+,)*MasterKeepIOState(?:,[^\r\n]*)?\s*$' 'CubeMX must track the SPI1 IO-retention parameter.'
Require $ioc '(?m)^RCC\.SPI123Freq_Value=64000000\s*$' 'SPI123 kernel must be 64 MHz for a 4 MHz SPI1 clock.'
Require $main 'RCC_OscInitStruct\.PLL\.PLLSource\s*=\s*RCC_PLLSOURCE_HSE\s*;' 'PLL1 must use the external HSE oscillator.'
Require $ioc '(?m)^RCC\.PLLSourceVirtual=RCC_PLLSOURCE_HSE\s*$' 'CubeMX PLL source must be HSE.'
$hseHz = Read-ClockNumber $halConfig '(?m)^\s*#\s*define\s+HSE_VALUE\s+\(?\s*(\d+)' 'HSE_VALUE'
$pllM = Read-ClockNumber $main 'RCC_OscInitStruct\.PLL\.PLLM\s*=\s*(\d+)\s*;' 'PLL1 M'
$pllN = Read-ClockNumber $main 'RCC_OscInitStruct\.PLL\.PLLN\s*=\s*(\d+)\s*;' 'PLL1 N'
$pllQ = Read-ClockNumber $main 'RCC_OscInitStruct\.PLL\.PLLQ\s*=\s*(\d+)\s*;' 'PLL1 Q'
$pllFrac = Read-ClockNumber $main 'RCC_OscInitStruct\.PLL\.PLLFRACN\s*=\s*(\d+)\s*;' 'PLL1 FRACN'
$iocFrac = Read-ClockNumber $ioc '(?m)^RCC\.PLLFRACN=(\d+)\s*$' 'CubeMX PLL1 FRACN'
if ($pllFrac -ne 0 -or $iocFrac -ne 0) {
    throw "SPI123 clock calculation requires PLL1 fractional mode disabled (generated FRACN=$pllFrac, CubeMX FRACN=$iocFrac)."
}
if ($pllM -eq 0 -or $pllQ -eq 0) { throw 'PLL1 M and Q divisors must be nonzero.' }
$spi123Hz = ([decimal]$hseHz / $pllM) * $pllN / $pllQ
if ($spi123Hz -ne 64000000) {
    throw "SPI123 PLL clock must be 64 MHz: HSE_VALUE=$hseHz / M=$pllM * N=$pllN / Q=$pllQ gives $spi123Hz Hz."
}
if (-not $HardwareOnly) {
    Require $main 'MX_SPI1_Init\s*\(\s*\)' 'main.c must initialize SPI1.'
}

Require $gpio 'HAL_GPIO_WritePin\s*\(\s*GPIOA\s*,\s*GPIO_PIN_4\s*,\s*GPIO_PIN_SET\s*\)' 'PA4 CS must start high.'
Require $gpio 'GPIO_PIN_4[\s\S]{0,240}GPIO_MODE_OUTPUT_PP[\s\S]{0,240}HAL_GPIO_Init\s*\(\s*GPIOA' 'PA4 must be push-pull output.'
Require $ioc '(?m)^PA4\.PinState=GPIO_PIN_SET\s*$' 'CubeMX PA4 default must be high.'
Require $ioc '(?m)^PA4\.Signal=GPIO_Output\s*$' 'CubeMX PA4 must be GPIO output.'
$spi1Pins = @{ PA5='SPI1_SCK'; PA6='SPI1_MISO'; PA7='SPI1_MOSI' }
foreach ($pin in $spi1Pins.Keys) {
    Require $ioc "(?m)^$pin\.Signal=$($spi1Pins[$pin])\s*`$" "$pin SPI1 assignment is incorrect."
}
Require $spi 'GPIO_InitStruct\.Pin\s*=\s*(?=[^;]*\bGPIO_PIN_5\b)(?=[^;]*\bGPIO_PIN_6\b)(?=[^;]*\bGPIO_PIN_7\b)[^;]+;[\s\S]{0,350}GPIO_InitStruct\.Alternate\s*=\s*GPIO_AF5_SPI1\s*;[\s\S]{0,120}HAL_GPIO_Init\s*\(\s*GPIOA\s*,\s*&GPIO_InitStruct\s*\)' 'PA5/6/7 must be initialized on GPIOA with SPI1 AF5.'
foreach ($pin in 0,1,3) {
    Require $ioc "(?m)^PE$pin\.Signal=GPIO_Input\s*`$" "PE$pin must be GPIO input."
    Forbid $ioc "(?m)^PE$pin\.Signal=GPIO_EXTI" "PE$pin must not use EXTI."
}
Require $gpio 'GPIO_InitStruct\.Pin\s*=\s*(?=[^;]*\bGPIO_PIN_0\b)(?=[^;]*\bGPIO_PIN_1\b)(?=[^;]*\bGPIO_PIN_3\b)[^;]+;[\s\S]{0,200}GPIO_InitStruct\.Mode\s*=\s*GPIO_MODE_INPUT\s*;[\s\S]{0,250}HAL_GPIO_Init\s*\(\s*GPIOE\s*,\s*&GPIO_InitStruct\s*\)' 'PE0/1/3 must be initialized as GPIOE inputs without EXTI.'
if (-not $HardwareOnly) {
    Require $project 'ETHERCAT_BRINGUP_ONLY=1' 'Keil target must define ETHERCAT_BRINGUP_ONLY=1.'
    Require $project 'Core[\\/]Inc[\\/]ethercat' 'Keil target needs the EtherCAT include path.'
    Require $project 'lan9252_spi\.c' 'Keil target is missing LAN9252 SPI source.'
    Require $project 'ethercat_service\.c' 'Keil target is missing EtherCAT service source.'
    Require $project 'lan9252_board\.c' 'Keil target is missing LAN9252 board source.'
}

foreach ($setting in @{
    Mode='SPI_MODE_MASTER'; Direction='SPI_DIRECTION_2LINES';
    DataSize='SPI_DATASIZE_8BIT'; FirstBit='SPI_FIRSTBIT_MSB'; NSS='SPI_NSS_SOFT'
}.GetEnumerator()) {
    Require $spi "hspi1\.Init\.$($setting.Key)\s*=\s*$($setting.Value)\s*;" "SPI1 $($setting.Key) is incorrect."
    Require $ioc "(?m)^SPI1\.$($setting.Key)=$($setting.Value)\s*`$" "CubeMX SPI1 $($setting.Key) is incorrect."
}
Require $ioc '(?m)^PA4\.GPIO_Speed=GPIO_SPEED_FREQ_MEDIUM\s*$' 'CubeMX PA4 must use medium speed.'
Require $ioc '(?m)^PA4\.GPIO_PuPd=GPIO_NOPULL\s*$' 'CubeMX PA4 must not use pull resistors.'
Require $gpio 'GPIO_PIN_4[\s\S]{0,240}GPIO_MODE_OUTPUT_PP[\s\S]{0,120}GPIO_NOPULL[\s\S]{0,120}GPIO_SPEED_FREQ_MEDIUM[\s\S]{0,120}HAL_GPIO_Init\s*\(\s*GPIOA' 'PA4 must use no pull and medium speed.'
Require $ioc 'ProjectManager\.functionlistsort=[^\r\n]*MX_SPI1_Init-SPI1' 'CubeMX initialization order must include SPI1.'
$board = Read-Required 'Core/Src/ethercat/lan9252_board.c'
$boardHeader = Read-Required 'Core/Inc/ethercat/lan9252_board.h'
Require $boardHeader 'const\s+LAN9252Transport\s*\*\s*LAN9252_BoardTransport\s*\(void\)' 'Board transport API is missing.'
Require $board 'HAL_SPI_TransmitReceive' 'Board adapter must use blocking full-duplex SPI.'
Forbid $board 'HAL_MAX_DELAY|HAL_SPI_\w+_(?:DMA|IT)\s*\(|__disable_irq\s*\(' 'Board adapter must use finite blocking calls without DMA/IRQ or global interrupt masking.'

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
