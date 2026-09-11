$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$spiSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\spi.c') -Raw
$spiHeader = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\spi.h') -Raw
$gpioSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\gpio.c') -Raw
$mainSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\main.c') -Raw
$irqSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Src\stm32h7xx_it.c') -Raw
$irqHeader = Get-Content -LiteralPath (Join-Path $projectRoot 'Core\Inc\stm32h7xx_it.h') -Raw
$ioc = Get-Content -LiteralPath (Join-Path $projectRoot 'emptytest.ioc') -Raw
$project = Get-Content -LiteralPath (Join-Path $projectRoot 'MDK-ARM\emptytest.uvprojx') -Raw

function Assert-Contains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        throw $Message
    }
}

function Assert-NotContains {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        throw $Message
    }
}

function Assert-FileExists {
    param([string]$RelativePath, [string]$Message)
    if (-not (Test-Path -LiteralPath (Join-Path $projectRoot $RelativePath))) {
        throw $Message
    }
}

Assert-FileExists 'Core\Inc\motor\biss_encoder.h' 'BiSS encoder transport header is missing.'
Assert-FileExists 'Core\Src\motor\biss_encoder.c' 'BiSS encoder transport source is missing.'

Assert-Contains $spiSource 'hspi4\.Init\.CLKPolarity\s*=\s*SPI_POLARITY_HIGH' `
    'SPI4 must idle high for BiSS-C.'
Assert-Contains $spiSource 'hspi4\.Init\.CLKPhase\s*=\s*SPI_PHASE_1EDGE' `
    'SPI4 must sample on the first falling edge.'
Assert-Contains $spiSource 'hspi4\.Init\.BaudRatePrescaler\s*=\s*SPI_BAUDRATEPRESCALER_128' `
    'SPI4 must start near 0.9375 MHz from the 120 MHz SPI45 clock.'
Assert-Contains $spiSource 'hspi4\.Init\.MasterKeepIOState\s*=\s*SPI_MASTER_KEEP_IO_STATE_ENABLE' `
    'SPI4 must hold SCK at its configured idle level between frames.'

Assert-Contains $spiHeader 'DMA_HandleTypeDef\s+hdma_spi4_rx' 'SPI4 RX DMA handle must be exported.'
Assert-Contains $spiHeader 'DMA_HandleTypeDef\s+hdma_spi4_tx' 'SPI4 TX DMA handle must be exported.'
Assert-Contains $spiSource 'DMA_REQUEST_SPI4_RX' 'SPI4 RX must use its DMAMUX request.'
Assert-Contains $spiSource 'DMA_REQUEST_SPI4_TX' 'SPI4 TX must use its DMAMUX request.'
Assert-Contains $spiSource '__HAL_LINKDMA\(spiHandle,\s*hdmarx,\s*hdma_spi4_rx\)' `
    'SPI4 RX DMA must be linked to the SPI handle.'
Assert-Contains $spiSource '__HAL_LINKDMA\(spiHandle,\s*hdmatx,\s*hdma_spi4_tx\)' `
    'SPI4 TX DMA must be linked to the SPI handle.'

Assert-Contains $irqSource 'void\s+DMA1_Stream0_IRQHandler\(void\)' 'SPI4 RX DMA IRQ is missing.'
Assert-Contains $irqSource 'HAL_DMA_IRQHandler\(&hdma_spi4_rx\)' 'RX IRQ must dispatch to HAL DMA.'
Assert-Contains $irqSource 'void\s+DMA1_Stream1_IRQHandler\(void\)' 'SPI4 TX DMA IRQ is missing.'
Assert-Contains $irqSource 'HAL_DMA_IRQHandler\(&hdma_spi4_tx\)' 'TX IRQ must dispatch to HAL DMA.'
Assert-Contains $irqHeader 'DMA1_Stream0_IRQHandler' 'RX DMA IRQ prototype is missing.'
Assert-Contains $irqHeader 'DMA1_Stream1_IRQHandler' 'TX DMA IRQ prototype is missing.'

Assert-Contains $gpioSource 'HAL_GPIO_WritePin\(ENC_TX_01_GPIO_Port,\s*ENC_TX_01_Pin,\s*GPIO_PIN_RESET\)' `
    'PE6 must be driven low before any SPI4 transaction.'
Assert-Contains $gpioSource 'ENC_TX_01_Pin[\s\S]*GPIO_MODE_OUTPUT_PP' `
    'PE6 must be a normal push-pull output.'
Assert-NotContains $spiSource 'GPIO_PIN_2\|GPIO_PIN_5\|GPIO_PIN_6[\s\S]{0,180}GPIO_AF5_SPI4' `
    'PE6 must not remain in the SPI4 alternate-function pin mask.'

Assert-Contains $mainSource 'HAL_SPI_TxRxCpltCallback[\s\S]*BissEncoder_OnTransferComplete' `
    'SPI4 DMA completion must be forwarded to the BiSS transport.'
Assert-Contains $mainSource 'HAL_SPI_ErrorCallback[\s\S]*BissEncoder_OnTransferError' `
    'SPI4 errors must be forwarded to the BiSS transport.'

Assert-Contains $ioc 'PE6\.Signal=GPIO_Output' 'CubeMX metadata must preserve PE6 as GPIO output.'
Assert-Contains $ioc 'SPI4\.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_128' `
    'CubeMX metadata must preserve the conservative SPI4 rate.'
Assert-Contains $ioc 'SPI4\.CLKPolarity=SPI_POLARITY_HIGH' 'CubeMX metadata must preserve CPOL high.'
Assert-Contains $ioc 'SPI4\.CLKPhase=SPI_PHASE_1EDGE' 'CubeMX metadata must preserve first-edge sampling.'

Assert-Contains $project '<FileName>biss_frame\.c</FileName>' 'Keil project is missing biss_frame.c.'
Assert-Contains $project '<FileName>biss_encoder\.c</FileName>' 'Keil project is missing biss_encoder.c.'
Assert-Contains $project '<FileName>encoder_angle\.c</FileName>' 'Keil project is missing encoder_angle.c.'
Assert-Contains $project '<FileName>crc32\.c</FileName>' 'Keil project is missing crc32.c.'
Assert-Contains $project '<FileName>motor_config_store\.c</FileName>' `
    'Keil project is missing motor_config_store.c.'
Assert-Contains $project '<IROM>[\s\S]*<Size>0x1E0000</Size>' `
    'Keil IROM must exclude Bank2 Sector7 reserved for calibration.'

$configHeader = Get-Content -LiteralPath `
    (Join-Path $projectRoot 'Core\Inc\motor\motor_config_store.h') -Raw
Assert-Contains $configHeader 'MOTOR_CONFIG_FLASH_ADDR\s+\(0x081E0000UL\)' `
    'Calibration address must be the start of H743 Bank2 Sector7.'

Write-Output 'BiSS-C SPI4/DMA configuration checks passed'
