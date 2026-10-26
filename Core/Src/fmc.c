#include "fmc.h"

/* USER CODE BEGIN 0 */
SRAM_HandleTypeDef hsram4;

/* Register peripheral: never run HAL_NOR_Init/autoselect after regeneration. */
static void HbiFmcConfigure(void)
{
    FMC_NORSRAM_TimingTypeDef timing = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    clock.PeriphClockSelection = RCC_PERIPHCLK_FMC;
    clock.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) { Error_Handler(); }

    hsram4.Instance = FMC_NORSRAM_DEVICE;
    hsram4.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
    hsram4.Init.NSBank = FMC_NORSRAM_BANK4;
    hsram4.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_ENABLE;
    hsram4.Init.MemoryType = FMC_MEMORY_TYPE_NOR;
    hsram4.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram4.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
    hsram4.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram4.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
    hsram4.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
    hsram4.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
    hsram4.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
    hsram4.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram4.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
    hsram4.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
    hsram4.Init.WriteFifo = FMC_WRITE_FIFO_DISABLE;
    hsram4.Init.PageSize = FMC_PAGE_SIZE_NONE;
    timing.AddressSetupTime = 15;
    timing.AddressHoldTime = 15;
    timing.DataSetupTime = 63;
    timing.BusTurnAroundDuration = 15;
    timing.CLKDivision = 2;
    timing.DataLatency = 2;
    timing.AccessMode = FMC_ACCESS_MODE_A;
    if (HAL_SRAM_Init(&hsram4, &timing, &timing) != HAL_OK) { Error_Handler(); }
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef *handle)
{
    GPIO_InitTypeDef gpio = {0};
    if (handle->Instance != FMC_NORSRAM_DEVICE) return;
    __HAL_RCC_FMC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FMC;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
               GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 |
               GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio);
    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);
    gpio.Pin = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOG, &gpio);
}

/* USER CODE END 0 */

void MX_FMC_Init(void)
{
  /* USER CODE BEGIN FMC_Init 0 */
  HbiFmcConfigure();
  return; /* Skip any CubeMX-generated NOR flash probing below this block. */
  /* USER CODE END FMC_Init 0 */
}
