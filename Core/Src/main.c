/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/*
 * 快速控制周期必须与 TIM8 更新事件周期严格一致：
 * 240 MHz / (2 * (11999 + 1)) = 10 kHz，因此 dt = 1 / 10 kHz = 100 us。
 * 后续修改 TIM8 时钟或 ARR 时，必须同步更新此常量，否则电角度积分和
 * 未来 PI 控制器的离散时间都会产生比例误差。
 */
#define MOTOR_FAST_TICK_S (0.0001f)

/*
 * 当前只启用开环模式。20 Hz/s 表示目标电角频率每秒最多变化20 Hz，
 * 用于避免启动瞬间给定频率阶跃；mode 字段预留给后续闭环模式选择。
 */
static const MotorControlConfig motor_config = {
  /* 默认保持已经通过实机验证的电压开环。需要电流闭环时调用模式请求接口。 */
  .mode = MOTOR_CONTROL_OPEN_VOLTAGE,
  .frequency_slew_hz_per_s = 20.0f,
  .voltage_slew_pu_per_s = 5.0f
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  /*
   * MotorControl_Init() 只完成以下安全初始化，不向功率级输出 PWM：
   *   1. 将通用 DRV8323 对象绑定到 SPI2、PC1(CS)、PC4(ENA)；
   *   2. 清零开环角度和频率状态；
   *   3. 将 TIM8 CCR1/2/3 预置为50%中性占空比。
   */
  if (MotorControl_Init(&motor_config) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * 开环命令：Ud=0、Uq=0.08 pu、目标电角频率=1 Hz。
   * 低 q 轴电压和低电角频率仅用于无负载确认相序与转向；带电前必须结合
   * 母线电压、电机参数和负载重新评估。电角频率不是机械转速。
   */
  MotorControl_SetOpenLoopCommand(0.0f, 0.08f, 1.0f);

  /*
   * 角度开环、电流闭环的保守初值：Id=0 A、Iq=0.3 A、电角频率=1 Hz。
   * 上电仍进入电压开环；调试确认 ADC 偏置和电流方向后，可在主循环、通信
   * 命令或调试器中调用：
   *   MotorControl_RequestMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
   * 切回原开环则调用：
   *   MotorControl_RequestMode(MOTOR_CONTROL_OPEN_VOLTAGE);
   */
  MotorControl_SetCurrentCommand(0.0f, 0.3f, 1.0f);

  /*
   * 启动顺序由控制层保证：拉高 PC4 -> 等待 DRV8323 就绪 -> 写三个配置
   * 寄存器 -> 启动 TIM8 三路主 PWM 和三路互补 PWM。任一步失败均进入
   * Error_Handler()，且 PWM 启动失败时会重新拉低驱动使能。
   */
  if (MotorControl_Start() != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * MotorControl_Start() 已启动 ADC 注入中断、TIM8 基准和内部 CH4 采样触发。
   * 禁止在此重复启动 TIM8，否则 HAL 状态机会返回错误或破坏采样时序。
   */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*
   * HAL 可能把其他基础定时器的更新事件也分派到此回调，因此必须检查
   * Instance。只有 TIM8 更新事件属于电机10 kHz快速控制周期。
   */
  if (htim->Instance == TIM8)
  {
    /* 中断内仅执行确定时长的浮点运算和 CCR 写入，禁止阻塞外设访问。 */
    MotorControl_FastTick(MOTOR_FAST_TICK_S);
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  uint32_t phase_a_raw;
  uint32_t phase_b_raw;

  if (hadc->Instance != ADC1)
  {
    return;
  }

  /*
   * 注入序列顺序由 adc.c 固定：rank1=PB1/I_A，rank2=PB0/I_B。
   * 回调中只读取已经完成的 JDR，并把控制交给电机编排层；不执行阻塞操作。
   */
  phase_a_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
  phase_b_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
  MotorControl_CurrentSampleComplete(phase_a_raw, phase_b_raw, MOTOR_FAST_TICK_S);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
