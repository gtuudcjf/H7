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
#include "biss_encoder.h"
#include "motor_control.h"
#include "motor_params.h"

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
 * 电流 PI 控制器的离散时间都会产生比例误差。
 */
#define MOTOR_FAST_TICK_S (MOTOR_CONTROL_PERIOD_S)

/*
 * 编译时启动模式只在这里选择：
 *   MOTOR_CONTROL_OPEN_VOLTAGE
 *       虚拟电角度 + 直接Ud/Uq，电流只监视、不参与调节；
 *   MOTOR_CONTROL_OPEN_ANGLE_CURRENT
 *       虚拟电角度 + Id/Iq电流PI；
 *   MOTOR_CONTROL_ENCODER_ANGLE_CURRENT
 *       编码器电角度 + Id/Iq电流PI，需要有效的编码器校准记录。
 *   MOTOR_CONTROL_ENCODER_SPEED_CURRENT
 *       编码器机械速度PI生成Iq，复用模式3的编码器电角度和10 kHz电流PI。
 *
 * 四种模式共用同一套启动、采样、保护和PWM输出框架。.mode只决定
 * “角度从哪里来”和“Ud/Uq由谁生成”。四套目标命令会在启动前全部预置，
 * 所以编译前只改.mode即可选择启动状态。运行中不要直接写本const结构，
 * 应调用MotorControl_SwitchTo...()，切换会在10 kHz控制边界统一生效。
 *
 * frequency_slew_hz_per_s只作用于模式1/2的虚拟角频率；模式3/4使用编码器
 * 角度。voltage_slew_pu_per_s用于从电流闭环退回模式1时平滑恢复开环电压。
 */
static const MotorControlConfig motor_config = {
  .mode = MOTOR_CONTROL_ENCODER_SPEED_CURRENT,
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
   * 建议阅读本工程时把以下三个调用看成三个不同层次：
   *   BissEncoder_Init()：只建立SPI4/DMA编码器采集对象；
   *   MotorControl_Init()：只初始化控制状态和软件模块，不启动功率级；
   *   MotorControl_Start()：按安全顺序启动ADC校准、驱动和实时中断。
   */
  /*
   * 只绑定 SPI4/DMA，不会立即产生编码器时钟。PE6 已在 MX_GPIO_Init()
   * 中先配置为低电平，因此编码器接口上电期间不会出现无意义发送。
   */
  if (BissEncoder_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }

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
   * 四种启动模式的命令全部预置，motor_config.mode决定实际使用哪一套。
   * 这些数值是当前24 V母线、当前电机和当前负载的实机验证值，不是通用值；
   * 更换电机/母线/负载时，必须结合motor_params.h从低电压、低电流重新验证：
   *   模式1：电压开环，Ud=0、Uq=0.08 pu、电角频率=1 Hz；
   *   模式2：虚拟角度+电流闭环，Id=0、Iq=0.8 A、电角频率=1 Hz；
   *   模式3：编码器角度+电流闭环，Id=0、Iq=0.6 A。
   * Id通常保持0；Iq的正负决定转矩方向。模式3不是位置环，不会保持目标角度。
   */
  MotorControl_SetOpenLoopCommand(0.0f, 0.08f, 1.0f);
  MotorControl_SetCurrentCommand(0.0f, 0.8f, 1.0f);
  MotorControl_SetEncoderCurrentCommand(0.0f, 0.6f);

  /* 模式4当前以50 rpm目标运行；速度环Iq上限保持0.7 A。 */
  MotorControl_SetSpeedCommand(50.0f);

	if (MotorControl_SetSpeedPiGains(
				MOTOR_SPEED_PI_KP_A_PER_RPM,
				MOTOR_SPEED_PI_KI_A_PER_RPM_S,
				MOTOR_SPEED_PI_KAW_PER_S) != HAL_OK)
	{
		Error_Handler();
	}
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
    /*
     * 处理编码器标定请求和停机后的 Flash 保存。该服务无忙等，但可能执行
     * Flash 擦写，所以只能放在主循环，禁止移动到 TIM8/ADC/SPI 中断。
     * 编码器校准由调试/通信命令显式调用 MotorControl_RequestEncoderCalibration()，
     * 上电不会自动校准或自动转动。
     */
    MotorControl_Service();
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI4)
  {
    /*
     * DMA已收满6字节；这里只解析、校验并发布编码器快照。
     * FOC不会直接读取DMA缓冲区，而是在控制边界获取一致的快照副本。
     */
    BissEncoder_OnTransferComplete();
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI4)
  {
    BissEncoder_OnTransferError(hspi->ErrorCode);
  }
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI4)
  {
    BissEncoder_OnAbortComplete();
  }
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
