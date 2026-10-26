#ifndef HBI_TEST_HAL_H
#define HBI_TEST_HAL_H
#include <stdint.h>
#define ETHERCAT_BRINGUP_ONLY 1
typedef struct { uint32_t odr, mode[16], alternate[16], pull[16]; } GPIO_TypeDef;
extern GPIO_TypeDef ports[7];
#define GPIOA (&ports[0])
#define GPIOB (&ports[1])
#define GPIOC (&ports[2])
#define GPIOD (&ports[3])
#define GPIOE (&ports[4])
#define GPIOF (&ports[5])
#define GPIOG (&ports[6])
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_2 0x0004U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_5 0x0020U
#define GPIO_PIN_6 0x0040U
#define GPIO_PIN_7 0x0080U
#define GPIO_PIN_8 0x0100U
#define GPIO_PIN_9 0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_AF_PP 2
#define GPIO_MODE_ANALOG 3
#define GPIO_NOPULL 0
#define GPIO_SPEED_FREQ_LOW 0
#define GPIO_SPEED_FREQ_VERY_HIGH 3
#define GPIO_AF12_FMC 12
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
#define __HAL_RCC_FMC_CLK_ENABLE() ((void)0)
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
void HAL_GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *config);
void HAL_GPIO_WritePin(GPIO_TypeDef *port,uint32_t pin,unsigned value);
void Error_Handler(void);
#define HAL_OK 0
typedef int HAL_StatusTypeDef;
typedef struct { uint32_t PeriphClockSelection,FmcClockSelection; } RCC_PeriphCLKInitTypeDef;
#define RCC_PERIPHCLK_FMC 1
#define RCC_FMCCLKSOURCE_D1HCLK 0
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *clock);
typedef struct {
    uint32_t NSBank, DataAddressMux, MemoryType, MemoryDataWidth, BurstAccessMode;
    uint32_t WaitSignalPolarity, WaitSignalActive, WriteOperation, WaitSignal;
    uint32_t ExtendedMode, AsynchronousWait, WriteBurst, ContinuousClock, WriteFifo, PageSize;
} FMC_NORSRAM_InitTypeDef;
typedef struct {
    uint32_t AddressSetupTime,AddressHoldTime,DataSetupTime,BusTurnAroundDuration;
    uint32_t CLKDivision,DataLatency,AccessMode;
} FMC_NORSRAM_TimingTypeDef;
typedef struct { void *Instance, *Extended; FMC_NORSRAM_InitTypeDef Init; } SRAM_HandleTypeDef;
#define FMC_NORSRAM_DEVICE ((void*)1)
#define FMC_NORSRAM_EXTENDED_DEVICE ((void*)2)
#define FMC_NORSRAM_BANK4 6
#define FMC_DATA_ADDRESS_MUX_ENABLE 2
#define FMC_MEMORY_TYPE_NOR 8
#define FMC_NORSRAM_MEM_BUS_WIDTH_16 16
#define FMC_BURST_ACCESS_MODE_DISABLE 0
#define FMC_WAIT_SIGNAL_POLARITY_LOW 0
#define FMC_WAIT_TIMING_BEFORE_WS 0
#define FMC_WRITE_OPERATION_ENABLE 4096
#define FMC_WAIT_SIGNAL_DISABLE 0
#define FMC_EXTENDED_MODE_DISABLE 0
#define FMC_ASYNCHRONOUS_WAIT_DISABLE 0
#define FMC_WRITE_BURST_DISABLE 0
#define FMC_CONTINUOUS_CLOCK_SYNC_ONLY 0
#define FMC_WRITE_FIFO_DISABLE 2097152
#define FMC_PAGE_SIZE_NONE 0
#define FMC_ACCESS_MODE_A 0
HAL_StatusTypeDef HAL_SRAM_Init(SRAM_HandleTypeDef *h,FMC_NORSRAM_TimingTypeDef *r,FMC_NORSRAM_TimingTypeDef *w);
void HAL_SRAM_MspInit(SRAM_HandleTypeDef *h);
typedef struct { uint32_t CYCCNT,CTRL,LAR; } FakeDWT;
typedef struct { uint32_t DEMCR; } FakeCoreDebug;
extern FakeDWT fake_dwt;
extern FakeCoreDebug fake_core_debug;
extern uint32_t SystemCoreClock;
#define DWT (&fake_dwt)
#define CoreDebug (&fake_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 0x1000000
#define DWT_CTRL_CYCCNTENA_Msk 1
uint32_t HAL_GetTick(void);
void fake_dsb(void);
#define __DSB() fake_dsb()
#define __ISB() ((void)0)
void fake_nop(void);
#define __NOP() fake_nop()
typedef struct {
    uint32_t Enable,Number,BaseAddress,Size,SubRegionDisable,TypeExtField;
    uint32_t AccessPermission,DisableExec,IsShareable,IsCacheable,IsBufferable;
} MPU_Region_InitTypeDef;
#define MPU_REGION_ENABLE 1
#define MPU_REGION_NUMBER0 0
#define MPU_REGION_NUMBER1 1
#define MPU_REGION_SIZE_4GB 31
#define MPU_REGION_SIZE_64KB 15
#define MPU_TEX_LEVEL0 0
#define MPU_TEX_LEVEL2 2
#define MPU_REGION_NO_ACCESS 0
#define MPU_REGION_FULL_ACCESS 3
#define MPU_INSTRUCTION_ACCESS_DISABLE 1
#define MPU_ACCESS_SHAREABLE 1
#define MPU_ACCESS_NOT_SHAREABLE 0
#define MPU_ACCESS_NOT_CACHEABLE 0
#define MPU_ACCESS_NOT_BUFFERABLE 0
#define MPU_PRIVILEGED_DEFAULT 4
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *r);
void HAL_MPU_Enable(uint32_t flags);
#endif
