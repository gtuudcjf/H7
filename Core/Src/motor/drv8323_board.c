/**
 * @file drv8323_board.c
 * @brief 将通用 DRV8323 驱动绑定到当前 H7 工程的已确认引脚。
 *
 * 当前硬件连接：SPI2_SCK=PI1、SPI2_MISO=PC2、SPI2_MOSI=PC3、
 * DRV_CS=PC1、DRV_ENA=PC4。片选由 GPIO 软件控制，不使用 SPI 硬件 NSS。
 */
#include "drv8323_board.h"

#include "drv8323.h"
#include "main.h"
#include "spi.h"

/*
 * 单电机板只有一个驱动芯片，因此板级对象保持为文件内静态实例。
 * 上层 motor_control.c 不需要知道 SPI 和 GPIO 细节。
 */
static Drv8323Device drv8323;

HAL_StatusTypeDef Drv8323Board_Init(void)
{
    /* 默认退出电流采样校准，避免正常运行时放大器输入仍被内部短接。 */
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port, DRV_CAL_Pin, GPIO_PIN_RESET);

    /* SPI2 和 GPIO 已由 main.c 中的 CubeMX 初始化函数先行配置。 */
    Drv8323_Bind(&drv8323,
                 &hspi2,
                 DRV_CS_GPIO_Port,
                 DRV_CS_Pin,
                 DRV_ENA_GPIO_Port,
                 DRV_ENA_Pin);
    return HAL_OK;
}

HAL_StatusTypeDef Drv8323Board_EnableForPwm(void)
{
    /* 将板级资源转交给通用驱动执行唤醒及寄存器配置。 */
    return Drv8323_EnableAndConfigure6Pwm(&drv8323);
}

void Drv8323Board_Disable(void)
{
    /* 这里只控制驱动使能；PWM 的停止由 motor_control.c 负责排序。 */
    Drv8323_Disable(&drv8323);
}

void Drv8323Board_SetCurrentCalibration(bool enabled)
{
    HAL_GPIO_WritePin(DRV_CAL_GPIO_Port,
                      DRV_CAL_Pin,
                      enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
