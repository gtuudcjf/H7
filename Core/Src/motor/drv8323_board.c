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
#if ETHERCAT_MOTOR_OPEN_ANGLE
#include "motor_safety.h"
volatile Drv8323StartupDiagnostics g_drv8323_startup_diagnostics;
#endif

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
    HAL_StatusTypeDef status;
#if ETHERCAT_MOTOR_OPEN_ANGLE
    g_drv8323_startup_diagnostics.active = 1U;
    ++g_drv8323_startup_diagnostics.session;
    g_drv8323_startup_diagnostics.configure_status = 0xFFFFFFFFU;
    g_drv8323_startup_diagnostics.attempted_mask = 0U;
    g_drv8323_startup_diagnostics.valid_mask = 0U;
    g_drv8323_startup_diagnostics.permission_lost = 0U;
    for (uint32_t address = 0U; address < 7U; ++address) {
        g_drv8323_startup_diagnostics.reg[address] = 0xFFFFU;
        g_drv8323_startup_diagnostics.read_status[address] = 0xFFFFFFFFU;
    }
#endif
    /* 将板级资源转交给通用驱动执行唤醒及寄存器配置。 */
    status = Drv8323_EnableAndConfigure6Pwm(&drv8323);
#if ETHERCAT_MOTOR_OPEN_ANGLE
    g_drv8323_startup_diagnostics.configure_status = (uint32_t)status;
    if (status == HAL_OK) {
        /* Foreground only: ADC/calibration/phase PWM have not started.
         * Read once, stop at the first transport failure, never repair/retry.
         * A failed diagnostic does not replace the original configure result. */
        for (uint32_t address = 0U; address < 7U; ++address) {
            uint16_t value;
            HAL_StatusTypeDef read_status;
            if (!MotorSafety_OutputPermitted()) break;
            g_drv8323_startup_diagnostics.attempted_mask |= 1UL << address;
            read_status = Drv8323_ReadRegister(&drv8323, (uint8_t)address, &value);
            g_drv8323_startup_diagnostics.read_status[address] = (uint32_t)read_status;
            if (read_status != HAL_OK) break;
            g_drv8323_startup_diagnostics.reg[address] = value;
            g_drv8323_startup_diagnostics.valid_mask |= 1UL << address;
        }
        /* The independent guard can revoke while a blocking read is running.
         * Preserve the existing fail-closed startup check after the added I/O. */
        if (!MotorSafety_OutputPermitted()) {
            g_drv8323_startup_diagnostics.permission_lost = 1U;
            Drv8323_Disable(&drv8323);
            status = HAL_ERROR;
        }
    }
    g_drv8323_startup_diagnostics.active = 0U;
#endif
    return status;
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
