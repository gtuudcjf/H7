/**
 * @file drv8323_board.h
 * @brief H7 板级 DRV8323 绑定：SPI2、PC1(CS)、PC4(ENA)。
 *
 * 本文件是通用驱动与当前 PCB 之间的唯一连接点。PC5(DRV_CAL) 用于
 * 电流采样零偏校准；启动寄存器诊断在校准和 PWM 启动之前执行。
 */
#ifndef MOTOR_DRV8323_BOARD_H
#define MOTOR_DRV8323_BOARD_H

#include <stdbool.h>

#include "stm32h7xx_hal.h"
#include "../ethercat/ethercat_build.h"

#if ETHERCAT_MOTOR_OPEN_ANGLE
/* One foreground snapshot after configuration, before CAL/ADC/PWM startup.
 * reg[index] uses the register address 0x00..0x06 as index. Only valid_mask
 * entries are usable; read_status=0xffffffff means not attempted.
 * Read only when active=0. Stop preserves it; the next configure replaces it.
 * This is not a runtime fault log and does not drive control decisions. */
typedef struct {
    uint32_t session, active, configure_status;
    uint32_t attempted_mask, valid_mask, permission_lost;
    uint16_t reg[7];
    uint32_t read_status[7];
} Drv8323StartupDiagnostics;
extern volatile Drv8323StartupDiagnostics g_drv8323_startup_diagnostics;
#endif

/** @brief 绑定 SPI2、PC1 和 PC4，但不唤醒驱动、不发送 SPI 数据。 */
HAL_StatusTypeDef Drv8323Board_Init(void);

/** @brief 使用当前板级对象唤醒并配置 DRV8323 为 6x PWM 模式。 */
HAL_StatusTypeDef Drv8323Board_EnableForPwm(void);

/** @brief 使用当前板级对象拉低 PC4，使 DRV8323 禁止输出。 */
void Drv8323Board_Disable(void);

/**
 * @brief 控制 PC5/DRV_CAL，使三个电流采样放大器进入或退出零点校准。
 * @param enabled true 时拉高 DRV_CAL，false 时拉低。
 * @note 校准期间 TIM8 主输出必须保持关闭。
 */
void Drv8323Board_SetCurrentCalibration(bool enabled);

#endif /* MOTOR_DRV8323_BOARD_H */
