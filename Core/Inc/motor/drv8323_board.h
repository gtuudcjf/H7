/**
 * @file drv8323_board.h
 * @brief H7 板级 DRV8323 绑定：SPI2、PC1(CS)、PC4(ENA)。
 *
 * 本文件是通用驱动与当前 PCB 之间的唯一连接点。PC5(DRV_CAL) 在当前
 * 开环阶段保持低电平，留待移植电流采样零偏校准时使用。
 */
#ifndef MOTOR_DRV8323_BOARD_H
#define MOTOR_DRV8323_BOARD_H

#include "stm32h7xx_hal.h"

/** @brief 绑定 SPI2、PC1 和 PC4，但不唤醒驱动、不发送 SPI 数据。 */
HAL_StatusTypeDef Drv8323Board_Init(void);

/** @brief 使用当前板级对象唤醒并配置 DRV8323 为 6x PWM 模式。 */
HAL_StatusTypeDef Drv8323Board_EnableForPwm(void);

/** @brief 使用当前板级对象拉低 PC4，使 DRV8323 禁止输出。 */
void Drv8323Board_Disable(void);

#endif /* MOTOR_DRV8323_BOARD_H */
