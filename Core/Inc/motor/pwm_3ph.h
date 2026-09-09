/**
 * @file pwm_3ph.h
 * @brief TIM8 三相主/互补 PWM 的硬件适配层。
 *
 * 本层是控制算法与 H7 定时器之间的边界：上层只传递 0.0～1.0 占空比，
 * 本层负责换算 CCR、成组启动主/互补输出和错误回滚。以后更换定时器或
 * 引脚时，只需要调整此模块及 CubeMX 外设配置。
 */
#ifndef MOTOR_PWM_3PH_H
#define MOTOR_PWM_3PH_H

#include "stm32h7xx_hal.h"
#include "svpwm.h"

/**
 * @brief 将三相 CCR 初始化为50%中性占空比，但不使能任何输出通道。
 */
HAL_StatusTypeDef Pwm3ph_Init(void);

/**
 * @brief 依次启动 TIM8 CH1/1N、CH2/2N、CH3/3N。
 * @return HAL_OK 六路全部启动；失败时回滚已经启动的通道。
 */
HAL_StatusTypeDef Pwm3ph_Enable(void);

/**
 * @brief 将占空比恢复到中性值，并按 CH3 到 CH1 的顺序停止全部输出。
 * @return HAL_OK 全部停止成功，否则返回遇到的第一个 HAL 错误。
 */
HAL_StatusTypeDef Pwm3ph_Disable(void);

/**
 * @brief 将 SVPWM 占空比限幅、换算并写入 TIM8 CCR1/CCR2/CCR3。
 * @param duty U/V/W 三相归一化占空比；NULL 输入被忽略。
 * @note TIM8 已启用 CCR 预装载，写入值在更新事件处同步装载。
 */
void Pwm3ph_ApplyDuty(const MotorPwmDuty *duty);

#endif /* MOTOR_PWM_3PH_H */
