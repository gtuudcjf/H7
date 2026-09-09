/**
 * @file motor_control.h
 * @brief 电机控制统一入口。未来电流环和速度环只替换电压命令来源。
 *
 * 调用顺序：MotorControl_Init() -> 设置命令 -> MotorControl_Start()，随后
 * 由固定周期中断调用 MotorControl_FastTick()。硬件驱动、开环轨迹、SVPWM
 * 和 TIM8 分属不同模块，上层应用不直接访问这些模块的内部状态。
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32h7xx_hal.h"

typedef enum
{
    MOTOR_CONTROL_STOPPED = 0, /**< PWM 不参与控制，驱动应处于禁止状态。 */
    MOTOR_CONTROL_OPEN_LOOP    /**< 使用给定 dq 电压和积分电角度生成 PWM。 */
} MotorControlMode;

typedef struct
{
    MotorControlMode mode;          /**< 预留的初始模式字段；当前初始化始终从 STOPPED 开始。 */
    float frequency_slew_hz_per_s; /**< 开环电角频率变化率上限，单位 Hz/s。 */
} MotorControlConfig;

/**
 * @brief 初始化驱动绑定、开环状态和三相 PWM 中性占空比。
 * @param config 控制配置；为 NULL 时频率斜坡默认使用 20 Hz/s。
 * @return HAL_OK 初始化完成，否则返回板级驱动或 PWM 初始化错误。
 * @note 本函数不使能 DRV8323，也不启动 TIM8 PWM 输出。
 */
HAL_StatusTypeDef MotorControl_Init(const MotorControlConfig *config);

/**
 * @brief 设置开环 dq 电压与目标电角频率。
 * @param ud_pu d轴标幺电压。
 * @param uq_pu q轴标幺电压，通常用于产生转矩方向的旋转磁场。
 * @param electrical_frequency_hz 目标电角频率，负值表示反向旋转。
 * @note 电角频率不是机械转速；两者还与电机极对数有关。
 */
void MotorControl_SetOpenLoopCommand(float ud_pu,
                                     float uq_pu,
                                     float electrical_frequency_hz);

/**
 * @brief 先唤醒并配置驱动芯片，再开启 TIM8 六路 PWM。
 * @return HAL 状态；PWM 启动失败时会重新禁止驱动芯片。
 */
HAL_StatusTypeDef MotorControl_Start(void);

/**
 * @brief 停止控制，先关闭六路 PWM，再禁止驱动芯片。
 * @return PWM 停止过程返回的 HAL 状态。
 */
HAL_StatusTypeDef MotorControl_Stop(void);

/**
 * @brief 固定周期快速任务：开环更新 -> SVPWM -> 写入 TIM8 CCR。
 * @param dt_s 控制周期，单位秒；必须与实际中断频率一致。
 * @note 函数运行于高优先级中断上下文，禁止加入 HAL_Delay、阻塞 SPI/UART 等操作。
 */
void MotorControl_FastTick(float dt_s);

#endif /* MOTOR_CONTROL_H */
