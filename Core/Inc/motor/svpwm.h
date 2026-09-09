/**
 * @file svpwm.h
 * @brief 与 MCU 外设无关的 dq 电压到三相 PWM 占空比变换。
 *
 * 输入输出均使用标幺值，不依赖母线电压、TIM8 ARR 或 HAL。后续电流环
 * 只需生成新的 MotorVoltageDq，仍可复用此模块；母线电压前馈和限幅策略
 * 可以在闭环阶段放在调用本函数之前。
 */
#ifndef MOTOR_SVPWM_H
#define MOTOR_SVPWM_H

typedef struct
{
    float ud_pu; /**< 转子同步坐标系 d 轴归一化电压。 */
    float uq_pu; /**< 转子同步坐标系 q 轴归一化电压。 */
} MotorVoltageDq;

typedef struct
{
    float phase_u; /**< U 相占空比，范围 [0.0, 1.0]。 */
    float phase_v; /**< V 相占空比，范围 [0.0, 1.0]。 */
    float phase_w; /**< W 相占空比，范围 [0.0, 1.0]。 */
} MotorPwmDuty;

/**
 * @brief 计算中心对齐 SVPWM 占空比。
 * @param voltage dq 标幺电压命令；过调制时函数会保持矢量方向并压缩幅值。
 * @param electrical_angle_pu 电角度，单位为一周，函数内部会环绕至 [0,1)。
 * @param duty 返回三相占空比；本函数不写任何定时器寄存器。
 * @note 输入或输出指针为 NULL 时函数直接返回。
 */
void Svpwm_Compute(const MotorVoltageDq *voltage,
                   float electrical_angle_pu,
                   MotorPwmDuty *duty);

#endif /* MOTOR_SVPWM_H */
