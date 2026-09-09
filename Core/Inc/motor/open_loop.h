/**
 * @file open_loop.h
 * @brief 开环相角发生器；未来电流环接入后仍可保留作起动和调试用途。
 *
 * 电角度采用标幺周表示：0.0 对应 0 rad，0.5 对应 pi rad，1.0 与 0.0
 * 等价。采用标幺周可以直接使用 frequency_hz * dt_s 积分，避免在状态中
 * 重复保存 2*pi 系数。
 */
#ifndef MOTOR_OPEN_LOOP_H
#define MOTOR_OPEN_LOOP_H

#include "svpwm.h"

typedef struct
{
    float electrical_angle_pu;      /**< 当前电角度，范围 [0,1)。 */
    float electrical_frequency_hz; /**< 当前电角频率，可为负以反转方向。 */
    float target_frequency_hz;     /**< 目标电角频率。 */
    float frequency_slew_hz_per_s; /**< 频率斜坡，必须为正数。 */
    float ud_pu;                   /**< 恒定 d 轴电压命令。 */
    float uq_pu;                   /**< 恒定 q 轴电压命令。 */
} OpenLoopState;

/**
 * @brief 清零开环状态并设置频率斜坡上限。
 * @param state 开环状态对象。
 * @param frequency_slew_hz_per_s 频率变化率，传入负值时自动取绝对值。
 */
void OpenLoop_Init(OpenLoopState *state, float frequency_slew_hz_per_s);

/**
 * @brief 更新 dq 电压命令和目标电角频率，不立即改变当前频率和角度。
 */
void OpenLoop_SetCommand(OpenLoopState *state,
                         float ud_pu,
                         float uq_pu,
                         float electrical_frequency_hz);

/**
 * @brief 执行一个固定周期的频率斜坡与电角度积分。
 * @param state 开环状态对象。
 * @param dt_s 实际调用周期，单位秒且必须大于0。
 * @param voltage 返回当前 dq 电压命令，供 SVPWM 使用。
 */
void OpenLoop_Step(OpenLoopState *state, float dt_s, MotorVoltageDq *voltage);

#endif /* MOTOR_OPEN_LOOP_H */
