/**
 * @file current_pi.h
 * @brief d/q 轴电流 PI、电压矢量限幅与抗积分饱和。
 */
#ifndef CURRENT_PI_H
#define CURRENT_PI_H

#include <stdbool.h>

#include "foc_transform.h"

typedef struct
{
    float kp_v_per_a;
    float ki_v_per_a_s;
    float kaw_per_s;
    float output_limit_v;
} CurrentPiConfig;

typedef struct
{
    CurrentPiConfig config;
    float integrator_d_v;
    float integrator_q_v;
} CurrentPiController;

typedef struct
{
    FocDq error_a;
    FocDq proportional_v;
    FocDq integrator_v;
    FocDq unsaturated_v;
    FocDq voltage_v;
    bool saturated;
} CurrentPiResult;

bool CurrentPi_Init(CurrentPiController *controller, const CurrentPiConfig *config);
bool CurrentPi_SetGains(CurrentPiController *controller,
                        float kp_v_per_a,
                        float ki_v_per_a_s,
                        float kaw_per_s);
bool CurrentPi_SetOutputLimit(CurrentPiController *controller, float output_limit_v);
void CurrentPi_Reset(CurrentPiController *controller);

/**
 * @brief 预置积分器，使当前误差下的 PI 输出从给定电压开始。
 * @note 用于电压开环切换到电流闭环时降低电压阶跃。
 */
bool CurrentPi_PreloadOutput(CurrentPiController *controller,
                             const FocDq *reference_a,
                             const FocDq *feedback_a,
                             const FocDq *requested_voltage_v);

/**
 * @brief 执行一次 d/q 电流 PI。
 * @param sample_period_s 实际控制周期，必须为有限正数。
 */
bool CurrentPi_StepDq(CurrentPiController *controller,
                      const FocDq *reference_a,
                      const FocDq *feedback_a,
                      float sample_period_s,
                      CurrentPiResult *result);

#endif /* CURRENT_PI_H */
