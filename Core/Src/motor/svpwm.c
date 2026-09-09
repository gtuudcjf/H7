/**
 * @file svpwm.c
 * @brief 纯 SVPWM 数学模块；保持与 STM32 HAL 解耦，方便后续电流环复用。
 *
 * 计算路径：dq --逆Park--> alpha/beta --逆Clarke--> 三相参考值
 * --共模注入--> 0～1占空比。这里采用 min/max 共模注入形式，效果等价于
 * 传统扇区判断法，但分支更少，便于在快速控制中断中稳定执行。
 */
#include "svpwm.h"

#include <math.h>

#define MOTOR_TWO_PI_F     (6.28318530717958647692f)  /**< 一周对应的弧度数。 */
#define MOTOR_SQRT3_BY_2_F (0.86602540378443864676f)  /**< sqrt(3)/2。 */

static float Svpwm_Clamp(float value, float low, float high)
{
    /* 最终保护：保证传给 PWM 层的占空比始终属于闭区间 [0,1]。 */
    if (value > high)
    {
        return high;
    }
    if (value < low)
    {
        return low;
    }
    return value;
}

static float Svpwm_WrapUnit(float value)
{
    /* 允许调用者传入累计多周或负方向电角度。 */
    value = fmodf(value, 1.0f);
    if (value < 0.0f)
    {
        value += 1.0f;
    }
    return value;
}

void Svpwm_Compute(const MotorVoltageDq *voltage,
                   float electrical_angle_pu,
                   MotorPwmDuty *duty)
{
    float angle;
    float sin_angle;
    float cos_angle;
    float alpha;
    float beta;
    float phase_u;
    float phase_v;
    float phase_w;
    float phase_max;
    float phase_min;
    float common_mode;
    float span;

    if ((voltage == 0) || (duty == 0))
    {
        return;
    }

    /* 三角函数使用弧度，因此先把标幺周转换为 [0,2*pi)。 */
    angle = Svpwm_WrapUnit(electrical_angle_pu) * MOTOR_TWO_PI_F;
    sin_angle = sinf(angle);
    cos_angle = cosf(angle);

    /* 逆 Park：dq 坐标系的电压命令转换为静止 αβ 坐标系。 */
    alpha = (voltage->ud_pu * cos_angle) - (voltage->uq_pu * sin_angle);
    beta = (voltage->ud_pu * sin_angle) + (voltage->uq_pu * cos_angle);

    /* 逆 Clarke：得到三相相电压的相对值。 */
    phase_u = alpha;
    phase_v = (-0.5f * alpha) + (MOTOR_SQRT3_BY_2_F * beta);
    phase_w = (-0.5f * alpha) - (MOTOR_SQRT3_BY_2_F * beta);

    /* 最大值和最小值既用于过调制判断，也用于计算零序共模分量。 */
    phase_max = fmaxf(phase_u, fmaxf(phase_v, phase_w));
    phase_min = fminf(phase_u, fminf(phase_v, phase_w));
    span = phase_max - phase_min;

    /*
     * 线性调制区的相间最大差不能超过 1。超出时等比例压缩，
     * 避免某一相单独饱和而使电压矢量方向发生突变。
     */
    if (span > 1.0f)
    {
        phase_u /= span;
        phase_v /= span;
        phase_w /= span;
        phase_max /= span;
        phase_min /= span;
    }

    /*
     * 注入 -(max+min)/2 的零序分量，把三相参考值在可用母线范围内居中；
     * 加 0.5 后得到以50%为零电压中心的占空比。
     */
    common_mode = 0.5f * (phase_max + phase_min);
    duty->phase_u = Svpwm_Clamp(0.5f + phase_u - common_mode, 0.0f, 1.0f);
    duty->phase_v = Svpwm_Clamp(0.5f + phase_v - common_mode, 0.0f, 1.0f);
    duty->phase_w = Svpwm_Clamp(0.5f + phase_w - common_mode, 0.0f, 1.0f);
}
