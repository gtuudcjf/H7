/**
 * @file open_loop.c
 * @brief 开环频率斜坡和标幺电角度积分。
 *
 * 本模块不读取编码器，也不使用电流反馈；它只生成一个连续旋转的电角度。
 * 因而适合无位置传感器的初始开环验证，但不能代替带负载下的闭环控制。
 */
#include "open_loop.h"

#include <math.h>

static float OpenLoop_WrapUnit(float value)
{
    /* fmodf 同时支持正向和反向累计；负余数需要再平移回 [0,1)。 */
    value = fmodf(value, 1.0f);
    if (value < 0.0f)
    {
        value += 1.0f;
    }
    return value;
}

void OpenLoop_Init(OpenLoopState *state, float frequency_slew_hz_per_s)
{
    if (state == 0)
    {
        return;
    }

    /* 从零角度、零频率和零电压开始，避免初始化时突加旋转矢量。 */
    state->electrical_angle_pu = 0.0f;
    state->electrical_frequency_hz = 0.0f;
    state->target_frequency_hz = 0.0f;
    /* 变化率只表示幅值，方向由目标频率的正负号决定。 */
    state->frequency_slew_hz_per_s = fabsf(frequency_slew_hz_per_s);
    state->ud_pu = 0.0f;
    state->uq_pu = 0.0f;
}

void OpenLoop_SetCommand(OpenLoopState *state,
                         float ud_pu,
                         float uq_pu,
                         float electrical_frequency_hz)
{
    if (state == 0)
    {
        return;
    }

    /* 电压命令立即更新；只有频率通过 Step() 中的斜坡逐渐变化。 */
    state->ud_pu = ud_pu;
    state->uq_pu = uq_pu;
    state->target_frequency_hz = electrical_frequency_hz;
}

void OpenLoop_Step(OpenLoopState *state, float dt_s, MotorVoltageDq *voltage)
{
    float max_frequency_step;
    float frequency_error;

    if ((state == 0) || (voltage == 0) || (dt_s <= 0.0f))
    {
        return;
    }

    /* 单次允许变化量 = Hz/s * s，单位为 Hz。 */
    max_frequency_step = state->frequency_slew_hz_per_s * dt_s;
    frequency_error = state->target_frequency_hz - state->electrical_frequency_hz;
    if (frequency_error > max_frequency_step)
    {
        state->electrical_frequency_hz += max_frequency_step;
    }
    else if (frequency_error < -max_frequency_step)
    {
        state->electrical_frequency_hz -= max_frequency_step;
    }
    else
    {
        state->electrical_frequency_hz = state->target_frequency_hz;
    }

    /*
     * 标幺电角度积分：一赫兹表示每秒前进一整周，所以无需乘 2*pi。
     * 结果环绕到 [0,1)，正负频率均能连续运行。
     */
    state->electrical_angle_pu = OpenLoop_WrapUnit(
        state->electrical_angle_pu + (state->electrical_frequency_hz * dt_s));

    /* 当前开环阶段 dq 电压不经 PI 调节，原样交给 SVPWM。 */
    voltage->ud_pu = state->ud_pu;
    voltage->uq_pu = state->uq_pu;
}
