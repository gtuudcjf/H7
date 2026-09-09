/**
 * @file test_open_loop.c
 * @brief SVPWM 与开环相角算法的宿主机 C99 单元测试。
 *
 * 运行示例（安装任意 C99 编译器后）：
 *   gcc -std=c99 -ICore/Inc/motor tests/test_open_loop.c \
 *       Core/Src/motor/svpwm.c Core/Src/motor/open_loop.c -lm -o test_open_loop
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "open_loop.h"
#include "svpwm.h"

#define FLOAT_EPSILON (0.0001f)

static void AssertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < FLOAT_EPSILON);
}

/* 防止零矢量被错误调制成非中性 PWM。 */
static void Test_SvpwmZeroVectorProducesNeutralDuty(void)
{
    const MotorVoltageDq voltage = {0.0f, 0.0f};
    MotorPwmDuty duty;

    Svpwm_Compute(&voltage, 0.37f, &duty);

    AssertNear(duty.phase_u, 0.5f);
    AssertNear(duty.phase_v, 0.5f);
    AssertNear(duty.phase_w, 0.5f);
}

/* 防止过调制时只夹紧单相、造成电压矢量方向突变。 */
static void Test_SvpwmOvermodulationStillReturnsValidDuty(void)
{
    const MotorVoltageDq voltage = {2.0f, -2.0f};
    MotorPwmDuty duty;

    Svpwm_Compute(&voltage, 0.25f, &duty);

    assert((duty.phase_u >= 0.0f) && (duty.phase_u <= 1.0f));
    assert((duty.phase_v >= 0.0f) && (duty.phase_v <= 1.0f));
    assert((duty.phase_w >= 0.0f) && (duty.phase_w <= 1.0f));
}

/* 防止正向频率跨越一周时电角度溢出而失去标幺范围。 */
static void Test_OpenLoopPositiveFrequencyWrapsAngle(void)
{
    OpenLoopState state;
    MotorVoltageDq voltage;

    OpenLoop_Init(&state, 1000.0f);
    OpenLoop_SetCommand(&state, 0.0f, 0.1f, 2.0f);
    state.electrical_angle_pu = 0.9f;
    OpenLoop_Step(&state, 0.1f, &voltage);

    AssertNear(state.electrical_angle_pu, 0.1f);
    AssertNear(voltage.ud_pu, 0.0f);
    AssertNear(voltage.uq_pu, 0.1f);
}

/* 防止反转时负电角度没有被环绕到 [0,1)。 */
static void Test_OpenLoopNegativeFrequencyWrapsAngle(void)
{
    OpenLoopState state;
    MotorVoltageDq voltage;

    OpenLoop_Init(&state, 1000.0f);
    OpenLoop_SetCommand(&state, 0.0f, 0.1f, -2.0f);
    state.electrical_angle_pu = 0.1f;
    OpenLoop_Step(&state, 0.1f, &voltage);

    AssertNear(state.electrical_angle_pu, 0.9f);
}

int main(void)
{
    Test_SvpwmZeroVectorProducesNeutralDuty();
    Test_SvpwmOvermodulationStillReturnsValidDuty();
    Test_OpenLoopPositiveFrequencyWrapsAngle();
    Test_OpenLoopNegativeFrequencyWrapsAngle();

    puts("open-loop algorithm tests passed");
    return 0;
}
