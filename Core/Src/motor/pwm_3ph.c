/**
 * @file pwm_3ph.c
 * @brief 仅封装 TIM8 输出，不包含电机控制算法。
 *
 * 通道对应关系由 CubeMX 固定：CH1/1N=U相、CH2/2N=V相、CH3/3N=W相。
 * TIM8 使用中心对齐模式，主输出和互补输出由同一个 CCR 控制；互补极性、
 * 死区和 MOE 均由高级定时器硬件处理，而不是由软件分别计算六个波形。
 */
#include "pwm_3ph.h"

#include "tim.h"

static float Pwm3ph_ClampDuty(float duty)
{
    /* 防止算法异常或过调制把 CCR 写到定时器有效范围之外。 */
    if (duty > 1.0f)
    {
        return 1.0f;
    }
    if (duty < 0.0f)
    {
        return 0.0f;
    }
    return duty;
}

static uint32_t Pwm3ph_DutyToCompare(float duty)
{
    /* ARR+1 是一个完整 PWM 周期对应的计数刻度数量。 */
    const uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(&htim8) + 1U;

    /*
     * duty=0 对应 CCR=0；duty=1 对应 CCR=ARR+1，在 PWM1 模式下形成
     * 100%有效电平。对输入先限幅，避免无符号转换产生回绕。
     */
    return (uint32_t)(Pwm3ph_ClampDuty(duty) * (float)period_counts);
}

static HAL_StatusTypeDef Pwm3ph_StartChannel(uint32_t channel)
{
    HAL_StatusTypeDef status;

    /* 先打开 CHx 主输出，HAL 会设置对应的 CCxE。 */
    status = HAL_TIM_PWM_Start(&htim8, channel);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 再打开 CHxN，HAL 会设置 CCxNE，并确保高级定时器 MOE 置位。 */
    return HAL_TIMEx_PWMN_Start(&htim8, channel);
}

static HAL_StatusTypeDef Pwm3ph_StopChannel(uint32_t channel)
{
    HAL_StatusTypeDef status;

    /* 停机时先撤去互补输出，再撤去同相主输出。 */
    status = HAL_TIMEx_PWMN_Stop(&htim8, channel);
    if (status != HAL_OK)
    {
        return status;
    }

    return HAL_TIM_PWM_Stop(&htim8, channel);
}

void Pwm3ph_ApplyDuty(const MotorPwmDuty *duty)
{
    if (duty == 0)
    {
        return;
    }

    /* U/V/W 的比较值分别映射到 TIM8 CCR1/CCR2/CCR3。 */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Pwm3ph_DutyToCompare(duty->phase_u));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Pwm3ph_DutyToCompare(duty->phase_v));
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Pwm3ph_DutyToCompare(duty->phase_w));
}

HAL_StatusTypeDef Pwm3ph_Init(void)
{
    const MotorPwmDuty neutral_duty = {0.5f, 0.5f, 0.5f};

    /*
     * PWM1 模式下50%是零共模偏置的中性调制点。此时仅写 CCR，CCxE、
     * CCxNE 和 MOE 尚未开启，所以初始化不会向功率级主动输出 PWM。
     */
    Pwm3ph_ApplyDuty(&neutral_duty);
    return HAL_OK;
}

HAL_StatusTypeDef Pwm3ph_Enable(void)
{
    HAL_StatusTypeDef status;

    /*
     * 按相依次启动。如果某一相失败，停止之前已经成功的相，保证上层不会
     * 在“只开启部分桥臂”的状态下继续运行。
     */
    status = Pwm3ph_StartChannel(TIM_CHANNEL_1);
    if (status != HAL_OK)
    {
        return status;
    }
    status = Pwm3ph_StartChannel(TIM_CHANNEL_2);
    if (status != HAL_OK)
    {
        (void)Pwm3ph_StopChannel(TIM_CHANNEL_1);
        return status;
    }
    status = Pwm3ph_StartChannel(TIM_CHANNEL_3);
    if (status != HAL_OK)
    {
        (void)Pwm3ph_StopChannel(TIM_CHANNEL_2);
        (void)Pwm3ph_StopChannel(TIM_CHANNEL_1);
    }
    return status;
}

HAL_StatusTypeDef Pwm3ph_Disable(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    HAL_StatusTypeDef channel_status;
    const MotorPwmDuty neutral_duty = {0.5f, 0.5f, 0.5f};

    /* 先撤销旋转电压矢量，再逐相关闭输出。 */
    Pwm3ph_ApplyDuty(&neutral_duty);

    /* 即使某一通道停止失败，也继续尝试关闭其余通道。 */
    channel_status = Pwm3ph_StopChannel(TIM_CHANNEL_3);
    if (channel_status != HAL_OK)
    {
        status = channel_status;
    }
    channel_status = Pwm3ph_StopChannel(TIM_CHANNEL_2);
    if ((status == HAL_OK) && (channel_status != HAL_OK))
    {
        status = channel_status;
    }
    channel_status = Pwm3ph_StopChannel(TIM_CHANNEL_1);
    if ((status == HAL_OK) && (channel_status != HAL_OK))
    {
        status = channel_status;
    }
    return status;
}
