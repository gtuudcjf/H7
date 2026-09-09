/**
 * @file motor_control.c
 * @brief 开环控制编排层，不直接操作 TIM8 寄存器。
 *
 * 本层负责各模块的启动、停止和快速周期调用顺序。未来接入电流环时，
 * 可以把 OpenLoop_Step() 产生的 dq 电压替换为电流 PI 输出；SVPWM 和
 * Pwm3ph_ApplyDuty() 的接口无需变化。速度环则在更慢的周期生成 iq 目标值。
 */
#include "motor_control.h"

#include "drv8323_board.h"
#include "open_loop.h"
#include "pwm_3ph.h"
#include "svpwm.h"

/* 运行模式仅由本文件修改，防止中断在未完成初始化时执行控制计算。 */
static MotorControlMode motor_mode = MOTOR_CONTROL_STOPPED;

/* 开环角度、频率斜坡及 dq 电压命令的持久状态。 */
static OpenLoopState open_loop_state;

HAL_StatusTypeDef MotorControl_Init(const MotorControlConfig *config)
{
    /* 未提供配置时使用保守的默认电角频率变化率。 */
    float slew_hz_per_s = 20.0f;

    if (config != 0)
    {
        slew_hz_per_s = config->frequency_slew_hz_per_s;
    }

    /* 这里只完成句柄和引脚绑定，驱动仍保持 ENABLE=0。 */
    if (Drv8323Board_Init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 初始化算法状态和 50% 中性占空比，但尚不向功率级输出 PWM。 */
    OpenLoop_Init(&open_loop_state, slew_hz_per_s);
    motor_mode = MOTOR_CONTROL_STOPPED;
    return Pwm3ph_Init();
}

void MotorControl_SetOpenLoopCommand(float ud_pu,
                                     float uq_pu,
                                     float electrical_frequency_hz)
{
    /* 命令只写入开环状态，频率会在 FastTick 中按斜坡逐步逼近。 */
    OpenLoop_SetCommand(&open_loop_state, ud_pu, uq_pu, electrical_frequency_hz);
}

HAL_StatusTypeDef MotorControl_Start(void)
{
    HAL_StatusTypeDef status;

    /*
     * 启动顺序必须固定：先唤醒并配置 DRV8323，确认 SPI 过程无 HAL 错误，
     * 随后才允许 TIM8 六路 PWM 到达功率级。
     */
    status = Drv8323Board_EnableForPwm();
    if (status != HAL_OK)
    {
        return status;
    }

    /* PWM 六路均成功开启后，快速中断才允许更新开环占空比。 */
    status = Pwm3ph_Enable();
    if (status == HAL_OK)
    {
        motor_mode = MOTOR_CONTROL_OPEN_LOOP;
    }
    else
    {
        /* 部分 PWM 通道启动失败时，立即拉低 ENABLE，避免功率级状态不完整。 */
        Drv8323Board_Disable();
    }
    return status;
}

HAL_StatusTypeDef MotorControl_Stop(void)
{
    HAL_StatusTypeDef status;

    /* 先阻止快速中断继续改写 CCR，再执行硬件停机。 */
    motor_mode = MOTOR_CONTROL_STOPPED;

    /* 先撤去全部 INHx/INLx，再拉低 ENABLE，满足驱动芯片的安全停机顺序。 */
    status = Pwm3ph_Disable();
    Drv8323Board_Disable();
    return status;
}

void MotorControl_FastTick(float dt_s)
{
    MotorVoltageDq voltage;
    MotorPwmDuty duty;

    /* STOPPED 状态下中断仍可能到来，但不得计算或更新 PWM。 */
    if (motor_mode != MOTOR_CONTROL_OPEN_LOOP)
    {
        return;
    }

    /* 1. 更新带斜坡的电角频率、电角度，并取得当前 dq 电压命令。 */
    OpenLoop_Step(&open_loop_state, dt_s, &voltage);

    /* 2. 将 dq 电压和电角度转换为三相中心对齐 PWM 占空比。 */
    Svpwm_Compute(&voltage, open_loop_state.electrical_angle_pu, &duty);

    /* 3. 写入预装载 CCR；新占空比在 TIM8 更新事件边界同步生效。 */
    Pwm3ph_ApplyDuty(&duty);
}
