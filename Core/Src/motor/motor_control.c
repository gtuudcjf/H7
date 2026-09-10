/**
 * @file motor_control.c
 * @brief H7 电机控制编排：校准、开环、电流环、模式切换和故障处理。
 *
 * 快速路径不执行阻塞式 SPI、延时或日志输出。TIM8 更新中断只负责统一
 * 电角度时基和电压开环；ADC 注入完成中断只负责采样监视和电流闭环。
 */
#include "motor_control.h"

#include "adc.h"
#include "current_pi.h"
#include "current_sense.h"
#include "drv8323_board.h"
#include "foc_transform.h"
#include "motor_params.h"
#include "open_loop.h"
#include "pwm_3ph.h"
#include "svpwm.h"
#include "tim.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define CURRENT_CALIBRATION_DISCARD_COUNT (16U)
#define CURRENT_CALIBRATION_SAMPLE_COUNT  (256U)
#define CURRENT_CALIBRATION_RAIL_MARGIN   (512.0f)
#define CURRENT_CALIBRATION_MAX_SPAN      (64U)
#define CURRENT_ADC_RUNTIME_RAIL_MARGIN   (32U)
#define CURRENT_OVERCURRENT_CONFIRM_COUNT (2U)
#define CURRENT_ADC_TIMEOUT_TICKS         (3U)
#define CURRENT_ALIGNMENT_TIME_S          (0.75f)
#define CURRENT_PI_KP_MAX_V_PER_A         (2.0f)
#define CURRENT_PI_KI_MAX_V_PER_A_S       (2000.0f)
#define CURRENT_PI_KAW_MAX_PER_S          (2000.0f)

volatile MotorControlDebug g_motor_control_debug;

static MotorControlConfig motor_config;
static MotorControlMode motor_mode = MOTOR_CONTROL_STOPPED;
static MotorControlMode requested_mode = MOTOR_CONTROL_OPEN_VOLTAGE;
static MotorRunState run_state = MOTOR_RUN_STATE_STOPPED;
static MotorFaultCode fault_code = MOTOR_FAULT_NONE;

static OpenLoopState open_loop_state;
static MotorVoltageDq open_voltage_target;
static MotorVoltageDq last_voltage_pu;
static bool open_voltage_blend_active;
static float open_target_frequency_hz;

static CurrentSenseConfig current_sense_config;
static CurrentSenseOffsets current_offsets;
static CurrentSenseCalibration current_calibration;
static CurrentPhaseCurrents phase_current;
static FocAlphaBeta alpha_beta_current;
static FocDq current_feedback_dq;
static bool current_feedback_valid;
static bool current_sense_ready;
static bool current_calibration_failed;

static CurrentPiController current_pi;
static CurrentPiResult current_pi_result;
static FocDq current_reference_target;
static FocDq current_reference_active;
static float current_target_frequency_hz;
static float current_voltage_limit_pu = MOTOR_POLE_VOLTAGE_LIMIT_START_PU;
static float alignment_elapsed_s;

static uint32_t adc_age_ticks;
static uint32_t overcurrent_count;
static bool pwm_enabled;
static bool sampling_started;

static float MotorControl_Clamp(float value, float low, float high)
{
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

static float MotorControl_MoveToward(float current, float target, float maximum_step)
{
    const float error = target - current;

    if (error > maximum_step)
    {
        return current + maximum_step;
    }
    if (error < -maximum_step)
    {
        return current - maximum_step;
    }
    return target;
}

static bool MotorControl_ModeIsImplemented(MotorControlMode mode)
{
    return (mode == MOTOR_CONTROL_OPEN_VOLTAGE) ||
           (mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
}

static void MotorControl_UpdateDebugState(void)
{
    g_motor_control_debug.mode = motor_mode;
    g_motor_control_debug.requested_mode = requested_mode;
    g_motor_control_debug.run_state = run_state;
    g_motor_control_debug.fault = fault_code;
    g_motor_control_debug.current_sense_ready = current_sense_ready ? 1U : 0U;
    g_motor_control_debug.adc_age_ticks = adc_age_ticks;
    g_motor_control_debug.overcurrent_count = overcurrent_count;
    g_motor_control_debug.electrical_angle_pu = open_loop_state.electrical_angle_pu;
    g_motor_control_debug.electrical_frequency_hz = open_loop_state.electrical_frequency_hz;
}

static void MotorControl_DisablePowerStage(void)
{
    if (pwm_enabled)
    {
        (void)Pwm3ph_Disable();
        pwm_enabled = false;
    }
    Drv8323Board_SetCurrentCalibration(false);
    Drv8323Board_Disable();
}

static void MotorControl_EnterFault(MotorFaultCode fault)
{
    fault_code = fault;
    motor_mode = MOTOR_CONTROL_FAULT;
    run_state = MOTOR_RUN_STATE_FAULT;
    requested_mode = MOTOR_CONTROL_FAULT;
    CurrentPi_Reset(&current_pi);
    MotorControl_DisablePowerStage();
    MotorControl_UpdateDebugState();
}

static HAL_StatusTypeDef MotorControl_EnablePwm(void)
{
    HAL_StatusTypeDef status;

    if (pwm_enabled)
    {
        return HAL_OK;
    }

    status = Pwm3ph_Enable();
    if (status == HAL_OK)
    {
        pwm_enabled = true;
    }
    return status;
}

static bool MotorControl_WriteVoltage(const MotorVoltageDq *voltage)
{
    MotorPwmDuty duty;

    if ((voltage == 0) || !isfinite(voltage->ud_pu) || !isfinite(voltage->uq_pu))
    {
        MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        return false;
    }

    Svpwm_Compute(voltage, open_loop_state.electrical_angle_pu, &duty);
    Pwm3ph_ApplyDuty(&duty);
    last_voltage_pu = *voltage;
    g_motor_control_debug.ud_pu = voltage->ud_pu;
    g_motor_control_debug.uq_pu = voltage->uq_pu;
    return true;
}

static void MotorControl_UpdateOpenVoltageBlend(float dt_s)
{
    float maximum_step;

    if (!open_voltage_blend_active)
    {
        return;
    }

    maximum_step = motor_config.voltage_slew_pu_per_s * dt_s;
    if (maximum_step <= 0.0f)
    {
        open_loop_state.ud_pu = open_voltage_target.ud_pu;
        open_loop_state.uq_pu = open_voltage_target.uq_pu;
    }
    else
    {
        open_loop_state.ud_pu = MotorControl_MoveToward(
            open_loop_state.ud_pu, open_voltage_target.ud_pu, maximum_step);
        open_loop_state.uq_pu = MotorControl_MoveToward(
            open_loop_state.uq_pu, open_voltage_target.uq_pu, maximum_step);
    }

    if ((open_loop_state.ud_pu == open_voltage_target.ud_pu) &&
        (open_loop_state.uq_pu == open_voltage_target.uq_pu))
    {
        open_voltage_blend_active = false;
    }
}

static void MotorControl_StartSelectedMode(void)
{
    if (MotorControl_EnablePwm() != HAL_OK)
    {
        MotorControl_EnterFault(MOTOR_FAULT_DRIVER);
        return;
    }

    if (requested_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT)
    {
        if (!current_sense_ready)
        {
            MotorControl_EnterFault(MOTOR_FAULT_CURRENT_CALIBRATION);
            return;
        }

        motor_mode = MOTOR_CONTROL_OPEN_ANGLE_CURRENT;
        run_state = MOTOR_RUN_STATE_ALIGNING;
        alignment_elapsed_s = 0.0f;
        current_reference_active.d = 0.0f;
        current_reference_active.q = 0.0f;
        open_loop_state.electrical_angle_pu = 0.0f;
        open_loop_state.electrical_frequency_hz = 0.0f;
        open_loop_state.target_frequency_hz = 0.0f;
        CurrentPi_Reset(&current_pi);
    }
    else
    {
        requested_mode = MOTOR_CONTROL_OPEN_VOLTAGE;
        motor_mode = MOTOR_CONTROL_OPEN_VOLTAGE;
        run_state = MOTOR_RUN_STATE_RUNNING;
    }
    MotorControl_UpdateDebugState();
}

static void MotorControl_FinishCalibration(void)
{
    const bool valid = CurrentSenseCalibration_GetOffsets(
        &current_calibration,
        MOTOR_ADC_FULL_SCALE_COUNT,
        CURRENT_CALIBRATION_RAIL_MARGIN,
        CURRENT_CALIBRATION_MAX_SPAN,
        &current_offsets);

    Drv8323Board_SetCurrentCalibration(false);
    current_sense_ready = valid;
    current_calibration_failed = !valid;
    if (valid)
    {
        fault_code = MOTOR_FAULT_NONE;
        g_motor_control_debug.phase_a_offset = current_offsets.phase_a_count;
        g_motor_control_debug.phase_b_offset = current_offsets.phase_b_count;
    }
    else
    {
        /* 保留已验证的电压开环，但明确禁止进入依赖电流反馈的模式。 */
        fault_code = MOTOR_FAULT_CURRENT_CALIBRATION;
        if (requested_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT)
        {
            MotorControl_EnterFault(MOTOR_FAULT_CURRENT_CALIBRATION);
            return;
        }
    }

    run_state = MOTOR_RUN_STATE_READY;
    MotorControl_StartSelectedMode();
}

static bool MotorControl_UpdateCurrentMeasurement(uint32_t phase_a_raw,
                                                  uint32_t phase_b_raw)
{
    const uint32_t upper_rail = (uint32_t)MOTOR_ADC_FULL_SCALE_COUNT -
                                CURRENT_ADC_RUNTIME_RAIL_MARGIN;
    float peak_current;

    if ((phase_a_raw <= CURRENT_ADC_RUNTIME_RAIL_MARGIN) ||
        (phase_b_raw <= CURRENT_ADC_RUNTIME_RAIL_MARGIN) ||
        (phase_a_raw >= upper_rail) ||
        (phase_b_raw >= upper_rail))
    {
        MotorControl_EnterFault(MOTOR_FAULT_ADC_RANGE);
        return false;
    }

    if (!CurrentSense_Convert(&current_sense_config,
                              &current_offsets,
                              phase_a_raw,
                              phase_b_raw,
                              &phase_current) ||
        !Foc_Clarke(&phase_current, &alpha_beta_current) ||
        !Foc_Park(&alpha_beta_current,
                  open_loop_state.electrical_angle_pu,
                  &current_feedback_dq))
    {
        MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        return false;
    }

    current_feedback_valid = true;
    peak_current = fmaxf(fabsf(phase_current.ia_a),
                         fmaxf(fabsf(phase_current.ib_a), fabsf(phase_current.ic_a)));
    if (peak_current > MOTOR_CURRENT_TRIP_A)
    {
        ++overcurrent_count;
        if (overcurrent_count >= CURRENT_OVERCURRENT_CONFIRM_COUNT)
        {
            MotorControl_EnterFault(MOTOR_FAULT_OVERCURRENT);
            return false;
        }
    }
    else
    {
        overcurrent_count = 0U;
    }

    g_motor_control_debug.ia_a = phase_current.ia_a;
    g_motor_control_debug.ib_a = phase_current.ib_a;
    g_motor_control_debug.ic_a = phase_current.ic_a;
    g_motor_control_debug.i_alpha_a = alpha_beta_current.alpha;
    g_motor_control_debug.i_beta_a = alpha_beta_current.beta;
    g_motor_control_debug.id_a = current_feedback_dq.d;
    g_motor_control_debug.iq_a = current_feedback_dq.q;
    g_motor_control_debug.overcurrent_count = overcurrent_count;
    return true;
}

static void MotorControl_ApplyModeRequest(void)
{
    FocDq requested_voltage_v;

    if ((requested_mode == motor_mode) || !MotorControl_ModeIsImplemented(requested_mode))
    {
        return;
    }

    if (requested_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT)
    {
        if (!current_sense_ready || !current_feedback_valid)
        {
            requested_mode = motor_mode;
            MotorControl_UpdateDebugState();
            return;
        }

        current_reference_active.d = MotorControl_Clamp(
            current_feedback_dq.d,
            -MOTOR_CURRENT_COMMAND_LIMIT_A,
            MOTOR_CURRENT_COMMAND_LIMIT_A);
        current_reference_active.q = MotorControl_Clamp(
            current_feedback_dq.q,
            -MOTOR_CURRENT_COMMAND_LIMIT_A,
            MOTOR_CURRENT_COMMAND_LIMIT_A);
        requested_voltage_v.d = last_voltage_pu.ud_pu * MOTOR_NOMINAL_VBUS_V;
        requested_voltage_v.q = last_voltage_pu.uq_pu * MOTOR_NOMINAL_VBUS_V;
        if (!CurrentPi_PreloadOutput(&current_pi,
                                     &current_reference_active,
                                     &current_feedback_dq,
                                     &requested_voltage_v))
        {
            MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
            return;
        }

        motor_mode = MOTOR_CONTROL_OPEN_ANGLE_CURRENT;
        run_state = MOTOR_RUN_STATE_RUNNING;
        alignment_elapsed_s = CURRENT_ALIGNMENT_TIME_S;
    }
    else
    {
        /* 从 PI 最后输出开始，再缓慢回到保存的开环命令。 */
        open_loop_state.ud_pu = last_voltage_pu.ud_pu;
        open_loop_state.uq_pu = last_voltage_pu.uq_pu;
        open_voltage_blend_active = true;
        motor_mode = MOTOR_CONTROL_OPEN_VOLTAGE;
        run_state = MOTOR_RUN_STATE_RUNNING;
    }
    MotorControl_UpdateDebugState();
}

static void MotorControl_RunCurrentLoop(float dt_s)
{
    FocDq step_target;
    MotorVoltageDq voltage_pu;
    float maximum_current_step;

    if (run_state == MOTOR_RUN_STATE_ALIGNING)
    {
        step_target.d = MOTOR_CURRENT_ALIGN_A;
        step_target.q = 0.0f;
        alignment_elapsed_s += dt_s;
        if (alignment_elapsed_s >= CURRENT_ALIGNMENT_TIME_S)
        {
            run_state = MOTOR_RUN_STATE_RUNNING;
        }
    }
    else
    {
        step_target = current_reference_target;
    }

    maximum_current_step = MOTOR_CURRENT_COMMAND_SLEW_A_PER_S * dt_s;
    current_reference_active.d = MotorControl_MoveToward(
        current_reference_active.d, step_target.d, maximum_current_step);
    current_reference_active.q = MotorControl_MoveToward(
        current_reference_active.q, step_target.q, maximum_current_step);

    if (!CurrentPi_StepDq(&current_pi,
                          &current_reference_active,
                          &current_feedback_dq,
                          dt_s,
                          &current_pi_result))
    {
        MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        return;
    }

    voltage_pu.ud_pu = current_pi_result.voltage_v.d / MOTOR_NOMINAL_VBUS_V;
    voltage_pu.uq_pu = current_pi_result.voltage_v.q / MOTOR_NOMINAL_VBUS_V;
    if (!MotorControl_WriteVoltage(&voltage_pu))
    {
        return;
    }

    g_motor_control_debug.id_ref_a = current_reference_active.d;
    g_motor_control_debug.iq_ref_a = current_reference_active.q;
    g_motor_control_debug.id_error_a = current_pi_result.error_a.d;
    g_motor_control_debug.iq_error_a = current_pi_result.error_a.q;
    g_motor_control_debug.ud_integrator_v = current_pi_result.integrator_v.d;
    g_motor_control_debug.uq_integrator_v = current_pi_result.integrator_v.q;
    g_motor_control_debug.ud_v = current_pi_result.voltage_v.d;
    g_motor_control_debug.uq_v = current_pi_result.voltage_v.q;
    g_motor_control_debug.voltage_saturated = current_pi_result.saturated ? 1U : 0U;
    MotorControl_UpdateDebugState();
}

HAL_StatusTypeDef MotorControl_Init(const MotorControlConfig *config)
{
    CurrentPiConfig pi_config;

    memset((void *)&g_motor_control_debug, 0, sizeof(g_motor_control_debug));
    if ((config == 0) || !MotorControl_ModeIsImplemented(config->mode) ||
        !isfinite(config->frequency_slew_hz_per_s) ||
        !isfinite(config->voltage_slew_pu_per_s) ||
        (config->frequency_slew_hz_per_s < 0.0f) ||
        (config->voltage_slew_pu_per_s < 0.0f))
    {
        fault_code = MOTOR_FAULT_INVALID_CONFIG;
        MotorControl_UpdateDebugState();
        return HAL_ERROR;
    }

    motor_config = *config;
    requested_mode = config->mode;
    motor_mode = MOTOR_CONTROL_STOPPED;
    run_state = MOTOR_RUN_STATE_STOPPED;
    fault_code = MOTOR_FAULT_NONE;

    OpenLoop_Init(&open_loop_state, config->frequency_slew_hz_per_s);
    open_voltage_target.ud_pu = 0.0f;
    open_voltage_target.uq_pu = 0.0f;
    last_voltage_pu = open_voltage_target;
    open_target_frequency_hz = 0.0f;
    open_voltage_blend_active = false;

    current_sense_config.adc_reference_v = MOTOR_ADC_REFERENCE_V;
    current_sense_config.adc_full_scale_count = MOTOR_ADC_FULL_SCALE_COUNT;
    current_sense_config.shunt_resistance_ohm = MOTOR_SHUNT_RESISTANCE_OHM;
    current_sense_config.amplifier_gain_v_per_v = MOTOR_CSA_GAIN_V_PER_V;
    current_sense_config.phase_a_polarity = MOTOR_CURRENT_IA_POLARITY;
    current_sense_config.phase_b_polarity = MOTOR_CURRENT_IB_POLARITY;
    current_offsets.phase_a_count = MOTOR_ADC_MID_SCALE_COUNT;
    current_offsets.phase_b_count = MOTOR_ADC_MID_SCALE_COUNT;
    current_sense_ready = false;
    current_calibration_failed = false;
    current_feedback_valid = false;

    pi_config.kp_v_per_a = MOTOR_CURRENT_PI_KP_V_PER_A;
    pi_config.ki_v_per_a_s = MOTOR_CURRENT_PI_KI_V_PER_A_S;
    pi_config.kaw_per_s = MOTOR_CURRENT_PI_KAW_PER_S;
    pi_config.output_limit_v = MOTOR_POLE_VOLTAGE_LIMIT_START_PU * MOTOR_NOMINAL_VBUS_V;
    if (!CurrentPi_Init(&current_pi, &pi_config))
    {
        fault_code = MOTOR_FAULT_INVALID_CONFIG;
        MotorControl_UpdateDebugState();
        return HAL_ERROR;
    }

    current_reference_target.d = 0.0f;
    current_reference_target.q = MOTOR_CURRENT_START_IQ_A;
    current_reference_active.d = 0.0f;
    current_reference_active.q = 0.0f;
    current_target_frequency_hz = 1.0f;
    current_voltage_limit_pu = MOTOR_POLE_VOLTAGE_LIMIT_START_PU;
    alignment_elapsed_s = 0.0f;
    adc_age_ticks = 0U;
    overcurrent_count = 0U;
    pwm_enabled = false;
    sampling_started = false;

    if (Drv8323Board_Init() != HAL_OK)
    {
        fault_code = MOTOR_FAULT_DRIVER;
        MotorControl_UpdateDebugState();
        return HAL_ERROR;
    }
    if (Pwm3ph_Init() != HAL_OK)
    {
        fault_code = MOTOR_FAULT_DRIVER;
        MotorControl_UpdateDebugState();
        return HAL_ERROR;
    }

    g_motor_control_debug.current_kp_v_per_a = pi_config.kp_v_per_a;
    g_motor_control_debug.current_ki_v_per_a_s = pi_config.ki_v_per_a_s;
    g_motor_control_debug.current_kaw_per_s = pi_config.kaw_per_s;
    g_motor_control_debug.current_voltage_limit_pu = current_voltage_limit_pu;
    MotorControl_UpdateDebugState();
    return HAL_OK;
}

void MotorControl_SetOpenLoopCommand(float ud_pu,
                                     float uq_pu,
                                     float electrical_frequency_hz)
{
    uint32_t interrupt_state;

    if (!isfinite(ud_pu) || !isfinite(uq_pu) || !isfinite(electrical_frequency_hz))
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    open_voltage_target.ud_pu = ud_pu;
    open_voltage_target.uq_pu = uq_pu;
    open_target_frequency_hz = electrical_frequency_hz;
    if ((motor_mode == MOTOR_CONTROL_STOPPED) ||
        ((motor_mode == MOTOR_CONTROL_OPEN_VOLTAGE) && !open_voltage_blend_active))
    {
        open_loop_state.ud_pu = ud_pu;
        open_loop_state.uq_pu = uq_pu;
    }
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
}

void MotorControl_SetCurrentCommand(float id_a,
                                    float iq_a,
                                    float electrical_frequency_hz)
{
    uint32_t interrupt_state;

    if (!isfinite(id_a) || !isfinite(iq_a) || !isfinite(electrical_frequency_hz))
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    current_reference_target.d = MotorControl_Clamp(
        id_a, -MOTOR_CURRENT_COMMAND_LIMIT_A, MOTOR_CURRENT_COMMAND_LIMIT_A);
    current_reference_target.q = MotorControl_Clamp(
        iq_a, -MOTOR_CURRENT_COMMAND_LIMIT_A, MOTOR_CURRENT_COMMAND_LIMIT_A);
    current_target_frequency_hz = electrical_frequency_hz;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
}

HAL_StatusTypeDef MotorControl_RequestMode(MotorControlMode mode)
{
    uint32_t interrupt_state;

    if (!MotorControl_ModeIsImplemented(mode) || (motor_mode == MOTOR_CONTROL_FAULT) ||
        ((mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) && current_calibration_failed))
    {
        return HAL_ERROR;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    requested_mode = mode;
    g_motor_control_debug.requested_mode = mode;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_SetCurrentPiGains(float kp_v_per_a,
                                                 float ki_v_per_a_s,
                                                 float kaw_per_s)
{
    uint32_t interrupt_state;
    bool valid;

    if (!isfinite(kp_v_per_a) || !isfinite(ki_v_per_a_s) || !isfinite(kaw_per_s) ||
        (kp_v_per_a < 0.0f) || (kp_v_per_a > CURRENT_PI_KP_MAX_V_PER_A) ||
        (ki_v_per_a_s < 0.0f) || (ki_v_per_a_s > CURRENT_PI_KI_MAX_V_PER_A_S) ||
        (kaw_per_s < 0.0f) || (kaw_per_s > CURRENT_PI_KAW_MAX_PER_S))
    {
        return HAL_ERROR;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    valid = CurrentPi_SetGains(&current_pi, kp_v_per_a, ki_v_per_a_s, kaw_per_s);
    if (valid)
    {
        g_motor_control_debug.current_kp_v_per_a = kp_v_per_a;
        g_motor_control_debug.current_ki_v_per_a_s = ki_v_per_a_s;
        g_motor_control_debug.current_kaw_per_s = kaw_per_s;
    }
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    return valid ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MotorControl_SetCurrentVoltageLimit(float limit_pu)
{
    uint32_t interrupt_state;
    bool valid;

    if (!isfinite(limit_pu) || (limit_pu <= 0.0f) ||
        (limit_pu > MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU))
    {
        return HAL_ERROR;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    valid = CurrentPi_SetOutputLimit(&current_pi, limit_pu * MOTOR_NOMINAL_VBUS_V);
    if (valid)
    {
        current_voltage_limit_pu = limit_pu;
        g_motor_control_debug.current_voltage_limit_pu = limit_pu;
    }
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    return valid ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MotorControl_Start(void)
{
    HAL_StatusTypeDef status;

    if ((motor_mode != MOTOR_CONTROL_STOPPED) || sampling_started)
    {
        return HAL_ERROR;
    }

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        fault_code = MOTOR_FAULT_ADC_START;
        MotorControl_UpdateDebugState();
        return status;
    }

    status = Drv8323Board_EnableForPwm();
    if (status != HAL_OK)
    {
        fault_code = MOTOR_FAULT_DRIVER;
        MotorControl_UpdateDebugState();
        return status;
    }

    CurrentSenseCalibration_Start(&current_calibration,
                                  CURRENT_CALIBRATION_DISCARD_COUNT,
                                  CURRENT_CALIBRATION_SAMPLE_COUNT);
    current_sense_ready = false;
    current_calibration_failed = false;
    current_feedback_valid = false;
    run_state = MOTOR_RUN_STATE_CURRENT_CALIBRATING;
    Drv8323Board_SetCurrentCalibration(true);

    status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    if (status != HAL_OK)
    {
        Drv8323Board_SetCurrentCalibration(false);
        Drv8323Board_Disable();
        MotorControl_EnterFault(MOTOR_FAULT_ADC_START);
        return status;
    }

    status = HAL_TIM_Base_Start_IT(&htim8);
    if (status != HAL_OK)
    {
        (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
        Drv8323Board_SetCurrentCalibration(false);
        Drv8323Board_Disable();
        MotorControl_EnterFault(MOTOR_FAULT_ADC_START);
        return status;
    }

    status = HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    if (status != HAL_OK)
    {
        (void)HAL_TIM_Base_Stop_IT(&htim8);
        (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
        Drv8323Board_SetCurrentCalibration(false);
        Drv8323Board_Disable();
        MotorControl_EnterFault(MOTOR_FAULT_ADC_START);
        return status;
    }

    sampling_started = true;
    MotorControl_UpdateDebugState();
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Stop(void)
{
    HAL_StatusTypeDef result = HAL_OK;

    motor_mode = MOTOR_CONTROL_STOPPED;
    run_state = MOTOR_RUN_STATE_STOPPED;
    requested_mode = motor_config.mode;
    MotorControl_DisablePowerStage();

    if (sampling_started)
    {
        if (HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4) != HAL_OK)
        {
            result = HAL_ERROR;
        }
        if (HAL_TIM_Base_Stop_IT(&htim8) != HAL_OK)
        {
            result = HAL_ERROR;
        }
        if (HAL_ADCEx_InjectedStop_IT(&hadc1) != HAL_OK)
        {
            result = HAL_ERROR;
        }
        sampling_started = false;
    }

    CurrentPi_Reset(&current_pi);
    adc_age_ticks = 0U;
    overcurrent_count = 0U;
    MotorControl_UpdateDebugState();
    return result;
}

HAL_StatusTypeDef MotorControl_ClearFault(void)
{
    HAL_StatusTypeDef status;

    if (motor_mode != MOTOR_CONTROL_FAULT)
    {
        return HAL_ERROR;
    }

    status = MotorControl_Stop();
    fault_code = MOTOR_FAULT_NONE;
    current_calibration_failed = false;
    MotorControl_UpdateDebugState();
    return status;
}

void MotorControl_FastTick(float dt_s)
{
    MotorVoltageDq open_voltage;

    if (!isfinite(dt_s) || (dt_s <= 0.0f))
    {
        if ((motor_mode == MOTOR_CONTROL_OPEN_VOLTAGE) ||
            (motor_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT))
        {
            MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        }
        return;
    }

    if ((motor_mode != MOTOR_CONTROL_OPEN_VOLTAGE) &&
        (motor_mode != MOTOR_CONTROL_OPEN_ANGLE_CURRENT))
    {
        return;
    }

    MotorControl_ApplyModeRequest();
    if (motor_mode == MOTOR_CONTROL_FAULT)
    {
        return;
    }

    if (motor_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT)
    {
        ++adc_age_ticks;
        if (adc_age_ticks > CURRENT_ADC_TIMEOUT_TICKS)
        {
            MotorControl_EnterFault(MOTOR_FAULT_ADC_TIMEOUT);
            return;
        }
        open_loop_state.target_frequency_hz =
            (run_state == MOTOR_RUN_STATE_ALIGNING) ? 0.0f : current_target_frequency_hz;
    }
    else
    {
        open_loop_state.target_frequency_hz = open_target_frequency_hz;
        MotorControl_UpdateOpenVoltageBlend(dt_s);
    }

    /* 两种模式共用这一角度积分点，切换时不会清零或重复推进电角度。 */
    OpenLoop_Step(&open_loop_state, dt_s, &open_voltage);
    if (motor_mode == MOTOR_CONTROL_OPEN_VOLTAGE)
    {
        (void)MotorControl_WriteVoltage(&open_voltage);
    }
    MotorControl_UpdateDebugState();
}

void MotorControl_CurrentSampleComplete(uint32_t phase_a_raw,
                                        uint32_t phase_b_raw,
                                        float dt_s)
{
    g_motor_control_debug.phase_a_raw = phase_a_raw;
    g_motor_control_debug.phase_b_raw = phase_b_raw;

    if (run_state == MOTOR_RUN_STATE_CURRENT_CALIBRATING)
    {
        if (CurrentSenseCalibration_AddSample(&current_calibration,
                                              phase_a_raw,
                                              phase_b_raw))
        {
            MotorControl_FinishCalibration();
        }
        g_motor_control_debug.calibration_sample_count = current_calibration.sample_count;
        MotorControl_UpdateDebugState();
        return;
    }

    if ((motor_mode == MOTOR_CONTROL_STOPPED) || (motor_mode == MOTOR_CONTROL_FAULT) ||
        !current_sense_ready)
    {
        return;
    }

    adc_age_ticks = 0U;
    if (!isfinite(dt_s) || (dt_s <= 0.0f) ||
        !MotorControl_UpdateCurrentMeasurement(phase_a_raw, phase_b_raw))
    {
        if ((motor_mode != MOTOR_CONTROL_FAULT) && (!isfinite(dt_s) || (dt_s <= 0.0f)))
        {
            MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        }
        return;
    }

    if (motor_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT)
    {
        MotorControl_RunCurrentLoop(dt_s);
    }
    MotorControl_UpdateDebugState();
}
