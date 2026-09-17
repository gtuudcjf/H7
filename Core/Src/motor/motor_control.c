/**
 * @file motor_control.c
 * @brief H7 电机控制编排：校准、开环、电流环、模式切换和故障处理。
 *
 * 快速路径不执行阻塞式 SPI、延时或日志输出。TIM8 更新中断只负责统一
 * 电角度时基和电压开环；ADC 注入完成中断只负责采样监视和电流闭环。
 */
#include "motor_control.h"

#include "adc.h"
#include "biss_encoder.h"
#include "current_pi.h"
#include "current_sense.h"
#include "drv8323_board.h"
#include "encoder_angle.h"
#include "encoder_calibration.h"
#include "foc_transform.h"
#include "motor_config_store.h"
#include "motor_params.h"
#include "motor_runtime_policy.h"
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
#define ENCODER_STALE_LIMIT_TICKS         (10U)

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
static FocDq encoder_current_reference_target;
static float current_target_frequency_hz;
static float current_voltage_limit_pu = MOTOR_POLE_VOLTAGE_LIMIT_START_PU;
static float alignment_elapsed_s;

static BissEncoderSnapshot encoder_snapshot;
static EncoderAngleConfig encoder_angle_config;
static EncoderAngleSample encoder_angle_sample;
static MotorCalibrationConfig encoder_saved_config;
static EncoderCalibration encoder_calibration;
static EncoderCalibrationCommand encoder_calibration_command;
static volatile bool encoder_calibration_requested;
static volatile bool encoder_calibration_active;
static volatile bool encoder_calibration_save_pending;
static volatile bool encoder_calibration_failure_pending;
static bool encoder_calibration_valid;
static float active_electrical_angle_pu;
static uint32_t encoder_calibration_last_sequence;

static uint32_t adc_age_ticks;
static uint32_t overcurrent_count;
static bool pwm_enabled;
static bool sampling_started;
static MotorStartupTrace startup_trace;

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
    return MotorRuntimePolicy_ClassifyStartMode(mode) != MOTOR_START_ACTION_INVALID;
}

static void MotorControl_UpdateDebugState(void)
{
    uint8_t index;

    g_motor_control_debug.mode = motor_mode;
    g_motor_control_debug.requested_mode = requested_mode;
    g_motor_control_debug.run_state = run_state;
    g_motor_control_debug.fault = fault_code;
    g_motor_control_debug.current_sense_ready = current_sense_ready ? 1U : 0U;
    g_motor_control_debug.adc_age_ticks = adc_age_ticks;
    g_motor_control_debug.overcurrent_count = overcurrent_count;
    g_motor_control_debug.electrical_angle_pu = active_electrical_angle_pu;
    g_motor_control_debug.electrical_frequency_hz =
        (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) ?
            0.0f : open_loop_state.electrical_frequency_hz;
    g_motor_control_debug.encoder_ready = encoder_snapshot.ready ? 1U : 0U;
    g_motor_control_debug.encoder_warning = encoder_snapshot.warning ? 1U : 0U;
    g_motor_control_debug.encoder_calibrated = encoder_calibration_valid ? 1U : 0U;
    for (index = 0U; index < BISS_FRAME_RAW_BYTES; ++index)
    {
        g_motor_control_debug.encoder_raw[index] = encoder_snapshot.raw[index];
    }
    g_motor_control_debug.encoder_received_crc = encoder_snapshot.received_crc;
    g_motor_control_debug.encoder_calculated_crc = encoder_snapshot.calculated_crc;
    g_motor_control_debug.encoder_direction = encoder_calibration_valid ?
        encoder_angle_config.direction : 0;
    g_motor_control_debug.encoder_frame_status = encoder_snapshot.frame_status;
    g_motor_control_debug.encoder_calibration_state = encoder_calibration.state;
    g_motor_control_debug.encoder_calibration_failure = encoder_calibration.failure;
    g_motor_control_debug.encoder_position_raw = encoder_snapshot.position_raw;
    g_motor_control_debug.encoder_sequence = encoder_snapshot.sequence;
    g_motor_control_debug.encoder_age_ticks = encoder_snapshot.valid_age_ticks;
    g_motor_control_debug.encoder_valid_count = encoder_snapshot.valid_count;
    g_motor_control_debug.encoder_crc_error_count = encoder_snapshot.crc_error_count;
    g_motor_control_debug.encoder_frame_error_count = encoder_snapshot.frame_error_count;
    g_motor_control_debug.encoder_spi_error_count = encoder_snapshot.spi_error_count;
    g_motor_control_debug.encoder_timeout_count = encoder_snapshot.timeout_count;
    g_motor_control_debug.encoder_dma_guard_error_count =
        encoder_snapshot.dma_guard_error_count;
    g_motor_control_debug.encoder_electrical_zero_raw = encoder_calibration_valid ?
        encoder_angle_config.zero_raw : 0U;
    g_motor_control_debug.encoder_mechanical_angle_pu =
        encoder_angle_sample.mechanical_angle_pu;
    g_motor_control_debug.encoder_electrical_angle_pu =
        encoder_angle_sample.electrical_angle_pu;
    g_motor_control_debug.startup_trace_count = startup_trace.checkpoint_count;
    for (index = 0U; index < MOTOR_STARTUP_TRACE_CAPACITY; ++index)
    {
        g_motor_control_debug.startup_trace_stage[index] =
            startup_trace.checkpoint_stage[index];
        g_motor_control_debug.startup_trace_mode[index] =
            startup_trace.checkpoint_mode[index];
    }
    g_motor_control_debug.startup_invalid_detected =
        startup_trace.invalid_detected ? 1U : 0U;
    g_motor_control_debug.first_invalid_requested_mode =
        startup_trace.first_invalid_mode;
    g_motor_control_debug.first_invalid_startup_stage =
        startup_trace.first_invalid_stage;
    g_motor_control_debug.first_invalid_calibration_sample =
        startup_trace.first_invalid_sample;
    g_motor_control_debug.mode_integrity_error_count =
        startup_trace.integrity_error_count;
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

static void MotorControl_QuiesceRealtimeInterrupts(void)
{
    /*
     * 校准完成后功率级已关断，但 TIM8 更新与 ADC 注入完成中断
     * 仍以 10 kHz 运行。在这里只禁止实时中断源，不调用 HAL 停机、
     * 不写 Flash；完整的外设停止和 Flash 保存仍由前台 Service 完成。
     */
    __HAL_TIM_DISABLE_IT(&htim8, TIM_IT_UPDATE);
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC | ADC_IT_JEOS);
}

static void MotorControl_EnterFault(MotorFaultCode fault)
{
    fault_code = fault;
    motor_mode = MOTOR_CONTROL_FAULT;
    run_state = MOTOR_RUN_STATE_FAULT;
    requested_mode = MOTOR_CONTROL_FAULT;
    /* 任意控制故障都立即取消标定写 PWM 的资格，保留状态机失败信息供调试。 */
    encoder_calibration_requested = false;
    encoder_calibration_active = false;
    encoder_calibration_save_pending = false;
    encoder_calibration_failure_pending = false;
    memset(&encoder_calibration_command, 0, sizeof(encoder_calibration_command));
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

static bool MotorControl_WriteVoltage(const MotorVoltageDq *voltage,
                                      float electrical_angle_pu)
{
    MotorPwmDuty duty;

    if ((voltage == 0) || !isfinite(voltage->ud_pu) || !isfinite(voltage->uq_pu))
    {
        MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        return false;
    }

    Svpwm_Compute(voltage, electrical_angle_pu, &duty);
    Pwm3ph_ApplyDuty(&duty);
    last_voltage_pu = *voltage;
    active_electrical_angle_pu = electrical_angle_pu;
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
    const MotorStartAction start_action =
        MotorRuntimePolicy_ClassifyStartMode(requested_mode);

    /*
     * requested_mode若被破坏，必须在使能PWM前停机并报告配置故障。
     * 禁止自动退回电压开环，否则会把内存/DMA问题伪装成正常运行。
     */
    if (start_action == MOTOR_START_ACTION_INVALID)
    {
        MotorControl_EnterFault(MOTOR_FAULT_INVALID_CONFIG);
        return;
    }

    if (requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)
    {
        if (!current_sense_ready)
        {
            MotorControl_EnterFault(MOTOR_FAULT_CURRENT_CALIBRATION);
            return;
        }
        if (!encoder_calibration_valid)
        {
            MotorControl_EnterFault(MOTOR_FAULT_ENCODER_NOT_CALIBRATED);
            return;
        }
        if (!BissEncoder_GetSnapshot(&encoder_snapshot) ||
            !encoder_snapshot.ready ||
            (encoder_snapshot.valid_age_ticks > ENCODER_STALE_LIMIT_TICKS) ||
            !EncoderAngle_Update(&encoder_angle_config,
                                 encoder_snapshot.position_raw,
                                 &encoder_angle_sample))
        {
            MotorControl_EnterFault(MOTOR_FAULT_ENCODER_NOT_READY);
            return;
        }
    }

    if (MotorControl_EnablePwm() != HAL_OK)
    {
        MotorControl_EnterFault(MOTOR_FAULT_DRIVER);
        return;
    }

    if (start_action == MOTOR_START_ACTION_CURRENT_CONTROL)
    {
        if (!current_sense_ready)
        {
            MotorControl_EnterFault(MOTOR_FAULT_CURRENT_CALIBRATION);
            return;
        }

        motor_mode = requested_mode;
        run_state = (requested_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ?
            MOTOR_RUN_STATE_ALIGNING : MOTOR_RUN_STATE_RUNNING;
        alignment_elapsed_s = 0.0f;
        current_reference_active.d = 0.0f;
        current_reference_active.q = 0.0f;
        open_loop_state.electrical_angle_pu = 0.0f;
        open_loop_state.electrical_frequency_hz = 0.0f;
        open_loop_state.target_frequency_hz = 0.0f;
        active_electrical_angle_pu =
            (requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) ?
                encoder_angle_sample.electrical_angle_pu : 0.0f;
        CurrentPi_Reset(&current_pi);
    }
    else
    {
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
        if ((requested_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
            (requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT))
        {
            MotorControl_EnterFault(MOTOR_FAULT_CURRENT_CALIBRATION);
            return;
        }
    }

    run_state = MOTOR_RUN_STATE_READY;
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_BEFORE_MODE_SELECT,
        (uint8_t)requested_mode);

    /*
     * 编码器采集在电流零偏校准完成后才启动，因此模式3此时还没有首帧
     * 有效位置。先保持功率级PWM关闭并停在READY，FastTick取得首帧后
     * 再进入编码器角度电流闭环。缺少校准参数则立即报告明确故障。
     */
    if ((requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) &&
        encoder_calibration_valid)
    {
        MotorControl_UpdateDebugState();
        return;
    }
    MotorControl_StartSelectedMode();
}

static bool MotorControl_UpdateCurrentMeasurement(uint32_t phase_a_raw,
                                                   uint32_t phase_b_raw,
                                                   float electrical_angle_pu)
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
                  electrical_angle_pu,
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

static bool MotorControl_UpdateEncoderAngle(bool require_ready)
{
    /*
     * 编码器角度进入FOC前必须同时满足：已有有效帧、数据未过期、校准
     * 参数有效。仅“能读到encoder_position_raw”还不满足模式3的条件。
     */
    if (!BissEncoder_GetSnapshot(&encoder_snapshot) ||
        (require_ready && !encoder_snapshot.ready) ||
        (encoder_snapshot.sequence == 0U) ||
        (encoder_snapshot.valid_age_ticks > ENCODER_STALE_LIMIT_TICKS) ||
        !encoder_calibration_valid ||
        !EncoderAngle_Update(&encoder_angle_config,
                             encoder_snapshot.position_raw,
                             &encoder_angle_sample))
    {
        return false;
    }
    return true;
}

static void MotorControl_ApplyModeRequest(void)
{
    FocDq requested_voltage_v;
    float requested_angle_pu;

    if ((requested_mode == motor_mode) || !MotorControl_ModeIsImplemented(requested_mode))
    {
        return;
    }

    if ((requested_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
        (requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT))
    {
        if (!current_sense_ready || !current_feedback_valid)
        {
            requested_mode = motor_mode;
            MotorControl_UpdateDebugState();
            return;
        }

        if (requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)
        {
            if (!MotorControl_UpdateEncoderAngle(true))
            {
                requested_mode = motor_mode;
                MotorControl_UpdateDebugState();
                return;
            }
            requested_angle_pu = encoder_angle_sample.electrical_angle_pu;
        }
        else
        {
            /* 从编码器模式退出时，让虚拟角度从当前转子角度继续，避免相位阶跃。 */
            if (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)
            {
                open_loop_state.electrical_angle_pu = active_electrical_angle_pu;
            }
            requested_angle_pu = open_loop_state.electrical_angle_pu;
        }

        if (!Foc_Park(&alpha_beta_current,
                      requested_angle_pu,
                      &current_feedback_dq))
        {
            MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
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

        motor_mode = requested_mode;
        run_state = MOTOR_RUN_STATE_RUNNING;
        alignment_elapsed_s = CURRENT_ALIGNMENT_TIME_S;
        active_electrical_angle_pu = requested_angle_pu;
    }
    else
    {
        /* 从 PI 最后输出开始，再缓慢回到保存的开环命令。 */
        if (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)
        {
            open_loop_state.electrical_angle_pu = active_electrical_angle_pu;
        }
        open_loop_state.ud_pu = last_voltage_pu.ud_pu;
        open_loop_state.uq_pu = last_voltage_pu.uq_pu;
        open_voltage_blend_active = true;
        motor_mode = MOTOR_CONTROL_OPEN_VOLTAGE;
        run_state = MOTOR_RUN_STATE_RUNNING;
    }
    MotorControl_UpdateDebugState();
}

static bool MotorControl_RunCurrentLoop(float dt_s,
                                        float electrical_angle_pu)
{
    FocDq step_target;
    MotorVoltageDq voltage_pu;
    float maximum_current_step;

    /* 电流PI始终相同，三个分支只是在选择本周期的Id/Iq目标。 */
    if (encoder_calibration_active)
    {
        step_target.d = encoder_calibration_command.id_ref_a;
        step_target.q = encoder_calibration_command.iq_ref_a;
        current_reference_active = step_target;
    }
    else if (run_state == MOTOR_RUN_STATE_ALIGNING)
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
        step_target = (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) ?
            encoder_current_reference_target : current_reference_target;
    }

    if (!encoder_calibration_active)
    {
        maximum_current_step = MOTOR_CURRENT_COMMAND_SLEW_A_PER_S * dt_s;
        current_reference_active.d = MotorControl_MoveToward(
            current_reference_active.d, step_target.d, maximum_current_step);
        current_reference_active.q = MotorControl_MoveToward(
            current_reference_active.q, step_target.q, maximum_current_step);
    }

    if (!CurrentPi_StepDq(&current_pi,
                          &current_reference_active,
                          &current_feedback_dq,
                          dt_s,
                          &current_pi_result))
    {
        MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        return false;
    }

    voltage_pu.ud_pu = current_pi_result.voltage_v.d / MOTOR_NOMINAL_VBUS_V;
    voltage_pu.uq_pu = current_pi_result.voltage_v.q / MOTOR_NOMINAL_VBUS_V;
    if (!MotorControl_WriteVoltage(&voltage_pu, electrical_angle_pu))
    {
        return false;
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
    return true;
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
    MotorStartupTrace_Init(&startup_trace, (uint8_t)config->mode);
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_INIT_DONE,
        (uint8_t)requested_mode);

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
    encoder_current_reference_target.d = 0.0f;
    encoder_current_reference_target.q = 0.0f;
    current_reference_active.d = 0.0f;
    current_reference_active.q = 0.0f;
    current_target_frequency_hz = 1.0f;
    current_voltage_limit_pu = MOTOR_POLE_VOLTAGE_LIMIT_START_PU;
    alignment_elapsed_s = 0.0f;
    adc_age_ticks = 0U;
    overcurrent_count = 0U;
    pwm_enabled = false;
    sampling_started = false;

    memset(&encoder_snapshot, 0, sizeof(encoder_snapshot));
    memset(&encoder_angle_config, 0, sizeof(encoder_angle_config));
    memset(&encoder_angle_sample, 0, sizeof(encoder_angle_sample));
    memset(&encoder_saved_config, 0, sizeof(encoder_saved_config));
    memset(&encoder_calibration_command, 0, sizeof(encoder_calibration_command));
    EncoderCalibration_Init(&encoder_calibration);
    encoder_calibration_requested = false;
    encoder_calibration_active = false;
    encoder_calibration_save_pending = false;
    encoder_calibration_failure_pending = false;
    encoder_calibration_last_sequence = 0U;
    active_electrical_angle_pu = 0.0f;

    /*
     * Flash 中没有有效记录并不是启动故障：电压开环和虚拟角度电流环
     * 仍可照常运行。只有请求编码器角度模式时才要求该记录有效。
     */
    encoder_calibration_valid =
        (MotorConfigStore_Load(&encoder_saved_config) == HAL_OK) &&
        EncoderAngle_Init(&encoder_angle_config,
                          encoder_saved_config.electrical_zero_raw,
                          encoder_saved_config.encoder_direction,
                          encoder_saved_config.pole_pairs);
    (void)BissEncoder_GetSnapshot(&encoder_snapshot);

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

void MotorControl_SetEncoderCurrentCommand(float id_a, float iq_a)
{
    uint32_t interrupt_state;

    if (!isfinite(id_a) || !isfinite(iq_a))
    {
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    encoder_current_reference_target.d = MotorControl_Clamp(
        id_a, -MOTOR_CURRENT_COMMAND_LIMIT_A, MOTOR_CURRENT_COMMAND_LIMIT_A);
    encoder_current_reference_target.q = MotorControl_Clamp(
        iq_a, -MOTOR_CURRENT_COMMAND_LIMIT_A, MOTOR_CURRENT_COMMAND_LIMIT_A);
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
}

HAL_StatusTypeDef MotorControl_RequestEncoderCalibration(void)
{
    uint32_t interrupt_state;

    if ((motor_mode == MOTOR_CONTROL_FAULT) || !sampling_started ||
        !pwm_enabled || !current_sense_ready || encoder_calibration_active ||
        encoder_calibration_requested || encoder_calibration_save_pending ||
        encoder_calibration_failure_pending ||
        !BissEncoder_GetSnapshot(&encoder_snapshot) ||
        !encoder_snapshot.ready ||
        (encoder_snapshot.valid_age_ticks > ENCODER_STALE_LIMIT_TICKS))
    {
        return HAL_ERROR;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    encoder_calibration_requested = true;
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    return HAL_OK;
}

void MotorControl_Service(void)
{
    HAL_StatusTypeDef status;
    uint32_t interrupt_state;
    bool safe_to_write;

    if (g_motor_control_debug.foreground_service_count < UINT32_MAX)
    {
        ++g_motor_control_debug.foreground_service_count;
    }

    /*
     * ADC 中断只负责先关断功率级并置位完成标志。停止定时器、擦除和写入
     * Flash 全部在主循环执行，避免在任何实时中断中出现不可预测的延时。
     */
    if (encoder_calibration_failure_pending)
    {
        /*
         * 在停止 TIM8/ADC 之前保持 pending，使快速中断只走安全返回路径。
         * 若提前清零，中断会在前台调用 Stop 前恢复常规控制负载。
         */
        g_motor_control_debug.encoder_terminal_stage = 10U;
        status = MotorControl_Stop();
        g_motor_control_debug.encoder_terminal_hal_status = (uint8_t)status;
        g_motor_control_debug.encoder_terminal_stage = 11U;
        MotorControl_EnterFault(MOTOR_FAULT_ENCODER_ALIGNMENT);
        g_motor_control_debug.encoder_terminal_stage = 12U;
        return;
    }

    if (encoder_calibration_save_pending)
    {
        /* pending 在停机和 Flash 写后校验完成前始终保持为1。 */
        g_motor_control_debug.encoder_save_stage = 2U;
        status = MotorControl_Stop();
        safe_to_write = (status == HAL_OK) && !pwm_enabled && !sampling_started;
        if (!safe_to_write)
        {
            g_motor_control_debug.encoder_save_stage = 6U;
            g_motor_control_debug.encoder_save_hal_status = (uint8_t)status;
        }
        else
        {
            g_motor_control_debug.encoder_save_stage = 3U;
        }
        g_motor_control_debug.encoder_save_stage = safe_to_write ? 4U : 6U;
        status = MotorConfigStore_Save(&encoder_saved_config, safe_to_write);
        if ((status != HAL_OK) ||
            !EncoderAngle_Init(&encoder_angle_config,
                               encoder_saved_config.electrical_zero_raw,
                               encoder_saved_config.encoder_direction,
                               encoder_saved_config.pole_pairs))
        {
            encoder_calibration_valid = false;
            if (g_motor_control_debug.encoder_save_stage != 6U)
            {
                g_motor_control_debug.encoder_save_stage = 7U;
            }
            g_motor_control_debug.encoder_save_hal_status = (uint8_t)status;
            MotorControl_EnterFault(MOTOR_FAULT_CONFIG_STORAGE);
            return;
        }

        encoder_calibration_valid = true;
        g_motor_control_debug.encoder_save_stage = 5U;
        g_motor_control_debug.encoder_save_hal_status = (uint8_t)HAL_OK;
        interrupt_state = __get_PRIMASK();
        __disable_irq();
        encoder_calibration_save_pending = false;
        if (interrupt_state == 0U)
        {
            __enable_irq();
        }
        MotorControl_UpdateDebugState();
        return;
    }

    if (!encoder_calibration_requested)
    {
        return;
    }

    if ((motor_mode == MOTOR_CONTROL_FAULT) || !sampling_started ||
        !pwm_enabled || !current_sense_ready ||
        !BissEncoder_GetSnapshot(&encoder_snapshot) ||
        !encoder_snapshot.ready ||
        (encoder_snapshot.valid_age_ticks > ENCODER_STALE_LIMIT_TICKS))
    {
        encoder_calibration_requested = false;
        MotorControl_UpdateDebugState();
        return;
    }

    interrupt_state = __get_PRIMASK();
    __disable_irq();
    encoder_calibration_requested = false;
    if (EncoderCalibration_Start(&encoder_calibration))
    {
        g_motor_control_debug.encoder_terminal_kind = 0U;
        g_motor_control_debug.encoder_terminal_stage = 0U;
        g_motor_control_debug.encoder_terminal_hal_status = (uint8_t)HAL_OK;
        g_motor_control_debug.service_count_at_terminal = 0U;
        memset(&encoder_calibration_command, 0, sizeof(encoder_calibration_command));
        encoder_calibration_last_sequence = encoder_snapshot.sequence;
        encoder_calibration_active = true;
        run_state = MOTOR_RUN_STATE_ENCODER_CALIBRATING;
        current_reference_active.d = 0.0f;
        current_reference_active.q = 0.0f;
        CurrentPi_Reset(&current_pi);
    }
    if (interrupt_state == 0U)
    {
        __enable_irq();
    }
    MotorControl_UpdateDebugState();
}

HAL_StatusTypeDef MotorControl_RequestMode(MotorControlMode mode)
{
    uint32_t interrupt_state;

    if (!MotorControl_ModeIsImplemented(mode) || (motor_mode == MOTOR_CONTROL_FAULT) ||
        (((mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
          (mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)) && current_calibration_failed) ||
        encoder_calibration_active || encoder_calibration_requested ||
        encoder_calibration_save_pending || encoder_calibration_failure_pending)
    {
        return HAL_ERROR;
    }

    if ((mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) &&
        (!current_sense_ready || !encoder_calibration_valid ||
         !MotorControl_UpdateEncoderAngle(true)))
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

HAL_StatusTypeDef MotorControl_SwitchToOpenVoltage(
    float ud_pu,
    float uq_pu,
    float electrical_frequency_hz)
{
    if (!isfinite(ud_pu) || !isfinite(uq_pu) ||
        !isfinite(electrical_frequency_hz))
    {
        return HAL_ERROR;
    }

    MotorControl_SetOpenLoopCommand(ud_pu, uq_pu, electrical_frequency_hz);
    return MotorControl_RequestMode(MOTOR_CONTROL_OPEN_VOLTAGE);
}

HAL_StatusTypeDef MotorControl_SwitchToOpenAngleCurrent(
    float id_a,
    float iq_a,
    float electrical_frequency_hz)
{
    if (!isfinite(id_a) || !isfinite(iq_a) ||
        !isfinite(electrical_frequency_hz))
    {
        return HAL_ERROR;
    }

    MotorControl_SetCurrentCommand(id_a, iq_a, electrical_frequency_hz);
    return MotorControl_RequestMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
}

HAL_StatusTypeDef MotorControl_SwitchToEncoderAngleCurrent(float id_a,
                                                           float iq_a)
{
    if (!isfinite(id_a) || !isfinite(iq_a))
    {
        return HAL_ERROR;
    }

    MotorControl_SetEncoderCurrentCommand(id_a, iq_a);
    return MotorControl_RequestMode(MOTOR_CONTROL_ENCODER_ANGLE_CURRENT);
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

    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_START_ENTER,
        (uint8_t)requested_mode);

    if ((motor_mode != MOTOR_CONTROL_STOPPED) || sampling_started)
    {
        return HAL_ERROR;
    }

    status = HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    if (status != HAL_OK)
    {
        MotorControl_EnterFault(MOTOR_FAULT_ADC_START);
        return status;
    }
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_ADC_CALIBRATED,
        (uint8_t)requested_mode);

    status = Drv8323Board_EnableForPwm();
    if (status != HAL_OK)
    {
        MotorControl_EnterFault(MOTOR_FAULT_DRIVER);
        return status;
    }
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_DRIVER_ENABLED,
        (uint8_t)requested_mode);

    CurrentSenseCalibration_Start(&current_calibration,
                                  CURRENT_CALIBRATION_DISCARD_COUNT,
                                  CURRENT_CALIBRATION_SAMPLE_COUNT);
    current_sense_ready = false;
    current_calibration_failed = false;
    current_feedback_valid = false;
    run_state = MOTOR_RUN_STATE_CURRENT_CALIBRATING;
    Drv8323Board_SetCurrentCalibration(true);
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_CURRENT_CALIBRATION_STARTED,
        (uint8_t)requested_mode);

    status = HAL_ADCEx_InjectedStart_IT(&hadc1);
    if (status != HAL_OK)
    {
        Drv8323Board_SetCurrentCalibration(false);
        Drv8323Board_Disable();
        MotorControl_EnterFault(MOTOR_FAULT_ADC_START);
        return status;
    }
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_ADC_STARTED,
        (uint8_t)requested_mode);

    status = HAL_TIM_Base_Start_IT(&htim8);
    if (status != HAL_OK)
    {
        (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
        Drv8323Board_SetCurrentCalibration(false);
        Drv8323Board_Disable();
        MotorControl_EnterFault(MOTOR_FAULT_ADC_START);
        return status;
    }
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_TIM8_STARTED,
        (uint8_t)requested_mode);

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
    MotorStartupTrace_RecordCheckpoint(
        &startup_trace,
        MOTOR_STARTUP_STAGE_SAMPLING_STARTED,
        (uint8_t)requested_mode);
    MotorControl_UpdateDebugState();
    return HAL_OK;
}

HAL_StatusTypeDef MotorControl_Stop(void)
{
    HAL_StatusTypeDef result = HAL_OK;
    HAL_StatusTypeDef stop_status;

    g_motor_control_debug.motor_stop_stage = 1U;
    g_motor_control_debug.motor_stop_hal_status = (uint8_t)HAL_OK;
    motor_mode = MOTOR_CONTROL_STOPPED;
    run_state = MOTOR_RUN_STATE_STOPPED;
    requested_mode = motor_config.mode;
    /*
     * 校准终止时，ADC回调已先关闭三相PWM、DRV_CAL和DRV8323。
     * 前台只需停止触发/采样外设，不再对同一GPIO关断路径重入。
     * 其他停机场景仍执行完整的功率级安全关断。
     */
    if (!encoder_calibration_save_pending &&
        !encoder_calibration_failure_pending)
    {
        MotorControl_DisablePowerStage();
    }
    g_motor_control_debug.motor_stop_stage = 2U;

    if (sampling_started)
    {
        g_motor_control_debug.motor_stop_stage = 3U;
        stop_status = HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
        g_motor_control_debug.motor_stop_hal_status = (uint8_t)stop_status;
        if (stop_status != HAL_OK)
        {
            result = HAL_ERROR;
        }
        g_motor_control_debug.motor_stop_stage = 4U;
        stop_status = HAL_TIM_Base_Stop_IT(&htim8);
        g_motor_control_debug.motor_stop_hal_status = (uint8_t)stop_status;
        if (stop_status != HAL_OK)
        {
            result = HAL_ERROR;
        }
        g_motor_control_debug.motor_stop_stage = 5U;
        g_motor_control_debug.motor_stop_stage = 6U;
        stop_status = HAL_ADCEx_InjectedStop_IT(&hadc1);
        g_motor_control_debug.motor_stop_hal_status = (uint8_t)stop_status;
        if (stop_status != HAL_OK)
        {
            result = HAL_ERROR;
        }
        g_motor_control_debug.motor_stop_stage = 7U;
        sampling_started = false;
    }

    CurrentPi_Reset(&current_pi);
    encoder_calibration_requested = false;
    encoder_calibration_active = false;
    memset(&encoder_calibration_command, 0, sizeof(encoder_calibration_command));
    adc_age_ticks = 0U;
    overcurrent_count = 0U;
    g_motor_control_debug.motor_stop_stage = 8U;
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

    /*
     * 电流零偏校准阶段保持与已验证版本相同的 ADC/TIM8 启动路径。
     * 校准结束后才允许 SPI4 DMA 进入后台采集；HAL_BUSY 属于正常状态。
     */
    if (MotorRuntimePolicy_EncoderAcquisitionAllowed(run_state))
    {
        BissEncoder_ControlTick();
        (void)BissEncoder_StartRead();
        (void)BissEncoder_GetSnapshot(&encoder_snapshot);
    }

    if (!isfinite(dt_s) || (dt_s <= 0.0f))
    {
        if ((motor_mode == MOTOR_CONTROL_OPEN_VOLTAGE) ||
            (motor_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
            (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT) ||
            encoder_calibration_active)
        {
            MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        }
        return;
    }

    /* 标定期间 PWM 只能由 ADC 电流快环写入，TIM8 不再推进原模式。 */
    if (encoder_calibration_active || encoder_calibration_save_pending ||
        encoder_calibration_failure_pending)
    {
        MotorControl_UpdateDebugState();
        return;
    }

    /*
     * .mode直接选择模式3时，在这里等待SPI4 DMA发布第一帧一致快照。
     * 等待期间三相PWM尚未使能，不会输出未知角度的电压矢量。
     */
    if ((run_state == MOTOR_RUN_STATE_READY) &&
        (motor_mode == MOTOR_CONTROL_STOPPED) &&
        (requested_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT))
    {
        if (MotorControl_UpdateEncoderAngle(true))
        {
            MotorControl_StartSelectedMode();
        }
        MotorControl_UpdateDebugState();
        return;
    }

    /*
     * 先验证当前模式，再处理运行时切换。上电电流校准期间motor_mode仍为
     * STOPPED；若在此阶段调用ApplyModeRequest，它会因电流反馈未就绪而
     * 将requested_mode错误回退为STOPPED，破坏配置的待启动模式。
     */
    if (!MotorRuntimePolicy_ModeRequestAllowed(motor_mode))
    {
        MotorControl_UpdateDebugState();
        return;
    }

    MotorControl_ApplyModeRequest();
    if (motor_mode == MOTOR_CONTROL_FAULT)
    {
        return;
    }

    if ((motor_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
        (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT))
    {
        ++adc_age_ticks;
        if (adc_age_ticks > CURRENT_ADC_TIMEOUT_TICKS)
        {
            MotorControl_EnterFault(MOTOR_FAULT_ADC_TIMEOUT);
            return;
        }

        if (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)
        {
            if (!MotorControl_UpdateEncoderAngle(false))
            {
                MotorControl_EnterFault(MOTOR_FAULT_ENCODER_STALE);
                return;
            }
            active_electrical_angle_pu = encoder_angle_sample.electrical_angle_pu;
        }
        else
        {
            open_loop_state.target_frequency_hz =
                (run_state == MOTOR_RUN_STATE_ALIGNING) ?
                    0.0f : current_target_frequency_hz;
            OpenLoop_Step(&open_loop_state, dt_s, &open_voltage);
            active_electrical_angle_pu = open_loop_state.electrical_angle_pu;
        }
    }
    else
    {
        open_loop_state.target_frequency_hz = open_target_frequency_hz;
        MotorControl_UpdateOpenVoltageBlend(dt_s);
        OpenLoop_Step(&open_loop_state, dt_s, &open_voltage);
        (void)MotorControl_WriteVoltage(
            &open_voltage, open_loop_state.electrical_angle_pu);
    }
    MotorControl_UpdateDebugState();
}

void MotorControl_CurrentSampleComplete(uint32_t phase_a_raw,
                                        uint32_t phase_b_raw,
                                        float dt_s)
{
    EncoderCalibrationInput calibration_input;
    EncoderCalibrationResult calibration_result;
    MotorVoltageDq zero_voltage = {0.0f, 0.0f};
    float electrical_angle_pu;
    float open_voltage_magnitude;

    g_motor_control_debug.phase_a_raw = phase_a_raw;
    g_motor_control_debug.phase_b_raw = phase_b_raw;

    if (run_state == MOTOR_RUN_STATE_CURRENT_CALIBRATING)
    {
        MotorStartupTrace_ObserveCalibration(
            &startup_trace,
            (uint8_t)requested_mode,
            current_calibration.sample_count);
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

    if (encoder_calibration_save_pending || encoder_calibration_failure_pending)
    {
        return;
    }

    /*
     * 编码器校准路径已实现，但当前尚未完成实机验证。它使用强制电角度
     * 和小Id电流寻找零点/方向；验证前不要把“编码器可读”视为“已校准”。
     */
    if (encoder_calibration_active)
    {
        (void)BissEncoder_GetSnapshot(&encoder_snapshot);
        calibration_input.current_sense_ready = current_sense_ready;
        calibration_input.encoder_valid =
            (encoder_snapshot.sequence != 0U) &&
            (encoder_snapshot.sequence != encoder_calibration_last_sequence) &&
            (encoder_snapshot.valid_age_ticks <= ENCODER_STALE_LIMIT_TICKS);
        calibration_input.position_raw = encoder_snapshot.position_raw;
        if (calibration_input.encoder_valid)
        {
            encoder_calibration_last_sequence = encoder_snapshot.sequence;
        }

        encoder_calibration_command = EncoderCalibration_Step(
            &encoder_calibration, &calibration_input, dt_s);
        if (encoder_calibration.state == ENCODER_CAL_COMPLETE)
        {
            g_motor_control_debug.encoder_terminal_kind = 1U;
            g_motor_control_debug.encoder_terminal_stage = 1U;
            g_motor_control_debug.service_count_at_terminal =
                g_motor_control_debug.foreground_service_count;
            if (EncoderCalibration_GetResult(&encoder_calibration,
                                             &calibration_result) &&
                MotorCalibrationConfig_Build(
                    &encoder_saved_config,
                    calibration_result.electrical_zero_raw,
                    calibration_result.encoder_direction,
                    MOTOR_POLE_PAIRS))
            {
                encoder_calibration_save_pending = true;
                g_motor_control_debug.service_count_at_save_request =
                    g_motor_control_debug.foreground_service_count;
                g_motor_control_debug.encoder_save_stage = 1U;
                g_motor_control_debug.encoder_save_hal_status =
                    (uint8_t)HAL_OK;
            }
            else
            {
                encoder_calibration_failure_pending = true;
            }
            encoder_calibration_active = false;
            MotorControl_DisablePowerStage();
            g_motor_control_debug.encoder_terminal_stage = 2U;
            MotorControl_QuiesceRealtimeInterrupts();
            g_motor_control_debug.encoder_terminal_stage = 3U;
            MotorControl_UpdateDebugState();
            g_motor_control_debug.encoder_terminal_stage = 4U;
            return;
        }
        if (encoder_calibration.state == ENCODER_CAL_FAILED)
        {
            g_motor_control_debug.encoder_terminal_kind = 2U;
            g_motor_control_debug.encoder_terminal_stage = 1U;
            g_motor_control_debug.service_count_at_terminal =
                g_motor_control_debug.foreground_service_count;
            encoder_calibration_failure_pending = true;
            encoder_calibration_active = false;
            MotorControl_DisablePowerStage();
            g_motor_control_debug.encoder_terminal_stage = 2U;
            MotorControl_QuiesceRealtimeInterrupts();
            g_motor_control_debug.encoder_terminal_stage = 3U;
            MotorControl_UpdateDebugState();
            g_motor_control_debug.encoder_terminal_stage = 4U;
            return;
        }

        adc_age_ticks = 0U;
        electrical_angle_pu = encoder_calibration_command.forced_electrical_angle_pu;
        if (!MotorControl_UpdateCurrentMeasurement(
                phase_a_raw, phase_b_raw, electrical_angle_pu))
        {
            return;
        }
        if (encoder_calibration_command.active)
        {
            (void)MotorControl_RunCurrentLoop(dt_s, electrical_angle_pu);
        }
        else
        {
            /* WAIT_VALID 的短暂阶段明确输出零矢量，绝不保留原模式电压。 */
            (void)MotorControl_WriteVoltage(&zero_voltage, electrical_angle_pu);
        }
        MotorControl_UpdateDebugState();
        return;
    }

    if ((motor_mode == MOTOR_CONTROL_STOPPED) || (motor_mode == MOTOR_CONTROL_FAULT) ||
        !current_sense_ready)
    {
        return;
    }

    /*
     * 电压开环允许继续使用原来的完整 SVPWM 范围。当开环矢量超过两电阻
     * 固定采样点能保证的 0.45 pu 范围时，本次低侧样本可能不代表相电流，
     * 因而只暂停软件电流监视并拒绝切入电流环，绝不能误判后破坏开环运行。
     */
    open_voltage_magnitude = sqrtf((last_voltage_pu.ud_pu * last_voltage_pu.ud_pu) +
                                   (last_voltage_pu.uq_pu * last_voltage_pu.uq_pu));
    if ((motor_mode == MOTOR_CONTROL_OPEN_VOLTAGE) &&
        (open_voltage_magnitude > MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU))
    {
        current_feedback_valid = false;
        overcurrent_count = 0U;
        g_motor_control_debug.overcurrent_count = 0U;
        return;
    }

    adc_age_ticks = 0U;
    if (!isfinite(dt_s) || (dt_s <= 0.0f))
    {
        MotorControl_EnterFault(MOTOR_FAULT_CONTROL_MATH);
        return;
    }

    if (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT)
    {
        if (!MotorControl_UpdateEncoderAngle(false))
        {
            MotorControl_EnterFault(MOTOR_FAULT_ENCODER_STALE);
            return;
        }
        electrical_angle_pu = encoder_angle_sample.electrical_angle_pu;
    }
    else
    {
        electrical_angle_pu = open_loop_state.electrical_angle_pu;
    }

    if (!MotorControl_UpdateCurrentMeasurement(
            phase_a_raw, phase_b_raw, electrical_angle_pu))
    {
        return;
    }

    if ((motor_mode == MOTOR_CONTROL_OPEN_ANGLE_CURRENT) ||
        (motor_mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT))
    {
        (void)MotorControl_RunCurrentLoop(dt_s, electrical_angle_pu);
    }
    MotorControl_UpdateDebugState();
}
