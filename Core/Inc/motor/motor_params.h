/**
 * @file motor_params.h
 * @brief 电流采样和电机控制的集中参数。
 *
 * 所有与硬件比例、保护阈值和首次上电保守设置有关的参数集中在此处，
 * 避免在 ADC、PI 和状态机中出现互相矛盾的常量。
 */
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

/* 电机参数。编码器接入后，机械角必须统一使用该极对数转换成电角度。 */
#define MOTOR_POLE_PAIRS                 (10U)

/* ADC 与 DRV8323 电流采样链。 */
#define MOTOR_ADC_REFERENCE_V            (3.3f)
#define MOTOR_ADC_FULL_SCALE_COUNT       (4095.0f)
#define MOTOR_ADC_MID_SCALE_COUNT        (2048.0f)
#define MOTOR_SHUNT_RESISTANCE_OHM       (0.020f)
#define MOTOR_CSA_GAIN_V_PER_V           (5.0f)
#define MOTOR_CURRENT_IA_POLARITY        (1.0f)
#define MOTOR_CURRENT_IB_POLARITY        (1.0f)
#define MOTOR_CURRENT_A_PER_COUNT        \
    (MOTOR_ADC_REFERENCE_V /             \
     (MOTOR_ADC_FULL_SCALE_COUNT * MOTOR_CSA_GAIN_V_PER_V * MOTOR_SHUNT_RESISTANCE_OHM))

/* 10 kHz 电流环与标称母线参数。 */
#define MOTOR_CONTROL_PERIOD_S           (0.0001f)
#define MOTOR_NOMINAL_VBUS_V             (24.0f)
#define MOTOR_POLE_VOLTAGE_LIMIT_START_PU (0.15f)
#define MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU   (0.45f)

/* 首次上电采用低电流、慢斜坡；提高前必须先验证采样方向与闭环稳定性。 */
#define MOTOR_CURRENT_ALIGN_A            (0.5f)
#define MOTOR_CURRENT_START_IQ_A         (0.3f)
#define MOTOR_CURRENT_COMMAND_LIMIT_A    (2.0f)
#define MOTOR_CURRENT_COMMAND_SLEW_A_PER_S (1.0f)
#define MOTOR_CURRENT_TRIP_A             (10.0f)

/* 未知电机参数条件下的保守电流 PI 初值，输出单位为伏特。 */
#define MOTOR_CURRENT_PI_KP_V_PER_A       (0.05f)
#define MOTOR_CURRENT_PI_KI_V_PER_A_S     (20.0f)
#define MOTOR_CURRENT_PI_KAW_PER_S        (100.0f)

/*
 * 编码器电角度校准参数。校准只使用 d 轴小电流，绝不施加 q 轴转矩命令；
 * 方向探测的 0.1 pu 电角度对应 10 对极电机约 1% 机械转角。
 */
/* 0.8 A 已在角度开环/电流闭环实机测试中验证可使转子缓慢移动。 */
#define MOTOR_ENCODER_ALIGN_CURRENT_A          (0.8f)
#define MOTOR_ENCODER_ALIGN_CURRENT_MAX_A      (0.8f)
#define MOTOR_ENCODER_ALIGN_RAMP_S             (0.4f)
#define MOTOR_ENCODER_ALIGN_SETTLE_S           (0.5f)
#define MOTOR_ENCODER_ALIGN_SAMPLE_COUNT       (128U)
#define MOTOR_ENCODER_ALIGN_STABILITY_COUNT    (128U)
/*
 * 先把强制电角度缓慢推到 0.2 pu，再从同一方向回到 0 pu。
 * 这个预扫动用来克服减速器/负载静摩擦，避免直接在 0 pu 处取样
 * 时转子其实仍停在上一次的位置。
 */
#define MOTOR_ENCODER_PREALIGN_STEP_PU          (0.2f)
#define MOTOR_ENCODER_PREALIGN_RAMP_S           (0.4f)
/*
 * 0.1 pu 在减速器静摩擦下只能产生 40..418 counts 且重复性不足。
 * 保持 0.8 A 上限不变，改用 0.2 pu/400 ms 缓慢探测；10对极时
 * 理论电机轴位移约 2621 counts（7.2度机械角）。
 */
#define MOTOR_ENCODER_DIRECTION_STEP_PU        (0.2f)
#define MOTOR_ENCODER_DIRECTION_RAMP_S         (0.4f)
#define MOTOR_ENCODER_DIRECTION_MIN_COUNT      (256)
#define MOTOR_ENCODER_DIRECTION_MAX_COUNT      (4096)
/* 方向探测后回到 0 pu，两次零点平均值必须在该范围内。 */
#define MOTOR_ENCODER_RETURN_RAMP_S             (0.4f)
#define MOTOR_ENCODER_RETURN_MAX_ERROR_COUNT    (256)
/* 回零验证后保持 0 pu 电角度，缓慢撤掉 Id，减小减速器回弹。 */
#define MOTOR_ENCODER_RELEASE_RAMP_S            (0.4f)
#define MOTOR_ENCODER_CAL_INVALID_LIMIT_TICKS  (100U)
#define MOTOR_ENCODER_CAL_STATE_TIMEOUT_S      (2.0f)

#endif /* MOTOR_PARAMS_H */
