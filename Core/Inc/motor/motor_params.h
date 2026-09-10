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
#define MOTOR_NOMINAL_VBUS_V             (48.0f)
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

#endif /* MOTOR_PARAMS_H */
