/**
 * @file motor_params.h
 * @brief 电流采样和电机控制的集中参数。
 *
 * 所有与硬件比例、保护阈值和首次上电保守设置有关的参数集中在此处，
 * 避免在 ADC、PI 和状态机中出现互相矛盾的常量。
 *
 * 更换电机时至少逐项复核：极对数、电流上限/过流阈值、电流PI、校准电流；
 * 更换功率板时至少复核：母线电压、采样电阻、放大倍数、ADC参考和电流极性；
 * 更换PWM频率时必须同时修改TIM8配置与MOTOR_CONTROL_PERIOD_S。
 */
#ifndef MOTOR_PARAMS_H
#define MOTOR_PARAMS_H

/*
 * [换电机必改] 电机极对数，不是机械极数。
 * 电角度 = 机械角度 * 极对数。该值还写入Flash校准记录；修改后旧记录会
 * 校验失败，必须重新执行编码器零点/方向校准，不能沿用旧电机的零点。
 */
#define MOTOR_POLE_PAIRS                 (10U)

/*
 * [换功率板/采样电路必改] ADC与DRV8323电流采样链。
 * 换算关系：A/count = Vref / (ADC满量程 * CSA增益 * 采样电阻)。
 * MOTOR_CSA_GAIN_V_PER_V必须与DRV8323寄存器中实际配置的CSA增益一致。
 * 若正电流命令下反馈方向相反，应先核对相序/采样通道，再修改POLARITY；
 * 不要用PI符号或编码器方向去掩盖电流采样接反。
 */
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

/*
 * [换母线或PWM频率必改] 10 kHz电流环与标称母线参数。
 * 当前工程没有实时母线电压采样，PI输出由“伏特”除以该标称值转成pu，
 * 所以MOTOR_NOMINAL_VBUS_V必须填写实际母线电压，否则输出比例会错误。
 * PERIOD必须与TIM8更新事件严格一致，当前为100 us。
 */
#define MOTOR_CONTROL_PERIOD_S           (0.0001f)
#define MOTOR_NOMINAL_VBUS_V             (24.0f)
#define MOTOR_POLE_VOLTAGE_LIMIT_START_PU (0.15f)
#define MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU   (0.45f)

/*
 * [换电机/负载必调] 电流命令、斜坡与软件过流保护。
 * COMMAND_LIMIT限制API目标；TRIP是运行保护，两者都不能超过电机、MOS、
 * 采样放大器和电源允许值。先用较小Iq确认方向，再逐步提高。
 */
#define MOTOR_CURRENT_ALIGN_A            (0.5f)
#define MOTOR_CURRENT_START_IQ_A         (0.3f)
#define MOTOR_CURRENT_COMMAND_LIMIT_A    (2.0f)
#define MOTOR_CURRENT_COMMAND_SLEW_A_PER_S (1.0f)
#define MOTOR_CURRENT_TRIP_A             (10.0f)

/*
 * [换电机通常需要重调] d/q共用的电流PI，输出单位为伏特。
 * 电阻、电感、PWM频率或母线变化都会影响合适增益。调试顺序通常是先Kp、
 * 再Ki，观察阶跃、噪声和voltage_saturated；Kaw只负责饱和后的抗积分饱和。
 */
#define MOTOR_CURRENT_PI_KP_V_PER_A       (0.05f)
#define MOTOR_CURRENT_PI_KI_V_PER_A_S     (20.0f)
#define MOTOR_CURRENT_PI_KAW_PER_S        (100.0f)

/*
 * 编码器电角度校准参数。校准只使用 d 轴小电流，绝不施加 q 轴转矩命令；
 * 方向探测的0.2 pu电角度在10对极电机上约等于0.02机械圈。
 * [换电机/减速器/负载必复核] 校准电流必须足以克服静摩擦，又不能造成
 * 过大冲击；角度步长和允许计数范围与极对数、编码器分辨率和负载相关。
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

/*
 * [换电机/编码器/减速器/负载后必须复核] 速度环使用电机轴编码器的机械转速。
 * 速度任务由10 kHz快速边界每10次分频得到1 kHz；PERIOD从电流环周期推导，
 * 禁止再维护一份彼此可能漂移的固定周期。编码器若改装到减速器输出端，不能只
 * 修改减速比：必须重新定义速度接口的物理含义并重新整定滤波器和速度PI。
 */
#define MOTOR_ENCODER_COUNTS_PER_TURN       (131072U)
#define MOTOR_SPEED_CONTROL_DIVIDER         (10U)
#define MOTOR_SPEED_CONTROL_PERIOD_S        \
    (MOTOR_CONTROL_PERIOD_S * (float)MOTOR_SPEED_CONTROL_DIVIDER)
#define MOTOR_SPEED_COMMAND_LIMIT_RPM       (100.0f)
#define MOTOR_SPEED_COMMAND_SLEW_RPM_PER_S  (20.0f)
#define MOTOR_SPEED_FILTER_CUTOFF_HZ        (20.0f)
#define MOTOR_SPEED_CURRENT_COMMAND_SLEW_A_PER_S (10.0f)

/*
 * [换电机/负载/减速器后必须重调] 速度PI的输出单位为A，下列值只用于首次低速验证。
 * 它们没有照搬F407中依赖隐含调用频率的离散增益。当前实机试验将转矩电流上限提高到0.7 A，
 * 确认方向、速度反馈和无持续饱和后，再逐步整定Kp、Ki，最后才考虑提高限幅。
 */
#define MOTOR_SPEED_PI_KP_A_PER_RPM         (0.03f)
#define MOTOR_SPEED_PI_KI_A_PER_RPM_S       (0.015f)
#define MOTOR_SPEED_PI_KAW_PER_S             (10.0f)
#define MOTOR_SPEED_IQ_LIMIT_A               (0.7f)

/* 单圈轴侧位置环：1 kHz P 控制输出速度目标，仍受现有速度/电流环限幅。 */
#define MOTOR_POSITION_KP_RPM_PER_DEG        (0.4f)
#define MOTOR_POSITION_SPEED_LIMIT_RPM       (20.0f)
#define MOTOR_POSITION_TOLERANCE_DEG         (0.5f)

#endif /* MOTOR_PARAMS_H */
