# H7 角度开环、电流闭环控制设计

日期：2026-09-10  
目标工程：STM32H743 + DRV8323 + TIM8 六路互补 PWM

## 1. 目标与边界

在已经通过电机实验的 H7 电压开环工程上，新增“电角度开环、电流闭环”模式。原有电压开环、SVPWM、TIM8 PWM 输出和 DRV8323 驱动层必须继续可用；运行过程中允许在电压开环与电流闭环之间请求安全切换。

本阶段不读取编码器参与控制，但统一定义电机极对数为 10，为后续编码器电角度和速度环预留接口。本阶段不实现速度闭环。

## 2. 已确认的硬件参数

- A 相采样：PB1 / ADC1_INP5。
- B 相采样：PB0 / ADC1_INP9。
- C 相无采样电阻，由 `Ic = -(Ia + Ib)` 重构。
- 两个低侧采样电阻均为 20 mΩ。
- ADC 分辨率 12 bit，参考电压按 3.3 V 计算。
- DRV8323 CSA 使用双向采样基准 `VREF/2`，CSA 增益配置为 5 V/V。
- PWM 与电流环频率为 10 kHz，控制周期 100 us。
- 标称母线电压暂按 48 V；当前没有可靠的实时母线电压反馈。
- 未使用 BKIN，保持现有硬件配置，不在本次修改范围内。

理论换算系数：

```c
current_a_per_count = 3.3f / (4095.0f * 5.0f * 0.020f);
/* 约 0.0080586 A/count */
```

DRV8323 CSA 的 5/10/20/40 V/V 可选增益及 `VREF_DIV` 行为以 TI DRV8323 数据手册为依据：<https://www.ti.com/lit/ds/symlink/drv8323.pdf>。

## 3. 软件分层

新增模块按职责拆分，不把采样、坐标变换和 PI 全部堆入中断：

- `motor_params`：硬件、电机、限制值和默认控制参数。
- `current_sense`：ADC 原始值、偏置校准、电流换算、两相采样有效性检查。
- `foc_transform`：Clarke、Park 及必要的反变换数学函数，不依赖 HAL。
- `current_pi`：d/q 轴 PI、圆形电压限幅、抗积分饱和和无扰初始化。
- `motor_control`：模式、启动状态机、控制链编排、故障锁存和切换请求。
- 继续复用 `open_loop`、`svpwm`、`pwm_3ph` 和 `drv8323_board`。

HAL 回调只负责读取转换结果并调用控制编排入口。硬件寄存器操作仍限制在相应的适配层。

## 4. 控制模式与安全切换

控制模式定义为：

```c
typedef enum
{
    MOTOR_CONTROL_STOPPED,
    MOTOR_CONTROL_OPEN_VOLTAGE,
    MOTOR_CONTROL_OPEN_ANGLE_CURRENT,
    MOTOR_CONTROL_ENCODER_CURRENT,
    MOTOR_CONTROL_FAULT
} MotorControlMode;
```

`MOTOR_CONTROL_ENCODER_CURRENT` 仅预留，不在本阶段进入运行状态。

对外提供模式请求接口。请求由 10 kHz 控制边界消费，不允许主循环或通信代码直接改 CCR：

```c
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_VOLTAGE);
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
```

两种运行模式共用同一个开环电角度状态，切换时不清零角度和频率：

- 电压开环切换到电流闭环：必须已经完成电流偏置校准；按当前电流反馈计算 d/q 电流，并用当前开环 `Ud/Uq` 预置 PI 积分器，降低输出突变。
- 电流闭环切换到电压开环：用 PI 最后一次 `Ud/Uq` 初始化开环电压，然后斜坡到用户设定的开环电压命令。
- 模式切换失败时保持原模式，不产生半切换状态。
- 默认上电模式仍为现有电压开环，保证原工程的默认行为。

## 5. PWM 同步采样

使用 TIM8 CH4 作为内部采样时刻发生器，不输出到 GPIO。CH4 在中心对齐计数上升过程约 95% 位置产生触发，ADC1 注入组仅保留两级：

1. PB1 / `I_A`。
2. PB0 / `I_B`。

ADC 注入转换完成中断驱动电流闭环，确保使用本周期的新鲜采样；TIM8 更新中断只在电压开环模式执行原有开环计算。两条路径不能在同一周期同时更新 PWM。

由于使用同一个 ADC 顺序采样两相，为低侧采样保留建立窗口，电流闭环的初始最大电压矢量为 0.15 pu，确认正常后可提高，但软件绝对上限为 0.45 pu。0.45 pu 对应的 SVPWM 占空比范围约为 11%～89%。后续需要更高调制度时，再升级为 ADC1/ADC2 同步采样或动态采样点。

## 6. 偏置校准

启动电流采样时：

1. 使能并配置 DRV8323，CSA 增益设为 5 V/V。
2. 拉高板级 `DRV_CAL`。
3. 启动 TIM8 计数、CH4 内部触发和 ADC 注入中断，但保持 TIM8 主 PWM 输出关闭。
4. 丢弃前 16 组样本。
5. 累计 256 组样本，计算 A/B 两路平均偏置和采样波动。
6. 校验偏置没有靠近 ADC 上下电源轨、波动没有超过配置阈值。
7. 拉低 `DRV_CAL`，将采样状态置为可用。

校准失败时禁止进入电流闭环，但保留原电压开环的可用性。DRV8323 硬件保护仍负责功率级的底层保护。

## 7. 电流重构与坐标变换

电流极性集中配置，默认均为正：

```c
ia = CURRENT_IA_POLARITY * (adc_ia - offset_ia) * current_a_per_count;
ib = CURRENT_IB_POLARITY * (adc_ib - offset_ib) * current_a_per_count;
ic = -(ia + ib);
```

采用幅值不变 Clarke/Park 形式，并与当前 SVPWM 逆 Park 方向配对：

```c
i_alpha = ia;
i_beta  = (ia + 2.0f * ib) / sqrtf(3.0f);

id =  i_alpha * cos_theta + i_beta * sin_theta;
iq = -i_alpha * sin_theta + i_beta * cos_theta;
```

如果实机验证发现电流反馈为正反馈，只修改 A/B 极性或相序配置，不在算法内部散布符号修正。

## 8. 保守 PI 与后续整定

由于电机实际相电阻和相电感尚未确认，首版不使用按假定参数计算的 500 Hz 高带宽 PI。采用偏保守、物理量单位明确的默认值：

```c
kp_v_per_a       = 0.05f;
ki_v_per_a_s     = 20.0f;
kaw_per_s        = 100.0f;
sample_period_s  = 0.0001f;
```

PI 输入单位为 A，输出单位为 V。d/q 轴未限幅输出完成后进行联合圆形限幅，保持电压矢量方向；最终除以当前母线电压估计值再交给 SVPWM。

采用回算抗积分饱和：

```c
u_unsat = kp * error + integrator;
integrator += (ki * error + kaw * (u_sat - u_unsat)) * sample_period_s;
```

参数保存在可检查的控制配置中，不在每次 PI 调用时重复赋值。提供只在安全条件下生效的参数设置接口，并拒绝负值、NaN、Inf 或超范围参数。

首轮参数和限制：

- 固定角度定位：`Id_ref = 0.5 A`，`Iq_ref = 0 A`，持续约 0.5 s。
- 进入旋转后：`Id_ref`降到 0，`Iq_ref`默认斜坡到 0.3 A。
- `Iq_ref`变化率默认 1 A/s。
- 用户允许的 d/q 电流指令绝对值最大 2 A。
- 软件相电流过流阈值 10 A。
- 初始电压矢量限制 0.15 pu，调试确认后才允许提高，绝对上限 0.45 pu。
- 默认目标电角频率 1 Hz，频率斜坡 20 Hz/s。

实机整定顺序为：先确认偏置、相序和反馈符号，再逐步提高 `Kp`；确认比例响应稳定后再提高 `Ki`。不能同时大幅修改两者。

## 9. 启动状态机与故障

电流闭环启动状态为：

```text
STOPPED -> DRIVER_CONFIG -> CURRENT_CALIBRATING
        -> READY -> ALIGN -> CURRENT_RUN
任意异常 -> FAULT
```

故障包括：

- 任意测量相或重构相连续两个样本超过 10 A。
- ADC 原始值接近电源轨。
- ADC 采样超时。
- 偏置校准失败。
- PI 或坐标变换出现 NaN/Inf。
- DRV8323 配置或通信失败。

ADC越界、计算异常和驱动错误立即关断；软件过流连续两次确认后关断，以抑制单点噪声误触发。进入故障后关闭 TIM8 PWM 输出、禁止 DRV8323、清零 PI 并锁存具体故障码，必须显式清除。

## 10. 调试可观测性

提供稳定命名的调试快照，便于 Keil/Ozone 观察：

- ADC A/B 原始值与校准偏置。
- `Ia/Ib/Ic`、`Ialpha/Ibeta`、`Id/Iq`。
- `Id_ref/Iq_ref` 和两轴误差。
- PI 比例项、积分项、未限幅及限幅后的 `Ud/Uq`。
- 电角度、电角频率、当前模式、启动状态。
- 饱和标志、过流计数和锁存故障码。

中断中不执行 `printf`、阻塞式 SPI 或编码器读取。

## 11. 验证要求

先用主机侧单元测试验证与 HAL 无关的代码：

- ADC 偏置和安培换算。
- `Ic` 重构。
- Clarke/Park 已知角度向量。
- PI 正负误差、积分和复位。
- 联合电压矢量限幅及抗积分饱和。
- 电角度回绕和频率斜坡。
- 开环/电流环切换的输出连续性。
- 无效配置和故障状态转换。

随后执行 Keil 工程全量编译，确认 0 error。硬件首次验证顺序：无母线校准检查、低压限流上电、0 A 指令、固定角定位、0.3 A 低速运行，最后才逐步增加电流和电压限制。

## 12. 对 F407 逻辑的修正

H7 实现明确避免以下原工程问题：

- 不在一次 ADC 回调链中重复执行电流采样和坐标变换。
- 不在 PI 每次调用时覆盖增益。
- 不使用存在历史误差更新问题的通用 PID 实现。
- 不在快速中断中执行阻塞式编码器 SPI。
- 不混用 12/16 bit ADC 满量程、5/40 倍增益等互相矛盾的宏。
- 不把极对数 10 硬编码在角度函数中；统一由电机参数模块提供。
- 不对 d/q 电压分别裁剪；使用联合矢量限幅和抗积分饱和。

