# H743：20 kHz控制、rad/s速度与24/48 V母线配置

本指南对应2026-10-28的软件修改。旧文档中的10 kHz电流环、1 kHz速度环、rpm接口及固定24 V配置属于旧基线；当前使用以本指南和源码为准。历史实机验证不代表20 kHz/48 V组合已完成实机验证。

## 1. 关键位置与频率

| 配置/行为 | 代码位置（相对仓库根目录） | 当前值 |
|---|---|---|
| PWM硬件时基、ADC触发点 | `Core/Src/tim.c`，`MX_TIM8_Init()`；`emptytest.ioc` | PSC=0，中心对齐ARR=6000，RCR=1，CCR4=5700 |
| 快控制/电流环周期 | `Core/Inc/motor/motor_params.h`，`MOTOR_CONTROL_PERIOD_S` | 50 μs，20 kHz |
| 速度/位置环周期 | 同文件，`MOTOR_SPEED_CONTROL_DIVIDER`、`MOTOR_SPEED_CONTROL_PERIOD_S` | 10分频，500 μs，2 kHz |
| 编码器采集频率 | 同文件，`MOTOR_ENCODER_CONTROL_DIVIDER` | 2分频，10 kHz |
| 分频行为 | `Core/Src/motor/motor_runtime_policy.c` | 速度任务每第10次运行；编码器首帧立即请求，其后每2次请求 |
| 中断入口 | `Core/Src/main.c`，TIM8更新/ADC注入完成回调 | 两个入口均使用50 μs软件周期 |
| 实际控制与编码器调度 | `Core/Src/motor/motor_control.c` | 电流PI在ADC回调内，速度/位置任务从TIM8分频 |

TIM8时钟240 MHz，中心对齐频率为240 MHz/(2×6000)=20 kHz。不要把电角频率、20 Hz测速低通截止频率或100 Hz遥测误认为控制频率。不要只改周期宏而不改TIM8和采样点。

2026-10-29修正：ADC的`EOCSelection`必须为`ADC_EOC_SEQ_CONV`。H7 HAL依据该选项使能JEOS（注入序列全部完成）中断；使用`ADC_EOC_SINGLE_CONV`会使能JEOC，可能在A相完成而B相尚未更新时进入回调，破坏双相数据一致性和每PWM周期一次控制的约束。`main.c`的ADC回调还检查JEOS，拒绝提前的单通道回调。宿主测试从生产`main.c`提取实际回调，验证旧B样本不会被使用。

编码器SPI4仍为120 MHz/128=0.9375 MHz；48位传输约51.2 μs，因此保持每100 μs发起一次请求。年龄维护和DMA超时检查仍每个允许采集的50 μs快周期执行。读取被禁止时复位采集分频；电流零偏校准期间不采集编码器。

## 2. 选择24 V或48 V

在`Core/Inc/motor/motor_params.h`只修改：

```c
#ifndef MOTOR_BUS_VOLTAGE_SELECT_V
#define MOTOR_BUS_VOLTAGE_SELECT_V (24) /* 改成48选择48 V */
#endif
```

也可在Keil C/C++的Define中加入`MOTOR_BUS_VOLTAGE_SELECT_V=48`，覆盖头文件默认值。只允许整数24或48，36、24.0f等配置编译失败。修改后重新构建并下载；这不是运行时切换，也不会检测实际接入电源。

| 项目 | 24 V | 48 V |
|---|---:|---:|
| `MOTOR_NOMINAL_VBUS_V` | 24 V | 48 V |
| 电流环启动限幅 | 3.6 V / 0.15 pu | 3.6 V / 0.075 pu |
| 电流环允许的最大限幅 | 10.8 V / 0.45 pu | 10.8 V / 0.225 pu |
| 预置开环Uq | 1.92 V / 0.08 pu | 1.92 V / 0.04 pu |
| 退回开环时的电压斜坡 | 120 V/s / 5 pu/s | 120 V/s / 2.5 pu/s |

`MOTOR_POLE_VOLTAGE_LIMIT_START_V`、`MOTOR_POLE_VOLTAGE_LIMIT_MAX_V`控制绝对电压限幅。最大标幺限幅还会裁剪到固定低侧采样允许的0.45 pu，避免浮点舍入越过采样边界。`MOTOR_CURRENT_SAMPLING_VOLTAGE_LIMIT_PU`独立描述采样窗口；切换母线时不能将它跟着减半。

`MotorControl_SetCurrentVoltageLimit()`仍接收pu，不能传入伏特。24 V下允许上限约0.45，48 V下约0.225。当前启动实际使用3.6 V上限，不是自动使用10.8 V。

当前没有实时母线电压采样；所选配置必须与真实供电一致。软件配置不证明MOS、电容、驱动板和其他器件支持48 V。

## 3. rad/s接口与参数

所有速度均指电机轴机械角速度：`rad/s = rpm × 2π/60`。位置给定、误差和反馈仍使用度；FOC角度仍使用pu。

| 参数/字段 | 当前值/含义 |
|---|---|
| `MOTOR_SPEED_COMMAND_LIMIT_RAD_S` | 10.4719755 rad/s，等效旧100 rpm |
| `MOTOR_SPEED_COMMAND_SLEW_RAD_S2` | 2.0943951 rad/s²，等效旧20 rpm/s |
| `MOTOR_SPEED_PI_KP_A_PER_RAD_S` | 0.2864789 A/(rad/s) |
| `MOTOR_SPEED_PI_KI_A_PER_RAD` | 0.14323945 A/rad |
| `MOTOR_POSITION_KP_RAD_S_PER_DEG` | 0.04188790 (rad/s)/° |
| `MOTOR_POSITION_SPEED_LIMIT_RAD_S` | 2.0943951 rad/s |
| `speed_target_rad_s` | 外部速度目标 |
| `speed_active_target_rad_s` | 斜坡后速度目标 |
| `speed_filtered_rad_s` | 滤波后的速度反馈 |

API函数名保留，但参数语义已改变，外部调用代码必须迁移：

```c
MotorControl_SetSpeedCommand(5.2359878f); /* 等效旧50 rpm */
/* 仅在具备校准和电流反馈等条件、且明确要转动时调用模式切换： */
MotorControl_SwitchToEncoderSpeedCurrent(5.2359878f);
MotorControl_SetSpeedPiGains(0.2864789f, 0.14323945f, 10.0f);
```

不能把旧的`SetSpeedCommand(50.0f)`原样保留：现在它表示50 rad/s，且会被速度限幅裁剪。

实现位置：`speed_estimator.*`中的差分测速/滤波；`speed_pi.*`中的误差、积分与输出预装载；`position_controller.*`中的rad/s输出；`motor_control.*`中的命令、限幅、切换与调试字段。速度PI的Kp/Ki及在线允许上限都按旧值乘60/(2π)换算。Kaw、电流限幅、20 Hz滤波截止频率不随单位变化。周期已经显式乘入积分，不再因频率翻倍而额外倍增Ki。

VOFA格式仍为8个float32加帧尾，通道4/5分别发送`speed_active_target_rad_s`/`speed_filtered_rad_s`，标签改为rad/s；不要再二次乘2π/60。通道0–3仍为电流A，6/7仍为位置度。名义遥测100 Hz，默认USB开启、UART关闭。

## 4. 保护、启动和校准

| 行为 | 20 kHz配置 | 实际时间 |
|---|---:|---:|
| ADC样本超时 | 年龄`>7` | 第8 tick，400 μs |
| 编码器旧样本保护 | 年龄`>21` | 第22 tick，1.1 ms |
| 编码器DMA超时 | 忙计数`>5` | 第6 tick，300 μs |
| 校准无新帧保护 | 连续无效计数`>201` | 第202 tick，10.1 ms |
| 软件过流确认 | 连续4个超限样本 | 名义200 μs |
| 零偏校准丢弃/平均 | 32/512个样本 | 1.6/25.6 ms |

上述计数集中在`motor_params.h`。编码器校准仍取128个有效新帧，正常10 kHz采集时约12.8 ms；重复快周期不重复计入同一帧。斜坡、等待和对齐时间按秒保持原值。

默认启动仍为编码器电流模式、Id=Iq=0；预置速度5.2359878 rad/s只有进入速度模式才使用。没有增加定时运动或自动校准。原编码器Flash记录格式和有效性条件不变；更换电机/编码器安装仍须主动重新校准。校准请求只能在电流/编码器就绪且采样和PWM运行时提交，前台Service停机保存后不会自动重启。

## 5. 软件与实机验证

安装宿主C编译器后运行（TinyCC/GCC/Clang；可通过环境变量`H7_HOST_CC`指定）。本机TinyCC位于工程内的`.tools/host-cc/tcc/tcc.exe`，测试脚本可自动找到它。该目录不提交到Git；不要长期依赖`TEMP`中的编译器路径。若之前设置过指向旧临时目录的`H7_HOST_CC`，先按下面命令重设。各行分开执行：

```powershell
$env:H7_HOST_CC = "$PWD\.tools\host-cc\tcc\tcc.exe"
python tests/run_control_tests.py
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1
```

测试包含双母线算法测试、非法母线编译拒绝、正反转和跨零测速、PI/位置环物理等效性、分频复位、校准采样时间及接线检查。TinyCC宿主兼容头只用于测试，不进入固件。

分别构建24 V/48 V固件。上板后先使用限流电源，确认PWM为20 kHz、每周期完成两相ADC采样、控制中断耗时小于50 μs、编码器CRC/超时计数没有持续增长，再检查电流和速度响应。100 Hz波形不能证明20 kHz采样窗口或中断时限正确，需示波器/逻辑分析仪确认。本次修改不包含烧录和实机验收。
