# H7 两电阻电流环首次调试说明

## 1. 当前工程默认行为

工程上电后仍默认进入已经通过电机实验的电压开环：

```c
MotorControl_SetOpenLoopCommand(0.0f, 0.08f, 1.0f);
```

与旧版本相比，上电时会先用 PC5/`DRV_CAL`完成约 27 ms 的 A/B 相电流偏置校准。校准完成后才开放 TIM8 的三相主/互补 PWM。校准失败不会删除电压开环能力，但会禁止进入电流闭环。

电流采样关系：

- PB1：`I_A`，ADC1 注入 rank 1。
- PB0：`I_B`，ADC1 注入 rank 2。
- `I_C = -(I_A + I_B)`。
- 采样电阻：20 mΩ。
- DRV8323 CSA：`VREF/2`、5 V/V。

## 2. 首次观察的调试变量

在 Keil Watch 或 Ozone Data Watch 中添加全局符号：

```text
g_motor_control_debug
```

先确认：

- `current_sense_ready == 1`。
- `fault == MOTOR_FAULT_NONE`。
- `phase_a_offset`、`phase_b_offset`通常应靠近 2048，且不能靠近 0 或 4095。
- 零电流时 `ia_a`、`ib_a`、`ic_a`靠近 0 A。
- `adc_age_ticks`在电流模式下不会持续增大。

DRV8323 必须已经获得正常工作电源，CSA 输出才有意义。不要把“功率桥不输出PWM”误认为驱动芯片可以完全不供电校准。

## 3. 安全进入角度开环、电流闭环

首次验证建议先让开环命令回到零并等待电机停止，然后执行一次完整停止、选模和重启：

```c
MotorControl_SetOpenLoopCommand(0.0f, 0.0f, 0.0f);
/* 等待电机停止后执行以下调用。 */
MotorControl_Stop();
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
MotorControl_SetCurrentCommand(0.0f, 0.3f, 1.0f);
MotorControl_SetCurrentVoltageLimit(0.15f);
MotorControl_Start();
```

这样会重新校准电流偏置，并经过固定角度定位状态。首次实验必须让电机空载、固定可靠且具备可立即断开母线电源的条件。

已经确认电流方向和稳定性后，可以在运行中请求无扰切换：

```c
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_VOLTAGE);
```

请求只在 10 kHz 控制边界执行。开环切入电流环时会用当前电压预置 PI；电流环切回开环时会从 PI 的最后电压斜坡回到保存的开环命令。

## 4. 首次电流方向判断

正 `Iq_ref`建立后重点观察：

- `iq_a`应当朝正方向跟随`iq_ref_a`。
- `iq_error_a`应逐渐减小。
- `uq_v`不应长期顶在电压限幅。
- 三相重构和应满足`ia_a + ib_a + ic_a`接近0。

如果正`Iq_ref`使`iq_a`快速向负方向增大，这是正反馈，必须立即停止，不要通过继续增加PI来尝试修复。断电后检查相序，并只在以下集中参数中修正采样极性：

```c
#define MOTOR_CURRENT_IA_POLARITY (1.0f)
#define MOTOR_CURRENT_IB_POLARITY (1.0f)
```

## 5. 保守 PI 与整定顺序

首次默认值：

```text
Kp  = 0.05 V/A
Ki  = 20.0 V/(A*s)
Kaw = 100 1/s
```

由于实际电机相电阻和相电感未确认，先保持`Iq=0.3 A`和`0.15 pu`电压上限。确认没有正反馈、明显噪声或持续饱和后，再按以下顺序调整：

1. 暂时保持`Ki=20`，小步提高`Kp`，例如`0.05 -> 0.075 -> 0.10 V/A`。
2. 每次只改一个参数，观察`iq_a`阶跃是否振荡、噪声是否明显放大。
3. `Kp`稳定后，再小步提高`Ki`，例如`20 -> 30 -> 50 V/(A*s)`，改善静差。
4. 只有电流跟踪稳定后，才逐步提高电压上限；软件拒绝超过`0.45 pu`。

运行时设置示例：

```c
MotorControl_SetCurrentPiGains(0.075f, 20.0f, 100.0f);
```

不要在第一次上电时同时增加`Iq`、PI增益和电压上限，否则无法判断异常来自哪个参数。

## 6. 保护值和故障恢复

- d/q电流命令限制：±2 A。
- 任一测量相或重构相连续两个样本超过10 A：锁存过流故障。
- ADC靠近上下电源轨、ADC采样超时、控制计算出现NaN/Inf：关断。
- 发生故障后，PWM和DRV8323使能均被撤销。

排除原因后执行：

```c
MotorControl_ClearFault();
MotorControl_Start();
```

不要在没有确认故障原因时连续清除并重启。

## 7. 构建与回归检查

Keil工程：`MDK-ARM/emptytest.uvprojx`。

源配置检查：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
```

纯算法测试源码位于：

```text
tests/test_open_loop.c
tests/test_current_control.c
```

当前电脑没有获准安装主机版 C 编译器，因此这些测试已用 ARMClang 执行零警告编译检查，并由 Keil 完整固件链接覆盖所有生产实现。以后安装 GCC/Clang 后，可把纯算法源文件链接成主机程序直接运行断言。
