# H743 编码器转速环实机验证指南

本文用于当前 STM32H743、17 位 BiSS-C 编码器、编码器位于电机轴且电机带减速器的硬件。
所有速度命令和反馈都是电机轴机械 rpm，不是减速器输出端 rpm。

本文是更换硬件或重新标定时的从零上电流程，不是当前固件的启动配置。当前工程启动于
模式 4、目标 50 rpm，速度 PI 为 `Kp=0.03 A/rpm`、`Ki=0.015 A/(rpm·s)`，
`Iq` 上限为 0.7 A。50 rpm 已进行实机试转，但电流反馈瞬态仍需进一步核查；
不能把这组参数直接用于另一台电机或负载。

## 1. 上电前条件

- 首次测试使用可限流电源、24 V 母线和可立即断电的急停手段；
- 电机与减速器固定牢靠，输出端不会碰撞限位或人员；
- 已完成当前电机的电流零偏校准和编码器方向/电角度零点校准；
- `encoder_ready=1`、`encoder_calibrated=1`，编码器错误计数不持续增加；
- 新硬件首次测试先临时切回模式 3，验证速度反馈符号后再进入模式 4；
- 本文后续低速步骤按 `Iq` 上限不超过 0.6 A 执行；当前工程的 0.7 A 是本机试验值，
  更换硬件后必须先降低限幅再重新验证。

由于减速器使电机轴不能安全、可靠地手动转动，本流程用模式 3 的小 `Iq` 产生可控运动来
验证编码器位置和速度符号。

## 2. 建议 Watch 列表

```text
mode
requested_mode
run_state
fault
id_ref_a
iq_ref_a
id_a
iq_a
speed_target_rpm
speed_active_target_rpm
speed_raw_rpm
speed_filtered_rpm
speed_error_rpm
speed_pi_proportional_a
speed_pi_integrator_a
speed_iq_command_a
speed_pi_saturated
speed_estimator_ready
speed_control_tick_count
encoder_position_raw
encoder_sequence
encoder_age_ticks
encoder_frame_status
encoder_crc_error_count
encoder_frame_error_count
encoder_spi_error_count
encoder_timeout_count
encoder_dma_guard_error_count
voltage_saturated
overcurrent_count
```

Watch 窗口逐行刷新，不是原子快照。判断方向和稳定性时看一段时间内的趋势，不要用一张截图
推断所有字段来自同一个控制周期。带功率运行时不要为了截图暂停 CPU。

## 3. 模式 3：先验证正方向

保持：

```c
.mode = MOTOR_CONTROL_ENCODER_ANGLE_CURRENT
```

1. 设置 `Id=0 A`、`Iq=+0.2 A`。
2. 启动后观察电机是否平稳克服减速器静摩擦。
3. 如果不转，先停机，再把 `Iq` 每次只增加 `+0.1 A`：`+0.3`、`+0.4`、`+0.5`、
   最多 `+0.6 A`。不要一次跳到上限。
4. 电机转动时确认：
   - `encoder_position_raw` 按同一方向持续变化，跨越 0/131071 时没有速度尖峰；
   - `speed_estimator_ready=1`；
   - `speed_raw_rpm` 与 `speed_filtered_rpm` 均为正；
   - `encoder_sequence` 持续增加，`speed_raw_rpm` 持续刷新；模式 3 下
     `speed_control_tick_count` 保持不变是正常现象，因为该计数只统计模式 4 的 PI 执行次数；
   - 编码器错误计数不持续增加。
5. 将 `Iq` 缓慢降回 0，安全停机。

若正 `Iq` 得到负速度，禁止直接进入模式 4。先检查是否加载了正确的编码器校准记录、相序是否
改变，并重新执行方向/零点校准；不要在速度 PI 中临时乘以 `-1` 掩盖方向问题。

## 4. 模式 3：验证负方向

重新启动模式 3：

1. 设置 `Id=0 A`、`Iq=-0.2 A`；
2. 若静摩擦阻止转动，每次只增加 `0.1 A` 的绝对值，最多到 `-0.6 A`；
3. 确认位置趋势反向，`speed_raw_rpm` 和 `speed_filtered_rpm` 均为负；
4. 将 `Iq` 缓慢降回 0，安全停机。

只有正、负两个方向都正确，才继续速度闭环。

## 5. 模式 4：零速进入

将 `main.c` 配置改为：

```c
.mode = MOTOR_CONTROL_ENCODER_SPEED_CURRENT
```

首次上电验证时，将当前 50 rpm 目标临时改成：

```c
MotorControl_SetSpeedCommand(0.0f);
```

上电后确认：

- `mode` 最终进入 `MOTOR_CONTROL_ENCODER_SPEED_CURRENT`；
- `speed_estimator_ready` 由 0 变为 1；
- `speed_active_target_rpm` 从实测速度平滑走向 0；
- `speed_iq_command_a` 没有突跳到 ±0.6 A；
- `speed_pi_saturated` 不应持续为 1；
- 电机没有冲击、抖动或持续爬行。

由于减速器摩擦存在，零速时出现少量保持电流并不必然是错误；持续顶到 ±0.6 A 才是不允许的。

## 6. 低速阶梯测试

按以下顺序逐项测试，每次都等待转速稳定并记录目标、反馈、Iq 和饱和状态：

```text
+5 rpm -> 0 rpm -> +10 rpm -> 0 rpm
-5 rpm -> 0 rpm -> -10 rpm -> 0 rpm
+20 rpm -> 0 rpm -> -20 rpm -> 0 rpm
```

每一步确认：

- `speed_active_target_rpm` 以约 20 rpm/s 变化，不是阶跃；
- `speed_filtered_rpm` 的符号正确，并能跟随 `speed_active_target_rpm`；
- `speed_iq_command_a` 始终在 ±0.6 A 内；
- 加速时允许短暂饱和，但稳态不应持续饱和；
- `id_a` 维持在 0 A 附近，`iq_a` 能合理跟随命令；
- 编码器、ADC 和电机故障均未出现。

低速稳定后才能逐步增加到 ±100 rpm。减速器输出速度按“电机轴 rpm / 减速比”换算，测试前要
确认输出端位移空间足够。

## 7. 必须立即停止的情况

出现以下任一情况，立即把目标降为 0；若响应异常则关闭 PWM/母线电源：

- 实际旋转方向与速度命令相反；
- `speed_pi_saturated` 持续为 1；
- `speed_iq_command_a` 长时间停在 ±0.6 A，但电机不加速；
- 编码器 CRC、帧、SPI、超时或 DMA guard 错误计数持续增加；
- 转速出现持续振荡，或正负电流来回快速切换；
- 出现冲击、异常噪声、减速器敲击或机械限位风险；
- `fault` 不再是 `MOTOR_FAULT_NONE`；
- 电流、驱动器或电机温升异常。

不要通过提高 Iq 上限来处理方向错误、反馈错误或振荡。先回到模式 3，确定电流环、编码器方向、
速度估算和机械负载分别正常。

## 8. 初次整定建议

当前实机试验配置（不是更换硬件后的通用初值）：

```text
Kp  = 0.03 A/rpm
Ki  = 0.015 A/(rpm*s)
Kaw = 10 1/s
Iq limit = 0.7 A
Speed-mode Iq command slew = 10 A/s
Current motor speed target = 50 rpm
```

整定顺序：

1. 保持 `Ki` 较小，先用 `Kp` 获得不过度振荡的速度响应；
2. 再逐步增加 `Ki`，消除减速器摩擦和负载造成的稳态速度误差；
3. 每次只修改一个参数，并重复 ±5、±10、±20 rpm；
4. 只有电流内环稳定、方向正确且速度环不持续饱和，才考虑提高速度或 Iq 限幅；
5. 更换电机、减速器、负载或安装方式后重新从模式 3 的 ±0.2 A 开始。

F407 工程中的离散 PI 系数没有直接复制，因为其数值隐含了原调用频率、速度单位、采样方法和
机械对象。当前 H743 实现显式使用 A/rpm、A/(rpm*s) 和秒，任何控制周期变化都应先复核
`MOTOR_CONTROL_PERIOD_S` 与 `MOTOR_SPEED_CONTROL_DIVIDER`，再重新整定。

## 9. 位置模式首次实机验证

位置模式复用本章已经验证的速度 PI、电流 PI、`0.7 A` Iq 上限和 `10 A/s` 电流命令
斜率。首次试验保持 `MOTOR_POSITION_SPEED_LIMIT_RPM = 20.0f`，不要同时提高位置增益、
速度限幅或 Iq 限幅。

1. 空载并确保机械行程不会碰限位，先确认模式 3、模式 4 和编码器校准仍正常；
   若从运行中的模式 4 切换，先把速度命令降到 0 rpm 并等待实际速度接近零。
2. 调用 `MotorControl_SwitchToEncoderPositionCurrent()`，等待 `mode` 切换完成且
   `position_control_ready == 1`；此时目标应等于反馈，电机应保持当前位置；
3. 先调用 `MotorControl_SetPositionCommand()` 做 ±2 度和 ±5 度阶跃；
4. 再测试 359→1 度与 1→359 度，确认走约 2 度的最短路径；
5. 小步正常后才测试更大角度，且始终保留停机和断电手段。

同时观察 `position_target_deg`、`position_feedback_deg`、`position_error_deg`、
`position_speed_target_rpm`、`speed_filtered_rpm`、`speed_iq_command_a`、`iq_ref_a`、
`iq_a`、`speed_pi_saturated` 和全部编码器故障计数。若方向错误、持续饱和、振荡、撞击、
异常噪声或温升，立即停止；不要用提高 Iq 上限来掩盖反馈方向或机械问题。

当前软件只完成静态测试和构建验证，尚未证明真实电机的位置稳定性和负载性能。
