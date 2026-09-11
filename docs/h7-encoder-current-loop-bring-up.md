# H7 编码器角度电流环调试说明

## 1. 本版本包含的三种模式

本版本没有位置环，也没有速度环。“编码器角度闭环”表示编码器提供 FOC 坐标
变换所需的真实转子电角度，控制量仍是 `Id_ref/Iq_ref`：

| 模式 | 角度来源 | 被闭环控制的量 |
|---|---|---|
| `MOTOR_CONTROL_OPEN_VOLTAGE` | 虚拟角度积分 | 无，直接给 `Ud/Uq` |
| `MOTOR_CONTROL_OPEN_ANGLE_CURRENT` | 虚拟角度积分 | `Id/Iq` |
| `MOTOR_CONTROL_ENCODER_ANGLE_CURRENT` | SPI4/ENC_01 编码器 | `Id/Iq` |

工程提交时仍默认使用已经过实机验证的
`MOTOR_CONTROL_OPEN_ANGLE_CURRENT`，并保留 `0.8 A / 1 Hz` 命令。新增模式不会
自动启动，必须先完成通信检查和一次显式标定。

后续速度环应放在本电流环外部：编码器机械角计算速度，速度 PI 输出
`Iq_ref`，再调用 `MotorControl_SetEncoderCurrentCommand()`；不要复制另一套
Clarke/Park、PI 或 SVPWM。

## 2. 编码器通信链和时序

控制使用减速前电机轴编码器 `ENC_01/SPI4`：

```text
PE2/SPI4_SCK
  -> U27 数字隔离器
  -> U28 MAX14783 差分发送器
  -> ENC_CLK_01 差分线
  -> 编码器 MA

编码器 SLO
  -> ENC_OUT_01 差分线
  -> U18 MAX14783 差分接收器
  -> U16 数字隔离器
  -> PE5/SPI4_MISO

PE6/ENC_TX_01 -> GPIO 推挽、始终为低；不参与只读 BiSS-C 数据传输
```

SPI4 初始配置为 CPOL=1、第一边沿采样、MSB first、8 bit、约 0.9375 MHz。
每个 100 us 控制周期发起一次 6 字节 DMA 传输，形成 48 个时钟。DMA 回调完成
13 ACK、Start、CDS、17 位位置、Error、Warning 和反相 CRC6 的解析。CRC 或
Error 不合格的帧只增加错误计数，不覆盖上一帧有效位置。

## 3. 第一阶段：只验证通信

先不要进入编码器电流模式，也不要请求标定。保持现有已验证模式运行，或在
具备板级驱动供电的条件下关闭/断开功率母线，只观察 SPI4 和调试变量。

示波器检查：

1. PE6 始终为低。
2. PE2 空闲为高，每 100 us 出现一组 48 个、约 0.9375 MHz 的时钟。
3. PE5 能看到与转轴位置有关的返回数据。

在 Keil Watch/Ozone 中展开 `g_motor_control_debug`，手动缓慢转动电机轴：

- `encoder_valid_count`应持续增加。
- 连续 32 个有效帧后 `encoder_ready == 1`。
- `encoder_frame_status == BISS_FRAME_OK`。
- `encoder_position_raw`覆盖 0～131071，并在一圈边界连续回绕。
- `encoder_sequence`持续增加，`encoder_age_ticks`通常为 0 或 1。
- `encoder_crc_error_count`、`encoder_frame_error_count`、
  `encoder_spi_error_count`和`encoder_timeout_count`不应持续增加。
- 编码器正常时 `encoder_warning == 0`。

通信不稳定时不要标定。先用差分端和 MCU 端同时确认时钟极性、返回数据建立
时间、供电、地和终端电阻，再考虑小幅降低 SPI4 时钟。

## 4. 第二阶段：执行一次方向和电角度零点标定

标定会给电机施加最高 0.2 A 的 d 轴电流，并产生约 1% 机械转角。必须连接
电机，空载并可靠固定，使用限流母线电源，且能够立即断电。标定前先将正常
控制降为零并确认转子停止，例如先切到零电压开环：

```c
MotorControl_SetOpenLoopCommand(0.0f, 0.0f, 0.0f);
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_VOLTAGE);
/* 等待电压斜坡回零、转子完全停止。 */
```

确认下列条件后，由应用命令或调试器调用一次：

```c
MotorControl_RequestEncoderCalibration();
```

接口只置请求标志，真正的状态转换由主循环中的 `MotorControl_Service()`完成；
ADC 中断执行 10 kHz 电流环，但绝不擦写 Flash。过程为：

1. 等待新鲜的 CRC 有效编码器帧。
2. 固定 0 pu 电角度，200 ms 内把 `Id`从 0 斜坡到 0.2 A，`Iq=0`。
3. 稳定 500 ms，再对 128 个不同序号的有效位置做环形平均。
4. 用 200 ms 把定子电角度从 0 推进到 0.1 pu，再稳定并采样，判断方向。
5. 立即关闭 PWM 和 DRV，使 TIM8/ADC 停止后才擦写 Bank2 Sector7。

观察：

- `run_state == MOTOR_RUN_STATE_ENCODER_CALIBRATING`。
- `encoder_calibration_state`按上述阶段前进。
- `iq_ref_a`始终为 0，`id_ref_a`不超过 0.2 A。
- 成功后电机停止，`encoder_calibrated == 1`，
  `encoder_direction`为 `+1`或`-1`，且`fault == MOTOR_FAULT_NONE`。
- 失败后功率级关闭，`fault == MOTOR_FAULT_ENCODER_ALIGNMENT`；结合
  `encoder_calibration_failure`定位无数据、位置不稳定、移动量异常或超时。

Flash 记录固定在 `0x081E0000`，含 magic、版本、零点、方向、10 对极和 CRC32。
应用 IROM 已限制为 `0x001E0000`，不会覆盖该扇区。不要在电机运行时直接调用
`MotorConfigStore_Save()`。

## 5. 第三阶段：首次进入编码器角度电流环

断电重启后先确认 `encoder_calibrated == 1`，证明 Flash 记录加载成功。第一次
转矩测试使用 `Id=0 A, Iq=0.1 A`：

```c
MotorControl_SetEncoderCurrentCommand(0.0f, 0.1f);
MotorControl_Stop();
MotorControl_RequestMode(MOTOR_CONTROL_ENCODER_ANGLE_CURRENT);
MotorControl_Start();
```

`MotorControl_Start()`仍会重新校准 A/B 相电流零偏；完成后才进入编码器模式。
进入条件包括：电流零偏有效、Flash 标定有效、连续 32 帧有效且最新帧不陈旧。
任一条件不满足都会拒绝或关断，不会退化为虚拟角度继续运行。

观察：

- `mode == MOTOR_CONTROL_ENCODER_ANGLE_CURRENT`、`run_state == RUNNING`。
- `id_a`应接近 0；`iq_a`应朝 `+0.1 A`跟随。
- `electrical_angle_pu`与`encoder_electrical_angle_pu`一致，并随转子变化。
- `ud_v/uq_v`和 PI 积分项不应持续顶住电压限幅。
- `encoder_age_ticks <= 10`，错误计数不连续增加。

确认 0.1 A 稳定后才依次尝试 0.2 A、0.3 A。未经确认不要超过已在虚拟角度
模式验证过的 0.8 A，也不要同时提高电流、PI 增益和电压限幅。

以下任一现象必须立即停机并断开母线：`Id`快速增大、正 `Iq_ref`产生错误转矩
方向、明显高频振荡、连续 CRC/超时、异常噪声、功率器件发热或过流。先检查
电机相序、IA/IB 极性、编码器方向和零点，不能靠提高 PI 掩盖角度错误。

## 6. 模式回归与未来速度环接口

标定数据只影响编码器模式。完成测试后仍可请求：

```c
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_ANGLE_CURRENT);
MotorControl_RequestMode(MOTOR_CONTROL_OPEN_VOLTAGE);
```

每次离开编码器模式后都应分别回归原来的电压开环和虚拟角度电流环。编码器
通信错误在这两个旧模式中只更新诊断计数，不会关闭它们。

未来速度环只需周期性调用：

```c
MotorControl_SetEncoderCurrentCommand(0.0f, speed_pi_iq_ref_a);
```

速度 PI 的输出必须经过转矩电流限幅和斜坡；本版本已有的电流命令限幅和
10 kHz 电流环继续作为最内层保护。

## 7. 构建与自动回归

Keil 工程：`MDK-ARM/emptytest.uvprojx`。必须执行全量 Rebuild，确认新文件确实
参与编译并得到 `0 Error(s), 0 Warning(s)`。静态检查：

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
```

主机纯算法测试覆盖 BiSS-C 帧、角度换算、标定状态机和 Flash 记录校验。自动
测试与编译只能证明软件路径；SPI 建立时间、CRC 稳定度、电流极性、实际转矩
方向和零点准确性必须按以上步骤在硬件上确认。
