# H7 更换电机后的参数确认与重新校准手册

本文用于当前 STM32H743 电机控制工程在更换电机、编码器、减速器、功率板或母线电源后重新调试。目标是在不改变现有控制架构的前提下，按低风险顺序重新确认硬件参数、保护阈值、FOC 电流反馈、编码器电角度、速度环和位置环。

> 本文以当前代码为准。当前工程默认启动模式是编码器速度/电流双闭环，目标速度为 `50 rpm`；新电机首次上电前必须改成安全调试模式，不能直接沿用该默认配置。

## 1. 先明确哪些内容会自动完成

| 项目 | 是否自动完成 | 说明 |
|---|---|---|
| ADC 电流零偏校准 | 是 | 每次 `MotorControl_Start()` 都会重新执行 |
| 编码器通信就绪判断 | 是 | 连续有效帧后置位 `encoder_ready` |
| 编码器零位校准 | 否 | 必须显式调用 `MotorControl_RequestEncoderCalibration()` |
| 编码器方向判断 | 否 | 与零位一起在显式校准过程中完成 |
| 电机极对数识别 | 否 | 必须根据电机资料填写 |
| 编码器分辨率识别 | 否 | 必须根据编码器协议和数据手册填写 |
| 相序识别 | 否 | 必须通过低压、低电流测试确认 |
| 电流环 PI 自动整定 | 否 | 电机电阻、电感变化后需要人工验证和整定 |
| 速度环、位置环自动整定 | 否 | 负载和惯量变化后需要人工验证和整定 |

编码器校准结果保存在 Bank 2 Sector 7，地址为 `0x081E0000`。记录包含零位、方向、极对数、有效标志和 CRC。

校验代码位于：

- `Core/Inc/motor/motor_config_store.h`：Flash 地址和记录版本；
- `Core/Src/motor/motor_config_store.c:57`：记录有效性检查；
- `Core/Src/motor/motor_config_store.c:87`：擦除、写入和回读验证。

需要特别注意：

- 新旧电机极对数不同时，旧记录会因为极对数不匹配而自动失效；
- 新旧电机极对数相同时，旧记录可能仍被认为有效，但机械安装后的电角度零位通常已经改变；
- 因此，只要更换过电机、编码器、联轴器或编码器安装位置，都应主动重新校准。在新校准完成前，不得直接启动编码器电流环、速度环或位置环。

## 2. 调试前需要记录的资料

调试前建立一份电机档案，至少记录以下内容。未知项不要凭旧电机参数猜测，应查数据手册或从保守的小信号实验开始。

| 项目 | 新硬件数据 | 数据来源 |
|---|---|---|
| 电机型号 |  | 铭牌/数据手册 |
| 极对数 |  | 数据手册或人工确认 |
| 额定电压 |  | 数据手册 |
| 额定电流 |  | 数据手册 |
| 允许峰值电流及持续时间 |  | 数据手册 |
| 相电阻 |  | 数据手册/测量 |
| 相电感 |  | 数据手册/测量 |
| 额定转速 |  | 数据手册 |
| 编码器类型 |  | 当前工程为 BiSS-C |
| 编码器单圈有效位数/计数 |  | 当前工程为 17 位、131072 counts |
| 编码器安装位置 |  | 电机轴/减速器输出轴 |
| 减速比 |  | 无减速器时填 1 |
| 母线电压 |  | 实测值 |
| 负载惯量及机械限位 |  | 机构资料/现场确认 |

首次调试应满足：电机可靠固定、尽量空载或轻载、机械运动范围足够、使用限流电源，并能够立即断开母线或驱动使能。

## 3. 参数修改清单

集中参数文件为 `Core/Inc/motor/motor_params.h`。

### 3.1 更换电机后必须复核

| 参数 | 当前值 | 修改原则 |
|---|---:|---|
| `MOTOR_POLE_PAIRS` | 10 | 填极对数，不是磁极总数 |
| `MOTOR_CURRENT_ALIGN_A` | 0.5 A | 小于电机和驱动允许值，并足以稳定吸住转子 |
| `MOTOR_CURRENT_START_IQ_A` | 0.3 A | 首次调试应保守降低 |
| `MOTOR_CURRENT_COMMAND_LIMIT_A` | 2.0 A | 不得超过电机、驱动、电源和采样链能力 |
| `MOTOR_CURRENT_TRIP_A` | 10.0 A | 按整套硬件允许峰值设置，不能只看电机额定值 |
| `MOTOR_CURRENT_PI_KP_V_PER_A` | 0.05 | 电阻、电感或控制周期变化后重新整定 |
| `MOTOR_CURRENT_PI_KI_V_PER_A_S` | 20.0 | 电阻、电感或控制周期变化后重新整定 |
| `MOTOR_CURRENT_PI_KAW_PER_S` | 100.0 | 检查电压饱和后的积分恢复 |
| `MOTOR_ENCODER_ALIGN_CURRENT_A` | 0.8 A | 校准用 Id；小电机首次使用前必须降低 |
| `MOTOR_ENCODER_ALIGN_CURRENT_MAX_A` | 0.8 A | 不得小于目标校准电流，也不得超过安全值 |
| `MOTOR_ENCODER_COUNTS_PER_TURN` | 131072 | 按电机轴编码器每机械圈计数设置 |
| `MOTOR_SPEED_IQ_LIMIT_A` | 0.7 A | 新电机首次速度环测试应降低 |
| `MOTOR_SPEED_PI_*` | 见源码 | 电机和负载惯量变化后重新整定 |
| `MOTOR_POSITION_*` | 见源码 | 速度环稳定后重新验证 |

校准步长和计数阈值也需要与极对数、编码器分辨率、减速器和负载共同复核：

- `MOTOR_ENCODER_PREALIGN_STEP_PU`
- `MOTOR_ENCODER_DIRECTION_STEP_PU`
- `MOTOR_ENCODER_DIRECTION_MIN_COUNT`
- `MOTOR_ENCODER_DIRECTION_MAX_COUNT`
- `MOTOR_ENCODER_RETURN_MAX_ERROR_COUNT`

当前方向探测使用 `0.2 pu` 电角度移动，允许检测到 `256～4096` 个编码器计数。更换编码器分辨率、极对数或机械传动后，不能照搬此计数窗口。

### 3.2 只有相关硬件改变时才修改

| 硬件变化 | 需要复核的参数/配置 |
|---|---|
| 更换功率板或采样电阻 | `MOTOR_SHUNT_RESISTANCE_OHM`、CSA 增益、电流极性、过流阈值 |
| 更换电流放大器增益 | `MOTOR_CSA_GAIN_V_PER_V`，并核对 DRV8323 实际寄存器配置 |
| 更换 MCU ADC 参考或量程 | `MOTOR_ADC_REFERENCE_V`、`MOTOR_ADC_FULL_SCALE_COUNT` |
| 更换母线电源 | `MOTOR_NOMINAL_VBUS_V` 和电压限幅 |
| 更换 PWM 频率 | TIM8 配置、`MOTOR_CONTROL_PERIOD_S`、电流环 PI |
| 更换编码器协议或帧格式 | SPI、`biss_frame.*`、`biss_encoder.*`、`encoder_angle.*` |
| 编码器从电机轴移到输出轴 | 速度和位置的物理定义、减速比、极对数换算，不能只改 counts |

如果驱动板、母线和同型号编码器均未改变，CubeMX 外设配置通常可以保留，但仍要通过实际信号确认。

## 4. 首次上电前修改启动配置

启动配置位于 `Core/Src/main.c`：

- `main.c:85`：默认模式；
- `main.c:172`：开环电压命令；
- `main.c:173`：虚拟角电流命令；
- `main.c:174`：编码器电流命令；
- `main.c:175`：速度命令；
- `main.c:180`：位置命令。

当前默认值会直接请求编码器速度环和 `50 rpm`，不适合新电机首次上电。推荐临时改为虚拟角电流模式，并把电流指令置零：

```c
static const MotorControlConfig motor_config = {
    .mode = MOTOR_CONTROL_OPEN_ANGLE_CURRENT,
    /* 其余斜坡参数保持当前工程配置 */
};

MotorControl_SetCurrentCommand(0.0f, 0.0f, 0.0f);
MotorControl_SetEncoderCurrentCommand(0.0f, 0.0f);
MotorControl_SetSpeedCommand(0.0f);
```

如果电机额定电流明显低于原电机，修改启动模式还不够，必须同时降低 `MOTOR_CURRENT_ALIGN_A` 和 `MOTOR_ENCODER_ALIGN_CURRENT_A`。

对于尚未确认电流采样方向、相序或电流环稳定性的硬件，可先使用 `MOTOR_CONTROL_OPEN_VOLTAGE` 和非常小的电压指令检查功率级，再进入虚拟角电流模式。

## 5. 推荐执行顺序和阶段门槛

```text
资料与安全限制
    ↓
静态参数、保护值和安全启动模式
    ↓
ADC 零偏及功率级检查
    ↓
低压开环检查相序和编码器计数
    ↓
虚拟角电流环验证与整定
    ↓
显式执行编码器零位/方向校准
    ↓
断电重启并验证 Flash 记录
    ↓
编码器角度电流环
    ↓
速度环
    ↓
位置环
    ↓
恢复生产参数并固化记录
```

前一阶段未通过时不要进入下一阶段。尤其不能在电流环尚未确认负反馈时执行编码器校准。

## 6. 分阶段操作与验收标准

### 6.1 ADC 零偏和静态检查

`MotorControl_Start()` 会自动执行 ADC 内部校准和电流零偏采样。此时电机应静止，不应存在真实相电流。

通过 `g_motor_control_debug` 观察：

- `current_sense_ready == 1`；
- `phase_a_raw`、`phase_b_raw` 不在 ADC 量程边缘；
- `phase_a_offset`、`phase_b_offset` 稳定；
- 零命令下 `id_a`、`iq_a` 接近零；
- `fault == MOTOR_FAULT_NONE`；
- `overcurrent_count` 不持续增加。

未通过时优先检查 ADC 通道、采样时刻、采样电阻、CSA 增益和电流极性，不要继续校准编码器。

### 6.2 低压开环检查

使用 `MOTOR_CONTROL_OPEN_VOLTAGE`，从零开始缓慢增加很小的 `Uq` 和低电角频率。具体上限必须由电机额定电压、电阻和限流电源决定，不存在适用于所有电机的固定值。

验收标准：

- 电机能按指令方向平稳转动；
- 无缺相、剧烈抖动或异常啸叫；
- 三相电流趋势合理，功率板没有异常发热；
- 正负频率对应的方向明确；
- 手动或低速转轴时，`encoder_position_raw` 连续变化并在单圈边界正常回绕；
- `encoder_crc_error_count`、`encoder_frame_error_count`、`encoder_spi_error_count`、`encoder_timeout_count` 不持续增长。

开环异常时优先排查三相接线、PWM 输出、驱动故障和编码器通信，不要通过修改 PI 符号掩盖硬件问题。

### 6.3 虚拟角电流环

切换到 `MOTOR_CONTROL_OPEN_ANGLE_CURRENT`。先使用零命令，再给幅值很小、正负对称的 `Iq` 阶跃。仅在确认方向和反馈均正确后逐步增加。

观察：

- `iq_a` 是否跟随 `iq_ref_a`；
- `id_a` 是否跟随 `id_ref_a`；
- 正负 `Iq` 是否产生相反且可预期的转矩；
- `voltage_saturated` 是否长期为 1；
- 电流是否振荡，是否出现过流或控制数学故障。

调参顺序：先降低 `Ki`，调整 `Kp` 到响应足够快且无明显振荡，再逐步提高 `Ki` 消除静差，最后检查限幅后的积分恢复。运行时可使用：

```c
MotorControl_SetCurrentPiGains(kp, ki, kaw);
MotorControl_SetCurrentVoltageLimit(limit_pu);
```

接口声明位于 `Core/Inc/motor/motor_control.h:173`。验证后的值必须写回 `motor_params.h`，否则重新上电后不会保留。

### 6.4 编码器通信检查

执行校准前必须满足：

- `encoder_ready == 1`；
- `encoder_frame_status == BISS_FRAME_OK`；
- `encoder_age_ticks` 通常为 0 或 1，且不超过控制允许范围；
- 手动转动一整圈时位置覆盖预期计数范围；
- 所有通信错误计数不持续增加。

当前工程的 BiSS-C 物理链路和 SPI4 时序详见 `docs/h7-encoder-current-loop-bring-up.md`。该文档的通信检查部分仍可使用，但其中旧的校准电流和旧调试字段不能作为当前依据。

### 6.5 执行编码器零位和方向校准

校准前确认：

1. 电流环已在虚拟角模式稳定运行；
2. 电机停止并能在安全范围内转动；
3. 编码器数据新鲜、CRC 正常；
4. 校准电流适合新电机；
5. 负载、减速器静摩擦不会阻止预定移动；
6. 可立即切断电源。

由应用命令或调试器只调用一次：

```c
MotorControl_RequestEncoderCalibration();
```

主循环必须继续调用 `MotorControl_Service()`。实际校准状态机位于 `Core/Src/motor/encoder_calibration.c:160`，当前顺序为：

1. 等待有效编码器；
2. `ALIGN_ZERO`：缓慢建立 d 轴对齐电流；
3. `PREALIGN_MOVE`：预移动以克服静摩擦；
4. `PREALIGN_RETURN`：回到零电角度；
5. `SETTLE_ZERO`：稳定并采集零位；
6. `DIRECTION_MOVE`：沿固定电角度方向移动；
7. `SETTLE_FINAL`：采样移动后位置并判断方向；
8. `RETURN_ZERO`、`SETTLE_RETURN`：回零并检查重复性；
9. `RELEASE_CURRENT`：缓慢撤掉 Id；
10. `COMPLETE` 或 `FAILED`。

校准全程应保持 `iq_ref_a == 0`。当前代码的校准 Id 上限是 `0.8 A`，不是旧文档中的 `0.2 A`。

成功标准：

- `run_state` 进入校准状态后正常退出；
- 无 `MOTOR_FAULT_ENCODER_ALIGNMENT` 或 `MOTOR_FAULT_CONFIG_STORAGE`；
- 功率输出安全关闭；
- Flash 写入和回读成功；
- 重新上电后 `encoder_calibrated == 1`。

校准完成后控制器会停止功率级和实时采样，不会自动重新启动。推荐断电、确认电机停止后重新上电。

### 6.6 编码器角度电流环

重新上电后切换到 `MOTOR_CONTROL_ENCODER_ANGLE_CURRENT`。先使用 `Id=0`、`Iq=0`，再使用新电机允许范围内的很小正负 `Iq`。

验收标准：

- 正 `Iq` 和负 `Iq` 产生相反方向的稳定转矩；
- `iq_a` 跟随命令，`id_a` 接近零；
- `electrical_angle_pu` 随转子连续变化；
- `voltage_saturated` 不长期置位；
- 编码器错误计数不持续增加；
- 无抖动、失步、突然反转或异常大电流。

方向错误时按“电机相序 → 电流采样通道和极性 → 编码器校准 → 极对数”的顺序检查，不要直接把速度 PI 增益改成负数。

### 6.7 速度环

只有编码器角度电流环通过后，才切换到 `MOTOR_CONTROL_ENCODER_SPEED_CURRENT`。

建议顺序：

1. 速度目标设为 `0 rpm`；
2. 临时降低 `MOTOR_SPEED_IQ_LIMIT_A`；
3. 从 `±5 rpm` 开始验证；
4. 再测试 `±10 rpm`、`±20 rpm`；
5. 分别检查启动、稳定、减速、停止和正反转；
6. 先调 `Kp`，再调 `Ki`，最后提高 Iq 限幅和速度斜坡。

运行时调参接口：

```c
MotorControl_SetSpeedPiGains(kp, ki, kaw);
MotorControl_SetSpeedIqLimit(iq_limit_a);
```

观察 `speed_target_rpm`、`speed_active_target_rpm`、`speed_filtered_rpm`、`speed_iq_command_a` 和 `speed_pi_saturated`。若长期饱和，应先确认负载、电流限幅、方向和速度反馈，而不是继续增加积分增益。

### 6.8 位置环

只有速度环在正反方向、加减速和停止状态下均稳定后，才进入 `MOTOR_CONTROL_ENCODER_POSITION_CURRENT`。

建议从保持当前位置开始，再测试 `±2°`、`±5°` 小步进，并单独检查 `359° ↔ 1°` 的单圈边界。

观察：

- `position_target_deg`；
- `position_feedback_deg`；
- `position_error_deg`；
- 内层速度和电流是否饱和。

位置环振荡时，先降低 `MOTOR_POSITION_KP_RPM_PER_DEG` 或 `MOTOR_POSITION_SPEED_LIMIT_RPM`，不要在同一次实验中同时改位置环、速度环和电流环参数。

## 7. 编码器校准失败的处理

| 现象 | 优先检查 |
|---|---|
| 编码器未就绪 | SPI4 时序、供电、隔离器、差分收发器、CRC、帧格式 |
| 零位采样不稳定 | 机械振动、校准电流过小或过大、编码器噪声、负载回弹 |
| 移动计数小于下限 | 电流不足、静摩擦过大、方向步长过小、分辨率/极对数配置错误 |
| 移动计数大于上限 | counts 配置错误、步长过大、极对数错误、编码器装在不同轴端 |
| 返回零位误差过大 | 减速器回差、机械松动、校准电流不足、回零时间不够 |
| 状态超时 | 电机被卡住、编码器帧无效、校准斜坡/稳定时间不适合当前机构 |
| Flash 保存失败 | 地址/扇区配置、写保护、电压、擦写错误、链接空间冲突 |

修改校准电流、角度步长、计数窗口或时间参数时，每次只改变一类参数并记录结果，避免同时修改后无法判断原因。

## 8. CubeMX 配置的修改条件

CubeMX 工程为 `emptytest.ioc`。只有外设、引脚或实时周期确实改变时才重新生成代码。

| 变化 | CubeMX 修改点 | 生成后重点检查 |
|---|---|---|
| PWM 频率/计数周期 | TIM8 Prescaler、Period、中心对齐方式 | `Core/Src/tim.c`、死区、互补输出、控制周期常量 |
| PWM 引脚或相序 | TIM8 CH1/2/3 及互补通道 GPIO | `tim.c`、`gpio.c`、实际栅极信号 |
| ADC 通道 | ADC1 Regular/Injected channel | `Core/Src/adc.c`、A/B 相映射 |
| ADC 采样时刻 | TIM8 TRGO/触发源及 ADC edge | `adc.c`、`tim.c`、电流噪声 |
| 编码器 SPI | SPI4 模式、分频、DMA、GPIO | `Core/Src/spi.c`、DMA 中断、BiSS 实际波形 |
| 驱动使能/故障脚 | GPIO 输入输出及中断 | `gpio.c` 和板级驱动代码 |

推荐修改流程：

1. 备份当前 `.ioc` 和已经验证的参数；
2. 在 CubeMX 中修改 `emptytest.ioc`；
3. 重新生成代码；
4. 检查 `tim.c`、`adc.c`、`spi.c`、`gpio.c` 的差异；
5. 确认用户代码区未被覆盖；
6. PWM 周期改变时同步修改 `MOTOR_CONTROL_PERIOD_S`；
7. 重新整定电流环，并从本手册的静态检查阶段重新开始。

## 9. 必须保留的调试观测值

本工程已经精简过调试字段。换电机调试时，至少保留并观察以下 `g_motor_control_debug` 字段：

- 状态：`mode`、`requested_mode`、`run_state`、`fault`；
- ADC：`phase_a_raw`、`phase_b_raw`、两个 offset、`current_sense_ready`、`adc_age_ticks`；
- 电流：`id_a`、`iq_a`、`id_ref_a`、`iq_ref_a`、`voltage_saturated`；
- 编码器：`encoder_ready`、`encoder_calibrated`、`encoder_frame_status`、`encoder_position_raw`、`encoder_age_ticks`；
- 编码器错误计数：CRC、frame、SPI、timeout、DMA guard；
- 速度：目标、斜坡目标、反馈、Iq 输出、PI 饱和；
- 位置：目标、反馈、误差；
- 保护：`overcurrent_count`。

这些字段足以覆盖换电机的主要判断，不需要为了调试重新加入大量只使用一次的内部变量。

## 10. 参数固化和版本记录

每一阶段通过后记录：

- 电机、编码器、功率板和母线版本；
- 修改过的宏及最终值；
- 电流环、速度环和位置环阶跃条件；
- 峰值电流、稳定误差、饱和情况和温升；
- 编码器校准是否成功、重启后是否加载；
- 最终默认启动模式和命令。

运行时 API 调整只用于实验，最终确认值应写回 `motor_params.h`。完成全部验证后，再把 `main.c` 恢复为项目需要的默认模式、速度或位置目标。

## 11. 编译和静态回归

修改参数或文档后，至少执行现有主机回归：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1
```

固件构建可使用工程现有 Keil 项目：

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\emptytest.uvprojx' -j0
```

要求编译结果为 `0 Error(s)`。主机测试和编译只能验证代码与配置的一致性，不能替代开环、电流方向、编码器方向、过流保护和机械限位的实机验证。

## 12. 最终放行清单

- [ ] 新电机极对数已确认并写入代码
- [ ] 编码器分辨率、安装轴端和协议已确认
- [ ] 额定电流、峰值电流和软件过流阈值已复核
- [ ] 母线、电流采样比例和 PWM 周期与实际硬件一致
- [ ] 默认启动模式已改为安全调试模式
- [ ] ADC 零偏和静态电流检查通过
- [ ] 低压开环相序、方向和编码器计数检查通过
- [ ] 虚拟角电流环正负小电流测试通过
- [ ] 编码器零位和方向已重新校准
- [ ] 断电重启后 `encoder_calibrated == 1`
- [ ] 编码器角度电流环通过
- [ ] 速度环从低速到目标速度逐级通过
- [ ] 位置环小角度、边界和负载测试通过
- [ ] 运行时验证参数已固化到 `motor_params.h`
- [ ] 最终启动模式和目标命令已恢复
- [ ] 主机回归和 Keil 编译通过
- [ ] 实机温升、噪声、急停和机械限位已经验证

## 13. 相关文档

- `docs/h7-motor-control-code-analysis.md`：完整运行流程和关键代码分析；
- `docs/h7-open-loop-bring-up.md`：开环电压调试；
- `docs/h7-current-loop-bring-up.md`：电流采样和电流环调试；
- `docs/h7-encoder-current-loop-bring-up.md`：BiSS-C 通信链路及历史校准说明；
- `docs/h7-speed-loop-bring-up.md`：速度环和位置环调试。

如旧文档中的参数、默认模式或调试字段与本文不一致，以当前源码 `motor_params.h`、`motor_control.h` 和 `main.c` 为最终依据。
