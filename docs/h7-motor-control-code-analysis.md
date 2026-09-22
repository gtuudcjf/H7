# STM32H743 电机控制工程代码逻辑分析

本文用于理解当前工程、定位模式切换入口，以及指导更换电机后的参数复核。
分析更新日期为2026-09-18。本文描述的是当前24 V母线、17位BiSS-C编码器、
10极对电机的已验证基线；更换硬件后必须重新从低电压、低电流开始验证。

## 1. 当前功能状态

| 功能 | 代码状态 | 当前实机状态 |
|---|---|---|
| TIM8六路互补PWM、DRV8323驱动 | 已实现 | 已验证，可驱动电机 |
| 模式1：电压开环 | 已实现 | 已验证 |
| PB1/I_A、PB0/I_B两电阻采样 | 已实现 | 已验证能采集并完成零偏校准 |
| 模式2：虚拟角度、电流闭环 | 已实现 | 已验证电机能够低速转动 |
| SPI4模拟BiSS-C读取17位位置 | 已实现 | 已验证，可稳定取得有效位置帧 |
| 编码器机械角/电角度换算 | 已实现并有算法测试 | 已验证角度连续变化 |
| 编码器方向与电角度零点校准 | 已实现 | 已实机完成并写入Flash |
| 模式3：编码器角度同步、电流闭环 | 已实现 | 已以Iq=0.6 A实机运行 |

注意：模式3不是“位置环”。它只是用编码器给出的转子电角度代替虚拟电角度，
使d/q电流跟随真实转子坐标系。模式4在该电流环外增加速度PI来生成`Iq_ref`；
当前工程仍没有位置给定、位置误差或位置PI。

## 2. 建议的代码阅读顺序

不要从某一个大文件逐行读到底。应按“硬件时基—编排层—算法层—输出层”阅读：

1. `Core/Src/main.c`
   - 查看外设初始化顺序、默认模式、初始命令和三个HAL回调入口。
2. `Core/Inc/motor/motor_runtime_policy.h`
   - 先认识控制模式、运行阶段以及允许的状态转换。
3. `Core/Inc/motor/motor_control.h`
   - 查看上层可以调用的接口和`g_motor_control_debug`调试字段。
4. `Core/Src/motor/motor_control.c`
   - 重点阅读`Init`、`Start`、`FastTick`、`CurrentSampleComplete`和
     `StartSelectedMode`，先忽略小型数学辅助函数。
5. 编码器链路：
   - `spi.c` → `biss_encoder.c` → `biss_frame.c` → `encoder_angle.c`。
6. 电流环链路：
   - `current_sense.c` → `foc_transform.c` → `current_pi.c`。
7. 电压输出链路：
   - `svpwm.c` → `pwm_3ph.c` → `tim.c`。
8. 最后阅读状态最多、且涉及机械动作和Flash的校准路径：
   - `encoder_calibration.c` → `motor_config_store.c`。

建议分三轮阅读，不要第一次就陷入所有细节：

### 第一轮：只回答“什么时候调用谁”

只读`main.c`、`motor_runtime_policy.h`和`motor_control.h`。画出三个执行上下文：

- 主循环：只运行`MotorControl_Service()`，允许执行停机和Flash操作；
- TIM8更新中断：固定10 kHz节拍，推进角度、编码器调度和模式请求；
- ADC注入完成中断：取得同周期电流样本，执行保护和模式2/3/4电流PI。

### 第二轮：只跟踪一条数据链

从ADC原始值开始，沿`current_sense -> foc_transform -> current_pi -> svpwm
-> pwm_3ph`一直读到TIM8 CCR。先不读编码器校准状态机。

### 第三轮：比较四种模式

在`MotorControl_FastTick()`和`MotorControl_CurrentSampleComplete()`中分别找：

1. 电角度从哪里来；
2. Ud/Uq由直接命令还是PI产生；
3. 最终在哪个函数汇合写PWM。

理解这三点后，再阅读BiSS-C、Flash校准和无扰切换细节。

## 3. 总体分层

```text
main.c（配置、启动、HAL回调）
    |
    +-- motor_control.c（状态机和实时任务编排）
    |     +-- current_sense.c（ADC计数 -> 三相电流）
    |     +-- foc_transform.c（abc -> alpha/beta -> d/q）
    |     +-- current_pi.c（Id/Iq误差 -> Ud/Uq）
    |     +-- open_loop.c（虚拟电角度和频率斜坡）
    |     +-- encoder_angle.c（编码器位置 -> 机械角/电角度）
    |     +-- svpwm.c（Ud/Uq和电角度 -> 三相占空比）
    |     +-- pwm_3ph.c（三相占空比 -> TIM8 CCR1/2/3）
    |
    +-- biss_encoder.c（SPI4/DMA异步采集和有效快照）
          +-- biss_dma_buffer.c（DMA缓存行隔离和越界保护）
          +-- biss_frame.c（同步、17位位置、状态位、CRC6）
```

`motor_control.c`是工程的编排中心，但不是所有算法的实现位置。分析时应先确定
它调用了哪个模块，再进入对应的小文件查看公式。

## 4. 上电启动流程

`main()`按以下顺序运行：

```text
HAL/时钟初始化
 -> GPIO、ADC1、SPI2、SPI3、SPI4、TIM8等外设初始化
 -> BissEncoder_Init(&hspi4)
 -> MotorControl_Init(&motor_config)
 -> 设置各模式的目标命令
 -> MotorControl_Start()
 -> while(1)中反复执行MotorControl_Service()
```

### 4.1 `BissEncoder_Init()`

这里只绑定SPI4、初始化DMA缓冲区和清空统计值，不会立即输出编码器时钟。

### 4.2 `MotorControl_Init()`

主要完成：

- 保存默认模式到`requested_mode`，此时`motor_mode`仍为`STOPPED`；
- 初始化虚拟角度发生器；
- 装入20 mΩ采样电阻、DRV8323增益和ADC比例；
- 初始化电流PI；
- 尝试从Flash读取以前保存的编码器零点和方向；
- 初始化DRV8323板级对象和TIM8 PWM封装；
- 不使能功率输出。

Flash中没有有效编码器校准记录时，模式1和模式2仍可运行，但模式3会被拒绝。

### 4.3 `MotorControl_Start()`

真实的硬件启动顺序为：

1. ADC1内部校准；
2. 使能并配置DRV8323；
3. 打开`DRV_CAL`，开始电流采样零偏校准；
4. 启动ADC注入序列中断；
5. 启动TIM8更新中断；
6. 启动TIM8 CH4内部采样触发；
7. 丢弃前16个样本，再平均256个样本；
8. 关闭`DRV_CAL`并检查零偏范围；
9. 根据`requested_mode`启动选定模式和PWM。

启动过程中`motor_mode`保持`STOPPED`是正常行为，真正进入模式1/2/3发生在电流
零偏校准完成之后。若编译时直接选择模式3，零偏完成后会先停在`READY`，保持
三相PWM关闭，等第一批新鲜且ready的编码器帧到达后才进入模式3。

## 5. 三条实时执行链路

### 5.1 TIM8更新中断：统一10 kHz时基

调用链：

```text
TIM8_UP_TIM13_IRQHandler
 -> HAL_TIM_IRQHandler
 -> HAL_TIM_PeriodElapsedCallback
 -> MotorControl_FastTick(100 us)
```

`MotorControl_FastTick()`负责：

- 在允许的运行阶段推进编码器DMA采集；
- 在模式1/2中推进虚拟电角度和频率斜坡；
- 在模式3中读取最新编码器电角度；
- 检查ADC样本是否超时；
- 在模式1中直接更新SVPWM；
- 处理运行中的模式切换请求。

它不执行电流PI。电流PI放在ADC注入转换完成回调中，使控制计算与真实采样时刻
对齐。

### 5.2 ADC1注入转换完成：电流闭环路径

TIM8 CH4的`OC4REF`经`TRGO2`触发ADC1注入序列：

```text
TIM8 CH4内部比较事件
 -> ADC1 rank1读取PB1/I_A
 -> ADC1 rank2读取PB0/I_B
 -> ADC_IRQHandler
 -> HAL_ADCEx_InjectedConvCpltCallback
 -> MotorControl_CurrentSampleComplete()
```

正常电流环计算顺序：

```text
ADC原始值
 -> 减去A/B两相零偏并换算为安培
 -> Ic = -(Ia + Ib)
 -> Clarke变换得到Ialpha/Ibeta
 -> 使用本模式的电角度进行Park变换
 -> 得到Id/Iq反馈
 -> Id/Iq PI输出Ud/Uq（伏特）
 -> 除以标称母线电压得到标幺Ud/Uq
 -> SVPWM
 -> TIM8 CCR1/CCR2/CCR3
 -> 三路主PWM和三路互补PWM
```

### 5.3 SPI4/DMA：编码器采集路径

软件调用链：

```text
MotorControl_FastTick
 -> BissEncoder_ControlTick（年龄和超时）
 -> BissEncoder_StartRead
 -> HAL_SPI_TransmitReceive_DMA（发送6个0，仅用于产生48个时钟）
 -> DMA1 Stream0/1 + SPI4 EOT中断
 -> HAL_SPI_TxRxCpltCallback
 -> BissEncoder_OnTransferComplete
 -> BissFrame_Parse17
 -> 发布BissEncoderSnapshot
```

当前SPI4连接逻辑是：

- PE2/SPI4_SCK产生BiSS-C MA时钟；
- PE5/SPI4_MISO接收编码器SLO数据；
- PE6保持普通GPIO低电平，不作为MOSI使用；
- TX DMA并不是向编码器发送命令，只是不断向SPI发送0，使主机产生时钟；
- RX DMA同时采集48位数据。

## 6. BiSS-C帧如何得到17位位置

`biss_frame.c`把一次DMA得到的6字节视为48位MSB-first采样窗口。当前解析格式为：

```text
13位ACK低电平 | Start | CDS | 17位位置 | Error | Warning | 6位CRC
```

有效响应共40位，因此48位窗口允许0～8位前导空闲。解析器搜索“连续13个0后紧跟
Start=1”的位置，再跳过Start/CDS，提取17位`position_raw`。

随后进行两项检查：

1. `Error`必须表示编码器状态正常；`Warning`会记录但不会直接丢弃位置；
2. 对“17位位置+Error+Warning”计算反相CRC6，并与接收CRC比较。

只有同步、Error和CRC全部有效时，`position_raw`才会更新。坏帧只更新诊断字段，
不会覆盖上一帧有效位置。

## 7. 编码器快照与关键变量

SPI4回调和电机控制中断可能并发访问编码器数据，因此控制层不直接读取DMA数组。
`BissEncoder_GetSnapshot()`在短临界区内复制整个结构体，保证同一轮FOC使用同一帧。

关键变量含义：

- `encoder_raw[6]`：最近解析帧的6个原始字节；
- `encoder_position_raw`：最近有效17位位置，范围0～131071；
- `encoder_sequence`：每成功发布一帧加1；
- `encoder_valid_count`：累计有效帧数；
- `encoder_age_ticks`：距离最近有效帧的10 kHz周期数；
- `encoder_ready`：连续32帧有效后置1；
- `encoder_frame_status`：最近一次帧解析结果；
- `encoder_crc_error_count`：累计CRC错误；
- `encoder_frame_error_count`：累计同步/状态错误；
- `encoder_spi_error_count`：累计SPI硬件/HAL错误；
- `encoder_timeout_count`：DMA超时次数；
- `encoder_dma_guard_error_count`：DMA越界保护区被破坏次数。

Keil在程序运行时逐项刷新Watch变量，不保证所有字段来自同一个瞬间。因此可能短暂
看到`received_crc != calculated_crc`，或`Id/Iq/error`不满足同一时刻的算术关系。
带功率运行时不要为了截图直接暂停CPU，暂停可能让PWM停留在不可控瞬间。判断稳定性
应观察错误计数趋势；确需一致快照时，先把Iq命令降为0并安全停机，再暂停读取。

## 8. 17位位置到FOC电角度

这一段在`encoder_angle.c`中实现，当前实机校准结果已保存到Flash：

```text
relative = position_raw - electrical_zero_raw
mechanical_angle_pu = wrap(relative * direction / 131072)
electrical_angle_pu = wrap(mechanical_angle_pu * pole_pairs)
```

本工程`MOTOR_POLE_PAIRS=10`，因此机械轴转一圈时，电角度循环10次。

- `electrical_zero_raw`：转子d轴与控制坐标零轴对齐时的编码器位置；
- `direction`：`+1`或`-1`，用于统一编码器计数方向与电机正方向；
- `mechanical_angle_pu`：机械角归一化到`[0,1)`；
- `electrical_angle_pu`：FOC Park/逆Park使用的电角度，归一化到`[0,1)`。

当前已验证的电角度零点原始值为67046、方向为-1。它们只适用于当前电机、编码器
安装位置和相序；拆装联轴器、改变相线、修改极对数或更换电机后必须重新校准。
能够读取`encoder_position_raw`只证明通信和帧解析正确，不代表旧校准仍然有效。

## 9. 四种控制模式的联系与区别

### 模式1：`MOTOR_CONTROL_OPEN_VOLTAGE`

- 角度来源：`open_loop.c`积分得到的虚拟电角度；
- 控制量：直接设置`Ud/Uq`标幺电压；
- 电流反馈：只用于监视和保护，不参与调节；
- 主路径：`FastTick -> OpenLoop_Step -> SVPWM -> PWM`。

### 模式2：`MOTOR_CONTROL_OPEN_ANGLE_CURRENT`

- 角度来源：虚拟电角度；
- 控制量：给定`Id_ref/Iq_ref`，电流PI计算`Ud/Uq`；
- 主路径：
  - `FastTick`推进虚拟角度；
  - ADC回调用该角度执行Park、电流PI和SVPWM。
- 已证明电流闭环能工作，但虚拟角度不跟随转子，带载能力和动态性能受限。

### 模式3：`MOTOR_CONTROL_ENCODER_ANGLE_CURRENT`

- 角度来源：编码器位置经过零点、方向和极对数换算后的真实电角度；
- 控制量：给定`Id_ref/Iq_ref`，仍复用与模式2相同的电流PI；
- 进入条件：电流采样有效、编码器ready、帧未过期、校准记录有效；
- 当前状态：已在24 V母线下以`Id=0 A、Iq=0.6 A`完成实机运行验证。

### 模式4：`MOTOR_CONTROL_ENCODER_SPEED_CURRENT`

- 角度来源：与模式3相同的编码器真实电角度；
- 控制量：1 kHz速度PI生成限幅后的`Iq_ref`，10 kHz电流PI仍负责`Ud/Uq`；
- 进入/退出：保存活动电流参考，模式4强制`Id_ref=0`且`|Iq_ref|`不超过运行限值；
- 当前状态：代码与构建已验证，首次实机必须按低速指南从模式3方向检查开始。

四种模式最终都复用同一个`MotorControl_WriteVoltage()`、SVPWM和TIM8输出层，因此
不会有多个模块同时写CCR。模式差异只发生在“电角度从哪里来”和“Ud/Uq由谁产生”。

## 10. 编码器校准路径

校准入口是`MotorControl_RequestEncoderCalibration()`。请求不会直接在调用点执行，
而是由主循环中的`MotorControl_Service()`切换到校准状态，再由ADC控制周期推进：

```text
IDLE
 -> WAIT_VALID（等待新鲜有效编码器帧）
 -> ALIGN_ZERO（小Id递增、Iq=0，强制电角度为零）
 -> PREALIGN_MOVE（缓慢扫到0.2 pu，克服静摩擦）
 -> PREALIGN_RETURN（从同一方向缓慢回到0 pu）
 -> SETTLE_ZERO（等待机械稳定并采样零点）
 -> DIRECTION_MOVE（小幅改变强制电角度以判断方向）
 -> SETTLE_FINAL（采样方向探测终点并检查位移范围）
 -> RETURN_ZERO（缓慢回到0 pu）
 -> SETTLE_RETURN（采样回零位置并检查零点重复性）
 -> RELEASE_CURRENT（保持0 pu并缓慢将Id降到0，减小机械回弹）
 -> COMPLETE或FAILED
```

成功后构建包含零点、方向、极对数和CRC32的配置记录。功率级和实时采样全部停止后，
主循环才擦写Flash。该设计避免在ADC/TIM8中断中执行耗时Flash操作。

实机已证明单次方向位移可能偶然合格，但从探测终点再次启动时会因
静摩擦停在旧位置。因此现在只有“方向位移合格+回零误差合格”才会保存配置。
当前流程已在现有电机/减速器上完成一次成功校准；每次更换电机或机械安装后仍要核对：

- 强制d轴电流方向是否正确；
- 转子实际移动是否在减速器和负载允许范围内；
- 编码器计数方向判断是否正确；
- 零点重复校准的一致性；
- Flash保存地址是否与最终固件链接范围冲突。

## 11. 如何自己分析一次运行过程

建议每次只回答以下五个问题：

1. 当前执行上下文是什么？
   - 主循环、TIM8中断、ADC中断还是SPI4/DMA回调。
2. 本周期使用的电角度来自哪里？
   - 模式1/2来自`open_loop_state`，模式3来自`encoder_angle_sample`。
3. 本周期的`Ud/Uq`由谁产生？
   - 模式1由开环命令直接给定；模式2/3由电流PI产生。
4. 数据是否具备有效性条件？
   - 电流看`current_sense_ready`和`adc_age_ticks`；编码器看`ready`、
     `sequence`、`age_ticks`及错误计数。
5. 最终是谁写PWM？
   - 所有路径都必须汇合到`MotorControl_WriteVoltage()`，再进入SVPWM和
     `Pwm3ph_ApplyDuty()`。

只要沿着这五个问题追踪，就不会把编码器通信、位置环、电流环和PWM输出混在一起。

## 12. 建议观察的Keil字段

通用状态：

- `mode`：当前真正执行的模式；
- `requested_mode`：待应用或已应用的目标模式；
- `run_state`：启动/对齐/校准/运行阶段；
- `fault`：锁存故障原因。

电流环：

- `id_ref_a/iq_ref_a`与`id_a/iq_a`；
- `ud_v/uq_v`与`ud_pu/uq_pu`；
- `voltage_saturated`、`overcurrent_count`、`adc_age_ticks`。

编码器：

- `mode`、`requested_mode`、`run_state`、`fault`；
- `encoder_ready`应保持1；
- `encoder_sequence`和`encoder_valid_count`应持续增加；
- `encoder_age_ticks`通常应保持很小；
- `encoder_position_raw`应与机械位置一致且范围正确；
- 五类编码器错误计数不应持续增加；
- `encoder_calibrated`应为1才能进入模式3；
- `encoder_mechanical_angle_pu`和`encoder_electrical_angle_pu`应随转子连续环绕。

## 13. 模式选择与运行时切换

### 13.1 编译时选择启动模式

只修改`Core/Src/main.c`中的一个字段：

```c
static const MotorControlConfig motor_config = {
    .mode = MOTOR_CONTROL_ENCODER_ANGLE_CURRENT,
    .frequency_slew_hz_per_s = 20.0f,
    .voltage_slew_pu_per_s = 5.0f
};
```

三个合法值分别是：

| `.mode` | 角度来源 | Ud/Uq来源 | 启动附加条件 |
|---|---|---|---|
| `MOTOR_CONTROL_OPEN_VOLTAGE` | 虚拟角度 | 直接给定 | 电流零偏失败仍可保留开环 |
| `MOTOR_CONTROL_OPEN_ANGLE_CURRENT` | 虚拟角度 | Id/Iq PI | 电流零偏必须有效 |
| `MOTOR_CONTROL_ENCODER_ANGLE_CURRENT` | 编码器角度 | Id/Iq PI | 电流零偏、编码器ready、Flash校准均有效 |

`main.c`在`MotorControl_Start()`之前预置了三套命令，只有所选模式对应的一套会被
执行。换电机后不能只改`.mode`而照搬原命令，应先降低Uq/Iq再验证。

### 13.2 运行时切换

运行中不要写`motor_config.mode`，它是只读启动配置，而且控制层已在初始化时复制。
使用以下组合接口：

```c
MotorControl_SwitchToOpenVoltage(ud_pu, uq_pu, electrical_frequency_hz);
MotorControl_SwitchToOpenAngleCurrent(id_a, iq_a, electrical_frequency_hz);
MotorControl_SwitchToEncoderAngleCurrent(id_a, iq_a);
```

接口先保存目标，再发布`requested_mode`；`MotorControl_FastTick()`只在统一10 kHz
边界调用`MotorControl_ApplyModeRequest()`。进入电流环时，代码用新角度重算当前
Id/Iq并预装载PI；退回电压开环时，从PI最后电压平滑过渡到开环目标。因此不要绕过
这些接口直接写内部状态或TIM8 CCR。

同一模式内只更新目标时使用：

```c
MotorControl_SetOpenLoopCommand(...);
MotorControl_SetCurrentCommand(...);
MotorControl_SetEncoderCurrentCommand(...);
```

## 14. 更换电机后的修改清单

不要从调PI开始。推荐顺序是“硬件比例 → 保护 → 相序/方向 → 编码器 → PI”。

### 14.1 必须确认的电机与功率参数

集中修改`Core/Inc/motor/motor_params.h`：

| 参数 | 何时修改 | 填错后的典型表现 |
|---|---|---|
| `MOTOR_POLE_PAIRS` | 电机极对数变化 | 电角度倍率错误，模式3抖动/反转/大电流 |
| `MOTOR_NOMINAL_VBUS_V` | 母线电压变化 | PI伏特到pu换算错误，转矩偏弱或过调制 |
| `MOTOR_CURRENT_COMMAND_LIMIT_A` | 电机/功率级允许电流变化 | 命令过大或能力被不必要限制 |
| `MOTOR_CURRENT_TRIP_A` | 结合电机、MOS、采样量程设定 | 过小误报，过大失去软件保护 |
| `MOTOR_CURRENT_*_POLARITY` | 电流采样通道方向变化 | 电流环变成正反馈；必须低电流确认 |
| `MOTOR_CURRENT_PI_*` | 电阻、电感或PWM频率变化 | 跟随慢、振荡、噪声或频繁饱和 |
| `MOTOR_ENCODER_ALIGN_CURRENT_A` | 静摩擦/额定电流变化 | 校准不动或冲击过大 |

### 14.2 功率板或采样电路变化

若更换采样电阻、DRV8323 CSA增益或ADC参考，同时修改：

- `MOTOR_SHUNT_RESISTANCE_OHM`；
- `MOTOR_CSA_GAIN_V_PER_V`，并核对`drv8323_board/drv8323`中的实际寄存器配置；
- `MOTOR_ADC_REFERENCE_V`和ADC分辨率对应满量程；
- CubeMX中的PB1/I_A、PB0/I_B注入通道及rank顺序。

`CurrentSense_Convert()`按
`Vref / (ADC满量程 * CSA增益 * 采样电阻)`计算安培/计数。任何一项不一致都会让
Keil中显示的电流和实际电流成比例错误，过流保护也会随之失真。

### 14.3 编码器或机械安装变化

- 相同17位BiSS-C编码器但重新安装：重新执行方向/零点校准；
- 极对数变化：修改`MOTOR_POLE_PAIRS`并重新校准，旧Flash记录会失效；
- 编码器分辨率或帧格式变化：修改`biss_frame.*`和`encoder_angle.c`中的位置尺度，
  不能只改极对数；
- 编码器接口或引脚变化：修改CubeMX/SPI4/DMA和`biss_encoder.*`；
- 交换电机相线：原编码器方向/零点关系可能变化，必须重新校准。

### 14.4 PWM频率变化

TIM8当前为中心对齐10 kHz。修改PSC/ARR或时钟树后必须同步：

- `MOTOR_CONTROL_PERIOD_S`；
- ADC注入触发点TIM8 CH4；
- 电流PI离散增益的实机验证；
- 编码器DMA超时与数据年龄阈值（它们以控制tick计数）。

### 14.5 推荐重新上电验证顺序

1. PWM禁止状态检查ADC零偏和DRV故障；
2. 模式1用很小Uq和低电角频率确认相序/旋向；
3. 模式2从小Iq开始确认电流反馈是负反馈；
4. 空载观察电流PI是否饱和或振荡；
5. 重新执行编码器方向/零点校准并断电重启验证Flash加载；
6. 模式3先以`Id=0、Iq=0`进入，再逐步增加Iq；
7. 最后才提高电流上限、速度或负载。

## 15. 修改需求到代码位置的快速索引

| 需求 | 首先查看/修改 |
|---|---|
| 改上电模式 | `main.c`中的`motor_config.mode` |
| 改四种模式启动命令 | `main.c`四个`MotorControl_Set...Command()` |
| 运行时切模式 | `motor_control.h`的`MotorControl_SwitchTo...()` |
| 改电机极对数/母线/电流限制/PI | `motor_params.h` |
| 改电流采样硬件 | `motor_params.h`、`current_sense.*`、`adc.c/.ioc` |
| 改PWM频率/引脚 | `tim.c/.ioc`、`pwm_3ph.*`、`MOTOR_CONTROL_PERIOD_S` |
| 改编码器协议/分辨率 | `biss_frame.*`、`encoder_angle.*` |
| 改编码器SPI/DMA | `spi.c/.ioc`、`biss_encoder.*`、`biss_dma_buffer.*` |
| 改驱动芯片或引脚 | `drv8323.*`、`drv8323_board.*`、GPIO/SPI2配置 |
| 改校准动作 | `encoder_calibration.*`和`motor_params.h`校准参数 |
| 查控制状态/故障 | `g_motor_control_debug`和`MotorFaultCode` |

## 16. 第四种模式：编码器转速外环 + 电流内环

当前工程新增了 `MOTOR_CONTROL_ENCODER_SPEED_CURRENT`。它不是位置环，而是在已经验证的
编码器角度电流环外面增加一层机械转速 PI：

```text
BiSS-C 17 位位置 + sequence
    -> speed_estimator.c（每 1 ms、只处理新的 sequence）
    -> 电机轴机械转速 rpm（20 Hz 一阶低通）
    -> speed_pi.c（转速误差 -> Iq_ref，当前限幅 ±0.7 A）
    -> current_pi.c（10 kHz，Id/Iq -> Ud/Uq）
    -> SVPWM -> TIM8 PWM
```

两个实时频率的职责必须分清：

- 10 kHz：TIM8 快速节拍、ADC 同步采样、Clarke/Park、电流 PI、SVPWM；
- 1 kHz：由 10 kHz 节拍十分频，计算编码器差分转速；仅模式 4 执行速度 PI；
- 模式 3 也运行速度估算器，但只用于观测，不改变其直接 `Iq` 命令；
- ADC 回调中不运行速度 PI，速度环只能生成 `Iq_ref`，不能直接写 PWM。

### 16.1 四种模式的最终区别

| `.mode` | 电角度来源 | `Ud/Uq` 来源 | `Iq_ref` 来源 |
|---|---|---|---|
| `MOTOR_CONTROL_OPEN_VOLTAGE` | 虚拟角度 | 开环电压命令 | 不使用 |
| `MOTOR_CONTROL_OPEN_ANGLE_CURRENT` | 虚拟角度 | 电流 PI | 直接电流命令 |
| `MOTOR_CONTROL_ENCODER_ANGLE_CURRENT` | 编码器角度 | 电流 PI | 直接电流命令 |
| `MOTOR_CONTROL_ENCODER_SPEED_CURRENT` | 编码器角度 | 电流 PI | 1 kHz 速度 PI |

启动模式仍只由 `main.c` 的 `motor_config.mode` 决定。当前配置为模式 4，
`main.c` 预置 `MotorControl_SetSpeedCommand(50.0f)`。更换电机、减速器或负载后，
应先回到低电流的模式 3 验证，并将速度目标改回 0 后重新逐级试验。

运行中切换使用：

```c
MotorControl_SwitchToEncoderSpeedCurrent(target_motor_rpm);
```

从模式 3 进入模式 4 时，代码先等待速度估算器就绪，并用当前 `Iq` 预装载速度 PI，
避免切换瞬间的电流阶跃。退出模式 4 时保留当时的有功电流状态，再由已有电流斜坡过渡到
目标模式命令。

### 16.2 转速单位与减速器

编码器安装在减速器前的电机轴上，所以所有速度接口和调试量均为“电机轴机械 rpm”：

```text
减速器输出转速 rpm = 电机轴转速 rpm / 减速比
```

例如减速比为 50:1 时，命令 `50 rpm` 对应输出端约 `1 rpm`。当前代码没有把减速比放入
闭环计算，因为反馈传感器就在电机轴上。若今后把编码器移到输出端，不能只增加一个减速比
常量；必须重新定义速度接口、每转计数、方向、滤波频率和 PI 参数。

### 16.3 阅读转速环代码的顺序

1. `speed_estimator.h/.c`：先理解 `position_raw + sequence + dt -> rpm`，特别是 17 位回绕、
   重复 sequence 不制造假零速、以及方向校准。
2. `speed_pi.h/.c`：理解物理单位、输出限幅、反算抗饱和和无扰预装载。
3. `MotorControl_RunSpeedTask()`：理解 10 分频、模式 3 只观测、模式 4 才运行 PI。
4. `MotorControl_ApplyModeRequest()`：理解模式 3/4 之间如何无扰切换。
5. `MotorControl_CurrentSampleComplete()`：确认速度环产生的 `Iq_ref` 最终仍通过原 10 kHz
   电流内环执行。

### 16.4 更换电机、编码器、减速器或负载后的修改点

集中从 `motor_params.h` 检查，不要先盲调速度 PI：

| 变化 | 必查参数/代码 | 原因 |
|---|---|---|
| 编码器分辨率变化 | `MOTOR_ENCODER_COUNTS_PER_TURN`、BiSS 帧解析 | 直接决定 rpm 比例 |
| 编码器安装方向变化 | 重新执行编码器校准 | 决定电角度与速度正负号 |
| 电机极对数变化 | `MOTOR_POLE_PAIRS` 并重新校准 | 决定机械角到电角度的倍率 |
| PWM/控制频率变化 | `MOTOR_CONTROL_PERIOD_S`、电流 PI、速度分频 | 两层控制周期必须一致 |
| 电机电阻/电感变化 | `MOTOR_CURRENT_PI_*` | 电流内环带宽首先发生变化 |
| 转矩常数变化 | `MOTOR_SPEED_PI_*`、`MOTOR_SPEED_IQ_LIMIT_A` | 同一 Iq 产生的转矩不同 |
| 减速比、惯量、摩擦或负载变化 | 速度 PI、加减速斜率、Iq 限幅 | 外环对象的惯量和阻尼变化 |
| 电机/功率级额定电流变化 | 电流命令/保护限值 | 防止把控制限幅当成硬件保护 |

当前实机试验参数：命令限幅 ±100 rpm、命令斜率 20 rpm/s、速度 PI 输出限幅
±0.7 A、模式 4 电流指令斜率 10 A/s，`Kp=0.03 A/rpm`、`Ki=0.015 A/(rpm*s)`、
`Kaw=10 1/s`。这些值没有照搬
F407 工程中依赖隐含调用周期的离散系数；移植时将调用周期和物理单位显式化，避免 H743
控制频率变化后获得完全不同的闭环增益。当前 50 rpm 运行仍有约 3 rpm 量级的速度偏差，
并观测到电流反馈瞬态偏离，不能将这些试验值视为已完成安全定型。

### 16.5 模式 4 的关键 Watch 字段

- `speed_target_rpm`：用户要求值，已经过 ±100 rpm 限制；
- `speed_active_target_rpm`：经过 20 rpm/s 斜坡后的 PI 实际目标；
- `speed_raw_rpm` / `speed_filtered_rpm`：原始与低通后的电机轴速度；
- `speed_error_rpm`：实际送入 PI 的误差；
- `speed_pi_proportional_a` / `speed_pi_integrator_a`：PI 两部分，单位 A；
- `speed_iq_command_a`：速度环生成的 `Iq_ref`；
- `speed_pi_saturated`：是否碰到当前速度环电流限幅；
- `speed_estimator_ready`：是否已取得足够的新位置样本；
- `speed_control_tick_count`：真正以新编码器样本执行速度 PI 的次数。

带减速器时无法可靠手动转动电机轴，因此方向验证采用模式 3 的小电流驱动法。完整步骤见
`docs/h7-speed-loop-bring-up.md`。
