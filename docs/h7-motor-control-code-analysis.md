# STM32H743 电机控制工程代码逻辑分析

本文用于理解当前工程，不代表所有功能均已完成实机验证。分析日期为
2026-09-14，后续调试应以最新测试结果和硬件保护条件为准。

## 1. 当前功能状态

| 功能 | 代码状态 | 当前实机状态 |
|---|---|---|
| TIM8六路互补PWM、DRV8323驱动 | 已实现 | 已验证，可驱动电机 |
| 模式1：电压开环 | 已实现 | 已验证 |
| PB1/I_A、PB0/I_B两电阻采样 | 已实现 | 已验证能采集并完成零偏校准 |
| 模式2：虚拟角度、电流闭环 | 已实现 | 已验证电机能够低速转动 |
| SPI4模拟BiSS-C读取17位位置 | 已实现 | 已验证，可稳定取得有效位置帧 |
| 编码器机械角/电角度换算 | 已实现并有算法测试 | 尚未完成整机角度一致性验证 |
| 编码器方向与电角度零点校准 | 已实现 | 尚未实机验证 |
| 模式3：编码器角度同步、电流闭环 | 已实现 | 尚未实机验证 |

注意：模式3不是“位置环”。它只是用编码器给出的转子电角度代替虚拟电角度，
使d/q电流跟随真实转子坐标系。当前工程没有位置给定、位置误差或位置PI；未来
的速度环应位于电流环外层，由速度PI生成`Iq_ref`。

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
8. 最后阅读尚未验证的校准路径：
   - `encoder_calibration.c` → `motor_config_store.c`。

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
零偏校准完成之后。

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
看到`received_crc != calculated_crc`，同时`frame_status`仍为OK。需要判断同一帧数据
时应暂停CPU再读取；判断长期稳定性应观察各错误计数是否持续增加。

## 8. 17位位置到FOC电角度

这一段在`encoder_angle.c`中实现，但当前还依赖未实机验证的校准结果：

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

目前能够读取`encoder_position_raw`只证明通信和帧解析正确，不能证明上述零点、方向
及最终电角度正确。

## 9. 三种控制模式的联系与区别

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
- 当前状态：软件路径已存在，但在零点校准完成前不应进行实机运行验证。

三种模式最终都复用同一个`MotorControl_WriteVoltage()`、SVPWM和TIM8输出层，因此
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
新流程仍需逐状态实机验证，不应直接启动模式3。尤其要核对：

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

## 12. 当前阶段建议观察的Keil字段

在暂不进行校准的情况下，先持续验证编码器采集稳定性：

- `mode`、`requested_mode`、`run_state`、`fault`；
- `encoder_ready`应保持1；
- `encoder_sequence`和`encoder_valid_count`应持续增加；
- `encoder_age_ticks`通常应保持很小；
- `encoder_position_raw`应与机械位置一致且范围正确；
- 五类编码器错误计数不应持续增加；
- `encoder_calibrated`保持0是当前阶段的正常结果；
- `encoder_mechanical_angle_pu`和`encoder_electrical_angle_pu`在未加载有效校准记录、
  且未进入模式3时可能保持0，不能据此否定原始位置采集。

后续开始零点校准时，再单独制定低电流、限位和故障退出的实机测试步骤。
