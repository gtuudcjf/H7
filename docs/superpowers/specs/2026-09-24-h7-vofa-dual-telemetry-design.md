# H743 VOFA+ 双串口三环遥测设计

## 目标与边界

在现有 H743 电机工程中增加供 VOFA+ 绘图的只读遥测。普通 USART1 和 USB CDC 虚拟串口均可独立启用、停用；两者同时启用时发送相同的采样格式。遥测故障、USB 未连接或上位机读取缓慢时，电机的 10 kHz 控制、模式状态机、保护和校准行为保持原样。

现有硬件配置为 USART1 TX=PB14、RX=PB15、115200/8N1，以及 USB OTG FS DM=PA11、DP=PA12 的 CDC 设备。遥测仅发送，不解析串口接收命令，不改变这两套外设的引脚和基本配置。VOFA+ 中两个端口分别作为独立 COM 口使用；串口编号由 Windows 枚举决定，不能固定为截图中的 COM3。

## 数据协议

采用 VOFA+ `JustFloat`：每帧为八个按序排列的小端 IEEE-754 32 位浮点数，随后附加固定的 4 字节帧尾 `00 00 80 7F`。整帧 36 字节，不含文本、时间戳或换行。

| 通道 | 数据源 | 含义 | 单位 |
|---|---|---|---|
| 1 | `g_motor_control_debug.id_ref_a` | d 轴电流活动目标 | A |
| 2 | `g_motor_control_debug.id_a` | d 轴电流反馈 | A |
| 3 | `g_motor_control_debug.iq_ref_a` | q 轴电流活动目标 | A |
| 4 | `g_motor_control_debug.iq_a` | q 轴电流反馈 | A |
| 5 | `g_motor_control_debug.speed_active_target_rpm` | 实际送入速度 PI 的目标 | rpm |
| 6 | `g_motor_control_debug.speed_filtered_rpm` | 滤波后的机械转速反馈 | rpm |
| 7 | `g_motor_control_debug.position_target_deg` | 单圈位置目标 | ° |
| 8 | `g_motor_control_debug.position_feedback_deg` | 单圈机械位置反馈 | ° |

速度通道 5 选择斜率限制后的活动目标，便于直接与速度 PI 的反馈比较；位置模式中它会跟随位置 P 环的输出。其它模式下不活动的环所对应字段保留控制层当前的调试值，不由遥测层伪造数据。启动、停机和故障阶段也可以发送；VOFA+ 看到的是控制层已发布的最近调试值。字段说明应提醒用户，停止后部分反馈字段可能保持最后一次值。

## 运行时序与模块边界

新增独立的 `motor_telemetry` 模块负责节拍、取样、JustFloat 编码和两个发送端口的状态。`main()` 完成外设与电机对象初始化后初始化遥测模块；主循环继续调用 `MotorControl_Service()`，再调用 `MotorTelemetry_Service()`。不在 TIM8、ADC 或 SPI4 中断中取样、编码或发起传输。

默认每 10 ms 产生一次样本，即 100 Hz。36 字节帧在 USART1 的 115200/8N1 下约需 3.13 ms，留有发送余量。用 `HAL_GetTick()` 和无符号差值检查节拍；主循环忙于现有 Flash 保存时不补发积压帧，只在后续服务周期继续取最新值。调试结构中的八个浮点字段在短暂关闭中断的临界区内复制，随后恢复进入前的中断状态。编码、端口状态检查和发送均在临界区外。

USART1 用 `HAL_UART_Transmit_IT()` 发送，配置低于电机实时中断的 USART1 IRQ 优先级。发送缓冲区在 UART 完成或错误回调前不能覆盖。USB CDC 用现有 `CDC_Transmit_FS()` 发起异步传输，只在设备已配置且上一帧完成时使用其独立缓冲区；`CDC_TransmitCplt_FS()` 和 `CDC_DeInit_FS()` 通知遥测模块释放端口忙状态。两个端口互不共享发送缓冲区或忙标志。一端忙、未连接或发起传输失败，只丢弃该端本周期帧；另一端照常发送。遥测层不得使用阻塞等待、忙循环或动态内存分配。

两个布尔使能项在模块初始化时分别设定，默认均开启。公开独立的运行时使能接口，供未来通信命令或调试操作切换。关闭端口时允许已经开始的一帧自然完成，后续帧停止；再次开启后从下一次服务节拍发送最新数据。无需更改控制层的目标命令接口。

## 改动范围

- 新增 `Core/Inc/motor/motor_telemetry.h` 与 `Core/Src/motor/motor_telemetry.c`：独立遥测接口与实现。
- 在 `Core/Src/main.c` 增加初始化、主循环服务和 UART 完成/错误回调转发。
- 在 `Core/Src/usart.c` 与 `Core/Src/stm32h7xx_it.c` 接通 USART1 TX 中断及低优先级 NVIC 配置；同步更新 `emptytest.ioc`，避免 CubeMX 再生成时丢失。
- 在 `USB_DEVICE/App/usbd_cdc_if.c` 的已有用户代码区接通 CDC 完成、断开通知，并防止未枚举时进入传输函数。
- 在 `MDK-ARM/emptytest.uvprojx` 中登记新增源码。
- 增加针对协议编码、发送调度和端口独立性的主机测试，以及 VOFA+ 使用说明。

不改 `motor_control.c`、FOC/PI 算法、PWM、ADC、BiSS-C 和已有的电机参数。新模块只读取已有的 `g_motor_control_debug` 字段，不直接读取控制层内部静态变量。

## 错误处理与观察

遥测发送错误不进入 `MOTOR_CONTROL_FAULT`，只在遥测模块中增加每端口的已发送帧数、忙而丢帧数和错误数，供 Keil Watch 观察。USB 未连接视为该端不可发送，不影响 UART；USART1 未连接也不会阻塞 USB。编码前校验浮点值是否有限，若控制层某个调试字段不是有限数，则该通道置零并增加遥测数据异常计数，避免 JustFloat 帧内出现特殊值造成绘图混乱。此处理只作用于发出的副本，不回写控制层。

## 验收

1. 主机测试验证八通道顺序、36 字节帧长、字节序、帧尾、非有限值处理、节拍与单端忙状态。
2. 现有工程集成检查保持通过；Keil 全量构建包含新模块且无编译/链接错误。
3. 普通串口与 USB CDC 各自连接 VOFA+ 时，按上述顺序显示八条波形；断开其中一路，另一条继续运行。
4. 实机运行时检查 10 kHz 控制与电流采样故障计数没有因遥测增加而异常。软件测试和构建无法代替此项硬件验证。
