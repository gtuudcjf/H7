# H743 三环控制 VOFA+ 双端口遥测

## 工程与数据格式

遥测仅在主循环 `MotorControl_Service()` 后运行；不修改控制算法、参数、PWM、ADC 或编码器链路。主循环每隔至少 10 ms 采样一次（名义 100 Hz），UART 与 USB CDC 各自拥有独立的 36 字节发送缓冲区。异步发送期间不会覆盖缓冲区；端口忙或 USB 未就绪时丢弃当前帧，不排队、不等待。

两端口使用相同的 VOFA+ **JustFloat** 格式：8 个小端 float32，后接 `00 00 80 7F`。不要在这些端口混发 printf 文本或其他协议。

| VOFA 通道（从 0 开始） | 调试字段 | 含义 | 单位 |
| --- | --- | --- | --- |
| 0 | `id_ref_a` | d 轴电流给定 | A |
| 1 | `id_a` | d 轴电流反馈 | A |
| 2 | `iq_ref_a` | q 轴电流给定 | A |
| 3 | `iq_a` | q 轴电流反馈 | A |
| 4 | `speed_active_target_rpm` | 速度 PI 实际使用的斜坡后给定 | rpm |
| 5 | `speed_filtered_rpm` | 滤波速度反馈 | rpm |
| 6 | `position_target_deg` | 位置目标 | ° |
| 7 | `position_feedback_deg` | 单圈位置反馈 | ° |

字段来自 `g_motor_control_debug`。非有限数值（NaN/Inf）发送为 0。位置环模式下，通道 4 是位置环输出经速度斜坡处理后的给定，并非固定启动速度命令；不在某环运行的模式中，对应字段可能为 0 或保留最近值。停机/故障后遥测继续发送调试字段，不意味着电机仍运行或反馈仍更新。请结合原有控制状态和故障字段判断。

## 普通串口 USART1

板上连接为 PB14/TX、PB15/RX；按原理图的 USART1_TX/RX 网络确认连接器脚位。使用 **3.3 V TTL USB 转串口**，板 TX 接转换器 RX，GND 共地；本功能不需要板 RX。不要接 RS-232 电平，也不要接入转换器的电源脚给已供电板子重复供电。

VOFA+ 设置：数据引擎 `JustFloat`，数据接口 `串口`，选择转换器对应 COM 号，115200 波特率，8 数据位、1 停止位、无校验、无流控。36 字节/帧、100 Hz 占用约 36000 bit/s（含 8N1 开销），约 31% 串口带宽。

USART1 中断优先级为 5，低于 TIM8/ADC（0）、SPI4/DMA（1）、TIM6（2）和 USB（4）。使用 `HAL_UART_Transmit_IT`，没有阻塞等待和 DMA 资源改动。

## USB 虚拟串口

使用支持数据传输的 Type-C 线连接板上 USB CDC 接口（PA11/DM、PA12/DP）。板子保持正常电机电源和安全接地条件；本软件不判断 USB/电机电源回灌，连接前按板子供电设计确认。

Windows 枚举出新的 COM 号后，VOFA+ 同样选择 `JustFloat` + `串口`，但选择 **板子的 USB CDC COM**，不是 USB 转串口的 COM。可填 115200/8N1；USB 数据速度不由该波特率决定，当前工程不使用主机 line coding 改变遥测周期，也不以 DTR 作为发送开关。

USB 未枚举时安全跳过发送；主机不读数据时可能保持 busy，后续帧被丢弃。断开/复位的 CDC DeInit 会释放在途状态，并将未完成帧计为错误；重新枚举后自动恢复。软件不因断开而改变电机状态。

## 两端口独立启停

默认两端口均开启。在 `Core/Inc/motor/motor_telemetry.h` 分别设置 `MOTOR_TELEMETRY_UART_ENABLED`、`MOTOR_TELEMETRY_USB_ENABLED` 为 `true` 或 `false`，重新编译即可改变开机默认状态。

运行中可在**主循环上下文**调用：

```c
MotorTelemetry_SetUartEnabled(false); /* 仅停止新的 UART 帧 */
MotorTelemetry_SetUsbEnabled(true);   /* USB 独立运行 */
```

禁止在 ISR 调用 Service 或启停函数。禁用不会取消已提交帧，该帧仍可完成；不要再次 Init 活跃模块。不要直接修改 debug 镜像作为控制开关，当前没有实现上位机接收命令。

遥测端口的忙状态、完成数、丢帧数和错误数只在模块内部维护，不再复制为全局 debug 镜像。调试时直接观察 UART 与 USB 两路 VOFA 波形是否连续，并分别启停两个端口确认互不阻塞。USB 未连接或主机停止读取时，该路允许丢帧；这不会阻止 UART 继续发送。

## 波形与实机验证

VOFA 图表的 Δt 设为 **10 ms**（截图中的 1 ms 不是本项目采样周期），建立 8 个通道，按上表命名。JustFloat 帧没有时间戳；若丢帧或主循环被其他服务延迟，VOFA 的均匀时间轴不能代表精确实际时间。100 Hz 用于低速环路趋势观察，不能用来验证 20 kHz 电流环纹波。

1. 保持现有限流、保护和机械安全条件，先只接普通串口，确认 8 通道及电流/速度/位置给定与反馈对应；对照 Keil 调试字段。
2. 仅连接 USB，确认 CDC COM 枚举和同样的通道；串口不接不影响 USB。
3. 两端口都接，用两个 VOFA 实例/连接分别打开两个 COM（同一 COM 不得被重复占用），确认各自波形。
4. 分别禁用 UART、USB，确认只停止相应端口的新帧；重新开启应恢复。
5. USB 拔插、主机停止读取，确认 USB 波形可恢复，并确认 UART 与原有控制未受影响。
6. 对照新增前后控制误差，以及原有实时超时/故障计数。软件构建成功不等于实机实时性已验证。

## 阅读代码

`motor_telemetry_core.c` 负责帧编码、10 ms 调度、独立忙状态与计数；`motor_telemetry.c` 负责八字段快照与 HAL/CDC 适配。UART 完成/错误回调在 main.c，USB 完成/DeInit 回调在 usbd_cdc_if.c。USB 提交只短暂屏蔽 USB 中断，控制中断仍可抢占；八字段快照才使用保存/恢复 PRIMASK 的短临界区。

新增接线均在 CubeMX USER CODE 区域，USART1 IRQ 同时记录在 .ioc。重新生成后仍应复核 Keil Motor 组中两个遥测源文件与中断是否重复生成或遗漏，并重新运行接线测试。

## 软件验证范围

执行 `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1`，包含编码/调度和 H7 适配层行为测试、接线检查，以及现有六项配置/集成回归。主机行为测试使用 HAL/CDC 替身，不模拟物理 USB 和 ISR 实际延迟；实际 COM 枚举、拔插恢复和电机控制误差仍须按上述步骤实机验证。
