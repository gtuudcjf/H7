# DRV8323 从 F407 到 H743 的移植对比

## 1. 对比范围和有效基线

原 F407 工程同时存在 `Core/Src/DRV8323.c` 和 `Function/DRV8323.c`，但 Keil
工程文件实际编译的是 `Function/DRV8323.c`。因此本文以 `Function` 目录版本
作为原工程有效基线，与 H7 工程的以下模块对比：

- `Core/Src/motor/drv8323.c`：与 MCU 引脚无关的 16 位 SPI 寄存器访问；
- `Core/Src/motor/drv8323_board.c`：SPI2、片选和使能引脚绑定；
- `Core/Src/motor/motor_control.c`：驱动与 PWM 的安全启动/停止顺序。

## 2. 硬件资源映射

| 功能 | 原 F407 工程 | H743 新工程 | 说明 |
|---|---|---|---|
| SPI 外设 | SPI1 | SPI2 | 按 H7 原理图和 CubeMX 框架迁移 |
| SCLK | PA5 | PI1 | H7 使用 AF5 SPI2 |
| MISO/SDO | PA6 | PC2 | DRV8323 SDO 为开漏，PCB 必须提供上拉 |
| MOSI/SDI | PA7 | PC3 | H7 使用 AF5 SPI2 |
| 软件片选 | SPI1_NSS | PC1/DRV_CS | 空闲保持高，传输期间拉低 |
| 驱动使能 | DRV_ENA | PC4/DRV_ENA | 初始化默认低，配置前拉高 |
| CSA 校准 | 原控制流程使用 | PC5/DRV_CAL | 当前开环阶段保持低，电流环阶段再接入 |
| 附加常高控制 | 无对应迁移逻辑 | PB5 | 按当前硬件要求初始化后持续为高 |

## 3. SPI 配置和帧格式

DRV8323 的一帧固定为16位：B15为读写位，B14:B11为4位地址，B10:B0为
11位数据。数据 MSB First，SCLK 空闲为低，芯片在下降沿采样 SDI，因此两版
工程都采用 CPOL=0、CPHA=2EDGE。

H7 的 SPI2 使用16位数据帧，`HAL_SPI_TransmitReceive()` 的 `Size=1`，缓冲区
为 `uint16_t`，所以每次片选低电平期间发送且只发送一个完整16位字。

原 F407 有效代码虽然也把 SPI1 配置为16位模式，却使用两个字节的缓冲区并将
`Size` 传为2。HAL 在16位模式下把 `Size` 解释为半字数量，这可能导致发送两个
半字、访问缓冲区边界之外的数据，并使 DRV8323 因时钟数不是16而拒绝该帧。
H7 实现消除了这个隐患。

H7 SPI2 内核时钟约为64 MHz，32分频后 SCLK 约为2 MHz，低于 DRV8323
允许的10 MHz上限。软件片选在每次 HAL 收发前拉低、收发后无条件拉高。
数据手册还要求相邻帧之间 nSCS 高电平不少于400 ns；当前函数调用和 HAL
路径提供了实际间隔，但代码没有通过硬件定时器显式约束这个最小时间。

## 4. 配置寄存器等效性

| 寄存器 | H7 配置字 | 与原工程对应参数 | 实际含义 |
|---|---:|---|---|
| Driver Control, 0x02 | `0x0001` | `PWM_MODE_6X`, `CLR_FLT=1` | 6x PWM，清除锁存故障 |
| CSA Control, 0x06 | `0x02C3` | `VREF_DIV_2`, `CSA_GAIN_40`, `SEN_LVL_1_0` | VREF/2、40 V/V、电流检测阈值1.0 V |
| OCP Control, 0x05 | `0x031F` | `DEADTIME_400NS`, `OCP_LATCH`, `OCP_DEG_4US`, `VDS_LVL_1_88` | 400 ns驱动死区、锁存式OCP、4 us消隐、1.88 V阈值 |

高、低侧栅极驱动电流寄存器在原工程初始化函数中没有写入，H7 也保持芯片
复位默认值，所以两者在该部分同样等效。

注意：TIM8 当前 `BDTR.DTG=0`，表示 MCU 互补 PWM 本身不插入死区；上表
400 ns是 DRV8323内部配置的驱动死区，两者属于不同层级。

## 5. ENABLE、启动和停止顺序

原工程把 ENABLE 拉高后等待100 ms，再分别写三个寄存器，每次写入之间也等待
100 ms。H7 在 ENABLE 拉高后等待5 ms。DRV8323数据手册规定 ENABLE 拉高后
SPI ready和输出就绪时间为1 ms，因此5 ms满足启动要求，且不会把延时带入
10 kHz快速中断。

H7 启动顺序：

1. GPIO 初始化时 PC4保持低，TIM8通道尚未输出；
2. `Drv8323_EnableAndConfigure6Pwm()` 拉高PC4并等待5 ms；
3. 写入 Driver Control、CSA Control和OCP Control；
4. 三次 SPI 传输均返回 HAL_OK 后，启动 TIM8 CH1/1N、CH2/2N、CH3/3N；
5. 六路 PWM 全部成功后才把控制模式切换为 OPEN_LOOP。

H7 停止顺序：

1. 先把软件模式切换为 STOPPED，阻止快速中断继续更新CCR；
2. 停止三路主 PWM 和三路互补 PWM；
3. 最后拉低 PC4，使驱动进入关闭/休眠过程。

该停止顺序符合芯片对 ENABLE 拉低前先让 INHx/INLx 进入低电平的要求。

## 6. H7 实现相对原工程的改进

- 通用寄存器驱动与 H7 板级引脚分离，后续换板只改 board 层；
- 参数、地址和数据范围有显式检查；
- SPI 忙、超时和错误能够逐层返回到 `main()`；
- 配置失败会拉低 ENABLE，而原工程初始化无条件返回成功；
- PWM部分启动失败会回滚已启动通道，并禁止驱动；
- 阻塞 SPI 和 HAL_Delay 只存在于启动流程，不进入10 kHz快速中断；
- 控制层为后续电流环和速度环保留统一入口，算法层不直接依赖HAL。

## 7. 当前仍需关注的事项

1. **缺少写后读回校验。** HAL_OK只能证明MCU完成了SPI传输，不能证明
   DRV8323存在且接受了配置。带母线前建议读取0x02、0x05、0x06并比较有效位。
2. **确认SDO上拉。** DRV8323的SDO是开漏输出；如果PCB没有外部上拉，
   写操作可能仍显示HAL_OK，但读回数据不可靠。
3. **确认帧间片选高电平。** 数据手册要求两帧间 nSCS 高电平至少400 ns。
   后续加入读回校验时，可以用逻辑分析仪确认或在驱动层显式保证该间隔。
4. **故障输入未纳入软件。** 当前硬件未配置BKIN，按现阶段要求不处理；
   后续带功率测试前仍建议规划 nFAULT 状态读取和软件停机路径。
5. **CSA校准尚未接入。** PC5保持低符合当前开环阶段；移植电流环时应先完成
   ADC零偏采样，再决定何时使用DRV_CAL校准内部CSA。
6. **栅极驱动电流沿用默认值。** 带功率前需要结合MOSFET栅极电荷、开关损耗、
   EMI和振铃实测，确认默认 IDRIVEP/IDRIVEN 是否合适。

## 8. 对后续闭环移植的接口意义

电流环加入后，应保留 `Svpwm_Compute()`、`Pwm3ph_ApplyDuty()` 和当前驱动层，
用电流 PI 的 `ud/uq` 输出替代 `OpenLoop_Step()` 给出的固定电压。速度环应以
更低频率运行，只生成 `iq` 目标；编码器读取、速度估算和阻塞通信不得放进
TIM8的10 kHz快速中断。这样开环、调试和闭环模式能够共享同一套驱动与PWM
安全管理逻辑。

## 9. 参考资料

- Texas Instruments, DRV832x 6-to-60-V Three-Phase Smart Gate Driver Datasheet:
  https://www.ti.com/lit/ds/symlink/drv8323.pdf
