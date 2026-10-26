# H743 + LAN9252 HBI 首次 OP 上板手册

本工程默认是 **HBI 通信专用固件**：保留三环控制源代码，但不初始化电机外设、不启动电机、不输出 PWM。目标是 TwinCAT 下 FreeRun、10 ms 周期进入 OP，验证 2 字节输出和 6 字节输入的 PDO 回环。

软件编译和主机测试不能证明实板已经进入 OP。本次没有连接调试器烧录、写板上 EEPROM 或操作 TwinCAT；下面的实板步骤由操作者执行并保存证据。

## 1. 文件和配置

| 项目 | 本次使用值 |
|---|---|
| Keil 工程 / 目标 | `MDK-ARM/emptytest.uvprojx` / `emptytest` |
| 启动宏 | `ETHERCAT_BRINGUP_ONLY=1`；工程和头文件均默认 1 |
| 通信固件 | `MDK-ARM/emptytest/emptytest.hex` |
| ESI | `EtherCAT/H743-LAN9252-HBI-LAB.xml` |
| 名称 | H743-LAN9252-HBI-LAB |
| 身份 | Vendor `0x00F0C743`，Product `0x9252`，Revision **2**，Serial 1 |
| LAN9252 PDI | `0x89`，16 位单阶段地址/数据复用 HBI |
| HBI PDI Configuration | `0x80`：低有效 CS/RD/WR/ALE；CS 限定地址锁存 |
| FMC | Bank4，`0x6C000000`，240 MHz，异步 NOR 总线类型、模式 A |
| 初始时序 | ADDSET=15，ADDHLD=15，DATAST=63，BUSTURN=15 |
| 同步 | FreeRun 轮询，DC/AL Event/SYNC 中断关闭 |

Vendor ID 是沿用 SPI 实验工程的本地实验值，不是 ETG 正式分配的产品身份。Revision 2 用于与先前 SPI Revision 1 区分。

不要烧录 `emptytest/motor-check/motor-check.hex`：它只是电机路径的编译回归产物。模式 0 保留了旧电机启动行为和 PB5 恒高行为；按本次 PDF，PB5 与 PC4 共网。恢复电机上板测试前需要单独解决旧行为与该接线的冲突，本次不使用模式 0 驱动实板。

## 2. 上电前逐项核查接口

接线依据项目根目录的 `SCH_Schematic1_2026-09-30.pdf` 第 1、2 页。F407 示例及其 V3 原理图仅用于对照，不是本板引脚来源。

| LAN9252 信号 | H743 引脚 | 固件配置 / 核查 |
|---|---|---|
| AD0 | PD14 | AF12 FMC D0；同时与 PA7 共网 |
| AD1 | PD15 | AF12 FMC D1；同时与 PA6 共网 |
| AD2 | PD0 | AF12 FMC D2 |
| AD3 | PD1 | AF12 FMC D3 |
| AD4 | PE7 | AF12 FMC D4 |
| AD5 | PE8 | AF12 FMC D5；同时与 PA4 共网 |
| AD6 | PE9 | AF12 FMC D6 |
| AD7 | PE10 | AF12 FMC D7 |
| AD8 | PE11 | AF12 FMC D8 |
| AD9 | PE12 | AF12 FMC D9；同时与 PA5 共网 |
| AD10 | PE13 | AF12 FMC D10 |
| AD11 | PE14 | AF12 FMC D11 |
| AD12 | PE15 | AF12 FMC D12 |
| AD13 | PD8 | AF12 FMC D13 |
| AD14 | PD9 | AF12 FMC D14 |
| AD15 | PD10 | AF12 FMC D15 |
| CS | PG12 | AF12 FMC NE4，低有效 |
| RD | PD4 | AF12 FMC NOE，低有效 |
| WR | PD5 | AF12 FMC NWE，低有效 |
| ALELO | PB7 | AF12 FMC NL/NADV；上升沿锁存地址 |
| ALEHI | PF1 | 固定高；单阶段不使用第二个锁存脉冲 |
| ALELO 另一共网脚 | PF0 | Analog/高阻，经 R94 连接，不能输出 FMC A0 |
| 其余独立地址脚 | PF2、PF3 | Analog/高阻 |
| SPI1 共网脚 | PA4、PA5、PA6、PA7 | 全部 Analog/高阻，不能初始化 SPI1 |
| IRQ | PE3 | 输入，仅观察 |
| SYNC0 / SYNC1 | PE0 / PE1 | 输入，不使用同步中断 |
| DRV_ENA 共网 | PB5、PC4 | 上电先预置两个输出锁存均为低，再切输出模式 |
| RST# | LAN9252 原 RC 复位网络 | 不映射成 MCU GPIO |

核对 R94～R97 的实际焊接和网络连通性，确认所有 AD 数据位顺序正确。当前方案不要求拆焊 R94：PF0 高阻让 PB7 单独驱动 ALELO。先确认供电、地线、25 MHz 晶振和复位正常，再通电。实测 PB5、PC4 均低，无电机 PWM。

## 3. 从已工作的 SPI 配置切换到 HBI

**ESI 是电脑描述，SII 是板上 EEPROM 二进制，两者不是同一文件。** 复制 ESI 不会改变板上接口模式，也不能把 XML 改名 BIN 后下载。

1. 保留能进入 OP 的 SPI 固件、SPI ESI 和原 TwinCAT 工程。只连接这一块待测板，确认从站位置与物理端口。
2. 通过当前可用的 EtherCAT 链路，进入从站 Advanced Settings → ESC Access → E²PROM / Hex Editor，执行设备到电脑的 Upload/Read，保存完整原 EEPROM。再次读取并比较，保存 SHA256、板号、时间。不要用 Download/Write 做备份。
3. 将新 ESI 放入本机实际使用的 TwinCAT `Config/Io/EtherCAT` 目录，重载 Device Descriptions 或重启 XAE。选择唯一的 **H743-LAN9252-HBI-LAB、Revision 2**；保留 SPI Revision 1 描述用于恢复。
4. 用 TwinCAT 的 EEPROM Update，从已加载的 HBI 描述生成匹配的 SII。核对下表。由工具重算配置区 CRC；不只手改一个 PDI 字节。ESI 声明的 2048 字节是这份实验 SII 的目标大小，不代表 24FC512 只有 2048 字节。
5. 切换过程中避免让 SPI 固件在 HBI 配置下运行，或让 FMC 在 SPI 配置下驱动共享数据脚。建议用调试器烧录 HBI 固件后**停在 main 入口、尚未初始化 FMC**，保持 MCU 不访问 LAN9252；随后通过 EtherCAT 写入候选 SII。
6. 写入前再次确认目标板、原始备份和候选内容。执行主机到 EEPROM 的写入，回读核验身份、配置区 CRC 及 SM/PDO 类别，然后彻底断电重启。MCU 与 LAN9252 都要重新上电，不能只复位 MCU。

| SII 核对项 | 候选值 |
|---|---|
| 身份 | Vendor `0x00F0C743`，Product `0x9252`，Revision 2，Serial 1 |
| ConfigData | `890E80CC8813F000000000800000` |
| PDI 读回 | ESC `0x0140=0x89`，`0x0150=0x80` |
| SM0 / SM1 | `0x1000` / `0x1080`；128 字节；Control `0x26` / `0x22` |
| SM2 / SM3 | `0x1100` / `0x1400`；2 / 6 字节；Control `0x64` / `0x20` |
| RxPDO | `0x1C12 → 0x1600 → 0x7000:01`，16 位 |
| TxPDO | `0x1C13 → 0x1A00 → 0x6000:01`，16 位；`0x6000:02`，32 位 |
| 协议 | CoE，FreeRun，无 DC |

本次没有生成或验证实板 SII 二进制；使用匹配版本 TwinCAT 从上述 ESI 生成。首次写后若身份不符，先检查完整回读，而不是绕过固件的 PDI/EEPROM 校验。

恢复 SPI 时同样先让 MCU 停止驱动总线，恢复原 SII 和 SPI 固件，再彻底断电重启。

## 4. 调试器观察与分层定位

Watch 中加入 `ethercat_init_status`、`ethercat_diagnostics`、`*ethercat_diagnostics` 和 `g_lan9252_first_fault`。诊断数据每 100 ms 更新，不能用 MCU 暂停时的快照判断连续周期性能。

| 观察项 | 正常值 / 含义 |
|---|---|
| `ethercat_init_status` | 1=初始化 READY；2=LAN9252 错误；3=SSC 错误。READY 不等于 OP |
| `byte_test` | `0x87654321`；由 `base+0x64` 和 `base+0x66` 两次半字读取拼接 |
| `id_rev` | 高 16 位为 `0x9252` |
| `hw_cfg` | READY 位 `0x08000000` 置位 |
| `pdi_control` | `0x89`，不是 EtherCAT AL 状态 |
| `eeprom_status` | 保留原值；`0xF800` 掩码下应无错误/未就绪位 |
| `al_status` / `al_status_code` | AL 低四位 1=INIT、2=PREOP、4=SAFEOP、8=OP；bit4=错误；正常 code=0 |
| `last_driver_error` | 0=健康；非零首错锁存，不因后续读成功清除 |
| `hbi_timeout_count` | 初始化以来的超时计数，不因后续读成功清零 |
| `pdi_error_count` / `lost_link_count` | 当前硬件饱和计数快照；比较基线与终值，不当作永久累计 |
| `ethercat_service_max_us` | 初始化以来单次服务最大耗时，不是主站周期或网络时延 |
| `rx_control_word` / `tx_status_word` / `tx_cycle_counter` | 最近一次应用数据影子 |

驱动错误枚举：1 参数，2 总线错误，3 Direct 超时，4 BYTE_TEST 不匹配到期，5 READY 超时，6 芯片 ID，7 PDI，8 CSR 超时，9 PRAM 超时，10 EEPROM，11 SM 超时，12 微秒时基不可用。BYTE_TEST 不匹配使用枚举 4，不计入 `hbi_timeout_count`；应同时查看 `last_driver_error`。

固件显式解锁 Cortex-M7 DWT，然后用固定次数指令检查 CYCCNT 确实递增，不依赖调试器解锁。若自检失败，记录错误 12，不开始 HBI 探测。正式冷启动验收至少有一轮脱离调试器运行。

`g_lan9252_first_fault.valid=1` 才解释其他字段：`address` 是 LAN9252 物理寄存器偏移，`esc_address` 是逻辑 ESC 请求（`0xFFFFFFFF` 表示直接探测），`write` 表示方向，`phase=1/2` 表示低/高半字，`reason=1/2/3` 表示访问前超预算/回调失败/完成过晚。首个物理故障保留到 MCU 复位；BYTE_TEST/PDI 等语义不匹配可能没有物理故障快照。

| 失败位置 | 优先检查 |
|---|---|
| HardFault，尚无 BYTE_TEST | MPU 区域、FMC 时钟、Bank4、访问宽度；当前 region1 为 TEX/C/B=010/0/0 的非共享 Device、不可执行 |
| BYTE_TEST 全 FF / 全 0 / 错位 | EEPROM 是否切到 HBI、ALE/CS/RD、AD 接线与共网 GPIO、供电/复位 |
| BYTE_TEST 正确但 READY 不就绪 | EEPROM 读载入、晶振、复位和供电稳定性 |
| PDI / EEPROM 错误 | 用 TwinCAT 网络侧读 `0x0140/0x0150` 和 EEPROM；重新核对候选与回读 |
| CSR / PRAM 超时 | 记录首错；检查读写时序、两半字访问及共享引脚是否被重复初始化 |
| PREOP 不能到 SAFEOP | 身份、邮箱、SM/PDO 映射、同步配置和 AL Status Code |
| SAFEOP 不能到 OP | 主站周期帧、输出有效、watchdog、AL Status Code |

驱动启动探测失败后不自动循环重试。若首次供电/EEPROM 载入较慢，待配置正确、供电稳定后重新上电。发生不完整半字对或物理总线故障时，优先对 MCU/LAN9252 一起断电重启，避免沿用 LAN9252 内部未完成的 DWORD 状态。

不要在 Watch/Memory 窗口直接连续浏览 `0x6C000000` 地址区：调试器可能执行额外读取，破坏 FIFO 或 DWORD 计数。看软件诊断结构即可。不要在总线访问中间单步。

## 5. TwinCAT 逐级进入 OP

1. 使用 SPI 已验证的同一专用网卡、网线及 LAN9252 IN 端口。扫描应显示 HBI-LAB、Revision 2；移除旧实验工程里对 Revision 1 的错误绑定。
2. 在 Config Mode 下设实际周期为 **10 ms**，同步为 FreeRun，关闭 DC。使用 Toggle Free Run State 启动实际周期 I/O；若此前在 Run Mode，按 TwinCAT 要求先 Reload Devices。
3. 确认主站 Cyclic 帧计数持续增长，再依次请求 INIT、PREOP、SAFEOP、OP。每一步记录原始 AL 状态与 code，不靠 LED 判断。
4. 正常 OP 要同时满足：从站 `al_status` 低四位 8 且无错误位、`al_status_code=0`、主站状态正常、`WcState=0`。
5. 若失败，先记录完整 AL code、主站错误文字和 MCU 诊断。不要为进入 OP 关闭 watchdog、伪造映射或强行吞掉访问错误。

## 6. PDO 回环与稳定性验收

主站周期输出 ControlWord 依次设置 `0x0000 → 0x0001 → 0x0002 → 0x0003`，每个保持至少 1 秒。预期 StatusWord 为 `0 → 1 → 0 → 1`，其余位为 0。这不是 CiA402 状态字；输出不会控制电机。不要只用 SDO 写值代替周期 PDO。

输入 6 字节是 little-endian 的 StatusWord 两字节加 CycleCounter 四字节。CycleCounter 在有效 OP 应用迭代中递增，不保证每个主站周期只加 1，不能用它反推总线周期。离开有效交换或驱动失败会复位应用影子；主站掉线时可能冻结旧输入，应结合 MCU 快照判断。

连续运行至少 10 分钟，保存开始、过程与结束的 OP/code、WcState、PDO、HBI 超时、PDI/链路计数及服务最大耗时。无新增 HBI 超时、无持续通信错误，PB5/PC4 全程低且无 PWM。

再完成 3 次完全断电重启，以及一次拔线数秒后恢复测试。纯网络掉线且 HBI 健康时由主站和 SSC 状态机恢复，必要时确认错误并重新逐级请求。若出现 HBI 首错锁存，先排除物理原因并完全重启；不要不断自动清错。1 ms 周期是后续性能项目，不属于本轮 OP 的前置条件。

记录表：

| 轮次 / 时间 | 固件 SHA256 | 实际周期 | AL/code | WcState | 回环 | HBI 超时前/后 | PDI/链路计数前/后 | PB5/PC4/PWM | 结果 |
|---|---|---|---|---|---|---|---|---|---|
| 首次 10 分钟 | | 10 ms | | | | | | | |
| 冷启动 1 / 2 / 3 | | 10 ms | | | | | | | |
| 拔线 / 恢复 | | 10 ms | | | | | | | |

## 7. 重新构建和 CubeMX

在工程根目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_ethercat_suite.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/build_hbi.ps1
```

前两项使用本机 GCC/TinyCC/Clang，可用 `NATIVE_CC` 指定路径；SSC 旧源码仅对 GCC 关闭 misleading-indentation 警告，其余保持 `-Wall -Werror`。构建脚本用现有 ARMCC 5，先对两个模式做真实预处理，再分别完整编译模式 0 和模式 1，最后保留默认的通信固件。个人 `.uvoptx` 逐字节保留；不自动烧录。

`emptytest.ioc` 已按 HBI 引脚、时序和安全电平维护；**本次未执行 CubeMX 完整重生成**。重生成后必须审查 diff 并重新执行全部检查：

- 保留 main 中包住生成外设调用的 USER CODE 条件边界，通信模式只能进入 GPIO/FMC/EtherCAT。
- 保留 `HAL_SRAM_MODULE_ENABLED`、新增 Keil 源文件组和包含目录。NOR 类型描述的是 FMC 总线，不允许执行 NOR Flash 识别命令。
- `fmc.c` 的 USER CODE 0 保留自定义配置和 SRAM MSP；`FMC_Init 0` 调用它并提前返回，跳过生成器可能追加的 NOR 初始化。不得删除这个返回或改用 `HAL_NOR_Init`。
- 确认 GPIO 的通信分支、安全输出顺序、PA4～PA7/PF0 高阻以及 MPU region1 没有被覆盖。不能直接把生成结果烧录上板。

## 参考资料

- [Microchip LAN9252 数据手册](https://ww1.microchip.com/downloads/en/DeviceDoc/00001909A.pdf)：HBI 16 位字地址映射、DWORD 组合、时序及 EEPROM PDI 配置。
- [ST RM0433](https://www.st.com/resource/en/reference_manual/rm0433-stm32h742-stm32h743-753-and-stm32h750-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)：H743 FMC 和内存映射。
- [Arm Cortex-M7 DWT 解锁说明](https://community.arm.com/forums/f/architectures-and-processors-forum/55634/cortex-m7-how-to-enable-watchpoints)：DWT LAR 的软件锁与解锁值；寄存器字段同时按项目实际 CMSIS `core_cm7.h` 核查。
- [SOES LAN9252 驱动](https://github.com/OpenEtherCATsociety/SOES/blob/master/soes/hal/rt-kernel-lan9252/esc_hw.c)：CSR/PRAM 流程对照；本项目复用自身已验证的 SSC，并未切换到 SOES。
- [Beckhoff Toggle Free Run State](https://infosys.beckhoff.com/content/1033/tc3_userinterface/2954025611.html)、[ESI 安装说明](https://infosys.beckhoff.com/content/1033/ethercatsystem/1036998411.html)、[Online/EEPROM 与状态](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1446518411.html)。
