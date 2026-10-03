# H743 + LAN9252 首次 OP 台架操作手册

本手册对应 `emptytest` 的 EtherCAT-only 固件。目标是先验证通信回环，再逐步缩短主站周期。代码和主机测试通过不代表实板已经进入 OP；本任务没有烧录 H743、写 EEPROM 或进行实板验收。保存每一步的截图、诊断数值、固件提交号、TwinCAT build、网卡型号和配置，失败时停在该检查点。

## 1. 台架与固件准备

1. 确认接的是本块 H743/LAN9252 板，辨认 LAN9252 的 IN/OUT 端口与 SPI1 引脚。专用网线直连主站网卡和从站 IN，不经过办公网络、普通交换机或路由器。
2. 在硬件允许逻辑电源独立供电的情况下，保持电机功率母线断电。先核对原理图和实际供电连接，不假设存在某个跳帽或板卡支持某种分离供电拓扑。检查 PC4/DRV enable 实际为低；功率级始终关闭，PDO 只回环，不是电机指令。
3. 工程 `MDK-ARM/emptytest.uvprojx`，目标 `emptytest`，保留当前 ARMCC 5 配置。C/C++ 的 Define 必须包含 `ETHERCAT_BRINGUP_ONLY=1`；`Core/Inc/main.h` 的默认值也是 1。不要为了 OP 改成 0：模式 0 会走原电机启动流程。SSC 配置保持 FreeRun、轮询、DC/AL Event IRQ/SYNC 中断关闭。
4. 运行下列主机检查，**依次运行，不要并行**（测试共用 TEMP 下的 TinyCC 工具和输出）：

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_ethercat_suite.ps1
   powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1
   ```

5. Task 9 的完整 Keil 编译、链接和 map 检查通过后才能选取对应固件。后续烧录须由操作者明确执行/授权；本手册不自动烧录。完整构建应备份/校验个人 `.uvoptx`，不得让构建覆盖个人调试设置。
6. 新增 Keil 组 `Application/User/EtherCAT`（6 个源文件）及 `Middlewares/EtherCAT_SSC`（7 个源文件）。包含 `../Core/Inc/ethercat` 和 `../Middlewares/Third_Party/EtherCAT_SSC/Inc`；F407 参考工程不参与构建。

启动顺序是公共 MPU/HAL/系统时钟 → GPIO → 明确 PC4 低、PA4 CS 高 → SPI1 → `EtherCAT_Init()` → 不断 `EtherCAT_Service()`。仅 SPI1 开始工作（SPI123 内核 64 MHz、分频 16、SCK 4 MHz、模式 0、8 位、软件 CS）。ADC、TIM6/TIM8、SPI2/3/4、USART1、USB 不在此模式初始化。无 PWM、ADC 启动、编码器初始化/标定、电机服务、遥测服务或命令解析；HAL 电机/编码器回调也被编译门关闭。现有时钟、USB 和 SPI2/3/4 配置未修改，模式 0 的启动次序保留。

### CubeMX 重生成检查

`emptytest.ioc` 保留 SPI1/PA4/PA5–7、PE0/1/3 输入和 PC4 输出；`ProjectManager.KeepUserCode=true`。`main.c` 的 `USER CODE SysInit` 内开始 `#if ETHERCAT_BRINGUP_ONLY`，其 `#else` 包住生成的外设调用，`USER CODE 2` 内关闭条件；循环和回调条件也在 USER CODE 内。因此重生成的外设调用仍在电机分支。不要移动条件边界到生成区之外。

每次重生成后先看 diff，再重跑上述检查。CubeMX 可能重写 Keil Define、包含目录或用户组，必须恢复/核对这些项；它还可能给模式 0 的生成列表补上 `.ioc` 中的 SPI1 调用，需人工审查其影响。不得顺手改 ADC/TIM 电机配置。默认 H7 检查使用真正 ARM 预处理器检查模式 1 和模式 0，不能用 `-HardwareOnly` 代替完整安全检查。

## 2. 三类状态与调试器观察

在 Keil Watch 加入 `ethercat_init_status`、`ethercat_diagnostics` 和 `*ethercat_diagnostics`，展开后按下表记录。结构在服务内部静态保存，指针是供调试器使用的全局 volatile 指针；无需增加串口输出。调试器暂停 MCU 会停止轮询并可能引发主站看门狗，正式稳定性记录须保持 MCU 运行，观察时注明暂停过。

| 观察项 | 含义与检查 |
|---|---|
| `ethercat_init_status` | 初始化返回结果：0 UNINITIALIZED、1 READY、2 LAN9252_ERROR、3 SSC_ERROR。READY 仅表示初始化成功，不是 OP；此变量不会随之后故障自动改写。 |
| `byte_test`、`id_rev`、`hw_cfg` | SPI 探测证据：BYTE_TEST 应为 `0x87654321`，ID_REV 高 16 位应为 `0x9252`，HW_CFG READY 位应置位。 |
| `pdi_control`、`eeprom_status` | 上电 probe 得到的原始 ESC PDI/EEPROM 值；PDI 必须为 `0x80`，EEPROM 错误由驱动检查。PDI `0x80` 是接口类型，**不是 EtherCAT OP 状态**。 |
| `al_status`、`al_status_code`、`al_event` | 运行时每 100 ms 读取 `0x0130`、`0x0134`、`0x0220`（16 位）。AL Status 低四位 1 INIT、2 PREOP、4 SAFEOP、8 OP；bit4 表示错误。AL code 是错误原因，不把它当状态。 |
| `last_driver_error`、`spi_timeout_count` | 首个端口故障保持到显式重新初始化；超时计数不因后续一次读成功清零。具体枚举见 `Core/Inc/ethercat/lan9252_spi.h`。 |
| `pdi_error_count`、`lost_link_count` | PDI 错误来自 `0x030D` 单字节；lost-link 是 `0x0310..0x0312` 三个单字节计数之和，包含通常不用的端口 2。硬件计数会饱和，是当前快照，不是软件终身累计。先保存基线，再看增长。 |
| `ethercat_service_max_us` | 本次初始化以来单次服务的最大耗时；不是主站周期/网络延迟，也不能单独证明 1 ms 实时性。 |
| `rx_control_word`、`tx_status_word`、`tx_cycle_counter` | 每 100 ms 发布的通信影子；transport 故障时复位为零。 |

寄存器位置、PDI 从 EEPROM 装载及 Host PDI 范围参见 [Microchip LAN9252 数据手册 DS00001909A](https://ww1.microchip.com/downloads/en/DeviceDoc/00001909A.pdf) 第 12.3、12.8、12.14 节。这里要求精确 `0x80` 是本项目 SPI 配置的约束，不能用任意 Host PDI 值替代。

TwinCAT 的 `InfoData.State` 是**主站组合诊断值**：低四位也是 1/2/4/8，但高位含身份不符、不存在、连接等信息。例如不是看到任意含 8 的数字就判定正常 OP；记录完整十六进制值并与从站原始 AL status/code 对照。`WcState=0` 表示上一个周期工作计数有效，不能替代 AL 状态，也不能单凭它定位所有从站的根因。[Beckhoff Online/InfoData.State](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1446518411.html)、[Beckhoff WcState](https://infosys.beckhoff.com/content/1033/b110_ethercat_optioninterface/1984417163.html)。

### EEPROM 未配置前的预期现象

旧的/空白 SII 可能使扫描身份为 `PFFFFFFFF RFFFFFFFF`、alias `0xFFFF`，probe 在 PDI 不等于 `0x80` 时返回 `LAN9252_ERR_PDI`（枚举值 7），或返回 `LAN9252_ERR_EEPROM`（10）。这时主循环仍活着，但 SSC 不会开始状态转换。probe 遇到首个错误立即返回：PDI 错误之后的 `eeprom_status`、AL status 等零值可能是**尚未读取**；诊断计数零也可能尚未采样，不能据此说 EEPROM/网络没有错误。保留先前已读字段。EEPROM 写入以后必须断电重启、重新 probe，不用旧结构作验收。

若 BYTE_TEST/READY/CHIP_ID 失败，先检查逻辑供电、CS、SPI 接线/模式、LAN9252 晶振和复位；若这三项通过而 PDI/EEPROM 失败，进入下面的 SII 准备流程。驱动故障不会调用永久 `Error_Handler()`；公共系统时钟或 HAL 外设启动错误仍可能进入原有 `Error_Handler()`，此时没有成功建立 SPI 通信，应先诊断平台启动，不能把它归为 SSC AL 错误。

## 3. TwinCAT 主站与 ESI 安装

1. 新建/打开 TwinCAT XAE 工程，记录实际 build，使用 Config/FreeRun 进行首次台架测试。选择真正连接板卡的网卡，不按电脑默认网卡猜测。TwinCAT 网卡列表入口为 `TwinCAT → Show real-time Ethernet compatible devices`，或 EtherCAT 设备 Adapter 页的 `Compatible Devices...`；只按官方支持情况安装驱动。[Beckhoff 实时网卡驱动安装](https://infosys.beckhoff.com/content/1033/ps2001-2420-1001/10831919243.html)。
2. 当前笔记本 Realtek 若位于 `for demo use only`，仅用来做发现、状态切换和通信功能实验。该分类不能保证实时能力；即使 1 ms/10 分钟通过也不能据此声明正式实时或 DC 性能。结论依据网卡驱动分类，而非把所有 Realtek 型号一概判为不支持。[Beckhoff 对 demo 分类的说明](https://infosys.beckhoff.com/content/1033/tf6105_tc3_opc_ua_pub_sub/10404117771.html)。
3. 把仓库 `EtherCAT/H743-LAN9252-LAB.xml` 复制到**本机实际 XAE 安装所用的** `Config\Io\EtherCAT` 目录。官方传统默认是 TwinCAT 3 的 `C:\TwinCAT\3.1\Config\Io\EtherCAT`（TwinCAT 2 为 `C:\TwinCAT\IO\EtherCAT`）；如果安装目录不同，以实际目录为准，不要同时往几个猜测路径复制。查看已安装描述文件/安装配置确认位置。[Beckhoff ESI 安装位置](https://infosys.beckhoff.com/content/1033/ethercatsystem/1036998411.html)。
4. 执行已在官方文档确认的 `TwinCAT → EtherCAT Devices → Reload Device Descriptions`，或关闭并重新打开 XAE/System Manager。检查没有 XML/XSD 错误，并能选择唯一的 `H743-LAN9252-LAB`、产品 `0x9252`、revision 1 描述。[Beckhoff 重载 ESI 缓存](https://infosys.beckhoff.com/content/1033/el407x/19869660683.html)。
5. 用实际版本的扫描功能扫描连接的 EtherCAT 从站；保留扫描结果。如果无有效 SII，设备尚未被识别为 LAB 是预期现象，不选择 F407/ACTI/其他厂商 ESI，也不把临时 OnlineDescription 当成正确 LAB 定义。初始周期设置 **10 ms**，从站同步方式 FreeRun/DC 关闭；找到实际版本所显示的主站/任务周期设置并确认生效，不把 UI 刷新周期误当总线周期。首次不要创建运动轴/NC 配置。

## 4. SII 备份、准备与写入审批点

电脑上的 ESI 是 XML 设备描述；板上 EEPROM 存的是带配置区、身份和类别的**二进制 SII**。复制 XML 到电脑不会修改板上 EEPROM；也不能把 XML 文本改名 `.bin` 后下载。当前仓库没有提供经实板验证的 SII 二进制。可用匹配版本的 TwinCAT EEPROM Update 从已加载的 LAB 描述生成，或用经过校验的生成工具准备 2048 字节 SII；无论哪种方式，都必须与当前固件和 ESI 一致。[Beckhoff EEPROM Update 的描述选择/写后校验](https://infosys.beckhoff.com/content/1033/el40xx/4348710539.html)、[Beckhoff 二进制 `.bin` 入口](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1446518411.html)。

### 只读备份与候选镜像核对

1. 仅连接待操作的一块板，确认扫描位置、物理端口、当前身份与照片，避免选择另一台设备。保持逻辑供电和网络稳定，功率级关闭。
2. 选择从站，打开 `EtherCAT → Advanced Settings... → ESC Access → E²PROM → Hex Editor`（界面可能拼作 E2PROM/EEPROM）。这是用户提供截图对应的 EEPROM 入口；不是 `ESC Access → Memory` 的寄存器窗口。[Beckhoff Advanced Settings 入口与 Hex Editor](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1356630411.html)。
3. 在当前版本上确认 `Upload/Read` 的方向是**设备 → 电脑**，读出整个实际 EEPROM 内容，保存带板号、时间的原始二进制备份及截图，另存散列。再次只读上传并比较；即使全 `FF` 也保存。若 UI 的方向/保存方式不明确，停止并查对应 build 帮助，不试按 Download/Write。按钮名字随版本变化，本手册不推测未验证的隐藏菜单。
4. 在电脑上准备候选 SII/EEPROM Update 选择，逐项核对以下表格；保存候选内容/摘要与 ESI/固件提交号。配置区的校验值必须由生成工具正确生成，不手改一个 PDI 字节就认为镜像有效。勿在 Hex Editor 里边试边写。

| 核对项 | 本项目值 |
|---|---|
| 名称/身份 | `H743-LAN9252-LAB`；Vendor `0x00F0C743`；Product `0x00009252`；Revision `0x00000001`；Serial `0x00000001` |
| PDI/大小 | 上电 `0x0140` 读回 `0x80`；SII 目标有效大小 2048 字节；ESI ConfigData `800E00CC8813f000000000800000` |
| SM0 / SM1 | mailbox 主站→从站 `0x1000`/128/control `0x26`；从站→主站 `0x1080`/128/control `0x22` |
| SM2 / SM3 | Rx output `0x1100`/2/control `0x64`；Tx input `0x1400`/6/control `0x20` |
| 固定映射 | `0x1C12 → 0x1600 → 0x7000:01` (16 bit)；`0x1C13 → 0x1A00 → 0x6000:01` (16 bit)、`:02` (32 bit) |
| 协议/同步 | CoE mailbox；FreeRun；不宣告 DC；无 CiA402/FoE/EoE |

Vendor `0x00F0C743` 是自定义本地实验值，**不是 ETG 分配的正式 Vendor ID**。实验身份不能发布为商用产品/一致性认证。当前 serial 1 仅用于这块单板实验；如果扩展多板，应统一制定唯一序列号并同步 SII、固件 `0x1018` 与相关测试，不能只改 EEPROM。所谓“唯一选择”同时要求已加载 ESI 中没有同身份冲突条目和操作目标确实是这块物理板。

### 必须停止的写入前确认

**EEPROM 写入前必须停下请求用户确认。** 展示：目标板/从站地址、原始备份路径和散列、拟写内容及身份、2048 字节配置、PDI `0x80`、匹配固件/ESI、写后回读与断电计划。得到明确确认之前，不按 `Download`、`Write`、`EEPROM Update` 的最终 `OK`，不做固件烧录。本手册与代码提交本身不构成写入授权。

### 得到确认后才执行的写入与验证

1. `Download/Write` 是**电脑 → EEPROM**；`Upload/Read` 是备份/回读。如果用 EEPROM Update，选择唯一的 LAB 描述或已核对的二进制，最终 OK 会写设备。确认期间若任何参数/目标改变，回到上面的审批点。不要用 Firmware Update/FoE 当成 SII 写入。
2. 写入及验证过程中保持供电与网络不间断。保存成功/校验结果；随后再次只读上传整段有效 SII，与候选二进制比较，检查身份、配置区校验和类别。工具若不暴露导出镜像，至少保存回读文件并用已核对的 SII 解析工具比较字段；只看一个“成功”弹窗不够。
3. 写完必须**彻底断开 LAN9252/EEPROM 逻辑电源并重新上电**（确认电源实际下降，避免调试器/其他线路反向供电；不要在写入中断电）。再重新扫描和 probe，PDI 为 `0x80`、EEPROM 无装载/校验错误、身份完全符合上表，才继续状态转换。[Beckhoff 要求硬件重启](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1446518411.html)。
4. 若仍然全 `FF`/PDI 错误，停下检查 EEPROM 电源、SCL/SDA、上拉、地址、写保护和配置区校验。不要重复盲写，更不要放宽固件探测条件。

## 5. 手动逐级状态检查

先使主站周期输出 ControlWord 为 0，确保实际发送循环处于运行状态。TwinCAT EtherCAT master 的 Online 页显示各 slave；在已选中的**从站行**使用 `Request 'INIT'/'PREOP'/'SAFEOP'/'OP' state`。master 页上方的 Init/Op 按钮是 master 状态请求，注意选中对象。每一级先读实际从站状态、完整 InfoData.State、AL status/code，截图后再继续。[Beckhoff Online 状态请求与作用对象](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1446518411.html)。

| 请求 | 通过条件 | 未通过时先查 |
|---|---|---|
| INIT | 身份正确，新的 probe 通过；AL 低四位 1，code 0；PC4 仍低 | SPI/probe、SII 装载、供电。网络寄存器读 WKC=1 只证明那次访问成功，不能证明 MCU 已运行 SSC。 |
| PREOP | AL 2/code 0；mailbox 可用；在线 CoE 可读 `0x1000`、`0x1008` 和 `0x1018:01..04`，身份逐项相等 | SM0/1 地址/长度/方向，ESI 与 SII/固件一致性，SSC 服务是否持续。此状态不验收周期 PDO。 |
| SAFEOP | AL 4/code 0；SM2/3、FMMU 和固定 PDO 长度 2/6 正确，输入通道可见；通信影子保持安全零 | PDO mapping/assignment、SM/FMMU 和同步设置。此应用在尚无有效 OP 输出时回环/计数为零是预期。 |
| OP | AL 8、错误位未置、code 0；已收到主站有效周期输出；WcState=0，PDO 回环与计数可观测 | 主站周期是否实际发送、首次有效输出、watchdog、网卡以及驱动 sticky fault。 |

INIT/PREOP/SAFEOP/OP 的应用通信能力参见 [Beckhoff 状态机说明](https://infosys.beckhoff.com/content/1033/tc3_io_intro/1446518411.html)。失败时保存 AL code 和诊断再处理原因、确认错误/退回 INIT 后重新逐级请求；不要靠增加主站状态转换 timeout、mailbox timeout、watchdog 或 SPI 超时隐藏问题。

### AL 错误分流（主站读到的 0x0134）

| 代码 | 官方含义；本项目排查方向 |
|---|---|
| `0x0016` | Invalid mailbox configuration；核对 128 字节 SM0/1。 [Beckhoff 0016](https://infosys.beckhoff.com/content/1033/ethercatsystem/1072502923.html) |
| `0x0017` | Invalid Sync Manager configuration；核对地址、方向和 PDO/SM 长度 2/6，重新计算主站配置。 [Beckhoff 0017](https://infosys.beckhoff.com/content/1033/ethercatsystem/1072497163.html) |
| `0x001B` | Sync manager watchdog；主站未及时发送过程数据（例如周期任务停了）。先恢复实际输出周期和网络，**不增加 WD 掩盖**。 [Beckhoff 001B](https://infosys.beckhoff.com/content/1033/ethercatsystem/1072491019.html) |
| `0x001D` / `0x001E` | Invalid Output / Input Configuration；分别核对 SM2 的 2 字节 Rx 和 SM3 的 6 字节 Tx。 [Beckhoff 001D](https://infosys.beckhoff.com/content/1033/ethercatsystem/1072487947.html)、[001E](https://infosys.beckhoff.com/content/1033/ethercatsystem/1072486411.html) |
| `0x0027` | FreeRun not supported；与本固件目标相矛盾，查选择了错误 ESI/固件或同步参数。 [Beckhoff 0027](https://infosys.beckhoff.com/content/1033/ethercatsystem/1058701707.html) |

上述是故障分类，不保证每种内部故障必定给同一 AL code；SPI 故障时 MCU 可能无法更新 AL 寄存器，主站看到的状态也可能是旧值。其他代码按 [Beckhoff AL Status Codes 索引](https://infosys.beckhoff.com/content/1033/ethercatsystem/1037010571.html) 查询，不凭显示文字猜测。

## 6. 10 ms 回环、1 ms 稳定性和拔线恢复

1. 先在 **10 ms、FreeRun、DC 关闭** 下保持 OP，主站实际周期输出按顺序给 ControlWord `0x0000 → 0x0001 → 0x0002 → 0x0003`（每个值保持至少 1 秒）。应看到 StatusWord `0 → 1 → 0 → 1`；其他位均为 0。输入 6 字节顺序是 little-endian StatusWord 两字节 + CycleCounter 四字节，输出两字节。不要只做 CoE/SDO 写 ControlWord 来代替周期 PDO；应用必须先收到有效 PDO。
2. CycleCounter 在有效 OP 且收到有效输出后，每次 **FreeRun 应用迭代**递增，不保证每个主站周期恰好加 1，主站可能一次看到跳过多个值；这是应用循环活跃证据，不能据它计算主站频率。正常溢出按 uint32 回绕；离开有效交换/出错会复位为零。
3. 保存开始/结束 CRC、Lost Frames、Tx/Rx Errors、PDI Error、lost-link、SPI timeout、AL code 和服务最大耗时。10 ms 先稳定且没有持续错误增长，才能改 **1 ms**，核对周期确实生效，再重新逐级进入 OP。
4. 1 ms 连续运行至少 **10 分钟**：持续记录 WcState（应为 0）、OP/code 0、回环和计数变化，CRC/Lost Frames/PDI/SPI timeout 无持续增长，PC4 低且没有 PWM。至少保存开始/结束数值和过程中异常时间，不能只在结束截图一次；不要在这段记录中打断点/暂停 MCU。[Beckhoff WcState 与输入冻结说明](https://infosys.beckhoff.com/content/1033/b110_ethercat_optioninterface/1984417163.html)。
5. 保持功率级关闭，记下基线，在稳定 OP 时拔掉 IN 网线数秒，确认主站 WcState/状态变化，watchdog/SSC 使应用影子安全复位（不要把主站冻结的旧输入当成从站未复位）；读 MCU 诊断可验证零影子，PC4 全程低。lost-link 增长一次属于此刻人为断线证据，随后观察是否停止增长。
6. 插回同一端口，等待链路恢复，保存状态/错误。**如果 SPI 健康**，EtherCAT 掉线/AL 回退由 SSC 和主站状态流程处理，必要时主站确认错误、退 INIT 并逐级请求；不需因正常网线掉线无条件重启 MCU。再次验收 OP、WcState=0 和回环，再保持 1 ms 至少 10 分钟，记录恢复是否需主站重扫描/状态请求。
7. **如果 `last_driver_error != LAN9252_OK`**，这是被锁存的 SPI/端口故障：服务停止 SSC MainLoop，不能只靠插回网线恢复。保存原始故障，修复供电/SPI原因后，明确 MCU reset 或在串行主循环上下文显式调用 `EtherCAT_Init()` 才会重新探测/初始化；不要从 ISR、并发调试调用或自动不断重试。正常固件只启动时 init 一次，没有后台自动清错。

验收记录至少包括两轮 1 ms 的持续时间、网卡驱动分类、所有计数基线/终值、AL 状态与 code、拔线及恢复时间、是否发生 SPI 锁存故障、回环序列、PC4 实测证据。Realtek demo 结果标记为功能实验；正式实时/DC/电机 EtherCAT 联动属于后续独立设计与验证。

## 7. 最终软件修复构建交接记录（2026-10-03）

实际 `D:/Keil_v5/UV4/UV4.exe`、target `emptytest`、ARMCC
`5.06 update 5 (build 528)` 全量 rebuild：0 errors、0 warnings。
固定 `ETHERCAT_BRINGUP_ONLY=1`；生成
`MDK-ARM/emptytest/emptytest.hex`，SHA256：
`DB629EA63516FF082BDF78BA55C2410A92C0C79BDF76CD5C7EA8A0D5700E66E9`。
此产物已包含最终 mailbox 长度、记录名称及末字节/错误缓冲所有权修复，
不是先前 `7b590b0` 的构建产物；默认 128、最小 34 字节 mailbox 保持不变。
烧录前应再次核对该文件散列与对应源码提交；构建记录不构成烧录授权。

主机 EtherCAT 套件 8 项全部通过（0 failed、0 pending，7879 个 native
检查/行为用例）；电机回归套件 8 项通过。map 确认 core/port/app/service
和 SSC 实际链接，`MotorControl_Start/Init` 已移除，没有动态分配调用。
RO 42032 字节、RW+ZI 9496 字节（含 1024 字节栈和原有未使用的 512
字节 heap 保留区）；24 个静态 mailbox block 共 3072 字节。
链接器栈估计 584 字节含未解析函数指针；实际 transport callback 最深
184 字节，保守相加 768 字节，另预留约 104 字节 FP 异常帧及对齐。
这些是静态分析估计，运行时栈水位仍需实板验证。

可复查完整编译错误 RED/GREEN、警告归零、存储对齐/访问指令、map、
USER CODE 与全局 diff 记录：
`.superpowers/sdd/2026-10-01-h743-lan9252-first-op/task-9-report.md`
；最终修复 RED/GREEN、最新构建日志/散列及对应提交记录：
`.superpowers/sdd/2026-10-01-h743-lan9252-first-op/software-final-fix-report.md`
（本机验证记录）。当前项目/选项的用户编辑已逐字节保留。本次未执行 CubeMX 重生成、TwinCAT ESI 导入、EEPROM
写入、烧录或实板 OP/稳定性测试；后续按本手册逐项验收。
