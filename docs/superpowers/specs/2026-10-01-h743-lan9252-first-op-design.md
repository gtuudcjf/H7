# H743 + LAN9252 首次 EtherCAT OP 设计

## 目标与范围

本阶段在现有 STM32H743 电机控制工程中增加一个最小、可诊断、可替换的 EtherCAT 从站子系统，使当前自研板通过板载 LAN9252 和 SPI1 完成 `INIT -> PREOP -> SAFEOP -> OP`，并用最小周期 PDO 回环证明数据交换真实有效。

第一阶段只验证 EtherCAT 通信，不把 EtherCAT 输出转换为电流、转速或位置命令。测试固件必须保持 DRV8323 功率级关闭，不启动 TIM8 PWM，不进入电机校准或闭环运行。现有电机控制算法、参数、编码器链路和 VOFA 电机观测逻辑不在本阶段修改范围内。

本阶段的最终验收是：TwinCAT 能以预期身份识别设备；从站依次进入 OP；主站写入 16 位控制字，从站返回 16 位状态字与 32 位递增计数器；在关闭 Distributed Clocks 的轮询模式下稳定运行；通信错误不会导致固件永久阻塞或意外启动电机。

## 已确认的硬件和现状

当前板卡使用 LAN9252、外部串行 EEPROM、25 MHz 晶振和两个 EtherCAT 网络端口。首次扫描已经确认 TwinCAT 能通过笔记本 Realtek 网卡发现 LAN9252，ESC Memory 读取的 Working Counter 为 1，因此网络物理层、LAN9252 EtherCAT 数据链路和主站基础配置已经能够工作。

板载 EEPROM 没有加载有效 SII，TwinCAT 显示 Product Code 和 Revision 为 `0xFFFFFFFF`，Configured Station Alias 为 `0xFFFF`，设备类型和 ESI 为空。PDI Control 也没有被配置成 LAN9252 SPI 所需的 `0x80`。因此当前无法依靠 H743 处理 EtherCAT AL 状态转换。

H743 工程目前只初始化 SPI2、SPI3、SPI4：SPI2用于驱动器，SPI3和SPI4用于编码器。LAN9252 使用独立 SPI1：

| 信号 | H743 引脚 | 第一阶段用途 |
|---|---|---|
| LAN9252 CS | PA4 | GPIO推挽输出，默认高 |
| SPI1 SCK | PA5 | SPI1 AF5 |
| SPI1 MISO | PA6 | SPI1 AF5 |
| SPI1 MOSI | PA7 | SPI1 AF5 |
| SYNC0 | PE0 | 普通输入，不启用中断 |
| SYNC1 | PE1 | 普通输入，不启用中断 |
| IRQ | PE3 | 普通输入，不启用中断 |

PE2继续用于现有SPI4时钟，与EtherCAT IRQ不存在冲突。

## 实验设备身份

由于当前没有ETG分配的Vendor ID，第一次OP验证使用明确标注为仅限本地台架的实验身份：

| 字段 | 值 |
|---|---|
| Device Name | `H743-LAN9252-LAB` |
| Vendor ID | `0x00F0C743` |
| Product Code | `0x00009252` |
| Revision | `0x00000001` |
| Serial Number | `0x00000001` |

该Vendor ID不是ETG正式分配值，不得用于产品发布、外部设备目录、合规声明或一致性测试。获得正式Vendor ID后，必须同时替换ESI、SII、SSC常量和对象字典`0x1018`，并提升Revision。

Identity Object `0x1018`采用相同的32位值。XML默认数据按小端字节表示：

| 子索引 | 含义 | 默认数据 |
|---|---|---|
| `0x1018:01` | Vendor ID | `43C7F000` |
| `0x1018:02` | Product Code | `52920000` |
| `0x1018:03` | Revision | `01000000` |
| `0x1018:04` | Serial Number | `01000000` |

## ESI、SII与SSC的一致性

TwinCAT电脑上的ESI XML、写入外部EEPROM的SII和H743固件内的SSC对象字典必须从同一套配置生成或逐项校验。以下字段禁止各自独立修改：

- Vendor ID、Product Code、Revision、Serial Number；
- Device Name和Group Type；
- Mailbox起始地址与长度；
- SyncManager起始地址、长度、方向和控制字节；
- PDO对象、映射对象、位宽和顺序；
- CoE对象字典，尤其`0x1018`、`0x1600`、`0x1A00`、`0x1C12`和`0x1C13`；
- LAN9252配置区及其校验值。

现有`407 LAN9252 Ethercat spi/LAN9252-EVB-SPIXML.xml`只作为模板。保留其LAN9252 SPI配置、Mailbox和Process RAM布局；不沿用Vendor `0x9/ACTI`、示例板名称、LED/开关/模拟量语义和DC声明。原文件不覆盖，新文件使用独立名称。

## 最小ESI/SII设计

EEPROM配置区沿用参考XML中经过LAN9252 SPI示例使用的结构，PDI Control必须在上电加载后得到`0x80`。SII的目标有效大小为2048字节；物理EEPROM更大不要求填满。

Mailbox和SyncManager固定为：

| SyncManager | 起始地址 | 长度 | 方向/用途 |
|---|---:|---:|---|
| SM0 | `0x1000` | 128字节 | Mailbox Out，主站到从站 |
| SM1 | `0x1080` | 128字节 | Mailbox In，从站到主站 |
| SM2 | `0x1100` | 2字节 | RxPDO，主站到从站 |
| SM3 | `0x1400` | 6字节 | TxPDO，从站到主站 |

启用CoE和Mailbox，第一阶段关闭FoE、EoE、SoE、AoE、CiA402和EEPROM Emulation。ESI不声明可选DC工作模式，SSC关闭DC、AL Event中断、EtherCAT定时器中断和通用中断支持，以FreeRun轮询方式启动。

## 最小对象字典与PDO

RxPDO使用2字节控制字：

| 对象 | 类型 | PDO映射 |
|---|---|---|
| `0x7000:01 ControlWord` | `UINT16` | RxPDO `0x1600` |

TxPDO使用6字节：

| 对象 | 类型 | PDO映射 |
|---|---|---|
| `0x6000:01 StatusWord` | `UINT16` | TxPDO `0x1A00` |
| `0x6000:02 CycleCounter` | `UINT32` | TxPDO `0x1A00` |

`ControlWord`只用于通信回环，不触发电机。`StatusWord`的位0镜像`ControlWord`的位0，其余位保留为0；`CycleCounter`在每次有效应用周期递增。这样主站既能验证输出送达，又能确认从站输入持续刷新。

SM分配对象固定为：

- `0x1C12`分配`0x1600`到SM2；
- `0x1C13`分配`0x1A00`到SM3。

## 软件架构

EtherCAT实现分成四层，SSC生成代码不与STM32 HAL或电机模块相互包含：

```text
TwinCAT / EtherCAT network
            |
        LAN9252 ESC
            |
   lan9252_spi (SPI/CSR/PRAM)
            |
   lan9252_port (SSC HW_*适配)
            |
      SSC状态机与CoE
            |
   ethercat_app / ethercat_service
```

建议文件边界：

```text
Core/Inc/ethercat/lan9252_regs.h
Core/Inc/ethercat/lan9252_spi.h
Core/Inc/ethercat/lan9252_port.h
Core/Inc/ethercat/ethercat_app.h
Core/Inc/ethercat/ethercat_service.h

Core/Src/ethercat/lan9252_spi.c
Core/Src/ethercat/lan9252_port.c
Core/Src/ethercat/ethercat_app.c
Core/Src/ethercat/ethercat_service.c

Middlewares/Third_Party/EtherCAT_SSC/
EtherCAT/H743-LAN9252-LAB.xml
```

### LAN9252 SPI传输层

SPI1第一阶段使用Master、全双工、8位、MSB First、CPOL Low、CPHA 1 Edge、软件NSS、无DMA、无SPI中断。当前SPI123内核时钟为64 MHz，预分频16，得到4 MHz启动频率。

传输层实现：

- `0x0B` FAST_READ和`0x02` WRITE直接寄存器访问；
- 通过`ECAT_CSR_DATA/ECAT_CSR_CMD`访问`0x0000..0x0FFF`的EtherCAT Core寄存器；
- 通过PRAM读写命令和FIFO访问`0x1000`以上Process RAM；
- 显式的小端打包、非对齐安全访问和长度检查；
- 每个Busy轮询和HAL传输都有有限超时；
- 不使用动态内存，不调用全局`__disable_irq()`，不在错误路径永久循环。

初始化必须依次验证：

1. `BYTE_TEST(0x64) == 0x87654321`；
2. `HW_CFG.READY == 1`；
3. `ID_REV`的Chip ID为`0x9252`；
4. EtherCAT Core `PDI Control(0x0140) == 0x80`；
5. EEPROM状态没有装载或校验错误；
6. CSR访问能够正确读回ESC Type、Revision、Build和AL Status。

### SSC硬件适配层

`lan9252_port`实现SSC要求的`HW_Init`、`HW_Release`、`HW_GetALEventRegister`、`HW_EscRead`、`HW_EscWrite`和SyncManager接口。第一阶段所有访问仅发生在主循环上下文，ISR版本不得由中断调用，因此CSR命令寄存器不存在主循环/中断并发访问。

参考F407项目中的CSR和PRAM算法，但不复制StdPeriph初始化、`SPI1->DR`、`GPIO_DeInit(GPIOA)`、F407 GPIO寄存器、无超时循环或全局关中断宏。参考工程的SYNC中断向量也不作为H743实现依据。

### EtherCAT服务层

对`main.c`只暴露：

```c
EtherCATStatus EtherCAT_Init(void);
void EtherCAT_Service(void);
const EtherCATDiagnostics *EtherCAT_GetDiagnostics(void);
```

`EtherCAT_Init`完成LAN9252自检、SSC `HW_Init`和`MainInit`。失败时保持功率级关闭并进入可观测的通信故障状态，不调用会隐藏原因的永久`Error_Handler`循环。

`EtherCAT_Service`在主循环中尽可能频繁调用SSC `MainLoop`，并用`HAL_GetTick()`补偿调用SSC所需的1 ms时基。若主循环跨过多个毫秒，只补齐协议计时，不在单次服务中无限追赶。首版不使用TIM8、TIM6或LAN9252外部中断作为SSC时基。

### 应用映射层

`APPL_OutputMapping`只把RxPDO复制到通信影子变量；`APPL_InputMapping`只生成状态字和计数器。映射函数中禁止：

- 调用电机启动或目标设置接口；
- 直接写TIM8 CCR或DRV8323；
- Flash擦写；
- UART/USB打印；
- 阻塞等待或动态内存分配。

## 与现有电机工程的隔离

增加编译期配置`ETHERCAT_BRINGUP_ONLY`，首次OP固件固定启用。启用时：

- 不调用`MotorControl_Start()`；
- DRV8323使能保持安全关闭；
- 不启动TIM8 PWM、ADC电流闭环和电机校准；
- 主循环运行EtherCAT服务和低频诊断；
- 不接收或执行任何电机运动命令。

EtherCAT初始化失败、掉线、PDO看门狗异常或AL状态回退均不能解除上述安全状态。将来接入电机控制需要独立的新设计和安全状态机，不属于本阶段。

## 错误处理与必要观测值

底层错误使用明确枚举返回，至少区分：SPI HAL错误、直接寄存器超时、BYTE_TEST不匹配、READY超时、Chip ID错误、PDI不是SPI、CSR超时、PRAM超时、EEPROM状态错误和SSC初始化失败。

保留以下必要观测值，供Keil Watch和低频VOFA遥测使用：

- `byte_test`、`id_rev`、`hw_cfg`、`pdi_control`、`eeprom_status`；
- `al_status`、`al_status_code`、`al_event`；
- `pdi_error_count`、`lost_link_count`；
- `last_driver_error`、`spi_timeout_count`；
- `ethercat_service_max_us`；
- `rx_control_word`、`tx_status_word`、`tx_cycle_counter`。

诊断输出限制为10至20 Hz，不在每次PDO或SPI事务中发送。诊断通道失败不改变EtherCAT状态，也不影响功率级安全状态。

## EEPROM烧写流程

首次写入使用与固件匹配的`H743-LAN9252-LAB.xml`。写入前关闭电机功率级并保存空白EEPROM的原始读取结果。TwinCAT安装ESI后重新启动XAE，在Config Mode中扫描LAN9252，选择匹配的实验设备执行EEPROM Update。写入和校验过程中不得断电或拔网线。

写入完成后必须彻底断电再上电。重新扫描后必须满足：

- 设备显示`H743-LAN9252-LAB`，不再显示`PFFFFFFFF RFFFFFFFF`；
- Vendor ID、Product Code和Revision与设计一致；
- `PDI Control(0x0140)`为`0x80`；
- EEPROM状态无加载/校验错误。

如果写入失败或重新上电仍为全`0xFF`，停止固件调试，优先检查EEPROM供电、SCL/SDA、上拉、电气地址和写保护。

## 状态转换与调试顺序

状态转换逐级验证，不直接请求OP：

1. INIT：LAN9252可扫描，身份正确，H743 PDI自检通过；
2. INIT到PREOP：SSC处理AL请求，SM0/SM1 Mailbox配置成功，CoE对象可读；
3. PREOP到SAFEOP：SM2/SM3、FMMU和PDO映射与ESI一致；
4. SAFEOP到OP：主站发送有效输出，过程数据看门狗和首次输出条件满足；
5. OP：控制字、状态字和计数器连续交换。

每次失败优先记录TwinCAT AL Status Code、H743诊断结构和ESC PDI错误计数，不通过延长状态转换超时掩盖问题。

## 测试与验收

### 主机测试

LAN9252协议核心应通过不依赖HAL的模拟传输测试，覆盖：

- FAST_READ/WRITE命令、地址、dummy字节和数据字节序；
- CSR命令字构造与Busy超时；
- PRAM非4字节对齐、短包、跨DWORD和FIFO处理；
- 所有错误路径释放CS并返回确定错误码；
- PDO结构大小固定为2字节和6字节；
- StatusWord镜像与CycleCounter递增逻辑。

工程集成检查应确认SPI1引脚、4 MHz初始速率、PA4默认高、IRQ/SYNC无EXTI、TIM8优先级和现有SPI2/3/4未被改变。

### 实机验收

1. 写入SII并重新上电后身份与`0x0140`正确；
2. H743启动诊断依次通过BYTE_TEST、READY、ID_REV、PDI和CSR读回；
3. TwinCAT手动状态转换依次到OP，AL Status Code为无错误；
4. FreeRun或10 ms周期下控制字回环和计数器连续工作；
5. 再降至1 ms运行至少10分钟，`WcState == 0`，CRC、Lost Frames、PDI Error和驱动超时不持续增长；
6. 拔网线后从站输出影子变量进入安全值，功率级仍关闭；
7. 重新连接、复位并扫描后能够再次进入OP；
8. 整个过程没有启动PWM或驱动电机。

笔记本Realtek网卡的`for demo use only`模式可以完成首次功能验收，但1 ms结果只作为实验记录，不作为正式实时性能或DC一致性结论。

## 明确推迟的功能

以下内容不进入本阶段：

- Distributed Clocks、SYNC0、SYNC1；
- LAN9252 IRQ/AL Event中断；
- EtherCAT PDO直接控制电机；
- CiA402状态机；
- FoE固件升级、EoE、SoE、AoE；
- 正式ETG Vendor ID和一致性认证；
- 1 ms电机闭环与EtherCAT同步性能优化。

这些功能只有在最小PDO轮询模式稳定进入OP后才分别设计和验证。
