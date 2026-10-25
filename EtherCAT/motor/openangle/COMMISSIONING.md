# 模式 2 联动：集中验证清单

> 2026-10-23最新决定：用户要求暂停采样异常硬件排查，恢复旧过流阈值并优先手动正反转。
> 当前有效关断为任一换算相电流>10A连续2次（原电机保护保留）；0.6A仅保留历史统计，
> 不再触发实验停机。10A是旧软件配置，不是已核定的电机安全额定值，采样异常尚未解决。
> 不再把下文历史B4～B7的重复无电机录屏/测量作为下一步任务；当前操作见C。
> 最新用户批准对照旧模式2的设定点：对齐Id=0.5A、运行Iq=0.8A，启动前需求预置±6rpm（±1Hz）。
> 已拆分对齐/运行参数，保留用户现有1.0A参考限幅；不是再次提高过流门槛，也不是完整恢复旧固件。
> 用户报告6/12rpm均可转后，最新仅将本模式运动上限延至6秒、总会话延至10秒；
> 控制策略与独立看门狗共享配置，心跳100ms及其他关断保护不变。噪声/匀速性尚未验收。

常规入口仍为功率锁0的DryRun。2026-10-21单独批准准备BenchCheck离线构建，
不包含烧录、启动或接通动力的批准。不要在原DryRun工程手改POWER_ENABLED。
原电机/编码器在本板上尚未验证；0.1 A 等参数不保证起转，也不代表硬件保护已验证。

## 文件与回退

- 新 Keil 工程：`MDK-ARM/ethercat_openangle.uvprojx`，目标 `ethercat_openangle_dryrun`。
- 原 Keil 工程：`MDK-ARM/ethercat_motor.uvprojx`，目标 `ethercat_motor_dryrun`，仍为编码器速度版。
- 新 PLC：本目录 `plc/MAIN.TcPOU`，或 `plc/EtherCATMotorOpenAngleDemo.tpzip`。
- 原 PLC：`EtherCAT/motor/plc`，未替换。
- 修改前源码备份：`tmp/openangle_baseline_20261020_150331`，166文件哈希核对通过。
- 旧 ESI/SII、EEPROM备份和用户活动 TwinCAT 工程不由本次程序更新覆盖。

## A. 复用现有 TwinCAT 工程，只做新版 DryRun

1. 保存当前 TwinCAT 工程并另存完整备份；停止在线 PLC、退出登录后再修改程序。
   不覆盖原唯一副本。保持原网卡、revision2从站、13个变量链接和10ms任务。
2. 编译新 Keil 工程，核对宏 OPEN_ANGLE=1、POWER_ENABLED=0、目标名称和新输出目录。
   下载仍是操作者单独执行的步骤；当前未自动下载。不要误选旧LocalMotor自动启动目标。
3. 保持逻辑电源/网线，动力输出隔离；操作者确认PC4低、TIM8 MOE低。
4. 在工程副本中，替换 MAIN 的完整声明和 ST 实现为 `plc/MAIN.TcPOU` 对应内容。
   `GVL_MotorIO` 的13个映射变量不删、不重命名、不重新建立；其注释可参照新GVL更新。
   或导入新tpzip作为独立PLC项目，但那条路线需核对/建立对应链接，不是首选。
5. 编译，确认新声明中默认 bStart/bReset/bFreezeHeartbeat=FALSE、rDemandRpm=0。
   由操作者按现有流程激活配置、登录和运行PLC。PLC Start不是MAIN.bStart。
6. 检查 bOpenAngleProfile=TRUE、bElectricalOutputOff=TRUE、bSpeedMeasurementValid=FALSE、
   State=8、WcState=FALSE、两序号推进，ControlWord=256、TargetSpeed=0。
   Ready可以不依赖编码器，但不代表已经校准ADC。
7. 若当前有初始化/通信故障先记录原因，按匹配profile的bReset上升沿复位，随后写回FALSE。
8. 仅在确定功率锁定的情况下请求一次零目标bStart，预期 BridgeFault=9，
   PC4/MOE保持低；这证明请求到达功率锁，**不证明实际启动路径通过**。
   释放bStart后复位，不能通过修改Ready/校准标志绕过检查。
9. 心跳冻结、不断电拔线、SAFEOP/OP各做一次集中回归；只记录关键前后状态，
   不重做EEPROM、网卡、设备扫描和PDO布局。确认恢复通信不自启、显式复位后恢复基线。

## B. 功率路径与实时保护——准备已批准，实机尚未验收

### B0. 最小改动与构建选择

不复制控制源码，不另存整套工程，不重复整目录备份；沿用既有备份和本文件。
仅在原build脚本增加显式实验模式，复用同一份模式2工程模板和PLC：

```powershell
powershell -File tools/build_ethercat_motor.ps1 -Mode OpenAngleBenchCheck
```

- 生成工程：`MDK-ARM/motor_build_openanglebenchcheck.uvprojx`。
- 目标：`motor_build_openanglebenchcheck`。
- 输出：`MDK-ARM/motor_speed/OpenAngleBenchCheck/motor_build_openanglebenchcheck.axf`。
- 宏：OPEN_ANGLE=1、MOTOR_CONTROL=1、POWER_ENABLED=1、BRINGUP_ONLY=0。
- 禁止自动调试下载、无构建后用户命令、不生成HEX；脚本只执行Keil编译。
- **AXF本身仍可被手工下载且具有功率许可。无HEX不是安全隔离，不要现在点击Download/Debug。**
- 原 `ethercat_openangle.uvprojx` 和 `OpenAngleDryRun` 仍是POWER_ENABLED=0；旧速度
  `PowerPathCheck`不是本模式的实验入口，不能混用。

当前不增加调试旁路：对齐给定0.5A、运行给定0.8A，参考矢量上限1.0A，
电压0.15pu，三相任一相>10A连续2有效样本按原保护关断，需求±20等效rpm，斜坡20等效rpm/s，
心跳100ms、许可10s/首次非零6s，正常停止上限2s。实际电流仍需独立测量，不能由参考上限推断。

### B1. 下载前门槛（当前先完成这些确认，不启动）

1. 核对实际电源/母线额定值、逻辑与动力供电关系、DRV使能接线、限流和独立断电手段。
   不根据“电机未接”推断功率级已隔离；不能分离供电时先停下来核对硬件方案。
2. 明确PC4、实际栅极驱动使能、低压PWM信号的测点和安全参考地；TIM8 MOE是寄存器位，
   必须结合实际输出观察，不能当成一个独立的外部测点。测量接线未确认前不接动力节点探头。
3. 复核ADC通道、分流电阻/增益、零偏、量程/极性、实际极对数；本板原电机代码未实机验证。
4. 保持现有PLC、PDO链接和10ms任务；bStart/bReset/bFreezeHeartbeat=FALSE，rDemandRpm=0。
5. 核对所选AXF/宏后，另行批准具体下载和实验接线。仅同意离线编译不等于允许本步骤。

### B2. 无动力输出条件下的控制路径检查（B1满足并明确批准后）

1. 操作者下载并复位后，先核对新版profile、OP、零命令、PC4/MOE关闭与未自动启动。
   如果这一步失败，不发送Start。
2. 仅在已确认电机不可能获得动力的接线下请求一次零目标Start；此时不再以故障9为预期，
   因为实验构建已允许功率路径。记录实际driver/ADC初始化结果，不预设一定成功。
3. 观察 `g_motor_control_debug` 的 run_state/fault/current_sense_ready、phase_a/b_raw、
   phase_a/b_offset、id/iq及参考值，记录从零偏校准到ALIGNING的过程。只观察，不写内部状态。
4. 原代码ADC校准失败必须停机，不改成降级开环；无电机的空载电流闭环/限幅不能当作对齐通过。
5. 零命令在对齐后最多等约500ms就自动撤电流停机，返回OutputOff；这可能快于画面刷新。
   不要反复点击Start，也不要延长窗口以方便观察。需要采集时使用合适的连续记录方式。

### B3. 保护验收与首个电流实验

- 新鲜度/失联：记录最后有效命令与实际撤销信号波形，验证100ms心跳保护及<=110ms验收目标。
  程序停止输出或画面显示故障，不等同于测得电气关断延迟。
- ADC：验证零偏与有效采样后才考虑小电流。对齐期间采样失效也应关断；不能用
  暂停整个MCU的断点来模拟“仅ADC停止”，因为TIM6等保护也可能被一起暂停。
- 3s/5s：零目标自动停机可能先发生，正常运动也受3s限制，故“等待5s没有故障”不算验证。
  5s独立上限、前台延迟恢复不能重开输出的实测需要可控故障注入方案；本次只准备生产构建，
  没有加入这种注入旁路，不能把已有宿主测试写成实机通过。
- 接动力/接电机必须另行确认：固定空载台架、限流、独立断电、电流采样/相序/极对数。
  固定磁场对齐也可能移动转子；先做短时小电流对齐，再考虑C阶段。
- 出现采样无效、异常电流、持续限幅不转、噪声/发热/振荡或保护失效立即停止并断开动力；
  不自动加电流、电压、时限或绕过故障。普通稳定性警告本轮不展开，但不能将失联/无效反馈绕过。

每项记录“操作、接线/功率状态、故障码、实际波形、是否通过”；没做的留待验证。

### B4. 2026-10-21：无电机启动的 RAM 诊断记录

本次只增加记录，不改变控制参数、保护阈值、PLC、PDO或任务周期。新增
`g_motor_openangle_diagnostics`，只在 OPEN_ANGLE=1 构建中存在，占216字节RAM。
仍使用原 BenchCheck 工程及同名AXF，不另建工程版本；调试符号必须来自本次AXF，
不能继续使用旧AXF的符号或硬编码RAM地址。

板子断电后记录会丢失，**不能断电后再读**。每次被接受的Start清空上一轮记录并增加
`session`；Stop及故障复位不清空它。`session>0` 且 `active=0` 表示本轮记录已冻结。
运行时Watch各字段不是原子快照，不用断点暂停正在运行的电机来抓取。

下次仍保持电机三相断开、24V/1A，不改限流、不进行抱闸或转动实验。保持调试器连续运行
和周期刷新，在确认OP、零命令、故障已复位后只做一次原定零需求启动。停机后不要再次Start，
在断电前截图整个结构体，优先保留：

| 字段 | 含义 |
| --- | --- |
| `session / active / calibrated / sample_count` | 本次编号、是否仍在记录、零偏流程成功、有效换算样本数（不含校准样本） |
| `offset_a / offset_b` | 本轮校准的ADC零偏 |
| `peak_abs_phase_a` | 三相电流绝对值的最大值，单位A；不是仅A相 |
| `over_limit_total / max_consecutive_over_limit` | 超过0.6A的总样本数／最大连续次数 |
| `consecutive_over_limit` | 最后有效样本结束时的连续超限次数，不在Stop时清零 |
| `first / first_over_limit / peak / last` | 首个、首次超限、最大值所在、最后的有效换算样本；均含同次转换的raw_a/b、Ia/Ib/Ic、Id/Iq、run_state、at_ms |
| `stop_reason / motor_fault / policy_fault` | 第一次终止分类、当时电机故障、当时本地实验策略故障 |
| `stop_run_state / stop_raw_a / stop_raw_b / duration_ms` | 终止前阶段、终止时最新原始值、从Start到记录终止的毫秒数 |

`stop_reason`：0=尚未终止，1=外部Stop API，2=本地实验正常自动结束，3=本地实验策略故障，
4=电机控制故障。分类1不能区分PLC停止、通信保护或外部清理原因，应结合TwinCAT故障码；
也不表示所有HAL清理必然成功。`run_state`：3=ALIGNING，5=RUNNING。
`at_ms` 是系统毫秒时间戳，与 `started_ms` 的无符号差为相对启动时间。

原始值可能在停止换算后仍更新，因此 `stop_raw_a/b` 不保证与 `last` 同次转换。
比较电流必须使用 `first/peak/last` 内部配对数据。`calibrated=1` 仅表示通过现有校准判定，
不等于真实零偏、增益与相电流已验收。

诊断不会触发或放宽保护。过流触发样本在原安全撤销之后才保存；Stop/清故障不能覆盖
第一次终止记录。记录不能替代示波器测量电气关断延迟，新增执行开销尚未实测。

### B5. 2026-10-21：DRV8323 启动寄存器读回

新增 Watch：`g_drv8323_startup_diagnostics`。仍使用原 BenchCheck 工程和同名AXF，
必须重新下载对应固件并使用本次符号，不能沿用旧RAM地址。
先进入Debug并Run，确认uwTick更新，再恢复TwinCAT OP和处理启动前故障锁存。
电机三相仍断开、24V/1A、零需求，只启动一次；原B4电流记录也要保留。

该快照只在 OPEN_ANGLE=1 的前台驱动配置调用中生成：原配置写入及清故障之后，
CAL校准、ADC采样和三相PWM之前。**不是运行中/故障发生时的寄存器记录**，
不能用启动时的零故障值否定之后发生的硬件故障。Stop不再访问已休眠芯片，也不重开ENABLE。

停机后确认 `session>0`、`active=0`，展开 `reg` 和 `read_status` 并截图：

| 索引/地址 | 内容 | 本工程期望/解释 |
| --- | --- | --- |
| `reg[0] / 0x00` | Fault Status 1 | 保留原始故障位，通常无故障为0 |
| `reg[1] / 0x01` | Fault Status 2 | 保留原始故障位，通常无故障为0 |
| `reg[2] / 0x02` | Driver Control | 6x PWM；写入0x0001的CLR_FLT位会自清，不能要求读回仍为1 |
| `reg[3] / 0x03` | Gate Drive HS | 原程序没有写该寄存器，记录实际值 |
| `reg[4] / 0x04` | Gate Drive LS | 原程序没有写该寄存器，记录实际值 |
| `reg[5] / 0x05` | OCP Control | 配置写入值0x031F |
| `reg[6] / 0x06` | CSA Control | 配置写入值0x0203：VREF/2、5V/V、非CSA_FET、校准位0 |

- `configure_status`：原配置过程的HAL结果，0=OK；不是芯片配置一致性判定。
- `read_status[i]`：0=OK、1=ERROR、2=BUSY、3=TIMEOUT，0xFFFFFFFF=未尝试。
- `attempted_mask`：尝试过读取的地址位；`valid_mask`：HAL成功返回的地址位，全部成功为0x7F。
  它只表示传输完成，不证明SPI电气通信正确；全0/全0x7FF或不符预期仍需分析。
  未成功读取的 `reg[i]` 保留0xFFFF，不能当作真实寄存器值。
- `permission_lost=1`：读取期间原独立安全许可被撤销；函数保持驱动关闭并返回错误。
- 读取首个失败即停止，不重试、不改寄存器、不放宽保护；读回失败/数值不符本身不新增控制判定。
  如果读回不完整，先保留截图，不重复启动或以此认为硬件已通过。
- 下一次板级配置调用会覆盖记录并增加session；未走到该调用的启动失败不会产生新快照。
  断电会丢失记录；不要在Watch中调用读寄存器函数或写内部诊断变量。

本次诊断不改变增益、阈值、PWM、PLC/PDO；新增前台SPI开销仍需实机观察。

### B6. 2026-10-22：一次 Start 完成 PWM 关闭/开启采样对照

目标仍是尽快验证电机正反转，本轮只安排一次无电机零需求对照，不重复旧测试、
不增加工程版本或整目录备份。沿用同一个 BenchCheck 工程及同名AXF，重新下载后
必须使用新符号；没有自动下载、调试器/PLC操作或硬件验收。

一次被接受的Start按顺序执行：原DRV配置/读回及CAL零偏校准 → CAL拉低、保持
TIM8 CH4原ADC触发但不使能六路相PWM，记录约100ms → 原对齐/零需求自动停止。
原采样时刻、增益、0.6A连续3样本保护、采样范围/数学/超时保护、100ms通信保护、
5s会话和PLC启动超时均不放宽。等待段也使用真实电流换算和保护，异常会终止本轮，
不会为“收满100ms”或“继续对照”绕过保护。100ms是采样观察时长，不是电机控制参数。

新增Watch `g_motor_openangle_baseline_diagnostics`，结构与原电流记录相同，占216字节。
正常情况下两份记录session相同、active均为0：baseline是CAL低且三相PWM未使能时的
约100ms采样；原 `g_motor_openangle_diagnostics` 只计后续开启PWM的换算样本，但其
duration_ms仍从整个Start算起，包含新增100ms。baseline的run_state=2表示READY，
后续原记录仍为3/5；两份记录不混合样本，Stop/清故障不覆盖已冻结结果。
保护的连续超限计数不在两段切换时清零；若跨边界凑齐3个样本会照常停机，
因此分别查看两段的最大连续次数不一定单独能看到3。
baseline只在成功校准后开始，若新启动在校准前失败，旧baseline可能仍留在RAM，
必须核对session，不能把旧记录与新Start比较。

操作仍为：电机U/V/W断开、24V/1A → 下载同名AXF → Debug/Run并确认uwTick增长 →
TwinCAT OP、零命令并处理启动前故障锁存 → 录屏，只发一次零需求Start → 自动停止后
释放bStart，不复位、不重复启动 → 展开两份记录截图并保存，再断电。
此次先保留两份记录的session/active/calibrated/sample_count、offset_a/b、
peak_abs_phase_a、over_limit_total/max_consecutive_over_limit、stop_reason/motor_fault/
policy_fault；有超限时再展开各自peak、first_over_limit。既有DRV启动快照可一并保留，
不用重复录制寄存器每一项。只比较超限比例和幅度，不能直接比较100ms/约1.35s的总次数。

等待段sampling_started仍为真，CH4可使MOE为高；不要把MOE低或bElectricalOutputOff为真
当作此段预期。代码逐样本检查CH1/1N..CH3/3N使能位未开，触发通道CH4保持原配置；
这不等于独立测得功率级实际波形或实现电气隔离。

判断只分两类：无相PWM时已明显异常，优先检查正常采样零点/模拟链/ADC；无相PWM时
稳定而开启后异常，优先检查开关相关采样窗口/耦合。差异只提供定位方向，不自动证明根因。
本轮结果用于决定下一项必要修正，不扩展为反复跑同样录屏；不能以无故障直接批准接电机。
进入正反转前仍需确认采样可信、台架固定和抱闸按电机要求释放，方向切换必须先停止。

### B7. 2026-10-23：仅 BenchCheck 的 ADC 分频减半对照

20261022_152457.mp4 的同一 session=1 中，无相PWM基线100ms/998样本，
158次超过0.6A（15.83%）、峰值1.79533815A；后段13501样本/1731次（12.82%）、
峰值2.06933069A，两段最长连续超限均为2、故障均为0。基线第1ms已超限，
第39ms出现峰值，末样本仍超限；现有证据不支持仅追查三相PWM开启瞬间。
这些数值为ADC换算及重构，不是独立测得的相电流；不能据此认定真实过流。

本轮只有一个实验变量：当 OPEN_ANGLE=1 且 POWER_ENABLED=1 时，ADC1 的
ClockPrescaler 从 DIV1 改为 DIV2。非功率OpenAngle DryRun、旧编码器/本地模式均保留DIV1。
原TIM8 10kHz触发、32.5周期采样设置、双通道顺序、JEOS、PI、DRV增益、100ms基线、
0.6A连续3样本保护及全部时限均不变；物理采集/转换时间随ADC时钟变化，HAL的BOOST配置
也按原逻辑随时钟自动选择。目的是检查ADC时钟/采集建立时间敏感性，不是证明原配置超频，
也不是已验证修复。根据源码与HAL，PLL2P核时钟为50MHz（HSE=25MHz假设），实际ADC时钟
还受芯片修订的内部除2影响，不能直接把DIV1解释为实际50MHz。

保持U/V/W断开、24V/1A，仍用同一BenchCheck工程和同名AXF，需手动重新下载。
先Debug/Run、确认uwTick持续增长，Watch增加 hadc1.Init.ClockPrescaler，当前HAL应为
0x00040000（DIV2，十进制262144），EOCSelection仍为8。只读，不在Watch改运行中ADC寄存器。
随后恢复OP，要求State=8、WcState=FALSE，零需求、bStart=FALSE；启动前锁存故障仅按原
bReset边沿复位一次，恢复bReset=FALSE，确认nPhase=0、MotorFault/BridgeFault=0。
不能恢复则停止，不强写nPhase/状态/映射输出，不反复复位。

只发一次零需求Start，等原流程自动停止，再释放bStart。若出现故障同样保留结果，不再
Reset/Start。两份冻结记录均active=0且session匹配后保存：session、sample_count、
peak_abs_phase_a、over_limit_total、max_consecutive_over_limit、stop_reason、motor_fault、
policy_fault；有超限再展开peak/first_over_limit。无需重录DRV寄存器展开全过程。
如电源持续限流、明显压降/反复重启、异常发热或气味，应直接断开24V，不继续采样。

与上述DIV1结果比较超限比例、峰值和结束原因，不比较两段总超限次数。
若明显改善，只说明该时序变化有效，仍需结合PWM开启段确认；若近似不变，不继续叠加分频、
滤波或放宽阈值试错，转向正常CAL低时的模拟输入/参考与连接核对。
本轮不接电机、不测试抱闸、不直接批准正反转；单次短窗改善也不等于全部硬件已验收。

## C. 2026-10-23：恢复旧阈值后手动正转—停止—反转

本次用户选择推进功能验证，不代表历史硬件门槛或采样问题已验收；不再追加无电机测量。
用户随后批准对照旧模式2的对齐0.5A/运行0.8A设定点；已新增OAE_ALIGN_CURRENT_A，
OAE_CURRENT_A只用于运行，保留用户现有1.0A参考限幅。电压限幅仍0.15pu，
过流关断仍>10A连续2次，斜坡和时限均未改；这不是完整复原旧固件或安全性验收。
给定位置是 `Core/Inc/motor/open_angle_experiment.h`，对齐消费者是
`Core/Src/motor/motor_control.c` 的 `MotorControl_RunCurrentLoop`，运行消费者是
`Core/Src/motor/open_angle_experiment.c` 的 `OAE_Step`。需重新下载同名AXF后生效。
ADC仍为上次Bench DIV2，原100ms基线自动执行，但不再要求专门展开截图。
用户报告6/12rpm均能转动后，本模式时限由总会话5秒/运动3秒改为10秒/6秒；
独立看门狗复用OAE_SESSION_MS/OAE_MOTION_MS，其他模式仍保持原时限。
运动计时从首次非零指令开始，含加速和减速，不是保证恒速保持6秒。

1. 先在电机U/V/W断开状态手动下载当前同名BenchCheck AXF，保持PLC启动/停止变量FALSE。
   随后断电并确认储能释放后连接U/V/W，固定电机、保持空载，清空转轴/联轴器周围。
   外部抱闸按电机要求接好额定12V并正确释放，不能带抱闸锁定强行启动；不把12V抱闸接到24V。
   如带垂直/重力负载，不使用此空载步骤。保持24V/1A，不自行提高电源限流，备好直接断24V的手段。
2. 重新给板供电，先Keil Debug/Run并确认uwTick增长，保持周期刷新，
   随后TwinCAT OP/PLC运行。不要在功率运行时暂停MCU或用断点。
3. 确认State=8、WcState=FALSE、nPhase=0、MotorFault/BridgeFault=0；
   bStart/bStop/bDisable/bReset/bFreezeHeartbeat均FALSE。启动前锁存故障按原bReset边沿复位后释放，
   不强写nPhase/映射输出；无法恢复则不继续启动。
4. 在启动前把rDemandRpm预置+6.0（等效需求，不是实测转速），再把bStart写TRUE，
    使用“写入值”而非强制。启动期间线上目标仍为0，对齐完成且Running确认后PLC才发送+6（1Hz）。
   不等待Running后才慢慢输入，否则会错过500ms零目标窗口。
5. 首次只观察一次短时运动，在发Start后约4秒把bStop写TRUE，请求减速停止。
   如不转、只抖动、异响、异常发热或电源持续限流，立即停止，不反复启动/自行加流；
   不能正常停机时用独立断电，不等待软件时限。出现故障保留MotorFault/BridgeFault/nPhase，
    不提高阈值或时限。参考给定不保证实际电流跟踪或起转，未转不自动等同通信失败。
6. 确认nPhase=0且bElectricalOutputOff=TRUE，再目视确认转轴完全停稳；
   ActualSpeed=0在本模式无证明力。随后把bStart、bStop写回FALSE，保持至少一次PLC扫描，
   确认无故障。运行中不把需求直接改负，不用bDisable模拟正常减速。
7. 仅当前一次已正常连续转动并停稳后，把rDemandRpm预置-6.0，再写bStart=TRUE；观察方向是否与前一次相反，
    同样在Start后约4秒写bStop=TRUE，并完成第6步。正负号不预先保证物理顺/逆时针方向。
8. 两次结果保存后rDemandRpm置0，释放启动/停止变量，再断电。只需要记录实际是否转动、
   两次方向是否相反、是否正常停止、故障码，不要求重新展开全部RAM/寄存器录屏。

当前保留首次非零需求6秒和总会话10秒上限；到时仍会产生故障12，不以“等自动停机”替代
上述主动停止；先观察短窗，不自行继续延长时限。不自动循环、不在运行中换向。
供电端1A限流不等于相电流1A保护，不能据此声称10A门槛已安全核定。

### 待办：采样异常（按用户要求暂缓）

- U/V/W未接、CAL低、三相PWM尚未开启时已有明显换算电流尖峰。
- DIV1：998样本/158次>0.6A，峰值1.79533815A；DIV2：998样本/105次，峰值1.90003705A。
  PWM开启后DIV2为13501样本/1364次，峰值2.20626402A，最长连续超限2，未证明改善。
- 原始ADC异常位置、真实相电流、模拟输入/参考/连接及闭环影响均未独立核实；
  本次只是恢复旧软件保护阈值，不是修复测量、不证明旧/新代码安全额定值。
- 现有over_limit_total等计数仍以0.6A为观察门槛，可能继续增长但不会单独触发MotorFault；
  真正超过10A时原两样本保护仍会关断，不能用放宽后的“无故障”作为采样合格证据。
