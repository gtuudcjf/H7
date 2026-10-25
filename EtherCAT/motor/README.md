# H743 + LAN9252：EtherCAT 电机低速联动实验

适用日期：2026-10-16。分支：`codex/ethercat-motor-speed`，通信基线：`60f6444`。

本目录是**实验软件交付，不是已完成的带电电机验收**。旧 revision 1 通信版不变；
电机版使用 revision 2、自定义 PDO，不是 CiA402，不创建 NC 轴，不使用 DC 同步。

## 1. 当前交付与禁止越过的门槛

- 固件：复用现有编码器速度环 / 电流环，新加命令桥接、独立 TIM6 许可监控。
- 默认工程：`MDK-ARM/ethercat_motor.uvprojx`，目标 `ethercat_motor_dryrun`。
  **默认 `ETHERCAT_MOTOR_POWER_ENABLED=0`，启动请求返回 BridgeFault=9，不打开功率级。**
- 主站：`plc/` 中的 ST 源码和 XAE 导出的 `.tpzip` PLC 工程包，10 ms 任务，
  无自动启停、正反转循环。`.plcproj` 仅是源码清单/模板，直接导入在本机不兼容，
  请使用 `.tpzip` 或从 Standard PLC Template 导入源码。
- 设备描述：`H743-LAN9252-MOTOR.xml`，输入 20 B / 输出 8 B。
- EEPROM：`H743-LAN9252-MOTOR-rev2-CANDIDATE.bin` 是 **2048 B 未经实机核验的候选**，
  不是已获准写入的镜像。`candidate-manifest.json` 明确记录未验证、未授权写入和 SHA256。

本次不执行下载、整片擦除、EEPROM 写入、主站配置激活或电机运动。阶段 B～E 的测量
须由操作者按下文完成。软件测试通过不等于 110 ms 关断指标或电机稳定性已经通过。

## 2. 架构和三种运行模式

```text
PLC 10 ms：操作按钮、目标速度、递增序号
  -> RxPDO -> SSC / SPI1 -> 前台桥接 -> 原 MotorControl
                               |        TIM8 / ADC：10 kHz 电流环
                               |        原速度环：1 kHz
                               <- 一致快照 -> TxPDO -> PLC
独立 TIM6 1 kHz -> 命令许可过期/禁止锁存 -> PC4 低 + TIM8 MOE 清除
```

编译开关在 `Core/Inc/ethercat/ethercat_build.h`：

| 模式 | BRINGUP_ONLY | MOTOR_CONTROL | MOTOR_POWER_ENABLED |
|---|---:|---:|---:|
| 原通信版 | 1 | 0 | 0 |
| 原本地电机版 | 0 | 0 | 0（此开关不约束旧本地模式） |
| 联动、功率锁定 | 0 | 1 | 0 |
| 联动、许可功率路径 | 0 | 1 | 1（须先通过无动力验证） |

两个 EtherCAT 模式同时为 1 会编译报错。功率开关只允许在联动模式使用。
原本地模式仍有原来的自动启动、非零预置目标，**不得当成无输出验证固件下载**。
联动模式只初始化 GPIO、SPI1/2/4、DMA、ADC、TIM6/8；不启 USB / 串口遥测。
启动模式固定为编码器速度/电流闭环，上电目标 0，不自动 `MotorControl_Start()`。

短临界区保护驱动拉高、PWM 通道使能与 TIM6 撤销许可之间的竞争；每个使能入口都
检查同一许可。TIM6 不执行完整 HAL 停机，前台随后清理 ADC/TIM8 并保留诊断。
SPI1 和 SSC 只在前台运行，不在电机中断中访问；不开放网络模式切换、校准或 Flash 写入。

## 3. 构建与回归

原 `emptytest.uvprojx/.uvoptx` 的用户修改保持原样。新工程独立存在，输出到
`MDK-ARM/motor_speed/`。当前脚本使用本机 `D:/Keil_v5/UV4/UV4.exe`。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_ethercat_motor.ps1 -Mode DryRun
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_ethercat_motor.ps1 -Mode Communication
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_ethercat_motor.ps1 -Mode LocalMotor
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build_ethercat_motor.ps1 -Mode PowerPathCheck

# tcc.exe 需要在 PATH 中；只在本地运行测试，不连接硬件。
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_ethercat_motor_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_ethercat_motor_build.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_ethercat_suite.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1
python tests/test_motor_artifacts.py
```

只有 DryRun 产出默认锁功率的 HEX；其他三条路径只生成编译/链接检查产物。
`PowerPathCheck` 的 AXF 不是实机验收许可，脚本没有下载步骤。
`tools/verify_motor_plc.ps1` 尝试建立隔离的临时 XAE 工程，仅离线构建，不激活、不登录、不下载。
不能将 XML 解析通过或文本契约检查当成 ST 编译通过；以实际 XAE 编译日志为准。

### 校准数据保护

当前校准区为 `0x081E0000`（Bank 2 Sector 7）；应用 IROM 的末端为 `0x081DFFFF`。
编译布局预留该区不代表下载器不会整片擦除。下载前必须备份有效校准记录，检查 Flash
算法和擦除设置，禁止 Whole Chip Erase；修改芯片/链接布局后重新核对。
没有有效校准记录时不允许启动。此固件不提供远程重新校准。

## 4. 固定 PDO：固件、ESI、SII、PLC 必须成套使用

所有字段是小端序，显式打包，不依赖 C 结构对齐。速度均为**电机轴机械速度**，
不是减速器输出轴速度。`50 = +5.0 rpm`，`-100 = -10.0 rpm`。

### RxPDO 0x1600：8 字节（主站输出、从站接收）

| 偏移 B | 对象 | 字段 | 类型 | 单位 |
|---:|---|---|---|---|
| 0 | 0x7000:01 | ControlWord | UINT16 | bit0 Enable，bit7 FaultReset |
| 2 | 0x7000:02 | TargetSpeed | INT16 | 0.1 rpm |
| 4 | 0x7000:03 | CommandSequence | UINT32 | PLC 每周期递增 |

其余控制位必须为 0。FaultReset 只接受停止、Enable=0、TargetSpeed=0 下的上升沿。
对象的 SDO 访问为只读，不能用 SDO 写入绕过 PDO 心跳和启动门槛。

### TxPDO 0x1A00：20 字节（主站输入、从站发送）

| 偏移 B | 对象 | 字段 | 类型 | 单位 / 含义 |
|---:|---|---|---|---|
| 0 | 0x6000:01 | StatusWord | UINT16 | 下表 |
| 2 | 0x6000:02 | CycleCounter | UINT32 | 联动运行且交换有效时的前台服务计数，不是 DC 时间 |
| 6 | 0x6000:03 | ActualSpeed | INT16 | 0.1 rpm |
| 8 | 0x6000:04 | ActualIq | INT16 | mA，q 轴电流，不是母线输入电流 |
| 10 | 0x6000:05 | MotorFault | UINT16 | 原电机故障枚举，见 motor_control.h |
| 12 | 0x6000:06 | BridgeFault | UINT16 | 下表 |
| 14 | 0x6000:07 | ActiveTargetSpeed | INT16 | 0.1 rpm，MCU 斜坡后的活动目标 |
| 16 | 0x6000:08 | AcceptedSequence | UINT32 | 最近接受的序号 |

反馈从实时控制内部状态以短临界区取得一致快照。无效反馈仍有数值，但不能作为当前
真实运行测量使用，须同时看 StatusWord.bit7 和 WcState。

| StatusWord 位 | 名称 |
|---:|---|
| 0 | 可启动（零命令准备完成、校准有效） |
| 1 | 运行中 |
| 2 | 故障 |
| 3 | OP 输出交换已有效，且 SPI 无故障 |
| 4 | 命令心跳新鲜 |
| 5 | 需要重新准备 / 使能 |
| 6 | 启动中 |
| 7 | 反馈有效 |

| BridgeFault | 含义 / 处理 |
|---:|---|
| 0 | 无故障 |
| 1 | 退出有效 OP，重新连接后须复位、重新准备 |
| 2 | SPI 锁存故障；**网络复位不清除**，保存首次故障后按原诊断流程处理 |
| 3 | 非法控制位、目标越界、非法启动目标等 |
| 4 | 心跳超时 / 独立许可已撤销 |
| 5 | 序号倒退或无效大跨度 |
| 6 | 电机启动失败，或本次外设初始化失败（后者不能网络复位） |
| 7 | 启动超过 2 s |
| 8 | 电机故障或运行反馈失效 |
| 9 | 默认功率测试门锁定；是 DryRun 的预期结果 |
| 10 | 完整停机清理失败，禁止继续启动 |

## 5. 启动、停机、失联规则

上电默认关闭驱动。准备条件：OP 有效、SPI 正常、无锁存故障、校准有效，连续至少
三个**新鲜向前序号**的 Enable=0 / TargetSpeed=0 命令；随后必须收到新的 Enable
0→1 边沿，而且该帧目标仍为 0。启动期间保持零目标，直到 Running 和反馈有效。
零偏校准、编码器检查沿用原控制层；启动超过 2 s 则关断并锁存故障。

序号重复不续租，跳号允许，UINT32 正常回绕允许；倒退或半范围以上跳变拒绝。
有效序号最后一次更新后 100 ms 撤销许可；即使网络持续重发旧帧也会停机。
Enable=0 优先停机，即使序号重复也立即撤销输出许可，不等待正常减速。

正常停止由 PLC 完成：目标先置 0，反馈有效且 |速度|<1 rpm 连续 200 ms 后撤销
Enable；超过 2 s 未完成即软件故障停机。故障停机是 PC4 低 / MOE 清除并自然停转，
不是瞬间刹停，也不是认证 STO。母线、电机故障等原有保护不放宽。

通信恢复允许回到 OP，但旧 Enable=1 不会再次启动。必须停止命令、显式复位可恢复
故障、重新完成三帧零命令、重新发启动边沿。PLC 的复位会等待从站反馈确认，而不是
写完复位位立即认为已清除。SPI 故障和启动外设初始化失败不能网络复位。

## 6. 主站准备：不复用 Free Run 手写值驱动带电电机

1. 保存旧 TwinCAT 工程、revision 1 ESI 和 EEPROM 的完整读出备份；记录设备地址、
   Vendor / Product / Revision / Serial。新 revision 2 Vendor=0x00F0C743，
   Product=0x00009252，Revision=2，Serial=1。
2. 核对候选 ESI 和 SII：SM0/1 为 128/128 B，SM2/3 为 8/20 B；SII 配置 CRC、
   身份、映射及长度须与候选 manifest 一致。现有软件检查**不替代 TwinCAT 的
   ESI 校验、EEPROM 页面检查及写后读回比较**。
3. SII 写入是独立审批点：先提交旧镜像、候选内容差异和目标身份给操作者确认。
   **没有明确确认，不写 EEPROM。** 不用 revision 1 的 SII 搭配 revision 2 固件冒充验收。
4. 在独立的实验 TwinCAT 工程中安装/载入电机 ESI（不要覆盖旧文件），按 revision 2
   匹配从站；导入 `plc/EtherCATMotorSpeedDemo.tpzip`（Add Existing PLC Project，
   或新建 Standard PLC Project 后添加 MAIN、GVL_MotorIO、PlcTask）。保留 Tc2_Standard、
   Tc2_System、Tc3_Module 库引用，清除模板中重复的 MAIN/任务。
5. PLC 任务 `PlcTask` = 10000 µs（10 ms），MAIN 为唯一程序调用；核对实时任务和
   EtherCAT 周期关联也为 10 ms，不只是 PLC 中计数器变化慢。无需 NC 轴。
6. 在过程映像中逐一链接同名字段：8 个 PDO 输入 -> GVL_MotorIO 输入，3 个输出 <-
   GVL_MotorIO 输出；额外将从站 WcState 和 InfoData.State 链到同名 PLC 输入。
   State 应为 8、WcState 为 0 才可启动。所有有符号速度和电流必须链接到 INT。
7. 离线 Build PLC 和系统配置，确认没有未链接变量、类型/长度不匹配、重复 MAIN。
   激活/登录/运行均为之后的人工步骤，不能由本仓库脚本自动执行。
8. 取消原来对 ControlWord / TargetSpeed 的手工 Force 或 Write；不要同时让 PLC 和
   Free Run 手写值成为控制源。PLC 操作变量全部非 RETAIN，上电禁用。

操作变量在 `MAIN` 的 Online 视图中修改：

| 变量 | 用途 |
|---|---|
| bStart | FALSE→TRUE 单次启动请求，随后写回 FALSE；一直 TRUE 不会连续启动 |
| rDemandRpm | rpm，不是 0.1 rpm；首轮先 0，再 +5 或 -5，绝对值≤20 |
| bStop | 正常减速停机，完成后写回 FALSE |
| bDisable | 直接撤销 Enable 的软件停机，非硬件急停 |
| bReset | FALSE→TRUE 复位请求，随后 FALSE；必须 bStart=FALSE |
| bFreezeHeartbeat | 只供阶段 B 故障注入，冻结序号，不用于正常操作 |
| nPhase | 0 空闲 / 10 启动 / 20 运行 / 30 回零 / 80 等复位确认 / 90 故障 |

观察 GVL_MotorIO 全部反馈，特别是 BridgeFault、MotorFault、StatusWord、
ActualSpeed、ActualIq、ActiveTargetSpeed、AcceptedSequence。不要把 PDO StatusWord
当成此前最低位回环含义，也不要用 OP 状态代替电机运行状态。

## 7. 分阶段实机验收表（尚未执行）

### B：无动力输出验证，未通过不得进入 C

先物理断开电机动力供电/接至已确认无危险的测试负载，同时保持控制逻辑和编码器供电。
用默认 DryRun 固件确认 PDO、零命令门槛、心跳、故障反馈；Start 得到故障 9 是预期。
**默认锁功率构建从未获得输出许可，因此仅靠它不能测出“已使能后撤销”的时间。**
如需测量真实 PC4 / MOE 撤销，须在动力通路物理隔离并核验后，明确批准使用
POWER_ENABLED=1 的测试构建，在示波器/逻辑分析仪上测量；未批准不执行此步骤。

| 用例 | 必须记录的结果 |
|---|---|
| 上电 / 残留 Enable=1 | PC4 低、MOE=0，不启动 |
| 未满三帧 / 非零目标启动 | 不启动；非法命令应故障 |
| 重复序号 / PLC STOP / 心跳冻结 | 最后有效序号后撤销输出许可 |
| 拔线 / 主站退出 OP | 禁止输出，前台完成清理 |
| 越界 ±20 rpm / 非法控制位 | 拒绝并停机，不静默限幅运行 |
| 模拟 SPI 失败 | 锁存，网络复位不能清除 |
| 前台阻塞 / 启动中注入失效 | 独立 TIM6 仍撤销，后续启动步骤不能重新打开输出 |
| 重连 / 旧 Enable=1 | 可恢复 OP，但不恢复运动 |
| 显式复位再三帧零命令 | 仅重新准备，不自动启动 |

在最大预期 TIM8/ADC/编码器中断负载下，用实际波形验收：最后新鲜心跳后 **≤110 ms**
撤销 PC4 / MOE。记录测试构建、序号时标、PC4/MOE波形、最大延迟及前台阻塞场景。
HAL_GetTick、TIM6、中断优先级及整体 CPU 执行均参与此保护，完全 CPU 失控或长期关闭
中断不在覆盖范围。不得带动力暂停 CPU 调试。独立切断动力电源的手段必须始终可用。

### C：同一硬件零速复查

仅在 B 通过并明确批准动力输出后，以 24 V、原电机/编码器/功率板/接线启动零速闭环。
检查有效校准、电流零偏、编码器新鲜度、Iq、速度及故障。网络 Iq 限制 0.3 A，沿用
20 rpm/s 斜坡；若静摩擦导致持续限流不转，停机，不自动增流。检查新增电机中断后
SPI 首次故障/超时计数、TIM8/ADC 执行时间和漏周期，不能只看仍处于 OP。

### D：最低速正反转，每一步人工确认

1. rDemandRpm=0，完成准备后脉冲 bStart，等待状态运行且反馈有效。
2. rDemandRpm=+5，观察约 3 s；方向不符、振荡、异响、持续限流立即停止。
3. rDemandRpm=0，bStop=TRUE，确认回零 200 ms 后禁用，再将 bStop 复位。
4. 重新准备/使能，再给 -5，回零并停机。不要直接从正目标跳到负目标。
5. 上述通过后，再分别人工执行 +10 和 -10。记录目标、活动目标、实际速度、Iq、故障。

### E：最低速下失联恢复

仅在 B～D 通过后以 +5 rpm 逐项测试拔线、心跳冻结、主站退出 OP。必须按 B 的已测
时限撤销输出；重连恢复 OP 但保持停机，只有新的复位/准备/启动才允许再运行。
网络恢复不应作为恢复危险运动的授权。

### 空白记录模板

| 日期/构建 SHA/硬件接线 | 阶段/用例 | 目标/实际 rpm | Iq mA | 故障/关断延迟 | 通过/证据文件 |
|---|---|---|---|---|---|
| 待填 | 待填 | 待填 | 待填 | 待填 | 未执行 |

## 8. 边界与参考

可编译、Host 测试通过、进入 OP、真实电机联动通过、关断时序通过，是五件不同的事。
本阶段不声明实时性能认证、安全认证或高精度速度性能。笔记本网络波动也不能自动排除
固件/硬件问题；持续增大的丢帧、CRC、SPI 错误仍需分层排查。

参考：[Beckhoff Automation Interface / PLC 项目](https://infosys.beckhoff.com/content/1033/tc3_automationinterface/242730891.html)、
[SII 格式读写参考实现](https://github.com/synapticon/siitool/blob/master/sii.c)、
[Microchip LAN9252 数据手册](https://ww1.microchip.com/downloads/en/DeviceDoc/00001909A.pdf)。
