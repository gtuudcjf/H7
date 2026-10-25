# EtherCAT Open-Angle Current Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Do not dispatch agents unless the user chooses that execution method.

**Goal:** 在现有 H743/LAN9252 工程中增加固定模式 2 的 EtherCAT 联动 DryRun，保留旧速度闭环及通信基线，不访问实际在线输出。

**Architecture:** 现有 8/20 字节 PDO 加应用模式握手，桥接层按编译固定 profile 分派，电机层继续使用现有虚拟角度和电流 PI。专用纯 C 实验策略处理频率/电流约束、停止和时限，TIM6 负责不依赖前台的最终许可撤销。PLC 只发操作请求和心跳，不假造实际转速。

**Tech Stack:** C99/ARMCC5、STM32H743 HAL、LAN9252/SSC、PowerShell/TinyCC 宿主测试、Keil、TwinCAT 3.1.4024.78/ST 离线编译。

**Spec:** `docs/superpowers/specs/2026-10-20-ethercat-open-angle-current-design.md`（2026-10-20 用户确认）

## Global Constraints

- 同一工作目录增量修改，不创建分支/工作树、不提交、不回退或覆盖用户已有改动。
- 不写 EEPROM、不烧录 MCU、不激活/登录/启动 TwinCAT 在线程序、不改活动工程或系统 ESI。
- 保留旧 Communication、LocalMotor、速度 DryRun 的默认行为；新目标固定模式 2 且默认功率锁 0。
- revision 2 身份、Rx 8/Tx 20 字节、13 个 GVL I/O 名字/类型/方向、现有 ESI/SII 保持不变。
- 新 ControlWord bit8 握手；0x0100 禁用、0x0101 启用、0x0180 复位；无 bit8 的旧速度协议不变。
- 模式 2 StatusWord bit8 为模式、bit9 为控制快照有效、bit10 为完整电气停机；bit7 始终 0。
- 网络 ±20 rpm；等效磁场斜坡 20 rpm/s；对齐/运行候选 0.1 A，参考矢量上限 0.3 A，电压不超过 0.15 pu。
- 三相任一绝对电流超过 0.6 A 连续 3 个有效样本关断；ADC 失败不允许降级输出。
- 启动 2 s、启动后零命令等待 500 ms、正常停止 2 s；许可起 5 s/首次非零起 3 s 取先到者。
- 保留 100 ms 心跳和 TIM6 1 kHz 保护；停止/反向/故障不得靠重复命令刷新同一会话时限。
- 软件测试不替代机械停稳、硬件电流验证或 ≤110 ms 波形验收，功率解锁仍需独立确认。

## Review Focus

1. 旧 PLC 配新固件或反向错配，不能短暂对齐或拉高 PC4；Task 1/4 的 mismatch 测试覆盖。
2. 电气许可撤销后启动函数的后续步骤不能重启 PWM；Task 4 的 delay-then-enable 测试覆盖。
3. 时间戳从 UINT32_MAX 回绕、零频率反复命令不能延长会话；Task 2/4 覆盖。
4. 未测转速为 0 不等于停稳，PLC 不能因此跨过停止或自动反向；Task 5 覆盖。
5. 异步快照中的旧 current_valid 不能在 ADC 停止后维持 Running；Task 3/4 覆盖。

## File Structure

共用文件只增加隔离清晰的 profile/模式分支，不重构整个 FOC：

- 修改 `Core/Inc/ethercat/ethercat_build.h`：固定 profile 编译配置和非法组合检查。
- 修改 `Core/Inc/ethercat/ethercat_motor_core.h`、`Core/Src/ethercat/ethercat_motor_core.c`：兼容旧 API 的 profile 状态机。
- 新建 `Core/Inc/motor/open_angle_experiment.h`、`Core/Src/motor/open_angle_experiment.c`：纯 C 候选参数、会话/停止/方向/电流保护策略。
- 修改 `Core/Inc/motor/motor_control.h`、`Core/Src/motor/motor_control.c`、`Core/Src/main.c`：专用反馈与模式 2 接入，旧接口语义不变。
- 修改 `Core/Inc/motor/motor_command_guard.h`、`Core/Src/motor/motor_command_guard.c`、`Core/Inc/motor/motor_safety.h`、`Core/Src/motor/motor_safety.c`：独立会话时限和原因快照。
- 修改 `Core/Src/ethercat/ethercat_motor_bridge.c`：profile 分派和前台完整 Stop。
- 新建 `MDK-ARM/ethercat_openangle.uvprojx`：独立目标/输出目录，不覆盖旧工程设置。
- 新建 `EtherCAT/motor/openangle/plc/MAIN.TcPOU`、`GVL_MotorIO.TcGVL`、`PlcTask.TcTTO`：专用 PLC 源，旧 PLC 原样保留。
- 修改 `tools/build_ethercat_motor.ps1`、`tools/verify_motor_plc.ps1`：显式选择新目标/源目录，原默认参数不变。
- 新建专用 tests、runner、协议/验收说明；原测试保留并运行。

## Task 1: Profile protocol with unchanged wire layout

**Files:** core/build 配置文件；新建 `tests/test_ethercat_openangle_core.c`；修改 `tests/test_ethercat_motor_core.c` 仅补兼容断言。

**Interfaces:**
- 保留 `void EMC_Init(EMC_Core *c)`，默认旧速度协议。
- 增加 `EMC_Profile`，值 0 为旧速度、1 为模式 2；`void EMC_InitProfile(EMC_Core *c, EMC_Profile profile)`。
- 保留原反馈字段；增加 `uint8_t start_permitted, control_valid, output_off, stop_requested`，旧 profile 不读取新字段。
- 增加 fault 11/12/13/14，保持原 0～10 编号；宏值和协议设计一致。
- `EMC_SetFeedback/Receive/Poll/Pack` 的签名不变，只有选中模式 2 才读取额外字段和解析 bit8。

- [ ] 在工作区 `tmp/openangle_baseline_<唯一时间戳>` 备份本次拟修改的已有源文件、工程和脚本，并保存 SHA256 与旧 ESI/SII/PLC/活动工程 hash 清单；不覆盖已有备份，不把此前 dirty 内容计为本次修改。
- [ ] 增加 `test_old_plc_cannot_arm_new_profile`：缺 ack 的三次零帧不能 READY/续期；使能产生 fault11，START action=0。
- [ ] 增加 `test_new_plc_cannot_arm_old_profile`：旧 core 对 0x0100 报原 command fault，START action=0。
- [ ] 增加 profile 三零帧、零目标边沿启动、未校准但 start_permitted=1 可进入 STARTING、bit7=0/bit8=1/bit9 独立、停机 bit10、未知位拒绝、禁用重复序号优先、序号回绕测试；旧模式无额外状态位。
- [ ] 先用 TinyCC 编译运行新测试，记录未实现功能导致的失败；不是编译器缺失或路径错误。
- [ ] 实现上述接口和状态判断。正常完整 Stop 与意外丢 Running 分开：收到 stop_requested 时产生 STOP action，不能误报 motor fault；启动条件不伪造 calibrated。
- [ ] 运行新 core 测试和原 `tests/run_ethercat_motor_tests.ps1`，预期均无失败；差异核对后保留未提交修改。

## Task 2: Pure mode-2 experiment policy

**Files:** 新建 `open_angle_experiment.h/.c`、`tests/test_open_angle_experiment.c`、`tests/run_ethercat_openangle_tests.ps1`。

**Interfaces:**
- `OAE_State` 保存阶段、首个非零方向、停机锁存、开始/首次非零/待零/停机时间戳和过流样本计数。
- `OAE_Input` 包含 `uint32_t now_ms`、`float command_rpm, active_frequency_hz, active_id_a, active_iq_a`、`uint8_t aligning, control_valid`。
- `OAE_Result` 包含 `float target_frequency_hz, id_ref_a, iq_ref_a`、`uint8_t request_stop`、`uint16_t bridge_fault`。
- `void OAE_Init(OAE_State *s)`；`void OAE_Start(OAE_State *s, uint32_t now_ms)`；`OAE_Result OAE_Step(OAE_State *s, const OAE_Input *in)`。
- `uint8_t OAE_CheckPhaseCurrent(OAE_State *s, float ia, float ib, float ic)`；`float OAE_RpmToHz(float rpm)`；`float OAE_HzToRpm(float hz)`，极对数取 `MOTOR_POLE_PAIRS`。
- 专用参数宏集中在新 header，不覆盖原 `motor_params.h`；现有 OpenLoop 是频率斜坡唯一所有者，现有电流参考斜坡是电流斜坡唯一所有者。

- [ ] 先写测试断言：10 对极下 5 rpm≈0.833333 Hz；20 rpm/s≈3.333333 Hz/s；-20/20 合法，越界与 NaN/Inf 拒绝。
- [ ] 写对齐 Id=0.1/Iq=0、运行 Id=0/Iq=0.1、矢量不超过0.3、停止先等活动频率回零再撤电流、回零后非零不能取消停止、初始零等待500 ms、停止2000 ms、反向 fault13 测试。
- [ ] 写 overcurrent：0.6 不算越界，0.6001 连续3有效样本触发；任一相、清零计数、非有限输入均覆盖；参考限幅不是实际电流保证。
- [ ] 写 start+5000/first_nonzero+3000、边界前1ms、重复命令不能刷新、uint32回绕测试；先运行观察失败。
- [ ] 实现纯策略：只读真实活动状态，不伪造速度；停止阶段结果 request_stop 交给前台执行完整 HAL Stop；独立硬时限由 Task 4 执行，纯策略只辅助报告。
- [ ] 新 runner 使用现有可用 TinyCC，`-Wall -Werror`，输出独立检查数；运行预期全部通过。

## Task 3: Motor mode adapter and honest coherent feedback

**Files:** 修改 `motor_control.h/.c`、`main.c`；新建 `tests/test_openangle_motor_integration.ps1` 与 `tests/test_openangle_motor_adapter.c`。

**Interfaces:**
- 保留原 `MotorControl_GetFeedback(MotorControlFeedback *)` 与旧速度协议字段语义。
- 新增 `MotorOpenAngleFeedback`：`float iq_a, active_frequency_hz`，`uint16_t motor_fault, bridge_fault`，`uint8_t start_permitted, running, control_valid, output_off, stop_requested`。
- `void MotorControl_GetOpenAngleFeedback(MotorOpenAngleFeedback *feedback)`：临界区一致快照，校验 ADC age/有限值/实际模式；不刷新旧数据的时间戳。
- `void MotorControl_SetOpenAngleSpeedCommand(float equivalent_rpm)`：只更新专用实验请求，运行中由 OAE 管理，不调用速度 PI。
- `ETHERCAT_MOTOR_OPEN_ANGLE=1` 只在 motor profile 下合法；旧工程缺省0。

- [ ] 写 ARM 预处理测试：新模式选择模式2、零目标、没有自动Start、没有 SPI4/编码器初始化/采集路径、没有 USB/UART遥测；旧模式仍含原编码器分支。
- [ ] 写生产适配函数宿主测试：校准失败、ADC超时、NaN电流、alignment到running、停机未清理完/PC4仍高/MOE仍高不能 output_off；旧快照 valid 不等于新 control_valid。
- [ ] 运行新测试，记录实现前失败；实现一致快照、OAE 10 kHz 接入、专用0.1A对齐/运行、电压0.15pu、相电流检查，保留原电流PI/SVPWM算法。
- [ ] 禁止新模式通过普通远程/运行期接口切成模式1/3/4/6；main 不调用 Biss 初始化，FastTick 与初始化时的编码器调用在新构建中排除。
- [ ] 原 mode4 校准门槛和 local参数不变；模式2 ADC校准失败硬停，current sample age无效立刻关断；中断中只锁存Stop请求，不执行完整HAL启停。
- [ ] 运行适配和原电机回归；预处理断言/适配宿主测试通过只算软件结果，不声称实际 ADC/PWM 已测。

## Task 4: Bridge and independent guard integration

**Files:** guard/safety header/source、`ethercat_motor_bridge.c`；新建 `tests/test_ethercat_openangle_bridge.c`；扩展 `tests/test_motor_command_guard.c`、`tests/test_motor_safety.c` 和 motor_guard stubs。

**Interfaces:**
- 旧 `MCG_Permit/Tick/Feed/Disarm/Revoke` 调用行为保持。
- 增加 `void MCG_StartSession(MotorCommandGuard *g, uint32_t now_ms)`、`void MCG_MarkMotion(MotorCommandGuard *g, uint32_t now_ms)`、`uint8_t MCG_TripReason(const MotorCommandGuard *g)`；原因值 none=0/heartbeat=1/runtime=2，已有 session 不刷新起点。新字段在MCG_Init/Reset完整初始化，旧构建不启动会话时限。
- safety 增加 `void MotorSafety_MarkMotion(void)`、`uint8_t MotorSafety_TripReason(void)`；新 profile 的 Arm 成功时开启5000ms会话，首个合法非零请求标记3000ms运动时限。
- 桥接 Init 依编译配置调用 EMC_InitProfile；Observe 按 profile 读取旧/新快照；旧路径不调用新接口。

- [ ] 写测试：POWER=0时 mode2正确握手后Start返回fault9且PC4/MOE全0；POWER=1宿主假硬件可测试真实状态路径，但不连接板子。
- [ ] 测试5000/3000ms独立于前台、seq Feed不延长、UINT32回绕、退出OP/丢ack/非法帧都关断；先运行并观察失败。
- [ ] 实现 TIM6 guard 扩展并保持临界区完整；检查Start入口、driver唤醒、采样PWM、三相PWM入口共同检查锁存。
- [ ] 实现桥接mode2分派、频率命令、运行/停止状态、独立时限故障12；raw Enable=0即使重复或错误profile也必须先关断。
- [ ] 宿主模拟Start前台暂停100ms及5000ms，执行TIM6，再恢复后续使能步骤，断言不能重新拉高PC4/MOE；原速度路径同测。
- [ ] 测试故障→恢复OP+旧Enable=1不Start；禁用零帧+Reset+新三帧+新边沿才可接受下一次启动；正常零停止后旧Enable=1也不自动重启。
- [ ] 原runner及新runner全部通过；不通过不进入PLC/硬件交付。

## Task 5: PLC source that preserves existing I/O links

**Files:** 新建 `EtherCAT/motor/openangle/plc/MAIN.TcPOU`、`GVL_MotorIO.TcGVL`、`PlcTask.TcTTO`；修改 `tools/verify_motor_plc.ps1` 参数；新建 `tests/test_openangle_plc_artifacts.py`。

**Interfaces:**
- 保持 `GVL_MotorIO` 现有13个I/O变量和旧文件名/类型，不新增需链接的PDO变量。
- 继续使用 `MAIN.bStart/bStop/bDisable/bReset/rDemandRpm/bFreezeHeartbeat/nPhase/bLocalFault`。
- 新增显示变量 `bOpenAngleProfile, bControlFeedbackValid, bElectricalOutputOff, bSpeedMeasurementValid`，最后者固定FALSE，不控制硬件。
- 离线验证脚本增加 `-PlcSourceDir`、`-ProjectName`，默认旧目录/名称不变；新归档名 `EtherCATMotorOpenAngleDemo.tpzip`。

- [ ] 写构件测试：13变量映射完全相同、10ms周期、所有状态包括故障/禁用均携带bit8、禁止原bit7/ActualSpeed参与启停、禁止任何自动非零需求和直接符号翻转。
- [ ] 先运行测试确认新源缺失/行为断言失败；从旧PLC声明/逻辑复制到新目录后适配，不编辑活动用户工程。
- [ ] 新MAIN等待profile+Ready才接受Start边沿；以Running+bit9确认启动，以bit10确认电气停止；fault/reset等待真实确认而非把写入当成功。
- [ ] 运行离线XAE编译：`powershell -File tools/verify_motor_plc.ps1 -PlcSourceDir EtherCAT/motor/openangle/plc -ProjectName EtherCATMotorOpenAngleDemo`；只创建脚本拥有的隐藏实例，只操作工作区临时工程，不调用Activate/Restart/Login/Start。
- [ ] 若COM启动被权限阻止，申请工具权限；不可绕过或关闭用户XAE实例。若环境仍不可用，交付源且明确标记PLC编译待验证，不伪造归档成功。
- [ ] 归档导出后再导入另一临时工程实际编译；排除越界路径/自动boot载荷。hash核对旧PLC与活动工程未改变。

## Task 6: Isolated builds, full regression and consolidated handoff

**Files:** 新建 `MDK-ARM/ethercat_openangle.uvprojx`；修改 build runner增加显式 `OpenAngleDryRun` 选择；新建 `EtherCAT/motor/openangle/PROTOCOL.md`、`COMMISSIONING.md`、`SOFTWARE_VERIFICATION.md`；完善新测试runner。

- [ ] 先写配置测试：独立目标名、输出目录、OPEN_ANGLE=1、POWER_ENABLED=0、旧工程默认不变；不生成默认可下载的power-on目标；先运行观察失败。
- [ ] 从现有Keil工程创建新目标并加入OAE文件，旧目标按需纳入共用源但保留旧宏和输出目录。宏冲突通过真实预处理器拒绝。
- [ ] 执行 `powershell -File tests/run_ethercat_motor_tests.ps1`、`powershell -File tests/run_ethercat_openangle_tests.ps1`、原EtherCAT/电机相关回归和新的PLC构件测试，逐项记录命令与退出码。
- [ ] 分别执行 build runner 的 `Communication`、`LocalMotor`、`DryRun`、`PowerPathCheck`、`OpenAngleDryRun`；只编译链接，不下载。功率分支仅编译检查仍不得输出“硬件通过”。
- [ ] 原/新ARM工程预期0 Error(s),0 Warning(s)，归档/日志/新DryRun AXF独立列出；编译器/工具不可用必须明确说明，不以预处理替代链接。
- [ ] 对照baseline核对ESI/SII/EEPROM文件、旧PLC和用户活动TwinCAT工程hash，确认8/20字节与对象字典/映射未变化。git diff只列本次增量，不提交dirty工作树。
- [ ] 写协议说明：新增位/故障、模式握手、ActualSpeed无效、ActualIq坐标系、ActiveTargetSpeed仅磁场速度，以及旧PLC/新固件错配拒启。
- [ ] 写一份三组集中实机清单：新DryRun短回归；保护时限和采样/对齐；逐次批准的短时正转/停机/反转。记录已做旧通信实验作为基线，不要求用户重配网卡、EEPROM、PDO链接。
- [ ] 最终报告分别列软件测试、ARM编译、PLC编译、实机未完成项，明确默认锁功率。交付操作只到备份后导入/替换MAIN，不远程覆盖当前在线工程。

## Plan self-review

- Spec 1～3 对应Task3/6，4对应Task1/5，5对应Task2/3，6对应Task1～4，7对应Task5，8对应所有测试和Task6；9通过本交接门槛执行。
- profile标识、确认位、bit7/9区分、时限单位、故障编号在各任务一致；OAE输出不假造实际速度。
- Five Review Focus 已分别落实到Task1/4、4、2/4、5、3/4的具体断言。
- 既有dirty worktree与用户已做实验保留；本计划不含提交步骤，因为用户要求分支和提交暂不处理。

## Execution handoff

2026-10-20 用户已批准本计划并选择当前会话主代理连续实施（Native）。六项实现与
验证已记录在 `.superpowers/sdd/2026-10-20-ethercat-open-angle-current/progress.md`；
上方复选项保留原计划模板，实际完成状态以该账本和软件验证记录为准。
最后按执行技能安排一次只读独立审查。没有提交、创建分支/工作树或清理备份。
实施批准不包含实机功率解锁、烧录或在线运行权限。
